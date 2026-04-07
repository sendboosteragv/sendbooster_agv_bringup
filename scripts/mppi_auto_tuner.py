#!/usr/bin/env python3
"""
MPPI Auto-Tuner for Sendbooster AGV (Jetson Orin Nano optimized)

Automated parameter tuning for Nav2 MPPI controller + AMCL.
Gazebo runs once (unrestricted), Nav2 restarts per trial with CPU limits
simulating Jetson Orin Nano (6 ARM cores ≈ 3 x86 cores).

Two-phase approach:
  Phase 1: CPU screening — find param combos that keep CPU < 60%
  Phase 2: Quality tuning — optimize critic weights within CPU budget

Usage:
  python3 scripts/mppi_auto_tuner.py
  python3 scripts/mppi_auto_tuner.py --phase1-only
  python3 scripts/mppi_auto_tuner.py --jetson-cores 3  # CPU core limit

Output:
  config/nav2_params_mppi_tuned.yaml  — optimized parameters
  tuning_logs/tuning_results.json     — full trial log
"""

import argparse
import copy
import json
import os
import signal
import subprocess
import sys
import time
import random
import threading
from pathlib import Path

import yaml

# Project paths
PROJECT_DIR = Path(__file__).resolve().parent.parent
BASE_YAML = PROJECT_DIR / 'config' / 'nav2_params_mppi.yaml'
KEEPOUT_MASK = PROJECT_DIR / 'map' / 'keepout_mask.yaml'
MAP_FILE = PROJECT_DIR / 'map' / 'my_map.yaml'
LOG_DIR = PROJECT_DIR / 'tuning_logs'

# CPU budget: target 60% max on Jetson Orin Nano
CPU_BUDGET = 60.0
CPU_HARD_LIMIT = 70.0

# Jetson Orin Nano: 6 ARM A78AE ≈ 3 x86 cores
JETSON_CORES = 3

# ── Phase 1: CPU-impactful parameters ──
PHASE1_COMBOS = [
    # Low CPU
    {'batch_size': 300, 'time_steps': 20, 'iteration_count': 1, 'max_particles': 500, 'max_beams': 30},
    {'batch_size': 500, 'time_steps': 32, 'iteration_count': 1, 'max_particles': 500, 'max_beams': 30},
    {'batch_size': 500, 'time_steps': 32, 'iteration_count': 1, 'max_particles': 1000, 'max_beams': 45},
    # Medium CPU
    {'batch_size': 1000, 'time_steps': 48, 'iteration_count': 1, 'max_particles': 1000, 'max_beams': 45},
    {'batch_size': 1000, 'time_steps': 56, 'iteration_count': 1, 'max_particles': 1000, 'max_beams': 45},
    {'batch_size': 1500, 'time_steps': 48, 'iteration_count': 1, 'max_particles': 1000, 'max_beams': 45},
    {'batch_size': 1500, 'time_steps': 56, 'iteration_count': 1, 'max_particles': 1000, 'max_beams': 45},
    # High CPU
    {'batch_size': 2000, 'time_steps': 56, 'iteration_count': 1, 'max_particles': 1000, 'max_beams': 45},
    {'batch_size': 2000, 'time_steps': 56, 'iteration_count': 1, 'max_particles': 1500, 'max_beams': 60},
    {'batch_size': 2000, 'time_steps': 56, 'iteration_count': 2, 'max_particles': 1000, 'max_beams': 45},
    # Max CPU
    {'batch_size': 2000, 'time_steps': 56, 'iteration_count': 2, 'max_particles': 1500, 'max_beams': 60},
    {'batch_size': 1000, 'time_steps': 48, 'iteration_count': 2, 'max_particles': 1500, 'max_beams': 60},
    # Sweet spot candidates
    {'batch_size': 1000, 'time_steps': 32, 'iteration_count': 1, 'max_particles': 1500, 'max_beams': 60},
    {'batch_size': 1500, 'time_steps': 32, 'iteration_count': 1, 'max_particles': 1500, 'max_beams': 45},
    {'batch_size': 1500, 'time_steps': 48, 'iteration_count': 1, 'max_particles': 1500, 'max_beams': 60},
]

# ── Phase 2: Quality-tuning parameters (critic weights) ──
PHASE2_SPACE = {
    'GoalCritic.cost_weight': [3.0, 5.0, 8.0],
    'PathAlignCritic.cost_weight': [8.0, 14.0, 20.0],
    'PathFollowCritic.cost_weight': [3.0, 5.0, 8.0],
    'ObstaclesCritic.cost_weight': [1.0, 2.0, 3.0, 5.0],
    'PreferForwardCritic.cost_weight': [3.0, 5.0, 8.0],
    'temperature': [0.1, 0.3, 0.5, 1.0, 3.0],
}


def load_base_yaml():
    with open(BASE_YAML, 'r') as f:
        return yaml.safe_load(f)


def patch_yaml(base, params):
    """Patch MPPI and AMCL parameters into a copy of the base YAML."""
    patched = copy.deepcopy(base)
    follow = patched['controller_server']['ros__parameters']['FollowPath']
    amcl = patched['amcl']['ros__parameters']

    FLOAT_PARAMS = {'temperature', 'vx_max', 'vx_min', 'wz_max', 'model_dt',
                    'vx_std', 'vy_std', 'wz_std', 'prune_distance',
                    'transform_tolerance', 'cost_weight', 'repulsion_weight',
                    'critical_weight', 'collision_cost', 'collision_margin_distance',
                    'near_goal_distance', 'max_path_occupancy_ratio',
                    'threshold_to_consider', 'max_angle_to_furthest'}
    INT_PARAMS = {'batch_size', 'time_steps', 'iteration_count', 'cost_power',
                  'trajectory_point_step', 'offset_from_furthest',
                  'max_particles', 'max_beams'}

    for key in ['batch_size', 'time_steps', 'iteration_count', 'temperature',
                'vx_max', 'vx_min', 'wz_max', 'model_dt']:
        if key in params:
            val = params[key]
            follow[key] = float(val) if key in FLOAT_PARAMS else int(val)

    for key, val in params.items():
        if '.' in key:
            critic_name, param_name = key.split('.', 1)
            if critic_name in follow:
                if param_name in FLOAT_PARAMS:
                    val = float(val)
                elif param_name in INT_PARAMS:
                    val = int(val)
                follow[critic_name][param_name] = val

    for key in ['max_particles', 'max_beams']:
        if key in params:
            amcl[key] = int(params[key])

    return patched


def ensure_float_params(d):
    """Recursively ensure MPPI float params are float type in the dict."""
    MUST_BE_FLOAT = {'temperature', 'vx_max', 'vx_min', 'vy_max', 'wz_max',
                     'model_dt', 'vx_std', 'vy_std', 'wz_std', 'prune_distance',
                     'cost_weight', 'repulsion_weight', 'critical_weight',
                     'collision_cost', 'collision_margin_distance',
                     'near_goal_distance', 'max_path_occupancy_ratio',
                     'threshold_to_consider', 'max_angle_to_furthest'}
    if isinstance(d, dict):
        for k, v in d.items():
            if isinstance(v, dict):
                ensure_float_params(v)
            elif k in MUST_BE_FLOAT and isinstance(v, int):
                d[k] = float(v)


def write_trial_yaml(patched, trial_num):
    """Write patched YAML to a temp file."""
    ensure_float_params(patched)
    path = f'/tmp/nav2_mppi_trial_{trial_num}.yaml'
    with open(path, 'w') as f:
        yaml.dump(patched, f, default_flow_style=False, allow_unicode=True)
    return path


# ── Simulation lifecycle ──

def launch_gazebo():
    """Launch Gazebo once (unrestricted CPU). Returns process."""
    cmd = [
        'ros2', 'launch', 'sendbooster_agv_bringup', 'simulation_mppi.launch.py',
        'headless:=true', 'gazebo:=true', 'nav:=false',
    ]
    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    print('  Gazebo launching...')
    time.sleep(15)
    return proc


def launch_nav(params_yaml_path, jetson_cores=JETSON_CORES):
    """Launch Nav2 in Docker container with Jetson Orin Nano constraints.

    Container gets `jetson_cores` CPUs and 7GB RAM.
    Host network shares ROS2 topics with Gazebo on host.
    """
    container_name = 'sendbooster-nav-trial'
    # Kill any leftover container
    subprocess.run(['docker', 'rm', '-f', container_name], capture_output=True)

    home = os.path.expanduser('~')
    cores_str = ','.join(str(i) for i in range(jetson_cores))
    cmd = [
        'docker', 'run', '--rm',
        '--name', container_name,
        '--network', 'host', '--ipc', 'host',
        f'--cpuset-cpus={cores_str}',
        '--memory=7g', '--memory-swap=7g',
        '-v', f'{home}/ros2_ws:{home}/ros2_ws:rw',
        '-v', '/tmp:/tmp:rw',
        'sendbooster-sim',
        'bash', '-c',
        f'source {home}/ros2_ws/install/setup.bash && '
        f'ros2 launch sendbooster_agv_bringup simulation_mppi.launch.py '
        f'headless:=true gazebo:=false nav:=true '
        f'params_file:={params_yaml_path} '
        f'map:={MAP_FILE} '
        f'localization:=amcl',
    ]
    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    return proc


def kill_nav(proc):
    """Kill Nav2 Docker container (keep Gazebo running on host)."""
    subprocess.run(['docker', 'stop', '-t', '5', 'sendbooster-nav-trial'], capture_output=True)
    subprocess.run(['docker', 'rm', '-f', 'sendbooster-nav-trial'], capture_output=True)
    try:
        proc.wait(timeout=10)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
    time.sleep(3.0)


def kill_gazebo(proc):
    """Kill Gazebo process group."""
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=15)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=5)
        except Exception:
            pass
    subprocess.run(['pkill', '-f', 'gzserver'], capture_output=True)
    subprocess.run(['pkill', '-f', 'gzclient'], capture_output=True)
    time.sleep(3.0)


def wait_for_nav2_ready(timeout=180):
    """Wait until navigate_to_pose action is available."""
    t0 = time.monotonic()
    while (time.monotonic() - t0) < timeout:
        try:
            result = subprocess.run(
                ['ros2', 'action', 'list'],
                capture_output=True, text=True, timeout=10,
            )
            if 'navigate_to_pose' in result.stdout:
                return True
        except (subprocess.TimeoutExpired, Exception):
            pass
        time.sleep(3.0)
    return False


def run_metrics_collector(quick=True, timeout=600):
    """Run the metrics collector in the Nav2 Docker container.

    Runs inside the same container so CPU measurement reflects
    Jetson Orin Nano constraints (3 CPUs, 7GB RAM).
    """
    cmd = [
        'docker', 'exec', 'sendbooster-nav-trial',
        'bash', '-c',
        f'source {os.path.expanduser("~")}/ros2_ws/install/setup.bash && '
        f'ros2 run sendbooster_agv_bringup nav_goal_metrics_collector.py '
        f'--ros-args -p use_sim_time:=true '
        f'-p quick:={"true" if quick else "false"}',
    ]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout,
        )
        for line in result.stdout.split('\n'):
            if line.startswith('METRICS_JSON:'):
                return json.loads(line[len('METRICS_JSON:'):])
        for line in result.stderr.split('\n'):
            if 'METRICS_JSON:' in line:
                idx = line.index('METRICS_JSON:')
                return json.loads(line[idx + len('METRICS_JSON:'):])
    except subprocess.TimeoutExpired:
        pass
    except json.JSONDecodeError:
        pass
    return None


def score_config(metrics):
    """Score a configuration. Higher is better."""
    if metrics is None:
        return -9999.0

    sr = metrics.get('success_rate', 0.0)
    avg_time = metrics.get('avg_time_to_goal', 999.0)
    smoothness = metrics.get('avg_path_smoothness', 0.0)
    cpu = metrics.get('cpu_avg_percent', 100.0)

    if cpu > CPU_HARD_LIMIT:
        return -1000.0 + sr * 10

    score = 0.0
    score += sr * 50.0                          # max 50 pts
    score += max(0, 30.0 - avg_time) * 1.0      # faster = more pts (max ~30)
    score += smoothness * 10.0                   # max ~10 pts
    score -= max(0, cpu - 50.0) * 0.5            # soft penalty above 50%

    return round(score, 2)


def docker_cpu_monitor(container_name, jetson_cores, samples, stop_event):
    """Monitor Docker container CPU via docker stats (background thread).

    docker stats shows CPU% relative to total system cores.
    We normalize to `jetson_cores` to get Jetson-equivalent CPU%.
    E.g. on 16-core host, if docker stats shows 150%, that's 150/300*100 = 50% on 3 cores.
    """
    num_system_cores = os.cpu_count() or 16
    while not stop_event.is_set():
        try:
            result = subprocess.run(
                ['docker', 'stats', container_name, '--no-stream',
                 '--format', '{{.CPUPerc}}'],
                capture_output=True, text=True, timeout=5,
            )
            pct_str = result.stdout.strip().replace('%', '')
            if pct_str:
                # docker stats: % of ALL system cores
                # Convert to % of jetson_cores
                system_pct = float(pct_str)
                jetson_pct = system_pct / jetson_cores
                samples.append(jetson_pct)
        except Exception:
            pass
        time.sleep(2.0)


def run_trial(trial_num, params, base_yaml, quick=True, jetson_cores=JETSON_CORES):
    """Run a single trial: launch Nav2, collect metrics, kill Nav2."""
    print(f'\n{"="*60}')
    print(f'Trial {trial_num}: {params}')
    print(f'{"="*60}')

    patched = patch_yaml(base_yaml, params)
    yaml_path = write_trial_yaml(patched, trial_num)

    nav_proc = launch_nav(yaml_path, jetson_cores)

    print(f'  Waiting for Nav2 (Docker: {jetson_cores} cores, 7GB)...')
    if not wait_for_nav2_ready(timeout=180):
        print('  FAILED: Nav2 not ready after 180s')
        kill_nav(nav_proc)
        return None

    print('  Nav2 ready. Waiting 10s for AMCL convergence...')
    time.sleep(10.0)

    # Start docker stats CPU monitoring
    cpu_samples = []
    stop_event = threading.Event()
    cpu_thread = threading.Thread(
        target=docker_cpu_monitor,
        args=('sendbooster-nav-trial', jetson_cores, cpu_samples, stop_event),
        daemon=True,
    )
    cpu_thread.start()

    print('  Running metrics collector...')
    metrics = run_metrics_collector(quick=quick, timeout=600)

    # Stop CPU monitoring
    stop_event.set()
    cpu_thread.join(timeout=5)

    kill_nav(nav_proc)

    if metrics:
        # Override CPU with docker stats measurement
        if cpu_samples:
            metrics['cpu_avg_percent'] = round(sum(cpu_samples) / len(cpu_samples), 2)
            metrics['cpu_max_percent'] = round(max(cpu_samples), 2)
            metrics['cpu_samples'] = len(cpu_samples)

        s = score_config(metrics)
        metrics['score'] = s
        print(f'  Results: success={metrics["success_rate"]:.0%} '
              f'time={metrics["avg_time_to_goal"]:.1f}s '
              f'cpu={metrics["cpu_avg_percent"]:.1f}% (docker stats, {jetson_cores}-core) '
              f'score={s:.1f}')
    else:
        print('  FAILED: No metrics collected')

    return metrics


def run_phase1(base_yaml, jetson_cores):
    """Phase 1: CPU screening with quick tests."""
    print('\n' + '=' * 60)
    print(f'PHASE 1: CPU Budget Screening (Jetson sim: {jetson_cores} cores)')
    print(f'Testing {len(PHASE1_COMBOS)} configurations (quick mode)')
    print(f'CPU budget: {CPU_BUDGET}% | Hard limit: {CPU_HARD_LIMIT}%')
    print('=' * 60)

    results = []
    for i, params in enumerate(PHASE1_COMBOS, 1):
        metrics = run_trial(i, params, base_yaml, quick=True, jetson_cores=jetson_cores)
        results.append({'params': params, 'metrics': metrics})

    feasible = []
    for r in results:
        if r['metrics'] is None:
            continue
        if r['metrics'].get('cpu_avg_percent', 100) <= CPU_BUDGET:
            feasible.append(r)

    feasible.sort(key=lambda r: r['metrics'].get('score', -9999), reverse=True)

    print(f'\nPhase 1 complete. {len(feasible)}/{len(results)} configs within CPU budget.')
    for i, r in enumerate(feasible[:5], 1):
        m = r['metrics']
        print(f'  #{i}: score={m["score"]:.1f} cpu={m["cpu_avg_percent"]:.1f}% '
              f'success={m["success_rate"]:.0%} | {r["params"]}')

    return feasible, results


def run_phase2(base_yaml, feasible_configs, num_trials=20, jetson_cores=JETSON_CORES):
    """Phase 2: Quality tuning — vary critic weights on best CPU configs."""
    if not feasible_configs:
        print('No feasible configs from Phase 1. Aborting Phase 2.')
        return []

    top_bases = feasible_configs[:3]

    print('\n' + '=' * 60)
    print('PHASE 2: Quality Optimization')
    print(f'Testing {num_trials} configurations (full waypoint test)')
    print('=' * 60)

    critic_combos = []
    for _ in range(num_trials):
        combo = {}
        for key, values in PHASE2_SPACE.items():
            combo[key] = random.choice(values)
        critic_combos.append(combo)

    results = []
    trial_offset = len(PHASE1_COMBOS)

    for i, critic_params in enumerate(critic_combos, 1):
        base_config = top_bases[(i - 1) % len(top_bases)]
        merged_params = {**base_config['params'], **critic_params}

        metrics = run_trial(trial_offset + i, merged_params, base_yaml,
                            quick=False, jetson_cores=jetson_cores)
        results.append({'params': merged_params, 'metrics': metrics})

    results.sort(key=lambda r: (r['metrics'] or {}).get('score', -9999), reverse=True)

    print(f'\nPhase 2 complete. Top 5 configurations:')
    for i, r in enumerate(results[:5], 1):
        m = r['metrics']
        if m:
            print(f'  #{i}: score={m["score"]:.1f} cpu={m["cpu_avg_percent"]:.1f}% '
                  f'success={m["success_rate"]:.0%} time={m["avg_time_to_goal"]:.1f}s')
            print(f'        {r["params"]}')

    return results


def write_tuned_yaml(base_yaml, best_params, output_path):
    """Write the optimized YAML file."""
    patched = patch_yaml(base_yaml, best_params)
    ensure_float_params(patched)
    with open(output_path, 'w') as f:
        f.write('# Nav2 Parameters — MPPI Auto-Tuned for Jetson Orin Nano\n')
        f.write(f'# Tuned on {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
        f.write(f'# CPU target: {CPU_BUDGET}% (Jetson Orin Nano, {JETSON_CORES} simulated cores)\n')
        f.write(f'# Best params: {json.dumps(best_params)}\n')
        f.write('#\n')
        yaml.dump(patched, f, default_flow_style=False, allow_unicode=True)
    print(f'\nOptimized config written to: {output_path}')


def save_log(all_results, output_path):
    """Save full tuning log as JSON."""
    clean = []
    for r in all_results:
        clean.append({
            'params': {k: (int(v) if isinstance(v, (int,)) else float(v) if isinstance(v, float) else v)
                       for k, v in r['params'].items()},
            'metrics': r['metrics'],
        })
    with open(output_path, 'w') as f:
        json.dump(clean, f, indent=2)
    print(f'Tuning log saved to: {output_path}')


def main():
    parser = argparse.ArgumentParser(description='MPPI Auto-Tuner (Jetson Orin Nano)')
    parser.add_argument('--phase1-only', action='store_true',
                        help='Run only Phase 1 (CPU screening)')
    parser.add_argument('--phase2-trials', type=int, default=20,
                        help='Number of Phase 2 trials (default: 20)')
    parser.add_argument('--jetson-cores', type=int, default=JETSON_CORES,
                        help=f'CPU cores to simulate Jetson (default: {JETSON_CORES})')
    parser.add_argument('--output', type=str,
                        default=str(PROJECT_DIR / 'config' / 'nav2_params_mppi_tuned.yaml'),
                        help='Output path for tuned YAML')
    args = parser.parse_args()

    jetson_cores = args.jetson_cores
    LOG_DIR.mkdir(exist_ok=True)

    base_yaml = load_base_yaml()

    # Launch Gazebo once (unrestricted)
    print('Launching Gazebo (unrestricted CPU)...')
    gazebo_proc = launch_gazebo()

    try:
        all_results = []

        # Phase 1
        feasible, phase1_results = run_phase1(base_yaml, jetson_cores)
        all_results.extend(phase1_results)

        if not args.phase1_only:
            # Phase 2
            phase2_results = run_phase2(base_yaml, feasible,
                                        num_trials=args.phase2_trials,
                                        jetson_cores=jetson_cores)
            all_results.extend(phase2_results)

        # Find best overall
        valid = [r for r in all_results if r['metrics'] is not None]
        if not valid:
            print('\nERROR: No valid results. Check simulation setup.')
            sys.exit(1)

        valid.sort(key=lambda r: r['metrics']['score'], reverse=True)
        best = valid[0]

        print('\n' + '=' * 60)
        print('BEST CONFIGURATION:')
        print(f'  Score: {best["metrics"]["score"]:.1f}')
        print(f'  Success rate: {best["metrics"]["success_rate"]:.0%}')
        print(f'  Avg time: {best["metrics"]["avg_time_to_goal"]:.1f}s')
        print(f'  CPU: {best["metrics"]["cpu_avg_percent"]:.1f}%')
        print(f'  Params: {best["params"]}')
        print('=' * 60)

        write_tuned_yaml(base_yaml, best['params'], args.output)
        save_log(all_results, str(LOG_DIR / 'tuning_results.json'))

        with open(LOG_DIR / 'tuning_summary.txt', 'w') as f:
            f.write(f'MPPI Auto-Tuning Summary ({time.strftime("%Y-%m-%d %H:%M:%S")})\n')
            f.write(f'Jetson simulation: {jetson_cores} cores, CPU budget {CPU_BUDGET}%\n')
            f.write('=' * 60 + '\n\n')
            f.write(f'Total trials: {len(all_results)}\n')
            f.write(f'Valid trials: {len(valid)}\n\n')
            f.write('Top 5 configurations:\n')
            for i, r in enumerate(valid[:5], 1):
                m = r['metrics']
                f.write(f'  #{i}: score={m["score"]:.1f} '
                        f'cpu={m["cpu_avg_percent"]:.1f}% '
                        f'success={m["success_rate"]:.0%} '
                        f'time={m["avg_time_to_goal"]:.1f}s\n')
                f.write(f'       {r["params"]}\n\n')

    finally:
        print('\nShutting down Gazebo...')
        kill_gazebo(gazebo_proc)


if __name__ == '__main__':
    main()
