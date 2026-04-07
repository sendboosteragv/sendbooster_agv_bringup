#!/bin/bash
# Manually activate Nav2 lifecycle nodes in correct order
# Avoids race condition in Nav2 Humble's autostart
set -e

echo "=== Activating Nav2 lifecycle nodes ==="

# Phase 1: Localization
echo "[1/4] Configuring + activating map_server..."
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
sleep 1

echo "[2/4] Configuring + activating amcl..."
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
sleep 2

# Phase 2: Navigation
for node in controller_server smoother_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother; do
    echo "[3/4] Configuring $node..."
    ros2 lifecycle set /$node configure
done
sleep 1

for node in controller_server smoother_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother; do
    echo "[4/4] Activating $node..."
    ros2 lifecycle set /$node activate
    sleep 0.5
done

echo "=== All Nav2 nodes active ==="
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /amcl
