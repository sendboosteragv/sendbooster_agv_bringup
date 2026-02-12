/**
 * @file motor_driver.cpp
 * @brief MDROBOT Motor Driver Communication Implementation
 * Based on MD_controller (https://github.com/jjletsgo/MD_controller)
 */

#include "sendbooster_agv_bringup/motor_driver.hpp"
#include <iostream>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <vector>

namespace sendbooster_agv_bringup
{

MotorDriver::MotorDriver(
    const std::string& port,
    int baudrate,
    uint8_t mdui_id,
    uint8_t mdt_id,
    uint8_t motor_id,
    int gear_ratio,
    int poles
)
: port_(port)
, baudrate_(baudrate)
, mdui_id_(mdui_id)
, mdt_id_(mdt_id)
, motor_id_(motor_id)
, gear_ratio_(gear_ratio)
, poles_(poles)
, packet_num_(0)
, step_(0)
, chk_sum_(0)
, max_data_num_(0)
, data_num_(0)
, header_count_(0)
, initialized_(false)
, last_error_(MotorError::NONE)
, tx_count_(0)
, rx_count_(0)
, error_count_(0)
, reconnect_attempts_(0)
{
    // Calculate PPR (Pulses Per Revolution)
    // PPR = poles * 3 (Hall U,V,W) * gear_ratio
    ppr_ = static_cast<double>(poles_ * 3 * gear_ratio_);
    tick_to_rad_ = (360.0 / ppr_) * M_PI / 180.0;

    // Initialize receive buffer
    std::memset(recv_buf_, 0, MAX_PACKET_SIZE);
}

MotorDriver::~MotorDriver()
{
    disconnect();
}

bool MotorDriver::connect()
{
    std::lock_guard<std::mutex> lock(mutex_);

    try {
        serial_ = std::make_unique<serial::Serial>(
            port_,
            baudrate_,
            serial::Timeout::simpleTimeout(100)  // 100ms timeout
        );

        if (serial_->isOpen()) {
            std::cout << "[MotorDriver] Serial port opened: " << port_
                      << " @ " << baudrate_ << " bps" << std::endl;
            last_error_ = MotorError::NONE;
            reconnect_attempts_ = 0;
            return true;
        }
    } catch (const serial::IOException& e) {
        setError(MotorError::SERIAL_OPEN_FAILED,
                 std::string("Failed to open port ") + port_ + ": " + e.what());
    } catch (const std::exception& e) {
        setError(MotorError::SERIAL_OPEN_FAILED,
                 std::string("Exception: ") + e.what());
    }
    return false;
}

void MotorDriver::disconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (serial_ && serial_->isOpen()) {
        serial_->close();
        std::cout << "[MotorDriver] Serial port closed" << std::endl;
    }
    initialized_ = false;
}

bool MotorDriver::reconnect()
{
    std::cout << "[MotorDriver] Attempting reconnection... (attempt "
              << reconnect_attempts_ + 1 << "/" << MAX_RECONNECT_ATTEMPTS << ")" << std::endl;

    // Close existing connection
    if (serial_ && serial_->isOpen()) {
        serial_->close();
    }
    serial_.reset();

    // Wait before reconnecting
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Try to reconnect
    if (connect()) {
        reconnect_attempts_ = 0;
        std::cout << "[MotorDriver] Reconnection successful" << std::endl;
        return true;
    }

    reconnect_attempts_++;
    if (reconnect_attempts_ >= MAX_RECONNECT_ATTEMPTS) {
        setError(MotorError::SERIAL_DISCONNECTED, "Max reconnection attempts reached");
    }

    return false;
}

bool MotorDriver::isConnected() const
{
    return serial_ && serial_->isOpen();
}

uint64_t MotorDriver::getCurrentTimeMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

void MotorDriver::setError(MotorError error, const std::string& msg)
{
    last_error_ = error;
    last_error_msg_ = msg.empty() ? errorToString(error) : msg;
    error_count_++;

    std::cerr << "[MotorDriver] Error: " << last_error_msg_ << std::endl;

    if (error_callback_) {
        error_callback_(error, last_error_msg_);
    }
}

uint8_t MotorDriver::calculateChecksum(const uint8_t* data, size_t length)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    return (~sum + 1) & 0xFF;
}

IByte MotorDriver::shortToBytes(int16_t value)
{
    IByte result;
    result.low = static_cast<uint8_t>(value & 0xFF);
    result.high = static_cast<uint8_t>((value >> 8) & 0xFF);
    return result;
}

int16_t MotorDriver::bytesToShort(uint8_t low, uint8_t high)
{
    return static_cast<int16_t>(low | (high << 8));
}

int32_t MotorDriver::bytesToLong(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4)
{
    return static_cast<int32_t>(b1 | (b2 << 8) | (b3 << 16) | (b4 << 24));
}

bool MotorDriver::sendPacket(uint8_t pid, const uint8_t* data, size_t data_size)
{
    if (!isConnected()) {
        // Try to reconnect
        if (!reconnect()) {
            return false;
        }
    }

    uint8_t packet[MAX_PACKET_SIZE];
    size_t idx = 0;

    // Build packet: [RMID, TMID, ID, PID, DataSize, Data..., Checksum]
    packet[idx++] = mdt_id_;       // RMID (Motor driver)
    packet[idx++] = mdui_id_;      // TMID (PC)
    packet[idx++] = motor_id_;     // Motor ID
    packet[idx++] = pid;           // PID
    packet[idx++] = static_cast<uint8_t>(data_size);  // Data size

    // Copy data
    for (size_t i = 0; i < data_size; ++i) {
        packet[idx++] = data[i];
    }

    // Calculate and append checksum
    uint8_t checksum = calculateChecksum(packet, idx);
    packet[idx++] = checksum;

    std::lock_guard<std::mutex> lock(mutex_);
    try {
        serial_->write(packet, idx);
        tx_count_++;
        return true;
    } catch (const serial::SerialException& e) {
        setError(MotorError::SERIAL_WRITE_FAILED, e.what());
        return false;
    } catch (const serial::IOException& e) {
        setError(MotorError::SERIAL_DISCONNECTED, e.what());
        return false;
    }
}

bool MotorDriver::sendCommand(uint8_t cmd)
{
    uint8_t data[1] = {cmd};
    return sendPacket(PID_COMMAND, data, 1);
}

bool MotorDriver::sendRpm(int rpm_left, int rpm_right, bool request_data)
{
    // Apply gear ratio (right motor is inverted for proper differential drive)
    int16_t rpm_left_actual = static_cast<int16_t>(rpm_left * gear_ratio_);
    int16_t rpm_right_actual = static_cast<int16_t>(-rpm_right * gear_ratio_);  // Inverted

    // Debug output (only when non-zero)
    static int16_t last_left = 0, last_right = 0;
    if (rpm_left_actual != last_left || rpm_right_actual != last_right) {
        std::cout << "[MotorDriver] sendRpm: input=(" << rpm_left << ", " << rpm_right
                  << ") actual=(" << rpm_left_actual << ", " << rpm_right_actual << ")" << std::endl;
        last_left = rpm_left_actual;
        last_right = rpm_right_actual;
    }

    IByte left_bytes = shortToBytes(rpm_left_actual);
    IByte right_bytes = shortToBytes(rpm_right_actual);

    // Build data for PID_PNT_VEL_CMD (207)
    // D1: Motor1 Enable, D2-3: Motor1 RPM (L/H)
    // D4: Motor2 Enable, D5-6: Motor2 RPM (L/H)
    // D7: Return data request
    uint8_t data[7];
    data[0] = 1;                    // D1: Motor1 Enable
    data[1] = left_bytes.low;       // D2: Motor1 RPM Low
    data[2] = left_bytes.high;      // D3: Motor1 RPM High
    data[3] = 2;                    // D4: Motor2 Enable
    data[4] = right_bytes.low;      // D5: Motor2 RPM Low
    data[5] = right_bytes.high;     // D6: Motor2 RPM High
    data[6] = request_data ? REQUEST_PNT_MAIN_DATA : 0;  // D7: Return data request

    return sendPacket(PID_PNT_VEL_CMD, data, 7);
}

bool MotorDriver::sendStop()
{
    return sendRpm(0, 0, false);
}

bool MotorDriver::emergencyStop()
{
    // Send stop command first
    sendStop();
    // Then send emergency stop command
    return sendCommand(CMD_ESTOP);
}

bool MotorDriver::setTorque(bool enable)
{
    return sendCommand(enable ? CMD_TORQUE_ON : CMD_TORQUE_OFF);
}

bool MotorDriver::requestMainData()
{
    uint8_t data[1] = {PID_MAIN_DATA};
    return sendPacket(PID_REQ_PID_DATA, data, 1);
}

bool MotorDriver::requestMotorData(uint8_t target_motor_id)
{
    // Temporarily change motor_id for request
    uint8_t original_id = motor_id_;
    motor_id_ = target_motor_id;

    uint8_t data[1] = {PID_MAIN_DATA};
    bool result = sendPacket(PID_REQ_PID_DATA, data, 1);

    motor_id_ = original_id;
    return result;
}

bool MotorDriver::resetPosition()
{
    uint8_t data[1] = {0};
    return sendPacket(PID_POSI_RESET, data, 1);
}

bool MotorDriver::resetAlarm()
{
    return sendCommand(CMD_ALARM_RESET);
}

bool MotorDriver::readResponse()
{
    if (!isConnected()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    size_t available = 0;
    try {
        available = serial_->available();
    } catch (const serial::IOException& e) {
        setError(MotorError::SERIAL_DISCONNECTED, e.what());
        return false;
    }

    if (available == 0) {
        return false;
    }

    // Read all available data
    std::vector<uint8_t> buffer(available);
    size_t bytes_read = 0;

    try {
        bytes_read = serial_->read(buffer.data(), available);
        rx_count_++;
    } catch (const serial::SerialException& e) {
        setError(MotorError::SERIAL_READ_FAILED, e.what());
        return false;
    } catch (const serial::IOException& e) {
        setError(MotorError::SERIAL_DISCONNECTED, e.what());
        return false;
    }

    if (bytes_read > 0) {
        // Find packet start (look for header pattern)
        bool packet_ok = false;
        for (size_t i = 0; i < bytes_read; ++i) {
            // Look for header start
            if (buffer[i] == mdui_id_ && i + 1 < bytes_read && buffer[i + 1] == mdt_id_) {
                // Found potential header, reset state and parse from here
                resetState();
                packet_ok = parsePacket(buffer.data() + i, bytes_read - i);
                if (packet_ok) {
                    break;  // Successfully parsed a packet
                }
            }
        }
        return packet_ok;
    }
    return false;
}

bool MotorDriver::parsePacket(const uint8_t* data, size_t length)
{
    bool packet_ok = false;

    for (size_t j = 0; j < length; ++j) {
        if (packet_num_ >= MAX_PACKET_SIZE) {
            resetState();
            return false;
        }

        uint8_t byte = data[j];

        switch (step_) {
            case 0:  // Check RMID/TMID (Header)
                if (byte == mdui_id_ || byte == mdt_id_) {
                    chk_sum_ += byte;
                    recv_buf_[packet_num_++] = byte;
                    header_count_++;
                    if (header_count_ >= 2) {
                        step_ = 1;
                    }
                } else {
                    resetState();
                }
                break;

            case 1:  // Check ID
                if (byte == 1 || byte == 2) {
                    chk_sum_ += byte;
                    recv_buf_[packet_num_++] = byte;
                    step_ = 2;
                } else {
                    resetState();
                }
                break;

            case 2:  // PID
                chk_sum_ += byte;
                recv_buf_[packet_num_++] = byte;
                step_ = 3;
                break;

            case 3:  // Data size
                max_data_num_ = byte;
                data_num_ = 0;
                chk_sum_ += byte;
                recv_buf_[packet_num_++] = byte;
                step_ = 4;
                break;

            case 4:  // Data
                recv_buf_[packet_num_++] = byte;
                chk_sum_ += byte;
                data_num_++;

                if (data_num_ >= MAX_DATA_SIZE) {
                    resetState();
                    break;
                }

                if (data_num_ >= max_data_num_) {
                    step_ = 5;
                }
                break;

            case 5:  // Checksum
                chk_sum_ += byte;
                recv_buf_[packet_num_++] = byte;

                if ((chk_sum_ & 0xFF) == 0) {
                    packet_ok = true;
                    processPacket();
                } else {
                    // Debug: print received packet on checksum error
                    std::cerr << "[MotorDriver] Checksum error - received bytes: ";
                    for (size_t i = 0; i < packet_num_; ++i) {
                        std::cerr << std::hex << static_cast<int>(recv_buf_[i]) << " ";
                    }
                    std::cerr << std::dec << "(sum=" << static_cast<int>(chk_sum_) << ")" << std::endl;
                    setError(MotorError::PACKET_CHECKSUM_ERROR, "Checksum mismatch");
                }
                resetState();
                break;
        }
    }

    return packet_ok;
}

void MotorDriver::parseMotorStatus(uint8_t status, MotorState& motor)
{
    // Parse status byte according to MDROBOT protocol
    // Bit 0: Overcurrent
    // Bit 1: Overvoltage
    // Bit 2: Undervoltage
    // Bit 3: Overtemperature
    // Bit 4: Hall sensor error
    motor.status = status;
    motor.overcurrent = (status & 0x01) != 0;
    motor.overvoltage = (status & 0x02) != 0;
    motor.undervoltage = (status & 0x04) != 0;
    motor.overtemp = (status & 0x08) != 0;
    motor.hall_error = (status & 0x10) != 0;

    // Report errors
    if (motor.overcurrent) {
        setError(MotorError::MOTOR_OVERCURRENT);
    }
    if (motor.overvoltage) {
        setError(MotorError::MOTOR_OVERVOLTAGE);
    }
    if (motor.undervoltage) {
        setError(MotorError::MOTOR_UNDERVOLTAGE);
    }
    if (motor.overtemp) {
        setError(MotorError::MOTOR_OVERTEMP);
    }
    if (motor.hall_error) {
        setError(MotorError::MOTOR_HALL_ERROR);
    }
}

void MotorDriver::processPacket()
{
    uint8_t pid = recv_buf_[3];
    uint8_t recv_motor_id = recv_buf_[2];
    uint64_t now = getCurrentTimeMs();

    MotorState* motor = nullptr;
    if (recv_motor_id == 1) {
        motor = &motor_left_;
    } else if (recv_motor_id == 2) {
        motor = &motor_right_;
    } else {
        return;  // Invalid motor ID
    }

    if (pid == PID_MAIN_DATA) {
        // Parse PID_MAIN_DATA (193) response
        // Offset 5: RPM (2 bytes)
        // Offset 7: Current (2 bytes)
        // Offset 9: Status (1 byte)
        // Offset 10-13: Reserved
        // Offset 14: Driver temp
        // Offset 15-18: Position (4 bytes)

        motor->rpm = bytesToShort(recv_buf_[5], recv_buf_[6]);
        motor->current = bytesToShort(recv_buf_[7], recv_buf_[8]);
        parseMotorStatus(recv_buf_[9], *motor);
        motor->driver_temp = recv_buf_[14];
        motor->position = bytesToLong(recv_buf_[15], recv_buf_[16], recv_buf_[17], recv_buf_[18]);
        motor->last_update_ms = now;

        initialized_ = true;
        last_error_ = MotorError::NONE;

    } else if (pid == PID_PNT_MAIN_DATA) {
        // Parse PID_PNT_MAIN_DATA (210) response - dual motor data
        // This contains data for both motors in one packet

        // Motor 1 data
        motor_left_.rpm = bytesToShort(recv_buf_[5], recv_buf_[6]);
        motor_left_.current = bytesToShort(recv_buf_[7], recv_buf_[8]);
        parseMotorStatus(recv_buf_[9], motor_left_);
        motor_left_.position = bytesToLong(recv_buf_[10], recv_buf_[11], recv_buf_[12], recv_buf_[13]);
        motor_left_.last_update_ms = now;

        // Motor 2 data (if available in packet)
        if (max_data_num_ >= 14) {
            motor_right_.rpm = bytesToShort(recv_buf_[14], recv_buf_[15]);
            motor_right_.current = bytesToShort(recv_buf_[16], recv_buf_[17]);
            parseMotorStatus(recv_buf_[18], motor_right_);
            motor_right_.position = bytesToLong(recv_buf_[19], recv_buf_[20], recv_buf_[21], recv_buf_[22]);
            motor_right_.last_update_ms = now;
        }

        initialized_ = true;
        last_error_ = MotorError::NONE;
    }
}

void MotorDriver::resetState()
{
    step_ = 0;
    packet_num_ = 0;
    chk_sum_ = 0;
    data_num_ = 0;
    max_data_num_ = 0;
    header_count_ = 0;
}

}  // namespace sendbooster_agv_bringup
