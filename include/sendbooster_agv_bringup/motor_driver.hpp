/**
 * @file motor_driver.hpp
 * @brief MDROBOT Motor Driver Communication Class
 * Based on MD_controller (https://github.com/jjletsgo/MD_controller)
 */

#ifndef SENDBOOSTER_AGV_BRINGUP__MOTOR_DRIVER_HPP_
#define SENDBOOSTER_AGV_BRINGUP__MOTOR_DRIVER_HPP_

#include <string>
#include <mutex>
#include <memory>
#include <functional>
#include <atomic>
#include <serial/serial.h>

namespace sendbooster_agv_bringup
{

// PID Constants (MDROBOT Protocol)
constexpr uint8_t PID_REQ_PID_DATA = 4;
constexpr uint8_t PID_TQ_OFF = 5;
constexpr uint8_t PID_COMMAND = 10;
constexpr uint8_t PID_POSI_RESET = 13;
constexpr uint8_t PID_PNT_VEL_CMD = 207;
constexpr uint8_t PID_MAIN_DATA = 193;
constexpr uint8_t PID_PNT_MAIN_DATA = 210;

// Command Constants
constexpr uint8_t CMD_ALARM_RESET = 1;
constexpr uint8_t CMD_POSI_RESET = 2;
constexpr uint8_t CMD_ESTOP = 5;
constexpr uint8_t CMD_TORQUE_OFF = 6;
constexpr uint8_t CMD_TORQUE_ON = 7;

// Request Constants
constexpr uint8_t REQUEST_PNT_MAIN_DATA = 2;

// Packet Constants
constexpr size_t MAX_PACKET_SIZE = 26;
constexpr size_t MAX_DATA_SIZE = 23;

/**
 * @brief Error codes for motor driver
 */
enum class MotorError : uint8_t
{
    NONE = 0,
    SERIAL_OPEN_FAILED,
    SERIAL_DISCONNECTED,
    SERIAL_WRITE_FAILED,
    SERIAL_READ_FAILED,
    PACKET_CHECKSUM_ERROR,
    PACKET_TIMEOUT,
    MOTOR_INIT_FAILED,
    MOTOR_OVERCURRENT,
    MOTOR_OVERVOLTAGE,
    MOTOR_UNDERVOLTAGE,
    MOTOR_OVERTEMP,
    MOTOR_HALL_ERROR,
    UNKNOWN
};

/**
 * @brief Convert error code to string
 */
inline const char* errorToString(MotorError error)
{
    switch (error) {
        case MotorError::NONE: return "None";
        case MotorError::SERIAL_OPEN_FAILED: return "Serial port open failed";
        case MotorError::SERIAL_DISCONNECTED: return "Serial disconnected";
        case MotorError::SERIAL_WRITE_FAILED: return "Serial write failed";
        case MotorError::SERIAL_READ_FAILED: return "Serial read failed";
        case MotorError::PACKET_CHECKSUM_ERROR: return "Packet checksum error";
        case MotorError::PACKET_TIMEOUT: return "Packet timeout";
        case MotorError::MOTOR_INIT_FAILED: return "Motor init failed";
        case MotorError::MOTOR_OVERCURRENT: return "Motor overcurrent";
        case MotorError::MOTOR_OVERVOLTAGE: return "Motor overvoltage";
        case MotorError::MOTOR_UNDERVOLTAGE: return "Motor undervoltage";
        case MotorError::MOTOR_OVERTEMP: return "Motor overtemperature";
        case MotorError::MOTOR_HALL_ERROR: return "Motor hall sensor error";
        default: return "Unknown error";
    }
}

/**
 * @brief Motor state data structure
 */
struct MotorState
{
    int16_t rpm = 0;
    int32_t position = 0;
    uint8_t status = 0;

    // Extended status (from PID_MAIN_DATA)
    int16_t current = 0;      // Motor current (mA)
    uint8_t driver_temp = 0;  // Driver temperature
    uint8_t motor_temp = 0;   // Motor temperature
    int16_t input_voltage = 0; // Input voltage (0.1V units)

    // Error flags
    bool overcurrent = false;
    bool overvoltage = false;
    bool undervoltage = false;
    bool overtemp = false;
    bool hall_error = false;

    // Timestamp
    uint64_t last_update_ms = 0;

    void clearErrors() {
        overcurrent = false;
        overvoltage = false;
        undervoltage = false;
        overtemp = false;
        hall_error = false;
    }

    bool hasError() const {
        return overcurrent || overvoltage || undervoltage || overtemp || hall_error;
    }
};

/**
 * @brief Byte conversion struct for short (16-bit) values
 */
struct IByte
{
    uint8_t low;
    uint8_t high;
};

/**
 * @brief Callback type for error notifications
 */
using ErrorCallback = std::function<void(MotorError, const std::string&)>;

/**
 * @brief MDROBOT Motor Driver Communication Class
 * Supports dual-channel BLDC motor driver (MD200T, MD400T, MD750T)
 */
class MotorDriver
{
public:
    /**
     * @brief Constructor
     * @param port Serial port path
     * @param baudrate Communication baudrate
     * @param mdui_id PC ID (default 184)
     * @param mdt_id Motor driver ID (default 183)
     * @param motor_id Motor ID for dual channel control
     * @param gear_ratio Motor gear ratio
     * @param poles Motor pole count
     */
    MotorDriver(
        const std::string& port = "/dev/ttyUSB0",
        int baudrate = 57600,
        uint8_t mdui_id = 184,
        uint8_t mdt_id = 183,
        uint8_t motor_id = 1,
        int gear_ratio = 15,
        int poles = 10
    );

    ~MotorDriver();

    /**
     * @brief Connect to motor driver
     * @return true if connection successful
     */
    bool connect();

    /**
     * @brief Disconnect from motor driver
     */
    void disconnect();

    /**
     * @brief Reconnect to motor driver
     * @return true if reconnection successful
     */
    bool reconnect();

    /**
     * @brief Check if connected
     * @return true if connected
     */
    bool isConnected() const;

    /**
     * @brief Send RPM command to dual motors using PID_PNT_VEL_CMD (207)
     * @param rpm_left Left motor RPM (before gear ratio)
     * @param rpm_right Right motor RPM (before gear ratio)
     * @param request_data Request return data
     * @return true if send successful
     */
    bool sendRpm(int rpm_left, int rpm_right, bool request_data = true);

    /**
     * @brief Stop both motors (send RPM 0)
     * @return true if send successful
     */
    bool sendStop();

    /**
     * @brief Emergency stop
     * @return true if send successful
     */
    bool emergencyStop();

    /**
     * @brief Set torque on/off
     * @param enable true to enable torque
     * @return true if send successful
     */
    bool setTorque(bool enable);

    /**
     * @brief Request main data (PID_MAIN_DATA = 193)
     * @return true if send successful
     */
    bool requestMainData();

    /**
     * @brief Request data for specific motor
     * @param motor_id Motor ID (1 or 2)
     * @return true if send successful
     */
    bool requestMotorData(uint8_t motor_id);

    /**
     * @brief Reset motor position to 0
     * @return true if send successful
     */
    bool resetPosition();

    /**
     * @brief Reset alarm/error
     * @return true if send successful
     */
    bool resetAlarm();

    /**
     * @brief Read and parse response from motor driver
     * @return true if valid packet received
     */
    bool readResponse();

    /**
     * @brief Check if motor driver is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Get left motor state
     * @return Left motor state
     */
    const MotorState& getLeftMotor() const { return motor_left_; }

    /**
     * @brief Get right motor state
     * @return Right motor state
     */
    const MotorState& getRightMotor() const { return motor_right_; }

    /**
     * @brief Get last error
     * @return Last error code
     */
    MotorError getLastError() const { return last_error_; }

    /**
     * @brief Get last error message
     * @return Last error message
     */
    const std::string& getLastErrorMessage() const { return last_error_msg_; }

    /**
     * @brief Set error callback
     * @param callback Callback function
     */
    void setErrorCallback(ErrorCallback callback) { error_callback_ = callback; }

    /**
     * @brief Get communication statistics
     */
    uint32_t getTxCount() const { return tx_count_; }
    uint32_t getRxCount() const { return rx_count_; }
    uint32_t getErrorCount() const { return error_count_; }
    uint32_t getChecksumOkCount() const { return checksum_ok_count_; }
    uint32_t getChecksumErrCount() const { return checksum_err_count_; }

    // Getters for parameters
    int getGearRatio() const { return gear_ratio_; }
    int getPoles() const { return poles_; }
    double getPPR() const { return ppr_; }
    double getTickToRad() const { return tick_to_rad_; }
    std::string getPort() const { return port_; }
    int getBaudrate() const { return baudrate_; }

private:
    /**
     * @brief Calculate checksum: (~sum + 1) & 0xFF
     * @param data Packet data
     * @param length Data length
     * @return Checksum byte
     */
    uint8_t calculateChecksum(const uint8_t* data, size_t length);

    /**
     * @brief Convert short to low/high bytes
     * @param value 16-bit value
     * @return IByte struct with low and high bytes
     */
    IByte shortToBytes(int16_t value);

    /**
     * @brief Convert two bytes to short (16-bit signed)
     * @param low Low byte
     * @param high High byte
     * @return 16-bit signed integer
     */
    int16_t bytesToShort(uint8_t low, uint8_t high);

    /**
     * @brief Convert four bytes to long (32-bit signed)
     * @param b1-b4 Bytes (little-endian)
     * @return 32-bit signed integer
     */
    int32_t bytesToLong(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4);

    /**
     * @brief Build and send packet
     * @param pid Command PID
     * @param data Data bytes
     * @param data_size Data size
     * @return true if send successful
     */
    bool sendPacket(uint8_t pid, const uint8_t* data, size_t data_size);

    /**
     * @brief Send command
     * @param cmd Command byte
     * @return true if send successful
     */
    bool sendCommand(uint8_t cmd);

    /**
     * @brief Parse received packet using state machine
     * @param data Raw received bytes
     * @param length Data length
     * @return true if valid packet parsed
     */
    bool parsePacket(const uint8_t* data, size_t length);

    /**
     * @brief Process valid received packet
     */
    void processPacket();

    /**
     * @brief Parse motor status byte
     * @param status Status byte
     * @param motor Motor state to update
     */
    void parseMotorStatus(uint8_t status, MotorState& motor);

    /**
     * @brief Reset state machine
     */
    void resetState();

    /**
     * @brief Set error and notify
     * @param error Error code
     * @param msg Error message
     */
    void setError(MotorError error, const std::string& msg = "");

    /**
     * @brief Get current time in milliseconds
     */
    uint64_t getCurrentTimeMs();

private:
    // Serial connection
    std::unique_ptr<serial::Serial> serial_;
    std::mutex mutex_;

    // Configuration
    std::string port_;
    int baudrate_;
    uint8_t mdui_id_;    // PC ID
    uint8_t mdt_id_;     // Motor driver ID
    uint8_t motor_id_;   // Motor ID

    // Motor parameters
    int gear_ratio_;
    int poles_;
    double ppr_;         // Pulses Per Revolution
    double tick_to_rad_; // Tick to radian conversion

    // Motor states
    MotorState motor_left_;
    MotorState motor_right_;

    // Receive buffers (pre-allocated, no heap alloc per read)
    uint8_t recv_buf_[MAX_PACKET_SIZE];
    uint8_t read_buf_[256];   // Serial read buffer
    uint8_t accum_buf_[512];  // Accumulation buffer for partial packets
    size_t accum_len_ = 0;    // Current accumulated data length
    size_t packet_num_;
    uint8_t step_;
    uint8_t chk_sum_;
    uint8_t max_data_num_;
    uint8_t data_num_;
    uint8_t header_count_;  // Track header bytes across parsePacket calls
    std::atomic<bool> initialized_;

    // Error handling
    MotorError last_error_;
    std::string last_error_msg_;
    ErrorCallback error_callback_;

    // Statistics
    std::atomic<uint32_t> tx_count_;
    std::atomic<uint32_t> rx_count_;
    std::atomic<uint32_t> error_count_;
    std::atomic<uint32_t> checksum_ok_count_{0};
    std::atomic<uint32_t> checksum_err_count_{0};

    // Reconnection
    int reconnect_attempts_;
    static constexpr int MAX_RECONNECT_ATTEMPTS = 5;
};

}  // namespace sendbooster_agv_bringup

#endif  // SENDBOOSTER_AGV_BRINGUP__MOTOR_DRIVER_HPP_
