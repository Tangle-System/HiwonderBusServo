
#ifndef _FLUIDUM_LIB_SERVO_h
#define _FLUIDUM_LIB_SERVO_h

#include <Arduino.h>

#include "driver/uart.h"

#include <deque>

#define BUS_SERVO_FRAME_HEADER 0x55

#define BUS_SERVO_MOVE_TIME_WRITE 1
#define BUS_SERVO_MOVE_TIME_READ 2
#define BUS_SERVO_MOVE_TIME_WAIT_WRITE 7
#define BUS_SERVO_MOVE_TIME_WAIT_READ 8
#define BUS_SERVO_MOVE_START 11
#define BUS_SERVO_MOVE_STOP 12
#define BUS_SERVO_ID_WRITE 13
#define BUS_SERVO_ID_READ 14
#define BUS_SERVO_ANGLE_OFFSET_ADJUST 17
#define BUS_SERVO_ANGLE_OFFSET_WRITE 18
#define BUS_SERVO_ANGLE_OFFSET_READ 19
#define BUS_SERVO_ANGLE_LIMIT_WRITE 20
#define BUS_SERVO_ANGLE_LIMIT_READ 21
#define BUS_SERVO_VIN_LIMIT_WRITE 22
#define BUS_SERVO_VIN_LIMIT_READ 23
#define BUS_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define BUS_SERVO_TEMP_MAX_LIMIT_READ 25
#define BUS_SERVO_TEMP_READ 26
#define BUS_SERVO_VIN_READ 27
#define BUS_SERVO_POS_READ 28
#define BUS_SERVO_OR_MOTOR_MODE_WRITE 29
#define BUS_SERVO_OR_MOTOR_MODE_READ 30
#define BUS_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define BUS_SERVO_LOAD_OR_UNLOAD_READ 32
#define BUS_SERVO_LED_CTRL_WRITE 33
#define BUS_SERVO_LED_CTRL_READ 34
#define BUS_SERVO_LED_ERROR_WRITE 35
#define BUS_SERVO_LED_ERROR_READ 36

// experimental features
#define BUS_SERVO_DYNAMIC_PD_ADJUST 100
#define BUS_SERVO_DYNAMIC_PD_WRITE 101
#define BUS_SERVO_DYNAMIC_PD_READ 102
#define BUS_SERVO_STATIC_PD_ADJUST 103
#define BUS_SERVO_STATIC_PD_WRITE 104
#define BUS_SERVO_STATIC_PD_READ 105

#define RECEIVE_BUFFER_SIZE 32
#define RX_TIMEOUT_US 5000
#define SHORT_TX_GAP_US 500
#define NORMAL_TX_GAP_US 750
#define LONG_TX_GAP_US 10000

#define CONSTANT_MOTOR_MODE 1
#define CONSTANT_POSITION_MODE 0

#define BUS_SERVO_BROADCAST_ID 254

#define BUS_SERVO_SW_OFFSET 100

class BusServoNetwork {
public:
    class Servo {
    public:
        Servo();
        Servo(const uint8_t id, BusServoNetwork* network);
        ~Servo();

        // returns if successful: position range -200 ~ 1200
        bool readPosition(int16_t& position_out);

        bool readTemperature(uint8_t& temperature_out);

        bool readVin(uint8_t& vin_out);

        // position: 0 ~ 1000
        void setPosition(const int16_t position, const int16_t time);

         // speed: -1000 ~ 1000
        void setSpeed(const int16_t speed); 

        // angles: 0 ~ 1000
        void setAngleLimits(const int16_t min_angle, const int16_t max_angle) const;

        // angles: 0 ~ 1000
        bool getAngleLimits(int16_t& min_angle, int16_t& max_angle) const;

        // offset: -125 ~ 125
        void adjustOffset(const int16_t offset, const bool unload_immediately) const;

        // save offset to servo memory
        void saveOffset() const;

        void load() const;
        void unload() const;

        // broadcast: 254
        uint8_t id() const;

        // broadcast: 254
        void id(const uint8_t id);

        // P: 0 ~ 2000 D: 0ms ~ 2000ms
        void adjustDynamicPD(const int16_t P, const int16_t D) const;

        // P: 0 ~ 2000 D: 0ms ~ 2000ms
        void adjustStaticPD(const int16_t P, const int16_t D) const;

        // save dynamic PD regulator values
        void saveDynamicPD() const;

        // save static PD regulator values
        void saveStaticPD() const;

        // read static PD regulator values
        bool getDynamicPD(int16_t& P, int16_t& D) const;

        // read static PD regulator values
        bool getStaticPD(int16_t& P, int16_t& D) const;

    private:
        BusServoNetwork* m_network;
        uint8_t m_id;
        int16_t m_lastReadPosition;
        bool m_motorMode;
    };

    BusServoNetwork(uart_port_t uart_port);
    ~BusServoNetwork();

    void begin(const int8_t tx_pin, const int8_t tx_en_pin, const int8_t rx_pin, const int8_t rx_en_pin);
    void end();

    void reset();

    Servo getServo(const uint8_t address);
    Servo getServoBroadcast();

private:

    bool m_beginCommunication();
    void m_endCommunication(const int32_t gap_after_packet);
    uint8_t m_computeChecksum(const uint8_t* packet, size_t size);
    size_t m_handleServoResponse(uint8_t* const received_data);
    void m_clearRxBuffer();
    bool m_transmit(const uint8_t data[], const size_t size);
    bool m_changeServoId(const uint8_t old_id, const uint8_t new_id);
    bool m_moveServo(const uint8_t id, const int16_t position, const uint16_t time);
    bool m_stopServoMove(const uint8_t id);
    bool m_writeServoPositionMode(const uint8_t id);
    bool m_writeServoMotorMode(const uint8_t id, const int16_t speed);
    bool m_writeServoAngleLimit(const uint8_t id, const int16_t min_angle, const int16_t max_angle);
    bool m_adjustServoOffset(const uint8_t id, const int16_t offset);
    bool m_writeServoOffset(const uint8_t id);
    bool m_readServoAngleLimit(const uint8_t id, int16_t& min_angle_out, int16_t& max_angle_out);
    bool m_readServoVin(const uint8_t id, uint8_t& raw_vin_out);
    bool m_readServoTemperature(const uint8_t id, uint8_t& raw_temperature_out);
    bool m_readServoPosition(const uint8_t id, int16_t& position_out);
    bool m_unloadServo(const uint8_t id);
    bool m_loadServo(const uint8_t id);

    bool m_adjustDynamicPD(const uint8_t id, const int16_t P, const int16_t D);
    bool m_adjustStaticPD(const uint8_t id, const int16_t P, const int16_t D);
    bool m_writeDynamicPD(const uint8_t id);
    bool m_writeStaticPD(const uint8_t id);
    bool m_readDynamicPD(const uint8_t id, int16_t& P_out, int16_t& D_out);
    bool m_readStaticPD(const uint8_t id, int16_t& P_out, int16_t& D_out);

private:
    uart_port_t m_uart_port;

    int8_t m_tx_pin;
    int8_t m_tx_en_pin;
    int8_t m_rx_pin;
    int8_t m_rx_en_pin;

    int64_t m_last_transmittion;
    int32_t m_current_gap;

    xSemaphoreHandle m_xSemaphoreCommunicating;
};

#endif
