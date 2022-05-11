
#include "HiwonderBusServo.h"

#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits integer

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

BusServoNetwork::Servo::Servo()
    : m_network(nullptr)
    , m_id(0)
    , m_lastReadPosition(0)
    , m_motorMode(true)
{
}

BusServoNetwork::Servo::Servo(const uint8_t id, BusServoNetwork* network)
    : m_network(network)
    , m_id(id)
    , m_lastReadPosition(0)
    , m_motorMode(true)
{
}

BusServoNetwork::Servo::~Servo()
{
}

// position: -200 ~ 1200
bool BusServoNetwork::Servo::readPosition(int16_t& position_out)
{
    if (m_network) {
        int16_t position;

        if (!m_network->m_readServoPosition(m_id, position)) {
            log_e("reading servo position failed");
            position_out = m_lastReadPosition;
            return false;
        }

        m_lastReadPosition = position_out = position - BUS_SERVO_SW_OFFSET;
        return true;
    } else {
        log_e("invalid servo network");
        return false;
    }
}

bool BusServoNetwork::Servo::readTemperature(uint8_t& temperature_out)
{
    if (m_network) {
        if (!m_network->m_readServoTemperature(m_id, temperature_out)) {
            log_e("reading servo temperature failed");
            return false;
        }

        return true;
    } else {
        log_e("invalid servo network");
        return false;
    }
}

bool BusServoNetwork::Servo::readVin(uint8_t& vin_out)
{
    if (m_network) {
        if (!m_network->m_readServoVin(m_id, vin_out)) {
            log_e("reading servo vin failed");
            return false;
        }

        return true;
    } else {
        log_e("invalid servo network");
        return false;
    }
}

void BusServoNetwork::Servo::setSpeed(const int16_t speed)
{
    if (m_network) {
        if (speed != 0) {
            if (!m_network->m_writeServoMotorMode(m_id, speed)) {
                log_e("writing servo speed failed");
            }
            m_motorMode = true;
        } else {
            if (!m_network->m_writeServoPositionMode(m_id)) {
                log_e("writing servo position mode failed");
            }
            m_motorMode = false;
        }
    } else {
        log_e("invalid servo network");
    }
}

void BusServoNetwork::Servo::setPosition(const int16_t position, const int16_t time)
{
    if (m_network) {

        if (m_motorMode) {
            if (!m_network->m_writeServoPositionMode(m_id)) {
                log_e("writing servo position mode failed");
            }
            m_motorMode = false;
        }

        if (!m_network->m_moveServo(m_id, position + BUS_SERVO_SW_OFFSET, time)) {
            log_e("setting servo position failed");
        }
    } else {
        log_e("invalid servo network");
    }
}

// position: 0 ~ 1000
void BusServoNetwork::Servo::setAngleLimits(const int16_t min_angle, const int16_t max_angle) const
{
    if (m_network) {
        if (!m_network->m_writeServoAngleLimit(m_id, min_angle + BUS_SERVO_SW_OFFSET, max_angle + BUS_SERVO_SW_OFFSET)) {
            log_e("setting of angle limits failed");
        }
    } else {
        log_e("invalid servo network");
    }
}

// position: 0 ~ 1000
bool BusServoNetwork::Servo::getAngleLimits(int16_t& min_angle, int16_t& max_angle) const
{
    if (m_network) {
        if (!m_network->m_readServoAngleLimit(m_id, min_angle, max_angle)) {
            log_e("reading angle limits failed");
            return false;
        }
    } else {
        log_e("invalid servo network");
        return false;
    }

    min_angle -= BUS_SERVO_SW_OFFSET;
    max_angle -= BUS_SERVO_SW_OFFSET;

    return true;
}

// offset: -125 ~ 125
void BusServoNetwork::Servo::adjustOffset(const int16_t offset, const bool unload_immediately) const
{
    ////delay(250);

    if (m_network) {
        if (!m_network->m_adjustServoOffset(m_id, offset)) {
            log_e("adjusting servo offset failed");
        }
        if (unload_immediately) {
            if (!m_network->m_unloadServo(m_id)) {
                log_e("unloading servo offset failed");
            }
            //delay(250);
        }
    } else {
        log_e("invalid servo network");
    }

    ////delay(250);
}

void BusServoNetwork::Servo::saveOffset() const
{
    //delay(250);

    if (m_network) {
        if (!m_network->m_writeServoOffset(m_id)) {
            log_e("writing servo offset failed");
        }
    } else {
        log_e("invalid servo network");
    }

    //delay(250);
}

void BusServoNetwork::Servo::load() const
{
    //delay(250);

    if (m_network) {
        if (!m_network->m_loadServo(m_id)) {
            log_e("load servo failed");
        }
    } else {
        log_e("invalid servo network");
    }

    //delay(250);
}

void BusServoNetwork::Servo::unload() const
{
    //delay(250);

    if (m_network) {
        if (!m_network->m_unloadServo(m_id)) {
            log_e("unload servo failed");
        }
    } else {
        log_e("invalid servo network");
    }

    //delay(250);
}

uint8_t BusServoNetwork::Servo::id() const
{
    return m_id;
}

void BusServoNetwork::Servo::id(const uint8_t id)
{
    if (!m_network) {
        log_e("invalid servo network");
        return;
    }

    // servo broadcast
    if (id >= BUS_SERVO_BROADCAST_ID) {
        if (!m_network->m_changeServoId(m_id, BUS_SERVO_BROADCAST_ID)) {
            log_e("m_changeServoId failed");
            return;
        }
        m_id = BUS_SERVO_BROADCAST_ID;
    } 
    // normal servo adress
    else {
        if (!m_network->m_changeServoId(m_id, id)) {
            log_e("m_changeServoId failed");
            return;
        }
        m_id = id;
    }
}

// P: 0 ~ 2000 D: 0ms ~ 2000ms
void BusServoNetwork::Servo::adjustDynamicPD(const int16_t P, const int16_t D) const
{
    if (!m_network) {
        log_e("invalid servo network");
        return;
    }
    
    if (!m_network->m_adjustDynamicPD(m_id, P, D)) {
        log_e("adjusting PD regulator failed");
        return;
    }

}

// P: 0 ~ 2000 D: 0ms ~ 2000ms
void BusServoNetwork::Servo::adjustStaticPD(const int16_t P, const int16_t D) const
{
    if (!m_network) {
        log_e("invalid servo network");
        return;
    }

    if (!m_network->m_adjustStaticPD(m_id, P, D)) {
        log_e("adjusting PD regulator failed");
        return;
    }
}

// save dynamic PD regulator values
void BusServoNetwork::Servo::saveDynamicPD() const
{
    if (!m_network) {
        log_e("invalid servo network");
        return;
    }

    if (!m_network->m_writeDynamicPD(m_id)) {
        log_e("saving PD failed");
         return;
    }
}

// save static PD regulator values
void BusServoNetwork::Servo::saveStaticPD() const
{
    if (!m_network) {
        log_e("invalid servo network");
         return;
    }

    if (!m_network->m_writeStaticPD(m_id)) {
        log_e("saving PD failed");
         return;
    }
}

// read static PD regulator values
bool BusServoNetwork::Servo::getDynamicPD(int16_t& P, int16_t& D) const
{
    if (!m_network) {
        log_e("invalid servo network");
        return false;
    }

    if (!m_network->m_readDynamicPD(m_id, P, D)) {
        log_e("reading servo dynamic PD failed");
        return false;
    }

    return true;
}

// read static PD regulator values
bool BusServoNetwork::Servo::getStaticPD(int16_t& P, int16_t& D) const
{
    if (!m_network) {
        log_e("invalid servo network");
        return false;
    }

    if (!m_network->m_readStaticPD(m_id, P, D)) {
        log_e("reading servo static PD failed");
        return false;
    }

    return true;
}

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

BusServoNetwork::BusServoNetwork(uart_port_t uart_port)
    : m_uart_port(uart_port)
    , m_tx_pin(-1)
    , m_tx_en_pin(-1)
    , m_rx_pin(-1)
    , m_rx_en_pin(-1)
    , m_last_transmittion(0)
    , m_current_gap(NORMAL_TX_GAP_US)
    , m_xSemaphoreCommunicating(xSemaphoreCreateMutex())
{
    assert(m_xSemaphoreCommunicating);
}

void BusServoNetwork::begin(const int8_t tx_pin, const int8_t tx_en_pin, const int8_t rx_pin, const int8_t rx_en_pin)
{
    m_tx_pin = tx_pin;
    m_tx_en_pin = tx_en_pin;
    m_rx_pin = rx_pin;
    m_rx_en_pin = rx_en_pin;

    pinMode(m_tx_en_pin, OUTPUT);
    pinMode(m_rx_en_pin, OUTPUT);

    if (m_uart_port == UART_NUM_0) {
        Serial.end();
    } else if (m_uart_port == UART_NUM_1) {
        Serial1.end();
    } else if (m_uart_port == UART_NUM_2) {
        Serial2.end();
    } else {
        abort();
    }

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(m_uart_port, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(m_uart_port, m_tx_pin, m_rx_pin, m_tx_en_pin, -1));

    // Setup UART buffered IO with event queue
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(m_uart_port, 256, 0, 0, nullptr, 0)); // blocking transmittion

    ESP_ERROR_CHECK(uart_set_mode(m_uart_port, uart_mode_t::UART_MODE_RS485_HALF_DUPLEX));

    digitalWrite(m_rx_en_pin, HIGH);
}

void BusServoNetwork::end()
{
    ESP_ERROR_CHECK(uart_driver_delete(m_uart_port));

    pinMode(m_tx_en_pin, INPUT_PULLDOWN);
    pinMode(m_rx_en_pin, INPUT_PULLDOWN);
}

void BusServoNetwork::reset()
{
    m_clearRxBuffer();
}

BusServoNetwork::Servo BusServoNetwork::getServo(const uint8_t address)
{
    return BusServoNetwork::Servo(address, this);
}

BusServoNetwork::Servo BusServoNetwork::getServoBroadcast()
{
    return BusServoNetwork::Servo(BUS_SERVO_BROADCAST_ID, this);
}

bool BusServoNetwork::m_beginCommunication()
{
    if (!xSemaphoreTake(m_xSemaphoreCommunicating, (RX_TIMEOUT_US + LONG_TX_GAP_US + NORMAL_TX_GAP_US + SHORT_TX_GAP_US) / 1000)) {
        log_e("Failed to take m_xSemaphoreCommunicating");
        return false;
    }

    const int16_t transmittion_delta = esp_timer_get_time() - m_last_transmittion;
    if (transmittion_delta < m_current_gap) {
        delayMicroseconds(m_current_gap - transmittion_delta); // little pause between transmiting packets
    }

    return true;
}

void BusServoNetwork::m_endCommunication(const int32_t gap_after_packet)
{
    m_last_transmittion = esp_timer_get_time();
    m_current_gap = gap_after_packet;

    xSemaphoreGive(m_xSemaphoreCommunicating);
}

uint8_t BusServoNetwork::m_computeChecksum(const uint8_t* packet, size_t size)
{
    uint16_t temp = 0;
    for (size_t i = 2; i < size - 1; i++) {
        temp += packet[i];
    }
    return ~temp;
}

enum state_t {
    RECEIVING_HEADER_1 = 0,
    RECEIVING_HEADER_2 = 1,
    RECEIVING_ID = 2,
    RECEIVING_LENGTH = 3,
    RECEIVING_COMMAND = 4,
    RECEIVING_PARAMETERS = 5,
    RECEIVING_CHECKSUM = 6,
};

size_t BusServoNetwork::m_handleServoResponse(uint8_t* const received_data)
{
    uint8_t index = 0;
    uint8_t length = 0;
    uint8_t buffer[RECEIVE_BUFFER_SIZE];
    int64_t receive_timeout = esp_timer_get_time() + RX_TIMEOUT_US;
    state_t state = RECEIVING_HEADER_1;

    while (true) {

        uint8_t data[RECEIVE_BUFFER_SIZE];
        size_t size = uart_read_bytes(m_uart_port, data, RECEIVE_BUFFER_SIZE, 0);

        for (size_t i = 0; i < size; i++) {

            switch (state) {

            case RECEIVING_HEADER_1:
                log_v("RECEIVING_HEADER_1");
                if (data[i] == BUS_SERVO_FRAME_HEADER) {
                    buffer[index++] = BUS_SERVO_FRAME_HEADER;
                    state = RECEIVING_HEADER_2;
                } else {
                    log_e("raceived crap: 0x%x", data[i]);
                }
                break;

            case RECEIVING_HEADER_2:
                log_v("RECEIVING_HEADER_2");
                if (data[i] == BUS_SERVO_FRAME_HEADER) {
                    buffer[index++] = BUS_SERVO_FRAME_HEADER;
                    state = RECEIVING_ID;
                } else {
                    log_e("receive header fail");
                }
                break;

            case RECEIVING_ID:
                log_v("RECEIVING_ID");
                buffer[index++] = data[i];
                state = RECEIVING_LENGTH;

                break;

            case RECEIVING_LENGTH:
                log_v("RECEIVING_LENGTH");
                length = data[i];
                log_v("length: %u", length);
                buffer[index++] = length;
                state = RECEIVING_COMMAND;

                break;

            case RECEIVING_COMMAND:
                log_v("RECEIVING_COMMAND");
                buffer[index++] = data[i];
                state = RECEIVING_PARAMETERS;

                break;

            case RECEIVING_PARAMETERS:
                log_v("RECEIVING_PARAMETERS");
                buffer[index++] = data[i];

                if (index - 2 >= length) {
                    state = RECEIVING_CHECKSUM;
                }

                break;

            case RECEIVING_CHECKSUM:

                buffer[index++] = data[i];

                uint8_t checksum = data[i];
                log_v("checksum: %u", checksum);
                uint8_t size = index;
                log_v("size: %u", size);

                if (m_computeChecksum(buffer, size) == checksum) {
                    log_v("Servo checksum OK");
                    memcpy(received_data, buffer, size);
                    return size;
                } else {
                    log_e("Servo checksum error");
                    return 0;
                }

                break;
            }
        }

        if (esp_timer_get_time() >= receive_timeout) {
            log_e("Servo response timeout");
            return 0;
        }
    }

    log_e("Servo response handle fail");
    return 0;
}

void BusServoNetwork::m_clearRxBuffer()
{
    ESP_ERROR_CHECK(uart_flush(m_uart_port));
}

bool BusServoNetwork::m_transmit(const uint8_t data[], const size_t size)
{

    const size_t transmitted = uart_write_bytes(m_uart_port, (const char*)data, size); // blocks until the transmittion is done

    if (transmitted != size) {
        log_e("transmittion fail");
        return false;
    }

    return true;
}

// position 0 - 1000  (0° ~ 240°)
// time 0 - 30000  (ms)
bool BusServoNetwork::m_moveServo(const uint8_t id, const int16_t position, const uint16_t time)
{
    uint16_t position_data;
    uint16_t time_data;

    if (position < 0) {
        position_data = 0;
    } else if (position > 1000) {
        position_data = 1000;
    } else {
        position_data = uint16_t(position);
    }

    if (time < 0) {
        time_data = 0;
    } else if (time > 30000) {
        time_data = 30000;
    } else {
        time_data = uint16_t(time);
    }

    uint8_t buffer[10];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 7;
    buffer[4] = BUS_SERVO_MOVE_TIME_WRITE;
    buffer[5] = GET_LOW_BYTE(position_data);
    buffer[6] = GET_HIGH_BYTE(position_data);
    buffer[7] = GET_LOW_BYTE(time_data);
    buffer[8] = GET_HIGH_BYTE(time_data);
    buffer[9] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(SHORT_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_changeServoId(const uint8_t old_id, const uint8_t new_id)
{
    uint8_t buffer[7];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = old_id;
    buffer[3] = 4;
    buffer[4] = BUS_SERVO_ID_WRITE;
    buffer[5] = new_id;
    buffer[6] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_stopServoMove(const uint8_t id)
{
    uint8_t buffer[6];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 3;
    buffer[4] = BUS_SERVO_MOVE_STOP;
    buffer[5] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_writeServoPositionMode(const uint8_t id)
{
    uint8_t buffer[10];

    buffer[0] = BUS_SERVO_FRAME_HEADER; // header
    buffer[1] = BUS_SERVO_FRAME_HEADER; // header
    buffer[2] = id; // id
    buffer[3] = 7; // length
    buffer[4] = BUS_SERVO_OR_MOTOR_MODE_WRITE; // command
    buffer[5] = CONSTANT_POSITION_MODE;
    buffer[6] = 0;
    buffer[7] = 0;
    buffer[8] = 0;
    buffer[9] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

// speed -1000 ~ 1000
bool BusServoNetwork::m_writeServoMotorMode(const uint8_t id, const int16_t speed)
{
    uint16_t speed_data;

    if (speed < -1000) {
        speed_data = -1000;
    } else if (speed > 1000) {
        speed_data = 1000;
    } else {
        speed_data = uint16_t(speed);
    }

    uint8_t buffer[10];

    buffer[0] = BUS_SERVO_FRAME_HEADER; // header
    buffer[1] = BUS_SERVO_FRAME_HEADER; // header
    buffer[2] = id; // id
    buffer[3] = 7; // length
    buffer[4] = BUS_SERVO_OR_MOTOR_MODE_WRITE; // command
    buffer[5] = CONSTANT_MOTOR_MODE;
    buffer[6] = 0;
    buffer[7] = GET_LOW_BYTE(speed_data);
    buffer[8] = GET_HIGH_BYTE(speed_data);
    buffer[9] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

// // speed -1000 ~ 1000
// bool BusServoNetwork::m_switchToPositionMode(const uint8_t id)
// {
//     uint8_t buffer[10];

//     buffer[0] = BUS_SERVO_FRAME_HEADER; // header
//     buffer[1] = BUS_SERVO_FRAME_HEADER; // header
//     buffer[2] = id; // id
//     buffer[3] = 7; // length
//     buffer[4] = BUS_SERVO_OR_MOTOR_MODE_WRITE; // command
//     buffer[5] = CONSTANT_POSITION_MODE;
//     buffer[6] = 0;
//     buffer[7] = 0;
//     buffer[8] = 0;
//     buffer[9] = m_computeChecksum(buffer, sizeof(buffer));

//     return m_transmit(buffer, sizeof(buffer), NORMAL_TX_GAP_US);
// }

bool BusServoNetwork::m_loadServo(const uint8_t id)
{
    uint8_t buffer[7];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 4;
    buffer[4] = BUS_SERVO_LOAD_OR_UNLOAD_WRITE;
    buffer[5] = 1;
    buffer[6] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(LONG_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_unloadServo(const uint8_t id)
{
    uint8_t buffer[7];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 4;
    buffer[4] = BUS_SERVO_LOAD_OR_UNLOAD_WRITE;
    buffer[5] = 0;
    buffer[6] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(LONG_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_writeServoAngleLimit(const uint8_t id, const int16_t min_angle, const int16_t max_angle)
{
    int16_t min_angle_data;
    int16_t max_angle_data;

    if (min_angle < 0) {
        min_angle_data = 0;
    } else if (min_angle > 1000) {
        min_angle_data = 1000;
    } else {
        min_angle_data = int16_t(min_angle);
    }

    if (max_angle < 0) {
        max_angle_data = 0;
    } else if (max_angle > 1000) {
        max_angle_data = 1000;
    } else {
        max_angle_data = int16_t(max_angle);
    }

    if (max_angle_data == 0) {
        max_angle_data = 1;
    }

    if (min_angle_data >= max_angle_data) {
        min_angle_data = max_angle_data - 1;
    }

    uint8_t buffer[10];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 7;
    buffer[4] = BUS_SERVO_ANGLE_LIMIT_WRITE;
    buffer[5] = GET_LOW_BYTE(min_angle_data);
    buffer[6] = GET_HIGH_BYTE(min_angle_data);
    buffer[7] = GET_LOW_BYTE(max_angle_data);
    buffer[8] = GET_HIGH_BYTE(max_angle_data);
    buffer[9] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_adjustServoOffset(const uint8_t id, const int16_t offset)
{
    int8_t offset_data;

    if (offset < -125) {
        offset_data = -125;
    } else if (offset > 125) {
        offset_data = 125;
    } else {
        offset_data = int8_t(offset);
    }

    uint8_t buffer[7];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 4;
    buffer[4] = BUS_SERVO_ANGLE_OFFSET_ADJUST;
    buffer[5] = uint8_t(offset_data);
    buffer[6] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_writeServoOffset(const uint8_t id)
{
    uint8_t buffer[6];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 3;
    buffer[4] = BUS_SERVO_ANGLE_OFFSET_WRITE;
    buffer[5] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_readServoAngleLimit(const uint8_t id, int16_t& min_angle_out, int16_t& max_angle_out)
{
    uint8_t buffer[6];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 3;
    buffer[4] = BUS_SERVO_ANGLE_LIMIT_READ;
    buffer[5] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {

        m_clearRxBuffer();

        if (m_transmit(buffer, sizeof(buffer))) {
            uint8_t response[RECEIVE_BUFFER_SIZE];

            if (m_handleServoResponse(response) > 0) {
                min_angle_out = response[6] << 8 | response[5];
                max_angle_out = response[8] << 8 | response[7];
                result = true;
            }
        }

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_readServoVin(const uint8_t id, uint8_t& raw_vin_out)
{
    uint8_t buffer[6];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 3;
    buffer[4] = BUS_SERVO_VIN_READ;
    buffer[5] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {

        m_clearRxBuffer();

        if (m_transmit(buffer, sizeof(buffer))) {
            uint8_t response[RECEIVE_BUFFER_SIZE];

            if (m_handleServoResponse(response) > 0) {
                raw_vin_out = response[5];
                result = true;
            }
        }

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_readServoTemperature(const uint8_t id, uint8_t& raw_temperature_out)
{
    uint8_t buffer[6];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 3;
    buffer[4] = BUS_SERVO_TEMP_READ;
    buffer[5] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {

        m_clearRxBuffer();

        if (m_transmit(buffer, sizeof(buffer))) {
            uint8_t response[RECEIVE_BUFFER_SIZE];

            if (m_handleServoResponse(response) > 0) {
                raw_temperature_out = response[5];
                result = true;
            }
        }

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_readServoPosition(const uint8_t id, int16_t& position_out)
{
    uint8_t buffer[6];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 3;
    buffer[4] = BUS_SERVO_POS_READ;
    buffer[5] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {

        m_clearRxBuffer();

        if (m_transmit(buffer, sizeof(buffer))) {
            uint8_t response[RECEIVE_BUFFER_SIZE];

            if (m_handleServoResponse(response) > 0) {
                position_out = response[6] << 8 | response[5];
                result = true;
            }
        }

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

// bool BusServoNetwork::m_readServoVin(const uint8_t id, int16_t& vin_out)
// {
//     uint8_t buffer[6];

//     buffer[0] = BUS_SERVO_FRAME_HEADER;
//     buffer[1] = BUS_SERVO_FRAME_HEADER;
//     buffer[2] = id;
//     buffer[3] = 3;
//     buffer[4] = BUS_SERVO_VIN_READ;
//     buffer[5] = m_computeChecksum(buffer, sizeof(buffer));

//     if (!m_beginCommunication()) {
//          return false;
//     }

//     m_clearRxBuffer();

//     if(transmit(buffer, sizeof(buffer))) {
//          log_e("Failed to transmit to the servo");
//         return false;
//     }

//     uint8_t response[RECEIVE_BUFFER_SIZE];
//     if (!m_handleServoResponse(response)) {
//         log_e("Failed to get the response from the servo");
//         return false;
//     }

//     vin_out = response[6] << 8 | response[5];

//   m_endCommunication(NORMAL_TX_GAP_US);

//     return true;
// }

bool BusServoNetwork::m_adjustDynamicPD(const uint8_t id, const int16_t P, const int16_t D)
{
    int16_t P_data;
    int16_t D_data;

    if (P < 0) {
        P_data = 0;
    } else if (P > 2000) {
        P_data = 2000;
    } else {
        P_data = int16_t(P);
    }

    if (D < 0) {
        D_data = 0;
    } else if (D > 2000) {
        D_data = 2000;
    } else {
        D_data = int16_t(D);
    }

    uint8_t buffer[10];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 7;
    buffer[4] = BUS_SERVO_DYNAMIC_PD_ADJUST;
    buffer[5] = GET_LOW_BYTE(P_data);
    buffer[6] = GET_HIGH_BYTE(P_data);
    buffer[7] = GET_LOW_BYTE(D_data);
    buffer[8] = GET_HIGH_BYTE(D_data);
    buffer[9] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_adjustStaticPD(const uint8_t id, const int16_t P, const int16_t D)
{
    int16_t P_data;
    int16_t D_data;

    if (P < 0) {
        P_data = 0;
    } else if (P > 2000) {
        P_data = 2000;
    } else {
        P_data = int16_t(P);
    }

    if (D < 0) {
        D_data = 0;
    } else if (D > 2000) {
        D_data = 2000;
    } else {
        D_data = int16_t(D);
    }

    uint8_t buffer[10];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 7;
    buffer[4] = BUS_SERVO_STATIC_PD_ADJUST;
    buffer[5] = GET_LOW_BYTE(P_data);
    buffer[6] = GET_HIGH_BYTE(P_data);
    buffer[7] = GET_LOW_BYTE(D_data);
    buffer[8] = GET_HIGH_BYTE(D_data);
    buffer[9] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_writeDynamicPD(const uint8_t id)
{
    uint8_t buffer[6];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 3;
    buffer[4] = BUS_SERVO_DYNAMIC_PD_WRITE;
    buffer[5] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_writeStaticPD(const uint8_t id)
{
    uint8_t buffer[6];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 3;
    buffer[4] = BUS_SERVO_STATIC_PD_WRITE;
    buffer[5] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {
        result = m_transmit(buffer, sizeof(buffer));

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_readDynamicPD(const uint8_t id, int16_t& P_out, int16_t& D_out)
{
    uint8_t buffer[6];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 3;
    buffer[4] = BUS_SERVO_DYNAMIC_PD_READ;
    buffer[5] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {

        m_clearRxBuffer();

        if (m_transmit(buffer, sizeof(buffer))) {
            uint8_t response[RECEIVE_BUFFER_SIZE];

            if (m_handleServoResponse(response) > 0) {
                P_out = response[6] << 8 | response[5];
                D_out = response[8] << 8 | response[7];
                result = true;
            }
        }

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

bool BusServoNetwork::m_readStaticPD(const uint8_t id, int16_t& P_out, int16_t& D_out)
{
    uint8_t buffer[6];

    buffer[0] = BUS_SERVO_FRAME_HEADER;
    buffer[1] = BUS_SERVO_FRAME_HEADER;
    buffer[2] = id;
    buffer[3] = 3;
    buffer[4] = BUS_SERVO_STATIC_PD_READ;
    buffer[5] = m_computeChecksum(buffer, sizeof(buffer));

    bool result = false;

    if (m_beginCommunication()) {

        m_clearRxBuffer();

        if (m_transmit(buffer, sizeof(buffer))) {
            uint8_t response[RECEIVE_BUFFER_SIZE];

            if (m_handleServoResponse(response) > 0) {
                P_out = response[6] << 8 | response[5];
                D_out = response[8] << 8 | response[7];
                result = true;
            }
        }

        m_endCommunication(NORMAL_TX_GAP_US);
    }

    return result;
}

BusServoNetwork::~BusServoNetwork()
{
}
