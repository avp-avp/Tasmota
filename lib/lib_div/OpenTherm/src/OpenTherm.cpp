/*
 * OpenTherm Library
 * Original Author: Ihor Melnyk (https://github.com/ihormelnyk/opentherm_library)
 * 
 * Copyright (c) 2019 Ihor Melnyk
 * MIT License
 * 
 * This software is released under the MIT License.
 * https://opensource.org/licenses/MIT
 *
 * ---------------------------------------------
 * Modifications and improvements by Alex Pavlov
 * Copyright (c) 2025 Alex Pavlov
 *
 * Description of changes:
 * - Added support for ESP32 RMT peripheral for OpenTherm communication in Arduino environment.
 *   (https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/rmt.html)
 */

#include "OpenTherm.h"
#if !defined(__AVR__)
#include "FunctionalInterrupt.h"
#endif

OpenTherm::OpenTherm(int inPin, int outPin, bool isSlave) :
    status(OpenThermStatus::NOT_INITIALIZED),
    inPin(inPin),
    outPin(outPin),
    isSlave(isSlave),
    response(0),
    responseStatus(OpenThermResponseStatus::NONE),
    responseTimestamp(0),
    processResponseCallback(NULL)
{
}

void OpenTherm::begin(void (*handleInterruptCallback)(void))
{
    pinMode(inPin, INPUT);
    pinMode(outPin, OUTPUT);
    if (handleInterruptCallback != NULL)
    {
        attachInterrupt(digitalPinToInterrupt(inPin), handleInterruptCallback, CHANGE);
    }
    else
    {
#if !defined(__AVR__)
        attachInterruptArg(
            digitalPinToInterrupt(inPin),
            OpenTherm::handleInterruptHelper,
            this,
            CHANGE
        );
#endif
    }

#ifdef OPENTHERM_USE_RMT
    setupRMT();
#endif

    activateBoiler();
    status = OpenThermStatus::READY;
}

void OpenTherm::begin(void (*handleInterruptCallback)(void), void (*processResponseCallback)(unsigned long, int))
{
    begin(handleInterruptCallback);
    this->processResponseCallback = processResponseCallback;
}

#if !defined(__AVR__)
void OpenTherm::begin()
{
    begin(NULL);
}

void OpenTherm::begin(std::function<void(unsigned long, OpenThermResponseStatus)> processResponseFunction)
{
    begin();
    this->processResponseFunction = processResponseFunction;
}
#endif

bool IRAM_ATTR OpenTherm::isReady()
{
    return status == OpenThermStatus::READY;
}

int IRAM_ATTR OpenTherm::readState()
{
    return digitalRead(inPin);
}

#define OT_LOCAL
#ifdef OT_LOCAL
void OpenTherm::setActiveState()
{
    digitalWrite(outPin, HIGH);
}

void OpenTherm::setIdleState()
{
    digitalWrite(outPin, LOW);
}
#else
void OpenTherm::setActiveState()
{
    digitalWrite(outPin, LOW);
}

void OpenTherm::setIdleState()
{
    digitalWrite(outPin, HIGH);
}
#endif

void OpenTherm::activateBoiler()
{
    setIdleState();
    delay(1000);
}

void OpenTherm::sendBit(bool high)
{
    if (high)
        setActiveState();
    else
        setIdleState();
    delayMicroseconds(500);
    if (high)
        setIdleState();
    else
        setActiveState();
    delayMicroseconds(500);
}

bool OpenTherm::sendRequestAsync(unsigned long request)
{
#ifdef OPENTHERM_USE_RMT
    if (!isReady()) {
        return false;
    }

    status = OpenThermStatus::REQUEST_SENDING;
    response = 0;
    responseStatus = OpenThermResponseStatus::NONE;

    sendRMT(request);
    responseTimestamp = micros();
    status = OpenThermStatus::RESPONSE_WAITING;
#else
    noInterrupts();
    const bool ready = isReady();

    if (!ready)
    {
        interrupts();
        return false;
    }

    status = OpenThermStatus::REQUEST_SENDING;
    response = 0;
    responseStatus = OpenThermResponseStatus::NONE;

#ifdef INC_FREERTOS_H
    BaseType_t schedulerState = xTaskGetSchedulerState();
    if (schedulerState == taskSCHEDULER_RUNNING)
    {
        vTaskSuspendAll();
    }
#endif

    interrupts();

    sendBit(HIGH); // start bit
    for (int i = 31; i >= 0; i--)
    {
        sendBit(bitRead(request, i));
    }
    sendBit(HIGH); // stop bit
    setIdleState();

    responseTimestamp = micros();
    status = OpenThermStatus::RESPONSE_WAITING;

#ifdef INC_FREERTOS_H
    if (schedulerState == taskSCHEDULER_RUNNING) {
        xTaskResumeAll();
    }
#endif

#endif
    return true;
}

unsigned long OpenTherm::sendRequest(unsigned long request)
{
    if (!sendRequestAsync(request))
    {
        return 0;
    }

    while (!isReady())
    {
        process();
        yield();
    }
    return response;
}

bool OpenTherm::sendResponse(unsigned long request)
{
#ifdef OPENTHERM_USE_RMT
    sendRMT(request);
    status = OpenThermStatus::READY;
    return true;
#else
    noInterrupts();
    const bool ready = isReady();

    if (!ready)
    {
        interrupts();
        return false;
    }

    status = OpenThermStatus::REQUEST_SENDING;
    response = 0;
    responseStatus = OpenThermResponseStatus::NONE;

#ifdef INC_FREERTOS_H
    BaseType_t schedulerState = xTaskGetSchedulerState();
    if (schedulerState == taskSCHEDULER_RUNNING)
    {
        vTaskSuspendAll();
    }
#endif

    interrupts();

    sendBit(HIGH); // start bit
    for (int i = 31; i >= 0; i--)
    {
        sendBit(bitRead(request, i));
    }
    sendBit(HIGH); // stop bit
    setIdleState();
    status = OpenThermStatus::READY;

#ifdef INC_FREERTOS_H
    if (schedulerState == taskSCHEDULER_RUNNING) {
        xTaskResumeAll();
    }
#endif

    return true;
#endif
}

unsigned long OpenTherm::getLastResponse()
{
    return response;
}

OpenThermResponseStatus OpenTherm::getLastResponseStatus()
{
    return responseStatus;
}

void IRAM_ATTR OpenTherm::handleInterrupt()
{
    if (isReady())
    {
        if (isSlave && readState() == HIGH)
        {
            status = OpenThermStatus::RESPONSE_WAITING;
        }
        else
        {
            return;
        }
    }

    unsigned long newTs = micros();
    if (status == OpenThermStatus::RESPONSE_WAITING)
    {
        if (readState() == HIGH)
        {
            status = OpenThermStatus::RESPONSE_START_BIT;
            responseTimestamp = newTs;
        }
        else
        {
            status = OpenThermStatus::RESPONSE_INVALID;
            responseTimestamp = newTs;
        }
    }
    else if (status == OpenThermStatus::RESPONSE_START_BIT)
    {
        if ((newTs - responseTimestamp < 750) && readState() == LOW)
        {
            status = OpenThermStatus::RESPONSE_RECEIVING;
            responseTimestamp = newTs;
            responseBitIndex = 0;
        }
        else
        {
            status = OpenThermStatus::RESPONSE_INVALID;
            responseTimestamp = newTs;
        }
    }
    else if (status == OpenThermStatus::RESPONSE_RECEIVING)
    {
        if ((newTs - responseTimestamp) > 750)
        {
            if (responseBitIndex < 32)
            {
                response = (response << 1) | !readState();
                responseTimestamp = newTs;
                responseBitIndex = responseBitIndex + 1;
            }
            else
            { // stop bit
                status = OpenThermStatus::RESPONSE_READY;
                responseTimestamp = newTs;
            }
        }
    }
}

#if !defined(__AVR__)
void IRAM_ATTR OpenTherm::handleInterruptHelper(void* ptr)
{
    static_cast<OpenTherm*>(ptr)->handleInterrupt();
}
#endif

void OpenTherm::processResponse()
{
    if (processResponseCallback != NULL)
    {
        processResponseCallback(response, (int)responseStatus);
    }
#if !defined(__AVR__)
    if (this->processResponseFunction != NULL)
    {
        processResponseFunction(response, responseStatus);
    }
#endif
}

void OpenTherm::process()
{
#ifdef OPENTHERM_USE_RMT
    if (status == OpenThermStatus::RESPONSE_WAITING)
    {
        uint32_t received = receiveRMT();
        if (received != 0)
        {
            response = received;
            responseStatus = isSlave ?
                (isValidRequest(response) ? OpenThermResponseStatus::SUCCESS : OpenThermResponseStatus::INVALID) :
                (isValidResponse(response) ? OpenThermResponseStatus::SUCCESS : OpenThermResponseStatus::INVALID);
            status = OpenThermStatus::READY;
            processResponse();
        }
        else
        {
            responseStatus = OpenThermResponseStatus::TIMEOUT;
            status = OpenThermStatus::DELAY;
            responseTimestamp = micros();
            processResponse();
        }
    }
#else
    noInterrupts();
    OpenThermStatus st = status;
    unsigned long ts = responseTimestamp;
    interrupts();

    if (st == OpenThermStatus::READY)
        return;

    unsigned long newTs = micros();
    if (st != OpenThermStatus::NOT_INITIALIZED && st != OpenThermStatus::DELAY && (newTs - ts) > 1000000)
    {
        status = OpenThermStatus::READY;
        responseStatus = OpenThermResponseStatus::TIMEOUT;
        processResponse();
    }
    else if (st == OpenThermStatus::RESPONSE_INVALID)
    {
        status = OpenThermStatus::DELAY;
        responseStatus = OpenThermResponseStatus::INVALID;
        processResponse();
    }
    else if (st == OpenThermStatus::RESPONSE_READY)
    {
        status = OpenThermStatus::DELAY;
        responseStatus = (isSlave ? isValidRequest(response) : isValidResponse(response)) ? OpenThermResponseStatus::SUCCESS : OpenThermResponseStatus::INVALID;
        processResponse();
    }
    else if (st == OpenThermStatus::DELAY)
    {
        if ((newTs - ts) > (isSlave ? 20000 : 100000))
        {
            status = OpenThermStatus::READY;
        }
    }
#endif
}

bool OpenTherm::parity(unsigned long frame) // odd parity
{
    byte p = 0;
    while (frame > 0)
    {
        if (frame & 1)
            p++;
        frame = frame >> 1;
    }
    return (p & 1);
}

OpenThermMessageType OpenTherm::getMessageType(unsigned long message)
{
    OpenThermMessageType msg_type = static_cast<OpenThermMessageType>((message >> 28) & 7);
    return msg_type;
}

OpenThermMessageID OpenTherm::getDataID(unsigned long frame)
{
    return (OpenThermMessageID)((frame >> 16) & 0xFF);
}

unsigned long OpenTherm::buildRequest(OpenThermMessageType type, OpenThermMessageID id, unsigned int data)
{
    unsigned long request = data;
    if (type == OpenThermMessageType::WRITE_DATA)
    {
        request |= 1ul << 28;
    }
    request |= ((unsigned long)id) << 16;
    if (parity(request))
        request |= (1ul << 31);
    return request;
}

unsigned long OpenTherm::buildResponse(OpenThermMessageType type, OpenThermMessageID id, unsigned int data)
{
    unsigned long response = data;
    response |= ((unsigned long)type) << 28;
    response |= ((unsigned long)id) << 16;
    if (parity(response))
        response |= (1ul << 31);
    return response;
}

bool OpenTherm::isValidResponse(unsigned long response)
{
    if (parity(response))
        return false;
    byte msgType = (response << 1) >> 29;
    return msgType == (byte)OpenThermMessageType::READ_ACK || msgType == (byte)OpenThermMessageType::WRITE_ACK;
}

bool OpenTherm::isValidRequest(unsigned long request)
{
    if (parity(request))
        return false;
    byte msgType = (request << 1) >> 29;
    return msgType == (byte)OpenThermMessageType::READ_DATA || msgType == (byte)OpenThermMessageType::WRITE_DATA;
}

void OpenTherm::end()
{
    detachInterrupt(digitalPinToInterrupt(inPin));
}

OpenTherm::~OpenTherm()
{
    end();
}

const char *OpenTherm::statusToString(OpenThermResponseStatus status)
{
    switch (status)
    {
    case OpenThermResponseStatus::NONE:
        return "NONE";
    case OpenThermResponseStatus::SUCCESS:
        return "SUCCESS";
    case OpenThermResponseStatus::INVALID:
        return "INVALID";
    case OpenThermResponseStatus::TIMEOUT:
        return "TIMEOUT";
    default:
        return "UNKNOWN";
    }
}

const char *OpenTherm::messageTypeToString(OpenThermMessageType message_type)
{
    switch (message_type)
    {
    case OpenThermMessageType::READ_DATA:
        return "READ_DATA";
    case OpenThermMessageType::WRITE_DATA:
        return "WRITE_DATA";
    case OpenThermMessageType::INVALID_DATA:
        return "INVALID_DATA";
    case OpenThermMessageType::RESERVED:
        return "RESERVED";
    case OpenThermMessageType::READ_ACK:
        return "READ_ACK";
    case OpenThermMessageType::WRITE_ACK:
        return "WRITE_ACK";
    case OpenThermMessageType::DATA_INVALID:
        return "DATA_INVALID";
    case OpenThermMessageType::UNKNOWN_DATA_ID:
        return "UNKNOWN_DATA_ID";
    default:
        return "UNKNOWN";
    }
}

// building requests

unsigned long OpenTherm::buildSetBoilerStatusRequest(bool enableCentralHeating, bool enableHotWater, bool enableCooling, bool enableOutsideTemperatureCompensation, bool enableCentralHeating2)
{
    unsigned int data = enableCentralHeating | (enableHotWater << 1) | (enableCooling << 2) | (enableOutsideTemperatureCompensation << 3) | (enableCentralHeating2 << 4);
    data <<= 8;
    return buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Status, data);
}

unsigned long OpenTherm::buildSetBoilerTemperatureRequest(float temperature)
{
    unsigned int data = temperatureToData(temperature);
    return buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TSet, data);
}

unsigned long OpenTherm::buildGetBoilerTemperatureRequest()
{
    return buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tboiler, 0);
}

// parsing responses
bool OpenTherm::isFault(unsigned long response)
{
    return response & 0x1;
}

bool OpenTherm::isCentralHeatingActive(unsigned long response)
{
    return response & 0x2;
}

bool OpenTherm::isHotWaterActive(unsigned long response)
{
    return response & 0x4;
}

bool OpenTherm::isFlameOn(unsigned long response)
{
    return response & 0x8;
}

bool OpenTherm::isCoolingActive(unsigned long response)
{
    return response & 0x10;
}

bool OpenTherm::isDiagnostic(unsigned long response)
{
    return response & 0x40;
}

uint16_t OpenTherm::getUInt(const unsigned long response)
{
    const uint16_t u88 = response & 0xffff;
    return u88;
}

float OpenTherm::getFloat(const unsigned long response)
{
    const uint16_t u88 = getUInt(response);
    const float f = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
    return f;
}

unsigned int OpenTherm::temperatureToData(float temperature)
{
    if (temperature < 0)
        temperature = 0;
    if (temperature > 100)
        temperature = 100;
    unsigned int data = (unsigned int)(temperature * 256);
    return data;
}

// basic requests

unsigned long OpenTherm::setBoilerStatus(bool enableCentralHeating, bool enableHotWater, bool enableCooling, bool enableOutsideTemperatureCompensation, bool enableCentralHeating2)
{
    return sendRequest(buildSetBoilerStatusRequest(enableCentralHeating, enableHotWater, enableCooling, enableOutsideTemperatureCompensation, enableCentralHeating2));
}

bool OpenTherm::setBoilerTemperature(float temperature)
{
    unsigned long response = sendRequest(buildSetBoilerTemperatureRequest(temperature));
    return isValidResponse(response);
}

float OpenTherm::getBoilerTemperature()
{
    unsigned long response = sendRequest(buildGetBoilerTemperatureRequest());
    return isValidResponse(response) ? getFloat(response) : 0;
}

float OpenTherm::getReturnTemperature()
{
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tret, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

bool OpenTherm::setDHWSetpoint(float temperature)
{
    unsigned int data = temperatureToData(temperature);
    unsigned long response = sendRequest(buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TdhwSet, data));
    return isValidResponse(response);
}

float OpenTherm::getDHWTemperature()
{
    unsigned long response = sendRequest(buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tdhw, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

float OpenTherm::getModulation()
{
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::RelModLevel, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

float OpenTherm::getPressure()
{
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::CHPressure, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

unsigned char OpenTherm::getFault()
{
    return ((sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::ASFflags, 0)) >> 8) & 0xff);
}

#ifdef OPENTHERM_USE_RMT
void OpenTherm::setupRMT() {

    rmt_config_t rmt_tx = {};
    rmt_tx.rmt_mode = RMT_MODE_TX;
    rmt_tx.channel = RMT_TX_CHANNEL;
    rmt_tx.gpio_num = (gpio_num_t)outPin;
    rmt_tx.clk_div = 80;  // 80 MHz / 80 = 1 MHz → 1 us per tick
    rmt_tx.mem_block_num = 1;  // 1 block is usually enough for OpenTherm signals

    rmt_tx.tx_config.carrier_en = false;              // No carrier
    rmt_tx.tx_config.loop_en = false;                 // No loop transmission
    rmt_tx.tx_config.carrier_freq_hz = 0;             // No carrier frequency
    rmt_tx.tx_config.carrier_duty_percent = 0;        // No carrier duty cycle
    rmt_tx.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;               // Not used (no carrier)
#ifdef OT_LOCAL
    rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;  // Set GPIO low after transmission (matches eot_level = 0 in your IDF example)
#else   
    rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;  // Set GPIO high after transmission (matches eot_level = 1 in your IDF example)
#endif  
    rmt_tx.tx_config.idle_output_en = true;           // Keep idle level after transmission

    rmt_config(&rmt_tx);
    rmt_driver_install(rmt_tx.channel, 0, 0);

    rmt_config_t rmt_rx = {};
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = (gpio_num_t)inPin;
    rmt_rx.clk_div = 80;  // 80 MHz / 80 = 1 MHz → 1 us per tick
    rmt_rx.mem_block_num = 1;

    rmt_rx.rx_config.filter_en = true;
    // Minimum valid signal duration: 200 us → 200 ticks
    rmt_rx.rx_config.filter_ticks_thresh = 200;

    // Maximum signal duration: 2 ms → 2000 ticks
    rmt_rx.rx_config.idle_threshold = 2000;

    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
}

void OpenTherm::sendRMT(uint32_t data) {
    constexpr int BITS = 34;  // 1 start bit + 32 data bits + 1 stop bit
    rmt_item32_t items[BITS];

    for (int i = 0; i < BITS; ++i) {
        bool bit;
        if (i == 0 || i >= BITS - 1) {
            bit = true;  // Start and Stop bits are always '1'
        } else {
            bit = (data >> (BITS - 2 - i)) & 1;
        }

        if (bit) {
            // Manchester encoding: '1' is high then low
            items[i].level0 = 1;
            items[i].duration0 = 500;  // 500 us high
            items[i].level1 = 0;
            items[i].duration1 = 500;  // 500 us low
        } else {
            // Manchester encoding: '0' is low then high
            items[i].level0 = 0;
            items[i].duration0 = 500;  // 500 us low
            items[i].level1 = 1;
            items[i].duration1 = 500;  // 500 us high
        }
    }

    // Transmit the Manchester encoded message
    rmt_write_items(RMT_TX_CHANNEL, items, BITS, true);  // true = wait for transmission to finish
    rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
}

bool OpenTherm::addManchesterHalfBit(bool signal){
    if (insideManchesterBit) {
        if (signal==lastManchesterSignal){
            status = OpenThermStatus::RESPONSE_INVALID;
            responseTimestamp = micros();
            return false;
        } else {
            response = (response << 1) | lastManchesterSignal;
            responseTimestamp = micros();
            responseBitIndex = responseBitIndex + 1;
            insideManchesterBit = false;
        }
    }
    else 
    {
        lastManchesterSignal = signal;
        insideManchesterBit = true;
    }

    if (responseBitIndex==33) {
        status = OpenThermStatus::RESPONSE_READY;
        responseTimestamp = micros();
        return false;
    }

    return true;
}

bool OpenTherm::addManchesterSignal(bool signal, uint32_t duration)
{
    constexpr int TOLERANCE = 100; // +/- tolerance for pulse width
    constexpr int HALF_BIT_US = 500;

    if (duration>HALF_BIT_US-TOLERANCE && duration<HALF_BIT_US+TOLERANCE){
        return addManchesterHalfBit(signal);
    }
    else if (duration>2*HALF_BIT_US-TOLERANCE && duration<2*HALF_BIT_US+TOLERANCE){
        if (!addManchesterHalfBit(signal)) return false;
        return addManchesterHalfBit(signal);
    } 
    else {
        status = OpenThermStatus::RESPONSE_INVALID;
        responseTimestamp = micros();
        return false;
    }
}


uint32_t OpenTherm::receiveRMT() {
    constexpr int MAX_BITS = 64;    
    rmt_item32_t* items = nullptr;
    size_t length = 0;
    RingbufHandle_t rb = nullptr;

    rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
    if (!rb) return 0;
    rmt_rx_start(RMT_RX_CHANNEL, true);

    items = (rmt_item32_t*)xRingbufferReceive(rb, &length, pdMS_TO_TICKS(100));
    if (!items) return 0;

    vRingbufferReturnItem(rb, items);

    if (status == OpenThermStatus::RESPONSE_READY) {
        if (isValidResponse(response)) {
            status = OpenThermStatus::READY;
            return response;
        } else {
            status = OpenThermStatus::RESPONSE_INVALID;
            responseTimestamp = micros();
        }
    } else if (status == OpenThermStatus::RESPONSE_INVALID) {
        responseTimestamp = micros();
    }

    return 0;
}

#endif