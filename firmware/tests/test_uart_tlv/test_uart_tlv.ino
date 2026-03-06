/**
 * @file test_uart_tlv.ino
 * @brief UART link-quality test — Arduino side
 *
 * Measures communication quality in both directions:
 *   Arduino → RPi : decode errors reported by Python (CRC/framing failures)
 *   RPi → Arduino : uartRxErrors in SYS_STATUS, visible on Python side
 *   Liveness      : time since last valid TLV heartbeat from RPi
 *
 * How to use:
 *   1. Set RPI_BAUD_RATE in config.h to the rate under test
 *   2. Upload this sketch
 *   3. Run:  python3 test_uart_arduino.py --baud <same rate>
 *   4. Sweep: 115200 / 230400 / 500000 / 1000000 / 2000000
 *      Highest rate with 0 errors on both sides is the safe limit.
 *
 * Hardware: Serial2 pin 16 (TX) / pin 17 (RX) → level shifter → RPi ttyAMA0
 *           Serial0 (USB) for this debug output
 */

#include "src/config.h"
#include "src/Scheduler.h"
#include "src/modules/MessageCenter.h"
#include "src/drivers/DCMotor.h"
#include "src/drivers/StepperMotor.h"

// Required by MessageCenter (extern references)
DCMotor      dcMotors[NUM_DC_MOTORS];
StepperMotor steppers[NUM_STEPPERS];

static uint32_t lostTotal  = 0;
static uint32_t goodTicks  = 0;
static bool     prevValid  = false;

// 100 Hz — drain RX FIFO and send telemetry
void taskUART() {
    MessageCenter::processIncoming();
    MessageCenter::sendTelemetry();
}

// 1 Hz — print one-line link quality status
void taskStatus() {
    bool valid = MessageCenter::isHeartbeatValid();

    if (!valid) {
        lostTotal++;
        goodTicks = 0;
        if (prevValid) DEBUG_SERIAL.println(F("[LINK] *** Liveness LOST ***"));
    } else {
        goodTicks++;
    }
    prevValid = valid;

    if (valid) {
        DEBUG_SERIAL.print(F("[LINK] OK    lastRx="));
        DEBUG_SERIAL.print(MessageCenter::getTimeSinceHeartbeat());
        DEBUG_SERIAL.print(F("ms  goodTicks="));
        DEBUG_SERIAL.print(goodTicks);
    } else {
        DEBUG_SERIAL.print(F("[LINK] LOST  totalLost="));
        DEBUG_SERIAL.print(lostTotal);
    }
    DEBUG_SERIAL.println();
    // RPi→Arduino RX errors (uartRxErrors) are reported in every SYS_STATUS
    // frame — read them on the Python side.
}

void setup() {
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL && millis() < 2000);

    DEBUG_SERIAL.println(F("================================"));
    DEBUG_SERIAL.println(F("  UART Link-Quality Test"));
    DEBUG_SERIAL.println(F("================================"));
    DEBUG_SERIAL.print(F("  Serial2 @ "));
    DEBUG_SERIAL.print(RPI_BAUD_RATE);
    DEBUG_SERIAL.println(F(" baud"));
    DEBUG_SERIAL.println(F("  uartRxErrors reported in SYS_STATUS (see Python)"));
    DEBUG_SERIAL.println(F("================================"));

    Scheduler::init();
    MessageCenter::init();
    Scheduler::registerTask(taskUART,   10,   1);   // 100 Hz
    Scheduler::registerTask(taskStatus, 1000, 2);   // 1 Hz
}

void loop() { Scheduler::tick(); }
