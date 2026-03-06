/**
 * @file test_encoder.ino
 * @brief Test sketch for encoder counting (Phase 3)
 *
 * This test verifies encoder counting functionality by:
 * 1. Initializing encoder counters for DC motors
 * 2. Attaching interrupt handlers
 * 3. Continuously printing encoder counts
 * 4. Allowing manual motor rotation to verify direction and accuracy
 *
 * Expected Behavior:
 * - Rotating motor forward increases count
 * - Rotating motor backward decreases count
 * - Count accuracy matches expected resolution (2x or 4x)
 * - Timestamp updates on each edge
 *
 * How to Test:
 * 1. Upload this sketch
 * 2. Open Serial Monitor @ 115200 baud
 * 3. Manually rotate motor shafts (or connect motors and send commands)
 * 4. Verify counts increment/decrement correctly
 * 5. Verify direction is correct for your wiring
 *
 * Hardware Setup:
 * - Connect encoder phase A to hardware interrupt pins (see pins.h)
 * - Connect encoder phase B to digital input pins
 * - No need to connect motor power for this test
 *
 * Verification:
 * 1. Compile success
 * 2. Encoder counts update when motor rotates
 * 3. Forward rotation increases count
 * 4. Backward rotation decreases count
 * 5. Count resolution matches configuration (2x or 4x mode)
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/modules/EncoderCounter.h"

// ============================================================================
// ENCODER INSTANCES
// ============================================================================

// Create encoder instances based on configuration
#if ENCODER_1_MODE == ENCODER_2X
EncoderCounter2x encoder1;
#else
EncoderCounter4x encoder1;
#endif

#if ENCODER_2_MODE == ENCODER_2X
EncoderCounter2x encoder2;
#else
EncoderCounter4x encoder2;
#endif

#if ENCODER_3_MODE == ENCODER_2X
EncoderCounter2x encoder3;
#else
EncoderCounter4x encoder3;
#endif

#if ENCODER_4_MODE == ENCODER_2X
EncoderCounter2x encoder4;
#else
EncoderCounter4x encoder4;
#endif

// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================

// Motor 1 encoder ISR
void encoderISR_M1() {
    encoder1.onInterruptA();
}

// Motor 2 encoder ISR
void encoderISR_M2() {
    encoder2.onInterruptA();
}

// Motor 1 phase B ISR (Rev. B: pin 3 = INT1, 4x mode only)
#if ENCODER_1_MODE == ENCODER_4X
void encoderISR_M1_B() {
    encoder1.onInterruptB();
}
#endif

// Motor 2 phase B ISR (Rev. B: pin 19 = INT4, 4x mode only)
#if ENCODER_2_MODE == ENCODER_4X
void encoderISR_M2_B() {
    encoder2.onInterruptB();
}
#endif

// Motor 3: PCINT2_vect — Port K
//   M3_ENC_A = A14 = PINK bit 6 (PCINT22)
//   M3_ENC_B = A15 = PINK bit 7 (PCINT23)
// attachInterrupt() does NOT work for PCINT pins on the Mega — must use raw ISR.
volatile uint8_t prevPINK_enc = 0;
ISR(PCINT2_vect) {
    uint8_t curPINK = PINK;
    uint8_t changed  = curPINK ^ prevPINK_enc;
    prevPINK_enc = curPINK;
    if (changed & (1 << 6)) encoder3.onInterruptA();  // A14 = M3_ENC_A
    if (changed & (1 << 7)) encoder3.onInterruptB();  // A15 = M3_ENC_B
}

// Motor 4: PCINT0_vect — Port B
//   M4_ENC_A = pin 11 = PINB bit 5 (PCINT5)
//   M4_ENC_B = pin 12 = PINB bit 6 (PCINT6)
volatile uint8_t prevPINB_enc = 0;
ISR(PCINT0_vect) {
    uint8_t curPINB = PINB;
    uint8_t changed  = curPINB ^ prevPINB_enc;
    prevPINB_enc = curPINB;
    if (changed & (1 << 5)) encoder4.onInterruptA();  // pin 11 = M4_ENC_A
    if (changed & (1 << 6)) encoder4.onInterruptB();  // pin 12 = M4_ENC_B
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize Debug Serial
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL && millis() < 2000); // Wait for connection

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("  Encoder Counter Test - Phase 3"));
    DEBUG_SERIAL.println(F("========================================"));

    // Initialize encoders (with direction flags from config.h)
    encoder1.init(PIN_M1_ENC_A, PIN_M1_ENC_B, ENCODER_1_DIR_INVERTED);
    encoder2.init(PIN_M2_ENC_A, PIN_M2_ENC_B, ENCODER_2_DIR_INVERTED);

    encoder3.init(PIN_M3_ENC_A, PIN_M3_ENC_B, ENCODER_3_DIR_INVERTED);
    encoder4.init(PIN_M4_ENC_A, PIN_M4_ENC_B, ENCODER_4_DIR_INVERTED);

    DEBUG_SERIAL.println(F("[Setup] Encoders initialized"));

    // M1: phase A (pin 2 = INT0), phase B (pin 3 = INT1, 4x only)
    attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1, CHANGE);
#if ENCODER_1_MODE == ENCODER_4X
    attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_B), encoderISR_M1_B, CHANGE);
#endif

    // M2: phase A (pin 18 = INT5), phase B (pin 19 = INT4, 4x only)
    attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), encoderISR_M2, CHANGE);
#if ENCODER_2_MODE == ENCODER_4X
    attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_B), encoderISR_M2_B, CHANGE);
#endif

    // M3: PCINT2 bank — A14 (PCINT22 = PINK bit 6), A15 (PCINT23 = PINK bit 7)
    // attachInterrupt() does not support PCINT pins; ISR(PCINT2_vect) handles this above.
    prevPINK_enc = PINK;                          // snapshot pin state before enabling
    PCICR  |= (1 << PCIE2);                       // enable Port K PCINT bank
    PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);   // unmask A14 and A15

    // M4: PCINT0 bank — pin 11 (PCINT5 = PINB bit 5), pin 12 (PCINT6 = PINB bit 6)
    prevPINB_enc = PINB;                          // snapshot pin state before enabling
    PCICR  |= (1 << PCIE0);                       // enable Port B PCINT bank
    PCMSK0 |= (1 << PCINT5) | (1 << PCINT6);     // unmask pins 11 and 12

    DEBUG_SERIAL.println(F("[Setup] Encoder interrupts attached"));

    // Print configuration
    DEBUG_SERIAL.print(F("  - Motor 1: "));
    DEBUG_SERIAL.print(ENCODER_1_MODE);
    DEBUG_SERIAL.print(F("x mode ("));
    DEBUG_SERIAL.print(encoder1.getResolutionMultiplier() * ENCODER_PPR);
    DEBUG_SERIAL.println(F(" counts/rev)"));

    DEBUG_SERIAL.print(F("  - Motor 2: "));
    DEBUG_SERIAL.print(ENCODER_2_MODE);
    DEBUG_SERIAL.print(F("x mode ("));
    DEBUG_SERIAL.print(encoder2.getResolutionMultiplier() * ENCODER_PPR);
    DEBUG_SERIAL.println(F(" counts/rev)"));

    DEBUG_SERIAL.print(F("  - Motor 3: "));
    DEBUG_SERIAL.print(ENCODER_3_MODE);
    DEBUG_SERIAL.print(F("x mode ("));
    DEBUG_SERIAL.print(encoder3.getResolutionMultiplier() * ENCODER_PPR);
    DEBUG_SERIAL.println(F(" counts/rev)"));

    DEBUG_SERIAL.print(F("  - Motor 4: "));
    DEBUG_SERIAL.print(ENCODER_4_MODE);
    DEBUG_SERIAL.print(F("x mode ("));
    DEBUG_SERIAL.print(encoder4.getResolutionMultiplier() * ENCODER_PPR);
    DEBUG_SERIAL.println(F(" counts/rev)"));

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("Test Instructions:"));
    DEBUG_SERIAL.println(F("  1. Manually rotate motor shafts"));
    DEBUG_SERIAL.println(F("  2. Verify count increases for forward rotation"));
    DEBUG_SERIAL.println(F("  3. Verify count decreases for backward rotation"));
    DEBUG_SERIAL.println(F("  4. Check timestamps update on each edge"));
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("Reading encoder counts..."));
    DEBUG_SERIAL.println(F("========================================"));
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    // Read encoder counts
    int32_t count1 = encoder1.getCount();
    int32_t count2 = encoder2.getCount();
    uint32_t time1 = encoder1.getLastEdgeUs();
    uint32_t time2 = encoder2.getLastEdgeUs();

    // Print encoder data
    DEBUG_SERIAL.print(F("M1: "));
    DEBUG_SERIAL.print(count1);
    DEBUG_SERIAL.print(F(" ["));
    DEBUG_SERIAL.print(time1);
    DEBUG_SERIAL.print(F("µs]  |  M2: "));
    DEBUG_SERIAL.print(count2);
    DEBUG_SERIAL.print(F(" ["));
    DEBUG_SERIAL.print(time2);
    DEBUG_SERIAL.print(F("µs]"));

    int32_t count3 = encoder3.getCount();
    uint32_t time3 = encoder3.getLastEdgeUs();
    DEBUG_SERIAL.print(F("  |  M3: "));
    DEBUG_SERIAL.print(count3);
    DEBUG_SERIAL.print(F(" ["));
    DEBUG_SERIAL.print(time3);
    DEBUG_SERIAL.print(F("µs]"));

    int32_t count4 = encoder4.getCount();
    uint32_t time4 = encoder4.getLastEdgeUs();
    DEBUG_SERIAL.print(F("  |  M4: "));
    DEBUG_SERIAL.print(count4);
    DEBUG_SERIAL.print(F(" ["));
    DEBUG_SERIAL.print(time4);
    DEBUG_SERIAL.print(F("µs]"));

    DEBUG_SERIAL.println();

    delay(200);  // Print at 5Hz
}
