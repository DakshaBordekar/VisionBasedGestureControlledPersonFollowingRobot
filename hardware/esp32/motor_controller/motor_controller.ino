/**
 * ============================================================================
 * Project: Vision-Based Gesture-Controlled Person-Following Robot
 * Target: ESP32-WROOM-32 / DevKit v1
 * File: hardware/esp32/motor_controller/motor_controller.ino
 * Description: Production-ready firmware for differential drive motor actuation.
 *              Receives ASCII command packets over UART: "<linear> <angular>\n"
 *              Computes inverse kinematics, sets directional GPIOs, and applies
 *              hardware PWM with watchdog failsafe and deadband compensation.
 * ============================================================================
 */

#include <Arduino.h>

// ============================================================================
// HARDWARE PIN DEFINITIONS (L298N Dual H-Bridge)
// ============================================================================
// Left Motor (Motor A)
static const int PIN_ENA = 25;  // Speed control (PWM)
static const int PIN_IN1 = 26;  // Direction Forward
static const int PIN_IN2 = 27;  // Direction Reverse

// Right Motor (Motor B)
static const int PIN_ENB = 17;  // Speed control (PWM)
static const int PIN_IN3 = 14;  // Direction Forward
static const int PIN_IN4 = 16;  // Direction Reverse

// Onboard diagnostic LED
static const int PIN_STATUS_LED = 2;

// ============================================================================
// PWM CONFIGURATION (ESP32 LEDC Peripheral)
// ============================================================================
static const int PWM_FREQ         = 20000; // 20 kHz ultrasonic frequency (no motor whine)
static const int PWM_RESOLUTION   = 8;     // 8-bit resolution: 0 - 255
static const int PWM_CHAN_LEFT    = 0;     // LEDC Channel 0 for Left Motor
static const int PWM_CHAN_RIGHT   = 1;     // LEDC Channel 1 for Right Motor

// ============================================================================
// ROBOT PHYSICAL PARAMETERS & LIMITS
// ============================================================================
static const float WHEEL_SEPARATION = 0.320f; // Track width in meters (32 cm)
static const float MAX_LINEAR_SPEED = 0.250f; // Maximum linear speed in m/s
static const float MIN_PWM_DEADZONE = 65.0f;  // Minimum PWM to overcome gearbox friction
static const float MAX_PWM_VALUE    = 255.0f;

// Watchdog safety timeout (milliseconds)
static const unsigned long COMMAND_TIMEOUT_MS = 500;

// ============================================================================
// INTERNAL STATE
// ============================================================================
static unsigned long last_command_received_time = 0;
static bool failsafe_active = true;

// Buffer for serial stream parsing
static const size_t SERIAL_BUF_SIZE = 64;
static char serial_buffer[SERIAL_BUF_SIZE];
static size_t serial_buf_index = 0;

// ============================================================================
// MOTOR ACTUATION PRIMITIVES
// ============================================================================
void stop_motors() {
    ledcWrite(PWM_CHAN_LEFT, 0);
    ledcWrite(PWM_CHAN_RIGHT, 0);
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    digitalWrite(PIN_STATUS_LED, LOW);
}

void set_left_motor(float speed) {
    if (fabs(speed) < 0.005f) {
        ledcWrite(PWM_CHAN_LEFT, 0);
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        return;
    }

    bool forward = (speed > 0.0f);
    digitalWrite(PIN_IN1, forward ? HIGH : LOW);
    digitalWrite(PIN_IN2, forward ? LOW : HIGH);

    float norm_speed = constrain(fabs(speed) / MAX_LINEAR_SPEED, 0.0f, 1.0f);
    int pwm = (int)(MIN_PWM_DEADZONE + norm_speed * (MAX_PWM_VALUE - MIN_PWM_DEADZONE));
    pwm = constrain(pwm, 0, 255);

    ledcWrite(PWM_CHAN_LEFT, pwm);
}

void set_right_motor(float speed) {
    if (fabs(speed) < 0.005f) {
        ledcWrite(PWM_CHAN_RIGHT, 0);
        digitalWrite(PIN_IN3, LOW);
        digitalWrite(PIN_IN4, LOW);
        return;
    }

    bool forward = (speed > 0.0f);
    digitalWrite(PIN_IN3, forward ? HIGH : LOW);
    digitalWrite(PIN_IN4, forward ? LOW : HIGH);

    float norm_speed = constrain(fabs(speed) / MAX_LINEAR_SPEED, 0.0f, 1.0f);
    int pwm = (int)(MIN_PWM_DEADZONE + norm_speed * (MAX_PWM_VALUE - MIN_PWM_DEADZONE));
    pwm = constrain(pwm, 0, 255);

    ledcWrite(PWM_CHAN_RIGHT, pwm);
}

void execute_differential_drive(float linear_x, float angular_z) {
    // Inverse kinematics: v_left = v - (w * L / 2), v_right = v + (w * L / 2)
    float half_wheel_track = (WHEEL_SEPARATION / 2.0f);
    float v_left  = linear_x - (angular_z * half_wheel_track);
    float v_right = linear_x + (angular_z * half_wheel_track);

    set_left_motor(v_left);
    set_right_motor(v_right);

    digitalWrite(PIN_STATUS_LED, HIGH);
}

// ============================================================================
// SERIAL PROTOCOL PARSER
// ============================================================================
void process_packet(const char* packet) {
    float linear_cmd = 0.0f;
    float angular_cmd = 0.0f;

    // Format expected: "<linear> <angular>" e.g. "0.200 -0.450"
    int parsed = sscanf(packet, "%f %f", &linear_cmd, &angular_cmd);

    if (parsed == 2) {
        last_command_received_time = millis();
        failsafe_active = false;
        execute_differential_drive(linear_cmd, angular_cmd);
    } else {
        Serial.printf("[ERR] Corrupt packet: %s\n", packet);
    }
}

void poll_serial_interface() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            if (serial_buf_index > 0) {
                serial_buffer[serial_buf_index] = '\0';
                process_packet(serial_buffer);
                serial_buf_index = 0;
            }
        } else {
            if (serial_buf_index < (SERIAL_BUF_SIZE - 1)) {
                serial_buffer[serial_buf_index++] = c;
            } else {
                // Buffer overflow protection: reset index
                serial_buf_index = 0;
            }
        }
    }
}

// ============================================================================
// ARDUINO LIFECYCLE
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.println("==================================================");
    Serial.println("  VBGC Rover ESP32 Motor Controller Initializing  ");
    Serial.println("==================================================");

    // Initialize Directional Output Pins
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);
    pinMode(PIN_STATUS_LED, OUTPUT);

    // Configure ESP32 LEDC PWM Peripheral
    ledcSetup(PWM_CHAN_LEFT, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHAN_RIGHT, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(PIN_ENA, PWM_CHAN_LEFT);
    ledcAttachPin(PIN_ENB, PWM_CHAN_RIGHT);

    stop_motors();

    last_command_received_time = millis();
    Serial.println("[OK] Motor Controller Ready. Awaiting commands...");
}

void loop() {
    poll_serial_interface();

    // Watchdog check: Halt motors if command stream times out
    if (!failsafe_active && (millis() - last_command_received_time > COMMAND_TIMEOUT_MS)) {
        stop_motors();
        failsafe_active = true;
        Serial.println("[WARN] Watchdog fired: Velocity command timeout. Motors halted.");
    }
}