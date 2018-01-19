// Copyright 2017 Jordan Ford

#include "Arduino.h"
#include "vehicle.h"
#include "lookup.h"

volatile long long odom = 0;
volatile uint16_t throttle_val = 0;
volatile uint16_t steering_val = 0;

void estop_isr(void) {
    analogWrite(THROTTLE_PIN, THROTTLE_STOP);
    analogWrite(STEERING_PIN, STEERING_CENTER);
    error_blink(ESTOP_ERR_PERIOD);
}

void error_blink(int period_ms) {
    while(1) {
        digitalWriteFast(LED_PIN, LOW);
        delay(period_ms>>1);
        digitalWriteFast(LED_PIN, HIGH);
        delay(period_ms>>1);
    }
}

void odom_isr() {
  static int prev_state = -1;

  int motor_seq_num = (digitalRead(MOTOR_PHASE0_PIN)<<2 | 
                       digitalRead(MOTOR_PHASE1_PIN)<<1 | 
                       digitalRead(MOTOR_PHASE2_PIN)) - 1;

  int state = hall_effect_sequence[motor_seq_num];
  if (prev_state < 0) {
    prev_state = state;
    return;
  }
  
  if (state-1 == prev_state || (state == 0 && prev_state == 5)) { odom++; }
  else if (state+1 == prev_state || (state == 5 && prev_state == 0)) { odom--; }
  else { 
      Serial.println(prev_state);
      Serial.println(state);
      estop_isr();
  }
  
  prev_state = state;
}

Vehicle::Vehicle(void) : servo_curr_val_(STEERING_CENTER),
                         servo_target_val_(STEERING_CENTER),
                         servo_last_chg_(0),
                         throttle_curr_val_(THROTTLE_STOP),
                         throttle_target_val_(THROTTLE_STOP),
                         throttle_last_chg_(0),
                         last_pub_(0) 
{
    
    // Set up PWM
    analogWriteResolution(PWM_RESOLUTION_BITS);
    analogWriteFrequency(THROTTLE_PIN, PWM_FREQUENCY_HZ);
    analogWriteFrequency(STEERING_PIN, PWM_FREQUENCY_HZ);
    
    // Set steering to the center and throttle off
    analogWrite(STEERING_PIN, STEERING_CENTER);
    analogWrite(THROTTLE_PIN, THROTTLE_STOP);
    
    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Attach ESTOP
    pinMode(ESTOP_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estop_isr, CHANGE);
    
    // Attach motor hall effect sensor interrupts
    pinMode(MOTOR_PHASE0_PIN, INPUT);
    pinMode(MOTOR_PHASE1_PIN, INPUT);
    pinMode(MOTOR_PHASE2_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MOTOR_PHASE0_PIN), odom_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_PHASE1_PIN), odom_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_PHASE2_PIN), odom_isr, CHANGE);
}

Vehicle::~Vehicle(void) {}

void Vehicle::set_steering_angle(float angle_rad) {
    // Cap the steering angle at the vehicle limits.
    angle_rad = (angle_rad >  MAX_STEERING_ANGLE_RAD) ?   MAX_STEERING_ANGLE_RAD : angle_rad;
    angle_rad = (angle_rad < -MAX_STEERING_ANGLE_RAD) ?  -MAX_STEERING_ANGLE_RAD : angle_rad;
    int steering_val = map( angle_rad, -MAX_STEERING_ANGLE_RAD, MAX_STEERING_ANGLE_RAD, STEERING_RIGHT_MAX, STEERING_LEFT_MAX );
    analogWrite(STEERING_PIN, steering_val);
}

void Vehicle::set_throttle_speed(float speed_mps) {
    float speed_tps = speed_mps * 1000.0 / (2*M_PI*VEHICLE_WHEEL_RAD_MM) * VEHICLE_GEAR_RATIO * 6;
    speed_tps = (speed_tps > MAX_THROTTLE_SPEED_TPS) ? MAX_THROTTLE_SPEED_TPS : speed_tps;
    speed_tps = (speed_tps < MIN_THROTTLE_SPEED_TPS) ? MIN_THROTTLE_SPEED_TPS : speed_tps;
    int speed_tps_rounded = (int)(speed_tps)/20 * 20 + ((int)(speed_tps)%20 >= 10 ? 20 : 0);
    int lookup_idx = (speed_tps_rounded + 1000)/20;
    throttle_val = lookup[lookup_idx];
    analogWrite(THROTTLE_PIN, throttle_val);
}

float Vehicle::calc_speed_ticks_per_sec(void) {
    static int32_t last_odom = odom;
    static uint32_t last_time = micros();
    uint32_t time = micros();
    if (time - last_time == 0) { return 0.0; }
    double speed_tps = (odom - last_odom) * 1e6 / (float)(time - last_time);
    last_time = time;
    last_odom = odom;
    return speed_tps;
}