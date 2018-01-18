#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <Arduino.h>

// Control board pinout
#define STEERING_PIN   22
#define THROTTLE_PIN   23
#define LED_PIN        13
#define ESTOP_PIN       2

// Motor hall effect pins
#define MOTOR_PHASE0_PIN 3
#define MOTOR_PHASE1_PIN 4
#define MOTOR_PHASE2_PIN 5

// Vehicle Mechanical Constants 
#define VEHICLE_GEAR_RATIO       5.62
#define VEHICLE_WHEEL_RAD_MM   32.385

// PWM
#define PWM_RESOLUTION_BITS 16
#define PWM_FREQUENCY_HZ    76

// Throttle Range
#define THROTTLE_FWD_MAX  ((1<<PWM_RESOLUTION_BITS)*1.00*PWM_FREQUENCY_HZ/1000.0)
#define THROTTLE_STOP ((1<<PWM_RESOLUTION_BITS)*1.50*PWM_FREQUENCY_HZ/1000.0)
#define THROTTLE_REV_MAX  ((1<<PWM_RESOLUTION_BITS)*2.00*PWM_FREQUENCY_HZ/1000.0)

// Steering Range
#define STEERING_LEFT_MAX   ((1<<PWM_RESOLUTION_BITS)*1.75*PWM_FREQUENCY_HZ/1000.0)
#define STEERING_CENTER     ((1<<PWM_RESOLUTION_BITS)*1.5*PWM_FREQUENCY_HZ/1000.0)
#define STEERING_RIGHT_MAX  ((1<<PWM_RESOLUTION_BITS)*1.25*PWM_FREQUENCY_HZ/1000.0)

// Error blink periods
#define ESTOP_ERR_PERIOD                1000
#define INIT_ACCEL_ERR_PERIOD           2000
#define VEHICLE_SINGLETON_ERR_PERIOD    3000

// Delay between changing PWM output (micros)
#define SERVO_DELAY_US             0
#define THROTTLE_SOFT_DELAY_US     0
#define THROTTLE_HARD_DELAY_US     0
#define PUBLISH_DELAY_US         100

// Vehicle max steering angle at the wheels.
#define MAX_STEERING_ANGLE_RAD  (M_PI*0.25)

constexpr uint8_t hall_effect_sequence[6] = {0, 2, 1, 4, 5, 3};
extern volatile long long odom;

void estop_isr(void);
void odom_isr(void);
void error_blink(int period_ms);
        
class Vehicle {
    public:
        Vehicle();
        ~Vehicle();

        void set_steering_angle(int angle_rad);
        void set_throttle_speed(int speed_mps);
        float calc_speed_ticks_per_sec(void);
        void routine(void);
        int32_t get_odom(void) {
          return odom;
        }

    private:
        uint16_t servo_curr_val_;
        uint16_t servo_target_val_;
        uint64_t servo_last_chg_;
        uint16_t throttle_curr_val_;
        uint16_t throttle_target_val_;
        uint64_t throttle_last_chg_;
        uint64_t last_pub_;

        void publish_data(void);
};

#endif