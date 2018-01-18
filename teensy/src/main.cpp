#include "vehicle.h"


uint32_t readline(char* line, uint32_t len) {
  int i = 0;
  while( Serial.available() ) {
    line[i] = Serial.read();
    if( line[i] == '\n' ) { 
        line[i] = '\0';
        return i;
    }

    ++i;
    if (i >= len) { return 0; }
  }
  return 0;
}

int main(void)
{

    // Initialize USB serial.
    Serial.begin(115200);

    // Initialize the Vehicle.
    Vehicle vehicle = Vehicle();

    char cmd_c;
    int cmd_val;
    char rcv[32];
    
    while (1) {
        
        cmd_c = 0; cmd_val = 0;

        int chars = readline(rcv, sizeof(rcv));
        int num_vals = sscanf(rcv, "%c%d", &cmd_c, &cmd_val);

        if (chars != 0 && num_vals == 2) {
            if (cmd_c=='t') {
                vehicle.set_throttle_speed(cmd_val);
                Serial.print("Throttle speed set to ");
                Serial.println(cmd_val);
            } else if (cmd_c=='s') {
                vehicle.set_steering_angle(cmd_val);
                Serial.print("Steering angle set to ");
                Serial.println(cmd_val);
            }
        }

        vehicle.routine();
    }

    // data logging from top forward speed
    /* for(int speed=THROTTLE_STOP; speed >= THROTTLE_FWD_MAX; speed -= 100) {
        analogWrite(THROTTLE_PIN, speed);
        delay(500);
        Serial.println(speed);
    }
    Serial.println("STARTING TEST AT TOP SPEED");
    for(int speed=THROTTLE_FWD_MAX; speed <= THROTTLE_STOP; speed += 100) {
        Serial.print(speed);
        Serial.print(",");
        Serial.println(vehicle.calc_speed_ticks_per_sec());
        analogWrite(THROTTLE_PIN, speed);        
        delay(500);
    }

    delay(2000);

    for(int speed=THROTTLE_STOP; speed <= THROTTLE_REV_MAX; speed += 500) {
        Serial.print(speed);
        Serial.print(",");
        Serial.println(vehicle.calc_speed_ticks_per_sec());
        analogWrite(THROTTLE_PIN, speed);        
        delay(500);
    } */

    // data logging from top reverse speed
    /* for(int speed=THROTTLE_STOP; speed <= THROTTLE_REV_MAX; speed += 100) {
        analogWrite(THROTTLE_PIN, speed);
        Serial.println(speed);
        delay(500);
    }
    Serial.println("STARTING TEST AT TOP NEGATIVE SPEED");
    for(int speed=THROTTLE_REV_MAX; speed >= THROTTLE_STOP; speed -= 100) {
        Serial.print(speed);
        Serial.print(",");
        Serial.println(vehicle.calc_speed_ticks_per_sec());
        analogWrite(THROTTLE_PIN, speed);
        delay(500);
    }

    delay(2000);

    for(int speed=THROTTLE_STOP; speed >= THROTTLE_REV_MAX; speed -= 100) {
        Serial.print(speed);
        Serial.print(",");
        Serial.println(vehicle.calc_speed_ticks_per_sec());
        analogWrite(THROTTLE_PIN, speed);        
        delay(500);
    } */


    return 0;
}






