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
    float cmd_val;
    char rcv[32];
    
    while (1) {
        
        cmd_c = 0; cmd_val = 0;

        int chars = readline(rcv, sizeof(rcv));
        int num_vals = sscanf(rcv, "%c%f", &cmd_c, &cmd_val);

        if (chars != 0) {
            if (cmd_c=='t' && num_vals == 2) {
                vehicle.set_throttle_speed(cmd_val);
                Serial.print("Throttle speed set to ");
                Serial.println(cmd_val);
            } else if (cmd_c=='s' && num_vals == 2) {
                vehicle.set_steering_angle(cmd_val);
                Serial.print("Steering angle set to ");
                Serial.println(cmd_val);
            } else if (cmd_c=='f' && num_vals == 1) {
                vehicle.set_throttle_speed(9999);
                Serial.println("Forward at full speed.");
            } else if (cmd_c=='r' && num_vals == 1) {
                vehicle.set_throttle_speed(-9999);
                Serial.println("Reverse at full speed.");
            } else if (cmd_c=='s' && num_vals == 1) {
                vehicle.set_throttle_speed(0);
                Serial.println("Throttle stopped.");
            }
        }
        Serial.print("throttle: ");
        Serial.print(throttle_val);
        Serial.print("\tsteering: ");
        Serial.println(steering_val);

    }

    return 0;
}
