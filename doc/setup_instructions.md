# Setup Instructions

## Parts List
| Component                        | Manufacturer & Model no.   |
| ---------------------------------|----------------------------|
| RC car chassis & drivetrain      | MST FXX-D                  |
| Sensored BLDC motor              | MST XBLS 601012            |
| Servo motor                      | Savox SAVSB2274SG          |
| Battery                          | Multistar 3S 11.1V 5200mAh |
| Voltage regulator                | Icstation LM2596 XL6009    |
| Single-board computer            | Nvidia Jetson TX2          |
| IMU                              | Variense VMU931            |
| LIDAR                            | Hokuyo URG-04LX-UG01       |
| ESC                              | VESC 4.12                  |

Other parts required include:
- gamepad controller
- acrylic plates  
- switches
- switch mount (can be 3D-printed)
- USB hub
- USB to micro USB (x1) and USB to mini USB (x2) cables
- USB Wi-Fi adapter (optional)
- Wireless switch transmitter and receiver (for emergency remote kill switch, optional)
- fasteners and standoffs
- wires and connectors

## Steps
1. Download the 3D mechanical design of the vehicle from [here](https://cmu.box.com/s/6bi81ib2nryrp9w1w69b6gy75ssbfk5l).
2. Laser cut the acrylic plates and 3D print the switch mount.
3. Setup the electrical connections according to the [circuit diagram](circuit_diagram.pdf).
5. Load the firmware into the VESC and calibrate its settings (instructions [here](vesc_setup.md)).
4. Assemble the car.
6. [Install ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) and all other [package dependencies](dependencies.md) on the Jetson TX2.
7. Set the udev rules on the Jetson (for the VESC, IMU and LIDAR) and the local computer (for the controller). Refer to the [rules file](99-usb-serial.rules) and use the same symbolic links, changing the values accordingly.
8. Have a local computer with ROS and the same packages installed to remotely control the car.
9. Setup the network connections between the local computer and the car (instructions [here](remote_connection.md)).
10. The vehicle is now ready for [test runs](test_runs.md).