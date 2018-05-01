# ROS packages dependencies
- ackermann_msgs
- rosserial
- serial
- gscam
- urg_node, urg_c, csm, laser_proc
- amcl, map_server
- robot_localization
- boost
- armadillo
- gstreamer (on Jetson)
```bash
$ sudo apt-get install ros-kinetic-ackermann-msgs
$ sudo apt-get install ros-kinetic-rosserial
$ sudo apt-get install ros-kinetic-serial
$ sudo apt-get install ros-kinetic-gscam
$ sudo apt-get install ros-kinetic-urg-node ros-kinetic-urg-c ros-kinetic-csm ros-kinetic-laser-proc
$ sudo apt-get install ros-kinetic-amcl ros-kinetic-map-server
$ sudo apt-get install ros-kinetic-robot-localization
$ sudo apt-get install libboost-dev
$ sudo apt-get install libarmadillo-dev
$ sudo apt-get install gstreamer1.0 libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
```

## Installing packages with no installation candidate
On Jetson TX2, many packages may not have their installation candidates.  
This may be solved by
```bash
$ sudo add-apt-repository restricted
$ sudo add-apt-repository multiverse
$ sudo add-apt-repository universe
$ sudo apt-get update
```

## Getting Jetson TX2 to recognize ACM devices
By default, the Jetson TX2 may not detect USB devices that report as ttyACM. The kernel needs to be modified in order to detect ACM USB devices. [These scripts](http://www.jetsonhacks.com/2017/07/31/build-kernel-ttyacm-module-nvidia-jetson-tx2/) by JetsonHacks will build the kernel and modules on the Jetson TX2 that includes the ACM module.
```bash
$ git clone https://github.com/jetsonhacks/buildJetsonTX2Kernel.git
$ cd buildJetsonTX2Kernel
$ ./getKernelSources.sh
```
The script will open an editor on the kernel configuration file. Check the USB modem ACM support module to enable it (you can use ctrl+F and find ACM). Save the configuration file as /usr/src/kernel/kernel-4.4/.config when done editing.  
You can verify that it is enabled by checking that the line `CONFIG_USB_ACM=y` exists on the config file. (Instruction obtained from [here](https://github.com/datlife/jetson-car/issues/7))  
Build the kernel after that.
```bash
$ ./makeKernel.sh
$ ./copyImage.sh
```
After a reboot, the Jetson TX2 should be able to recognize ACM devices.