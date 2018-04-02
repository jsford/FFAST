# Setup Instructions

These instructions are for setting up a Jetson TX2 for use with FFAST.

## Dependencies to install

### Lightweight Communications and Marshalling (LCM)
[LCM](https://github.com/lcm-proj/lcm) is an interprocess communication library developed by MIT for their entry in the DARPA autonomous driving challenge. It uses UDP multicast to send messages in a decentralized way without a central process to control the creation of and subscription to the message channels.  
Installation instructions can be found [here](https://lcm-proj.github.io/build_instructions.html).

Ensure you have these required packages:
* build-essential
* libglib2.0-dev
* openjdk-8-jdk
* python-dev

Download LCM from their [Downloads](https://github.com/lcm-proj/lcm/releases) page.  
From a terminal, run the following commands: 
```bash
$ unzip lcm-X.Y.Z.zip
$ cd lcm-X.Y.Z
$ ./configure
```
make sure python and java are enabled
```bash
$ make
$ sudo make install
$ sudo ldconfig
```

### Pods
The [pods](https://sourceforge.net/p/pods/home/Home/) project provides a standard format for organizing a modular codebase. Each module, or pod, places its external aspects in a standardized location. In this way, a programmer that wishes to interact with the pod already knows where to find what he needs.  
The PodsTool provides templates for building pods of a few different types.  
Installation instructions can be found [here](https://sourceforge.net/p/pods/wiki/PodsTool/).

Ensure you have subversion and cmake installed
```bash
$ svn checkout svn://svn.code.sf.net/p/pods/svn pods
$ cd pods
$ sudo make BUILD_PREFIX=/usr/local/
```
The install script provided with the tool doesn’t install the templates in a system-wide directory, so after you follow their installation instructions, you need to copy the /pods/data folder to the correct location. On my Ubuntu 16.04 system, I used the following command to do so. Note that the location may differ on your machine, especially if you have a different version of Python.
```bash
$ sudo cp -r _pods/data /usr/local/lib/python2.7/dist-packages/_pods/
```

### Flexiport
[Flexiport](http://gbiggs.github.io/flexiport/) is a flexible communications library that provides a consistent interface for communicating over a range of data port types. It is a dependency required for HokuyoAIST.

Clone the flexiport repository from [here](https://github.com/gbiggs/flexiport).  
Edit the CMakeLists.txt file and turn the build examples, python bindings and documentation options off.
```bash
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig
```
<br>

### HokuyoAIST
[HokuyoAIST](http://gbiggs.github.io/hokuyoaist/) is a driver for the Hokuyo lidar used on FFAST.

Clone the HokuyoAIST repository from [here](https://github.com/gbiggs/hokuyoaist)  
Edit the CMakeLists.txt file and turn the build examples, python bindings and documentation options off.
```bash
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig
```

After installation of these two libraries, the library folder is by default located within another directory that isn't where they are usually found in #include statements. To move them: 
```bash
$ cd /usr/local/include
$ sudo mv flexiport-2/* .
$ sudo rmdir flexiport-2
$ sudo mv hokuyoaist-3/* .
$ sudo rmdir hokuyoaist-3
```

## Getting Jetson TX2 to recognize ACM devices
By default, the Jetson TX2 may not detect USB devices that report as ttyACM. The kernel needs to be modified in order to detect ACM USB devices. [These scripts](http://www.jetsonhacks.com/2017/07/31/build-kernel-ttyacm-module-nvidia-jetson-tx2/) by JetsonHacks will build the kernel and modules on the Jetson TX2 that includes the ACM module.
```bash
$ git clone https://github.com/jetsonhacks/buildJetsonTX2Kernel.git
$ cd buildJetsonTX2Kernel
$ ./getKernelSources.sh
```
The script will open an editor on the kernel configuration file. Check the ACM module to enable it (you can use ctrl+F and find ACM). Save the configuration file as /usr/src/kernel/kernel-4.4/.config when done editing.  
You can verify that it is enabled by checking that the line `CONFIG_USB_ACM=y` exists on the config file. (Instruction was obtained from [here](https://github.com/datlife/jetson-car/issues/7))  
Build the kernel after that.
```bash
$ ./makeKernel.sh
$ ./copyImage.sh
```
After a reboot, the Jetson TX2 should be able to recognize ACM devices.

## Installing Arduino IDE on Jetson TX2
Follow the downloading and installation guide [here](https://www.arduino.cc/en/Guide/Linux).  
Download the Arduino IDE installer for LinuxARM from [here](https://www.arduino.cc/en/Main/Software).  
Extract the installation directory and move it into the /opt directory (the /opt directory is reserved for all the software and add-on packages that are not part of the default installation).
```bash
$ sudo mv arduino-1.8.5 /opt
$ cd /opt/arduino-1.8.5/
$ chmod +x install.sh
$ ./install.sh
```
After installation, several errors may occur while launching the Arduino IDE using the `./arduino` command.

If the error `/opt/arduino-1.8.5/java/bin/java: No such file or directory` is encountered, 32-bit Java needs to be installed. (Installation instructions were obtained from [here](https://github.com/openhab/openhabian/issues/57))
1. Enable 32-bit architecture
```bash
$ sudo dpkg --add-architecture armhf
```

2. Install 32-bit libraries
```
$ sudo apt-get install libc6:armhf libncurses5:armhf libstdc++6:armhf libx11-dev:armhf libxext-dev:armhf libxrender-dev:armhf libxtst-dev:armhf libfontconfig:armhf libxft-dev:armhf libgtk2.0-0:armhf libsm-dev:armhf libudev-dev:armhf libusb-dev:armhf
```
If the packages are unable to be located, refer to the section below on finding packages' installation candidates.

3. [Download](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html) and [install](https://docs.oracle.com/javase/8/docs/technotes/guides/install/linux_jdk.html) 32-bit Java 8 from Oracle website.

4. Extract and move the files into /opt/jdk

5. Update alternatives so the 32-bit version is the default java version. (Instructions were obtained from [here](https://www.digitalocean.com/community/tutorials/how-to-manually-install-oracle-java-on-a-debian-or-ubuntu-vps))
```bash
$ sudo update-alternatives --install /usr/bin/java java /opt/jdk/jdk1.8.0_161/bin/java 100
$ sudo update-alternatives --install /usr/bin/javac javac /opt/jdk/jdk1.8.0_161/bin/javac 100
```
Configure the default option
```bash
$ sudo update-alternatives --config java
$ sudo update-alternatives --config javac
```
In both cases select the option that points to the 32-bit executable in /opt/jdk/  
Verify that it has been successfully configured by running
```bash
$ update-alternatives --display java
$ update-alternatives --display javac
```
The output should look like
```bash
java - manual mode
  link best version is /usr/lib/jvm/java-8-openjdk-arm64/jre/bin/java
  link currently points to /opt/jdk/jdk1.8.0_161/bin/java
  link java is /usr/bin/java

javac - manual mode
  link best version is /usr/lib/jvm/java-8-openjdk-arm64/bin/javac
  link currently points to /opt/jdk/jdk1.8.0_161/bin/javac
  link javac is /usr/bin/javac
```


If the error `java.lang.UnsatisfiedLinkError: <some_library> : wrong ELF class: ELFCLASS64` is encountered, download the library with armhf architecture.  
For example, error: `java.lang.UnsatisfiedLinkError: /opt/arduino-1.8.5/java/lib/arm/libawt_xawt.so: libXrender.so.1: wrong ELF class: ELFCLASS64`  
Download libXrender by
```bash
$ sudo apt-get install libxrender-dev:armhf
```

If the error `can't load com.sun.java.swing.plaf.gtk.GTKLookAndFeel` is encountered, the arduino startup script needs to be edited. (Solution was obtained from [here](http://forum.arduino.cc/index.php?topic=26770.0))  
Remove the definition `"-Dswing.defaultlaf=com.sun.java.swing.plaf.gtk.GTKLookAndFeel"` from `JAVA_OPTIONS`.

## Installing Teensyduino on Jetson TX2
Follow the downloading and installation guide [here](https://www.pjrc.com/teensy/td_download.html).  
Set the udev rule for the Teensy device
```bash
$ cd /etc/udev/rules.d
$ sudo wget https://www.pjrc.com/teensy/49-teensy.rules --no-check-certificate
```
Download and run the Teensyduino Installer
```bash
$ wget https://www.pjrc.com/teensy/td_140/TeensyduinoInstall.linuxarm --no-check-certificate
$ chmod +x TeensyduinoInstall.linuxarm 
$ ./TeensyduinoInstall.linuxarm
```
When prompted to select the Arduino folder, choose the installation directory /opt/arduino-1.8.5/

When running the installer, several errors may again be encountered. These are the same errors regarding the 32-bit libraries that cannot be found, hence again just download the corresponding libraries in armhf architecture.  
If the installation is successful but uploading through the Arduino IDE fails due to the Teensy loader, it is the same error that still exists.  
Run the teensy executable:
```bash
$ ./opt/arduino-1.8.5/hardware/tools/teensy
```
and the missing 32-bit libraries will again be shown. Download these libraries in armhf architecture until running the executable opens up the Teensy window.

## Getting Arduino serial monitor to run
There may be some more errors encountered when opening the serial monitor on Arduino. If an exception was thrown regarding the Java libjSSC, follow these steps (obtained from [here](https://forum.arduino.cc/index.php?topic=400808.15)).  
First, open jssc-2.8.0.jar that is in the /opt/arduino-1.8.5/lib directory using an archive manager.  
In the .jar file there is a libs/linux/ directory, copy the libjSSC-2.8_armhf.so into the ~/.jssc/linux folder (you can use the terminal to navigate to that folder and enter the command `nautilus .` to open in files explorer). Create the .jssc folder if it doesn't exist.  
If the ~/.jssc/linux folder already has a libjSSC-2.8_armsf.so delete it. (notice the sf.so vs hf.so for the file we extracted.)  
Right click the libjSSC-2.8_armhf.so and click Make Link.  
Rename the link to libjSSC-2.8_armsf.so (again notice the sf.so)  
The next time you open the Arduino IDE and the Serial Monitor from there it should no longer print an exception and it should work.

## Installing packages with no installation candidate
On Jetson TX2, many packages may not have their installation candidates.  
This may be solved by
```bash
$ sudo add-apt-repository restricted
$ sudo add-apt-repository multiverse
$ sudo add-apt-repository universe
$ sudo apt-get update
```
