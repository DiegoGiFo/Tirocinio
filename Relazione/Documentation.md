- building;
- remote control;

# ROS BALANCING ROBOT

This is my version of a self-balancing robot. The robot uses two NEMA17 stepper motors. I decided to use stepper motors because they are more precise with respect to DC motors and have good performance in term of power consumption.
The electronics consists of an EleksMaker Mana SE board ,that contains an Arduino Nano and two stepper driver A4988, an MPU-6050 gyro/accelerometer, a buck converter DC-DC and a Raspberry Pi3.
The EleksMaker board is used to drive the motors and the Raspberry for the ROS implementation and the remote control.
The mechanical construction of the entire robot is made up of several 3D printed parts that are screwed together with M3 screws.
The code, the schematic and the cad files are provided in the dowload section at the bottom of this page.

The goal of this robot is to learn all the phases of realization of a project and the usage of the ROS system.

## PART LIST

The overall cost of the robot is of about 200€. The component that i used are the follwing:

- EleksMaker Mana SE board --> [Link](https://www.amazon.com/EleksMaker-ManaSE-Stepper-Controller-Engraver/dp/B06Y5Q29CR);
- 2 x NEMA17 34mm stepper motors --> [Link](https://www.amazon.it/gp/product/B01B2C7JU8/ref=oh_aui_detailpage_o05_s01?ie=UTF8&psc=1);
- MPU-6050 gyro/accelerometer --> [Link](https://www.amazon.it/gp/product/B00PL70P7K/ref=oh_aui_detailpage_o03_s00?ie=UTF8&psc=1);
- 1 x Regolable buck converter DC-DC step down --> [Link](https://www.amazon.it/gp/product/B01MQGMOKI/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1);
- 1 x 11.1V 2200mAh 30C Li-polymer Battery (da aggiungere in seguito) --> Link[];
- Raspberry Pi 3 --> [Link](https://www.amazon.it/Raspberry-PI-Model-Scheda-madre/dp/B01CD5VC92/ref=sr_1_3?ie=UTF8&qid=1526569586&sr=8-3&keywords=raspberry+pi+3).

### THE MPU-6050 GYRO/ACCELEROMETER

The MPU-6050 sensor contains a MEMS accelerometer and a MEMS gyro in a single chip. It is very accurate, as it contains 16-bits analog to digital conversion hardware for each channel. Therefor it captures the x, y, and z channel at the same time. The sensor uses the I2C-bus to interface with the Arduino.
The orientation of the gyro is important. Make sure to mount the gyro in the exact same orientation as shown in the electronic schematic picture. Otherwise the software cannot calculate the correct angle and the robot will not work.

## ELECTRONIC SCHEMATIC

In this photo is explained how to connect the components of this project.

![electronic_schematic](https://github.com/DiegoGiFo/Tirocinio/blob/master/Relazione/tot_scheme.jpg?raw=true "Schematic")

## CODE

The commented code can be dowload here: [Code_download](https://github.com/DiegoGiFo/Autobalancing_Robot/tree/master/complete_vs/final_balancing_robot/final)

## COMMENTS

The robot is realized using some component and some software algorith that are explain in this section.

### THE MPU-6050 GYRO/ACCELEROMETER

![MPU-6050](https://github.com/DiegoGiFo/Tirocinio/blob/master/Relazione/mpu-6050.jpg?raw=true "Schematic")

The only gyro/accelerometer that is supported by the software is the MPU-6050.
The InvenSense MPU-6050 sensor contains a MEMS accelerometer and a MEMS gyro in a single chip. It is very accurate, as it contains 16-bits analog to digital conversion hardware for each channel. Therefor it captures the x, y, and z channel at the same time. The sensor uses the I2C-bus to interface with the Arduino.
The MPU-6050 always acts as a slave to the Arduino with the SDA and SCL pins connected to the I2C-bus.
