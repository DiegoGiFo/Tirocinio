- Small description of the project;
- Advantages and disadvantges;
- Areas of usage;

- Hardware (description of each component);
- software (commenting line by line);
- biulding;
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

- EleksMaker Mana SE board --> Link[https://www.amazon.com/EleksMaker-ManaSE-Stepper-Controller-Engraver/dp/B06Y5Q29CR];
- 2 x NEMA17 34mm stepper motors --> Link[https://www.amazon.it/gp/product/B01B2C7JU8/ref=oh_aui_detailpage_o05_s01?ie=UTF8&psc=1];
- MPU-6050 gyro/accelerometer --> Link[https://www.amazon.it/gp/product/B00PL70P7K/ref=oh_aui_detailpage_o03_s00?ie=UTF8&psc=1];
- 1 x Regolable buck converter DC-DC step down --> Link[https://www.amazon.it/gp/product/B01MQGMOKI/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1];
- 1 x 11.1V 2200mAh 30C Li-polymer Battery (da aggiungere in seguito) --> Link[];

## ELECTRONIC SCHEMATIC
