
#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro
#include <ros.h>                                             //Include the ros.h library so we can use the ROS system
#include <geometry_msgs/Twist.h>                             //Include geometry_msgs/twist since the received values from cmd_vel are of this type
//#include <hb_core_msgs/MotorCtrl.h>
#include <custom_msgs/imu_msgs.h>                            //Include custom_msgs/imu_msgs that allows us to publish on a ROS topic the values obtained from the IMU
#include <Arduino.h>

const int gyro_address = 0x68;                               //MPU-6050 I2C address (0x68 or 0x69)
int acc_calibration_value = 1000;                            //Enter the accelerometer calibration value

//Various settings
float pid_p_gain = 35;   //40                                      //Gain setting for the P-controller
float pid_i_gain = 1.0;  //1.2                                     //Gain setting for the I-controller
float pid_d_gain = 60;   //20                                      //Gain setting for the D-controller

////////////////////////////////////
int flag = 0;                                                //Variable needed to acquire the calibration value of the IMU

#define L 0.160f                                             // distance between the two wheels of the robot
#define R 0.040f                                             //radius of the wheel od the robot

const float A = L/(2.0f*R);                                  //Variables used to control the acceleration and the
const float B = 1.0f/R;                                      //deceleration of the robot
const float LAMBDA = 0.95f;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ROS

ros::NodeHandle  nh; // allows to create publisher/subscriber
//hb_core_msgs::MotorCtrl msg_vel;                             //Uncomment for publishing the motors velocity values
custom_msgs::imu_msgs gyro_msgs;                               //Delaration of the variable in which will be load the values of the IMU

float w_right, w_left;                                         //Angular velocity of the motors
float target_w_right = 0, target_w_left = 0;                   //Target velocity of the motors
float current_w_right = 0, current_w_left = 0;                 //Actual velocity of the motors

float received_linear, received_angular;                       //Variables used for the remote control of the robot
float cmd_value_right, cmd_value_left;

int get_direction(float x)
{
    if(x > 0) return 1;
    else return -1;
}

void motors_cb(const geometry_msgs::Twist &move)              //This function receives the values from the cmd_vel topic and processes this
{                                                             //in order to obtain the right value fot the motors control
  received_linear = move.linear.x;
  received_angular = move.angular.z;

  w_right = A*move.angular.z + B*move.linear.x;
  w_left = A*move.angular.z - B*move.linear.x;

  target_w_right = w_right;
  target_w_left = w_left;

  current_w_right = LAMBDA*current_w_right + (1-LAMBDA)*target_w_right;
  const float w_abs_right = fabs(current_w_right)*(100.0/M_PI);
  const float direction_right = get_direction(current_w_right);
  const float value_right = w_abs_right;
  cmd_value_right = value_right;

  current_w_left = LAMBDA*current_w_left + (1-LAMBDA)*target_w_left;
  const float w_abs_left = fabs(current_w_left)*(100.0/M_PI);
  const float direction_left = get_direction(current_w_left);
  const float value_left = w_abs_left;
  cmd_value_left = value_left;

  //msg_vel.dx = ((int)cmd_value_right);
  //msg_vel.sx = ((int)cmd_value_left);
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &motors_cb);
//ros::Publisher pub_vel("/velocity", &msg_vel);
ros::Publisher pub_gyro("/gyro_info", &gyro_msgs);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;

//Variable used for the acqusition of the gyro values
int gyro_pitch_data_raw, gyro_yaw_data_raw, gyro_roll_data_raw;
int gyro_temp_data_raw;
int accelerometer_data_raw, accelerometer_data_y_raw, accelerometer_data_z_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//function that calculates the acc_calibration_value
int take_acc_val(){
  if( flag == 0){
    Wire.beginTransmission(0x68);
    Wire.write(0x3F);
    Wire.endTransmission();
    Wire.requestFrom(0x68,2);
    return ((Wire.read()<<8|Wire.read())*-1);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup basic functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){

  Wire.begin();                                                             //Start the I2C bus as master
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode

  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                     //End the transmission with the gyro

  pinMode(2, OUTPUT);                                                       //Configure digital poort 2 as output
  pinMode(5, OUTPUT);                                                       //Configure digital poort 5 as output
  pinMode(3, OUTPUT);                                                       //Configure digital poort 3 as output
  pinMode(6, OUTPUT);                                                       //Configure digital poort 6 as output
  pinMode(13, OUTPUT);                                                      //Configure digital poort 13 as output

  for(receive_counter = 0; receive_counter < 500; receive_counter++){       //Create 500 loops
    if(receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));        //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset

  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //acquire the value of acc_calibration_value
  acc_calibration_value = take_acc_val();
  flag = 1;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //-------------------------------------------------------------------
  nh.initNode(); // initialize ROS nodes
  //nh.advertise(pub_vel);
  nh.advertise(pub_gyro);
  nh.subscribe(sub);
  //------------------------------------------------------------------------
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  if(receive_counter <= 25)receive_counter ++;                              //The received byte will be valid for 25 program loops (100 milliseconds)
  else received_byte = 0x00;                                                //After 100 milliseconds the received byte is deleted

  //Load the battery voltage to the battery_voltage variable.
  //85 is the voltage compensation for the diode.
  //Resistor voltage divider => (3.3k + 3.3k)/2.2k = 2.5
  //12.5V equals ~5V @ Analog 0.
  //12.5V equals 1023 analogRead(0).
  //1250 / 1023 = 1.222.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  /*battery_voltage = (analogRead(0) * 1.222) + 85;

  if(battery_voltage < 1050 && battery_voltage > 800){                      //If batteryvoltage is below 10.5V and higher than 8.0V
    digitalWrite(13, HIGH);                                                 //Turn on the led if battery voltage is to low
    //  low_bat = 1;                                                            //Set the low_bat variable to 1
  }*/

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3B);                                                         //Start reading at register 3B
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 6);                                        //Request 6 bytes from the gyro
  accelerometer_data_y_raw = Wire.read()<<8|Wire.read();
  accelerometer_data_z_raw = Wire.read()<<8|Wire.read();
  accelerometer_data_raw = Wire.read()<<8|Wire.read();
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;

  angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296;           //Calculate the current angle according to the accelerometer

  if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5){                     //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }

  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 6);                                        //Request 6 bytes from the gyro
  gyro_yaw_data_raw = Wire.read()<<8|Wire.read();
  gyro_pitch_data_raw = Wire.read()<<8|Wire.read();
  gyro_roll_data_raw = Wire.read()<<8|Wire.read();

  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable



  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x41);                                                         //Start reading at register 41
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  gyro_temp_data_raw = (Wire.read()<<8|Wire.read())/340.00+36.53;           //The value acquired from the IMU is converted in degrees centigrades

  gyro_msgs.Gyro_Acc_X = accelerometer_data_raw;                            //Assign to each field of the gyro_msgs the correspondent values
  gyro_msgs.Gyro_Acc_Y = accelerometer_data_y_raw;                          //in order to publish it on a ROS topic
  gyro_msgs.Gyro_Acc_Z = accelerometer_data_z_raw;

  gyro_msgs.Gyro_Vel_Yaw = gyro_yaw_data_raw;
  gyro_msgs.Gyro_Vel_Pitch = gyro_pitch_data_raw;
  gyro_msgs.Gyro_Vel_Roll = gyro_roll_data_raw;

  gyro_msgs.Gyro_Temp = gyro_temp_data_raw;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board.
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  angle_gyro -= gyro_yaw_data_raw * 0.0000003;                              //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid_i_mem > 400) pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
  else if(pid_i_mem < -400) pid_i_mem = -400;
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400) pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
  else if(pid_output < -400) pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if(pid_output < 5 && pid_output > -5)pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced

  if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1){    //If the robot tips over or the start variable is zero or the battery is empty
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Remote control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

  if(received_linear == 0 && received_angular < 0 ){                         //If the command sent to the robot is to turn left
    pid_output_left -= cmd_value_left;                                       //Decrease the left motor speed
    pid_output_right += cmd_value_right;                                     //Increase the right motor speed
  }

  if(received_linear == 0 && received_angular > 0 ){                         //If the command sent to the robot is to turn right
    pid_output_left += cmd_value_left;                                       //Increase the left motor speed
    pid_output_right -= cmd_value_right;                                     //Decrease the right motor speed
  }

  if(received_angular == 0 && received_linear < 0){                          //If the command sent to the robot is to go foreward
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                             //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_output > cmd_value_right * -1)pid_setpoint -= 0.005;              //Slowly change the setpoint angle so the robot starts leaning forewards
  }

  if(received_angular == 0 && received_linear > 0){                          //If the command sent to the robot is to go backward
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                              //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < cmd_value_right)pid_setpoint += 0.005;                   //Slowly change the setpoint angle so the robot starts leaning backwards
  }

  if(!((received_angular == 0 && received_linear > 0)||(received_angular == 0 && received_linear < 0 ))){                                         //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }

  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0){                                                    //If the setpoint is zero degrees
    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while(loop_timer > micros());
  loop_timer += 4000;

/////////////////////////////////////////
  pub_gyro.publish(&gyro_msgs);
  //pub_vel.publish(&msg_vel);
  nh.spinOnce();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect){
  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory){             //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0){                                     //If the throttle_left_motor_memory is negative
      PORTD &= 0b11011111;                                                  //Set output 3 low to reverse the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else PORTD |= 0b00100000;                                               //Set output 3 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_left_motor == 1)PORTD |= 0b00000100;             //Set output 2 high to create a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2)PORTD &= 0b11111011;             //Set output 2 low because the pulse only has to last for 20us

  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
      PORTD |= 0b01000000;                                                  //Set output 5 low to reverse the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else PORTD &= 0b10111111;                                               //Set output 5 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_right_motor == 1)PORTD |= 0b00001000;            //Set output 4 high to create a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)PORTD &= 0b11110111;            //Set output 4 low because the pulse only has to last for 20us
}
