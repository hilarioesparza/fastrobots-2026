--- 
title: "Lab 12"
description: "Lab Description and Report"
layout: default
---

# Lab 12: Inverted Pendulum

For the final lab of this course, students had the option of either navigating the built environment that we performed mapping and Bayes filtering on previously, or to implement an inverted pendulum stunt with the car. Because I am more interested in controls than mapping or navigation, I decided to attempt to implement the inverted pendulum stunt. For this stunt, I needed to stabilize the motion of the car while also attempting to balance one side of the car directly above the other side, an inherently very unstable position to put a car into.

### Control Logic

In order to control the car, I decided to use a linear quadratic regulator (LQR) to determine what control gains would be needed to optimally maintain the car’s upright position. To do this, I needed to model the system dynamics of the car, linearize this model about the point of interest, discretize the continuous model, and then finally use Python to solve the Ricatti Equation to derive the optimal gain K.

#### Mathematical Model

$$ 
\\sum_{i=1}^n X_i 
$$

```cpp
float acc_x;
float acc_y;
float acc_z;
float gyro_x;
float angular_speed;
float pitch_error;
float input;
int first_run_pendulum = 1;
float a_x = 0.5;
float a_y = 0.5;
float a_z = 0.5;
float g_x = 0.2;
float cutoff_degree = 50;
float cutoff_radians = (cutoff_degree*PI)/(180.0);
float gyro_bias;
int first_data_collected = 0;
float dt_pendulum = 0.00345077469;
float acc_pitch;
float pitch_offset;
unsigned long previous_time;


float avg_dt = 0;


void inverted_pendulum(){


   drive_motors(pwm_input);


   if (first_run_pendulum) {


       int place_holder = 0;
       while (place_holder < 500){
           if (myICM.dataReady()){
               myICM.getAGMT();
               gyro_bias = gyro_bias + myICM.gyrX();
               delay(2);
               place_holder = place_holder + 1;
           }
       }


       gyro_bias = gyro_bias/500;


       while (!first_data_collected){
           if (myICM.dataReady()){
               acc_x = myICM.accX();
               acc_y = myICM.accY();
               acc_z = myICM.accZ();
               gyro_x =myICM.gyrX() - gyro_bias;
               first_data_collected = 1;
           }
       }


       angular_speed = (gyro_x*PI)/180.0;
       acc_pitch = atan2(acc_y, acc_z);


       pitch = acc_pitch;
       pitch_offset = pitch;


       first_run_pendulum = 0;


       previous_time = micros();


   } else {
       if (myICM.dataReady()){
           myICM.getAGMT();
         
           acc_y = (1-a_y)*myICM.accY()+(a_y*acc_y);
           acc_z = (1-a_z)*myICM.accZ()+(a_z*acc_z);
           gyro_x = myICM.gyrX()-gyro_bias;


           acc_pitch = atan2(acc_y, acc_z);
           angular_speed = (gyro_x*PI)/180.0;


           current_time = micros();


           dt_pendulum = ((float)(current_time - previous_time))*1e-6;
           previous_time = current_time;


           pitch = 0.9*pitch + 0.1*acc_pitch;
           pitch_error = (-PI/2 - pitch_offset) - pitch;


           input = -1*(k0*pitch_error - k1*angular_speed);


           pwm_input = input;


           if ((abs(pitch_error) >= cutoff_radians)) {
               pwm_input = 0;
           } else {
               if (pwm_input >= 0){
                   if (pwm_input > 255){
                       pwm_input = 255;
                   }
               } else {
                   if (pwm_input < -255){
                       pwm_input = -255;
                   }
               }
           }


           if ( dt_counter < length_of_array ){
               yaw1[dt_counter] = pitch;
               pwm[dt_counter] = pwm_input;
               time_array[dt_counter] = current_time;
               dt_counter = dt_counter + 1;
           }
       }
   }
}
```

```cpp
void
loop()
{
   // Listen for connections
   BLEDevice central = BLE.central();


   // If a central is connected to the peripheral
   if (central) {
       Serial.print("Connected to: ");
       Serial.println(central.address());


       // While central is connected
       while (central.connected()) {


           // Send data
           write_data();


           // Read data
           read_data();


           if (distance_pid && !distance_pid_running){
               distanceSensor0.startRanging();
               run_distance_pid();
               distance_pid_running = 1;
           }


           if (distance_pid && distance_pid_running){
               run_distance_pid();
           }


           if (send_distance_pid){
               Serial.println("Sending Distance PID data");
               transmit_distance_pid_info();
           }


           if (angular_pid){
               Serial.println("Running angular pid");
               run_angular_pid();
           }


           if (send_angular_pid){
               Serial.println("Sending Angular PID data");
               transmit_angle_pid_info();
           }


           if (disable_motors){
               stop_motors();
           }


           if (just_drive_forward && !already_started){
               distanceSensor0.startRanging();
               just_drive();
           }


           if (just_drive_forward && already_started){
               just_drive();
           }


           if (initiate_drift && !drift_running){
               distanceSensor0.startRanging();
               do_the_drift();
               drift_running = 1;
           }


           if (initiate_drift && drift_running){
               do_the_drift();
           }


           if (run_inverted_pendulum){
               inverted_pendulum();
           } else {
               first_run_pendulum =1;
           }
       }


       analogWrite(9, 0);
       analogWrite(11, 0);
       analogWrite(12, 0);
       analogWrite(14, 0);


       Serial.println("Disconnected");
   }
}
```

```cpp
       case START_INVERTED_PENDULUM:
           dt_counter = 0;
           pwm_input = 0;
           run_inverted_pendulum = 1;
           break;
      
       case STOP_INVERTED_PENDULUM:
           run_inverted_pendulum = 0;
           drive_motors(0);
           transmit_pendulum_info();
           break;


       case SET_LQR_GAINS:
           success = robot_cmd.get_next_value(k0);
           if (!success)
               return;
           success = robot_cmd.get_next_value(k1);
           if (!success)
               return;
           break;
```

```python
from scipy.linalg import solve_discrete_are
import numpy as np
from scipy.linalg import expm

g = 9.81 # m/s^2
total_car_mass = 0.532 # 0.532 with no quarters, 0.5887 with quarters # kg
fraction_of_top = 0.4
fraction_of_bottom = 1 - fraction_of_top
m = fraction_of_top*total_car_mass # kg
M = fraction_of_bottom*total_car_mass # kg
l = 0.10195 # m
r = 0.04081 # m 
alpha = 0.065/255 # Nm/pwm 0.1/255
dt = 0.006148 # 0.00345077469 # seconds

# Example matrices for A'X + XA - XBR^-1B'X + Q = 0
A_cont = np.array([[0, 1], [(g*(M+m))/(M*l), 0]])
B_cont = np.array([[0], [alpha/(M*l*r)]])

A_dis = expm(A_cont*dt)
B_dis = np.linalg.inv(A_cont) @ (A_dis - np.eye(2)) @ B_cont

Q = np.array([[250, 0], [0, 100]])
R = np.array([[0.001]])

X = solve_discrete_are(A_dis, B_dis, Q, R)

K = (np.linalg.inv(R+B_dis.T@X@B_dis))@(B_dis.T@X@A_dis)

time = []
pitch = []
pwm = []

k0 = round(K[0][0],2)
k1 = round(K[0][1],2)

print("k0: " + str(k0))
print("k1: " + str(k1))

string = str(k0) + "|" + str(k1)

ble.send_command(CMD.SET_LQR_GAINS, string)
```

```python

ble.send_command(CMD.START_INVERTED_PENDULUM, "")

```

```python

ble.send_command(CMD.STOP_INVERTED_PENDULUM, "")

```

# <img src="Images/Lab 11/simulated_bayes.png" style="max-width:90%"/>

<iframe width="560" height="315" src="https://www.youtube.com/embed/3n78RNZNCmE" title="ECE 4160: Lab 8 Fast Drift" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen> </iframe> 
