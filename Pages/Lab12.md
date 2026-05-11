--- 
title: "Lab 12"
description: "Lab Description and Report"
layout: default
---

# Lab 12: Inverted Pendulum

For the final lab of this course, students had the option of either navigating the built environment that we performed mapping and Bayes filtering on previously, or to implement an inverted pendulum stunt with the car. Because I am more interested in controls than mapping or navigation, I decided to attempt to implement the inverted pendulum stunt. For this stunt, I needed to stabilize the motion of the car while also attempting to balance one side of the car directly above the other side, an inherently very unstable position to put a car into.

### Control Logic

In order to control the car, I decided to use a linear quadratic regulator (LQR) to determine what control gains would be needed to optimally maintain the car’s upright position. To do this, I needed to model the system dynamics of the car, linearize this model about the point of interest, discretize the continuous model, and then finally use Python to solve the Ricatti Equation to derive the optimal gain K.

#### Lagrangian Background

To develop a mathematical model of the car’s dynamics, it is necessary to start from a force balance equation and work backwards from there to see how the input affects the various states of the car. This, however, can be quite difficult for complex systems, and it is easier to use Lagrangian Mechanics as opposed to Newtonian Mechanics.

Lagrangian Mechanics states that the lagrangian, $$L$$, is equal to the kinetic energy $$K$$ minus the potential energy $$P$$. 

$$
L = K - P
$$

Once we know $$L$$, we can use the following relationship to determine how the inputs into the system affect its states:

$$
\frac{d}{dt}(\frac{\partial L}{\partial\dot{q}_i}) - \frac{\partial L}{\partial q_i} = Q_i
$$

Where $$q_i$$ represents one of the states of the system, and $$Q_i$$ represents the force affecting state $$q_i$$. To use this, therefore, we need to determine what $$K$$ and $$P$$ are.

#### Mathematical Model

I will use the following diagram (borrowed from lecture material) to represent the inverted pendulum system. The only change I will make is that the offset of $$\pi$$ will be an offset of $$\pi/2$$ in my system.

**INSERT PHOTO**

Here, $$M$$ is the mass of the lower portion of the pendulum, $$m$$ is the mass of the upper portion, $$l$$ is the length between them, $$\delta$$ is the drage due to friction, $$u$$ is the PWM input into the bottom wheels, $$\theta$$ is the pitch of the car relative to the ground, and $$x$$ is its distance from a wall.

From this, we have the following state representation.

$$
y =
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
$$

From this, we can derive equations for both $$P$$ and $$K$$:

$$
P = -mgl\cos\theta
$$
$$
K = \frac{1}{2}(M+m)\dot{x}^2 + \frac{1}{2}ml^2\dot{\theta}^2-m\dot{x}\dot{\theta}l\sin\theta
$$

This results in:

$$
L = K-P = \frac{1}{2}(M+m)\dot{x}^2 + \frac{1}{2}ml^2\dot{\theta}^2 - m\dot{x}\dot{\theta}l\sin\theta - mgl\sin\theta
$$

For $$q_1 = x$$, we have the following:

$$
Q_1 = F - \delta\dot{x}
$$

$$
\frac{\partial L}{\partial x} = 0
$$

$$
\frac{\partial L}{\partial \dot{x}} = (M+m)\dot{x} - m\dot{\theta}l\sin\theta
$$

$$
\frac{d}{dt}(\frac{\partial L}{\partial \dot{x}}) = (M+m)\ddot{x} - m\ddot{\theta}l\sin\theta - m\dot{\theta}^2l\cos\theta
$$

Resulting in:

$$
(M+m)\ddot{x} - m\ddot{\theta}l\sin\theta - m\dot{\theta}^2l\cos\theta = F - \delta\dot{x}
$$

For $$q_2 = \theta$$, we have the following:

$$
Q_1 = 0
$$

$$
\frac{\partial L}{\partial \theta} = -m\dot{x}\dot{theta}l\cos\theta - mgl\cos\theta
$$

$$
\frac{\partial L}{\partial \dot{\theta}} = ml^2\dot{\theta} - m\dot{x}l\sin\theta
$$

$$
\frac{d}{dt}(\frac{\partial L}{\partial \dot{\theta}}) = ml^2\ddot{\theta} - m\ddot{x}l\sin\theta - m\dot{x}\dot{\theta}l\cos\theta
$$

Resulting in:

$$
ml^2\ddot{\theta} - m\ddot{x}l\sin\theta + mgl\cos\theta = 0
$$

Combining the computed terms allows us to create the following matrix equation:

$$
\begin{bmatrix}
(M+m) & -ml\sin\theta \\
-ml\sin\theta & ml^2
\end{bmatrix}
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
\=
\begin{bmatrix}
F - \delta\dot{x} + m\dot{\theta}^2l\cos\theta \\
-mgl\cos\theta
\end{bmatrix}
$$

Solving the above equation for $\left[ \ddot{x},\ \ddot{\theta} \right]^T$ results in the following matrix, where $$\Delta = ml^2(M+m(1-\sin^2\theta))$$.

$$
\begin{bmatrix}
\ddot{x} \\
\ddot{\theta}
\end{bmatrix}
\=
\begin{bmatrix}
\frac{ml^2(F - \delta\dot{x} + m\dot{\theta}^2l\cos\theta - mg\sin\theta\cos\theta)}{\Delta} \\
\frac{ml\sin\theta(F - \delta\dot{x} + m\dot{\theta}^2l\cos\theta) - (M+m)(mgl\cos\theta)}{\Delta}
\end{bmatrix}
$$

From here, we get:

$$
\frac{d}{dt}
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
\=
\begin{bmatrix}
\dot{x} \\
\frac{ml^2(F - \delta\dot{x} + m\dot{\theta}^2l\cos\theta - mg\sin\theta\cos\theta)}{\Delta} \\
\dot{\theta} \\
\frac{ml\sin\theta(F - \delta\dot{x} + m\dot{\theta}^2l\cos\theta) - (M+m)(mgl\cos\theta)}{\Delta}
\end{bmatrix}
$$

#### Model Linearization

With the above matrix representation in hand, we can now linearize if about $$x=free$$, $$\dot{x}=0$$, $$\theta=\pi/2$$, and $$\dot{\theta}=0$$. After a lot of math, this eventually outputs:

$$
\frac{d}{dt}
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
\=
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\frac{\delta}{M} & \frac{mg}{M} & 0 \\
0 & 0 & 0 & 1 \\
0 & -\frac{\delta}{Ml} & \frac{g(M+m)}{Ml} & 0
\end{bmatrix}
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
\+
\begin{bmatrix}
0 \\
\frac{1}{M} \\
0 \\
\frac{1}{Ml}
\end{bmatrix}
F
$$

This, however, includes information of $$x$$ and $$\dot{x}$$, which I am not actually interested, so it can be reduced to the following form:

$$
\begin{bmatrix}
\dot{\theta} \\
\ddot{\theta}
\end{bmatrix}
\=
\begin{bmatrix}
0 & 1 \\
\frac{g(M+m)}{Ml} & 0
\end{bmatrix}
\begin{bmatrix}
\theta \\
\dot{\theta}
\end{bmatrix}
\+
\begin{bmatrix}
0 \\
\frac{1}{Ml}
\end{bmatrix}
F
$$

One remaining issue is that we can input PWM signals to the motors of the car, and I am unsure of how this maps to a force $$F$$. The force applied by the wheel will be equalt to the wheel's toqure divided by its radius, $$\frac{\tau}{r}$$, and I know that $$\tau$$ will be proportional to $$u$$ by some scaling factor $$\alpha$$ such that $$F = \frac{\tau}{r} = \frac{\alpha u}{r}$$. With this, we can rewrite the above expression to finally get:

$$
\begin{bmatrix}
\dot{\theta} \\
\ddot{\theta}
\end{bmatrix}
\=
\begin{bmatrix}
0 & 1 \\
\frac{g(M+m)}{Ml} & 0
\end{bmatrix}
\begin{bmatrix}
\theta \\
\dot{\theta}
\end{bmatrix}
\+
\begin{bmatrix}
0 \\
\frac{\alpha}{Mlr}
\end{bmatrix}
u
$$

$$
A
\=
\begin{bmatrix}
0 & 1 \\
\frac{g(M+m)}{Ml} & 0
\end{bmatrix}
$$

$$
B
\=
\begin{bmatrix}
0 \\
\frac{\alpha}{Mlr}
\end{bmatrix}
$$

With this state dynamics represntation in continuous time, we can now discretize $$A$$ and $$B$$ because the actual car updates in discrete time.

#### Model Discretization and Python Implementation

### Control 


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
