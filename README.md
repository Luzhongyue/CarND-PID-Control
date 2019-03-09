# CarND-PID-Control

## Introduction

Implement a PID controller in C++ to maneuver the vehicle around the track.The simulator will provide the cross track error (CTE) and 
the velocity (mph) in order to compute the appropriate steering angle.

![](https://github.com/Luzhongyue/CarND-PID-Control/blob/master/Images/simulator.png)

## What is PID

PID is propotion(P), integral(I) and derivative(D), the three parameters represents how we use our error to generate control instructions. 
The whole process is as follows:

![](https://github.com/Luzhongyue/CarND-PID-Control/blob/master/Images/PID.png)

Firstly,according to the feedback error and reference calculate the error, the error according to the specific situation can be a variety 
of measures, such as controling the vehicle in a specified path , erros are the current position of motor vehicles and the distance 
reference line, controling the speed of the vehicle in the set of values, the error is the difference between the current speed and the 
speed expected. After calculating errors, the proportion, integral and differential can be calculated according to errors, Kp, Ki, and Kd 
are three coefficients, they determine the proportion of these three effects on the final output.The sum of P,I,D is used as the 
final output signal.

P: P had the most directly  effect on the car's behavior. It causes the car to steer proportional (and opposite) to the car's distance from
the lane center (which is the CTE). Within a reasonable numerical range,t he larger the Kp is, the better the control effect is  
(the faster it returns to the reference line). However, when the position itself is far away from the reference line and the Kp 
coefficient is too high, the vehicle will lose control and become unstable.

D: The derivative weight(Kd) determines the influence of CTE change rate on feedback control. Intuitively speaking, increasing P  
will increase the tendency of the driverless car to move in the direction of the reference line. However, Increasing  D  willincrease the
"resistance" of the driverless car to the motion in the direction of the reference line, which make Makes the motion towards the reference 
line smoother. Systems that use t a P coefficient too large, a D coefficient too small are called underdamped, in this case, the driverless
car will oscillate along the reference line. Conversely, called overdampe, which will take a long time for the driveless car to correct its
error. The parameters P and D can be appropriately selected to enable the unmanned vehicle to quickly return to the reference line while 
maintaining a good motion on the reference line.

I: When there is a disturbance in the environment, P tends to move in the direction of the reference line, and D tries to counteract this 
tendency. As a result, the driverless car cannot always move along the reference line, this is called steady state error. In order to solve
the problem, we have to import intergral. Intergral is essentially the area of the graph from the actual route of the car to the reference 
line. After adding the integral term, the control function minimizes the vehicle route integral as much as possible, which will avoid the 
situlation called steady state error. 

The PID control function as showed below:
```
value = -Kp * p_error - Ki * i_error - Kd * d_error
```

The effect of PID is showed as below:

![](https://github.com/Luzhongyue/CarND-PID-Control/blob/master/Images/pidcompare.png)

## How to choose the three hyperparameters

The three hyperparameters can be chose using *Twiddle* algorithm. The principle is shown in the figure below：

![](https://github.com/Luzhongyue/CarND-PID-Control/blob/master/Images/twiddle.png)
