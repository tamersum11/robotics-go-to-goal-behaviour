# Robotics 101: Go to Goal Behaviour
In this study, has been simulated to go to goal behaviour of differential drive robot in MATLAB SIMULINK and has been implemented to Python code.

## Simulation of Go to Goal Behaviour in MATLAB SIMULINK
### MATLAB SIMULINK Structure of Go to Goal Behaviour System
</br > ![Go to Goal Behaviour System](/png/smlkhw311.PNG)

</br > 'distance to goal' Function:
```
function y = fcn(u)
%u(1) : x' - x
%u(2) : y' - y
y = sqrt(u(1)^2+u(2)^2);
```

</br > 'heading to goal' Function:
```
function y = fcn(u)
%u(1) : x_traj - x
%u(2) : y_traj - y
%u(3) : d*
y = sqrt(u(1)^2+u(2)^2)-u(3);
```

</br > 'angle correction' Function:
```
function y = fcn(u)
%u(1) : theta* - theta
y = atan2(sin(u(1)),cos(u(1)));
```

### Differential Drive Robot
Differential Drive Robot is the subsystem in Go to Goal Behaviour.
</br > 
</br > MATLAB SIMULINK Structure of Subsystem:
</br > ![Differential Drive Robot](/png/smlkhw312.PNG)

</br > 'Subsystem/diff_drive' Function:
```
function y = fcn(u)
%u(1) : v
%u(2) : w
%u(3) : x
%u(4) : y
%u(5) : phi
 
y = [u(1)*cos(u(5)); u(1)*sin(u(5));u(2)];
```
### Output of Go to Goal Behaviour System System in MATLAB SIMULINK 
</br > Output of Go to Goal Behaviour System for (10, 20) Goal Pose:
</br > ![Output of Go to Goal Behaviour-MATLAB](/png/smlkhw31.PNG)

## Output of Trajectory Following System in Python Implementation
</br > Output of Go to Goal Behaviour System for (10, 20) Goal Pose:
</br > ![Output of Go to Goal Behaviou-Python](/png/hw311.png)
