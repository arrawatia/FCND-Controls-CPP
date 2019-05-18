# Building a Controller

## Implementation 

The implementation is a modified version of the [Python controller solution](https://github.com/udacity/FCND-Controls/blob/solution/controller.py). 

### 0. Calculating motor commands

[C++ implementation](https://github.com/arrawatia/FCND-Controls-CPP/blob/master/src/QuadControl.cpp#L74-L95)

The motor commands are derived from the equations of motion. 

```
front left  ->  w1 ^ 2  = (cBar + pBar + qBar + rBar) / 4
front right ->  w2 ^ 2  = (cBar - pBar + qBar - rBar) / 4
rear right  ->  w3 ^ 2  = (cBar - pBar - qBar + rBar) / 4
rear left   ->  w4 ^ 2  = (cBar + pBar - qBar - rBar) / 4

where 
    pBar =  moment_x / (L / sqrt(2.f));
    qBar =  moment_y / (L / sqrt(2.f));
    rBar = -moment_z / kappa;
```

The NED coordinates has z axis pointed in the inverse direction compared to coordinate system used in the Python controller project, so `rBar` is inverted. 

Also, the order of propellors in quadrotor model used for deriving the equations in the lectures is different from the simulated quad. The student chat pointed me in the right direction.

### 1. Body rate controller

[C++ implementation](https://github.com/arrawatia/FCND-Controls-CPP/blob/master/src/QuadControl.cpp#L129-L135)

This controller takes in the desired body rates in rad/s and outputs the moments for all 3 axes. It is a P (proportional) controller.

I had to tune the `kPQR` gain to complete the first part of **Scenario 2: Body rate and roll/pitch control**. This required adding a constant amount of thrust to the altitude control logic to prevent the simulated quad from falling down. I chose to apply just enough thrust to counteract gravity and keep the quad in balanced hover.

### 2. Roll Pitch controller

[C++ Implementation](https://github.com/arrawatia/FCND-Controls-CPP/blob/master/src/QuadControl.cpp#L167-L193)

This controller uses the acceleration, thrust and vehicle attitude to output the body rate. 


The roll-pitch controller is a P controller which controls the roll(`p`) and pitch rates (`q`) in the body frame.  

It controls the R13 and R23 elements of the rotation matrix `R` used for converting between body-frame accelerations and world frame accelerations.


```
pR13  = k_p * (R13_target - R13_actual)
pR23  = k_p * (R23_target - R23_actual)
```
    
These values are converted into body frame angular velocities by

```
p = R10 * pR13 - R00 * pR23) / R22
q = R11 * pR13 - R01 * pR23) / R22
```

### 3. Altitude controller

[C++ Implementation](https://github.com/arrawatia/FCND-Controls-CPP/blob/master/src/QuadControl.cpp#L224-L243)

The altiude controller controls the commanded thrust based on position, velocity and acceleration.

The first iteration of the Altitude controller was a PD controller which was sufficient for completing **Scenario 3** successfully. 

The control equation for the PD controller is 

`u_1 = kPosZ * (target_position - position) + kVelZ * (target_velocity - velocity) + acceleration`

But **Scenario 4** introduces quads with non-ideal weight and the PD controller is not enough to handle this. As hinted in the exercise, I added a integrator component to the controller.

### 4. Lateral Position controller
[C++ Implementation](https://github.com/arrawatia/FCND-Controls-CPP/blob/master/src/QuadControl.cpp#L280-L293)

The Lateral position controller is a PD controller that controls the desired horizontal acceleration based on local NE position along with horizontal velocity and acceleration.

### 5. Yaw controller

[C++ Implementation](https://github.com/arrawatia/FCND-Controls-CPP/blob/master/src/QuadControl.cpp#L316-L328)

This is simple P controller to control the yaw angle. 


## Flight Evaluation Scenarios

- **Scenario 2: Body rate and roll/pitch control**

  A working body rate and roll/pitch controller were needed for this exercise. After tuning the gains for both these controller the quad levelled itself.

  As the Altitude controller is not implemented at this time, the quad falls down.Preventing this required adding a constant amount of thrust to the altitude control logic to prevent the simulated quad from falling down. I chose to apply just enough thrust to counteract gravity and keep the quad in balanced hover.

  Logs with PASS statements.

  ```
  Simulation #2 (../config/2_AttitudeControl.txt)
  PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
  PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
  ```

  A video showing the quad sucessfully levelling itself.

  [Scenario2.mov](video/Scenario2.mov)

- **Scenario 3: Position/velocity and yaw angle control** 

    Successfully completing this scenario needs the position, altitude and yaw controllers.

    This exercise needed a lot of tuning as we need to find the right values in a search space with 5 dimensions : `kpPosZ, kpPosXY, kpVelZ, kpVelXY and kpYaw`.

    After tuning, the 2 quads get to their destination points successfully with controlled rotation.    

    Logs with PASS statements.

    ```
    Simulation #2 (../config/3_PositionControl.txt)
    PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
    PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
    PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
    ```

    A video showing the both quads reaching the destination sucessfully.

    [Scenario3.mov](movie/Scenario3.mov)

- **Scenario 4: Non-idealities and robustness**

   This scenario added integral control to help with the mass imbalance.

   This scenario required tuning the system again but after successful tuning, all 3 quads reach their targets successfully.

   Logs with PASS statements.

    ```
    Simulation #2 (../config/4_Nonidealities.txt)
    PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
    PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
    PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
    ```

   A video showing the all quads reaching the destination sucessfully.

    [Scenario4.mov](video/Scenario4.mov)

- **Scenario 5: Tracking trajectories**

  This scenario was the most interesting. It was satisfying to see the drones fly in a figure 8 trajectory. The left drone does this almost perfectly but the right drone seems to be a little off. 

  But looking at the flight trajectories from the top shows that both are indeed flying in figure 8 trajectory.

  I tried tuning the gains some more to see of it would help but I suspect the trajectory for the right quad needs to be improved (as hinted in the prompt for the Extra challenges). At this point, I want to move on to the Estimation module and will come back to this once I have more time on hand.

  Logs with PASS statements.

    ```
    Simulation #2 (../config/5_TrajectoryFollow.txt)
    PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
    Simulation #3 (../config/5_TrajectoryFollow.txt)
    PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
    ```

   A video showing the both quads flying in a *figure eight* trajectory.

    [Scenario5.mov](video/Scenario5.mov)


## Tuned Parameters for the quad

### Position control gains
```
kpPosXY = 30
kpPosZ = 20
KiPosZ = 40
```

### Velocity control gains
```
kpVelXY = 13
kpVelZ = 10
```
### Angle control gains

    kpBank = 10
    kpYaw = 2

### Angle rate gains

    kpPQR = 95, 95, 6