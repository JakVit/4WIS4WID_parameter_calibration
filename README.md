# 4WIS4WID_parameter_calibration

This project focuses on the direct kinematics parameter calibration aimed at reducing odometry and trajectory following errors in 4WIS4WID mobile robots. The calibration process involves optimizing twelve different parameters to enhance the accuracy and efficiency of the robot's motion.

## Direct kinematics
Lee, M.H. and Li, T.H.S., 2015. Kinematics, dynamics and control design of 4WIS4WID mobile robots. The Journal of Engineering, 2015(1), pp.6-16.

<<<<<<< HEAD
![Example Image 1](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/57017fe4-b69d-47de-b6a2-1b585154ab60)

![Example Image 2](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/6249d6f2-4ff9-47e1-82b2-d12a7e05baaf)
![Example Image 3](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/ae8ca325-6249-4a74-bb47-3e49697bb7c2)


![image](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/e3bb7f99-ba69-42af-be0d-f6955ae69c89)

=======
![Example Image 1](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/9f58d3bb-f27e-4173-bb72-d1ab7f217b37)

![Example Image 2](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/b87fcfe8-e2ba-4d9e-b924-f3695d22f0f4)
![Example Image 3](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/2ba412b7-a483-4084-a0a9-a1febfd78b42)

![image](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/d6bd2705-5db2-4171-a49e-f1c9dee9aff6)
>>>>>>> master


12 different parameters : 

<<<<<<< HEAD
![image](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/75603978-5365-478d-9214-0d99ae5a9b9e)

=======
![image](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/5324b7e9-5530-4cda-bb34-2949956f4f53)
>>>>>>> master




## Robot used in experiment developed by:
Božić, M., Ćaran, B., Švaco, M., Jerbić, B. and Serdar, M., Mobile Wall-Climbing Robot for NDT inspection of vertical concrete structures.

<<<<<<< HEAD
<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/2c369f95-8c49-4c64-82e7-6ae829042f84" alt="Example Image 5" width="390">
=======
<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/1e694595-c267-4e1c-afe9-e3e4ff3c4e08" alt="Example Image 5" width="390">
>>>>>>> master

Robot Control - [ROS noetic repo](https://github.com/BCaran/wcr_onboard_ws/tree/master)

Absolute Measurements - [OptiTrack Motive - vrpn client ros](https://wiki.ros.org/vrpn_client_ros)

## Trajectories
<<<<<<< HEAD
<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/38ca78ce-2c36-46ae-b256-7558fd279198" alt="Example Image 5" width="390">
<br> <!-- Add a new line here -->
<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/2066368a-1d95-445e-99d2-59ff415b8e3d" alt="Example Image 5" width="390">
<br> 
<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/6e19ca21-d07d-4f3b-9cb1-46ee59dd6b19" alt="Example Image 5" width="390">
=======
<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/0bc92746-d64e-4957-a8f1-2f4eca21d0e1" alt="Example Image 5" width="390">
<br> <!-- Add a new line here -->
<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/f29c7431-2d2d-4bfa-8a6e-9301f1a00daa" alt="Example Image 5" width="390">
<br> 
<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/9d319c52-d190-4816-a5e4-060e571beb8f" alt="Example Image 5" width="390">
>>>>>>> master




## Dataset
- 6 trajectories x 5 repetitions = 30 total repetitions
- `abs_i.csv`: OptiTrack absolute data
- `odom_i.csv`: Logged motor signals
- Reading data: `read_data.m`
- Plotting data: `data_plotting.m`

## Optimization conducted in Matlab with 4 different optimization algorithms:
- Fmincon (`optim_fmincon.m`)
- Levenberg-Marquardt (via Simulink Parameter Estimator, `modelS.slx`, `prepareDataSimulink.m`)
- Genetic Algorithm (`optim_ga.m`)
- Particle Swarm (`optim_ps.m`)

## Cost Function
<<<<<<< HEAD
![Cost Function](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/a5ccf84c-c372-4254-a2d2-1dc8ccd4e8e8)

=======
![Cost Function](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/d43d850d-b6bf-4d89-a408-25b01fc8df45)
>>>>>>> master
