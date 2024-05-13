# 4WIS4WID_parameter_calibration

This project focuses on the direct kinematics parameter calibration aimed at reducing odometry and trajectory following errors in 4WIS4WID mobile robots. The calibration process involves optimizing twelve different parameters to enhance the accuracy and efficiency of the robot's motion.

## Direct kinematics
Lee, M.H. and Li, T.H.S., 2015. Kinematics, dynamics and control design of 4WIS4WID mobile robots. The Journal of Engineering, 2015(1), pp.6-16.


![Example Image 1](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/57017fe4-b69d-47de-b6a2-1b585154ab60)

![Example Image 2](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/6249d6f2-4ff9-47e1-82b2-d12a7e05baaf)
![Example Image 3](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/ae8ca325-6249-4a74-bb47-3e49697bb7c2)


![image](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/e3bb7f99-ba69-42af-be0d-f6955ae69c89)




12 different parameters : 


![image](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/75603978-5365-478d-9214-0d99ae5a9b9e)






## Robot used in experiment developed by:
Božić, M., Ćaran, B., Švaco, M., Jerbić, B. and Serdar, M., Mobile Wall-Climbing Robot for NDT inspection of vertical concrete structures.


<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/2c369f95-8c49-4c64-82e7-6ae829042f84" alt="Example Image 5" width="390">



Robot Control - [ROS noetic repo](https://github.com/BCaran/wcr_onboard_ws/tree/master)

Absolute Measurements - [OptiTrack Motive - vrpn client ros](https://wiki.ros.org/vrpn_client_ros)

## Trajectories

<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/38ca78ce-2c36-46ae-b256-7558fd279198" alt="Example Image 5" width="390">
<br> <!-- Add a new line here -->
<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/2066368a-1d95-445e-99d2-59ff415b8e3d" alt="Example Image 5" width="390">
<br> 
<img src="https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/6e19ca21-d07d-4f3b-9cb1-46ee59dd6b19" alt="Example Image 5" width="390">


## Dataset
- 6 trajectories x 5 repetitions = 30 total repetitions
- `abs_i.csv`: OptiTrack absolute data
- `odom_i.csv`: Logged motor signals
- Reading data: `read_data.m`
- Plotting data: `data_plotting.m`

## Optimization conducted in Matlab with 4 different optimization methods:
- Fmincon (`optim_fmincon.m`)
- Levenberg-Marquardt (via Simulink Parameter Estimator, `modelS.slx`, `prepareDataSimulink.m`)
- Genetic Algorithm (`optim_ga.m`)
- Particle Swarm (`optim_ps.m`)

## Cost Function
![Cost Function](https://github.com/JakVit/4WIS4WID_parameter_calibration/assets/144608817/a5ccf84c-c372-4254-a2d2-1dc8ccd4e8e8)
