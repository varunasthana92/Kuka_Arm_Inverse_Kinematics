# Kuka LBR iiwa 14 Forward and Inverse Kinematics for Weld Simulation

Welding simulation is demonstrated by solving the inverse kinematics for a manipulator with 7 revolute joints. Kuka LBR iiwa 14 has 7-Degrees of Freedom with a spherical wrist configuration at the end-effector. With 7-DOFs, infinite solution may exist to reach to the target position and orientation, hence for to avoid that one of the joint was considered locked. Project was part of the academic coursework ENPM662-Introduction to Robot Modeling at the Univeristy of Maryland-College Park.

<p align="center">
	<img src="https://github.com/varunasthana92/Kuka_Arm_Inverse_Kinematics/blob/master/images/weld_simulation.gif" width = 1000>
</p>

## Dependencies
* MATLAB
* V-Rep

## Output
Inverse kinematics have been solved by using the principle of decoupling of inverse position and inverse orientation problem for a spherical wrist. Code will output all the possible configurations of the joint parameters to reach the target position and orientation.

<p align="center">
	<img src="https://github.com/varunasthana92/Kuka_Arm_Inverse_Kinematics/blob/master/images/multiple_solution.gif" width = 1000><br>
	Multiple Solutions for joint angles<br>
</p>

<p align="center">
	<img src="https://github.com/varunasthana92/Kuka_Arm_Inverse_Kinematics/blob/master/images/matlab_output.png" width = 1000><br>
	MATLAB code output
	<br>
</p>

"kuka_vrep.ttt" is to be imported in the V-Rep to setup the simulatioon environemnet. Computed joint angles can then be used in the script file to visulaize the simulation. Trajectory waypoints have been used to do the welding simulation.  

For more details on how to construct the Denavit–Hartenberg table and the mathematical equations used, refer the "Report.pdf"

## Contact information
Name: Varun Asthana  
Email id: varunasthana92@gmail.com