## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/kinematicArm
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png

### Kinematic Analysis
The kuka arm to be analyzed looks as follows: 
![alt text][image1]

This images results in the following DH table:

i | Alpha | A | D | Theta
--- | --- | --- | --- | ---
1 | 0| 0| 0.75 | Q1
2 | -PI/2| 0.35| 0| Q2- PI/2
3 | 0| 1.25|0|Q3
4 | -PI/2|-0.054| 1.50|Q4
5 | PI/2| 0|0|Q5
6 | -PI/2|  0|   0|Q6
7(Gripper) | 0|   0| 0.303|0

This table directly translates into code written as such: 
```python
s = {alpha0:      0, a0:       0, d1:  0.75,
     alpha1:  -np.pi/2, a1:    0.35, d2:     0, q2: q2-np.pi/2,
     alpha2:      0, a2:    1.25, d3:     0,
     alpha3:  -np.pi/2, a3:  -0.054, d4:  1.50,
     alpha4:   np.pi/2, a4:       0, d5:     0,
     alpha5:  -np.pi/2, a5:       0, d6:     0,
     alpha6:      0, a6:       0, d7: 0.303, q7: 0}

````

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.



