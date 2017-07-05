## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/kinematicArm.PNG
[image2]: ./misc_images/theta1.PNG
[image3]: ./misc_images/theta2.PNG
[image4]: ./misc_images/working.png

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

```
After that, we must create transform matrices about each point:
```python
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])
    T2_3 = T2_3.subs(s)

    T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])
    T3_4 = T3_4.subs(s)

    T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])
    T4_5 = T4_5.subs(s)

    T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])
    T5_6 = T5_6.subs(s)

    T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                   0,                   0,            0,               1]])
    T6_7 = T6_7.subs(s)
```
Then we combine each of the transforms to go from 0 - 7(gripper):
```python
T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7
```
This "T0_7" represents the transform from the base_link to the gripper_link ... in theory. After some trial and error, it was brought to my attention that the URDF file that describes the arm does 2 final twists to the gripper that the table doesn't include! Therefore we must account for them: 

```python
# Correction for orientation difference between defintion of gripper link URDF v DH
# first rotate around z-axis by pi
R_z = Matrix([[cos(np.pi), -sin(np.pi), 0, 0],
             [sin(np.pi), cos(np.pi), 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])
# then rotate around y-axis by -pi/2
R_y = Matrix([[cos(-np.pi / 2), 0, sin(-np.pi / 2), 0],
             [0, 1, 0, 0],
             [-sin(-np.pi / 2), 0, cos(-np.pi / 2), 0],
             [0, 0, 0, 1]])
R_corr = R_z * R_y

```

Then applying the correction to the T0_7:

```python
#The final total homogenous transformation
T_Final = T0_7 * R_corr
```

This T_final represents the generic one with symbols that can be substituted with actual values when the time comes. 

So therefore, we need to come up with the transform using the end effector's position and the ROLL/PITCH/YAW.
Setting up the roll/pitch/yaw as such:

```python
 # To calculate for end effector
R_roll = Matrix([[1, 0, 0],
              [0, cos(roll), -sin(roll)],
              [0, sin(roll), cos(roll)]])

R_pitch = Matrix([[cos(pitch), 0, sin(pitch)],
               [0, 1, 0],
               [-sin(pitch), 0, cos(pitch)]])

R_yaw = Matrix([[cos(yaw), -sin(yaw), 0],
             [sin(yaw), cos(yaw), 0],
             [0, 0, 1]])
```
Then we subsitute in the values for roll/pitch/yaw given from the End Effector, after converting from quarternions:
```python
(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

#final rotation of end effector.
R_EE = (R_roll * R_pitch * R_yaw)
```
This finally gives us the rotational part of the solution. 

The last position is provided in just a coordinate of (x,y,z). The final steps break down into Inverse kinematics.


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

This problem can be broken down into 3 parts:
1.Finding the wrist
2.theta 1,2,3 
3.theta 4,5,6

##### Finding the wrist
The wrist is labelled as joint 5. This joint is 0.303 meters away from the end effector's position. There it was needed to take the rotation from the end effector and apply it to the end effector's position in the x,y,z dir. 
therefore:
wristPosition = (end_effectorPosition - (d6 + d7) * Rotation of end_effector)
```python
wx = (px - (d6 + d7) * R_EE[0,0]).subs(s)
wy = (py - (d6 + d7) * R_EE[1,0]).subs(s)
wz = (pz - (d6 + d7) * R_EE[2,0]).subs(s)
```
Now that the wrist is solved, you can use this to figure out the Inverse Position kinematics:
##### The Inverse Position Kinematics: Theta 1 2 3

Theta1/rotation around the base is the most simple theta to find.  An example given in the lecture for a RRP manipulator was provided, therefore the same rules were applied here:
```python
theta1 = atan2(wy, wx)
```
![Theta1][image2]

The real challenge came when calculating theta 2 and 3. After looking at various text books, I stumbled across a diagram that was critical to my understanding in solving these two angles. 

![Theta2][image3]

Along with reading the Slack channel and asking other students, turns out that there is a small misalingment in the robots arm after joint 4. This is the A of -0.053 in the DH parameters. The trig then worked itself out:
```python 
# Need to account for 0.054
internal_angle = atan2(wz - 1.94645, wx)
wx = wx - 0.054 * sin(internal_angle)
wz = wz + 0.054 * cos(internal_angle)
```
With this "recalculation" of the wrist, it is now aligned with the diagram above. Before this little addition, the wrist was always off by a little bit due to the crook in joint 4.
```python 
# Finding the distance from the origin to the newly slightly moved wrist center
wxdist = sqrt(wy*wy+wx*wx)

# grab the lengths for the segments to calculate theta 2/3 from DH table
l1 = s[a2]
l2 = s[d4]

# Moving the second joint to the origin to make the math cleaner later.
wxdist = wxdist - s[a1]
wzdist = wz - s[d1]

# Cosine law
D=(wxdist*wxdist + wzdist*wzdist - l1*l1-l2*l2)/(2*l1*l2)

# Clip d from going above 1
if (D>1):
 D=1

theta3 = atan2(-sqrt(1-D*D),D)

# equation Pulled from book
s1 = ((l1 + l2 * cos(theta3)) * wzdist - l2 * sin(theta3) * wxdist) / (wxdist * wxdist + wzdist * wzdist)
c1 = ((l1 + l2 * cos(theta3)) * wxdist + l2 * sin(theta3) * wzdist) / (wxdist * wxdist + wzdist * wzdist)
theta2 = atan2(s1, c1)

# Theta3 needs to be translated by 90 degrees
theta3 = -1*(theta3+pi/2)
theta2 = pi/2-theta2
```

##### The Inverse Position Kinematics: Theta 4 5 6

The final three sections' rotations are equal to the (first three rotations inversed) * (the entire transform). We now know the first three rotations from theta 1 2 3, abd we have a known R0_6.
Therefore to calculate the remaining 3 thetas, that formula gets you almost there:
```python
R3_6 = (R0_3)**-1 * R_EE[0:3,0:3]
```
Now convert that R3_6 into three euler angles that represent the robots last three angles. Using the function "euler_from_matrix" in the tf2 transformation library and the "ryzy" type of euler angle:
```python 
# Formula for taking the rotation to euler
(theta4, theta5, theta6) = tf.transformations.euler_from_matrix(np.array(R3_6[0:3,0:3]).astype(np.float64),"ryzx")


#Due to 0 case, translate theta 5 by 90 degrees
theta5 = (theta5 - np.pi/2)

# Sometimes theta 5 likes to go crazy and bump into itself.  This stops that.
if (theta5 > 2):
 theta5 = 2
if (theta5 < -2):
 theta5 = -2
#Due to 0 case, translate theta 6 by 90 degrees
theta6 = theta6 - np.pi/2
```
NOTE: theta 5 crashes into itself for some reason... so it was nessary to basically "clip" the angles in order for it to not do that.

### Results Discussion
The video of the arm can be see here: (not yet uploaded.)

Here is a sample picture of it after it grabbed an item and dropped it in the box. It is now in the process of grabbing the second item.

![working][image4]

It doesn't pick up every time, due to some wonky path planning that the project wants the arm to do. It also sometimes bumps into the bin, which is again, a problem with the path planning.

Also, I changed the .cpp file a little bit to allow the gripper to work better when in continuous mode on RViz. This allowed for testing to be smoother.

### Areas of improvement
Sometimes the arm arrives at the correct point, and in other cases, it comes from the side which makes it bump into the object when in the grasping state. It is very clear that the arm is in the right pose, therefore I believe it is not my inverse kinematics being the issue, but rather an issue with the motion planning algorithm. You can also see this type of behaviour in the demo mode as well.

If I had more time to impove the project, I would look into optimizing the code to make the arm smoother in its path planning, the Inverse kinematics to be smoother and have more efficient results.




