#!/usr/bin/env python


import rospy
import brickpi3
import numpy as np
import math
import time
from std_msgs.msg import String, Int16
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class

# Number of the Joints of the robot
nrOfJoints = 3

qNow=np.array([0.0] * nrOfJoints) # This will be our joints state, global access in this script
# This int can be used to check wether there are pending requests to be attended by the planner or not
pendantRequest = 0

# Joint order according to the motor connected to BrickPi3 ports
jointsOrder = [BP.PORT_C, BP.PORT_A, BP.PORT_B]

#Conversion from radians to degree of motors. Calculated considering the gear ratio
jointsScale = np.array([7*180/math.pi, 3*180/math.pi, 1.6*180/math.pi])

# callbackPlanner(data) This function is triggered when a ROS message is being received while in ratePendant.sleep()
# This function takes what the trajectory planner sends, according to the requests you do from this python code
# Also, it uses that information to command the motors, iteratively until they reach the jointPoints given a degrees precision and advances to the next
def callbackPlanner(data):
	global BP # We need to specify the globals to be modified
	global qNow
	global pendantRequest
rospy.loginfo(data)

# Degrees of precision of the motors
degreesThreshold = np.array([5, 5, 5])
#kp ki and kd are proportional, integral and derivative constants for control respectively
kp = [30] * len(data.points[0].positions)
ki = [20] * len(data.points[0].positions)
kd = [30] * len(data.points[0].positions)
h = len(data.points) # h contains the number of jointPoints
w = len(data.points[0].positions) # w contains number of joints
Matrix = [[0 for x in range(w)] for y in range(h)]

for idy, point in enumerate(data.points):
		# Flag for waiting until motors reach each point
		allReached = False
		# This array of flags indicates that each single joint arrived to the destination
		qReached=[False] * len(data.points[0].positions)
		timeBase= time.time()
while not allReached:
    if rospy.is_shutdown():
        BP.reset_all()
        return
    for idx, jointPos in enumerate(point.positions):
        rospy.loginfo("Joint %s position to send %s", idx, jointPos*jointsScale[idx])
        rospy.loginfo("Joint 1 %s, Joint 2 %s , Joint 3 %s", BP.get_motor_encoder(BP.PORT_C), BP.get_motor_encoder(BP.PORT_A), BP.get_motor_encoder(BP.PORT_B))

#Tell the motors to move to the desired position
K = kp[idx] + ki[idx] * (time.time()-timeBase)
if K > 100: K = 100
    BP.set_motor_position_kp(jointsOrder[idx], K) # We can cheat the control and add an integrator varying kp over time
    BP.set_motor_position_kd(jointsOrder[idx], kd[idx])
    BP.set_motor_position(jointsOrder[idx], jointPos*jointsScale[idx]) # Note how jointsScale is used!
    for idx, jointPort in enumerate(jointsOrder):
        if abs(BP.get_motor_encoder(jointPort) - int(round(data.points[idy].positions[idx]*jointsScale[idx]))) < degreesThreshold[idx]:
            qReached[idx]=True
        if all(flag == True for flag in qReached):
            allReached=True

# Now the request has been attended, let's refresh qNow values
	for idx, q in enumerate(jointsOrder):
	    qNow[idx] = float(BP.get_motor_encoder(q))/float(jointsScale[idx])
	#Request has been attended, success!
	pendantRequest = pendantRequest - 1
	rospy.loginfo("Request attended")

# ROS publisher and subscriber
rospy.init_node('task', anonymous=True)
ratePendant = rospy.Rate(0.5) #Hz
rateLoop = rospy.Rate(100) #Hz
pub = rospy.Publisher('request', String, queue_size=100)
subP = rospy.Subscriber('trajectory', JointTrajectory, callbackPlanner)

# Callback function for subscribing to matlab request
# determine which cellphone was requested
# send the requests to planner.cpp
# update the status of the phone
def callbackPhoneRequest(data):
	rospy.loginfo(data);
	phoneModel = data.data;
	if phoneModel == 1:
		drawPhone1();
	elif phoneModel == 2:
		drawPhone2();
	elif phoneModel == 3:
		drawPhone3();
	else:
		rospy.loginfo("unknown model: %s", phoneModel);

	rospy.loginfo("order processed")

subN = rospy.Subscriber('number', Int16, callbackPhoneRequest)

# This function will draw the frame of the robot
def drawFrame(x0, y0, z0):

    xyz=np.array([x0, y0, z0])
    eulerzyx=np.array([0.0, 0.0, 0.0]) # Don't care about rotation now
    sendRequest('J', qNow, xyz, eulerzyx, 1) #In theory one point for joint trajs is ok
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz=np.array([x0 + 50, y0, z0])
    sendRequest('L', qNow, xyz, eulerzyx, 20) #The most points the most accurate it should be
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz=np.array([x0 + 50, y0, z0 - 63])
    sendRequest('L', qNow, xyz, eulerzyx, 20) #The most points the most accurate it should be
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz=np.array([x0, y0, z0 - 63])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz=np.array([x0, y0, z0])
    eulerzyx=np.array([0.0, 0.0, 0.0]) # Don't care about rotation now
    sendRequest('L', qNow, xyz, eulerzyx, 1) #In theory one point for joint trajs is ok
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

def drawPhone1():
	#frame's top left corner
	x0 = -20;
	y0 = 250;
	z0 = 350;

	#position above the paper to move the pen to
	yOffset = 220;

# This function will draw the frame of the mobile
	drawFrame(x0, y0, z0);

	eulerzyx=np.array([0.0, 0.0, 0.0])

	# The frame is drawn, take it above the second rectangle
        xyz=np.array([x0 + 10, yOffset, z0 - 7])
        sendRequest('J', qNow, xyz, eulerzyx, 1)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

	#SCREEN
        xyz=np.array([x0 + 10, y0, z0 - 7])
        sendRequest('J', qNow, xyz, eulerzyx, 1)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 35, y0, z0 - 7])
        sendRequest('L', qNow, xyz, eulerzyx, 10)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 35, y0, z0 - 21])
        sendRequest('L', qNow, xyz, eulerzyx, 10)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 10, y0, z0 - 21])
        sendRequest('L', qNow, xyz, eulerzyx, 10)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 10, y0, z0 - 7])
        sendRequest('L', qNow, xyz, eulerzyx, 10)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

	# The screen is drawn, take the pen above the third rectangle
        xyz=np.array([x0 + 10, yOffset, z0 - 30])
        sendRequest('J', qNow, xyz, eulerzyx, 1)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

	#KEYBOARD
        xyz=np.array([x0 + 10, y0, z0 - 30])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 35, y0, z0 - 30])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 35, y0, z0 - 56])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 10, y0, z0 - 56])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 10, y0, z0 - 30])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

	# The keyboard is drawn, take the pen above the first vertical line
        xyz=np.array([x0 + 19, yOffset, z0 - 30])
        sendRequest('J', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

	# VERTICAL LINE 1
        xyz=np.array([x0 + 19, y0, z0 - 30])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 19, y0, z0 - 56])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

	# First vertical line is drawn take the pen above the second vertical line
        xyz=np.array([x0 + 26, yOffset, z0 - 30])
        sendRequest('J', qNow, xyz, eulerzyx, 1)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

	# VERTICAL LINE 2
        xyz=np.array([x0 + 26, y0, z0 - 30])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 26, y0, z0 - 56])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        # vertical line is drawn take the pen above the first horizontal line
        xyz=np.array([x0 + 10, yOffset, z0 - 39])
        sendRequest('J', qNow, xyz, eulerzyx, 1)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

	# HORIZONTAL LINE 1
        xyz=np.array([x0 + 10, y0, z0 - 39])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 35, y0, z0 - 39])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass


        # first horizontal line is drawn take the pen above the second horizontal line
        xyz=np.array([x0 + 10, yOffset, z0 - 48])
        sendRequest('J', qNow, xyz, eulerzyx, 1)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

	# HORIZONTAL LINE 2
        xyz=np.array([x0 + 10, y0, z0 - 48])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

        xyz=np.array([x0 + 35, y0, z0 - 48])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass


	# Drawing the phone is finished, go to initial location
        xyz=np.array([0, 220, 350])
        sendRequest('L', qNow, xyz, eulerzyx, 20)
        while pendantRequest > 0:
            ratePendant.sleep()
            pass

def drawPhone2():

# frame's top left corner
    x0 = -20;
    y0 = 250;
    z0 = 350;


# position above the paper to move the pen to
    yOffset = 220;

    drawFrame(x0, y0, z0);

    eulerzyx = np.array([0.0, 0.0, 0.0])

# The frame is drawn, take it above the second rectangle
    xyz = np.array([x0 + 10, yOffset, z0 - 7])
    sendRequest('J', qNow, xyz, eulerzyx, 1)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

# SCREEN
    xyz = np.array([x0 + 10, y0, z0 - 7])
    sendRequest('J', qNow, xyz, eulerzyx, 1)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 35, y0, z0 - 7])
    sendRequest('L', qNow, xyz, eulerzyx, 10)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 35, y0, z0 - 35])
    sendRequest('L', qNow, xyz, eulerzyx, 10)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 10, y0, z0 - 35])
    sendRequest('L', qNow, xyz, eulerzyx, 10)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 10, y0, z0 - 7])
    sendRequest('L', qNow, xyz, eulerzyx, 10)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

# The screen is drawn, take the pen above the third rectangle
    xyz = np.array([x0 + 10, yOffset, z0 - 30])
    sendRequest('J', qNow, xyz, eulerzyx, 1)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

# KEYBOARD
    xyz = np.array([x0 + 10, y0, z0 - 46])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 35, y0, z0 - 46])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 35, y0, z0 - 56])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 10, y0, z0 - 56])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 10, y0, z0 - 46])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

# The keyboard is drawn, take the pen above the first vertical line
    xyz = np.array([x0 + 35, yOffset, z0 - 46])
    sendRequest('J', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

# VERTICAL LINE 1
    xyz = np.array([x0 + 35, y0, z0 - 46])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 35, y0, z0 - 56])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

# First vertical line is drawn take the pen above the second vertical line
    xyz = np.array([x0 + 26, yOffset, z0 - 46])
    sendRequest('J', qNow, xyz, eulerzyx, 1)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

# VERTICAL LINE 2
    xyz = np.array([x0 + 26, y0, z0 - 46])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 26, y0, z0 - 56])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    # vertical line is drawn take the pen above the first horizontal line
    xyz = np.array([x0, yOffset, z0 - 38])
    sendRequest('J', qNow, xyz, eulerzyx, 1)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

        # HORIZONTAL LINE 1
    xyz = np.array([x0 + 10, y0, z0 - 38])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 35, y0, z0 - 38])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

# first horizontal line is drawn take the pen above the second horizontal line
    xyz = np.array([x0 + 10, yOffset, z0 - 48])
    sendRequest('J', qNow, xyz, eulerzyx, 1)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

# Drawing the phone is finished, go to initial location
    xyz = np.array([0, 220, 350])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

def drawPhone3():
    # frame's top left corner
    x0 = -20;
    y0 = 250;
    z0 = 350;

    # position above the paper to move the pen to
    yOffset = 220;

    drawFrame(x0, y0, z0);

    eulerzyx = np.array([0.0, 0.0, 0.0])

    # The frame is drawn, take it above the second rectangle
    xyz = np.array([x0 + 10, yOffset, z0 - 7])
    sendRequest('J', qNow, xyz, eulerzyx, 1)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    # SCREEN
    xyz = np.array([x0 + 10, y0, z0 - 7])
    sendRequest('J', qNow, xyz, eulerzyx, 1)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 35, y0, z0 - 7])
    sendRequest('L', qNow, xyz, eulerzyx, 10)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 35, y0, z0 - 35])
    sendRequest('L', qNow, xyz, eulerzyx, 10)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 10, y0, z0 - 35])
    sendRequest('L', qNow, xyz, eulerzyx, 10)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 10, y0, z0 - 7])
    sendRequest('L', qNow, xyz, eulerzyx, 10)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    # The screen is drawn, take the pen above the third rectangle
    xyz = np.array([x0 + 10, yOffset, z0 - 30])
    sendRequest('J', qNow, xyz, eulerzyx, 1)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    # KEYBOARD
    xyz = np.array([x0 + 10, y0, z0 - 46])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 35, y0, z0 - 46])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 35, y0, z0 - 56])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 10, y0, z0 - 56])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 10, y0, z0 - 46])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    # The keyboard is drawn, take the pen above the first vertical line
    xyz = np.array([x0 + 35, yOffset, z0 - 46])
    sendRequest('J', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    # VERTICAL LINE 1
    xyz = np.array([x0 + 35, y0, z0 - 46])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 35, y0, z0 - 56])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    # First vertical line is drawn take the pen above the second vertical line
    xyz = np.array([x0 + 26, yOffset, z0 - 46])
    sendRequest('J', qNow, xyz, eulerzyx, 1)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    # VERTICAL LINE 2
    xyz = np.array([x0 + 26, y0, z0 - 46])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 26, y0, z0 - 56])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    # SMALLER FRAME
    xyz = np.array([x0 + 5, y0, z0 - 5])
    eulerzyx = np.array([0.0, 0.0, 0.0])  # Don't care about rotation now
    sendRequest('J', qNow, xyz, eulerzyx,
                1)  # In theory one point for joint trajs is ok
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 45, y0, z0])
    sendRequest('L', qNow, xyz, eulerzyx,
                20)  # The most points the most accurate it should be
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 45, y0, z0 - 58])
    sendRequest('L', qNow, xyz, eulerzyx,
                20)  # The most points the most accurate it should be
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0, y0, z0 - 58])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    xyz = np.array([x0 + 5, y0, z0 - 5])
    eulerzyx = np.array([0.0, 0.0, 0.0])  # Don't care about rotation now
    sendRequest('L', qNow, xyz, eulerzyx,
                1)  # In theory one point for joint trajs is ok
    while pendantRequest > 0:
        ratePendant.sleep()
        pass

    # Drawing the phone is finished, go to initial location
    xyz = np.array([0, 220, 350])
    sendRequest('L', qNow, xyz, eulerzyx, 20)
    while pendantRequest > 0:
        ratePendant.sleep()
        pass



    # This function sends the ros message (your request)
	#Format of the request :
	# A char , L or J for traj type, "space"
	# Current state of the joints, separated by commas
	# Target position in world frame, separated by commas "space"
	#rotation in EulerZYX, separated by commas "space"
	# Number of points to interpolate
def sendRequest(typeTraj, qNow, xyz, eulerzyx, points = 1):
	global pendantRequest
	# Trajectory type, pass it as a
	request_str = typeTraj + ' '
	# QNow, pass it as a numpy array
	for q in qNow:
		request_str = request_str + str(q) + ','
	request_str = request_str[:-1] + ' ' # Remove last comma, add space for next word
	# XYZ, pass it as a numpy array
	for u in xyz:
		request_str = request_str + str(u) + ','
	request_str = request_str[:-1] + ' '
	# EulerZYX, pass it as a numpy array
	for a in eulerzyx:
		request_str = request_str + str(a) + ','
	request_str = request_str[:-1] + ' '
	# joint Points to be returned by the planner, pass it as an int number
	request_str = request_str + str(points)
	# Send the request_str
	pub.publish(request_str)
	# If we send request, it is pendant to be answered
	pendantRequest = pendantRequest + 1

# This is the main function
def task():
        global BP #Initialize the state of our joints, let's them be global to be refreshed on every movement
        #This variable should have their value in radians or as interpreted by your model in the planner

        # TO DO. Student code should start here:
        BP.reset_all() #Ensure the moveposset_motor_positionition previous command is not working
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) #Reset encoder A
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) #Reset encoder B
        BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) #Reset encoder C
        BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) #Reset encoder C
        # Tell the motors not to be in a hurry, slowly is more accurate
        BP.set_motor_limits(BP.PORT_C, 60, 90)
        BP.set_motor_limits(BP.PORT_A, 20, 90)
        BP.set_motor_limits(BP.PORT_B, 30, 90)


        ratePendant.sleep() # Wait a while before requesting

        while not rospy.is_shutdown():
                ratePendant.sleep()
                rospy.loginfo("Pendant requests: %s", pendantRequest)



if __name__ == '__main__':
	try:
		task()
	except (rospy.ROSInterruptException, KeyboardInterrupt):
                pass
        finally:
                rospy.loginfo("KeyboardInterrupt received")
                BP.reset_all()

