

inverse_kinematics(count,Motion_msg):
    Error = Pose2D() #error from desired point
    Error.x = x_goals[Count]-aruco_msg.x
	Error.y = y_goals[Count]-aruco_msg.y
    Error.theta = theta_goals[Count]-aruco_msg.theta

    if abs(Error.theta)<0.05  and abs(Error.x)<3 and abs(Error.y)<3:
			Motion_msg.x= 0
			Motion_msg.y= 0
			Motion_msg.z= 0
            output(Motion_msg)
			rospy.sleep(1)
			print(Count)
			if Count< (Limit-1):
				Count+=1
    else:
        x = (Error.x*math.cos(aruco_msg.theta) + Error.y*math.sin(aruco_msg.theta))
	    y = (-Error.x*math.sin(aruco_msg.theta) + Error.y*math.cos(aruco_msg.theta))

	    motion_msg.x= (2*x/3-Error.theta*5)*0.5
	    motion_msg.y= (-x/3+y/math.sqrt(3)-Error.theta*5)*0.5
	    motion_msg.z = (-x/3-y/math.sqrt(3)-Error.theta*5)*0.5
        output(Motion_msg)



		Error.x = x_goals[Count]-aruco_msg.x
		Error.y = y_goals[Count]-aruco_msg.y
		Error.theta = theta_goals[Count]-aruco_msg.theta

		if abs(Error.theta)<0.05  and abs(Error.x)<3 and abs(Error.y)<3:
			Motion_msg.x= 0
			Motion_msg.y= 0
			Motion_msg.z= 0
			output(Motion_msg)
			rospy.sleep(1)
			print(Count)
			if Count< (Limit-1):
				Count+=1

		else:
			Motion_msg = inverse_kinematics(Error,Motion_msg)

		print(f"{Motion_msg}")
		output(Motion_msg)

def inverse_kinematics(Error,motion_msg):

	x = (Error.x*math.cos(aruco_msg.theta) + Error.y*math.sin(aruco_msg.theta))
	y = (-Error.x*math.sin(aruco_msg.theta) + Error.y*math.cos(aruco_msg.theta))

	motion_msg.x= (2*x/3-Error.theta*5)*0.5
	motion_msg.y= (-x/3+y/math.sqrt(3)-Error.theta*5)*0.5
	motion_msg.z = (-x/3-y/math.sqrt(3)-Error.theta*5)*0.5
	return motion_msg
