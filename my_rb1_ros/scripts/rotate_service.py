#!/usr/bin/env python

import rospy
import math
from my_rb1_ros.srv import Rotate, RotateResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from tf.transformations import quaternion_multiply, quaternion_inverse

# Global variable to store current robot orientation as a quaternion
current_orientation = Quaternion()

# ✅ Convert Quaternion to List Format for tf.transformations
def quaternion_to_list(quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

# ✅ Odometry Callback: Updates `current_orientation`
def odometry_callback(msg):
    global current_orientation
    current_orientation = msg.pose.pose.orientation

# ✅ Compute Target Orientation Using Quaternion Multiplication
def compute_target_quaternion(target_angle):
    global current_orientation
    
    # ✅ Convert target angle to radians
    theta = math.radians(target_angle) / 2.0  # Half angle for quaternion
    
    # ✅ Create Rotation Quaternion for Z-axis rotation
    q_rotate = [0, 0, math.sin(theta), math.cos(theta)]
    
    # ✅ Convert current quaternion to list
    q_current = quaternion_to_list(current_orientation)
    
    # ✅ Multiply: Q_target = Q_rotate * Q_current
    q_target = quaternion_multiply(q_rotate, q_current)
    
    return q_target

# ✅ Rotate Robot Until Reaching Target Quaternion
def rotate_robot(target_angle):
    global current_orientation
    move_cmd = Twist()
    
    # ✅ Compute Target Quaternion
    q_target = compute_target_quaternion(target_angle)

    rospy.loginfo(f"🔄 Rotating {target_angle}° | Target Quaternion: {q_target}")

    rate = rospy.Rate(10)  # 10Hz control loop
    while not rospy.is_shutdown():
        # ✅ Convert current orientation to list
        q_current = quaternion_to_list(current_orientation)

        # ✅ Compute difference quaternion: Q_diff = Q_target * Q_current⁻¹
        q_diff = quaternion_multiply(q_target, quaternion_inverse(q_current))

        # ✅ Extract rotation difference (Z component)
        error = 2 * math.atan2(q_diff[2], q_diff[3])  # Extract yaw error

        # ✅ Stop if within threshold (~1°)
        if abs(math.degrees(error)) < 1:
            break

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.5 if error > 0 else -0.5  # ✅ Adjust rotation direction
        my_pub.publish(move_cmd)

        rospy.loginfo(f"⏳ Rotating... Quaternion Error: {q_diff} | Yaw Error: {math.degrees(error):.2f}°")
        rate.sleep()

    # ✅ Stop the robot once rotation is complete
    move_cmd.angular.z = 0.0
    my_pub.publish(move_cmd)
    rospy.loginfo("✅ Rotation Completed!")

# ✅ Service Callback: Handles Rotation Requests
def service_callback(request):
    try:
        # ✅ Validate input: Must be a number within -360° to 360°
        if not isinstance(request.degrees, (int, float)):
            rospy.logerr("❌ Invalid input: Angle must be a number.")
            return RotateResponse("Error: Invalid input. Angle must be a number.")

        if abs(request.degrees) >= 360:
            rospy.logerr("❌ Invalid input: Angle must be between -360 and 360 degrees.")
            return RotateResponse("Error: Input value must be smaller than 360°.")

        rospy.loginfo(f"🛠️ Service /rotate_robot called with {request.degrees}°")

        # ✅ Start rotation and track time
        start_time = rospy.Time.now()
        success = rotate_robot(request.degrees)

        # ✅ Timeout Handling: If rotation takes too long (e.g., 20 sec)
        if (rospy.Time.now() - start_time).to_sec() > 20:
            rospy.logerr("❌ Timeout: Rotation took too long.")
            return RotateResponse("Error: Rotation timeout exceeded.")

        # ✅ Check if rotation function returned success
        # if not success:
        #     rospy.logerr("❌ Rotation failed due to odometry issues.")
        #     return RotateResponse("Error: Rotation failed (Odometry issue).")

        rospy.loginfo("✅ Rotation Completed Successfully!")
        return RotateResponse("Rotation completed successfully!")

    except Exception as e:
        rospy.logerr(f"❌ Service failed due to exception: {str(e)}")
        return RotateResponse(f"Error: {str(e)}")


# ✅ Initialize ROS Node & Setup Service
rospy.init_node('rotate_robot_service')
rospy.Service("/rotate_robot", Rotate, service_callback)
my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.Subscriber("/odom", Odometry, odometry_callback)

rospy.loginfo("✅ Rotate Robot Service Ready!")
rospy.spin()  # Keep service running
