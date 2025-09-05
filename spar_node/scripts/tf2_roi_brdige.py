#!/usr/bin/env python3

import math

import rospy
import tf2_ros
from std_msgs.msg import Time
from geometry_msgs.msg import TransformStamped
from vision_msgs.msg import ObjectHypothesisWithPose
from std_msgs.msg import String

# Global Variables
tfbr = None
pub_found = None

# From tf2_listener 
tfBuffer = None
tfln = None
pub_roi = None

camera_name = "camera"
target_name = "target"

# Gobal variable 
time_found = 0

# Step 5: running this function 
def send_tf_target(transform_coor):
	# Step 6: Generate our "found" timestamp
	global time_found
	time_found = rospy.Time.now()

	# Step 7: Create a transform arbitrarily in the camera frame
	t = TransformStamped()
	t.header.stamp = time_found
	t.header.frame_id = camera_name
	t.child_frame_id = target_name

    # Step 8: Once we know where the target is, relative to the camera frame, we create and sent that transform (relative position target to camera)
	t.transform.translation.x = transform_coor.translation.x
	t.transform.translation.y = transform_coor.translation.y
	t.transform.translation.z = transform_coor.translation.z
	t.transform.rotation.x = 0.0
	t.transform.rotation.y = 0.0
	t.transform.rotation.z = 0.0
	t.transform.rotation.w = 1.0

	# Step 9: Send the transformation to TF and "found" timestamp to localiser
	tfbr.sendTransform(t)
	#pub_found.publish(time_found)


	return t

def callback_target_found(msg_in): 
	pass

# Step 2 calling callback function for solve_pnp subscriber 
def camera_callback(data):

	try: 
		# Step 3: Extracting pose data from message  
		#pose_local = data.pose.pose

		marker_id = int(data.child_frame_id)

		transform_coor = data.transform

		# Step 4: calling tf2_target function and passing through pose data from solve_pnp
		# Calling the transform function and passing through pose data
		target_tf = send_tf_target(transform_coor)

		# Step 10: calling timestamp from above function 
		global time_found 

		# Step 11: Lookup transform from "map" to "target" at time "msg_in.data",
		# and allow for 0.5 seconds to collected any additionally needed data
		t = tfBuffer.lookup_transform("map", "target", time_found, rospy.Duration(0.5))

		# Step 12: (next three rospy.loginfo lines)
		# Dump information to screen
		rospy.loginfo("Found target at the following location in the world:")
		rospy.loginfo("Found ArUco Marker: {}".format(marker_id))

        # This is the infromation that we want for the ROI.
		rospy.loginfo("[x: %0.2f; y: %0.2f; z: %0.2f]" % (t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z))

		rospy.loginfo("Sending ROI to target found coordinates...")
		
		# Step 13: Extracting transform coordinates 
		detection = ObjectHypothesisWithPose()
		#detection = data
		detection.id = marker_id
		detection.pose.pose.position.x = t.transform.translation.x
		detection.pose.pose.position.y = t.transform.translation.y
		detection.pose.pose.position.z = t.transform.translation.z

		# Step 14a: publishing coordinates 
		pub_detection.publish(detection)
		rospy.loginfo("ROI coordinates sent.")

		# Step 14b: Creating the message for GCS espeak
		speaker = String()
		speaker.data = "[id: %d; x: %0.2f; y: %0.2f]" % (marker_id, t.transform.translation.x,
                            t.transform.translation.y)
		
		# Publishing to GCS for espeak
		pub_speaker.publish(speaker)

	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
		rospy.logwarn(e)


if __name__ == '__main__':
	rospy.init_node('tf2_combined_v2')
	rospy.loginfo("tf2_combined_v2 sending target found...")

	# Setup tf2 broadcaster and timestamp publisher
	tfbr = tf2_ros.TransformBroadcaster()

	# Create a listener
    # This catches all messages sent using TF2
	tfBuffer = tf2_ros.Buffer()
	tfln = tf2_ros.TransformListener(tfBuffer)

	# Step 1: Subscriber for solve_pnp camera pose estimate 
	#camera_pose = rospy.Subscriber('/camera/pose', ObjectHypothesisWithPose, camera_callback, queue_size=2)
	camera_pose = rospy.Subscriber('/camera/pose', TransformStamped, camera_callback, queue_size=20)
	rospy.loginfo("Camera Pose received")

	# Publisher for ROI to demo_wp_roi
	pub_detection = rospy.Publisher('/target_detection/roi', ObjectHypothesisWithPose, queue_size=10)
	rospy.loginfo("Camera Pose passed to Waypoint Scripts")

	# Publisher for epeak to GCS
	pub_speaker = rospy.Publisher('spoken_text', String, queue_size=10)
	rospy.loginfo("Detection information passed to espeak")

	# Give the nodes a few seconds to configure
	rospy.sleep(rospy.Duration(2))

	# Send out our target messages
	# send_tf_target()

	# Keeps the script always on to always be ready to receive data 
	rospy.spin()

	# Give the nodes a few seconds to transmit data
	# then we can exit
	rospy.sleep(rospy.Duration(2))
	rospy.loginfo("tf2_combined completed")
