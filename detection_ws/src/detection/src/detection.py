#!/usr/bin/env python

import tensorflow as tf
import tensorflow_hub as hub
import cv2
from cv2 import *
from matplotlib import pyplot as plt
import numpy as np

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker

callback_working = False

marker = Marker()
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

# Initialize the CvBridge class
bridge = CvBridge()

# Load model
model = hub.load('detection_model/archive')
movenet = model.signatures['serving_default']

EDGES = {
  (0, 1): 'm',
  (0, 2): 'c',
  (1, 3): 'm',
  (2, 4): 'c',
  (0, 5): 'm',
  (0, 6): 'c',
  (5, 7): 'm',
  (7, 9): 'm',
  (6, 8): 'c',
  (8, 10): 'c',
  (5, 6): 'y',
  (5, 11): 'm',
  (6, 12): 'c',
  (11, 12): 'y',
  (11, 13): 'm',
  (13, 15): 'm',
  (12, 14): 'c',
  (14, 16): 'c'
}
    
def draw_keypoints(frame, keypoints, confidence_threshold):
  y, x, c = frame.shape
  shaped = np.squeeze(np.multiply(keypoints, [y,x,1]))

  for kp in shaped:
    ky, kx, kp_conf = kp
    if kp_conf > confidence_threshold:
	    cv2.circle(frame, (int(kx), int(ky)), 6, (0,255,0), -1)

def draw_connections(frame, keypoints, edges, confidence_threshold):
  y, x, c = frame.shape
  shaped = np.squeeze(np.multiply(keypoints, [y,x,1]))

  for edge, color in edges.items():
    p1, p2 = edge
    y1, x1, c1 = shaped[p1]
    y2, x2, c2 = shaped[p2]

    if (c1 > confidence_threshold) & (c2 > confidence_threshold):
	    cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,0,255), 4)

def loop_through_people(frame, keypoints_with_scores, edges, confidence_threshold):
  global marker
  marker.color.r = 0.0
  marker.color.g = 0.0
  marker.color.b = 0.0
  marker.color.a = 0.3
  marker.scale.x = 0.2
  marker.scale.y = 0.2
  marker.scale.z = 0.2
  marker.text = "NO PERSON"
  rospy.loginfo("")
  i = 0
  for person in keypoints_with_scores:
    draw_connections(frame, person, edges, confidence_threshold)
    draw_keypoints(frame, person, confidence_threshold)
    i += 1
    confidence = np.mean(person, axis=0)[2]
    if confidence > 0.3:
      marker.color.r = 1.0
      marker.color.g = 0.0
      marker.color.b = 0.0
      marker.color.a = 0.3
      marker.scale.x = 1.0
      marker.scale.y = 1.0
      marker.scale.z = 1.0
      marker.text = "PERSON"
      rospy.loginfo("Person " + str(i) + ":" + str(confidence) + " IS PERSON")
    else:
      rospy.loginfo("Person " + str(i) + ":" + str(confidence))
  
  marker_pub.publish(marker)
  #hier senden des signals schreiben

# Define a callback for the Image message
def image_callback(img_msg):
  global callback_working
  if not callback_working:
    callback_working = True
    # log some info about the image topic
    #rospy.loginfo(img_msg.header)

    try:
      frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
      rospy.logerr("CvBridge Error: {0}".format(e))# copy from
    # classify_image.py
    #image_data = cv2.imencode('.jpg', cv_image)[1].tostring()

    img = frame.copy()
    img = tf.image.resize_with_pad(tf.expand_dims(img, axis=0), 384,640)
    input_img = tf.cast(img, dtype=tf.int32)

    results = movenet(input_img)
    keypoints_with_scores = results['output_0'].numpy()[:,:,:51].reshape((6,17,3))

    loop_through_people(frame, keypoints_with_scores, EDGES, 0.5)

    cv2.imshow('Movenet Multipose', frame)
    cv2.waitKey(3)
    callback_working = False

def detection():
    global marker

    # Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
    rospy.init_node('opencv_example', anonymous=True)

    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 2
    marker.id = 0
    
    # Set the scale of the marker
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    
    # Print "Hello ROS!" to the Terminal and ROSLOG
    rospy.loginfo("Hello ROS!")

    #model = hub.load('https://tfhub.dev/google/movenet/multipose/lightning/1')
    #model = tf.loadLayersModel('saved_model.pb');

    #0 durch robomaster kamera ersetzen
    #cap = cv2.VideoCapture(0)
    
    # Initalize a subscriber to the "/camera/image_raw" topic with the function "image_callback" as a callback
    callback_working = False
    sub_image = rospy.Subscriber("/camera/image_raw", Image, image_callback)

    #while cap.isOpened():
    #  if cv2.waitKey(10) & 0xFF==ord('q'):
    #  break
    
    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
      rospy.spin()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detection()
    except rospy.ROSInterruptException:
        pass
