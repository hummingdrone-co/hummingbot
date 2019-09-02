import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

CAMERA_TOPIC_NAME = '/gazebo_client/hummingdrone_camera'

def callback(img):
    """Get the image from Gazebo and convert to openCV image format using CvBridgea"""
    #Debug
    #rospy.loginfo('Message received: \n%s' % (img.encoding))

    try:
        frame = bridge.imgmsg_to_cv2(img, 'bgr8')
    except CvBridgeError, e:
        print e

    # Convert the image to a Numpy array since most cv2 functions
    # require Numpy arrays.
    frame = np.array(frame, dtype=np.uint8)

    #Debug: Print Shape and size as byte
    rospy.loginfo(frame.shape)
    #rospy.loginfo(len(x.data))

    #Show image
    cv2.imshow('a', frame)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('rover_camera_receiver')

    rospy.loginfo('Camera receiver is running...')

    cv2.namedWindow("Image Window", 1)
    bridge = CvBridge()

    sub = rospy.Subscriber(CAMERA_TOPIC_NAME, Image, callback)

    rospy.spin()
