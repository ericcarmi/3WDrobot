import cv2
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

imagepub = rospy.Publisher("/output/image_raw/compressedLeft", CompressedImage, queue_size=10)
rospy.init_node("leftcamera",anonymous=True)

cameraL = cv2.VideoCapture(0)
cameraR = cv2.VideoCapture(1)

ret,imgL = cameraL.read()
retR,imgR = cameraR.read()

if(ret):
    msgL = CompressedImage()
    msgL.format = "jpeg"
    msgL.data = np.array(cv2.imencode('.jpg', imgL)[1]).tostring()
    imagepub.publish(msgL)
if(retR):
    msgR = CompressedImage()
    msgR.format = "jpeg"
    msgR.data = np.array(cv2.imencode('.jpg', imgR)[1]).tostring()
    imagepubR.publish(msgR)
