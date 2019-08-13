'''
This subscriber listens for a boolean trigger

Later, add other options for more than just regular images
Left/Right, Edges, Keypoints, Disparity
Or have toggle switches so they can be all updated at once, or any combo

'''

import rospy
from std_msgs.msg import Int8
import imagepublisher0
import imagepublisher1

def subcallback(data):
    if(data):
        img_trigger = True
    if(img_trigger): # Make option for left/right, only one at a time...
        imagepublisher0.pubimg()
        #imagepublisher1.pubimg()

def listener():
    rospy.init_node('imagesub',anonymous=True)
    rospy.Subscriber("imagetrigger",Int8, subcallback)
    rospy.spin()

if __name__ == '__main__':
    listener()
