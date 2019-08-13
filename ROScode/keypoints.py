import numpy as np
import cv2
from matplotlib import pyplot as plt

camL = cv2.VideoCapture(1)
camR = cv2.VideoCapture(0)
while True:
    ret,imgL = camL.read()
    retR,imgR = camR.read()

    if(ret and retR):
        imgL=cv2.resize(imgL,((640,360)))
        imgR=cv2.resize(imgR,((640,360)))
        #img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Initiate FAST object with default values
        fast = cv2.FastFeatureDetector_create(threshold=25)
        # find and draw the keypoints
        kpL = fast.detect(imgL,None)
        img2L = cv2.drawKeypoints(imgL, kpL, None,color=(255,0,0))
        kpR = fast.detect(imgR,None)
        img2R = cv2.drawKeypoints(imgR, kpR, None,color=(255,0,0))
        LRconcat = np.concatenate((img2L,img2R), axis=1)
        cv2.imshow('Keypoint Tracking', LRconcat)
        ch = cv2.waitKey(5)
        if ch == 27:
        #cv.imwrite("Left.jpg",imgL)
        #cv.imwrite("Right.jpg",imgR)
            break

camL.release()
camR.release()

# BFMatcher with default params
bf = cv2.BFMatcher()
matches = bf.knnMatch(imgL, imgR, k=2)


good = []
for m,n in matches:
    if m.distance < 0.3 * n.distance:
        good.append(m)

# Featured matched keypoints from images 1 and 2
pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
pts2 = np.float32([kp2[m.trainIdx].pt for m in good])

# Convert x, y coordinates into complex numbers
# so that the distances are much easier to compute
z1 = np.array([[complex(c[0],c[1]) for c in pts1]])
z2 = np.array([[complex(c[0],c[1]) for c in pts2]])

# Computes the intradistances between keypoints for each image
KP_dist1 = abs(z1.T - z1)
KP_dist2 = abs(z2.T - z2)

# Distance between featured matched keypoints
FM_dist = abs(z2 - z1)
