#!/usr/bin/env python
import cv2
import numpy as np

def list_ports():
    """
    Test the ports and returns a tuple with the available ports
    and the ones that are working.
    """
    is_working = True
    dev_port = 0
    working_ports = []
    available_ports = []
    while is_working:
        camera = cv2.VideoCapture(dev_port)
        if not camera.isOpened():
            is_working = False
            print("Port %s is not working." %dev_port)
        else:
            is_reading, img = camera.read()
            w = camera.get(3)
            h = camera.get(4)
            if is_reading:
                print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                working_ports.append(dev_port)
            else:
                print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                available_ports.append(dev_port)
        dev_port +=1
    return available_ports,working_ports

#print(list_ports())

cv2.namedWindow('image')
cv2.createTrackbar('HMin', 'image', 0, 179, lambda x: None)  # Hue is from 0-179 for OpenCV
cv2.createTrackbar('SMin', 'image', 0, 255, lambda x: None)
cv2.createTrackbar('VMin', 'image', 0, 255, lambda x: None)
cv2.createTrackbar('HMax', 'image', 0, 179, lambda x: None)
cv2.createTrackbar('SMax', 'image', 0, 255, lambda x: None)
cv2.createTrackbar('VMax', 'image', 0, 255, lambda x: None)

# Set default value for MAX HSV trackbars.
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

# Initialize variables
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
while True:
    img, frame = cap.read()
    img = frame
    # Get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin', 'image')
    sMin = cv2.getTrackbarPos('SMin', 'image')
    vMin = cv2.getTrackbarPos('VMin', 'image')
    hMax = cv2.getTrackbarPos('HMax', 'image')
    sMax = cv2.getTrackbarPos('SMax', 'image')
    vMax = cv2.getTrackbarPos('VMax', 'image')
    # Set minimum and maximum HSV values for thresholding
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])
    # Convert the image to HSV and apply the threshold
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    output = cv2.bitwise_and(img, img, mask=mask)

    cv2.imshow('thresholded_img', output)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()

import cv2
