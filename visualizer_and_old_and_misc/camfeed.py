import cv2
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
height=1080 / 2 #for some reason, ANYTHING else works for my HD camera for example 1079..
width=1920 / 2
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
if not cap.isOpened():
    print("Error: Could not open the camera.")
    exit()

while True:
    ret, frame = cap.read()
    cv2.imshow("main", frame)
    cv2.waitKey(1)