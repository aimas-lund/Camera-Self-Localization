import cv2

vcap = cv2.VideoCapture("rtsp://192.168.144.25:8554/main.264")

while(1):

    ret, frame = vcap.read()
    cv2.imshow('VIDEO', frame)
    cv2.waitKey(1)