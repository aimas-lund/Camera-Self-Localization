import cv2
cap = cv2.VideoCapture("rtsp://192.168.144.25:8554/main.264")
i = 0
while(cap.isOpened()):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    i = i + 1
    print(i)
    if cv2.waitKey(20) & 0xFF == ord('q'):
        print("no cap")
        break
cap.release()
cv2.destroyAllWindows()