import cv2

cap= cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')

width= int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height= int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

saveVideo = "False"
takePicture = "True"

if saveVideo == True:
    writer= cv2.VideoWriter('basicvideo.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width,height))

i = 0
while True:
    
    ret,frame= cap.read()

    if saveVideo == True:
       writer.write(frame)

    cv2.imshow('frame', frame)

    cv2.waitKey(1)
    
    if cv2.waitKey(1) & 0xFF == ord('p'):
        picture = frame
        print("Taking picture")
        print(i)
        im_dir = "Camera A8 Mini/camera_calibration/"
        im_name = "Image_" + str(i) + ".png"
        cv2.imshow('picture', picture)
        cv2.imwrite(im_dir+im_name, picture)
        print("Picture saved")
        i = i + 1
        ret, frame = cap.read()

    # print(i)



cap.release()
if saveVideo == True:
    writer.release()
cv2.destroyAllWindows()