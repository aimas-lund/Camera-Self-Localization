import cv2

cap= cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264')

width= int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height= int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

saveVideo = True
takePicture = False
showVideo = False

if saveVideo == True:
    writer= cv2.VideoWriter('videos/basicvideo.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width,height))

i = 0
while True:
    
    ret,frame= cap.read()

    if saveVideo == True:
        print("Recording video")
        writer.write(frame)

    if showVideo == True:
        cv2.imshow('frame', frame)

    cv2.waitKey(1)
    
    i = i + 1



    if takePicture == True:    
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

    print(i)

    if saveVideo == True:
        if cv2.waitKey(1) & 0xFF == ord('p'):
            print("Saving video")
            cap.release()
            writer.release()
            cv2.destroyAllWindows()


cap.release()
if saveVideo == True:
    print("Saving video")
    writer.release()
cv2.destroyAllWindows()