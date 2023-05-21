import cv2 as cv
import sys
import os


sys.path.insert(1, "/home/mbzirc/repositories/Camera-Self-Localization-MBZIRC/pc control/siyi_control")
sys.path.append("Camera-Self-Localization-MBZIRC/marker_localization_main")

from siyi_sdk import SIYISDK
from marker_detection import marker_detection

def main(openVideo, recordVideo, showVideo, videoName):
    """
    This function...
    """
    # Open the camera
    cam = SIYISDK(server_ip="192.168.144.25", port=37260)
    if not cam.connect():
        print("Error opening the camera with SIYISDK")
    else:
        print("Camera opened")

    # Open the video
    if openVideo == True:
        video_name = "small_2.mp4"
        folder = "Camera-Self-Localization-MBZIRC/data/"

        cap = cv.VideoCapture(folder + video_name)


        if cap.isOpened() == False:
            print("OpenCV couldn't open the video")
        
        
    if recordVideo == True:

        cap= cv.VideoCapture('rtsp://192.168.144.25:8554/main.264')
        if cap.isOpened() == False:
            print("OpenCV couldn't open the stream")
        width= int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
        height= int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))


        writer= cv.VideoWriter('videos/basicvideo.mp4', cv.VideoWriter_fourcc(*'DIVX'), 20, (width,height))

    detection = marker_detection(yellow_image=True, blue_image=True, edge_image=True)

    # Read until video is completed
    i = 0
    while(cap.isOpened()):
        # Capture frame-by-frame

        print(i)
        
        ret, frame = cap.read()

        if videoName == "small_2.mp4":
            frame = cv.rectangle(frame, (55,420), (85,440), (0,0,0), -1)

        if ret == True:
            i = i + 1
            x, y = detection.detection_loop(frame)
            # Display the resulting frame
            if showVideo == True:
                cv.imshow('Frame',frame)

            if recordVideo == True:
                writer.write(frame)

            if x != None:
                print(f"Pixels of the center: x={x}, y={y}")
            # Press Q on keyboard to  exit
            if cv.waitKey(25) & 0xFF == ord('q'):
                
                print("Saving video")
                cap.release()
                if recordVideo == True:
                    writer.release()
                cv.destroyAllWindows()
                
                break


        # Break the loop
        else:
            cap.release()
            if recordVideo == True:
                print("Saving video")
                writer.release()        
            cv.destroyAllWindows()
            break

    cap.release()
    if recordVideo == True:
        print("Saving video")
        writer.release()        
    cv.destroyAllWindows()

main(openVideo=False, recordVideo=True, showVideo=False, videoName=False)
