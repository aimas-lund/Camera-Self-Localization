import cv2 as cv
import sys

sys.path.append("Camera-Self-Localization-MBZIRC/marker_localization_main")

from src.localization.marker_detection.marker_detection import marker_detection

def main(video_name):
    """
    This function...
    """
    # Open the video
    folder = "Camera-Self-Localization-MBZIRC/data/"

    cap = cv.VideoCapture(folder + video_name)
    if cap.isOpened() == False:
        print("OpenCV couldn't open the video")
    else:
        detection = marker_detection(yellow_image=True, blue_image=True, edge_image=True)
    # Read until video is completed
    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        if video_name == "small_2.mp4":
            frame = cv.rectangle(frame, (55,420), (85,440), (0,0,0), -1)


        if ret == True:

            x, y = detection.detection_loop(frame)
            # Display the resulting frame
            cv.imshow('Frame',frame)

            if x != None:
                print(f"Pixels of the center: x={x}, y={y}")
            # Press Q on keyboard to  exit
            if cv.waitKey(25) & 0xFF == ord('q'):
                break


        # Break the loop
        else: 
            break

main("small_2.mp4")
