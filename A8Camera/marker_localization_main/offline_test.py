import os
import cv2

from src.localization.marker_detection.marker_detection import marker_detection


# image_folder_path = '../images_4'

# # Create the window
# image_name = "/{:06}.png".format(0)
# image = cv2.imread(image_folder_path + image_name)
# cv2.imshow('win_main', image)
# cv2.moveWindow("win_main", 0, 0);


detection = marker_detection(yellow_image=True, blue_image=True, edge_image=True)  # Toxic python stuff
vcap = cv2.VideoCapture("rtsp://192.168.144.25:8554/main.264")

# number_of_images = len(os.listdir(image_folder_path))
# i = 0



while True:
    # Load a new image
    ret, frame = vcap.read()
    # image_name = "/{:06}.png".format(i)
    # image = cv2.imread(image_folder_path + image_name)
    detection.detection_loop(frame)
    print(detection.old_h)
    
    # Show the image
    cv2.imshow('win_main', frame)
    cv2.moveWindow("win_main", 0, -1)
    # cv2.setWindowTitle("win_main", frame)
    
    # Yellow
    cv2.imshow('win_yellow', detection.yellow_image)
    cv2.moveWindow("win_yellow", 0 + 640, 0)
    # cv2.setWindowTitle("win_yellow", 'yellow' + frame)
    
    # Blue
    cv2.imshow('win_blue', detection.blue_image)
    cv2.moveWindow("win_blue", 0, 0 + 602)
    # cv2.setWindowTitle("win_blue", 'blue' + frame)
    
    # Edges
    cv2.imshow('win_edge', detection.edge_image)
    cv2.moveWindow("win_edge", 640, 0 + 602)
    # cv2.setWindowTitle("win_edge", 'edge' + frame)


    key = cv2.waitKey(0)
    # i += 1
    # if key == 27:
    #     break
    # elif key == 81 or key == 97:
    #     if i > 1:
    #         i -= 2
    #     elif i == 1:
    #         i -= 1

        
    # if i >= number_of_images:
    #     break
    
