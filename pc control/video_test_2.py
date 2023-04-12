import cv2 as cv
import sys
import time

sys.path.append("Camera-Self-Localization-MBZIRC/marker_localization_main")

from src.localization.marker_detection.marker_detection import marker_detection
from src.localization.localization import localization
from src.opencv_camera import camera
from src.kalman.kalman import kalman_filter

def main(video_name):
    """
    This function...
    """
    # Open the video
    folder = "Camera-Self-Localization-MBZIRC/data/"
    cam = camera("big_1.mp4")
    marker_localization = localization(image_fx=736.92, image_fy=738.61,
                                       image_height=cam.image_height, image_width=cam.image_width,
                                       marker_size=0.28*2,
                                       detection_class=marker_detection, ry_offset=3)

    kalman = kalman_filter()
    estimate_x = estimate_y = 0
    height = None
    x = 5.95
    estimate_x = 5
    y = 0
    initial_rz = None

    try:
        while True:
            t = time.time()
            print(x,y)
            height = float


            image = cam.recv_image()
            
            if height is None:
                continue

            old_x = x
            old_y = y
            x, y = marker_localization.update()

    except KeyboardInterrupt:
        print('Saving images, please wait')
        cam.write_images()

main("small_2.mp4")
