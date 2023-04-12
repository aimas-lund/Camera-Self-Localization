# Outside code
import time
import cv2

# Inside code
from src.localization.localization import localization
from src.localization.marker_detection.marker_detection import marker_detection
from src.opencv_camera import camera
from src.kalman.kalman import kalman_filter
from src.server import server as server_class
from src.log import log

# Config files
from src.localization.config import SIMULATION_RX_OFFSET, SIMULATION_RY_OFFSET, SIMULATION_RZ_OFFSET






cam = camera("big_1.mp4")

marker_localization = localization(image_fx=600, image_fy=600,
                                   image_height=cam.image_height, image_width=cam.image_width,
                                   marker_size=0.28*2,
                                   detection_class=marker_detection, ry_offset=3)  # raspberry cam on drones have ~3 degrees offset
kalman = kalman_filter()
estimate_x = estimate_y = 0

data_log = log('data', no_header=True)

server = server_class()

height = None
x = 5.95
estimate_x = 5
y = 0
initial_rz = None
try:
    while True:
        t = time.time()
        tmp = server.main_loop('S' + str(x) + ',' + str(-y) + 'E')# + 'S' + str(1.02121) + ',' + str(121211.2012121) + 'E')
        if tmp is not None:
            tmp_string = tmp.decode()
            value_strings = tmp_string.split(',')
            print(value_strings, x, y)
            data_log.log(t, *value_strings, x, y)
            height = float(value_strings[0])
            if initial_rz is None:
                initial_rz = float(value_strings[3])
            rz = initial_rz - float(value_strings[3])
        image = cam.recv_image()
        # cam.save_image(image.copy())
        if height is None:
            continue

        old_x = x
        old_y = y
        x, y = marker_localization.update(image, t, estimate_x, estimate_y, 0, 0, 0, 0, rz, height)
        if x is None:
            x = old_x
            y = old_y
            continue
        
        if x is not None:
            estimate_x = x
            estimate_y = y

        # x, y = kalman.update_loop([0,0,0,0], x, y, 0.8, t)
        # print(x, y)
        # cam.save_image(image, 'analyzed_image')
        # cv2.imshow('test', image)
        # key = cv2.waitKey(1)
        # if key == 27:
        #     break
        
except KeyboardInterrupt:
    print('Saving images, please wait')
    cam.write_images()
    data_log.write()