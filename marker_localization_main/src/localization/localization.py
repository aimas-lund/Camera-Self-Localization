import numpy as np

class localization():
    def __init__(self, image_fx, image_fy, detection_class,
                 image_height, image_width, marker_size,
                 detection_function_name='detection_loop',
                 rx_offset=0, ry_offset=0, rz_offset=0):
        # Setup the object detection
        self.detection_class = detection_class()
        self.detection_function_name = detection_function_name
        # Get the function (from the instance) that we need to call
        self.detection_function = getattr(self.detection_class, self.detection_function_name)

        # Angle offsets from the camera to the drone. Position offsets are negligible
        self.rx_offset = np.deg2rad(rx_offset)
        self.ry_offset = np.deg2rad(ry_offset)
        self.rz_offset = np.deg2rad(rz_offset)
        
        # It is assumed that the middle of the marker is detected (otherwise set marker size to 0)
        self.marker_size = marker_size

        # The focal length of the camera
        self.image_fx = image_fx
        self.image_fy = image_fy

        # The last received time
        self.old_time = None
        # Last estimated position
        self.old_estimate_x = None
        self.old_estimate_y = None

        # This could be removed, since it is done in the Kalaman filter,
        # but it is kept, so that it will work without a Kalman filter
        self.measurement_trust = 0.1

        # How much the low pass filter will allow the measurement to move
        # This is multiplied with the speed of the drone
        self.low_pass_filter_limit = 2

        # Since the limit is tied to the speed, an extra component is needed to always allow
        # the measurement to change
        self.low_pass_filter_drift = 0.1
        
        self.set_image_size(image_height, image_width)

    def set_image_size(self, image_height, image_width):
        self.image_height = image_height
        self.image_width = image_width

    def update(self, image, time, estimate_x, estimate_y, speed_x, speed_y, rx, ry, rz, drone_height):
        # Range finder does not work until a certain height
        if drone_height < 0.6:
            return None, None
        # pixel_x, pixel_y, image_height, image_width, camera_rx, camera_ry, camera_rz, drone_height
        # time, speed x and y, x and y estimate
        pixel_x, pixel_y = self.detection_function(image)
        if pixel_x is None:
            return None, None
        dist_x, dist_y = self.calculate_odometry(pixel_x, pixel_y, 0, 0, 0, drone_height)
        estimate_x, estimate_y = self.filter_position_estimate(dist_x, dist_y, time, estimate_x, estimate_y, speed_x, speed_y)
        # TODO send via ros2
        return estimate_x, estimate_y

    def calculate_odometry(self, pixel_x, pixel_y,
                           camera_rx, camera_ry, camera_rz, drone_height):
        # Calculate the transformation from camera to drone:
        rx = camera_rx + self.rx_offset
        Rx = np.array([[1, 0, 0],
                        [0, np.cos(rx), -np.sin(rx)],
                        [0, np.sin(rx), np.cos(rx)]])
        ry = camera_ry + self.ry_offset
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                        [0, 1, 0],
                        [-np.sin(ry), 0, np.cos(ry)]])

        rz = camera_rz + self.rz_offset
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                        [np.sin(rz), np.cos(rz), 0],
                        [0, 0, 1]])

        R = np.dot(Rz, Ry)
        R = np.dot(R, Rx)
        Rcam = np.array([[0, 0, -1],
                            [1, 0, 0],
                            [0, -1, 0]])
        R = np.dot(Rz, Ry)
        R = np.dot(R, Rx)
        R = np.dot(R, Rcam)

        # Calculate how far from the center of the image, the target object is
        error_x = (pixel_x - self.image_width/2) / self.image_fx
        error_y = (pixel_y - self.image_height/2) / self.image_fy
        v_cam = np.array([[error_x],
                          [error_y],
                          [1]])
        v_norm = np.divide(v_cam, np.linalg.norm(v_cam))
        v_world = np.dot(R, v_norm)

        # Get the angle to the object and calcualte the direct distance
        angle = np.pi/2 - np.arccos(-v_world[2,0])
        dist = (drone_height - self.marker_size/2) / np.tan(angle)

        # Get the x and y component of the distance
        dist_x = -v_world[0, 0] * dist
        dist_y = -v_world[1, 0] * dist

        return dist_x, dist_y



    def filter_position_estimate(self, dist_x, dist_y, time, estimate_x, estimate_y, speed_x, speed_y):
        # The first loop have to initialize the old variables and just use the pure estimate
        if self.old_time is not None:
            ts = time - self.old_time

            # Calculate the total speed of the drone
            # They could also be used individually, might be better
            speed = np.sqrt(speed_x**2 + speed_y**2)

            sign_x = np.sign(dist_x - self.old_estimate_x)
            sign_y = np.sign(dist_y - self.old_estimate_y)

            # If the new measure distance is much larger than what the drone could have moved,
            # then limit the change (LOW PASS FILTER)
            if abs(dist_x - self.old_estimate_x) > speed * ts * self.low_pass_filter_limit:
                # Add one in the correct direction
                dist_x = self.old_estimate_x +\
                        sign_x * speed * ts * self.low_pass_filter_limit +\
                        sign_x * self.low_pass_filter_drift

            if abs(dist_y - self.old_estimate_y) > speed * ts * self.low_pass_filter_limit:
                # Add one in the correct direction
                dist_y = self.old_estimate_y +\
                        sign_y * speed * ts * self.low_pass_filter_limit +\
                        sign_y * self.low_pass_filter_drift

        # Save the values for next loop
        self.old_time = time
        self.old_estimate_y = dist_y
        self.old_estimate_x = dist_x

        # Update the position based on the Kalman gain
        estimate_x = (dist_x * self.measurement_trust + estimate_x * (1 - self.measurement_trust))
        estimate_y = (dist_y * self.measurement_trust + estimate_y * (1 - self.measurement_trust))

        return estimate_x, estimate_y