import cv2
import numpy as np
import math


class marker_detection():
    def __init__(self, yellow_image=None, blue_image=None, edge_image=None):
        '''
        This class detects a specific marker and maintains a ROI around.
        If one of the images is not None, then the mask will be saved in the class.
        '''

        # Black image in the size of the image
        self.pure_black = None
        
        # Used to time out the ROI, if the detection keeps failing
        self.roi_cnt = 0

        # The center and size of the ROI
        self.roi_center_x = None
        self.roi_center_y = None
        self.roi_size_x = 100
        self.roi_size_y = 50
        
        self.old_h = None
        self.old_w = None
        
        # Used to make it easier to display the image
        self.image = None
        
        # Used for debugging. If set to None, they are unused
        self.yellow_image = yellow_image
        self.blue_image = blue_image
        self.edge_image = edge_image

    
    def detection_loop(self, image):
        '''
        This is the main loop function of the class.
        It takes an image as input and detects any single marker in the bottom of the image.
        If a marker is found the pixel position of the center of the marker is returned, otherwise None is returned.
        The loop maintains a ROI, in which it searches for the marker
        '''
        if image is None:
            raise ValueError('"None" was given as image, this is likely due to opencv oppening the wrong camera')

        image_height, image_width, _ = image.shape
        # Create the pure black but only once
        if self.pure_black is None:
            self.pure_black = np.ones((image_height, image_width, 3)) * 0
        elif not self.pure_black.shape == image.shape:
            self.pure_black = np.ones((image_height, image_width, 3)) * 0
        
        # Make the image variable only be something when a detection is made
        self.image = None
        # Make the top 40% black, since the object cannot be in that half, or at least should not
        image[0:int(0.4 * image_height), :] = self.pure_black[0:int(0.4 * image_height), :]  # This is kinda an old nasty work around but it doesn't matter much because of the ROI
        
        # Cut the roi out of the image
        if self.roi_center_x is not None:
            # Ensure the roi always is within the image
            if self.roi_center_y - self.roi_size_y < 0:
                self.roi_size_y = self.roi_center_y
            y_min = self.roi_center_y - self.roi_size_y
            if self.roi_center_y + self.roi_size_y > image_height:
                self.roi_size_y = image_height - self.roi_center_y
            y_max = self.roi_center_y + self.roi_size_y
            
            if self.roi_center_x - self.roi_size_x < 0:
                self.roi_size_x = self.roi_center_x
            x_min = self.roi_center_x - self.roi_size_x
            if self.roi_center_x + self.roi_size_x > image_width:
                self.roi_size_x = image_width - self.roi_center_x
            x_max = self.roi_center_x + self.roi_size_x
            
            image = image[y_min:y_max, x_min:x_max]

            
        # Ensure there is some image
        if image.shape[0] < 10 or image.shape[1] < 10:
            # Otherwise reset the ROI
            self.roi_center_x = None
            self.old_h = None
            self.old_w = None
            return None, None
        
        x, y = self.extract_marker(image)
        
        # Check if the marker was found
        if x is None:
            self.roi_cnt += 1
            # Reset the roi, if nothing can be found in it
            if self.roi_cnt > 10:
                self.roi_center_x = None
                self.old_h = None
                self.old_w = None
            return None, None

        self.roi_cnt = 0
        self.image = image
        return x, y
        

    def extract_marker(self, image):
        '''
        This function takes an image as input and returns the center of a specific marker.
        The specific marker must be three squares inside each other, which alternates in the blue and yellow colors
        The function also updates the ROI parameters of the class.
        If None is returned, then the detection failed to detect any markers
        '''
        image_h, image_w, _ = image.shape
        contours_blue, contours_yellow, contours_edges = self.get_contours(image)
        points = filter_contours([contours_yellow, contours_blue, contours_edges], image_h, image_w, None)
        if len(points) == 0:
            return None, None
        image, p, backup_detection = find_fiducial(points, image)
        
        if p is None:
            return None, None

        # The back up detection only detects the inner yellow square, therefore use the old
        # roi size
        if backup_detection:
            # Don't use a backup detection as the first detection
            if self.old_w is None:
                return None, None
            w = self.old_w
            h = self.old_h
        else:
            _,_,w,h = cv2.boundingRect(p[5])
        
        cv2.drawContours(image, [p[5]], -1, (0,255,0), 1)


        # Update ROI
        if self.old_w is not None and not backup_detection:
            # Check that the size of the object haven't changed a lot since last frame
            if abs((w - self.old_w)) + abs((h - self.old_h)) > 10:
                return None, None
        # Always choose the largest size to use as roi size
        if w > h:
            self.roi_size_x = self.roi_size_y = int(w * 2)
        else:
            self.roi_size_x = self.roi_size_y = int(h * 2)
        
        # This is the center pixel of the marker
        x = p[0]
        y = p[1]
        # Calculate the center of the ROI in the whole image frame. (x,y) is in the ROI frame
        if self.roi_center_x is not None:
            x = self.roi_center_x + x - self.roi_size_x
            y = self.roi_center_y + y - self.roi_size_y
        self.roi_center_x = int(x)
        self.roi_center_y = int(y)

        self.old_w = w
        self.old_h = h
        
        # x, y is the position in the whole camera frame, p0, p1 is the position in the ROI frame
        return  x, y#p[0], p[1]


    def get_contours(self, image):
        '''
        Takes an image as input and finds the contours of three filters. A blue color, a yellow and an edge filter.
        The found contours are the output
        '''
        contours_blue, mask_blue = get_contours_color(image, 'blue')
        if self.blue_image is not None:
            self.blue_image = mask_blue
        
        contours_yellow, mask_yellow = get_contours_color(image, 'yellow')
        if self.yellow_image is not None:
            self.yellow_image = mask_yellow
        

        # Get the edge map (contrast map)
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(image_gray,100,200)
        if self.edge_image is not None:
            self.edge_image = edges
        kernel = np.ones((2, 2), np.uint8)
        edges = cv2.dilate(edges, kernel)
        
        contours_edges, _ = cv2.findContours(edges, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

        contours_edges = sorted(contours_edges, key=cv2.contourArea)
        contours_edges = contours_edges[::-1]
        return contours_blue, contours_yellow, contours_edges


def get_contours_color(image, color):
    '''
    Takes an image and a color argument, which must be either 'blue' or 'yellow'
    The function then applies a color filter of the chosen image and returns the mask and contours
    '''
    # Create a completely black version of the image
    # This could be created once beforehand instead TODO
    black = np.full((image.shape[0], image.shape[1]), 0, dtype=np.uint8)

    if color == 'blue':
        # Create the in range borders
        lower = np.array([40, 0, 0])
        upper = np.array([256, 200, 200])

        # Find all the pixels which are blue
        index_pos = np.where((image[:,:,0] > image[:,:,1]) & (image[:,:,0] > image[:,:,2]))
    elif color == 'yellow':
        # Create the in range borders
        lower = np.array([0, 50, 50])
        upper = np.array([200, 256, 256])
        
        # Find all the pixels which are yellow
        index_pos = np.where((image[:,:,1] > image[:,:,0]) & (image[:,:,2] > image[:,:,0]))
    else:
        raise ValueError('"color" argument for "get_contours_color" must be either "blue" or "yellow"')
    
    # Convert the black map to a mask map, by making all the pixels,
    # which were the right color white
    black[index_pos[0], index_pos[1]] = 255
    
    # Create a mask from the borders
    mask = cv2.inRange(image, lower, upper)
    # Combine the two masks
    mask_2 = np.bitwise_and(mask, black)
    mask = np.bitwise_not(mask_2)
    
    # Find the contours
    contours, _ = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    
    # Sort the contours largest to smallets
    contours = sorted(contours, key=cv2.contourArea)
    contours = contours[::-1]
    return contours, mask

def filter_contours(contours, image_h, image_w, image=None):
    '''
    This function applies a series of checks to the contours, and discards any which do not fulfill the criterias.
    The function takes a list of the contours as input, as well as the image size.
    This must be the size of the cropped image, if it is cropped
    If an image is provided, then the accepted contours will be drawn on the image
    Lastly the function returns a list which contains information on the accepted contours
    '''
    points = []
    for i, cnt in enumerate(contours):
        for j, c in enumerate(cnt):
            # Actually no idea what this does
            if len(c) < 5:
                # Mby a check to ensure the contour is valid
                break
            # Fit a rectangle to the contour
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            
            (x, y), (w, h), a = rect
            
            # Make sure the rectangle is squareish
            if h * 1.5 < w or w * 1.5 < h:
                continue
            
            # Calculate the 
            area = cv2.contourArea(c)

            # Filter out very small contours
            # Since the contours are sorted, we can just stop early
            if i < 2:
                if i == 0:
                    if area < 3:
                        break
                else:
                    if area < 40:
                        break
            else:
                if h * w < 3:
                    break
            
            # Check that the fitted rectangle is mostly filled by the contour
            # This checks that the contour is squareish
            if abs(area - h * w) > area * 0.3:
                continue

            if h > image_h * 0.8 or w > image_w * 0.8:
                continue
            
            # Draw the contours on the image, if it is provided
            if image is not None:
                if i == 0:
                    color = (0, 0, 255)
                elif i == 1:
                    color = (0, 255, 0)
                else:
                    color = (255, 255, 0)
                
                color = (0, 0, 0)


                cv2.circle(image, (int(x), int(y)), radius=2, color=color, thickness=-1)
                image = cv2.drawContours(image, [c], -1, color, 1)
                cv2.drawContours(image, [box], 0, color, 1)
            
            # Save the contour
            points.append([x, y, area, i, j, c])
    return points

def find_fiducial(points, image=None):
    '''
    This function looks through the points given by "filter_contours"
    and selects the point where the marker is
    The function takes the points as input, and if an image is given, then the function will
    draw the selected marker point on the image
    '''
    COLOR_TO_NUM = {}
    COLOR_TO_NUM['yellow'] = 0
    COLOR_TO_NUM['blue'] = 1
    COLOR_TO_NUM['edge'] = 2
    largest_area = 0
    back_up_candidate = []
    # Points are sorted yellow > blue > edge > large to small contour area
    if len(points) == 0:
        return image, None, False
    for p in points:
        close_cnt = 0
        middle_x = p[0]
        middle_y = p[1]
        middle_area = p[2]

        # Prioritize the yellow detection as back up candidate
        if p[3] == COLOR_TO_NUM['blue'] or len(back_up_candidate) == 0:
            if middle_area > largest_area:
                largest_area = middle_area
                back_up_candidate = p
        if not p[3] == COLOR_TO_NUM['blue']:
            continue

        # Don't search through the edges as candiates
        if p[3] > COLOR_TO_NUM['blue']:
            break

        p_used = []
        for p_min in points:
            # Skip points of the same color
            if p[3] == p_min[3] or p[3] in p_used:
                continue
            # Ensure the middle of the contours are close to each other
            if math.sqrt((middle_x - p_min[0])**2 + (middle_y - p_min[1])**2) > 3:
                continue
            # Ensure the difference in contour size is not too large
            if middle_area > p_min[2]:
                max_diff = p_min[2] * 4
            else:
                max_diff = middle_area * 4
            if abs(middle_area - p_min[2]) > max_diff:
                continue
            close_cnt += 1
            p_used.append(p_min[3])
        
        if close_cnt > 1:
            if image is not None:
                image = cv2.circle(image, (int(middle_x), int(middle_y)), radius=3, color=(0, 255, 0), thickness=-1)
            return image, p, False
        
    # Select the largest candidate
    if len(back_up_candidate) > 0:
        if image is not None:
            image = cv2.circle(image, (int(back_up_candidate[0]), int(back_up_candidate[1])), radius=3, color=(255, 0, 0), thickness=-1)
        return image, back_up_candidate, True

    return image, None, False


# Run this file stand-alone to test the image detection with a plugged in camera
if __name__ == '__main__':
    camera = cv2.VideoCapture(0)
    cam = marker_detection()
    while True:
        _, image = camera.read()
        cam.detection_loop(image)
        if cam.image is not None:
            cv2.imshow('test', cam.image)
        else:
            cv2.imshow('test', image)
        key = cv2.waitKey(1)
        if key == 27:
            break
        