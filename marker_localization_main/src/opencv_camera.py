import cv2
import os

class camera():
    def __init__(self, video_name, folder = "Camera-Self-Localization-MBZIRC/data/"):
        self.camera = cv2.VideoCapture(folder + video_name)
        # Immediately update the size of the image
        self.image_height = self.image_width = None
        _ = self.recv_image()
        self.folders = []


    def recv_image(self):
        _, tmp_image = self.camera.read()
        self.image_height, self.image_width, _ = tmp_image.shape
        return tmp_image
    
    # Ayaya, don't look. This is nasty

    def save_image(self, image, image_folder='images'):
        if not os.path.exists(image_folder):
            os.makedirs(image_folder)
        if not hasattr(self, image_folder + '_number'):
            setattr(self, image_folder + '_number', 0)
            setattr(self, image_folder + '_list', [])
            self.folders.append(image_folder)
        
        number_variable = getattr(self, image_folder + '_number')
        image_list = getattr(self, image_folder + '_list')

        # Save the image
        image_list.append(image)

        setattr(self, image_folder + '_number', number_variable + 1)
        setattr(self, image_folder + '_list', image_list)
    

    def write_images(self):
        for folder in self.folders:
            image_list = getattr(self, folder + '_list')
            for i, image in enumerate(image_list):
                path = folder + "/{:06}.png".format(i)
                cv2.imwrite(path, image)
