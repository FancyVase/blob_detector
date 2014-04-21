#!/usr/bin/env python

import cv2
import numpy as np
from rgbd_image_processor import RGBDImageProcessor
NUM_FILTER = 3
MAX_DEPTH = 4000  ### key parameter. everything beyond this whited out and considered background
BUFFER_DEPTH = 2000

class ForegroundProcessor(RGBDImageProcessor):
    def __init__(self, node_name):
        super(ForegroundProcessor, self).__init__(node_name)
        self.background_subtractor =  cv2.BackgroundSubtractorMOG(history=5, nmixtures=3, backgroundRatio=0.8)
        self.depth_mask_trail = None
        self.filter_idx = 0

    def get_depth_mask(self, depth_image):
        fgmask = self.background_subtractor.apply(depth_image)

        kernel = np.ones((5,5), np.uint8)
        fgmask = cv2.erode(fgmask, kernel, iterations = 1)
        fgmask = cv2.dilate(fgmask, kernel, iterations = 1)
        #fgmask = self.filter_depth_mask(fgmask)
        return fgmask

    def clean_depth_image(self, depth_image):
        """
        http://www.morethantechnical.com/2011/03/05/neat-opencv-smoothing-trick-when-kineacking-kinect-hacking-w-code/
        need to get a mask for inpaintMask
        """
        depth_image = depth_image.copy()

        depth_image[depth_image > MAX_DEPTH] = MAX_DEPTH + BUFFER_DEPTH
        depth_image = (depth_image / (MAX_DEPTH + BUFFER_DEPTH)) * 255

        depth_image = depth_image.astype(np.uint8)

        mask = np.zeros_like(depth_image)
        mask[depth_image == 0] = 255

        mask = cv2.resize(mask, (320, 240))
        depth_image = cv2.resize(depth_image, (320, 240))


        depth_image = cv2.inpaint(depth_image, mask, 5.0, cv2.INPAINT_TELEA)

        return depth_image


    def process_rgbd_image(self, rgbd):
        depth_cleaned = self.clean_depth_image(rgbd.depth_raw)


        depth_mask = self.get_depth_mask(depth_cleaned)

        rgbd.depth_image_sm = depth_cleaned
        rgbd.depth_mask_sm = depth_mask

        rgbd.depth_color_sm = cv2.cvtColor(rgbd.depth_image_sm, cv2.COLOR_GRAY2BGR)


        self.process_depth_mask_image(rgbd)


    # override me
    def process_depth_mask_image(self, rgbd):
        self.show_depth_color(rgbd)

    def show_depth_color(self, rgbd):
        self._draw_status_text(rgbd.depth_color_sm)
        cv2.imshow('depth_color', rgbd.depth_color_sm)
        k = cv2.waitKey(1)

if __name__ == '__main__':
    fg = ForegroundProcessor('fg')
    fg.run()
