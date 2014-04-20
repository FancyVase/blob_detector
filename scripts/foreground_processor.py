#!/usr/bin/env python

import cv2
import numpy as np
from rgbd_image_processor import RGBDImageProcessor
NUM_FILTER = 3
MAX_DEPTH = 8000

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

        depth_image[depth_image > MAX_DEPTH] = MAX_DEPTH
        depth_image = (depth_image / MAX_DEPTH) * 255

        depth_image = depth_image.astype(np.uint8)

        mask = np.zeros_like(depth_image)
        mask[depth_image == 0] = 255

        mask = cv2.resize(mask, (320, 240))
        depth_image = cv2.resize(depth_image, (320, 240))


        depth_image = cv2.inpaint(depth_image, mask, 5.0, cv2.INPAINT_TELEA)

        return depth_image


    def process_rgbd_image(self, rgb, depth):
        depth_cleaned = self.clean_depth_image(depth)


        depth_mask = self.get_depth_mask(depth_cleaned)
        #depth_mask = np.zeros_like(depth_cleaned)
        depth_mask = cv2.resize(depth_mask, (640, 480))
        #depth = cv2.resize(depth_cleaned, (640, 480))

        self.process_depth_mask_image(depth_mask, rgb, depth)

    def filter_depth_mask(self, mask):
        if self.depth_mask_trail is None:
            self.depth_mask_trail = np.zeros((NUM_FILTER, mask.shape[0], mask.shape[1]))
        self.depth_mask_trail[self.filter_idx] = mask
        self.filter_idx = (self.filter_idx + 1) % NUM_FILTER

        mask=np.mean(self.depth_mask_trail, axis=0)
        print mask.shape, mask.dtype
        return mask


    # override me
    def process_depth_mask_image(self, depth_mask, rgb, depth):
        #depth[depth_mask != 255] = 0
        cv2.normalize(depth, depth, 0, 255, cv2.NORM_MINMAX)
        cv2.imshow('depth_mask', depth_mask)
        #k = cv2.waitKey(1)
        self.show_rgbd_image(rgb, depth)


if __name__ == '__main__':
    fg = ForegroundProcessor('fg')
    fg.run()
