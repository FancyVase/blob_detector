#!/usr/bin/env python

import cv2
import numpy as np
from rgbd_image_processor import RGBDImageProcessor

class ForegroundProcessor(RGBDImageProcessor):
    def __init__(self, node_name):
        super(ForegroundProcessor, self).__init__(node_name)
        self.background_subtractor =  cv2.BackgroundSubtractorMOG(history=5, nmixtures=3, backgroundRatio=0.7)

    def get_depth_mask(self, depth_image):
        depth = depth_image.copy()
        cv2.normalize(depth, depth, 0, 255, cv2.NORM_MINMAX)
        depth = depth.astype(np.uint8)

        fgmask = self.background_subtractor.apply(depth)

        kernel = np.ones((5,5), np.uint8)
        fgmask = cv2.erode(fgmask, kernel, iterations = 2)
        fgmask = cv2.dilate(fgmask, kernel, iterations = 1)

        return fgmask

    def process_rgbd_image(self, rgb, depth):
        depth_mask = None
        depth_mask = self.get_depth_mask(depth)
        self.process_depth_mask_image(depth_mask, rgb, depth)

    # override me
    def process_depth_mask_image(self, depth_mask, rgb, depth):
        depth[depth_mask != 255] = 0
        #cv2.normalize(depth, depth, 0, 255, cv2.NORM_MINMAX)
        cv2.imshow('depth_mask', depth_mask)
        #k = cv2.waitKey(1)
        self.show_rgbd_image(rgb, depth)


if __name__ == '__main__':
    fg = ForegroundProcessor('fg')
    fg.run()
