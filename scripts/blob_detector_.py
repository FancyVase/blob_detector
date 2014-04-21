#!/usr/bin/env python

from blob import Blob
from foreground_processor import ForegroundProcessor
import cv2
import operator
import rospy
from blob_detector.msg import Blob as BlobMsg
from blob_detector.msg import Blobs as BlobsMsg
import numpy as np


class BlobDetector(ForegroundProcessor):
    def __init__(self, node_name):
        super(BlobDetector, self).__init__(node_name)
        self.pub = rospy.Publisher('/blobs', BlobsMsg)

    def find_blobs(self, rgbd):
        mask = rgbd.depth_mask_sm
        contours0 = cv2.findContours( mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cv2.approxPolyDP(cnt, 3, True) for cnt in contours0[0]]        

        blobs = [Blob(contour=c, source_rgbd=rgbd) for c in contours]
        blobs = [b for b in blobs if b.area > 800] # filter

        [b.compute_params() for b in blobs] # cpu intensive initialization

        return blobs
        
    def process_depth_mask_image(self, rgbd):
        blobs = self.find_blobs(rgbd)
        #for blob in blobs:
        #    blob.set_world_coordinates_from_depth(rgbd.depth_raw)
        self.process_blobs(blobs, rgbd)


    def publish_blobs(self, blobs):
        blobs_msg = BlobsMsg()
        for blob in blobs:
            blob_msg = blob.to_msg()
            blobs_msg.blobs.append(blob_msg)

        self.pub.publish(blobs_msg)

    def show_blobs(self, blobs, rgbd):
        for blob in blobs:
            blob.draw(rgbd.depth_color_sm)
        self.show_depth_color(rgbd)


    def process_blobs(self, blobs, rgbd):
        self.publish_blobs(blobs)
        self.show_blobs(self, blobs, rgbd)

if __name__ == '__main__':
    bd = BlobDetector('fg')
    bd.run()
