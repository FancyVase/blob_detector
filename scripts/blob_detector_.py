#!/usr/bin/env python

from blob import Blob
from foreground_processor import ForegroundProcessor
import cv2
import operator
import rospy
from blob_detector.msg import Blob as BlobMsg
from blob_detector.msg import Blobs as BlobsMsg



class BlobDetector(ForegroundProcessor):
    def __init__(self, node_name):
        super(BlobDetector, self).__init__(node_name)
        self.pub = rospy.Publisher('/blobs', BlobsMsg)

    def find_blobs(self, mask):
        contours0 = cv2.findContours( mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cv2.approxPolyDP(cnt, 3, True) for cnt in contours0[0]]        

        blobs = [Blob(contour=c) for c in contours]
        blobs = [b for b in blobs if b.area > 800] # filter
        blobs = sorted(blobs, key=operator.attrgetter('area'), reverse=True)
        for i,b in enumerate(blobs):
            b.idx = i / 20.0
        return blobs
        
    def process_depth_mask_image(self, depth_mask, rgb, depth):
        blobs = self.find_blobs(depth_mask)
        for blob in blobs:
            blob.set_world_coordinates_from_depth(depth)
        self.process_blobs(blobs, rgb, depth)

    def process_blobs(self, blobs, rgb, depth):
        blobs_msg = BlobsMsg()
        blobs_msg.blobs = []
        for blob in blobs:
            blob_msg = BlobMsg()
            blob_msg.x = blob.x_w
            blob_msg.y = blob.y_w
            blob_msg.z = blob.z_w
            blobs_msg.blobs.append(blob_msg)

        self.pub.publish(blobs_msg)

        for blob in blobs:
            blob.draw(rgb)
        self.show_rgbd_image(rgb, depth)

if __name__ == '__main__':
    bd = BlobDetector('fg')
    bd.run()
