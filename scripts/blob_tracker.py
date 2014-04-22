#!/usr/bin/env python

from blob import Blob
from blob_detector_ import BlobDetector
import cv2
import rospy
from blob_detector.msg import Blob as BlobMsg
from blob_detector.msg import Blobs as BlobsMsg
import numpy as np


class BlobTracker(BlobDetector):
    def __init__(self, node_name):
        super(BlobTracker, self).__init__(node_name)
        self.prev_blobs = []
        self.current_id = 1
    
    def process_blobs(self, blobs, rgbd):
        for new_blob in blobs:
            for old_blob in self.prev_blobs:
                if new_blob.similar_to(old_blob):
                    new_blob.acquire_history(old_blob)
                    break
            if new_blob.id is None:
                new_blob.id = self.current_id
                self.current_id += 1
        self.prev_blobs = blobs
        self.show_blobs(blobs, rgbd)
        self.publish_blobs(blobs)



if __name__ == '__main__':
    bt = BlobTracker('blob_tracker')
    bt.run()
