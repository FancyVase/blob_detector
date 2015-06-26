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
        self.inactive_blobs = {} # holds the IDs and coordinates of blobs 
                                 # that disappeared with the format 
                                 # ID: [centroid_x, centroid_y, median_depth]
    
    def process_blobs(self, blobs, rgbd):
        new_ids = []

        for new_blob in blobs:
            for old_blob in self.prev_blobs:
                if new_blob.similar_to(old_blob):
                    new_blob.acquire_history(old_blob)
                    break

            if new_blob.id is None: # a new blob was created
                if len(self.prev_blobs) > len(blobs): # number of blobs onscreen decreased
                    for blob in self.prev_blobs: # stores data for blobs that disappeared
                        if blob.id not in self.inactive_blobs:
                            self.inactive_blobs[blob.id] = [blob.centroid_x, blob.centroid_y, blob.median_depth]

                if len(self.inactive_blobs) > 0:
                    blob_id, coords = min([blob_id for blob_id in self.inactive_blobs.items() if blob_id not in new_ids], key=lambda (_, v): abs(v[0] - new_blob.centroid_x) + abs(v[1] - new_blob.centroid_y) + 8*abs(v[2] - new_blob.median_depth))
                    new_blob.id = blob_id
                    self.inactive_blobs.pop(blob_id, None)

                else:
                    new_blob.id = self.current_id
                    self.current_id += 1

            new_ids.append(new_blob.id)

        if len(self.prev_blobs) > len(blobs):
            for old_blob in self.prev_blobs:
                if old_blob.id not in self.inactive_blobs:
                    self.inactive_blobs[old_blob.id] = [old_blob.centroid_x, old_blob.centroid_y, old_blob.median_depth]

        # print "old blobs"
        for old_blob in self.prev_blobs:
            if old_blob.id not in new_ids and old_blob.id not in self.inactive_blobs and old_blob.id is not None:
                self.inactive_blobs[old_blob.id] = [old_blob.centroid_x, old_blob.centroid_y, old_blob.median_depth]
            print old_blob.id
        # print "new blobs"
        # for blob in blobs:
        #     print blob.id

        # print new_ids
        # print self.inactive_blobs
        # print "-----------------------"

        # if there are no blobs onscreen
        # clear the dictionary in order to minimize data being stored unnecessarily
        if not any(self.prev_blobs): 
            self.inactive_blobs = {}

        self.prev_blobs = blobs
        
        self.show_blobs(blobs, rgbd)
        self.publish_blobs(blobs)    

if __name__ == '__main__':
    bt = BlobTracker('blob_tracker')
    bt.run()
