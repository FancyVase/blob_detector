import cv2
import numpy as np
import colorsys
import math

def rgb_by_index(idx):
    idx = idx  % 1.0
    r,g,b = colorsys.hsv_to_rgb(idx, 0.5, 0.5)
    return (int(r*255), int(g*255), int(b*255))

class Blob(object):
    def __init__(self, contour, source_rgbd):
        self.contour = contour
        self.source_rgbd = source_rgbd
        self.idx = 0

        self.area = cv2.contourArea(contour)
        self.moments = cv2.moments(contour)
        self.centroid_x = int(self.moments['m10']/(self.moments['m00'] + 1.0))
        self.centroid_y = int(self.moments['m01']/(self.moments['m00'] + 1.0))

        self._mask = None
        self.median_depth = None
        self.x_w, self.y_w, self.z_w = None, None, None

    def compute_params(self):
        self._set_mask()
        self._set_median_depth()
        self._set_world_coordinates()



    def _set_mask(self):
        mask = np.zeros_like(self.source_rgbd.depth_mask_sm)
        cv2.drawContours(mask, [self.contour], 
                        0, # draw the only contour
                        color = 1, 
                        thickness = -1, # filled
                        lineType = cv2.CV_AA)
        self._mask = mask

    def _set_median_depth(self):
        mask_large = cv2.resize(self._mask, (640, 480))
        median_depth = np.median(self.source_rgbd.depth_raw[mask_large != 0])
        median_depth = max(median_depth, 500) 
        self.median_depth = median_depth / 10 # in cms and not kinect's mm

    def _set_world_coordinates(self):
        centroid_x = self.centroid_x * 2
        centroid_y = self.centroid_y * 2

        x_p = -1*(centroid_x-320)
        y_p = -1*(centroid_y-240)
        f_p = 425


        self.x_w = (x_p * self.median_depth) / (math.sqrt((x_p * x_p) + (f_p*f_p)))

        self.y_w = (y_p * self.median_depth) / (math.sqrt((y_p * y_p) + (f_p*f_p)))
         
        self.z_w = math.sqrt((self.median_depth*self.median_depth) 
                              - ((self.x_w*self.x_w) 
                              + (self.y_w * self.y_w)))                


    def draw(self, image):
        cv2.drawContours(image, [self.contour], 
                        0, # draw the only contour
                        color = rgb_by_index(self.idx), 
                        thickness = -1, # filled
                        lineType = cv2.CV_AA)
        cv2.ellipse(image, box=((self.centroid_x, self.centroid_y), (6,6), 0), color=(0,0,255),
                    thickness = -1)
        if self.z_w is not None:
            cv2.putText(image, "%s %s %s" % (int(self.x_w), int(self.y_w), int(self.z_w)),
                        (self.centroid_x + 5, self.centroid_y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, # font_scale
                        (0, 100, 180), # color
                        2 # thickness
                        )
            cv2.putText(image, "d:%s a:%s" % (int(self.median_depth), int(self.area)),
                        (self.centroid_x + 5, self.centroid_y + 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, # font_scale
                        (0, 100, 180), # color
                        2 # thickness
                        )            


