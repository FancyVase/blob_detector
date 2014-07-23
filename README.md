blob_detector
=============

ROS package to detect adult human sized blobs in kinect depth image. 

The node `blob_tracker.py` reads depth image from kinect 1 using openni. It subtracts the background from the depth image, cleans it up, and finds large blobs in the cleaned foreground image. The tracker assigns IDs to blobs and publishes the id and the 3d location (in cms from the center of the kinect.)

### Inputs

    /camera/rgb/image_color
    /camera/depth_registered/image_raw

### Outputs

    /blobs

of type Blobs which is a list of Blob.

Blob.msg has the id and 3d location of the detected blob

```
uint64  id
float64 x
float64 y
float64 z
```

### Problems

Code is written to work with just one kinect and one person. If two people are in the scene and they overlap, the blobs merge together to become one Mega Blob. However, as long as they don't overlap, this will transmit distinct blobs and ids.


### Depends

Openni

### How to Launch

```
roslaunch openni_launch openni.launch 
rosrun blob_detector blob_tracker.py
```

