# cam_3d_reconstruction

This is a series tools and model for reconstructing dense pointcloud of the street using camera images recorded when driving.

## prepare_dataset
* Undistort, crop images
* Remove static scenes in a video
* Remove small objects in an object instance mask
* Make train and validation files list
* Extract poses from GPS information

## depth_to_points
* Given rgb images and absolute depth for each pixel, obatin a colorful pointcloud in camera frame
* Align isolated pointclouds with their poses 

## autowise_monodepth

## mask_rcnn_instance_segmentation