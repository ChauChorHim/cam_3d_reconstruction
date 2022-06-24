# cam_3d_reconstruction

This is a series tools and models for reconstructing dense pointcloud using camera images recorded when driving.

## Module
### prepare_dataset
prepare data (images, poses) for depth prediction and pointcloud alignment
* Undistort, crop images
* Make train and validation files list
* Extract relative and absolute poses from GPS information
* Remove small objects in an object instance mask
* Remove static scenes in a video

### postprocess_data
* npy file (predicted mask) to tiff
* Given rgb images and absolute depth for each pixel, obatin a colorful pointcloud in camera frame
* Align isolated pointclouds with absolute poses 

### monodepth2_autowise
[monodepth2](https://github.com/nianticlabs/monodepth2) with:
* autowise_dataset.py
* inference_depth_pose.py
* translation_loss to fit the scale (trainer.py)
* training with mask

### doc
Documenting issues about pipeline of cam_3d_reconstruction (in Chinese).

