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

### depth_to_points
* Given rgb images and absolute depth for each pixel, obatin a colorful pointcloud in camera frame
* Align isolated pointclouds with absolute poses 

### monodepth2_autowise (not upload yet)
[monodepth2](https://github.com/nianticlabs/monodepth2) with:
* autowise_dataset.py
* inference_depth_pose.py
* translation_loss to fit the scale (trainer.py)
* training with mask

### mask_rcnn_instance_segmentation (not upload yet)
[mask_rcnn](https://github.com/matterport/Mask_RCNN) with:
* run_pre_trained_model.py

## Installation

Install from source

```bash
  mkdir build
  cd build
  cmake ..
  make
```

## Prepare data
TBD

## Train depth estimation model
TBD

## Reconstruct dense pointcloud
TBD

## Demo results
TBD