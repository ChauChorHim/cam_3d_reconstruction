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

<!-- ### mask_rcnn_instance_segmentation (not upload yet)
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
## Pipeline
1. Prepare dataset
2. Train model
3. Estimate depth (and pose)
4. Reproject depth to camera coordinates
5. Align pointclouds

## Prepare dataset
This 'recipe' of *cam_3d_reconstruction* requires 'ingredients' including: **video sequences**, **camera intrinsic matrix** and **camera extrinsic matrix** (if scale recovery is necessary).

### Undistort and crop images
Two tools for undistorting and cropping images are available. Overwrite *Camera::load()* in order to fullfil the format of the file storing camera intrinsic matrix.

```bash
# Undistort images
./undistort_images \
	/path/to/camera/intrinsic/matrix
	/path/to/raw/folder \
	/path/to/undistorted/folder \

# Crop images
./crop_images \
	/path/to/camera/intrinsic/matrix
	/path/to/undistorted/folder \
	/path/to/cropped/folder \
    image_new_height \
    image_new_width \
    image_center_row \
    image_center_col
```

### Make train and val files list
train_files.txt and val_files.txt listing files names are required by the dataloader when training models.

```bash
# Make train and val files list
./make_train_val_files \
	/path/to/images/folder \
	/path/to/output/folder 
```
The format of the output file is like: 
```bash
0 balabala1.png
1 balabala2.png
...
...
...
```

Dropping static camera images during training is a method to level up quality of depth estimation (an assumption of monocular depth estimation is moving camera with static scene). 

**remove_static_scene** is a tool similar to **make_train_val_files** while removing the static images files as its name indicated.

```bash
./remove_static_scene \
	/path/to/cropped/folder/ \
	/path/to/output/folder 
```

## Train model
TBD

## Estimate depth (and pose)
TBD

## Reproject depth to camera coordinates
TBD

## Align pointclouds
TBD

## Demo results
TBD -->