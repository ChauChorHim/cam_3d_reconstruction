cd ../build
make -j4

cd ..
./bin/prepare_dataset/downsample_images \
	/home/Autowise/data/autowise/lingang_map5/camera_images/cam00/cropped \
	/home/Autowise/data/autowise/lingang_map5/camera_images/cam00/3 \
	88 \
	24
