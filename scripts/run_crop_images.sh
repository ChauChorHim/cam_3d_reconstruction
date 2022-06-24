cd ../../build
make

cd ../examples/bin
./crop_images \
	/home/Autowise/data/autowise/lingang_map5/raw/cam00.yaml \
	/home/Autowise/data/autowise/lingang_map5/camera_images/cam00/undistorted \
	/home/Autowise/data/autowise/lingang_map5/camera_images/cam00/cropped \
	480 1760 640 950

