cd ../build
make -j4

cd ..
./bin/prepare_dataset/remove_static_scene \
	/home/Autowise/data/autowise/lingang_map5/camera_images/cam00/cropped/ \
	/home/Autowise/data/autowise/lingang_map5/splits/test/ \