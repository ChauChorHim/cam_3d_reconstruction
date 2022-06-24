cd ../build
make -j4

cd ..
./bin/prepare_dataset/make_train_val_files \
	/home/Autowise/data/autowise/lingang_map5/camera_images/cam00/raw/ \
	/home/Autowise/data/autowise/lingang_map5/splits/ \