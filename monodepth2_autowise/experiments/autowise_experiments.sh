cd ..

CUDA_VISIBLE_DEVICES="1" python -m train \
	--data_path ~/data/autowise/lingang_map5/camera_images/cam00/cropped/ \
	--model_name 03 \
	--dataset autowise \
	--split lingang_map5 \
	--height 192 \
	--width 704 \
	--num_workers 16