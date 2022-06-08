cd ..

CUDA_VISIBLE_DEVICES="1" python -m train \
	--data_path ~/data/KITTI_raw \
	--model_name 01 \
	--png