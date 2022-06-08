import PIL.Image
import cv2
import numpy as np

def mask_loader(path):
    # open path as file to avoid ResourceWarning
    # (https://github.com/python-pillow/Pillow/issues/835)
    with open(path, 'rb') as f:
        with PIL.Image.open(f) as mask:
            return mask

def pil_loader(path):
    # open path as file to avoid ResourceWarning
    # (https://github.com/python-pillow/Pillow/issues/835)
    with open(path, 'rb') as f:
        with PIL.Image.open(f) as img:
            return img.convert('RGB')

# path = "/home/Autowise/data/autowise/lingang_map5/mask/cam00/tiff/1645427078.301061.tiff"
# npy_path = "/home/Autowise/data/autowise/lingang_map5/mask/cam00/npy/1645427078.301061.npy"
img_path = "/home/Autowise/data/autowise/lingang_map5/camera_images/cam00/cropped/1645427078.301061.jpg"
no_small_path = "/home/Autowise/data/autowise/lingang_map5/mask/cam00/mask_for_training/1645427078.301061.tiff"
#
# mask = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
# mask_npy = np.load(npy_path)
# mask = mask_loader(path)
img = pil_loader(img_path)
img.show()

img = np.array(img)

no_small = cv2.imread(no_small_path, cv2.IMREAD_GRAYSCALE)
# no_small = cv2.imread(no_small_path)
mask = no_small == 0
mask = np.array([mask, mask, mask])
mask = np.transpose(mask, (1, 2, 0))

img = img * mask

img = PIL.Image.fromarray(img)

img.show()

print(mask.shape)