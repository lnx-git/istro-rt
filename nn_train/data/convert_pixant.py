import os
import glob
import shutil
import random
import cv2
import click
import numpy as np


def convert(image_name, img_folder, mask_folder):
    print('save: image_name={}, img_folder={}, mask_folder={}'.format(image_name, img_folder, mask_folder))

    # check if 
    mask_name = image_name.replace('.jpg', '_color_mask.png')
    if not os.path.isfile(mask_name):
       print("save: error: mask doesn't exists - ignoring, mask_name={}".format(mask_name))
       return 0

    # prepare new image_name
    image_basename2 = os.path.basename(image_name)
    # remove '_camera' suffix
    image_basename2 = image_basename2.replace('_camera', '')
    image_basename2 = image_basename2.replace('.jpg', '.png')
    image_name2 = os.path.join(img_folder, image_basename2)

    # prepare new mask_name
    mask_basename2 = image_basename2.replace('.png', '_mask.png')
    mask_name2 = os.path.join(mask_folder, mask_basename2)

    # save image
    #shutil.copyfile(image_name, os.path.join(img_folder, image_basename))
    image = cv2.imread(image_name)
    image_width = image.shape[1]
    image_height = image.shape[0]
    print('image.shape: image_width={}, image_height={}'.format(image_width, image_height))
    if image_width != 640 or image_height != 480:
       print("save: error: wrong image dimensions!")
       return -1
    cv2.imwrite(image_name2, image)

    # save mask
    mask = cv2.imread(mask_name)
    mask_width = mask.shape[1]
    mask_height = mask.shape[0]
    print('mask.shape: mask_width={}, mask_height={}'.format(mask_width, mask_height))
    if mask_width != 640 or mask_height != 480:
       print("save: error: wrong mask dimensions!")
       return -1
    # remove white bounding box
    mask[0,:] = mask[1,:]
    mask[mask_height-1,:] = mask[mask_height-2,:]
    mask[:,0] = mask[:,1]
    mask[:,mask_width-1] = mask[:,mask_width-2]
    mask[0,0] = mask[1,1]
    mask[0,mask_width-1] = mask[1,mask_width-2]
    mask[mask_height-1,0] = mask[mask_height-2,1]
    mask[mask_height-1,mask_width-1] = mask[mask_height-2,mask_width-2]
    # convert 'road' label to white color
    label_ROAD_color = [128, 64, 128]    # print(mask[478, 1])
    mask[np.where((mask!=label_ROAD_color).all(axis=2))] = [0, 0, 0]
    mask[np.where((mask==label_ROAD_color).all(axis=2))] = [255,255,255]
    # convert to grayscale - for saving 8-bit PNG
    mask_gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(mask_name2, mask_gray)

    return 1


@click.command()
@click.option('--pix_folder', default='pixant',
              help='Path to folder with pixant images')
@click.option('--img_folder', default='img',
              help='Path to folder with images')
@click.option('--mask_folder', default='img/masks',
              help='Path to folder with masks')
def generate(pix_folder, img_folder, mask_folder):
    pixs = list(glob.glob(os.path.join(pix_folder, '*.jpg'), recursive=False))

    # create output directories
    os.makedirs(img_folder, exist_ok=True)
    os.makedirs(mask_folder, exist_ok=True)

    img_cnt = 0

    for i in pixs:
        res = convert(i, img_folder, mask_folder)
        if res > 0:
            img_cnt += 1
        if res < 0:
            break

    print('finished: pix_cnt={}, img_cnt={}'.format(len(pixs), img_cnt))


if __name__ == '__main__':
    generate()
