import os
import glob
import shutil
import random
import cv2
import click


def save(i, folder, mask_folder):
    iname = os.path.basename(i)

    # remove mask suffix
    maskname = iname.replace('.png', '_mask.png')

    # copy image
    dstfold = os.path.join(folder, 'image')
    os.makedirs(dstfold, exist_ok=True)
    shutil.copyfile(i, os.path.join(dstfold, iname))

    mask_path = os.path.join(mask_folder, maskname)
    dstmasks = os.path.join(folder, 'masks')
    os.makedirs(dstmasks, exist_ok=True)
    # copy mask
    mask = cv2.imread(mask_path, 0)
    # convert image to white/black
    mask[mask != 0] = 255
    cv2.imwrite(os.path.join(dstmasks, iname), mask)


@click.command()
@click.option('--val', type=bool, default=False,
              help='Boolean, whether to create validation set')
@click.option('--img_folder', default='img',
              help='Path to folder with images')
@click.option('--mask_folder', default='img/masks',
              help='Path to folder with masks')
@click.option('--trainfrac', default=0.85,
              help='Fraction of data to set as training - the rest is test data')
def generate(val, img_folder, mask_folder, trainfrac):
    imgs = list(glob.glob(os.path.join(img_folder, '*.png'), recursive=False))
    random.shuffle(imgs)

    n = int(len(imgs)*trainfrac)
    train = imgs[:n]
    test = imgs[n:]

    if val:
        val_n = int(n*0.9)
        val = train[val_n:]
        train = train[:val_n]
        for i in val:
            save(i, 'val', mask_folder)

    for i in train:
        save(i, 'train', mask_folder)

    for j in test:
        save(j, 'test', mask_folder)


if __name__ == '__main__':
    generate()
