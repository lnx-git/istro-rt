import glob
import os

import numpy as np
from keras.preprocessing.image import ImageDataGenerator
import skimage.io as io
from skimage import transform
import cv2
from imgaug import augmenters as iaa

Road = [128, 64, 128]
Noroad = [255, 73, 95]
COLORS = np.array([Road, Noroad])

aug_blurer = iaa.MotionBlur(k=(3, 7))
#aug_fogger = iaa.Fog()
aug_contraster = iaa.GammaContrast(gamma=(0.30, 1.50))


def rgb_to_hsv(img):
    #return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    red = img[:,:,0]
    green = img[:,:,1]
    blue = img[:,:,2]
    avgv = (red + green + blue)/3
    img2 = img.copy()
    img2[:,:,0] = avgv
    img2[:,:,1] = avgv
    img2[:,:,2] = avgv
    return img2


def normalize_image(img, colorspace='rgb'):
    if colorspace == 'rgb':
        return img / 255
        # return img / 127.5 - 1
    elif colorspace == 'hsv':
        # cv2 rgb to hsv returns H value in range [0, 180] if the image
        # is of type int. In case of float, the range is [0, 360]
        #return img / [360, 1, 255]
        return img / 255


def fix_mask(mask):
    m = mask.copy()
    m[m < 30] = 0.0
    m[m > 0] = 1.0
    return m


def train_generator(batch_size, train_path, image_folder, mask_folder,
                    img_target_size=(480, 640), augs={}, tohsv=False,
                    aug=False):
    if tohsv:
        image_datagen = ImageDataGenerator(preprocessing_function=rgb_to_hsv,
                                           **augs)
    else:
        image_datagen = ImageDataGenerator(**augs)
    masks_datagen = ImageDataGenerator(preprocessing_function=fix_mask, **augs)

    image_generator = image_datagen.flow_from_directory(
        train_path,
        class_mode=None,
        classes=[image_folder],
        color_mode='rgb',
        target_size=img_target_size,
        batch_size=batch_size,
        seed=1
    )

    mask_generator = masks_datagen.flow_from_directory(
        train_path,
        class_mode=None,
        classes=[mask_folder],
        color_mode='grayscale',
        target_size=img_target_size,
        batch_size=batch_size,
        seed=1
    )

    colorspace = 'rgb'
    if tohsv:
        colorspace = 'hsv'

    generator = zip(image_generator, mask_generator)

    for (img, mask) in generator:
        if aug:
            # blur augmentation
            img_aug = aug_blurer.augment_images(img)
            #img_aug = normalize_image(img_aug, colorspace=colorspace)
            #yield (img_aug, mask)

            # contrast augmentation
            img_aug2 = aug_contraster.augment_images(img_aug)
            img_aug2 = normalize_image(img_aug2, colorspace=colorspace)
            yield (img_aug2, mask)

        img = normalize_image(img, colorspace=colorspace)
        yield (img, mask)

        # add grayscale image
        img2 = rgb_to_hsv(img)
        img2 = normalize_image(img2, colorspace=colorspace)
        yield (img2, mask)

        if aug:
            # blur augmentation
            img2_aug = aug_blurer.augment_images(img2)
            # contrast augmentation
            img2_aug2 = aug_contraster.augment_images(img2_aug)
            img2_aug2 = normalize_image(img2_aug2, colorspace=colorspace)
            yield (img2_aug2, mask)



def test_data_generator(test_path, image_folder, img_target_size=(480, 640),
                        tohsv=False):
    augs = {}
    if tohsv:
        augs['preprocessing_function'] = rgb_to_hsv

    test_datagen = ImageDataGenerator(**augs)

    image_gen = test_datagen.flow_from_directory(
        test_path,
        class_mode=None,
        classes=[image_folder],
        color_mode='rgb',
        target_size=img_target_size,
        batch_size=1,
        shuffle=False
    )

    colorspace = 'rgb'
    if tohsv:
        colorspace = 'hsv'

    for img in image_gen:
        img = normalize_image(img, colorspace=colorspace)
        yield img


def eval_generator(batch_size, test_path, image_folder, mask_folder,
                   img_target_size=(480, 640), tohsv=False):
    return train_generator(batch_size, test_path, image_folder, mask_folder,
                           img_target_size, {}, tohsv=tohsv, aug=False)


def load_data_memory(train_paths, image_folder, mask_folder, resize=(640, 480),
                     tohsv=False):
    X = []
    Y = []

    colorspace = 'rgb'
    if tohsv:
        colospace = 'hsv'

    for train_path in train_paths:
        for i in sorted(glob.glob(os.path.join(train_path, image_folder, '*.png'))):
            img = cv2.imread(i).astype(np.float32)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img = cv2.resize(img, resize)
            if tohsv:
                img = rgb_to_hsv(img)
            img = normalize_image(img, colorspace=colorspace)
            X.append(img)

        for i in sorted(glob.glob(os.path.join(train_path, mask_folder, '*.png'))):
            mask = cv2.imread(i, cv2.IMREAD_GRAYSCALE)
            mask = cv2.resize(mask, resize)
            mask = mask.reshape(resize[1], resize[0], 1)
            mask = fix_mask(mask)
            Y.append(mask)

    X = np.array(X)
    Y = np.array(Y)
    return (X, Y)



def save_predicted_images(path, test_image_folder, predictions, img_target_size, tohsv=False):
    os.makedirs(path, exist_ok=True)
    test_imgs = sorted(glob.glob(os.path.join(test_image_folder, '*.png')))

    for p, t in zip(predictions, test_imgs):
        p[p <= 0.5] = 0
        p[p > 0.5] = 255

        img0 = io.imread(t)
        img0 = transform.resize(img0, img_target_size)
        basename = os.path.basename(t)
        #print("p.shape: {}, img0.shape: {}".format(p.shape, img0.shape))    # p.shape: (480, 640, 1), img0.shape: (480, 640, 3)

        # fixme: grayscale
        img = img0
        if tohsv:
            img = rgb_to_hsv(img0)

        # save predicted mask
        #io.imsave(os.path.join(path, 'mask_{}'.format(basename)), p)

        red = np.ones(img.shape, dtype=np.float) * (1, 0, 0)
        transparency = .35
        p /= 255
        p *= transparency
        # red over original image
        out = red*p + img*(1.0-p)

        # save mask overlaying image
        io.imsave(os.path.join(path, '{}'.format(basename)), out)


if __name__ == '__main__':
    data_gen_args = dict(horizontal_flip=True)
    gen = train_generator(2, 'data/train', 'image',
                          'masks',
                          augs=data_gen_args,
                          tohsv=False)
    for (img, mask) in gen:
        print(img)
