import glob
import os
import cv2
import numpy as np
import visionn_model

image_path = '/media/sf__shared/dataset/image'
pred_path = '/media/sf__shared/dataset/pred'

visionn_model.model_init()

os.makedirs(pred_path, exist_ok=True)

for image_name in sorted(glob.glob(os.path.join(image_path, '*.jpg'))):

    pred_basename = os.path.basename(image_name)
    # remove '_camera' suffix
    pred_basename = pred_basename.replace('_camera', '')
    pred_basename = pred_basename.replace('.jpg', '_pred.jpg')
    pred_name = os.path.join(pred_path, pred_basename)

    img = cv2.imread(image_name)
    p = visionn_model.model_predict(img)
    #cv2.imwrite(pred_name, p)

    p = p.astype(np.float)

    # save predicted mask
    #io.imsave(os.path.join(path, 'mask_{}'.format(basename)), p)

    red = np.ones(img.shape, dtype=np.float) * (0, 0, 1)
    transparency = .40
    p /= 255
    p *= transparency
    # red over original image
    out = red*p + (img/255)*(1.0-p)
    out *= 255

    # save mask overlaying image
    cv2.imwrite(pred_name, out)
