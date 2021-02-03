import cv2
import visionn_model

visionn_model.model_init()

img = cv2.imread('testing_image.png')

pred = visionn_model.model_predict(img)
pred = visionn_model.model_predict(img)
pred = visionn_model.model_predict(img)

# cv2.imwrite('testing_mask.png', pred)
