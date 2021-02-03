import time
import cv2 as cv
import numpy as np
import tensorflow.compat.v1 as tf

logger = None

tf_sess = None
input_tensor_name = None
output_tensor_name = None

### mtime ###

def mtime_begin():
    return int(time.time() * 1000)

def mtime_delta(t):
    # return time difference in milliseconds
    return int(mtime_begin() - t)

def mtime_delta2(t1, t2):
    return int(t2 - t1)

def mtime_end(ss, t):
    logger.debug('visionn_server::mtime(): m="' + ss + '", dt=' + str(mtime_delta(t)))

### model ###

def model_init(log):
    global logger
    global tf_sess
    global input_tensor_name
    global output_tensor_name

    logger = log

    input_names = ['input_1']
    output_names = ['convLast/Sigmoid']

    # Read Frozen Graph file from disk.
    logger.info('visionn_model::model_init(): msg="read file..."')
    #print('read file...')
    with tf.gfile.GFile("model.pb", "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())

    # Print graph input size
    for node in graph_def.node:
        if 'input_' in node.name:
            size = node.attr['shape'].shape
            image_size = [size.dim[i].size for i in range(1, 4)]
            break
    logger.info('visionn_model::model_init(): image_size="' + str(image_size) + '"')
    #print("image_size: {}".format(image_size))    # image_size: [480, 640, 3]

    # Set input and output tensor names
    input_tensor_name = input_names[0] + ":0"
    output_tensor_name = output_names[0] + ":0"

    logger.info('visionn_model::model_init(): input_tensor_name="' + input_tensor_name + '", output_tensor_name="' + output_tensor_name +'"')
    #print("input_tensor_name: {}\noutput_tensor_name: {}".format(input_tensor_name, output_tensor_name))

    # Create session and load graph
    #print('import graph...')
    logger.info('visionn_model::model_init(): msg="import graph - start..."')
    tf_config = tf.ConfigProto()
    tf_config.gpu_options.allow_growth = True
    tf_sess = tf.Session(config=tf_config)
    tf.import_graph_def(graph_def, name='')
    logger.info('visionn_model::model_init(): msg="import graph - finished"')
    return tf_sess

def model_predict(img):
    global tf_sess
    global input_tensor_name
    global output_tensor_name

    # Load image
    #print('load image...')
    #cv_image = cv.imread('testing_image.png')

    t0 = mtime_begin()
    img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    img_np = np.expand_dims(img_rgb, axis=0)
    img_np = img_np / 255
    #print("img.shape: {}, img_np.shape: {}".format(img.shape, img_np.shape))    # img.shape: (480, 640, 3), img_np.shape: (1, 480, 640, 3)

    t1 = mtime_begin()
    pred_np = tf_sess.run(output_tensor_name, feed_dict={input_tensor_name: img_np})    # predict using TF session

    t2 = mtime_begin()
    pred_img = np.squeeze(pred_np, axis=0)
    #print("pred_np.shape: {}, pred_img.shape: {}".format(pred_np.shape, pred_img.shape))    # pred_np.shape: (1, 480, 640, 1), pred_img.shape: (480, 640, 1)

    pred_img[pred_img <= 0.5] = 0
    pred_img[pred_img > 0.5] = 255
    pred_img = pred_img.astype(np.uint8)
    #cv.imwrite("testing_mask.png", pred_img)

    #print('finished...')
    logger.debug('visionn_model::model_predict(): dt1=' + str(mtime_delta2(t0, t1)) + ', dt2=' + str(mtime_delta2(t1, t2)) + ', dt3=' + str(mtime_delta(t2)))
    return pred_img
