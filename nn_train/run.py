import datetime
import glob
import os

import click
from matplotlib import pyplot as plt
plt.switch_backend('agg')

import keras.backend as K
import keras.callbacks
from keras.callbacks import ModelCheckpoint, EarlyStopping, CSVLogger

import segmentation_models as sm

from data_generator import (
    train_generator,
    test_data_generator,
    eval_generator,
    save_predicted_images,
    load_data_memory,
)
import models

AVAILABLE_MODELS = ['unet', 'fcn_vgg16_32s', 'segnet', 'resnet',
                    'resnet_bnn',
                    'unet_resnet34', 'unet_resnet50', 'unet_vgg16',
                    'linknet_vgg16', 'linknet_resnet50',
                    'fpn_resnet34', 'fpn_resnet50', 'fpn_vgg16']
MODEL_MAPPING = {
    'unet': models.unet,
    'fcn_vgg16_32s': models.fcn_vgg16_32s,
    'segnet': models.segnet,
    'resnet': models.resnet,
    'resnet_bnn': models.resnet_bnn,
 
    'unet_resnet34': models.unet_resnet34,
    'unet_resnet50': models.unet_resnet50,
    'unet_vgg16': models.unet_vgg16,

    'linknet_vgg16': models.linknet_vgg16,
    'linknet_resnet50': models.linknet_resnet50,

    'fpn_resnet34': models.fpn_resnet34,
    'fpn_resnet50': models.fpn_resnet50,
    'fpn_vgg16': models.fpn_vgg16
}


IMG_TARGET_SIZE = (480, 640)
RESIZE_TO = tuple(reversed(IMG_TARGET_SIZE))
INPUT_SIZE = IMG_TARGET_SIZE + (3,)
#BATCH_SIZE = 4
#BATCH_SIZE = 16
BATCH_SIZE = 6
N_TRAIN_SAMPLES = len(glob.glob('data/train/image/*.png', recursive=False))
N_VAL_SAMPLES = len(glob.glob('data/val/image/*.png', recursive=False))
N_TEST_SAMPLES = len(glob.glob('data/test/image/*.png', recursive=False))
LOSS = models.metrics.dice_coef_loss #'binary_crossentropy'
#LOSS = 'binary_crossentropy'

PRODUCTION_DATASET = 'data/datasets/deggendorf'
TRAIN_DIRECTORY = 'data/train'

class LossHistory(keras.callbacks.Callback):
    def on_train_begin(self, logs={}):
        self.losses = []

    def on_batch_end(self, batch, logs={}):
        self.losses.append(logs.get('loss'))


@click.group()
def cli():
    pass


@click.command(help='Train specified model')
@click.option('--model', '-m', type=click.Choice(AVAILABLE_MODELS),
              required=True, help='Model to train')
@click.option('--gen', '-g', type=bool, default=True,
              help='Whether to use generator for feeding data')
@click.option('--plot', '-p', type=bool, default=True,
              help='Wheter to plot losses and accuracies')
@click.option('--aug', '-a', type=bool, default=False,
              help='Whether to use data augmentation')
@click.option('--epochs', '-e', type=int, default=3,
              help='Number of epochs')
@click.option('--hsv', '-h', type=bool, default=False,
              help='Whether to convert rgb image to hsv')
@click.option('--weights', '-w', type=str, default='',
              help='Path pretrained weights')
@click.option('--production', type=bool, default=False,
              help='Whether to train on entire dataset for production')
def train(model, gen, plot, aug, epochs, hsv, weights, production):
    global N_TRAIN_SAMPLES
    date = datetime.datetime.now()
    now_str = date.strftime('%Y-%m-%d-%H%M%S')

    model_filename = '{}_{}_{}_{}_{}x{}_{}'.format(model,
                                          'gen' if gen else 'nogen',
                                          'aug' if aug else 'noaug',
                                          'hsv' if hsv else 'rgb',
                                          IMG_TARGET_SIZE[1],
                                          IMG_TARGET_SIZE[0],
                                          now_str)

    model_out = 'trained_models/{}.hdf5'.format(model_filename)
    model_checkpoint = ModelCheckpoint(model_out, monitor='val_loss',
                                       verbose=1, save_best_only=True)
#   early_stopping = EarlyStopping(monitor='val_loss', patience=int(epochs*0.05))
    early_stopping = EarlyStopping(monitor='val_loss', patience=int(epochs*1.00))

    csv_logger = CSVLogger('training.log')
    lhistory = LossHistory()

    _model = MODEL_MAPPING[model](input_size=INPUT_SIZE,
                                  loss=LOSS)

    if weights != '':
        _model.load_weights(weights)

    train_dir = TRAIN_DIRECTORY
    if production:
        train_dir = PRODUCTION_DATASET
        N_TRAIN_SAMPLES = len(glob.glob(os.path.join(train_dir,
                                                     'image/*.png'),
                                        recursive=False))

    if gen:
        data_gen_args = {}
        if aug:
            data_gen_args = dict(fill_mode='constant',
                                 #zoom_range=0.05,
                                 rotation_range=5,
                                 vertical_flip=False,
                                 horizontal_flip=True)

        my_data_gen = train_generator(BATCH_SIZE, train_dir,
                                      'image',
                                      'masks',
                                      img_target_size=IMG_TARGET_SIZE,
                                      augs=data_gen_args,
                                      tohsv=hsv,
                                      aug=aug)

        X_val, Y_val = load_data_memory(['data/val'], 'image', 'masks',
                                        resize=RESIZE_TO, tohsv=hsv)

        steps_per_epoch = N_TRAIN_SAMPLES // BATCH_SIZE
        if aug:
            steps_per_epoch *= 4
        else:
            steps_per_epoch *= 2

        history = _model.fit_generator(my_data_gen,
                                       steps_per_epoch=steps_per_epoch,
                                       epochs=epochs,
                                       callbacks=[model_checkpoint, early_stopping, csv_logger, lhistory],
                                       validation_data=(X_val, Y_val))
    # mainly just for testing purposes
    else:
        X_train, Y_train = load_data_memory(['data/train', 'data/val'],
                                            'image', 'masks',
                                            resize=RESIZE_TO,
                                            tohsv=hsv)

        history = _model.fit(X_train, Y_train,
                             epochs=epochs,
                             batch_size=BATCH_SIZE,
                             callbacks=[model_checkpoint, early_stopping, lhistory],
                             validation_split=0.1)

    if plot:
        plt.figure()
        plt.plot(history.history['loss'], label='training loss')
        plt.plot(history.history['val_loss'], label='validation loss')
        plt.legend(loc='best')
        plt.savefig('plots/loss_{}.png'.format(model_filename))

        plt.figure()
        plt.plot(history.history['acc'], label='train accuracy')
        plt.plot(history.history['val_acc'], label='validation accuracy')
        plt.plot(history.history['mean_iou'], label='train mean IoU')
        plt.plot(history.history['val_mean_iou'], label='validation mean IoU')
        plt.legend(loc='best')
        plt.savefig('plots/acc_{}.png'.format(model_filename))

    f = open("training_loss.log", "w+")
    for l in lhistory.losses:
        f.write('{}\n'.format(l))
    f.close()

@click.command(help='Evaluate specified model on a test set')
@click.option('--model', '-m', type=click.Choice(AVAILABLE_MODELS),
              required=True, help='Model to evaluate')
@click.option('--path', '-p', help='Path to saved model')
@click.option('--hsv', '-h', type=bool, default=False,
              help='Whether to convert rgb image to hsv')
def evaluate(model, path, hsv):
    _model = MODEL_MAPPING[model]
    _model = _model(pretrained_weights=path, input_size=INPUT_SIZE,
                    loss=LOSS)
    eval_gen = eval_generator(1, 'data/test', 'image', 'masks',
                              img_target_size=IMG_TARGET_SIZE,
                              tohsv=hsv)

    loss, acc, miou = _model.evaluate_generator(eval_gen, steps=N_TEST_SAMPLES,
                                                verbose=0)
    print('=======================================')
    print('Evaluation results')
    print('Loss: {}, Acc: {}, Mean IoU: {}'.format(loss, acc, miou))
    print('=======================================')

import time

@click.command(help='Evaluate specified model on a test set')
@click.option('--model', '-m', type=click.Choice(AVAILABLE_MODELS),
              required=True, help='Model to evaluate')
@click.option('--path', '-p', help='Path to saved model')
@click.option('--hsv', '-h', type=bool, default=False,
              help='Whether to convert rgb image to hsv')
def predict(model, path, hsv):
    _model = MODEL_MAPPING[model]
    _model = _model(pretrained_weights=path, input_size=INPUT_SIZE,
                    loss=LOSS)
    test_gen = test_data_generator('data/test', 'image',
                                   img_target_size=IMG_TARGET_SIZE,
                                   tohsv=hsv)
    results = _model.predict_generator(test_gen, steps=N_TEST_SAMPLES,
                                       verbose=1)

    save_predicted_images('data/predictions', 'data/test/image', results,
                          IMG_TARGET_SIZE, tohsv=hsv)


# https://www.dlology.com/blog/how-to-run-keras-model-on-jetson-nano/
import tensorflow.compat.v1 as tf
import numpy as np
import skimage.io as io
#from tensorflow.python.framework import graph_io
from tensorflow.keras.models import load_model
from keras.models import model_from_json
from tensorflow.keras.preprocessing import image

def freeze_graph(graph, session, output, save_pb_dir='.', save_pb_name='model.pb', save_pb_as_text=False):
    with graph.as_default():
        graphdef_inf = tf.graph_util.remove_training_nodes(graph.as_graph_def())
        graphdef_frozen = tf.graph_util.convert_variables_to_constants(session, graphdef_inf, output)
        #graph_io.write_graph(graphdef_frozen, save_pb_dir, save_pb_name, as_text=save_pb_as_text)
        tf.train.write_graph(graphdef_frozen, save_pb_dir, save_pb_name, as_text=save_pb_as_text)
        return graphdef_frozen

def freeze_session(session, keep_var_names=None, output_names=None, clear_devices=True):
    #https://stackoverflow.com/questions/50027652/tensorflow-error-attempting-to-use-uninitialized-value-beta1-power-18
    #print("uninit params:")
    #print(session.run(tf.report_uninitialized_variables()))
    #session.run(tf.global_variables_initializer())

    graph = session.graph
    with graph.as_default():
        freeze_var_names = list(set(v.op.name for v in tf.global_variables()).difference(keep_var_names or []))
        output_names = output_names or []
        output_names += [v.op.name for v in tf.global_variables()]
        #input_graph_def = graph.as_graph_def()
        input_graph_def = tf.graph_util.remove_training_nodes(graph.as_graph_def())
        if clear_devices:
            for node in input_graph_def.node:
                node.device = ""
        frozen_graph = tf.graph_util.convert_variables_to_constants(
            session, input_graph_def, output_names, freeze_var_names)
        return frozen_graph

def save_images(img, predictions, file_name_image, file_name_mask):
    p = predictions
    # Save output image
    p[p <= 0.5] = 0
    p[p > 0.5] = 255
    #p *= 255
    # save predicted mask
    io.imsave(file_name_mask, p)

    red = np.ones(img.shape, dtype=np.float) * (255, 0, 0)
    transparency = .35
    p /= 255
    p *= transparency
    # red over original image
    out = red*p + img*(1.0-p)

    # save mask overlaying image
    io.imsave(file_name_image, out)

@click.command(help='Freeze model and save it into .pb format')
@click.option('--model', '-m', type=click.Choice(AVAILABLE_MODELS),
              required=True, help='Model to evaluate')
@click.option('--path', '-p', help='Path to saved model')
@click.option('--hsv', '-h', type=bool, default=False,
              help='Whether to convert rgb image to hsv')
def freeze(model, path, hsv):
    _model = MODEL_MAPPING[model]
    _model = _model(pretrained_weights=path, input_size=INPUT_SIZE,
                    loss=LOSS)

    input_names = [t.op.name for t in _model.inputs]
    output_names = [t.op.name for t in _model.outputs]
    print('input names: {}\noutput_names: {}'.format(input_names, output_names))    # input_names: ['input_1'], output_names: ['convLast/Sigmoid']

    # Set input and output tensor names
    input_tensor_name = input_names[0] + ":0"
    output_tensor_name = output_names[0] + ":0"
    print("input_tensor_name: {}\noutput_tensor_name: {}".format(input_tensor_name, output_tensor_name))

    # Assign output_sensor
    tf_sess = K.get_session()
    #input_tensor = tf_sess.graph.get_tensor_by_name(input_tensor_name)
    #output_tensor = tf_sess.graph.get_tensor_by_name(output_tensor_name)

    # Load image
    print('load image...')
    image_size = [480, 640, 3]
    img = image.load_img('data/test0/IMG_20180708_144527_image.png', target_size=image_size[:2])
    x0 = image.img_to_array(img)
    x = np.expand_dims(x0, axis=0)    # x0.shape: (480, 640, 3), x.shape: (1, 480, 640, 3)
    print("x0.shape: {}, x.shape: {}".format(x0.shape, x.shape))
    x = x / 255    # normalize_image('rgb')

    # Calculate predictions
    print('Keras: inference...')
    for i in range(5):
        start_time = time.time()
        p0 = _model.predict(x)    # predict using keras model
        duration = time.time() - start_time
        print("duration: {}".format(duration))
    p = np.squeeze(p0, axis=0)
    print("Keras: p0.shape: {}, p.shape: {}".format(p0.shape, p.shape))    # p0.shape: (1, 480, 640, 1), p.shape: (480, 640, 1)

    print('Keras: save images...')
    save_images(x0, p, 'data/freeze_1k_image.png', 'data/freeze_1k_mask.png')

    print('TF graph: inference...')
    #print(tf.report_uninitialized_variables())
    #tf_sess.run(tf.global_variables_initializer())    # toto sa nesmie vykonat!!
    for i in range(5):
        start_time = time.time()
        p0 = tf_sess.run(output_tensor_name, feed_dict={input_tensor_name: x})    # predict using TF session
        duration = time.time() - start_time
        print("duration: {}".format(duration))
    p = np.squeeze(p0, axis=0)
    print("TF graph: p0.shape: {}, p.shape: {}".format(p0.shape, p.shape))    # p0.shape: (1, 480, 640, 1), p.shape: (480, 640, 1)

    print('TF graph: save images...')
    save_images(x0, p, 'data/freeze_2g_image.png', 'data/freeze_2g_mask.png')

    print('freeze model...')
    frozen_graph = freeze_graph(tf_sess.graph, tf_sess, output_names, './trained_models')
    #frozen_graph = freeze_session(tf_sess, output_names)
    #print('save frozen model...')
    #tf.train.write_graph(frozen_graph, './trained_models', 'model.pb', as_text=False)

    print('TF frozen: inference...')
    p0 = tf_sess.run(output_tensor_name, feed_dict={input_tensor_name: x})    # predict using TF session
    p = np.squeeze(p0, axis=0)
    print("TF frozen: p0.shape: {}, p.shape: {}".format(p0.shape, p.shape))    # p0.shape: (1, 480, 640, 1), p.shape: (480, 640, 1)

    print('TF frozen: save images...')
    save_images(x0, p, 'data/freeze_3f_image.png', 'data/freeze_3f_mask.png')

    print('finished...')

#import uff
#import tensorrt as trt    # does not support "create_inference_graph"
import tensorflow.contrib.tensorrt as trt

@click.command(help='Convert specified model to TensorRT format')
@click.option('--path', '-p', help='Path to saved model')
def convert(path):
    input_names = ['input_1']
    output_names = ['convLast/Sigmoid']    # resnet
    #output_names = ['conv2d_24/Sigmoid']    # unet
    #output_names = ['activation_26/Sigmoid']    # segnet

    # Read Frozen Graph file from disk.
    print('read file...')
    with tf.gfile.GFile(path, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())

    # Print graph input size
    for node in graph_def.node:
        if 'input_' in node.name:
            size = node.attr['shape'].shape
            image_size = [size.dim[i].size for i in range(1, 4)]
            break
    print("image_size: {}".format(image_size))    # image_size: [480, 640, 3]

    # Set input and output tensor names
    input_tensor_name = input_names[0] + ":0"
    output_tensor_name = output_names[0] + ":0"
    print("input_tensor_name: {}\noutput_tensor_name: {}".format(input_tensor_name, output_tensor_name))

    # Create session and load graph
    print('import graph...')
    tf_config = tf.ConfigProto()
    tf_config.gpu_options.allow_growth = True
    tf_sess = tf.Session(config=tf_config)
    tf.import_graph_def(graph_def, name='')

    # Assign output_sensor
    output_tensor = tf_sess.graph.get_tensor_by_name(output_tensor_name)

    # Load image
    print('load image...')
    img = image.load_img('data/test0/IMG_20180708_144527_image.png', target_size=image_size[:2])
    x0 = image.img_to_array(img)
    x = np.expand_dims(x0, axis=0)    # x0.shape: (480, 640, 3), x.shape: (1, 480, 640, 3)
    print("x0.shape: {}, x.shape: {}".format(x0.shape, x.shape))
    x = x / 255    # normalize_image('rgb')

    # Calculate predictions
    print('run...')
    for i in range(5):
        start_time = time.time()
        p0 = tf_sess.run(output_tensor_name, feed_dict={input_tensor_name: x})    # predict using TF session
        duration = time.time() - start_time
        print("duration: {}".format(duration))
    p = np.squeeze(p0, axis=0)
    print("p0.shape: {}, p.shape: {}".format(p0.shape, p.shape))    # p0.shape: (1, 480, 640, 1), p.shape: (480, 640, 1)

    print('save images...')
    save_images(x0, p, 'data/convert_image.png', 'data/convert_mask.png')

    print('TensorRT: optimize...')
    trt_graph = trt.create_inference_graph(
        input_graph_def=tf_sess.graph.as_graph_def(),
        outputs=output_names,
        max_batch_size=1,
        max_workspace_size_bytes=1 << 25,
        #precision_mode='FP16',
        precision_mode='INT8',
        minimum_segment_size=50
    )

    print('TensorRT: save model...')
    tf.train.write_graph(trt_graph, './trained_models', 'model_trt.pb', as_text=False)

    #print('write UFF model...')
    #uff_model = uff.from_tensorflow(graphdef=trt_graph, output_filename='./trained_models/model_trt.uff', output_nodes=output_names, text=False)

    print('finished...')


    # Clear any previous session.
    #tf.keras.backend.clear_session()
    # This line must be executed before loading Keras model.
    #tf.keras.backend.set_learning_phase(0) 

    #print('save model...')
    ##_model.save('model_pb/model.h5')
    #model_json = _model.to_json()
    #with open("model_pb/model.json", "w") as json_file:
    #    json_file.write(model_json)
    #_model.save_weights("model_pb/modelw.h5")

    ## Clear any previous session - before loading _model2
    #tf.keras.backend.clear_session()
    ## This line must be executed before loading Keras model.
    #tf.keras.backend.set_learning_phase(0)

    #print('load model...')
    ##_model2 = load_model('model_pb/model.h5')
    #json_file = open('model_pb/model.json', 'r')
    #loaded_model_json = json_file.read()
    #json_file.close()
    #_model2 = model_from_json(loaded_model_json)
    #_model2.load_weights("model_pb/modelw.h5")

    #print('initializer...')
    #with tf.Session() as sess:
    #    sess.run(tf.global_variables_initializer())


@click.command(help='Predict using PB file')
@click.option('--path', '-p', help='Path to saved model')
def predict2(path):
    # https://www.dlology.com/blog/how-to-run-keras-model-on-jetson-nano/
    # https://leimao.github.io/blog/Save-Load-Inference-From-TF-Frozen-Graph/
    input_names = ['input_1']
    output_names = ['convLast/Sigmoid']

    # Read Frozen Graph file from disk.
    print('read file...')
    with tf.gfile.GFile(path, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())

    # Print graph input size
    for node in graph_def.node:
        if 'input_' in node.name:
            size = node.attr['shape'].shape
            image_size = [size.dim[i].size for i in range(1, 4)]
            break
    print("image_size: {}".format(image_size))    # image_size: [480, 640, 3]

    # Set input and output tensor names
    input_tensor_name = input_names[0] + ":0"
    output_tensor_name = output_names[0] + ":0"
    print("input_tensor_name: {}\noutput_tensor_name: {}".format(input_tensor_name, output_tensor_name))

    # Create session and load graph
    print('import graph...')
    tf_config = tf.ConfigProto()
    tf_config.gpu_options.allow_growth = True
    tf_sess = tf.Session(config=tf_config)
    tf.import_graph_def(graph_def, name='')

    # Assign output_sensor
    output_tensor = tf_sess.graph.get_tensor_by_name(output_tensor_name)

    # Load image
    print('load image...')
    img = image.load_img('data/test0/IMG_20180708_144527_image.png', target_size=image_size[:2])
    x0 = image.img_to_array(img)
    x = np.expand_dims(x0, axis=0)    # x0.shape: (480, 640, 3), x.shape: (1, 480, 640, 3)
    print("x0.shape: {}, x.shape: {}".format(x0.shape, x.shape))
    x = x / 255    # normalize_image('rgb')

    # Calculate predictions
    print('run...')
    for i in range(5):
        start_time = time.time()
        p0 = tf_sess.run(output_tensor_name, feed_dict={input_tensor_name: x})    # predict using TF session
        duration = time.time() - start_time
        print("duration: {}".format(duration))
    p = np.squeeze(p0, axis=0)
    print("p0.shape: {}, p.shape: {}".format(p0.shape, p.shape))    # p0.shape: (1, 480, 640, 1), p.shape: (480, 640, 1)

    save_images(x0, p, 'data/predict2_image.png', 'data/predict2_mask.png')

    print('finished...')


cli.add_command(train)
cli.add_command(evaluate)
cli.add_command(predict)
cli.add_command(freeze)
cli.add_command(convert)
cli.add_command(predict2)

if __name__ == '__main__':
    cli()
