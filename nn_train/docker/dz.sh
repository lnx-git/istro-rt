#!/bin/bash
#docker run --gpus all -it --rm tensorflow/tensorflow:latest-gpu-jupyter bash
docker run -u $(id -u):$(id -g) --gpus all -it --rm -v /home/tf/smely_zajko/road-segmentation:/tf/road-segmentation -p 127.0.0.1:7001:7001/tcp tensorflow-szj:latest bash
