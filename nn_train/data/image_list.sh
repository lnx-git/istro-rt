#!/bin/bash
cd val/image/
pwd > ../../image_val.txt
ls -la >> ../../image_val.txt
cd ../..
cd train/image/
pwd > ../../image_train.txt
ls -la >> ../../image_train.txt 
cd ../..
