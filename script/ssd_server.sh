#!/bin/bash

cd ~/caffe
id=$1
port=$(( 9900+id ))
while sleep 1; do
	# ./.build_release/examples/ssd/ssd_server.bin -network_port=$port models/VGGNet/VOC0712/SSD_300x300/deploy.prototxt models/VGGNet/VOC0712/SSD_300x300/VGG_VOC0712_SSD_300x300_iter_120000.caffemodel;
	./build/examples/ssd/ssd_server.bin  -network_port=$port models/VGGNet/VOC0712/SSD_300x300/deploy.prototxt models/VGGNet/VOC0712/SSD_300x300/VGG_VOC0712_SSD_300x300_iter_120000.caffemodel
done
