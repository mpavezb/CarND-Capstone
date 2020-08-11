# Training

## Requirements

### Docker Server

Following requirements must be satisfied by the server which will run the docker image:
- NVIDIA Driver >= 384.130
- Docker >= 19.03

See: https://github.com/NVIDIA/nvidia-docker

### Docker Image

Following dependencies are satisfied by the image itself:
- python 2.7.12
- pip 20.0.2
- CUDA == 8.0.61
- cuDNN == 6.0.21
- tensorflow-gpu == 1.3.0


## Preparations

### Download Required Files

```bash
cd trailing/
mkdir -p artifacts && cd artifacts

# Download tensorflow models
git clone https://github.com/tensorflow/models.git

# Download Inception Network
curl -OL http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_11_06_2017.tar.gz

# Download  TrafficLight_Detection-TensorflowAPI
git clone https://github.com/vatsl/TrafficLight_Detection-TensorFlowAPI

# Manually download labeled dataset
# https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view?usp=sharing
```

### Setup files

1. Uncompress all artifacts.
2. Copy tensorflow models repository folder to `training/` and do `git checkout f7e99c0` inside.
3. Copy ssd_inception to `training/models/research`
4. Copy contents of TensorflowAPI repo inside `training/models/research`
5. Copy dataset into `training/models/research`.

### Build Docker Image

```bash
# Build
docker build . -t capstone_training

# Run
docker run --gpus all -v $PWD:/capstone_training --name udacity_training --rm -it capstone_training bash

# Connect more terminals
docker exec -it udacity_training /bin/bash
```

## Train

While training you can monitor the CPU (`htop`) and GPU (`watch -n 0.5 nvidia-smi`) usage, and make sure the GPU is working as intended.

```bash
docker run --gpus all -v $PWD:/capstone_training --name udacity_training --rm -it capstone_training bash

# ?
protoc object_detection/protos/*.proto --python_out=.

# Set up PYTHONPATH on every terminal!
export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim
export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/object_detection

# Test it
python object_detection/builders/model_builder_test.py

# Create data files
python data_conversion_udacity_real.py --output_path real_data.record

# Training
python object_detection/train.py --pipeline_config_path=config/ssd_inception-traffic_udacity_real.config --train_dir=data/real_training_data

# Save model
python object_detection/export_inference_graph.py --pipeline_config_path=config/ssd_inception-traffic_udacity_real.config --trained_checkpoint_prefix=data/real_training_data/model.ckpt-<#> --output_directory=frozen_models/frozen_real_inception

python object_detection/export_inference_graph.py --pipeline_config_path=config/faster_rcnn-traffic_udacity_real.config --trained_checkpoint_prefix=data/real_training_data/model.ckpt-10000 --output_directory=frozen_real/

#if saving throws an (something related to mark flag required) just comment out the respective line. This is because at it's current state object detection has some compatibility issues with tensorflow 1.13 -> it should be fine if you are using 1.14
```

## References:

- Using Docker and NVIDIA: https://github.com/NVIDIA/nvidia-docker
- DockerHub: nvidia/cuda containers: https://hub.docker.com/r/nvidia/cuda/
- DockerHub: tensorflow containers: https://hub.docker.com/r/tensorflow/tensorflow/
- Docker: Supported NVIDIA containers: https://gitlab.com/nvidia/container-images/cuda/blob/master/doc/supported-tags.md
