# Udacity capstone project dockerfile
# FROM nvidia/cuda:8.0-cudnn6-runtime
FROM tensorflow/tensorflow:1.3.0-gpu

# Fix debconf issue: A
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils

# Install Python dependencies
COPY requirements.txt ./requirements.txt
RUN apt-get update
RUN apt-get install -y python-pip
RUN pip install --upgrade pip
RUN pip install setuptools --upgrade
RUN pip install -r requirements.txt

# extra deps
RUN apt-get install -y protobuf-compiler python-tk
RUN pip install --upgrade protobuf

RUN mkdir /capstone_training
VOLUME ["/capstone_training"]
VOLUME ["/root/.ros/log/"]
WORKDIR /capstone_training/models/research

# Fix debconf issue: B
ENV DEBIAN_FRONTEND teletype