# Installation

Please use **one** of the two installation options, either native **or** docker installation.

## Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.

Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```

## Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

1. Build the docker container:
```bash
docker build . -t capstone
```

2. Run the docker container, and name it `udacity`:
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --name udacity  --rm -it capstone
```

3. If extra terminals are needed, you can connect to the named container:
```bash
docker exec -it udacity /bin/bash

# Remember to source ROS if needed
source devel/setup.bash
```


## Simulator

* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

Run simulator locally:
```bash
# Setup
unzip linux_sys_int.zip
chmod +x linux_sys_int/sys_int.x86_64

# Run
./linux_sys_int/sys_int.x86_64
```

## Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.
