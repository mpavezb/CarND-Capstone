This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Team Members

- Georgios Fagogenis, georgios.fagogenis@tttech-auto.com
- Matias Fernando Pavez Bahamondes, matias.pavez-bahamondes@tttech-auto.com (Team Lead)
- Natalia Estefanía Jurado Espín, natalia.jurado@tttech-auto.com

## Resources

- [Installation](./INSTALL.md): Local/Docker setup, simulator, and dependencies.
- [Testing](#testing): How to test on the simulator and the real car data.
- [Design](#design): TODO
- [Results](#results): TODO
- [Acknowledgements](#acknowledgements).


## Testing

### Simulator

1. Run the simulator locally: The camera must be enabled and the manual mode disabled.

2. Build and launch styx.launch locally or on the docker container:
```bash
# Build
cd ros
source /opt/ros/kinetic/setup.bash
catkin_make

# Launch
source devel/setup.sh
roslaunch launch/styx.launch
```

### Real World Data

1. Prepare the [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

## Design

TODO.
- [Traffic Light Detector Training](./training/README.md)

## Results

TODO

## Acknowledgements

- Most of the implementation for ROS nodes and controllers was obtained from Udacity classroom lessons.
- Labeled data, and training scripts for the traffic light detector were provided by user [vatsl](https://github.com/vatsl) on the repository [TrafficLight_Detection-TensorFlowAPI](https://github.com/vatsl/TrafficLight_Detection-TensorFlowAPI).
