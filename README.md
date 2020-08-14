This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Team Members

- Georgios Fagogenis, georgios.fagogenis@tttech-auto.com
- Matias Fernando Pavez Bahamondes, matias.pavez-bahamondes@tttech-auto.com (Team Lead)
- Natalia Estefanía Jurado Espín, natalia.jurado@tttech-auto.com

## Resources

- [Installation](./INSTALL.md): Local/Docker setup, simulator, and dependencies.
- [Testing](#testing): How to test on the simulator and the real car data.
- [Implementation](#implementation): Design and implementation details.
- [Results](#results).
- [Acknowledgements](#acknowledgements).

## Testing

### Build

Compile the project locally or on the docker container:

```bash
cd ros
source /opt/ros/kinetic/setup.bash
catkin_make
source devel/setup.sh
```

### Simulator

1. Run the simulator locally: The camera must be enabled and the manual mode disabled.
2. Launch styx.launch locally or on the docker container.

```bash
cd ros
roslaunch launch/styx.launch
```

### Real World Data

1. Run the [Rosbag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) file that was recorded on the Udacity self-driving car:
2. Launch the project in your project in site mode

```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag

cd ros
roslaunch launch/site.launch
```

## Implementation

### Waypoints

- TODO: About decceleration profile for the trajectory.

### DBW Controller

- TODO: How the PID for throttle was tuned.
- TODO: Insights on the yaw controller.
- TODO: How to deal with stop and go scenarios.

### Trafic Light Detector

Labeled data, and training scripts for the traffic light detector were provided by user [vatsl](https://github.com/vatsl) on the repository [TrafficLight_Detection-TensorFlowAPI](https://github.com/vatsl/TrafficLight_Detection-TensorFlowAPI).

A different container matching the exact dependencies as in the real car was used for training the model. Instructions can be found in the respective [training section](./training/README.md).

- TODO: About the dataset.
- TODO: About the model.
- TODO: About the results for simulation.
- TODO: About the results for real car data.

## Results

- TODO: Add images/video.
- TODO: (Improvement - Corner case): About how dealing with yellow lights would improve the stop.

## Acknowledgements

- Most of the implementation for ROS nodes and controllers was obtained from Udacity classroom lessons.
- Labeled data, and training scripts for the traffic light detector were provided by user [vatsl](https://github.com/vatsl) on the repository [TrafficLight_Detection-TensorFlowAPI](https://github.com/vatsl/TrafficLight_Detection-TensorFlowAPI).
