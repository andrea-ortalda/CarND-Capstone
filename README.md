This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Team Members
Luca Profumo

Andrea Ortalda

### The Project

![](record.gif)

In this project, the software for the Udacity self-driving car is implemented using ROS.

ROS nodes for the functionality integration as well as core algorithms for self-driving were implemented.

These are the main building blocks of the project that we implemented:
* Motion planning: Waypoint updater
* Control: Drive-by-wire controller
* Perception: Traffic light detector and classifier

#### Testing
The software was tested and tuned mainly on the Udacity simulator test track which contains both straight and curved road segments and several traffic lights.

Reprocessing tests with real bag files were also performed for the traffic light detection and classification.

Thanks to the tests performed in simulation and reprocessing, the software is ready for testing with test vehicle Carla by Udacity engineers.

#### Waypoint updater
The waypoint updater ROS node subscribes to the vehicle current pose, the base waypoints loaded from the map and the waypoints from the traffic light detector.

It publishes a set of final waypoints that the ego vehicle has to follow.

The core algorithm calculates the next waypoints to follow based on the vehicle current pose and updates their velocity to be able to stop at a stop line with red traffic light.

##### Code guidelines
See the waypoint_loader.py ROS node.

#### Drive-by-wire controller
The drive-by-wire controller node subscribes to the current vehicle velocity, the dbw enabled switch command and the twist command coming from the waypoint follower which contains target linear and angular velocity for the ego vehicle.

It publishes three commands for the vehicle: throttle percentage, brake torque and steering angle.
The throttle/brake decision is based on the difference between current velocity and target linear velocity. In case braking is needed, the braking torque is calculated from the target deceleration using vehicle characteristics (mass and wheel radius).

Controllers already implemented and tuned by Udacity engineers for throttle (PID) and steering angle (yaw controller) were used.

##### Code guidelines
* dbw_node.py is the ROS node for control
* twist_controller.py has the control algorithm code
* pid.py and yaw_controller.py are the actual controllers

#### Traffic light detector and classifier
The traffic light detector node subscribes to the current vehicle pose, the base waypoints, the waypoints of stop lines for each traffic lights from the map and the vehicle camera image stream.

It publishes the traffic waypoints containing the stop line waypoints when a red traffic light is detected.

The traffic waypoints are calculated searching the next stop line waypoints with respect to the ego vehicle current pose and they are published when a red traffic light is detected.

For detection and classification the traffic light classifier Class is used, which is using two different neural networks, one for traffic light box detection and extraction and the other for the traffic light state classification (red/yellow/green).

##### Traffic Light Detection
For the Traffic light detection, the [SSD MobileNet 11.6.17 version](http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_11_06_2017.tar.gz) was chosen analysing performances of other pre-trained models from the [Tensorflow model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md), for example [faster_rcnn_inception_v2_coco](http://download.tensorflow.org/models/object_detection/faster_rcnn_inception_v2_coco_2018_01_28.tar.gz). These models were higly performing, but they led to unwanted behavior due to the high FPS of the use-case. For this reason, a MobileNet that deals really well with high FPS was finally chosen. 

1. Real datum detection

<p align="center">
	<img src="/imgs/real_train_image.png" alt="Detect Real Datum"
	title="Detect Real Datum"  />
</p>

2. Simulation datum detection

<p align="center">
	<img src="/imgs/sim_data_train.png" alt="Detect Simulation Datum"
	title="Detect Simulation Datum"  />
</p>

##### Traffic Light Classification
After traffic light detection, classification on the processed image has to be performed. The architecure used in this project follows the structure provided by NVIDIA in their article [End to End Learning for Self-Driving Cars](https://arxiv.org/pdf/1604.07316v1.pdf).
This architecture was slightly modified with the addition of Dropout layers between Convolutional layers in oder to prevent overfitting.

[image0]: ./imgs/nvidia.png "Nvidia Architecture"
[image1]: ./imgs/architecture.png "Architecture Used"

| Nvidia Architecture |  Architecture Used  |
| :-----------------: | :-----------------: |
| ![alt text][image0] | ![alt text][image1] |

The following dictionare was created to match literal label to numeric encoding:

  label_dict = dict({'R' : 0, 'Y' : 1, 'G' : 2, 'O' : 3})

The neural network was trained using both real data and simulation data. Real data were taken from the [BOSCH simple traffic light dataset](https://hci.iwr.uni-heidelberg.de/node/6132), while simulation data were taken recording bag files along the track. A total of 4056 real images and 1804 simulation ones were used to train the network.
A splitting of 80%-20% was the choice for creating training and validation data sets, with the following result:

<p align="center">
	<img src="/imgs/model_classification.png" alt="Classification Model Result"
	title="Classification Model Result"  />
</p>

The model was then simply tested with an image per dataset, including the Udacity record images that were just used for testing purposes:

1. Real datum
<p align="center">
	<img src="/imgs/classify_real.png" alt="Classify Real datum"
	title="Classify Real datum"  />
</p>

2. Simulation datum

<p align="center">
	<img src="/imgs/classify_sim.png" alt="Classify Simulation datum"
	title="Classify Simulation datum"  />
</p>

3. Miscellaneous datum

<p align="center">
	<img src="/imgs/classify_train_data.png" alt="Classify Miscellaneous datum"
	title="Classify Miscellaneous datum"  />
</p>

4. Udacity Record datum

<p align="center">
	<img src="/imgs/classify_record.png" alt="Classify Udacity Record datum"
	title="Classify Udacity Record datum"  />
</p>

##### Code guidelines
* tl_detector.py is the ROS node
* tl_classifier.py is the Class for traffic light detection and classification
* Traffic_Light_detector.ipynb is the Jupyter notebook for the neural networks training and testing

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
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

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|               | Simulator |  Carla  |
| :-----------: | :-------: | :-----: |
| Nvidia driver |  384.130  | 384.130 |
|     CUDA      |  8.0.61   | 8.0.61  |
|     cuDNN     |  6.0.21   | 6.0.21  |
|   TensorRT    |    N/A    |   N/A   |
|    OpenCV     | 3.2.0-dev |  2.4.8  |
|    OpenMP     |    N/A    |   N/A   |

We are working on a fix to line up the OpenCV versions between the two.
