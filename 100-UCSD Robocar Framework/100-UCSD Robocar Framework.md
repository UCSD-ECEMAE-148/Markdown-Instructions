# UCSD Robocar Framework

## 1. Introduction

## 2. UCSD Robocar Framework Breakdown

The UCSD Robocar Framework is a collection of ROS 2 packages for each of the hardware components used on the robocar (e.g. the camera, VESC, LiDAR, etc.). The Nav package acts as the "brain" of the collection since it interacts with each of the other independent packages.

Having standalone packages instead of one major package makes deployment more robust. Additionally, as the robot becomes more sophisticated, the number of associated packages would likely increase to achieve many different types of tasks depending on the application.

So the idea is to develop a package that could in general be used on any car-like robot as well as being able to choose what packages your robot really needs without having to use the entire framework.

For example, lets say another company developed their own similar sensor, actuator and nav packages but they have not researched into lane detection. Instead of using the entire UCSD Robocar framework, they could easily just deploy the lane detection package and have some interpreter in their framework read the messages from the lane detection package to suit their needs.

Link to the official git repo: [**ROS 2**](https://gitlab.com/ucsd_robocar2/ucsd_robocar_hub2)

Note: The hub2 package is a *metapackage*. For specific details about any individual package, click on any of the packages in either hub to be taken to that packages' main repository.

### 2.1 Packages

Each UCSD ROS package has a README.md that explains in detail what config, nodes, launch files it has as well as topic/message information. When troubleshooting, consider outlining what problem you are having and what package that most likely the cause of such an error. Then reference the README for that package.

#### 2.1.1 Nav

The navigation package (nav_pkg) is the "brain" of the UCSD Robocar framework because it keeps all the launch files in its package to launch any node/launch file from the other packages used in the framework. This makes using the framework easier because you only really have to remember the name of the nav_pkg and what launch file you want to use rather than having to remember all the other package names and their own unique launch files.

[NAV2 README](https://gitlab.com/ucsd_robocar2/ucsd_robocar_nav2_pkg/-/blob/master/README.md)

#### 2.1.2 Lane Detection

The lane detection package is one method of navigating by identifying and tracking road markers. The basic principle behind this package is to detect road markers using openCV and then compute whats called the “cross-track-error” which is the difference between the center axis of the car and the centroid (center of “mass”) of the road mark which is then fed into a PID controller for tracking.

[Lane Detection2 README](https://gitlab.com/ucsd_robocar2/ucsd_robocar_lane_detection2_pkg/-/blob/master/README.md)

#### 2.1.3 Sensor

The sensor package contains all the required nodes/launch files needed to use the sensors that are equipped to the car.

[Sensor2 README](https://gitlab.com/ucsd_robocar2/ucsd_robocar_sensor2_pkg/-/blob/master/README.md)

#### 2.1.4 Actuator

The actuator package contains all the required nodes/launch files needed to use the actuators that are equipped to the car.

[Actuator2 README](https://gitlab.com/ucsd_robocar2/ucsd_robocar_actuator2_pkg/-/blob/master/README.md)

#### 2.1.7 Basics

The path package contains all the required nodes/launch files needed to subscribe/publish to the sensor/actuator messages within the framework for fast algorithm prototyping

[Basics2 README](https://gitlab.com/ucsd_robocar2/ucsd_robocar_basics2_pkg/-/blob/master/README.md)

### 2.2 Updating All Packages

A utility function was added to the ```~/.bashrc``` script that will automatically update all the packages in the framework and then rebuild and source it so it will be ready to start using ROS2!

To do so, in your terminal:
```
upd_ucsd_robocar
```
### 2.3 Launch Files

The launch file diagrams below show the very general approach of how the packages communicate with one another. With ROS, it just comes down to a combination of starting launch files and sending messages (through topics) to nodes. For specific details about messages types, topics, services and launch files used, please go to the readme for the specific package of interest!

The nav_pkg is at the base of each of the diagrams and rooting from it are the launch files it calls that will launch other nodes/launch files from all the other packages in the framework.

In ROS2, a dynamically built launch file (at run-time) is used to launch all the different nodes/launch files for various purposes such as data collection, navigation algorithms and controllers. This new way of creating launch files has now been simplified by just adding an entry to a yaml file of where the launch file is and a separate yaml file to indicate to use that launch file or not. There is only one file to modify and all that needs to be changed is either putting a “0” or a “1” next to the list of nodes/launch files. To select the nodes that you want to use, put a “1” next to it otherwise put a “0” which means it will not activate. In the figures below, instead of including the entire ros2 launch command, you will only see the names of the launch files that need to be turned on in the node config file explained more in detail [here](https://docs.google.com/document/d/1ygzU4uE3p38XZT8Rey_QmykXRS-VA-rtt_XBkC3gpCE/edit#heading=h.3ucils4zejvp)

![alt text](image.png)

ROS2-FOXY: ```all_components.launch.py, sensor_vizualization.launch.py```


![alt text](image-1.png)

ROS2-FOXY: ```all_components.launch.py, teleop_joy_vesc_launch.launch.py```

![alt text](image-2.png)

ROS2-FOXY: ```all_components.launch.py, camera_nav_calibration.launch.py```


## 3. Developer Tools

### 3.1 ROS Guidebooks

Links provided below are guides for ROS and ROS2 which include many examples, terminal commands and general concept explanations of the various features in ROS and ROS2.

* [UCSD ROS Guidebook](https://docs.google.com/document/d/1u7XS7B-Rl_emK3kVKEfc0MxHtwXGYHf5HfLlnX8Ydiw/edit)
* [UCSD ROS2 Guidebook](https://docs.google.com/document/d/1DJgVLnu_vN-IXKD3QrQVF3W-JC6RiQPVugHeFAioB58/edit?usp=sharing)

### 3.2 Gitlab

Since the framework uses a meta package (a package that contains multiple packages) we refer to individual packages as submodules.

#### 3.2.1 Adding New Submodules

1. ```git submodule add <remote_url>```
2. ```git commit -m "message"```
3. ```git push```

#### 3.2.2 Updating local submodules with remote submodules

#### 3.2.3 Updating remote submodules with local submodules

#### 3.2.4 Removing submodules

#### 3.2.5 Adding an existing package to git

### 3.3 Docker

Below is a go-to list of docker commands that can be used with the framework:

Some new lingo:
* Container name: **NAMES**

* Image name: **REPOSITORY**

* Image tag ID (comparable to branches in git): **TAG**

#### 3.3.1 Pulling/Running

* pulling image from docker hub: ```docker pull REPOSITORY:TAG```

* starting a stopped container: ```docker start NAMES```

* stopping a container: docker stop NAMES

* Using multiple terminals for a single docker container: ```docker exec -it NAMES bash```

* build docker image and git it a new name and tag: ```docker build -t REPOSITORY:TAG .```

#### 3.3.2 Updating/Creating/Sharing

* Saving changes made while in a container to the original image (change tag to create a new image): ```docker commit name_of_container REPOSITORY:TAG```

* Create a new image from a container: ```docker tag NAMES REPOSITORY:TAG```

* Pushing an image to Dockerhub: ```docker push REPOSITORY:TAG```

* Share files between host and Docker container:
  * From **host** to docker container: ```docker cp foo.txt container_id:/foo.txt```
  * From **docker container** to host: ```docker cp container_id:/foo.txt foo.txt```

#### 3.3.3 Listing

* list all images: ```docker images```

* list all running containers: ```docker ps```

* list all containers (including stopped): ```docker ps -a```

#### 3.3.4 Deleting

* delete specific container: ```docker rm NAMES```

* delete specific image: ```docker rmi REPOSITORY:TAG```

* delete ALL containers: ```docker rm -f $(docker ps -a -q)```

* delete ALL images: ```docker rmi -f $(docker images -q)```


## 4. Accessing Docker Images

### 4.1 UCSD Robocar Image

Link to image on Docker Hub: [Docker Image](https://hub.docker.com/r/djnighti/ucsd_robocar)

**Computer Architecture: ARM (Jetson)**

To pull the image from a terminal:
```
docker pull djnighti/ucsd_robocar:latest
```

### 4.2 Docker Setup

The exact "recipe" to build this image can be found [here](https://gitlab.com/ucsd_robocar2/ucsd_robocar_hub2/-/blob/master/docker_setup/docker_files/Dockerfile)

If using the virtual machine, this has already been done for you.

#### 4.2.1 Enable X_11 Port Forwarding

1. On your HOST machine (not the Jetson) enter these commands (Will have to enter every time)
```
xhost +
ssh -X jetson@ip_address
```

2. Now on the Jetson, run the following commands to obtain sudo access for docker commands (only needs to be ran once)
```
sudo usermod -aG docker ${USER}
su ${USER}
```

3. Now check that if X_11 forwarding is working:

```
xeyes
```

If some googly eyes pop up, X_11 is ready to go. IF X_11 PORT FORWARDING IS NOT SETUP, follow steps [here](https://gitlab.com/djnighti/ucsd_robo_car_simple_ros/-/blob/master/x11_forwarding_steps.txt) to get it set up. Then come back here to continue the steps below.

#### 4.2.2 Update Docker Daemon

1. Now modify the Docker ```daemon.json``` file (just delete the previous version, then create a new one)
```
sudo rm /etc/docker/daemon.json 
sudo nano /etc/docker/daemon.json
```
2. Within the empty ```daemon.json``` file, add:
```
{
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
}

```

3. Save changes to the file and reboot the Jetson:
```
sudo reboot now
```

#### 4.2.3 Running a Container

1. SSH back into the Jetson with the -X flag which enables X_11 Forwarding
```
ssh -X jetson@ip_address
```

2. Create a new function in the ~/.bashrc file with command line arguments to easily run a container
```
gedit ~/.bashrc
```
or 
```
nano ~/.bashrc
```

3. Copy this into the bottom of the .bashrc:
```
robocar_docker ()
{
    docker run \
    --name ${1}\
    -it \
    --privileged \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    --device /dev/video0 \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    djnighti/ucsd_robocar:${2:-devel}
}

```

**Notice the two arguments we have made for the bash command:**

\${1}: This will be the name of the container, ex. Name_this_container

\${2:devel}: This is the tag id of the image you want to launch a container from. If nothing is specified when calling at the command line (example shown below), the “devel” tag will be run. 

Don't modify the bash function &mdash; the arguments are intentional and are not meant to be hard-coded.

4. Source the ~/.bashrc script so the current terminal can see the new function we just added
```
source ~/.bashrc
```

5. Run the following command to enter the docker container
```
robocar_docker <CONTAINER_NAME>
```
6. To access the **same** docker container from another terminal (do this for as many terminals you want)
```
docker exec -it <CONTAINER_NAME> bash
```

At this point the docker setup is complete but don't forget to refer to the useful docker commands sections which includes deleting, creating and updating images locally and remotely.

### 4.3 Workspaces in Docker Container

#### 4.3.2 ros2_ws

ROS version: ROS2-FOXY

This workspace contains source compiled packages from [ucsd_robocar_hub2](https://gitlab.com/ucsd_robocar2/ucsd_robocar_hub2)

#### 4.3.3 sensor2_ws

ROS version: ROS2-FOXY

This workspace contains source compiled packages for various sensors in our inventory.

### 4.4 ROS Bridge

### 4.5 Utility functions in ```~/.bashrc```

* [Updating all packaging in the ucsd_robocar framework from gitlab:](https://docs.google.com/document/d/1ygzU4uE3p38XZT8Rey_QmykXRS-VA-rtt_XBkC3gpCE/edit#heading=h.dydr21q8ok61) ```upd_ucsd_robocar```
* [Source Noetic and ALL ROS packages and start roscore:](https://docs.google.com/document/d/1ygzU4uE3p38XZT8Rey_QmykXRS-VA-rtt_XBkC3gpCE/edit#heading=h.kh752plifc0s) ```source_ros1_init```
* [Source Noetic and ALL ROS packages](https://docs.google.com/document/d/1ygzU4uE3p38XZT8Rey_QmykXRS-VA-rtt_XBkC3gpCE/edit#heading=h.kh752plifc0s) ```source_ros1_pkg```
* [Source Noetic and ALL ROS packages and put user in ros1_ws:](https://docs.google.com/document/d/1ygzU4uE3p38XZT8Rey_QmykXRS-VA-rtt_XBkC3gpCE/edit#heading=h.kh752plifc0s) ```source_ros1```
* [Source foxy and ALL ROS2 packages:](https://docs.google.com/document/d/1ygzU4uE3p38XZT8Rey_QmykXRS-VA-rtt_XBkC3gpCE/edit#heading=h.hmc07r1l07ue) ```source_ros2_pkg```
* [Source foxy and ALL ROS2 packages and put user in ros2_ws:](https://docs.google.com/document/d/1ygzU4uE3p38XZT8Rey_QmykXRS-VA-rtt_XBkC3gpCE/edit#heading=h.hmc07r1l07ue) ```source_ros2```
* [Build all packages in ucsd_robocar:](https://docs.google.com/document/d/1ygzU4uE3p38XZT8Rey_QmykXRS-VA-rtt_XBkC3gpCE/edit#heading=h.hmc07r1l07ue) ```build_ros2```
* [Source ROS bridge:](https://docs.google.com/document/d/1ygzU4uE3p38XZT8Rey_QmykXRS-VA-rtt_XBkC3gpCE/edit#heading=h.4r55s7fis99g) ```source_ros_bridge```

## 5. Source ROS Version

### 5.1 Source ROS1

### 5.2 Source ROS2

We need to source ROS Foxy and the ros2_ws, below is an alias command that will do that automatically. The alias will also place you in the ros2_ws. This command needs to be run in every new terminal you want to use ROS2 in.  

From the terminal:
```
source_ros2
```
Another alias was made to rebuild the package if any changes were made to the source code. It will put you in the ros2_ws, then perform a colcon build and then source install/setup.bash to reflect the changes made.

From the terminal (This is only needs to be ran in 1 terminal, the changes will be reflected everywhere):
```
build_ros2
```
### 5.3 Source ROS Bridge

## 6. Hardware Configuration

Not all robots have the same hardware especially when it comes to their sensors and motors and motor controllers. This quick section shows how to select the hardware that is on your robot. There are differences between ROS1 and ROS2 on how this configuration works so please read accordingly. This configuration is only necessary for the UCSD Robocar Image and NOT UCSD Robocar Simple ROS Image.

### 6.1 ROS1

### 6.2 ROS2

In ROS2, the hardware configuration is as simple as flipping a switch. Since the launch files in ROS2 are now in python, we can dynamically build launch files! This means no more need to have several different “car configs” that may have different hardware on them and instead have a single launch file that is capable of launching any component you need by changing a single number (that number is explained below)! There is only one file to modify and all that needs to be changed is either putting a “0” or a “1” next to the list of hardware in the file. To select the hardware that your robot has and that you want to use, put a “1” next to it otherwise put a “0” which means it will not activate.

In the ```car_config.yaml``` file, there is a list of actuator and sensor packages that can be used with the car. Set the corresponding funtionality for each component according to the direction above &mdash; once that is done, you must build the packages again:

From a terminal:
```
source_ros2
nano src/ucsd_robocar_hub2/ucsd_robocar_nav2_pkg/config/car_config.yaml
build_ros2
```

## 7. Node Configuration

## 8. Sensor Visualization

## 9. Manual Control of Robot with Joystick

## 10. Integrating New Packages/Code into the Framework

## 11. Navigation

## 12. Data Collection

## 13. F1 Tenth Simulator

## 14. Troubleshooting

## 15. Frequently Used Linux Commands

