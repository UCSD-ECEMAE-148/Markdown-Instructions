# UCSD Robocar MAE & ECE 148

## Table of Contents

## Introduction

## Single Board Computer (SBC) Basic Setup

### Jetson Nano (JTN) Configuration

### Jetson Xavier (JNX) Configuration

## Hardware Setup

Subsections not included currently &mdash; see the Google Doc.

## Build OpenCV from Source

Since January 2020, Nvidia has not provided OpenCV optimized for use with the CUDA cores on the Jetson. Therefore, we must build OpenCV from source in order for it to be accelerated via CUDA.

Instructions can either be found in the 60 document, or [here](https://docs.google.com/document/d/1HX2zmjbVsyLnliEQ8wp97Y453g5qNAYHWtFQiKQ0elA/edit?usp=sharing).

## DonkeyCar AI Framework

This section was created referencing [this document](http://docs.donkeycar.com).

Make sure OpenCV was built from source or verify that CUDA support has been enabled.

### Setting up the DonkeyCar AI Framework

Begin by ssh-ing into the Jetson (or your SBC of choice).
To ensure a proper base for the installation, run:
```
sudo apt update -y
sudo apt upgrade -y
sudo usermod -aG dialout jetson
```

If packages are being held back, run:
```
sudo apt-get --with-new-pkgs upgrade
```

and (?)

```
sudo apt-get install -y build-essential python3 python3-dev python3-pip libhdf5-serial-dev hdf5-tools  libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran libxslt1-dev libxml2-dev libffi-dev libcurl4-openssl-dev libssl-dev libpng-dev libopenblas-dev openmpi-doc openmpi-bin libopenmpi-dev libopenblas-dev git nano
```

#### GPIO Access for Jetson

Install the RPi.GPIO clone for the Jetson Nano, [Jetson.GPIO](https://github.com/NVIDIA/jetson-gpio).
```
pip3 install Jetson.GPIO
```

If the pip install complains about ownership of the directory, execute:
```
sudo chown -R jetson:jetson /home/jetson/.cache/pip
```

**Warning**: the directory ```/home/jetson/.cache/pip/http``` or its parent directory is not owned by the current user and the cache has been disabled. Please check the permissions and owner of that directory. If executing pip with sudo, you may want sudo's -H flag.

If pip breaks for some reason, you can reinstall with:
```
python3 -m pip uninstall pip
sudo apt install python3-pip --reinstall
```

If the install requests elevated privileges, execute:
```
sudo pip3 install Jetson.GPIO
```

If pip has a new version, run:
```
pip3 install --upgrade pip
```

***
**As of 18 Sept 2022, the following section does not work with JetPack 4.6.2**

To make sure the ```jetson``` user can use the GPIO, run:
```
sudo groupadd -f -r gpio
sudo usermod -a -G gpio jetson
sudo cp /opt/nvidia/jetson-gpio/etc/99-gpio.rules /etc/udev/rules.d/
```
***

#### Creating a Virtual Environment for DonkeyCar AI Framework

If you want the virtual environment to be under the user's home directory, make sure to be on the home directory for the user ```jetson```.

If you have not done so, create the projects directory ```~/projects``` and a subdirectory to store virtual environments ```~/projects/envs```:
```
cd ~
mkdir projects
cd projects
mkdir envs
cd envs
pip3 install virtualenv
```
If there is a user permission error, run:
```
pip3 install virtualenv --user
```

We will create the virtual environment ```donkey``` since the framework is based on DonkeyCar. To do so, run:
```
python3 -m virtualenv -p python3 ~/projects/envs/donkey --system-site-packages
```

Since the Jetson will be working with Donkey primarily until custom projects begin, we will have it so the donkey environment is enabled whenever loggin in to the Jetson. You can remove this later if needed:

```
echo "source ~/projects/envs/donkey/bin/activate" >> ~/.bashrc
source ~/.bashrc
```

Whenever a virtual envirnonment is activated, you should see (NAME_OF_VIRTUAL_ENVIRONMENT) before your terminal prompt (e.g. ```(donkey) jetson@ucsdrobocar-xxx-yy:~$```)

