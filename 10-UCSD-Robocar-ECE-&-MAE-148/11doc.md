# GPS Path Following


## GNSS Configuration:
## Ublox:
This GPS doesn't require any configuration out of the gate, it just starts transmitting data right away. Later on the instructions will tell you what to add in myconfig.py to get it to work with donkeycar.

## How to Plug PointOneNav to Donkeycar

1.  Make sure your user is added to the dialout group. If not

    a.  sudo adduser jetson dialout

    b.  sudo reboot now

2.  Download
    [https://drive.google.com/file/d/1BK_UjH-He9d_D4eObWMHzpHHCqmtq75h/view?usp=share_link](https://drive.google.com/file/d/1BK_UjH-He9d_D4eObWMHzpHHCqmtq75h/view?usp=share_link)
    Or you can alternatively do ```git clone https://github.com/UCSD-ECEMAE-148/quectel```
4.  Unzip.

5.  Run

    a.  deactivate (This should get you out of the current environment)

    b.  wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
    
    c.  bash Miniforge-pypy3-Linux-aarch64.sh   

    d.  Reboot the jetson   

    e.  mamba create --name py37 -c conda-forge python=3.7 pip   

    f.  mamba activate py37

    g.   cd quectel-lg69t-am.0.15.0/p1_runner (or if you cloned from github, cd quectel/p1_runner)

    h.  python3 -m pip install -r requirements.txt

        i.  %If this fails, you can try just going to the p1_runner
            directory and running the python3 bin/config_tool.py
            command, and then doing "pip install" for all the
            missing things, you may just need

            1.  pip install pyserial

            2.  pip install fusion_engine_client

            3.  pip install pynmea

            4.  pip install ntripstreams

            5.  pip install websockets

    i.  python3 bin/config_tool.py reset factory

    j.  python3 bin/config_tool.py apply uart2_message_rate nmea gga on

    k.  python3 bin/config_tool.py save

    l.  ```python3 bin/runner.py --device-id <polaris_username>
        --polaris <polaris_password> --device-port /dev/ttyUSB1```
Note, the polaris passwords will be posted in the class discord. If they aren't up yet and you need them, you can ask the professor or TA's

> (if not getting any data including NANs try USB0)
> Also, note that the GPS will return NAN's inside, so you need to test it outside.

Note: The GPS corrections will only happen when you are actively running
runner.py. I recommend making a bashrc command that you can run to start
up the runner.py program easily in a 2nd terminal while using the GPS
for anything.
For example:
nano ~/.bashrc
Add the text:
```
function gps_corrections() {
mamba activate py37
python3 /projects/quectel-lg69t-am.0.15.0/p1_runner/bin/runner.py --device-id <polaris_username> --polaris <polaris_password> --device-port /dev/ttyUSB1  
}
```

Of course substituting in your own password and filepath.

Then ```source ~/.bashrc``` and you can just type gps_corrections to get corrections going

5.  Create a project with the DonkeyCar path follow template

    a.  Open a new terminal window

    b.  Make sure that the donkey car environment is running

        i.  ```source ~/projects/envs/donkey/bin/activate```

    c.  cd ~/projects

    d.  donkey createcar --path ./mycar --template path_follow

6.  Set the following in the myconfig.py 

    a.  GPS_SERIAL = "/dev/ttyUSB2" #(USB1 if USB0 used above) This applies for pointone nav
    GPS_SERIAL = "/dev/ttyUSB0" #This applies for ublox

    b.  GPS_SERIAL_BAUDRATE = 460800 #for pointone nav
    
    GPS_SERIAL_BAUDRATE = 38400 #for ublox

    c.  GPS_DEBUG = True

    d.  HAVE_GPS = True

    e.  GPS_NMEA_PATH = None

8.  Also set things like the VESC parameters in myconfig.py. You can
    copy these over from the donkeycar you created earlier.

9.  Run

    a.  python3 manage.py drive

10.  You should see GPS positions being outputted after you run Donkeycar. If you don't want to output set GPS_DEBUG to False

11. Configure button actions

a.  SAVE_PATH_BTN is the button to save the in-memory path to a file.

b.  LOAD_PATH_BTN is the button to (re)load path from the csv file into memory.

c.  RESET_ORIGIN_BTN is the button to set the current position as the origin.

d.  ERASE_PATH_BTN is the button to erase path from memory and reset the origin.

e.  TOGGLE_RECORDING_BTN is the button to toggle recording mode on or off. Note that there is a pre-assigned button in the web ui, so there is not need to assign this button to one of the web buttons if you are using the web ui.

f.  INC_PID_D_BTN is the button to change PID \'D\' constant by PID_D_DELTA.

g.  DEC_PID_D_BTN is the button to change PID \'D\' constant by -PID_D_DELTA

h.  INC_PID_P_BTN is the button to change PID \'P\' constant by PID_P_DELTA

i.  DEC_PID_P_BTN is the button to change PID \'P\' constant by -PID_P_DELTA

The logitech buttons are named stuff like "X" or "R1" See the example config below.

SAVE_PATH_BTN = "R1" # button to save path
LOAD_PATH_BTN = "X" # button (re)load path
RESET_ORIGIN_BTN = "B" # button to press to move car back to origin
ERASE_PATH_BTN = "Y" # button to erase path
TOGGLE_RECORDING_BTN = "L1" # button to toggle recording mode

11. Recording a path

a.  The algorithm assumes we will be driving in a continuous connected path such that the start and end are the same. You can adjust the space between recorded waypoints by editing the PATH_MIN_DIST value in myconfig.py You can change the name and location of the saved file by editing the PATH_FILENAME value.

b.  Enter User driving mode using either the web controller or a game controller.

c.  Move the car to the desired starting point

d.  Erase the path in memory (which will also reset the origin).

Note:  Make sure to reset the origin!!! If you didn't need to erase the path in memory you can just go ahead with toggling recording

e.  Toggle recording on.

f.  Drive the car manually around the track until you reach the desired starting point again.

g.  Toggle recording off.

h.  If desired, save the path.

13. Following a path

a.  Enter User driving mode using either the web controller or a
game controller.

b.  Move the car to the desired starting point - make sure it's the
        same one from when you recorded the path

c.  Reset the origin (be careful; don\'t erase the path, just reset
        the origin).

d.  Load the path

e.  Enter Autosteering or Autopilot driving mode. This is normally done by pressing the start button either once or twice If you are in Autosteering mode you will need to manually provide throttle for the car to move. If you are in Autopilot mode the car should drive itself completely.

14. Configuring Path Follow Parameters

 a.  So the algorithm uses the cross-track error between a desired line and the vehicle's measured position to decide how much and which way to steer. But the path we recorded is not a simple line; it is a lot of points that is typically some kind of circuit. As described above, we use the vehicle's current position to choose a short segment of the path that we use as our desired track. That short segment is recalculated every time we get a new measured car position. There are a few configuration parameters that determine exactly which two points on the path that we use to calculate the desired track line.

i.  PATH_SEARCH_LENGTH = None # number of points to search for closest point, None to search entire path

ii. PATH_LOOK_AHEAD = 1 # number of points ahead of the closest point to include in cte track

iii. PATH_LOOK_BEHIND = 1 # number of points behind the closest point to include in cte track

b.  Generally, if you are driving very fast you might want the look ahead to be larger than if driving slowly so that your steering can anticipate upcoming curves. Increasing the length of the resulting track line, by increasing the look behind and/or look ahead, also acts as a noise filter; it smooths out the track. This reduces the amount of jitter in the controller. However, this must be balanced with the true curves in the path; longer track segments effectively 'flatten' curves and so can result in understeer; not steering enough when on a curve.

15. Determining PID Coefficients

a.  The PID coefficients are the most important (and time consuming) parameters to configure. If they are not correct for your car then it will not follow the path. The coefficients can be changed by editing their values in the myconfig.py file.

b.  PID_P is the proportional coefficient; it is multiplied with the cross-track error. This is the most important parameter; it contributes the most to the output steering value and in some cases may be all that is needed to follow the line. If this is too small then car will not turn enough when it reaches a curve. If this to too large then it will over-react to small changes in the path and may start turning in circles; especially when it gets to a curve.

c.  PID_D is the differential coefficient; it is multiplied with the change in the cross-track error. This parameter can be useful in reducing oscillations and overshoot.

d.  PID_I is the integral coefficient; it is multiplied with the total accumulated cross-track error. This may be useful in reducing offsets caused by accumulated error; such as if one wheel is slightly smaller in diameter than another.

e.  Determining PID Coefficients can be difficult. One approach is:

i.  First determine the P coefficient.

ii. zero out the D and the I coefficients.

iii. Use a kind of 'binary' search to find a value where the vehicle will roughly follow a recorded straight line;
 probably oscillating around it. It will be weaving

iv. Next find a D coefficient that reduces the weaving (oscillations) on a straight line. Then record a path with a tight turn. Find a D coefficient that reduces the overshoot when turning.

v.  You may not even need the I value. If the car becomes unstable after driving for a while then you may want to start to set this value. It will likely be much smaller than the other values.

vi. Be patient. Start with a reasonably slow speed. Change one thing at a time and test the change; don't make many changes at once. Write down what is working.


vii. Once you have a stable PID controller, then you can figure out just how fast you can go with it before autopilot becomes unstable. If you want to go faster then set the desired speed and start tweaking the values again using the method suggested above.



## Optional Section: Configuring the GPS to Publish Coordinates to the ROS2 Topic /fix
### Most groups will not need to use this, this section only applies if you need it for your final project.

Setting up ROS2 gps publishing to /fix and /gps_fix:

1. Start a docker container with the djnighti/ucsd_robocar:devel  image (you need the newer ubuntu version in the docker for later steps to work)
	Document 100 pg 17 has instructions on how to do this if you are confused
2. Follow the instructions in https://github.com/PointOneNav/ros2-fusion-engine-driver/blob/main/README.md 
I will write them out here with a couple bug corrections for your convenience. Note that we are running ros2 foxy, not humble

3. Install some dependencies
apt-get install ros-foxy-gps-msgs
apt install ros-foxy-nmea-msgs
apt install ros-foxy-mavros

4. Configuring your device:
Navigate to the p1-host-tools that you set up earlier
```
python3 bin/config_tool.py apply uart2_message_rate fe ROSPoseMessage 100ms
python3 bin/config_tool.py apply uart2_message_rate fe ROSGPSFixMessage 100ms
python3 bin/config_tool.py save
```
The website instructions include a 3rd command which is setting up the imu. However our gps devices do not have the firmware loaded to support that, so that command will not 
work.

5. git clone https://github.com/PointOneNav/ros2-fusion-engine-driver.git

6. cd ros2-fusion-engine-driver

7. rosdep install -i --from-path ./ --rosdistro foxy -y

8. build_ros2
	This will take a long time, about three minutes. If you get an error saying various c++ commands are not found it is the iomanip loading error. Normally, the iomanip 
library is automatically loaded into the c++ compiler, but for some reason it is not being recognized on ours. 
	To fix, navigate to the file fusion-engine-driver/utils/conversion_utils.hpp and add #include <iomanip> to the top of the file

9. To run the command, do
ros2 run fusion-engine-driver fusion_engine_ros_driver --ros-args -p connection_type:=tty -p tty_port:=/dev/ttyUSB1
We are connecting through serial. 

10. To check that the gps output is being published, you can open a new terminal, enter the same container that the gps is running in, and do ros2 topic echo /fix and you 
should see lots of gps coordinates being published.


# Troubleshooting:
If you have version conflicts with the config.py, (specifically catkin version errors for me), try setting up the p1_tools inside of a docker container with the ucsd_robocar image. The docker containers run a newer version of ubuntu linux which can help fix these conflicts.

If you have significant configuration issues, running  
python3 bin/config_tool.py reset factory 
and then reconfiguring can be a good idea to fix them.

If the GPS is not responding to the configuration tool, reflashing the firmware can usually fix it.
To do this,
```git clone https://github.com/PointOneNav/firmware-tools/tree/main```
Then deactivate all virtual environments, and make a new environment with the command
```mamba create -n gpsfirmware python=3.11```
```mamba activate gpsfirmware```
Then download the pointone nav firmware from https://pointonenav.com/resources/ and get it on to your jetson somehow (rsync, sftp, etc.)
```
cd l69t/firmware-tools
```
```
python3 firmware_tool.py --port=/dev/ttyUSB1 /Path/To/firmware
```
If even this doesn't work, reflashing the bootloader the bootloader as per the instructions at the above github can help.
If you need to find a bootloader file, know that one exists in the quectel GPS repository linked above in the instructions in quectel-lg69t-am.0.15.0/sw/quectel-bootloader-1.0.2.bin 
Please don't hesitate to ask the TA's for help with any of this (firmware or bootloader)!

If you experience serious lag when running the gps laps, check your numpy version. Anything but 1.19.0 will create serious issues. 
To fix, do ```pip install numpy==1.19.0``` and then create a new car with ```donkey createcar --path ./newcar --template path_follow```


If you decide you don't need the fusion-engine-client and want fast build times, I recommend modifying the build_ros2 command in the ~./bashrc to exlude the 
fusion-engine-driver package for faster loading time, unless you really need it. Excluding the ntrip_client can save time too. 

Here is the changed build_ros2 command
function build_ros2() {
  cd /home/projects/ros2_ws 
  rm -rf build/ install/ log/ 
  colcon build --packages-ignore fusion-engine-driver ntrip_client
  source install/setup.bash
}
make sure to source ~/.bashrc after to have the changes take effect. Do all this inside a docker container
