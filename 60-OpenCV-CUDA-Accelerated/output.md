<div>

[OpenCV]{.c21}                

------------------------------------------------------------------------

[]{.c2}

[]{.c2}

</div>

[UCSD RoboCar OpenCV CUDA Accelerated]{.c21 .c28}

[]{.c2}

[Version 1.8 - 26Nov2022]{.c2}

[]{.c2}

[]{.c2}

[]{.c2}

[]{.c2}

[]{.c2}

[]{.c2}

[Prepared by]{.c2}

[Dr. Jack Silberman]{.c2}

[Department of Electrical and Computer Engineering]{.c2}

[and ]{.c2}

[Dominic Nightingale]{.c2}

[Department of Mechanical and Aerospace Engineering]{.c2}

[University of California, San Diego]{.c2}

[9500 Gilman Dr, La Jolla, CA 92093]{.c2}

[]{.c2}

[]{.c2}

[]{.c2}

        [![](images/image5.png){style="width: 300.61px; height: 71.02px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 300.61px; height: 71.02px;"}

        [![](images/image7.png){style="width: 134.50px; height: 134.50px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 134.50px; height: 134.50px;"}

        [![](images/image3.png){style="width: 302.87px; height: 64.50px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 302.87px; height: 64.50px;"}

[                ]{.c2}

------------------------------------------------------------------------

[]{.c2}

[\# Installing an Open Source Computer Vision (OpenCV) package  with
CUDA Support]{.c2}

[\# As of Jan 2020, NVIDIA is not providing OpenCV optimized to use CUDA
(GPU acceleration).]{.c2}

[\# Search the web if you are curious why.]{.c2}

\#
[[https://forums.developer.nvidia.com/t/opencv-cuda-python-with-jetson-nano/72902](https://www.google.com/url?q=https://forums.developer.nvidia.com/t/opencv-cuda-python-with-jetson-nano/72902&sa=D&source=editors&ust=1726197318229206&usg=AOvVaw0pJIrOYGk0cH4JZkWgFRt5){.c11}]{.c13}

[]{.c2}

["]{.c2}

[Hi,]{.c12}

[Replied your question inline:]{.c12}

[Jetson Nano GPU does not support OpenCV acceleration (through opencl)
with Python]{.c12}

[Our default OpenCV does support GPU acceleration.]{.c12}

[The common issue is there are some features that have not been
enabled.]{.c12}

[(This feature often requires third-party library installation)]{.c25}

["]{.c2}

[[https://www.youtube.com/watch?v=art0-99fFa8](https://www.google.com/url?q=https://www.youtube.com/watch?v%3Dart0-99fFa8&sa=D&source=editors&ust=1726197318230000&usg=AOvVaw3ecmo2FVU12A9KVM1eBgTv){.c11}]{.c13}

[]{.c2}

[]{.c2}

------------------------------------------------------------------------

[]{.c2}

# [Checking openCV build information]{.c14} {#h.3boutaiszk8f .c16}

[\# ssh to the Single Board Computer (SBC)]{.c2}

[\# Check to see if OpenCV for CUDA is available, search on the terminal
output for CUDA),]{.c2}

[\# if not,  we build OpenCV from source to use CUDA Acceleration]{.c2}

[]{.c2}

[\# From a terminal:]{.c2}

[python]{.c2}

[\>\>import cv2]{.c2}

[\>\>print cv2.getBuildInformation() ]{.c2}

[\>\>exit ()]{.c2}

[ ]{.c2}

[\# ex:]{.c2}

[]{.c2}

[    NVIDIA CUDA:                   YES (ver 10.2, CUFFT CUBLAS
FAST_MATH)]{.c2}

[    NVIDIA GPU arch:             53 62 72]{.c2}

[    NVIDIA PTX archs:]{.c2}

[]{.c2}

[  cuDNN:                         YES (ver 8.0)]{.c2}

[]{.c2}

[  OpenCL:                        YES (no extra features)]{.c2}

[    Include path:              
 /tmp/build_opencv/opencv/3rdparty/include/opencl/1.2]{.c2}

[    Link libraries:              Dynamic load]{.c2}

[]{.c2}

[You can also use jtop to check if OpenCV was compiled with CUDA.]{.c2}

[jtop]{.c2}

[6]{.c2}

[]{.c2}

[![](images/image11.png){style="width: 435.50px; height: 248.73px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 435.50px; height: 248.73px;"}

[]{.c2}

[\# If OpenCV is not using CUDA acceleration, let\'s build and install
the latest OpenCV optimized]{.c2}

[\# for CUDA]{.c2}

[]{.c2}

[\# As of 03Apr2022, I have not seen NVIDIA supplying OpenCV GPU
accelerated on their]{.c2}

[\# JetPack. We need to compile it from source. If you ask me, nice
experience for you.]{.c2}

[\# Let's do this]{.c2}

------------------------------------------------------------------------

# []{.c14} {#h.jxhoa7kcd9g4 .c16 .c24}

# [Installing openCV with CUDA]{.c14} {#h.k7mmblytcair .c16}

[\# This step may take several hours]{.c21} [to compile and build OpenCV
in a SBC such as the]{.c10}

[\# Jetson Nano (JTN) or Jetson Xavier NX (JNX). It took my JTN four
hours to complete.]{.c10}

[\# For the JTN, please make sure the fan is installed including
software to enable high ]{.c10}

[\# CPU power mode]{.c10}

[]{.c2}

[\# Better do it overnight. Be patient, please keep in mind that you are
using a low power ]{.c2}

[\# single board computer (SBC)]{.c2}

[\# Make sure you are using the external power supply to power the
jetson, not a USB cable.]{.c2}

[]{.c2}

\#
[[https://devtalk.nvidia.com/default/topic/1054949/process-to-install-opencv-4-1-on-nano/](https://www.google.com/url?q=https://devtalk.nvidia.com/default/topic/1054949/process-to-install-opencv-4-1-on-nano/&sa=D&source=editors&ust=1726197318232482&usg=AOvVaw3TCl7LYtalf_z7H197t48W){.c11}]{.c13}

\#
[[https://github.com/mdegans/nano_build_opencv](https://www.google.com/url?q=https://github.com/mdegans/nano_build_opencv&sa=D&source=editors&ust=1726197318232671&usg=AOvVaw0MDIkfqz8fi53kA6njA30R){.c11}]{.c13}

[]{.c2}

[\# Please make sure your computer will not sleep so you maintain an
active ssh connection]{.c2}

[\# while you are building OpenCV because you will need to type the user
jetson password ]{.c2}

[\# again to complete the install. ]{.c2}

[\# Reconnecting on the same ssh session is not trivial]{.c2}

[]{.c2}

[![](images/image9.png){style="width: 624.00px; height: 165.33px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 624.00px; height: 165.33px;"}

[]{.c2}

## [\# Using a Linux Host? Use Screen]{.c17} {#h.cbsfos7szrlx .c20}

\# You should consider using
[[screen](https://www.google.com/url?q=https://ma.ttias.be/screen-a-must-for-ssh/&sa=D&source=editors&ust=1726197318233284&usg=AOvVaw2FS8roTbC4395mQWzL2txP){.c11}]{.c13}[ to
enable you to reconnect to a previous ssh session. ]{.c2}

[\# Do you need to install screen?]{.c2}

[]{.c2}

[sudo apt-get install screen]{.c2}

[]{.c2}

[\# From a terminal on Linux host or on your VM:]{.c2}

[screen]{.c2}

[\# ssh jetson@ name of IP address of your machine]{.c2}

[ssh jetson@ucsdrobocar-xxx-yy.local]{.c2}

[]{.c2}

[\# If you get disconnected, open a terminal again on your computer then
type]{.c2}

[screen -r]{.c2}

[]{.c2}

[\# screen -r should reconnect to the previous ssh session that was
open]{.c2}

[]{.c2}

[\# Here is some more detailed information on using screen]{.c2}

[[https://www.howtogeek.com/howto/ubuntu/keep-your-ssh-session-running-when-you-disconnect](https://www.google.com/url?q=https://www.howtogeek.com/howto/ubuntu/keep-your-ssh-session-running-when-you-disconnect&sa=D&source=editors&ust=1726197318234086&usg=AOvVaw2XcDpoU2cE0S0nQnRR4Xlg){.c11}]{.c13}

[\# if not installed in your host]{.c6}

[sudo apt-get install screen]{.c6}

[]{.c6}

[Now you can start a new screen session by just typing screen at the
command line. You'll be shown some information about screen. Hit enter,
and you'll be at a normal prompt.]{.c6}

[]{.c6}

[To disconnect (but leave the session running)]{.c6}

[]{.c6}

[Hit Ctrl + A and then Ctrl + D in immediate succession. You will see
the message \[detached\]]{.c6}

[]{.c6}

[To reconnect to an already running session]{.c6}

[]{.c6}

[screen -r]{.c6}

[]{.c6}

[To reconnect to an existing session, or create a new one if none
exists]{.c6}

[]{.c6}

[screen -D -r]{.c6}

[]{.c2}

[\# More info on using screen]{.c2}

[[https://ma.ttias.be/screen-a-must-for-ssh/](https://www.google.com/url?q=https://ma.ttias.be/screen-a-must-for-ssh/&sa=D&source=editors&ust=1726197318234977&usg=AOvVaw19l5GMHaE-IaUczDV1NztF){.c11}]{.c13}

[screen -ls]{.c6}

[There are screens on:]{.c6}

[27111.screen_untarring  (Detached)]{.c6}

[27097.screen_disking    (Detached)]{.c6}

[2 Sockets in /var/run/screen/S-root.]{.c6}

[This will show a list of all running screen-sessions at any given time.
You can pick up a previous screen-session, by typing]{.c6}

[]{.c6}

[screen -r \<name_of_session\>]{.c6}

[screen -r]{.c30}

------------------------------------------------------------------------

## []{.c17} {#h.zdkduk3j5481 .c19}

[\# Since your SBC will be busy, how can you see it is working for
you?]{.c2}

[\#]{.c2}

\# Besides htop or top, there is another cool utility called
[[jetson_stats](https://www.google.com/url?q=https://github.com/rbonghi/jetson_stats&sa=D&source=editors&ust=1726197318235705&usg=AOvVaw0GfPbvlWHkLRmDtCHsWs4S){.c11}]{.c13}

[sudo apt-get install python3-pip]{.c2}

[sudo -H pip3 install -U jetson-stats]{.c2}

[]{.c2}

[\# reboot]{.c2}

[sudo reboot now]{.c2}

[\# Then you can run]{.c2}

[jtop]{.c2}

[4]{.c2}

[]{.c2}

[You can use jtop to add more swap space using the left and right keys
and clicking the plus button]{.c2}

[![](images/image14.png){style="width: 384.50px; height: 170.58px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 384.50px; height: 170.58px;"}

[]{.c2}

[Add 4G of swap and press \<S\> to enable it.]{.c2}

[]{.c2}

[![](images/image8.png){style="width: 289.50px; height: 207.53px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 289.50px; height: 207.53px;"}

## [Removing old version of OpenCV ]{.c17} {#h.qbolrby1hrw .c20}

[\# First lets remove the current OpenCV version and reboot the
JTN]{.c2}

[]{.c2}

[sudo sudo apt-get purge \*libopencv\*]{.c2}

[sudo reboot now]{.c2}

[]{.c2}

## [Compiling OpenCV from source]{.c17} {#h.dqmd5h5g4njr .c20}

[\# screen then ssh back to the SBC]{.c2}

[\# If needed, create the \~/projects directory]{.c2}

[cd\~]{.c2}

[mkdir projects]{.c2}

[cd projects]{.c2}

[]{.c2}

[]{.c2}

[\# For Jetson Nano ]{.c2}

git clone
[[https://github.com/mdegans/nano_build_opencv.git](https://www.google.com/url?q=https://github.com/mdegans/nano_build_opencv.git&sa=D&source=editors&ust=1726197318237326&usg=AOvVaw3FCSurBR4SgrjfPe_qkdVm){.c11}]{.c13}

[cd nano_build_opencv]{.c2}

[]{.c2}

[\# For the Jetson Nano, lets modify the script to use 2 CPU cores
vs. 1. ]{.c2}

[\# Note: for the TX2 and Xaviers the script automatically uses more CPU
cores.]{.c2}

[\# Need to modify the configuration for the CUDNN we are using]{.c2}

[\# First see what you have installed using jtop use option INFO press
7]{.c2}

[]{.c2}

[]{.c2}

[![](images/image6.png){style="width: 412.28px; height: 288.60px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 412.28px; height: 288.60px;"}

[![](images/image13.png){style="width: 486.57px; height: 309.69px; margin-left: -10.61px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 465.36px; height: 309.69px;"}

[#Remove the 8.7 from the CUDA_ARCH_BIN version, it causes a
compatibility issue]{.c2}

[#Also, make sure the CUDNN_VERSION='8.0', it will fail otherwise]{.c2}

[]{.c2}

[nano build_opencv.sh]{.c2}

[]{.c2}

[\# modify this line to have JOBS=2]{.c2}

[else]{.c2}

[    JOBS=1  # you can set this to 4 if you have a swap file]{.c2}

[]{.c2}

[JOBS=2]{.c2}

[\# save the file / exit nano]{.c2}

[]{.c2}

[]{.c2}

[]{.c2}

[]{.c2}

\#
[[https://opencv.org/releases/](https://www.google.com/url?q=https://opencv.org/releases/&sa=D&source=editors&ust=1726197318238434&usg=AOvVaw3GYKxvbJBmBbBEwU3YtXNM){.c11}]{.c13}

[\# look for the latest version]{.c2}

[]{.c2}

[![](images/image1.png){style="width: 365.09px; height: 255.06px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 365.09px; height: 255.06px;"}

\# As of 24Nov23, [4.8.0]{.c21}[ is the latest version]{.c2}

[]{.c2}

[\# You can use jtop to have Jetson Clocks enabled to improve the
performance of the Jetson]{.c2}

[\# 6 CTRL  and then press s]{.c2}

[\# To make it enabled at boot too, press e]{.c2}

[]{.c2}

[![](images/image2.png){style="width: 380.73px; height: 281.31px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 380.73px; height: 281.31px;"}

[]{.c2}

[]{.c2}

[]{.c2}

[\# From a terminal]{.c2}

./build_opencv.sh [4.8.0]{.c21}

[]{.c2}

[]{.c2}

\# Again, this will take a good while.[ You may consider doing this at
night before you go to bed. It is really boring]{.c2}

[\# after a while (hours) looking that the SBC compile and build OpenCV
from source]{.c2}

[\#  Please make sure the computer you are using to SSH, disable sleep,
and if using a notebook computer ]{.c2}

[\# have in connected to the charger ]{.c2}

[\# Preferable you use screen before ssh to be able to reconnect in
]{.c2}

[\# case your computer lose the SSH connection.]{.c2}

[\#  If you wish, you can see that the SBC cores of the CPU are]{.c2}

[\#  busy by opening another terminal or a new tab on the same terminal
window then]{.c2}

[\#  ssh to the SBC and run the command top]{.c2}

[\#  I like better jtop (if needed, install htop or jtop).]{.c2}

[\#]{.c2}

[\# ex:]{.c2}

[![](images/image10.png){style="width: 378.50px; height: 248.09px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 378.50px; height: 248.09px;"}

[]{.c2}

[\# After a few hours you see that you need to enter a little
information to finish the build. That is when we have]{.c2}

[\# problems if you get disconnected from your SSH]{.c2}

[]{.c2}

[\#  You can leave the temporary builds in place]{.c2}

[Do you wish to remove temporary build files in /tmp/build_opencv ?
]{.c2}

[(Doing so may make running tests on the build later impossible)]{.c2}

Y/N[ y]{.c10}

[\# Enter y]{.c10}

[\# save some space on disc ]{.c2}

[]{.c10}

# [Checking the Install]{.c14} {#h.8nyvs9xjvw3u .c16}

[\# When the installation is complete, let\'s check the version of
OpenCV that was installed]{.c2}

[   ]{.c2}

[\# First reboot SBC]{.c2}

[sudo reboot now]{.c2}

[]{.c2}

[\# ssh back to the SBC]{.c2}

[]{.c2}

[#Using jtop to confirm the OpenCV GPU (CUDA)  compiled]{.c2}

[]{.c2}

[jtop]{.c2}

[7 INFO]{.c2}

[]{.c2}

[]{.c2}

[\# look for the OpenCV version and that it was compiled with CUDA]{.c2}

[]{.c2}

[]{.c2}

[![](images/image4.png){style="width: 442.88px; height: 322.31px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 442.88px; height: 322.31px;"}

[![](images/image12.png){style="width: 720.00px; height: 36.00px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 720.00px; height: 36.00px;"}

[]{.c2}

## [\# Let's make sure Python3 finds OpenCV (CV2)]{.c17} {#h.lwhoncvefnzl .c20}

[\# Need to adapt this for the python and OpenCV versions that you are
using]{.c2}

[]{.c2}

[python3 -c \'import cv2 as cv; print(cv.\_\_version\_\_)\']{.c2}

[#Traceback (most recent call last):]{.c2}

[  #File \"\<string\>\", line 1, in \<module\>]{.c2}

[#ModuleNotFoundError: No module named \'cv2\']{.c2}

[]{.c2}

[\# If you don't see the version of the OpenCV that you built listed, it
is because you forgot to delete the previous]{.c2}

[\# OpenCV version that was installed, go back to this document
"Removing old version of OpenCV" and then]{.c2}

[\# build and  install OpenCV again.]{.c2}

[]{.c2}

[]{.c2}

[\# What version of python3 do you have?]{.c2}

[]{.c2}

[python3]{.c2}

[#Python 3.8.10 (default, May 26 2023, 14:05:08) ]{.c2}

[#\[GCC 9.4.0\] on linux]{.c2}

[#Type \"help\", \"copyright\", \"credits\" or \"license\" for more
information.]{.c2}

[\>\>\> exit()]{.c2}

[]{.c2}

[ex:]{.c2}

[cd /usr/local/lib/python3.6/site-packages/cv2/python-3.6]{.c2}

[\# or]{.c2}

[cd /usr/local/lib/python3.8/site-packages/cv2/python-3.8]{.c2}

[]{.c2}

[\# remove the previous cv2.so  in case it is still there.]{.c2}

[sudo rm -rf cv2.so]{.c2}

\# mv cv2.cpython-36m-[xxx]{.c26}[-linux-gnu.so cv2.so]{.c2}

[]{.c2}

[ls cv2.\*]{.c2}

[]{.c2}

[\# cv2.cpython-38-aarch64-linux-gnu.so]{.c2}

[]{.c2}

[ln -s
/usr/local/lib/python3.8/site-packages/cv2/python-3.8cv2.cpython-38-aarch64-linux-gnu.so
cv2.so cv2.so]{.c2}

[]{.c2}

[]{.c2}

[sudo cp cv2.cpython-36m-aarch64-linux-gnu.so cv2.so]{.c2}

[\# or]{.c2}

[sudo cp cv2.cpython-38-aarch64-linux-gnu.so cv2.so]{.c2}

[ ]{.c2}

[\# Alternatively you could just rename the file]{.c2}

[\# sudo mv cv2.cpython-36m-aarch64-linux-gnu.so cv2.so]{.c2}

[\# sudo cp cv2.cpython-38-aarch64-linux-gnu.so cv2.so]{.c2}

[]{.c2}

[\# Lets create a link at the home directory for cv2]{.c2}

[]{.c2}

[cd \~]{.c2}

[sudo rm -rf cv2.so]{.c2}

[ln -s /usr/local/lib/python3.6/site-packages/cv2/python-3.6/cv2.so
cv2.so]{.c2}

[\# or ]{.c2}

[ln -s /usr/local/lib/python3.8/site-packages/cv2/python-3.8/cv2.so
cv2.so]{.c2}

[]{.c2}

[]{.c2}

[Checking install for python3]{.c2}

[python3]{.c2}

[import cv2]{.c2}

[cv2.\_\_version\_\_]{.c2}

[exit ()]{.c2}

[]{.c2}

[\# Resulting on something similar to this ]{.c2}

[]{.c2}

[jetson@ucsdrobocar-xxx-yy]{.c29 .c23 .c21}[:]{.c23}[\~]{.c18}[\$
python3]{.c9}

[Python 3.6.9 (default, Jun 29 2022, 11:45:57) ]{.c9}

[\[GCC 8.4.0\] on linux]{.c9}

[Type \"help\", \"copyright\", \"credits\" or \"license\" for more
information.]{.c9}

[\>\>\> import cv2]{.c9}

[\>\>\> cv2.\_\_version\_\_]{.c9}

[\'4.6.0\']{.c9}

[\>\>\> exit()]{.c9}

[jetson@ucsdrobocar-xxx-yy]{.c23 .c21 .c29}[:]{.c23}[\~]{.c18}[\$ ]{.c9}

[]{.c2}

[Python 3.8.10 (default, May 26 2023, 14:05:08) ]{.c2}

[\[GCC 9.4.0\] on linux]{.c2}

[Type \"help\", \"copyright\", \"credits\" or \"license\" for more
information.]{.c2}

[\>\>\> import cv2]{.c2}

[\>\>\> cv2.\_\_version\_\_]{.c2}

[\'4.8.0\']{.c2}

[\>\>\> exit ()]{.c2}

[]{.c2}

[]{.c2}

[\# Here is quicker way to test cv2 on Python 3]{.c2}

[]{.c2}

[python3 -c \'import cv2 as cv; print(cv.\_\_version\_\_)\']{.c2}

[]{.c2}

[]{.c2}

[4.6.0]{.c9}

[\# or]{.c9}

[4.8.0]{.c9}

[]{.c2}

[]{.c2}

[]{.c2}

[]{.c2}

[end of Installing an Open Source Computer Vision (OpenCV) package  with
CUDA Support]{.c10}

[]{.c10}

[]{.c10}

<div>

------------------------------------------------------------------------

[]{.c2}

[![](images/image5.png){style="width: 181.50px; height: 42.61px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 181.50px; height: 42.61px;"}        [![](images/image7.png){style="width: 49.86px; height: 49.86px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 49.86px; height: 49.86px;"}        [![](images/image3.png){style="width: 215.50px; height: 46.37px; margin-left: 0.00px; margin-top: 0.00px; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px);"}]{style="overflow: hidden; display: inline-block; margin: 0.00px 0.00px; border: 0.00px solid #000000; transform: rotate(0.00rad) translateZ(0px); -webkit-transform: rotate(0.00rad) translateZ(0px); width: 215.50px; height: 46.37px;"}

[]{.c2}

</div>
