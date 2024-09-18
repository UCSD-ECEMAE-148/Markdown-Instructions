# Setting up DonkeySim on a Mac M1 or newer (ARM Based)
The class virtual machine will not run on ARM based macs, but you can install the software natively instead.

If you are on a linux or windows machine but are having issues with the virtual machine, following these instructions with a few changes for your platform could work for you too. Installing donkeysim/donkeycar natively is more work, but it will run faster and use less space than the virtual machine.   

We also have lab computers with donkeysim set up available to use, if you run into difficulties with setting up donkeysim on your computer don't hesitate to use the lab computers. We also have joysticks in the lab which make driving much more convenient.  

These instructions are new for Fall 2024, and the donkey_sim app was just compiled for ARM macs by me (Alexander). Please let me know about any issues/improvements!  

Download the donkeysim video game from here:  
[https://drive.google.com/file/d/1FRDT7DiuKDhAuoKyqt8KyMnRAYsALx5Y/view?usp=sharing](https://drive.google.com/file/d/1FRDT7DiuKDhAuoKyqt8KyMnRAYsALx5Y/view?usp=sharing)  
To run the app, you may need to go to your security settings and click "run anyway," although it is not from an identified developer.    
Note that the arm64 compiled simulator does have a known bug where there are a few invisible cones outside the track that your car can collide with. I am working on fixing it!  

[https://docs.donkeycar.com/guide/deep_learning/simulator/](https://docs.donkeycar.com/guide/deep_learning/simulator/) is what I referenced when writing this document. For your convenience I will write out the steps here:

1. Download miniconda from [https://conda-forge.org/download/](https://conda-forge.org/download/), and run the bash script to install it. You can open your terminal app, and ```cd Downloads``` and then ```bash Miniforge3-MACOSX-arm64.sh```
Note: If you get prompted asking to change your shell to zsh by the terminal, please change the shell to zsh with the suggested command.
3. Follow all the prompts the script provides
4. Once the script is installed, it will ask if you want to automatically start conda by default. I suggest putting no, but then note you will have to run the command ```eval "$(/Users/YOUR_USER/miniforge3/bin/conda shell.zsh hook)"``` to start conda every time you start the terminal. The terminal should tell you the proper path.
5. Restart your computer to get the install to complete. Then open the terminal, and start conda with the command above.   
6. ```conda create -n donkey python=3.11```  
7. ```conda activate donkey```  
8. ```pip install donkeycar\[pc\]```
9. ```pip install git+https://github.com/tawnkramer/gym-donkeycar```
10. ```donkey createcar --path ./mycar```
11. ```cd mysim```
12. ```nano myconfig.py```
13. ```python manage.py drive```
14. ```cd mycar``` then ```open myconfig.py```
15. Edit myconfig.py to add in the lines
    ```DONKEY_GYM = True  ```
    
```DONKEY_SIM_PATH = "remote"  ```

```DONKEY_GYM_ENV_NAME = "donkey-warren-track-v0"```

```SIM_HOST = "127.0.0.1" ``` 

```GYM_CONF = { "body_style" : "car01", "body_rgb" : (255, 205, 0), "car_name" : "UCSD-148-YourName", "font_size" : 30} # body style(donkey|bare|car01) body rgb 0-255  ```

```GYM_CONF["racer_name"] = "UCSD-148-YourName"  ```

```GYM_CONF["country"] = "USA"  ```

```GYM_CONF["bio"] = "pls work"  ```

And then save and exit

12. Start the donkey_sim app, and then run ```python3 manage.py drive```
13. You should be able to go to http://localhost:8887 to control the car. Note, it will be easier to control the car if you hook up a joystick. Instructions to do that are in the other virtual machine document.
14. Follow the other document for the rest of the instructions. For here on everything should work the same.
