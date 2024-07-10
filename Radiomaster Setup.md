# Radiomaster Controller Setup

## Hardware Setup

### Required Hardware includes:

- 5V Relay
- Arduino Pro Micro
- Radiomaster receiver (RX)
- Radiomaster controller (TX)
- 2x 3.7 LiPo cells

The relay, Arduino, and receiver should be wired together according to the schematic. *Do not forget to put the cells into the controller. The LED will illuminate without them, but the controller won't turn on.*

### Flashing the Arduino

To flash the Arduino, one needs the ELRS Controller files at this Github repo:

https://github.com/NikitaB04/ELRSController

Additionally, one needs the PlatformIO extension/IDE in VSCode.

Once PlatformIO is installed, open the IDE using the icon on the left. Then open the ELRS Controller folder (open it as a PlatformIO project) and move to the main.cpp file.

With the Arduino plugged in, make sure the correct COM port is selected at the bottom edge of the screen. Then select **Upload**, also listed at the bottom of the screen.

If there are no errors, then the code has uploaded successfully; otherwise, fix them and try uploading again.

### Installing ELRS Configurator

Install the configurator according to your system OS:

https://www.expresslrs.org/quick-start/installing-configurator/

### Installing TX Firmware

You can flash the TX firmware in one of 2 ways (I have not tried the Uart method)

- Through EdgeTXPassthrough (over USB cable)
- Via Wifi (only for first-time setup)

These are selected with the **Flashing Method** setting in the Configurator.

#### Firmware Settings
Select the following settings for your transmitter under the **Configurator** tab. **Make sure to use a unique binding phrase.**

<image>

#### EdgeTX Passthrough

Once the configurator settings are selected, make sure the correct serial device is selected.

The controller must be on beforehand. Plug it in using the top USB port and select the USB serial option that appears onscreen using the cylindrical roller (pressing = select) to navigate the menu.

The port should resemble

    /dev/tty/ACM0 EdgeTX

Flash the firmware. Select to download the Lua script generated so you can transfer it to your controller.

Unplug the controller and power it off. Then remove the SD card at the bottom. Connect it to your computer and navigate to

    SCRIPTS/TOOLS/

Drop the Lua file here, replacing the previous one.


#### Selecting Wifi

If you are installing via Wifi, build/download the firmware. Once Wifi is enabled on the controller, you will choose the firmware file to install.

##### Enabling TX Wifi

Once the controller is charged/is on, use the cylindrical roller (pressing = select) to navigate the menu.

Press the **SYS** button to bring up the **Tools** menu. Select the first entry *ExpressLRS.*

Navigate the menu until you reach the option **Wifi Connectivity** and select it.

Select the **Enable Wifi** option. The controller will begin broadcasting a Wifi network:

- SSID: ExpressLRS TX
- pass: expresslrs

Join the network. A window will open at 10.0.0.1. Select the **Wifi** tab and browse your machine for the firmware you built above. Start the install process; once it completes, be sure to download the Lua script it displays.

Check that the firmware update was successful by returning to **SYS/Tools/ExpressLRS** menu on the controller. Navigating to the bottom of the menu should show the firmware version like so:

<image>

### Installing RX Firmware

Similar to the controller, select the following settings in the configurator for the RX firmware, **USE THE SAME BINDING PHRASE AS THE TX**:

<image>

Build the firmware, making sure to set the baudrate to 115200. The receiver must be in Wifi mode (breathing green LED), which occurs after waiting 60 seconds after powering on. **The RX needs more power than the Arduino can provide on its 5V pin, otherwise it resets each time it enalbes Wifi mode. Using the buck converter is advised.**

Connect to the RX network on your machine.

- SSID: ExpressLRS RX
- Pass: expresslrs

This should bring up a menu just as with the TX. Supply the firmware.bin file you downloaded.

#### Using Wifi after first flash

If you need to reflash your receiver after you initially flashed it (say you need to pick a new binding phrase), it is still possible to flash it using Wifi.

Connect to the network you assigned the RX in the initial flash.

Set all the settings you want in the configurator and select Wifi as the flashing method.

Power cycle the RX 3-4 times (each time waiting for the LED to turn on, then unplugging)

Wait for the RX to enter Wifi mode

At the bottom of the configurator menu there will be **Network Devices** section.

Select your receiver (it may be difficult to identify if there are multiple RXs on the network)

Flash the firmware.

Once the LED begins flashing again you can power the RX off (it will reenter Wifi mode if left on)

The next time the RX turns on when the TX is on, they will be bound based on the binding phrase match.

### Binding the TX and RX

Once both TX and RX firmware are updated, they should be bound together once they are both powered due to having the same binding phrase.

### Input Configuration/Troubleshooting

Theoretically your inputs should be read by default once the TX and RX are bound. If that is not the case, return to **SYS/Tools/ExpressLRS**. Scroll to the bottom and select **Other Devices**, then select your RX. Once loaded, change **CSRF** to **SBUS**.

### VESC Setup

To activate the kill switch on the VESC, the setting must be enabled in the VESC tool. Navigate to **General** under **App Settings** and find the **Kill Switch Mode** setting.

Verify your the throw configuration of the relay (i.e. if ground or 5V is on NC or NO). Here 5V is normally closed, so to enable the kill switch we want to set **Kill Switch Mode** to **ADC2 Low** since ground is normally open.