# MotionMoteTx
OpenEnergyMonitor motion sensor based on MotionMote from LowPowerLab.

The idea of this project is to have a motion-sensing power-save mode on an kiosk display controlled by a Raspberry Pi, such as a Dakboard.

The flow is: motion sensed > jeelib radio packet sent > emonHub recieves packet and updates MQTT topic > node-RED gets message and turns on connected display.

Install the included shell script in /home/pi/rpi-hdmi.sh on the Raspberry Pi connected to the display.

Then do this to make the script executable:
chmod +x /home/pi/rpi-hdmi.sh

Use the Arduino IDE to install the MotionMoteTx.ino firmware from this repository in a MotionMote from lowpowerlab.com

Update your emonhub.conf on your emonHub or emonPi from OpenEnergyMonitor.org to include your new MotionMoteTx node. I used node 9, but use whatever node you want.

[[9]]
   nodename = MotionMoteTx9
   [[[rx]]]
      names = motion, temperature, humidity, pressure, battery
      datacode = h
      scales = 1,.1,.1,.01,1
      units = "",F,%,in,V

Install Node-RED if you haven't already done so. It is best to go to https://nodered.org and learn how to get the latest version for the Rasberry Pi, rather than use the default install from Raspberry Pi Foundation.

In Node-RED got to Manage Palette and install the module node-red-dashboard.

Install the Node-RED flow from this repository and modify to connect it to your MQTT server. Update the topic emon/MotionMoteTx9/motion to use your nodename from the step above.


