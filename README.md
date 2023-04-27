# temi_fleet_adapter_python

This fleet adapter is written for the Temi robot. It uses [temi-mqtt-client](https://github.com/open-rmf/temi-mqtt-client) app (forked from [hapi-robo/temi-mqtt-client](https://github.com/hapi-robo/temi-mqtt-client)) and [pytemi](https://github.com/hapi-robo/pytemi) package for communication between Temi and RMF over MQTT.

## Connect to Temi robot

Go to `Settings` then `Developer Tools` to access your Temi's ADB shell. This is required for you to install and start applications on your robot.


## Setup

### 1. Install dependencies
```
pip3 install nudged aiohttp
```


### 2. Set up a MQTT Broker

Configure MQTT on your PC with an authorized user and password. These pieces of information will be filled in your MQTT client app later on.

- Install MQTT
```
sudo apt install mosquitto mosquitto_clients
```

- Update the `.conf` and `acl` files for mosquitto if you have not already done so. Add the appropriate MQTT user and password and allow readwrite access to the `temi/#` topic.

Sample `myconfig.conf`:
```
persistence false

#mqtt
listener 1883
protocol mqtt

# websockets
listener 9001
protocol websockets

allow_anonymous true
password_file /etc/mosquitto/passwd

acl_file /etc/mosquitto/acl
```

Sample `/etc/mosquitto/acl`:
```
# Allow anonymous access to the sys
topic read $SYS/#

# Allow anonymous to read temi
topic read temi/#

# For user rmf_temi_mqtt
user rmf_temi_mqtt
topic temi/#
```

- Restart mosquitto once you're done updating

```bash
sudo systemctl restart mosquitto
```


### 3. Preparing the temi_mqtt_client APK

1. Clone [temi_mqtt_client] to your workspace
2. Prepare a `secrets.properties` file in your root folder. It should have the information below filled out. The serial number can be found via your Temi in `Settings` -> `About` -> `Serial Number`.
```
MQTT_HOSTNAME="<mqtt-broker-hostname>"
MQTT_USERNAME="<mqtt-broker-username>"
MQTT_PASSWORD="<mqtt-broker-password>"
ROBOT_NAME="<robot-name>"
ROBOT_SERIAL="<robot-serial-number>
```
1. Build the APK in Android Studio and `adb install` the app using the APK file.
```bash
adb install TemiMqttClient.apk
```
1. Start the app on your Temi, filling out the hostname if necessary
```bash
adb shell # to enter Temi's terminal

# inside terminal
# list packages to find temi-mqtt-client's full package name
pm list packages | grep mqtt

# list activities to find associated Activity
dumpsys package | grep mqtt | grep Activity

# start the application
am start -n com.hrst.temi_mqtt_client/.MainActivity
```
5. Press the `Connect` button on your screen

Your Temi should **now** be connected to your MQTT Broker.

### Install RMF

If you have not already done so, install RMF (instructions available on [open-rmf/rmf](https://github.com/open-rmf/rmf)).


### Preparing config for the fleet adapter
Fill out the `config.yaml` file with some important configuration, such as your robot name and serial number. You should also provide transformation mappings between Temi's and RMF's coordinates.


## Try it out

Before starting the fleet adapter, make sure to source your workspace with both RMF and the fleet adapter.
```bash
source ~/rmf_temi_ws/install/setup.bash
```

With your fleet adapter config file and navigation graphs ready, you can run the fleet adapter:
```bash
ros2 run temi_fleet_adapter_python full_control_fleet_adapter -c path_to_config_file -n path_to_nav_graph
```


## Additional features

The `temi-mqtt-client` app also offers video activity and teleoperation capabilities for Temi that can be used alongside [RMF Web](https://github.com/open-rmf/rmf-web).

### Set up RMF Web

Clone the RMF Web [taskv2-teleop](https://github.com/open-rmf/rmf-web/tree/taskv2-teleop) branch and set up according to its README instructions.

### Try it out

Launch RMF, your Temi fleet adapter, and RMF Web. A teleop tab should be visible in the web UI with buttons to tele-operate your robot, as well as an option to start a video call between the web and Temi's screen.