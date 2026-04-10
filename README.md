# temi_fleet_adapter_python

This fleet adapter is written for the Temi robot. It uses [temi-mqtt-client](https://github.com/open-rmf/temi-mqtt-client) app (forked from [hapi-robo/temi-mqtt-client](https://github.com/hapi-robo/temi-mqtt-client)) and [pytemi](https://github.com/hapi-robo/pytemi) package for communication between Temi and RMF over MQTT.


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

- Update the `.conf` and `acl` files for mosquitto if you have not already done so. Add the appropriate MQTT user and password and allow readwrite access to the `temi/#` topic. Here we're using `rmf_temi_mqtt` as the user.

Sample `myconfig.conf`:
```
persistence true
persistence_location /var/lib/mosquitto/

listener 1883
protocol mqtt

# websockets
listener 9001
protocol websockets

allow_anonymous true

acl_file /etc/mosquitto/acl

log_dest file /var/log/mosquitto/mosquitto.log

include_dir /etc/mosquitto/conf.d

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

Set up a mosquitto password if you haven't:
```
sudo mosquitto_passwd -c /etc/mosquitto/passwd rmf_temi_mqtt
```

- Restart mosquitto once you're done updating

```bash
sudo systemctl restart mosquitto
```


### 3a. Preparing the temi_mqtt_client APK

1. Clone [temi-mqtt-client](https://github.com/open-rmf/temi-mqtt-client/tree/xiyu/temi) to your workspace (check out to `xiyu/temi` branch)
2. Prepare a `secrets.properties` file in your `temi-mqtt-client` root folder. It should have the information below filled out. The serial number can be found via your Temi in `Settings` -> `About` -> `Serial Number`.
```
MQTT_HOSTNAME="<mqtt-broker-hostname>"
MQTT_USERNAME="<mqtt-broker-username>"
MQTT_PASSWORD="<mqtt-broker-password>"
ROBOT_NAME="<robot-name>"
ROBOT_SERIAL="<robot-serial-number>
```
3. Build the APK in Android Studio.

### 3b. Install the APK on your Temi

1. Connect to the Temi via adb
  - On Temi, go to Settings > Developer Tools > ADB Connection. Click on `Open Port`.
  - On your PC, connect to the Temi using the displayed IP.
```bash
adb connect x.x.x.x
```

2. Install the APK from the directory where the .apk is:
```bash
adb install TemiMqttClient.apk
```

3. Start the app on your Temi
```bash
# list packages to find temi-mqtt-client's full package name
adb shell pm list packages | grep mqtt

# start the application
adb shell am start -n com.hrst.temi_mqtt_client/.MainActivity
```
4. Press the `Connect` button on your screen

Your Temi should **now** be connected to your MQTT Broker.

If your MQTT broker IP may change after building the client APK, you can update the address via the application on the Temi in the textbox above `Connect`.

### 3c. Stop/Uninstall the APK on your Temi

To stop the running application,
```bash
adb shell am force-stop com.hrst.temi_mqtt_client
```

To uninstall the application from your Temi,
```bash
adb uninstall com.hrst.temi_mqtt_client
```

## Install RMF

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