# ROS driver for ks103 Ultrasound Range Finder

This ros package is designed to drive multiple ks103 sensors on a RaspberryPi.

## Prerequisites

1. A RaspberryPi, having Ubuntu, ROS installed
2. Having permission to i2c device (test by running `i2c-detect -y 1`, you will need `i2c-tools` for this). The following commands help grant i2c permission to user.

    ```bash
    sudo groupadd i2c
    sudo chown :i2c /dev/i2c-1
    sudo chmod g+rw /dev/i2c-1
    sudo usermod -aG i2c $USER
    ```

3. Having ks103 sensors connected to RaspberryPi's default I2C SDA/SCL pins

## How to Use

Set the params in `launch/example.yml`, then

```
roslaunch ks103-ultrasound example.launch
```

Range are advertised in topics `range/i` (`sensor_msgs/Range`), in which `i` being the index of each ks103 set in `launch/example.yml`.