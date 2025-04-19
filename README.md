# Flir Lepton ROS package
A ROS 2 node for the Flir Lepton 3. Fuctioning but not finalized. Tested on the NVIDIA Jetson Nano, linked to I2C Bus 1

## Pin Mapping

Camera Pin  | Nvidia Jetson Nano Pin         
----------- | :-----------------------
CS          | GPIO_PZ0 (31)
MOSI        | SPI_1_MOSI (19)
MISO        | SPI_1_MISO (21)
CLK         | SPI_1_SCK (23)
GND         | GND (9)
VIN         | 3.3 VDC Power (1)
SDA         | I2C_2_SDA, I2C Bus 1 (3)
SCL         | I2C_2_SCL, I2C Bus 1 (5)

## flir_lepton_node

### Publish
  - `~/camera_thermal/image`: `sensor_msgs/msg/Image`

### Execution
```bash
ros2 run flir_lepton flir_lepton
```
## Acknowledgements
This package is based built off of the ROS 1 node https://github.com/turing-lab/flir_lepton

And the flir lepton SDK implementation from https://github.com/Myzhar/Lepton3_Jetson.

Given the nature of the the source material, this can likely work for other spi i2c compatable devices such as the raspberry pi. 

### Services (OLD)
  - `~/performFCC`: No arguments

### Parameters (OLD)
  - `~/typeColormap`: int, default: 3, (`RAINBOW` = 1, `GRAYSCALE` = 2, `IRONBLACK` = 3)
  - `~/typeLepton`: int, default: 2, (Lepton 2.x = 2, Lepton 3.x = 3)
  - `~/spiSpeed`: int [Mhz], default: 20, (10 - 30)
  - `~/rangeMin`: int, default: automatic scaling range adjustment, (0 - 65535)
  - `~/rangeMax`: int, default: automatic scaling range adjustment, (0 - 65535)
  - `~/autoScale`: int, default: 1 (yes = 1 or no = 0)
  - `~/loglevel`: int, default: 0 (0 - 255)

