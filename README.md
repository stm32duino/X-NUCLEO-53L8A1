# X-NUCLEO-53L8A1

Arduino library to support the X-NUCLEO-53L8A1 based on VL53L8CX Time-of-Flight 8x8 multizone ranging sensor with wide field view.

This sensor uses I2C or SPI to communicate. I2C or SPI instance is required to access to the sensor.
The APIs provide simple distance measure and multizone detection in both polling and interrupt modes.

## Examples

There are 6 examples with the X-NUCLEO-53L8A1 library using I2C and SPI. 
To use the X_NUCLEO_53L8A1 in SPI mode J7, J8, J9 jumpers must be on SPI.

* X_NUCLEO_53L8A1_HelloWorld_I2C: This example code is to show how to get multizone detection and proximity values of the VL53L8CX sensor in polling mode using I2C communication.

* X_NUCLEO_53L8A1_HelloWorld_SPI: This example code is to show how to get multizone detection and proximity values of the VL53L8CX sensor in polling mode using SPI communication.

* X_NUCLEO_53L8A1_ThresholdsDetection_I2C: This example code is to show how to configure the thresholds detection in interrupt mode using I2C communication.

* X_NUCLEO_53L8A1_ThresholdsDetection_SPI: This example code is to show how to configure the thresholds detection in interrupt mode using SPI communication.

* X_NUCLEO_53L8A1__MultiSensorRanging_I2C: This example code is to show how to make three VL53L8CX ToF sensors run simultaneously in polling mode using I2C communication.
  
* X_NUCLEO_53L8A1__MultiSensorRanging_SPI: This example code is to show how to make three VL53L8CX ToF sensors run simultaneously in polling mode using SPI communication.

## Dependencies

This package requires the following Arduino library:

* STM32duino VL53L8CX: https://github.com/stm32duino/VL53L8CX


## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-53L8A1

The VL53L8CX datasheet is available at
https://www.st.com/en/imaging-and-photonics-solutions/vl53L8cx.html
