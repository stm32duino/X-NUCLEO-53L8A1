/**
 ******************************************************************************
 * @file    X_NUCLEO_53L7A1_MultiSensorRanging_I2C.ino
 * @author  STMicroelectronics
 * @version V2.0.0
 * @date    27 June 2024
 * @brief   Arduino test application for the X-NUCLEO-53L7A1 based on VL53L8CX
 *          proximity sensor.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2024 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include <vl53l8cx.h>

#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#else
  #define DEV_I2C Wire
#endif
#define SerialPort Serial

/* Please uncomment the line below if you want also to use the satellites */
#define SATELLITES_MOUNTED

#define LPN_TOP_PIN A3
#define PWREN_TOP_PIN 11

#ifdef SATELLITES_MOUNTED
  #define LPN_LEFT_PIN A0
  #define PWREN_LEFT_PIN 6

  #define LPN_RIGHT_PIN 13
  #define PWREN_RIGHT_PIN 12
#endif

// Components.
VL53L8CX sensor_vl53l8cx_top(&DEV_I2C, LPN_TOP_PIN);
#ifdef SATELLITES_MOUNTED
  VL53L8CX sensor_vl53l8cx_left(&DEV_I2C, LPN_LEFT_PIN);
  VL53L8CX sensor_vl53l8cx_right(&DEV_I2C, LPN_RIGHT_PIN);
#endif

uint8_t status;

/* Setup ---------------------------------------------------------------------*/

void setup()
{
  // Initialize serial for output.
  SerialPort.begin(460800);

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Enable PWREN top pin if present
  if (PWREN_TOP_PIN >= 0) {
    pinMode(PWREN_TOP_PIN, OUTPUT);
    digitalWrite(PWREN_TOP_PIN, HIGH);
    delay(10);
  }

#ifdef SATELLITES_MOUNTED
  // Enable PWREN left pin if present
  if (PWREN_LEFT_PIN >= 0) {
    pinMode(PWREN_LEFT_PIN, OUTPUT);
    digitalWrite(PWREN_LEFT_PIN, HIGH);
    delay(10);
  }
  // Enable PWREN right pin if present
  if (PWREN_RIGHT_PIN >= 0) {
    pinMode(PWREN_RIGHT_PIN, OUTPUT);
    digitalWrite(PWREN_RIGHT_PIN, HIGH);
    delay(10);
  }
#endif

  // Configure VL53L8CX top component.
  sensor_vl53l8cx_top.begin();
  // Switch off VL53L8CX top component.
  sensor_vl53l8cx_top.off();

#ifdef SATELLITES_MOUNTED
  // Configure (if present) VL53L8CX left component.
  sensor_vl53l8cx_left.begin();
  //Switch off (if present) VL53L8CX left component.
  sensor_vl53l8cx_left.off();

  // Configure (if present) VL53L8CX right component.
  sensor_vl53l8cx_right.begin();
  // Switch off (if present) VL53L8CX right component.
  sensor_vl53l8cx_right.off();
#endif
  //Initialize all the sensors
  sensor_vl53l8cx_top.on();
  status = sensor_vl53l8cx_top.set_i2c_address(0x10);
  status = sensor_vl53l8cx_top.init();

#ifdef SATELLITES_MOUNTED
  sensor_vl53l8cx_left.on();
  status = sensor_vl53l8cx_left.set_i2c_address(0x12);
  status = sensor_vl53l8cx_left.init();
  sensor_vl53l8cx_right.on();
  status = sensor_vl53l8cx_right.set_i2c_address(0x14);
  status = sensor_vl53l8cx_right.init();
#endif

  // Start Measurements
  status = sensor_vl53l8cx_top.start_ranging();
#ifdef SATELLITES_MOUNTED
  status = sensor_vl53l8cx_left.start_ranging();
  status = sensor_vl53l8cx_right.start_ranging();


#endif
}

void loop()
{
  VL53L8CX_ResultsData results;
  uint8_t NewDataReady = 0;
  char report[128];
  uint8_t status;

  do {
    status = sensor_vl53l8cx_top.check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    // Read measured distance. RangeStatus = 5 and 9 means valid data
    status = sensor_vl53l8cx_top.get_ranging_data(&results);

    snprintf(report, sizeof(report), "VL53L8CX Top: Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
             results.target_status[0],
             results.distance_mm[0],
             results.signal_per_spad[0]);
    SerialPort.print(report);
  }

#ifdef SATELLITES_MOUNTED
  NewDataReady = 0;

  do {
    status = sensor_vl53l8cx_left.check_data_ready(&NewDataReady);
  } while (!NewDataReady);


  if ((!status) && (NewDataReady != 0)) {
    // Read measured distance. RangeStatus = 5 and 9 means valid data
    status = sensor_vl53l8cx_left.get_ranging_data(&results);
    snprintf(report, sizeof(report), "VL53L8CX Left: Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
             results.target_status[0],
             results.distance_mm[0],
             results.signal_per_spad[0]);
    SerialPort.print(report);
  }

  NewDataReady = 0;
  do {
    status = sensor_vl53l8cx_right.check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    // Read measured distance. RangeStatus = 5 and 9 means valid data
    status = sensor_vl53l8cx_right.get_ranging_data(&results);
    snprintf(report, sizeof(report), "VL53L8CX Right: Status = %3u, Distance = %5u mm, Signal = %6u kcps/spad\r\n",
             results.target_status[0],
             results.distance_mm[0],
             results.signal_per_spad[0]);
    SerialPort.print(report);
  }

#endif
}
