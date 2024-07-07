# -*- coding: utf-8 -*-
""" 
  @file DFRobot_MultiGasSensor.py
  @note DFRobot_MultiGasSensor Class infrastructure, implementation of underlying methods
  @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      [PengKaixing](kaixing.peng@dfrobot.com)
  @version  V2.0
  @date  2021-03-31
  @url https://github.com/DFRobot/DFRobot_MultiGasSensor
"""
import serial
import time
import smbus
import spidev
import os
import math
import RPi.GPIO as GPIO
import logging
import paho.mqtt.client as mqtt

hostname = "192.168.1.134"
broker_port = 1883

topic = "df_robot_gas"
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print("Failed to connect, return code %d\n", rc)

def on_message(client, userdata, msg):
    print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

client.on_connect = on_connect
client.on_message = on_message
client.connect(hostname, broker_port, 60)

client.loop_start()

logger = logging.getLogger()
logger.setLevel(logging.INFO)  # Display all the print information
# logger.setLevel(logging.FATAL)# Use this option if you don't want to display too many prints but only printing errors
ph = logging.StreamHandler()
formatter = logging.Formatter("%(asctime)s - [%(filename)s %(funcName)s]:%(lineno)d - %(levelname)s: %(message)s")
ph.setFormatter(formatter)
logger.addHandler(ph)

I2C_MODE = 0x01
UART_MODE = 0x02

sendbuf = [0] * 9
recvbuf = [0] * 9

def fuc_check_sum(i, ln):
    '''!
      @brief CRC check function
      @param i  CRC original data list
      @param ln Length
      @return CRC check value
    '''
    tempq = 0
    for j in range(1, ln - 1):
        tempq += i[j]
    tempq = (((~tempq) & 0xff) + 1)
    return tempq

def clear_buffer(buf, length):
    '''!
      @brief List values are reset
      @param buf List to be cleared
      @param length Length
    '''
    for i in range(0, length):
        buf[i] = 0

class DFRobot_GasType:
    '''!
      @brief Enumerates all known sensor types. DFRobot_MultiGasSensor.gastype
      @n     will be set to one of these.
    '''
    O2 = "O2"
    CO = "CO"
    H2S = "H2S"
    NO2 = "NO2"
    O3 = "O3"
    CL2 = "CL2"
    NH3 = "NH3"
    H2 = "H2"
    HCL = "HCL"
    SO2 = "SO2"
    HF = "HF"
    PH3 = "PH3"
    UNKNOWN = ""

class DFRobot_MultiGasSensor(object):
    '''!
      @brief This is a sensor parent class which can be used in complex environments to detect various gases.
      @details To detect gases like O2, CO, H2S, NO2, O3, CL2, NH3, H2, HCL, 
      @n       SO2, HF, PH3, which is achieved by just switching corresponding probes.
      @n       Meanwihle, it supports gas high/low threshold alarm.
      @n       Function
    '''  
    INITIATIVE = 0x03
    PASSIVITY = 0x04
    O2 = 0x05
    CO = 0x04
    H2S = 0x03
    NO2 = 0x2C
    O3 = 0x2A
    CL2 = 0x31
    NH3 = 0x02
    H2 = 0x06
    HCL = 0X2E
    SO2 = 0X2B
    HF = 0x33
    PH3 = 0x45
    GASCON = 0x00
    GASKIND = 0x01
    ON = 0x01
    OFF = 0x00
    gasconcentration = 0.0 # Raw, uncorrected sensor measurement.
    gastype = ""
    gasunits = ""
    temp = 0.0
    tempSwitch = OFF

    def __init__(self, bus, Baud):
        if bus != 0:
            self.i2cbus = smbus.SMBus(bus)
            self.__uart_i2c = I2C_MODE
        else:
            self.ser = serial.Serial("/dev/ttyAMA0", baudrate=Baud, stopbits=1)
            self.__uart_i2c = UART_MODE
            if self.ser.isOpen == False:
                self.ser.open()

    def __getitem__(self, k):
        if k == recvbuf:
            return recvbuf

    def __set_gastype(self, probe_type):
        '''!
          @brief   Sets instance gas type and units based on type read from sensor.
          @param probe_type Byte received from sensor indicating sensor type.
        '''
        if probe_type == self.O2:
            self.gastype = DFRobot_GasType.O2
            self.gasunits = "%%"
        elif probe_type == self.CO:
            self.gastype = DFRobot_GasType.CO
            self.gasunits = "ppm"
        elif probe_type == self.H2S:
            self.gastype = DFRobot_GasType.H2S
            self.gasunits = "ppm"
        elif probe_type == self.NO2:
            self.gasunits = "ppm"
            self.gastype = DFRobot_GasType.NO2
        elif probe_type == self.O3:
            self.gasunits = "ppm"
            self.gastype = DFRobot_GasType.O3
        elif probe_type == self.CL2:
            self.gasunits = "ppm"
            self.gastype = DFRobot_GasType.CL2
        elif probe_type == self.NH3:
            self.gasunits = "ppm"
            self.gastype = DFRobot_GasType.NH3
        elif probe_type == self.H2:
            self.gastype = DFRobot_GasType.H2
            self.gasunits = "ppm"
        elif probe_type == self.HCL:
            self.gastype = DFRobot_GasType.HCL
            self.gasunits = "ppm"
        elif probe_type == self.SO2:
            self.gastype = DFRobot_GasType.SO2
            self.gasunits = "ppm"
        elif probe_type == self.HF:
            self.gastype = DFRobot_GasType.HF
            self.gasunits = "ppm"
        elif probe_type == self.PH3:
            self.gastype = DFRobot_GasType.PH3
            self.gasunits = "ppm"
        else:
            self.gastype = DFRobot_GasType.UNKNOWN
            self.gasunits = ""

    def __adc_to_temp(self, temp_ADC):
        '''!
          @brief Converts temperature ADC measurement to temperature.
          @param temp_ADC 10-bit A/D measurement from onboard temperature sensor.
        '''
        Vpd3 = float(temp_ADC / 1024.0) * 3
        Rth = Vpd3 * 10000 / (3 - Vpd3)
        return 1 / (1 / (273.15 + 25) + 1 / 3380.13 * (math.log(Rth / 10000))) - 273.15

    def __temp_correction(self, Con):
        '''!
          @brief Performs temperature correction of sensor value.
          @param Con Measured value from sensor.
        '''

        if self.tempSwitch != self.ON:
            return 0.0

        if self.gastype == DFRobot_GasType.O2:
            pass
        elif self.gastype == DFRobot_GasType.CO:
            if (self.temp > -40) and (self.temp <= 20):
                Con = Con / (0.005 * self.temp + 0.9)
            elif (self.temp > 20) and (self.temp <= 40):
                Con = Con / (0.005 * self.temp + 0.9) - (0.3 * self.temp - 6)
            else:
                Con = 0.0
        elif self.gastype == DFRobot_GasType.H2S:
            if (self.temp > -20) and (self.temp <= 20):
                Con = Con / (0.005 * self.temp + 0.92)
            elif (self.temp > 20) and (self.temp <= 60):
                Con = Con / (-0.005 * self.temp + 1.12)
            else:
                Con = 0.0
        elif self.gastype == DFRobot_GasType.NO2:
            if (self.temp > -30) and (self.temp <= 20):
                Con = Con / (0.005 * self.temp + 0.8)
            elif (self.temp > 20) and (self.temp <= 45):
                Con = Con / (-0.005 * self.temp + 1.2)
            else:
                Con = 0.0
        elif self.gastype == DFRobot_GasType.O3:
            if (self.temp > -20) and (self.temp <= 20):
                Con = Con / (0.01 * self.temp + 0.9)
            elif (self.temp > 20) and (self.temp <= 40):
                Con = Con / (0.01 * self.temp + 0.8)
            else:
                Con = 0.0
        elif self.gastype == DFRobot_GasType.CL2:
            if (self.temp > -40) and (self.temp <= 20):
                Con = Con / (0.005 * self.temp + 0.9)
            elif (self.temp > 20) and (self.temp <= 40):
                Con = Con / (0.01 * self.temp + 0.7)
            else:
                Con = 0.0
        elif self.gastype == DFRobot_GasType.NH3:
            if (self.temp > -40) and (self.temp <= 0):
                Con = Con / (0.01 * self.temp + 0.9)
            elif (self.temp > 0) and (self.temp <= 20):
                Con = Con / (-0.005 * self.temp + 1)
            elif (self.temp > 20) and (self.temp <= 40):
                Con = Con / (-0.01 * self.temp + 1.1)
            else:
                Con = 0.0
        elif self.gastype == DFRobot_GasType.H2:
            if (self.temp > -40) and (self.temp <= 20):
                Con = Con / (0.005 * self.temp + 0.9)
            elif (self.temp > 20) and (self.temp <= 60):
                Con = Con / (-0.005 * self.temp + 1.1)
            else:
                Con = 0.0
        elif self.gastype == DFRobot_GasType.HCL:
            if (self.temp > -30) and (self.temp <= 20):
                Con = Con / (0.005 * self.temp + 0.8)
            elif (self.temp > 20) and (self.temp <= 50):
                Con = Con / (-0.005 * self.temp + 1.2)
            else:
                Con = 0.0
        elif self.gastype == DFRobot_GasType.SO2:
            if (self.temp > -40) and (self.temp <= 20):
                Con = Con / (0.01 * self.temp + 0.8)
            elif (self.temp > 20) and (self.temp <= 40):
                Con = Con / (-0.005 * self.temp + 1.2)
            else:
                Con = 0.0
        elif self.gastype == DFRobot_GasType.HF:
            if (self.temp > -20) and (self.temp <= 20):
                Con = Con / (0.01 * self.temp + 1)
            elif (self.temp > 20) and (self.temp <= 50):
                Con = Con / (-0.01 * self.temp + 1.4)
            else:
                Con = 0.0
        elif self.gastype == DFRobot_GasType.PH3:
            if (self.temp > -30) and (self.temp <= 20):
                Con = Con / (0.01 * self.temp + 0.9)
            elif (self.temp > 20) and (self.temp <= 50):
                Con = Con / (-0.01 * self.temp + 1.3)
            else:
                Con = 0.0

        return Con

    def read_gas_concentration(self):
        '''!
          @brief Read the concentration of gas
          @return The gas concentration of gas
        '''
        return self.gasconcentration

    def read_gas_type(self):
        '''!
          @brief Read the gas type
          @return Gas type
        '''
        return self.gastype

    def read_temp(self):
        '''!
          @brief Read temperature
          @return Temperature
        '''
        return self.temp

    def change_acquire_mode(self, mode):
        '''!
          @brief Set mode to "initiative" or "passive" reporting.
          @param mode Can be either INITIATIVE or PASSIVITY.
        '''
        clear_buffer(sendbuf, 9)
        sendbuf[0] = 0x11
        sendbuf[1] = 0x02
        sendbuf[2] = mode
        sendbuf[8] = fuc_check_sum(sendbuf, 9)
        self.__send_data()

    def __send_data(self):
        '''!
          @brief Send the data
        '''
        if self.__uart_i2c == UART_MODE:
            self.ser.write(sendbuf)
        else:
            self.i2cbus.write_i2c_block_data(0x73, 0x11, sendbuf)
        time.sleep(0.2)

    def __recv_data(self):
        '''!
          @brief receive the data
        '''
        if self.__uart_i2c == UART_MODE:
            while self.ser.inWaiting() == 0:
                time.sleep(0.1)
            self.ser.readinto(recvbuf)
        else:
            recvbuf = self.i2cbus.read_i2c_block_data(0x73, 0x11, 9)
        time.sleep(0.2)

    def __analyse_data(self):
        '''!
          @brief Analyze the data
        '''
        if recvbuf[0] != 0x16:
            logger.info("recvData error")
        elif recvbuf[8] != fuc_check_sum(recvbuf, 9):
            logger.info("check sum error")
        elif recvbuf[1] == 0x09:
            self.gasconcentration = (recvbuf[2] << 24 | recvbuf[3] << 16 | recvbuf[4] << 8 | recvbuf[5]) / 100000.0
            self.temp = self.__adc_to_temp((recvbuf[6] << 8 | recvbuf[7]) & 0x3ff)
            self.gasconcentration = self.__temp_correction(self.gasconcentration)
            logger.info(f"gastype: {self.gastype} Concentration: {self.gasconcentration} {self.gasunits} temp: {self.temp:.2f}")
        elif recvbuf[1] == 0x07:
            self.__set_gastype(recvbuf[2])

    def set_temp_compensation(self, OnOff):
        '''!
          @brief Enable/disable temperature compensation.
          @param OnOff Use ON or OFF constant.
        '''
        self.tempSwitch = OnOff

    def acquire_data(self):
        '''!
          @brief Get the data of concentration and type
        '''
        clear_buffer(sendbuf, 9)
        sendbuf[0] = 0x11
        sendbuf[1] = 0x01
        sendbuf[8] = fuc_check_sum(sendbuf, 9)
        self.__send_data()
        self.__recv_data()
        self.__analyse_data()

if __name__ == "__main__":
    # Initialize the sensor (I2C mode)
    sensor = DFRobot_MultiGasSensor(bus=1, Baud=9600)

    # Change to initiative mode
    sensor.change_acquire_mode(sensor.INITIATIVE)

    # Enable temperature compensation
    sensor.set_temp_compensation(sensor.ON)

    while True:
        # Acquire data from the sensor
        sensor.acquire_data()

        # Read gas concentration
        concentration = sensor.read_gas_concentration()
        gastype = sensor.read_gas_type()
        temp = sensor.read_temp()

        # Print the gas concentration, type, and temperature
        print(f"Gas Type: {gastype}")
        print(f"Gas Concentration: {concentration} {sensor.gasunits}")
        print(f"Temperature: {temp:.2f} °C")

        # Publish the data to the MQTT broker
        payload = f"Gas Type: {gastype}, Concentration: {concentration} {sensor.gasunits}, Temperature: {temp:.2f} °C"
        result = client.publish(topic, payload)

        # Check if the message was published successfully
        status = result[0]
        if status == 0:
            print(f"Sent `{payload}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")

        # Wait for a second before reading the data again
        time.sleep(1)

