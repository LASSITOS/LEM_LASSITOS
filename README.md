# LEM_LASSITOS
This repository contains the open source code, and plans for the LEM sounder develped as part of the LASSITOS project.


I should add some informations here

## Construction of LEM


## Operating the LEM 
### LEM commands


### Data link LEM and UAV
The LEM Radio Link (LoRa 900MHz) is used to communicate to the LEM during measurements.  The user can send control commands (START, STOP, SETFREQ,... ) connecting via serial bluetooth to the LEM_Base.  Additionally RTK correction messages are send from a GPS base station to the LEM. The LEM commands are transmitted over a own NMEA message ($LEMMS,...). This allows to separate them from the RTK message stream on the LEM.
The RTK messages can be passed to the LEM_Base over Serial UART (3.3V)  or USB. Only one GPS base station can be used if the RTK stream is split on the laptop.  See schematics below for passing RTK correction over UART (option1) or splitting on Laptop and passing over USB (Option2).

![image](https://github.com/user-attachments/assets/1cff1870-c0f9-4925-81b3-5b921f9edefc)
![image](https://github.com/user-attachments/assets/3f3f5b83-de97-4ef5-9f18-0094e748522a)


## Data download

###	ESP32 data: Laser, IMU, general

###	LEM raw data

##	Data Postprocessing
