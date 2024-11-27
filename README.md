# LEM_LASSITOS
This repository contains the open source code, and plans for the LEM sounder develped as part of the LASSITOS project.


I should add some informations here

## Construction of LEM


## Operating the LEM 
### LEM commands


### Data link LEM and UAV
The LEM Radio Link (LoRa 900MHz) is used to communicate to the LEM during measurements.  The user can send control commands (START, STOP, SETFREQ,... ) connecting via serial bluetooth or USB serial to the LEM_Base.  The LEM commands are transmitted over a own NMEA message ($LEMMS,...). This allows to separate them from the RTK message stream that is transmitted over the same channel.
The RTK messages can be passed to the LEM_Base over Serial UART (3.3V) or USB. Only one GPS base station can be used if the RTK stream is split on the laptop between then LEM and UAV.  See schematics below for passing RTK correction over UART (option1) or splitting on Laptop and passing over USB (Option2).

![Schematic data link wiht one RTK base station](images/DataLink1.svg)
![Schematic data link wiht 2 RTK base stations](images/DataLink2.svg)

## Data download

###	ESP32 data: Laser, IMU, general

###	LEM raw data

##	Data Postprocessing
