/*
  Code for controlling LEM esp32 microprocessor.
  The sea ice thickness EM sounder LEM was developed as part of the LASSITOS project
  By: A.Capelli
  Date: April 7th, 2023
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Based on Examples from multiple libraries and hardware.
  
*/

#include "FS.h"
#include "SD.h"
#include "SPI.h"

//HardwareSerial
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

HardwareSerial RS232(2);
HardwareSerial IMX5(1);
#define baudrateSerial 230400 //115200 //921600  //230400  // 115200 57600// baudrate USB serial port


#define Version "LEM sounder ESP32 v1.6 (ESPNOW)"
String LEM_version= "LEM v2.0";
String LEM_ID="LEM02";


unsigned long lastTime = 0;  //Simple local timer
unsigned long lastTime1 = 0;  //Simple local timer
unsigned long lastTime2 = 0;  //Simple local timer
unsigned long lastTime3 = 0;  //Simple local timer
unsigned long lastTime_STROBE =0;
unsigned long lastTime_flushSD = 0;
unsigned long logTime_laser;
unsigned long logTime_IMX5;
unsigned long lastTime_VBat=0; 
unsigned long lastTime_Temp=0; 
unsigned long lastTime_ASCB=0; 
unsigned long lastTime_CAL =0;
unsigned long startTime=0;     //keep track of ESP32 measuing time


TaskHandle_t TaskStart;
TaskHandle_t TaskEnd;
TaskHandle_t TaskSwitchMulti;
TaskHandle_t TaskCalMulti;
bool CalMulti_on = false;
TaskHandle_t TaskCal;

// settings PIN SPI 
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#define SCK 18
#define MISO 19
#define MOSI 23
#define SPI_rate    400000  // DAC up to 80 MHz 
#define SPI_rate_SD 20000000  
#define CS_SD 5   // cip select  SD card
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

SPIClass spi = SPIClass(VSPI);

uint16_t addr;
char addrStr[5];
uint16_t dat;
char datStr[19];
uint32_t msg;
uint16_t out;





//Other PINs
//-=-=-=-=-=-=-=-=-=-=-=-=-=
// #define CD_pin 36  // chip detect pin is shorted to GND if a card is inserted. (Otherwise pulled up by 10kOhm)
#define PIN_VBat 34    

float VBat=0;
unsigned long VBat_intervall = 10000;



// settings SD card
//------------------

#define sdWriteSize 16384     // Write data to the SD card in blocks of 512 bytes
#define WriteSize_Laser 23  // Write data to buffer in blocks (should be shorter than expected message)
#define WriteSize_IMX5 30  // Write data to buffer in blocks (should be shorter than expected message)
#define myBufferSize 49152
#define tempBufferSize 16384      // must be bigger than sdWriteSize,WriteSize_Laser and WriteSize_IMX5
#define LaserBufferSize 2048 
#define flushSD_intervall 120000  // Flush SD every ** milliseconds to avoid losing data if ESP32 crashes
#define tempBufferIMX5_length 256


uint8_t myBuffer[myBufferSize];       // Create our own buffer to hold the data while we write it to SD card
uint8_t tempBuffer[tempBufferSize];   // Create temporay buffer
uint8_t myBuffer_laser[LaserBufferSize];  // Create buffer for laser data
uint8_t tempBufferIMX5[tempBufferIMX5_length]; // Create buffer for IMX5 data

int tempBufferIMX5_i=0;
int BufferTail = 0;
int BufferHead = 0;
int bitesToWrite;


char dataFileName[24];  //Max file name length is 23 characters)
File dataFile;          //File where data is written to


bool measuring = false;
bool StartFlag = false;
bool StopFlag = false;
bool StopMultiFlag = false;
bool  flagcalMulti = false;
bool closefileFlag = false;

unsigned long lastPrint;  // Record when the last Serial print took place




// settings ESP now
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Support.h"

unsigned long  ESP_inbytes=0;
int  ESP_inbytes_buff=0;
unsigned long ESP_inbyteslast=0;
unsigned long  ESP_inbyteslasttime=0;
unsigned long lastTime_MSGtoBase=0;
int intervall_MSGtoBase=10000;  // Intervall I'm herer messages to base (ms) 
char txString[512];  // String containing messages to be send to BLE terminal
char txString2[128];
char subString[64];
bool BLE_message = false;
#define CharBufferSize 512 
char myBuffer_Char[CharBufferSize];  // Create buffer for laser data



// settings UART microcontroller MSP430 
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int MSP430_Rx = 35;  // Software RX pin,
int MSP430_Tx = 4;  // Software TX pin,
#define baudrateMSP430  19200   //must be same as radio
SoftwareSerial UART_MSP430(MSP430_Rx,MSP430_Tx);

bool ignore_MSP430_status=0
;   // Deactivating status check for MSP430!!!!!!!!!  Danger!!!!!! Should be 0 if not in debug mode.


// settings UART radio
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// int Radio_Rx = 39;  // Software RX pin,
// int Radio_Tx = 26;  // Software TX pin,
// #define baudrateRadio  19200 //115200   //must be same as MSP430
// SoftwareSerial Radio(Radio_Rx,Radio_Tx);

// settings altimeter LSD70A
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int PIN_Rx = 14;  // 16 = Hardware RX pin,
int PIN_Tx = 32;  // 17 = Hardware TX pin,

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define baudrateRS232  230400  //115200
#define Laser_BufferSize 2048  // Allocate 1024 Bytes of RAM for UART serial storage
#define Laser_log_intervall 2000

#include "LaserParser.h"
/* A parser is declared with 1 handlers at most */
LaserParser<3> parserLaser;

struct {   // Create a structure containig Laser data 
  bool valid=false;
  unsigned long timestp=0;
  float dT=0;

  float Temp=0;
  float Dist=0;
  float SigQual=0;
  int LaserError=0;
} Laser_data;

// settings INS IMX5
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
int  IMX5_Rx = 33;//13;  //  Hardware RX pin, to PIN10 on IMX5
int  IMX5_Tx =13;  //  Hardware TX pin, to PIN8 on IMX5
int  IMX5_strobe = 99; //  GPIO for STROBE input (first), to  PIN2 on IMX5 !!!!!!!!Reserved for radio!!!!!!!!  !!!!!!!!NOT Assigned. Need to find a free pin for using it!!!!!!!!
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define baudrateIMX5 234375 //57600 //230400  //115200
#define IMX5_BufferSize 8192  // Allocate 1024 Bytes of RAM for UART serial storage
#define IMX5_log_intervall 20000
#define STROBE_intervall 5000
// char asciiMessage[] = "$ASCB,512,,,200,,,,30000,,30000,,,";  // // Get PINS1 @ 100ms, 30s  on the connected serial port, leave all other broadcasts the same, and save persistent messages.
char asciiMessage[] = "$ASCB,516,,,5,,10,,,,,,,";     // new IMX5 has different IMU data rate  (16 ms). I dont know why!! But it is ok.
//char asciiMessage[] = "$ASCE,516,4,5,6,100,0,0"; // change to newer ASCE messages. Not working.
char asciiMessageformatted[128];
int IMXrate=100;
int IMUdataRate=16;
unsigned long ASCB_intervall = 3000;
char asciiMessage2[] = "$ASCB,0,,,5,,10,,10,,,,,";     // new IMX5 has different IMU data rate  (16 ms). I dont know why!! But it is ok.
char asciiMessage3[] = "$ASCB,0,,,,,60,,60,,,,,";     // new IMX5 has different IMU data rate  (16 ms). I dont know why!! But it is ok.


long Year =1999;
long Month =88;
long Day =77;
long Hour =1;
long Minute = 2;
long Second = 3;



// --------------------------------
// settings NMEA parser
//---------------------------------
#include "LEM_NMEAParser.h"  // modified version of NMEAParser allowing NMEA messages longer than 82 bytes
NMEAParser<3> LEM_parser;

NMEAParser<5> IMX_parser;

long N_PINS1=0;
bool validTime=false;

struct {             // Create a structure containig data PGPSP
  bool valid=false;
  unsigned long timestp=0;
  unsigned long timevalid=3000;
  int TOW=0;
  int GPSWeek=0;
  int status=0;
  float lat=0;
  float lon=0;
  float el_HAE=0;
  float MSL=0;
  float pDOP=0;
  float hAcc=0;
  float vAcc=0;
  float Vel_X=0;
  float Vel_Y=0;
  float Vel_Z=0;
  float sAcc=0;
  float cnoMean=0;
  float towOffset=0;
  float leapS=0;

  int fix ;
  int NSat ;
  int BasePos; 
  int flags ;
  bool fixOK ; 
} PGPSP_data;

#define GPS_STATUS_NUM_SATS_USED_MASK 	0x000000FF
#define GPS_STATUS_FIX_MASK 	0x00001F00
#define GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MASK 	0x03000000
#define GPS_STATUS_FLAGS_MASK 	0xFFFFE000
#define GPS_STATUS_FLAGS_FIX_OK 	0x00010000

struct {             // Create a structure containig data of GPGGA messages
  bool valid=false;
  unsigned long timestp=0;
  
  int Hour=0;
  int Minute=0;
  int Second=0;
  float lat=0;
  float lon=0;
  int fix=0;
  int NSat ;
  float hDop=0;
  float MSL=0;
 
} GGA_data;


struct {             // Create a structure containig data PINS1
  bool valid=false;
  unsigned long timestp=0;
  unsigned long timevalid=1000;
  float TOW=0;
  int GPSWeek=0;
  int INSstatus=0;
  int Hardwarestatus=0;
  float roll=0;
  float pitch=0;
  float yaw=0;
  float Vel_X=0;
  float Vel_Y=0;
  float Vel_Z=0;
  float lat=0;
  float lon=0;
  float el_HAE=0;

  int fix ;
  int Base; 
} PINS1_data;

#define INS_STATUS_GPS_NAV_FIX_MASK 	0x03000000
#define INS_STATUS_RTK_ERR_BASE_MASK 	0x30000000


// --------------------------------
// Settings I2C, Amp. Cswitches, Temp.
//---------------------------------
#include <Wire.h>
#include "Adafruit_MCP9808.h"

Adafruit_MCP9808 sensor = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens1 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens2 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens3 = Adafruit_MCP9808();
Adafruit_MCP9808 tempsens4 = Adafruit_MCP9808();


// I2C addresses
#define  MA12070P_address 0x20
#define  CSwitchTx_address 0x24
#define  CSwitchCal_address 0x23 //0x26
#define  Temp1_address 0x18  // 000 ; Tx, Tail
#define  Temp2_address 0x19  // 001 ;  Rx, Tip
#define  Temp3_address 0x1A  // 010 ;  Battery
#define  Temp4_address 0x1B  // 011


#define N_TempSens 3
int TempSens_addr[] ={Temp1_address,Temp2_address,Temp2_address};
unsigned long Temp_intervall = 10000;  // logging rate for temperature in ms
int CheckTSensor[] ={true,true,false};  // Check or not sensor


// settings DAC
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
#include "SC18IS602B.h"
SC18IS602B spiBridge;   //SC18IS602B spiBridge(-1,a1,a2,a3) for changing address??   
/// New PCB has different chip. SC18IS606 !!!!!!!!    
// Everything should work but can't use pin3. Modifications should be easy. Change 3 to 2 in pin number check + increase buffer from 200 to 1024 bits.

#define triggerDAC 15   
#define PIN_EN 27    // must be =1 at startup
#define PIN_MUTE 12  // must be =0 at startup





//-=-=-=-=-=-=-=-=-=-=-=-
// Function generation settings
//-=-=-=-=-=-=-=-=-=-=-=-
uint64_t clock_divider=1;
uint64_t AWG_clock_freq = 7372800; // 125000000;   #Frequency of clock used by DAC AD9102. It is the same clock that is used for the ADCs.
uint16_t freqAdd;
uint16_t freqDat;
float gain=0;
uint16_t gainDAT = 0x1000;   // Should not be higher than maxGAIN 

#define MAXGAIN 0x1800

//resonant frequencies:
#define F1 654 //684
#define F2 1111 //1079
#define F3 1835 //1908
#define F4 5655 //5754
#define F5 6524 //6848
#define F6 7625   //8207

#define F46 4557 //4725
#define F456 3744 //3900


uint64_t freqs[] ={F1,F2,F3,F456,F46,F4,F5,F6};
uint16_t Nfreq=8;  //Number of frequencies to use.  Not larger than length of 'freqs'.


int ifreq =2;
uint64_t freq=freqs[ifreq];
int Nmulti =0;
int i_multi =0;


int Nmulti_reg=3;
uint64_t Multifreqs[]={1,3,6,0,0,0,0,0,0,0}; //Must be same or larger than Nfreq
int MultiPeriod=300;
int MultiTime=0;


// settings CSwitch
//-=-=-=-=-=-=-=-
// states of CSwitch for each frequency
#define stateF1 0x80
#define stateF2 0x40
#define stateF3 0x02
#define stateF4 0x01
#define stateF5 0x10
#define stateF6 0x08

uint8_t CSw_states[] ={stateF1,stateF2,stateF3,stateF4+stateF5+stateF6,stateF4+stateF6,stateF4,stateF5,stateF6};
uint8_t CSw_state=CSw_states[ifreq];
uint8_t CSwitch =0 ;  // Controlling witch switch is open. 0 all a close.  1 for first, 2 for second,   4  for third. Sum for combinations. 
uint8_t  CSwitch_code =0xFF;


// settings CalCoil
//-=-=-=-=-=-=-=-
int CAL_states[]={0,1,2,4,8,12,14};
int CAL_state=0;
int N_cal=7;
bool calibrating=0;
bool cal_on=0;
int  CAL_intervall_on=3000;
int  CAL_intervall_off=500;
int  N_CalMulti_on=5;
int  N_CalMulti_off=2;
int  CALMulti_intervall_on=Nmulti_reg*MultiPeriod*N_CalMulti_on;
int  CALMulti_intervall_off=Nmulti_reg*MultiPeriod*N_CalMulti_off;


// Non operational variables for passing LEM values to datafile 
//-=-=-=-=-=-=-=-=-=-=-=- 
int N_CalCoil=320; 
int N_RxCoil=1014; 
int N_BuckCoil=32;
float Rs[]={80.19,147.6,402.2, 620.7,278.4,127.6};   //Old values(77.5,143.5,400, 619,277,125,)
float d_Rx=1.92;
float d_Bx=0.56;
float d_Cx=1.695;
float L_CalCoil=0.01146 ;  
float A_CalCoil= 0.00502 ; 
float A_RxCoil= 0.00502;
float A_BuckCoil= 0.00502;
float distCenter=0.244;
float offsetLaser=0.073;




//-=-=-=-=-=-=-=-=-=-=-=-
// Microcontroller communication settings
//-=-=-=-=-=-=-=-=-=-=-=-
char CDM_start[]= "S";
char CDM_stop[]= "T";
char CDM_status[]="Q";
char CDM_reset[]= "R";				 

uint GAIN_ADC=0;

// status commands
String str_RUNNING ="RUNNING";
String str_SD_FULL ="SD FULL";
String str_SD_ERR ="SD ERROR";
String str_CALIBRATING ="CALIBRATING";
String str_STARTING ="STARTING";
String str_READY ="READY";
String str_CAL_DONE ="CALIBRATION_DONE\r\n";
String str_GAIN ="GAIN";


char status_out[64];

// status flags Micro
bool MSP430_CMD_OK =0;
bool MSP430_CALIBRATING =0;
bool MSP430_STARTING =0;
bool MSP430_SD_ERR =0;
bool MSP430_measuring =0;
bool MSP430_SD_FULL = 0;

// other flags
bool ADCstarted=0;  //flag checking if ADC responded to start command
bool SD_mounted =0;
bool SD_filecreated =0;






// /////////////////////////////////////////////
// --------------------------------
// other functions
//---------------------------------
// /////////////////////////////////////////////
float readVBat(){
	int VBat_int= analogRead(PIN_VBat);
	VBat =float(VBat_int)/4095*3.2*10.09;
	// Serial.printf("Voltage battery GPIO value: %d \n", VBat_int);
  // Serial.printf("Voltage battery: %05.2f V\n", VBat);
	return VBat;
}

void logVBat(){
    readVBat();
	  sprintf(subString,"VBat %05.2f\n",VBat );
	  int msglen=11;
    for (int i = 0; i < msglen; i++){
      tempBuffer[i] = uint8_t(subString[i]);
    } 
    writeToBuffer(tempBuffer, msglen);
}


void logTemp(Adafruit_MCP9808 sensor, int  sensorNumber){
  float c = sensor.readTempC();
  // Serial.printf("Temp%d %07.4f C\n",sensorNumber,c); 
  sprintf(subString,"Temp%d %07.4f\n",sensorNumber,c );
  int msglen=strlen(subString);
  for (int i = 0; i < msglen; i++){
      tempBuffer[i] = uint8_t(subString[i]);
    } 
  writeToBuffer(tempBuffer, msglen);
}

void readTemp(Adafruit_MCP9808 sensor, int  sensorNumber){
  float c = sensor.readTempC();
  sprintf(txString,"Temperature: %.4f* C\n",c);
  Serial.println(txString);
  
}


void Log_timestamp(char *prefix){
  // long time = millis()-startTime;
  unsigned long time = myTime();
  char stmpMSG [64];
  sprintf(stmpMSG,"%s @%d\n",prefix,time );
  int msglen=strlen(stmpMSG);
  for (int i = 0; i < msglen; i++){
      tempBuffer[i] = uint8_t(stmpMSG[i]);
    } 
  writeToBuffer(tempBuffer, msglen);
}





void writeToBuffer(uint8_t *tempBuf, int &bitesToWrite) {
  // move BufferHead if circular buffer is full
  int nDump;
  if (BufferTail >= BufferHead) {
    nDump = bitesToWrite - (myBufferSize + BufferHead - BufferTail);
  } else {
    nDump = bitesToWrite - (BufferHead - BufferTail);
  }
  if (nDump >= 0) {
    Serial.print("Data buffer is getting full. Dumping N data:");
    Serial.println(nDump + 1);
    BufferHead = (BufferHead + nDump + 1) % myBufferSize;
  }

  // write data to buffer
  if ((BufferTail + bitesToWrite) < myBufferSize) {
    for (int i = 0; i < bitesToWrite; i++) {
      myBuffer[BufferTail] = tempBuf[i];
      BufferTail++;
    }
  } else {
    int a = myBufferSize - BufferTail;
    for (int i = 0; i < a; i++) {
      myBuffer[BufferTail] = tempBuf[i];
      BufferTail++;
    }
    BufferTail = 0;
    for (int i = 0; i < (bitesToWrite - a); i++) {
      myBuffer[BufferTail] = tempBuf[i + a];
      BufferTail++;
    }
  }
}






void startMeasuring() {
  // BLE_message = true;
  
  // Configure DAC
  //-----------
  stop_trigger();
  delay(10);
  configureResFreq(ifreq);
  setCswitchCal(0);
  delay(10);
  digitalWrite(PIN_MUTE , HIGH);  // unmute
  
  
  makeFiles(SD);
  delay(200);

  lastPrint = millis();      // Initialize lastPrint
  lastTime_MSGtoBase=millis();
  logTime_laser = millis();  // logTime_laser
  logTime_IMX5 = logTime_laser;
  BufferTail = 0;
  BufferHead = 0;
  StopMultiFlag = true;
  closefileFlag = false;

  // setting up GPS for automatic messages
  setupINS();
  delay(50);
  
  // Write start of data to datafile
  dataFile.println(F("###Data###"));  
  
  // Start logging data
  measuring = true;
  
  
  //start Laser measuremens
  RS232.write("DT");
  RS232.write(0x0D);
  delay(200);
	
	
  while (RS232.read() >= 0){
    ;  // flush the receive buffer.
  }

  // while (IMX5.read() >= 0)
    // ;  // flush the receive buffer.
  while (IMX5.available() > 0){
    char k = IMX5.read();
  }
  
  char infoMsg[] = "$INFO";
  FormatAsciiMessage(infoMsg, sizeof(infoMsg), asciiMessageformatted);
  IMX5.write(asciiMessageformatted);  //send instruction for sending ASCII messages



  xTaskCreatePinnedToCore(
                    TaskStartCode,  /* Task function. */
                    "TaskStart",    /* name of task. */
                    10000,       	/* Stack size of task */
                    NULL,        	/* parameter of the task */
                    1,           	/* priority of the task */
                    &TaskStart,     /* Task handle to keep track of created task */
                    0); 			/* Core where the task should run */ 

}


void TaskStartCode(void * pvParameters) {
  
  while(true){
  // Start MSP430
  //-----------
  startMSP430();
  
  // Start DAC
  //-----------
  if (Nmulti==0){
	  run();
  } else {
    StopMultiFlag = false;

  //       xTaskCreatePinnedToCore(TaskCode_SwitchMulti,"TaskSwitchMulti",10000, NULL, 1,  &TaskSwitchMulti,0); 
  }

  if (SD_filecreated){
    strcat(txString, "\r\nFile succesfully created on ESP32 SD card");
    Serial.println(txString);

  } else{
    strcat(txString, "\r\n ESP32 SD not working. Abort.");
    Serial.println(txString);
	  interrupt_Measuring();
	  vTaskDelete(TaskStart);
  }
  
  strcat(txString, "\r\nStarted measurement succesfully!");
  Serial.println(txString);
  BLE_message = true; //Send_tx_String(txString);
  
  vTaskDelete(TaskStart);
  
  }
}



// Interrupt data measuring process when there is an error at startup. No successful measurement
void interrupt_Measuring() {
  // if (Nmulti){
  //   StopMultiFlag=true;
  // }else{
  measuring = false;  // Set flag to false
  BLE_message = true;
  Serial.println("unsuccesful  mesurment!");
  
  //Stop DAC
  stop_trigger();
  digitalWrite(PIN_MUTE , LOW);  // mute
  
  //Stop Micro
  delay(50);
  stopMicro();

  delay(25);
  if (SD_filecreated){
	dataFile.println(F("# unsuccesful measurement stopped"));
  }

  closefileFlag = true;

  BLE_message = true; //Send_tx_String(txString);
  // }
}


// Stop data measuring process
void stopTask(void * pvParameters) {
  while(true){
  Serial.println("Turning dataMeasuring OFF!");
  
  // if (Nmulti!=0){
  //     vTaskDelete(TaskSwitchMulti);
  //     Serial.println("Turned of SwitchMulti");
  // }

  //Stop DAC
  // Serial.println("Stop DAC");
  StopMultiFlag = true;
  stop_trigger();
  digitalWrite(PIN_MUTE , LOW);  // mute
  // Serial.println("DAC stopped");
  
  //Stop Micro
  delay(1000);
  stopMicro();
  
  measuring = false;  // Set flag to false
  
  delay(100);
  emptyBuffer();
  delay(100);
  closefileFlag = true;

  // Serial.println("Datafile closed.");                     
  strcat(txString, "Measurement stopped successfully");
  BLE_message = true; //Send_tx_String(txString);
  Serial.println("Measurement stopped successfully");

  delay(100);
  vTaskDelete(TaskEnd);
  }
}


void closeFile(fs::FS &fs){
  // output file string
  char filesstring[50];
  strcpy(txString, "Closed file:");
  sprintf(filesstring, " File: %s", dataFileName);
  strcat(txString, filesstring);
  Serial.print("Closing file:");
  Serial.println(filesstring);
  delay(25);
  Write_stop();
  dataFile.close();  // Close the data file
  Serial.println("Datafile closed."); 
  BLE_message = true;  
}

void emptyBuffer(){
  //  Empty data buffer
  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  Serial.print("Total bytes in buffer:");
  Serial.print((myBufferSize + BufferTail - BufferHead) % myBufferSize);
  Serial.print(", buffer tail:");
  Serial.print(BufferTail);
  Serial.print(", buffer head:");
  Serial.println(BufferHead);
  int bytesTot = 0;
  int remainingBytes = (myBufferSize + BufferTail - BufferHead) % myBufferSize;  // Check if there are any bytes remaining in the file buffer

  while (remainingBytes > 0)  // While there is still data in the file buffer
  {
    int bytesToWrite = remainingBytes;  // Write the remaining bytes to SD card sdWriteSize bytes at a time
    if (bytesToWrite > sdWriteSize) {
      bytesToWrite = sdWriteSize;
    }
    // write data to buffer
    if ((BufferHead + bytesToWrite) < myBufferSize) {  // case head + bytesToWrite smaller than buffer size
      for (int i = 0; i < bytesToWrite; i++) {
        tempBuffer[i] = myBuffer[BufferHead];
        BufferHead++;
        bytesTot++;
      }
    } else {
      int a = myBufferSize - BufferHead;
      for (int i = 0; i < a; i++) {
        myBuffer[i] = tempBuffer[BufferHead];
        BufferHead++;
        bytesTot++;
      }
      BufferHead = 0;
      for (int i = a; i < (bytesToWrite); i++) {
        tempBuffer[i] = myBuffer[BufferHead];
        BufferHead++;
        bytesTot++;
      }
    }
    dataFile.write(tempBuffer, bytesToWrite);
    remainingBytes -= bytesToWrite;
  }
  Serial.print("Buffer emptied! Data written:");
  Serial.println(bytesTot);
  
}


// Stop data measuring process
void stop_Measuring(fs::FS &fs) {
  measuring = false;  // Set flag to false
  // BLE_message = true;
  Serial.println("Turning dataMeasuring OFF!");
  
  //Stop DAC
  stop_trigger();
  digitalWrite(PIN_MUTE , LOW);  // mute
  
  //Stop Micro
  delay(1000);
  stopMicro();
  


  //Read data INS+Laser
  // =-=-=-=-=-=-=-=-=-
   //  Laser data
  int times=0;
  while (RS232.available() ) {
    bitesToWrite = RS232.available();
    if (tempBufferSize < bitesToWrite) {
      bitesToWrite = tempBufferSize;
    }
    RS232.readBytes(tempBuffer, bitesToWrite);
    writeToBuffer(tempBuffer, bitesToWrite);
    times++;
    delay(1);
  }
  Serial.print("Wrote Laser data to buffer N times. N=");
  Serial.println(times);
   
  //  INS data
  times=0;
  while (IMX5.available() ) {
    bitesToWrite = IMX5.available();
    if (tempBufferSize < bitesToWrite) {
      bitesToWrite = tempBufferSize;
    }

    IMX5.readBytes(tempBuffer, bitesToWrite);
    writeToBuffer(tempBuffer, bitesToWrite);
    times++;
    delay(1);
  }
  Serial.print("Wrote INS data to buffer N times. N=");
  Serial.println(times);


  emptyBuffer();

  // output file string
  char filesstring[50];
  strcpy(txString, "Closed file:");
  sprintf(filesstring, " File: %s", dataFileName);
  strcat(txString, filesstring);
  Serial.print("Closing file:");
  Serial.println(filesstring);
  delay(25);
  Write_stop();
  dataFile.close();  // Close the data file
  // Serial.println("Datafile closed.");                     
  strcat(txString, "Measurement stopped successfully");
  BLE_message = true; //Send_tx_String(txString);
}



	



// /////////////////////////////////////////////
// -%--------------------------------------------
// Setup and loop
// %%------------------------------------------------------------------------------------------------------------------------------
void setup() {

  Serial.begin(baudrateSerial);

	//Mute and Enable PINs Amplifier MA12070 
	pinMode(PIN_EN , OUTPUT); 
	digitalWrite(PIN_EN , HIGH);   // must be =1 at startup
	pinMode(PIN_MUTE , OUTPUT); 
	digitalWrite(PIN_MUTE , LOW);  // must be =0 at startup
	

	//SPI over I2C setup
	spiBridge.begin();  //Possible double wire begin??
	spiBridge.configureSPI(false, SC18IS601B_SPIMODE_0, SC18IS601B_SPICLK_461_kHz);

	// Initialize the I2C transmitter.	
	Wire.begin();	
	Serial.println("Wire set up");
  
	// Set the initial state of the pins on the PCF8574 devices
	Wire.beginTransmission(CSwitchTx_address); // device 1
    Wire.write(0xff); // all ports off
    uint8_t error = Wire.endTransmission();
    Serial.printf("endTransmission on CSwitch Tx: %u\n", error);
    Wire.begin();
    Wire.beginTransmission(CSwitchCal_address); // device 2
    Wire.write(0xff); // all ports off
    error = Wire.endTransmission();
    Serial.printf("endTransmission on CSwitch CalibCoil: %u\n", error);
	

	if (!tempsens1.begin(Temp1_address)) {
		Serial.println("Couldn't Temperature sensor 1! Check your connections and verify the address is correct.");
	  }	
	if (!tempsens2.begin(Temp2_address)) {
		Serial.println("Couldn't Temperature sensor 2! Check your connections and verify the address is correct.");
	  }
	if (!tempsens3.begin(Temp3_address)) {
		Serial.println("Couldn't Temperature sensor 2! Check your connections and verify the address is correct.");
	  }
	  if (!tempsens4.begin(Temp4_address)) {
		Serial.println("Couldn't Temperature sensor 2! Check your connections and verify the address is correct.");
	  }
	  
  
	//enable MA12070 to be allow to acces registers
	digitalWrite(PIN_EN , LOW);
	write_I2C (MA12070P_address , 0x1D, 0x01 ); // use power mode BBB
	write_I2C (MA12070P_address , 0x25, 0x00 ); // low gain better SignaltoNoise
	

  // Setup RS232 connection to Laser
  //--------------------
  Serial.println("Connecting to LASER");
  RS232.begin(baudrateRS232, SERIAL_8N1, PIN_Rx, PIN_Tx);  // Use this for HardwareSerial
  RS232.setRxBufferSize(Laser_BufferSize);
  xTaskCreate(serialEventLaserTask, "serialEventLaserTask", 1024*6, NULL, 5, NULL);
  Serial.println("Connected to LASER");
  //    delay(500);
  //  	RS232.write(0x1B);  // stop sending data
  //    delay(500);
  //  	RS232.write("SD 0 3");  // set data format
  //  	RS232.write(0x0D);
  //    delay(100);


  // Setup IMX5 connection
  //--------------------
  Serial.println("Connecting to IMX5");
  IMX5.begin(baudrateIMX5, SERIAL_8N1, IMX5_Rx, IMX5_Tx);  // Use this for HardwareSerial
  IMX5.setRxBufferSize(IMX5_BufferSize);
  Serial.println("Started serial to IMX5");
  xTaskCreate(serialEventIMX5Task, "serialEventIMX5Task", 1024*6, NULL, 5, NULL);
  // pinMode(IMX5_strobe, OUTPUT);
  // digitalWrite(IMX5_strobe, LOW);
  // pinMode(IMX5_Rx2, INPUT);


  // Setup UART_MSP430 connection
  //--------------------
  UART_MSP430.begin(baudrateMSP430 );  
  Serial.println("Started serial to IMX5");

  // Setup UART_Radio connection
  //--------------------
  // Radio.begin(baudrateRadio )  
  // Serial.println("Started serial to IMX5");


  // Setup ESPNOW connection //
  InitESPNow();
    while(!ScanForSlave()){
    Serial.println("REP ESP not found. trying again in 5 seconds.");
    delay(5000);
  }
  Serial.println("REP ESP found.");
  xLemToBaseQueue = xQueueCreate( QUEUE_LENGTH, sizeof(ESP_Msg_t) );   // create a queue of 10 messages
  xTaskCreate(espNOWsendTask, "espNOWsendTask", 1024*6, NULL, 4, NULL);
  //xTaskCreate(serialEventRadioTask, "serialEventRadioTask", 1024*6, NULL, 5, NULL);
  


  // Setup SPI connection
  //--------------------
  spi.begin(SCK, MISO, MOSI, CS_SD);  
  Serial.println("Started SPI");
  pinMode(CS_SD, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
  digitalWrite(CS_SD, HIGH);
  // pinMode(CS_DAC, OUTPUT); //set up slave select pins as outputs as the Arduino API doesn't handle automatically pulling SS low
  // digitalWrite(CS_DAC, HIGH);


  pinMode(triggerDAC , OUTPUT); 
  digitalWrite(triggerDAC, HIGH);
  
  
  // Setup SD connection
  //--------------------
  if (!SD.begin(CS_SD, spi, SPI_rate_SD)) {
    Serial.println("Card Mount Failed");
    // while (1)
      ;
  } else{
    Serial.println("SD mounted");
	SD_mounted=1;
  }

  uint8_t cardType = SD.cardType();
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else if (cardType == CARD_NONE) {
    Serial.println("No SD card attached. ");
  } else {
    Serial.println("UNKNOWN");
  }
  //    listDir(SD, "/", 0);
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));


  // Setup parser NMEA messages INS //
  IMX_parser.setErrorHandler(errorHandler);
  IMX_parser.addHandler("GNZDA", ZDA_Handler);
  IMX_parser.addHandler("PINS1", PINS1_Handler);
  IMX_parser.addHandler("GPGGA",   GGA_Handler);
  IMX_parser.addHandler("PGPSP",   PGPSP_Handler);
  
  IMX_parser.setDefaultHandler(unknownCommand);
  validTime=false;

  // Setup parser NMEA messages BaseStation//
  LEM_parser.setErrorHandler(errorHandler2);
  LEM_parser.addHandler("LEMMS", LEM_Handler);
  LEM_parser.setDefaultHandler(unknownCommand);

  // Setup parser Laser messages //
  parserLaser.setErrorHandler(errorHandlerLaser);
  parserLaser.setLaserHandler(Laser_Handler);


  BLE_message = true;
  strcpy(txString, "Setup completeded. Waiting for command 'START' over Serial of Radio for starting measuring data!");
  Send_tx_String(txString);

}


void loop() {

  //################################# Data logging #############################################

  if (measuring) {  // if measuring is active check for INS and Laser data and save them to SD card

    // //  INS data
    // // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // if (IMX5.available() >= WriteSize_IMX5) {
      // bitesToWrite = IMX5.available();
      // if (tempBufferSize < bitesToWrite) {
        // bitesToWrite = tempBufferSize;
      // }

      // IMX5.readBytes(tempBuffer, bitesToWrite);
      // writeToBuffer(tempBuffer, bitesToWrite);
      // logTime_IMX5 = millis();
      // for(int i=0; i<bitesToWrite; i++ ){
          // IMX_parser << tempBuffer[i];
          // // Serial.print(tempBuffer[i]);
        // }

      // // check if an entire message was written
      // if (tempBuffer[bitesToWrite-5]!='*' and IMX5.available()>0 ){
        // bitesToWrite = IMX5.available();
        // if (tempBufferSize < bitesToWrite) {
          // bitesToWrite = tempBufferSize;
        // }
        // IMX5.readBytes(tempBuffer, bitesToWrite);
        // writeToBuffer(tempBuffer, bitesToWrite);
        // for(int i=0; i<bitesToWrite; i++ ){
          // IMX_parser << tempBuffer[i];
          // // Serial.print(tempBuffer[i]);
        // }
      // }
      // delay(1);
    // }



    // //  Laser data
    // // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // if (RS232.available() >= WriteSize_Laser) {
    //   bitesToWrite = RS232.available();
    //   if (tempBufferSize < bitesToWrite) {
    //     bitesToWrite = tempBufferSize;
    //   }
    //   RS232.readBytes(tempBuffer, bitesToWrite);
    //   writeToBuffer(tempBuffer, bitesToWrite);
      
      
    //   for(int i=0; i<bitesToWrite; i++ ){
    //     parserLaser << tempBuffer[i];
    //   }
      
    //   logTime_laser = millis();
    //   delay(1);
    // }



    //  Switch multifreq
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (Nmulti>0 & (millis()-MultiTime >MultiPeriod ) & StopMultiFlag==false){
      // long tik =  millis();
      stop_trigger();
      digitalWrite(PIN_MUTE , LOW);  // mute
      
      // Notify TaskCalMulti if frequency cycle ended
      if (i_multi==0 &&  CalMulti_on){
        xTaskNotifyGive( TaskCalMulti );  
      }
      
      delay(3);
      int i=Multifreqs[ i_multi ];
      freq=freqs[i];
      configureSineWave();
      uint8_t error =setCswitchTx2(CSw_states[i]);
      parsePrint_I2C_error(error);
      delay(3);
      digitalWrite(PIN_MUTE , HIGH);  // unmute
      run2();
      trigger();
      
      char FrqStr[32];
      sprintf(FrqStr,"SWF%d ",i_multi );
      Log_timestamp(FrqStr);

      // int tok=millis();
      // Serial.printf("New frequency is: %d,",freq);
      // Serial.printf("Total switching time: %d ms,",tok-tik);
      // Serial.printf("Switching intervall: %d ms,",tik-MultiTime);
      // Serial.println(" ");
      Serial.print(".");
      MultiTime=millis();
      i_multi=(i_multi+1)%Nmulti;
      
    }







    //  Write data to SD
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (((myBufferSize + BufferTail - BufferHead) % myBufferSize) > sdWriteSize) {
      lastTime = millis();
	  
      // write data to buffer
      if ((BufferHead + sdWriteSize) < myBufferSize) {  // case head + sdWritesize smaller than buffer size
        for (int i = 0; i < sdWriteSize; i++) {
          tempBuffer[i] = myBuffer[BufferHead];
          BufferHead++;
        }
      } else {
        int a = myBufferSize - BufferHead;
        for (int i = 0; i < a; i++) {
          tempBuffer[i] = myBuffer[BufferHead];
          BufferHead++;
        }
        BufferHead = 0;
        for (int i = a; i < sdWriteSize; i++) {
          tempBuffer[i] = myBuffer[BufferHead];
          BufferHead++;
        }
      }
      lastTime2 = millis();


      dataFile.write(tempBuffer, sdWriteSize);
      lastTime3 = millis();
      Serial.print(",Data written:");
      Serial.print(sdWriteSize);
      // Serial.print(" ,write to tempBuffer (ms):");
      // Serial.print(lastTime2-lastTime);
      Serial.print(" , in time (ms):");
      Serial.println(lastTime3-lastTime);
      delay(1);

      // include stamp for writing to SD
      strcpy(subString,"stamp SDwrite\n");
      int msglen=strlen(subString);
      for (int i = 0; i < msglen; i++){
          tempBuffer[i] = uint8_t(subString[i]);
        } 
      writeToBuffer(tempBuffer, msglen);
      // Serial.println("stamp SDwrite" );

    }


    //==========================================================

    
    
    // Call GPS info\url
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
	  if (lastTime_ASCB + ASCB_intervall < millis()) {
      lastTime_ASCB = millis();
      send_ASCB(asciiMessage2,sizeof(asciiMessage2));
      }


    // flush SDCard every "flushSD_intervall"
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (lastTime_flushSD + flushSD_intervall < millis()) {
      dataFile.flush();
      dataFile.flush();
      lastTime_flushSD = millis();
      Serial.println("flushed");
    }

    // Read battery voltage
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
	  if (lastTime_VBat + VBat_intervall < millis()) {
      lastTime_VBat = millis();
      logVBat();
	  }

    // Get Temperature
    // =-=-=-=-=-=-=-=-=-=-=-=-=-=-
    if (lastTime_Temp + Temp_intervall < millis()) {
      lastTime_Temp = millis();
      
      if (CheckTSensor[0]){
        logTemp(tempsens1,1);
      }
      if (CheckTSensor[1]){
        logTemp(tempsens2,2);
      }
      if (CheckTSensor[2]){
        logTemp(tempsens3,3);
      }
    }

  if (millis()-lastTime_MSGtoBase >intervall_MSGtoBase) {      // Send periodic messages to base
        
        // Serial.print("send check string");
        strcpy(txString2, "$LEMMS,");

        PINS1_status_msg_short(subString);
        strcat(txString2, subString);
        
        // add Laser data
        if (Laser_data.valid ){
          Laser_status_msg_short(subString);
          strcat(txString2, subString);
        }
        
        Send_tx_String(txString2);
        lastTime_MSGtoBase=millis();
      }


  } else {
    delay(50);  // wait 50 ms if not 
	//  Serial.print("loop() running on core ");
	// Serial.println(xPortGetCoreID());

    if (millis()-lastTime_MSGtoBase >intervall_MSGtoBase) {      // Send periodic messages to base
      LEM_ID.toCharArray(txString2,64);
      strcat(txString2, ": ");

      if (update_GPSstatus() ){
        msg_GPSStatus_short(subString);
        strcat(txString2, subString);
      }else{
        strcat(txString2, "no GPS,Status");
      }
      // sprintf(subString, ", N_PINS1:%02d", N_PINS1 );
      // strcat(txString2, subString);
      
      // add Laser data
      if (Laser_data.valid ){
        Laser_status_msg(subString);
        strcat(txString2, subString);
      }
      
      Send_tx_String(txString2);
      lastTime_MSGtoBase=millis();
    }
  }


  //################################# Do other stuff #############################################

  // Moving STOP and START in loop should make them independent of BLE
  if (StopFlag) {
    delay(100);
    StopFlag = false;
    // measuring = false;
	  // stop_Measuring(SD);
	xTaskCreatePinnedToCore(stopTask,"TaskEnd",10000, NULL, 1,  &TaskEnd,0); 
  }

  if (flagcalMulti) {
    delay(100);
    flagcalMulti = false;
    xTaskCreatePinnedToCore(CalInFlight_Multi_notify,"TaskCalMulti",10000, NULL, 1,  &TaskCalMulti,0); 
  }
  // if (StopMultiFlag) {
  //   StopFlag = false;
  //   // measuring = false;
	//   // stop_Measuring(SD);
  // }

  if (closefileFlag) {
    closefileFlag = false;
	  closeFile(SD);
    Serial.println("End of measurement");  
  }

  if (StartFlag) {
    StartFlag = false;
    startMeasuring();
  }


  // Check if there is message in txSTring to send over ESPNOW and Serial
  if ( BLE_message) {
    Send_tx_String(txString);
    BLE_message = false;
    strcpy(txString, "");
    delay(10); // bluetooth stack will go into congestion, if too many packets are sent
  }


  if (Serial.available()) {  // Check Serial inputs
    String rxValue = Serial.readString();
    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++)
        Serial.print(rxValue[i]);
        Serial.println();
		parse(rxValue);
      Serial.println("*********");
    }
  }

  if (ESP_inbytes_buff>1000) {      // show total bites coming in over ESPNOW
      ESP_inbytes+=ESP_inbytes_buff/1000;
      ESP_inbytes_buff=0;
      Serial.print("Total kb received:");
      Serial.println(ESP_inbytes);
      Serial.print("TESPNOw rate kB/s:");
      Serial.println((float)(ESP_inbytes-ESP_inbyteslast)/ESP_inbyteslasttime);
      ESP_inbyteslast=ESP_inbytes;
      ESP_inbyteslasttime=millis();
  }
  delay(5);
}
