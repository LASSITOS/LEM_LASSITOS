

// /////////////////////////////////////////////
// ---------------------------------------------
// SPI functions
// ---------------------------------------------
// ///////////////////////////////////////////
uint8_t mode = SPI_MODE0;
uint32_t spiCommand32( uint32_t msg , int CS ) {  
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, mode));
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  uint32_t out= spi.transfer32(msg);
  digitalWrite(CS, HIGH); //pull ss high to signify end of data transfer
  spi.endTransaction();
  // Serial.printf("Command to SPI send with CS: %d",CS);
  return out;
}

uint32_t spiCommandMSP32( uint32_t msg , int CS ) {  
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  uint32_t out= spi.transfer32(msg);
  digitalWrite(CS, HIGH); //pull ss high to signify end of data transfer
  spi.endTransaction();
  // Serial.printf("Command to SPI send with CS: %d",CS);
  return out;
}


uint8_t spiCommand8( uint8_t msg , int CS ) {  
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, mode));
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  uint8_t out= spi.transfer(msg);
  digitalWrite(CS, HIGH); //pull ss high to signify end of data transfer
  spi.endTransaction();
  // Serial.printf("Command to SPI send with CS: %d",CS);
  return out;
}

uint8_t spiCommandMSP8( uint8_t msg , int CS ) {  
  // Uses SPI_MODE1 for communicating with MSP430. Different protocoll from DAC.
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, SPI_MODE0)); 
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  uint8_t out= spi.transfer(msg);
  digitalWrite(CS, HIGH); //pull ss high to signify end of data transfer
  spi.endTransaction();
  // Serial.printf("Command to SPI send with CS: %d",CS);
  return out;
}


void spiTransfer( uint8_t msg[], uint32_t size, int CS, uint8_t out[] ) {  
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, mode));
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  for (int i=0; i<size; ++i){
    out[i]=spi.transfer(msg[i]);    
    // Serial.print(".");
  }
  digitalWrite(CS, HIGH); //pull ss high to signify end of data transfer
  spi.endTransaction();
}


void spiTransfer2( uint8_t msg[], uint32_t size, int CS ) {  
  // Serial.print("start transfer");
  spi.beginTransaction(SPISettings(SPI_rate, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, LOW); //pull SS slow to prep other end for transfer
  for (int i=0; i<size; ++i){
    spi.transfer(msg[i]);    
    // Serial.print(".");
  }
  digitalWrite(CS, HIGH); //pull ss high to signify end of data transfer
  spi.endTransaction();
  // Serial.print("stop transfer");
}




// /////////////////////////////////////////////
// ---------------------------------------------
// I2c to SPI functions + DAC communication
// ---------------------------------------------
// /////////////////////////////////////////////

uint32_t I2CspiCommand( uint32_t msg ) {  //use it as you would the regular arduino SPI API
  uint8_t out[4];
  uint8_t in[4];
  // Serial.print("Message to send: ");
  // Serial.printf("%x \n",msg);

  in[3]=msg & 0xff;
  in[2]=(msg >> 8 )& 0xff;
  in[1]=(msg >> 16 )& 0xff;
  in[0]=(msg >> 24 )& 0xff;
  
  // Serial.print("decomposed msg: ");
  // Serial.printf("%x, %x, %x, %x\n ",in[0],in[1],in[2],in[3]);

  spiBridge.spiTransfer(0,in,4,out);

  // Serial.print("decomposed out: ");
  // Serial.printf("%x, %x, %x, %x\n ",out[0],out[1],out[2],out[3]);

  return out[3]+ (out[2]<< 8) +  (out[1]<< 16) +(out[0] << 24) ;
}

void writeMSG (uint16_t addr,uint16_t dat) {
  uint32_t msg = (0x00 << 24) + (addr << 16) + dat;
  I2CspiCommand(  msg );
}

void writeReg (uint16_t addr,uint16_t dat) {
  //write register DAC AD9102
  writeMSG(  addr,dat);
  delay(1);
  writeMSG( 0x1D , 0x01 );
}

uint32_t readReg(uint16_t addr){
  //Read register DAC AD9102
//   Serial.println("Reading register");
   uint32_t msg = (0x80 << 24) + (addr << 16) + 0x0000;
   uint16_t out=I2CspiCommand(  msg );  //first 4 bytes are dropped by saving into uint16_t 
   return out;
}





// /////////////////////////////////////////////
// --------------------------------
// I2C communication
//---------------------------------
// /////////////////////////////////////////////
void setCswitchTx ( uint8_t state ){

   CSwitch_code = 0xFF-state;
	 //Write message to preiferal
	//  Serial.printf("New state is: %d\n", state);
	 Serial.printf("Setting Cswitch #: 0X%x to: 0X%x\n", CSwitchTx_address,CSwitch_code);
	 Wire.beginTransmission(CSwitchTx_address); // For writing last bit is set to 0 (set to 1 for reading)
	 Wire.write(CSwitch_code );
	 uint8_t error = Wire.endTransmission(true);
	 parsePrint_I2C_error(error);
}

uint8_t setCswitchTx2 ( uint8_t state ){
   CSwitch_code = 0xFF-state;
	 //Write message to preiferal
	//  Serial.printf("New state is: %d\n", state);
	//  Serial.printf("Setting Cswitch #: 0X%x to: 0X%x\n", CSwitchTx_address,CSwitch_code);
	 Wire.beginTransmission(CSwitchTx_address); // For writing last bit is set to 0 (set to 1 for reading)
	 Wire.write(CSwitch_code );
	 return Wire.endTransmission(true);
	//  parsePrint_I2C_error(error);
}

void setCswitchCal ( uint8_t state ){

	CSwitch_code = 0xFF-state;

	 //Write message to the slave
	//  Serial.printf("New state is: %d\n", state);
	 Serial.printf("Setting Cswitch Cal #: 0X%x to: 0X%x\n", CSwitchCal_address,CSwitch_code);
	 Wire.beginTransmission(CSwitchCal_address); // For writing last bit is set to 0 (set to 1 for reading)
	 Wire.write(CSwitch_code );
	 uint8_t error = Wire.endTransmission(true);
   parsePrint_I2C_error(error);
   
}

void parsePrint_I2C_error(int error){
  if (error>0){
      Serial.printf("EndTransmission with error: %u\n", error);
      switch(error){
        case 1:
          Serial.println("data too long to fit in transmit buffer");
          break;
        case 3:
          Serial.println("received NACK on transmit of data");
          break;
        case 2:
          Serial.println("received NACK on transmit of address");
          break;
        case 4:
          Serial.println("other error");
          break;
        case 5:
          Serial.println("timeout. Device can't be reached");
          break;
      }
  }
}

void write_I2C (uint8_t device, uint8_t address, uint8_t msg ){
  //Write message to the slave
  Serial.printf("Writing to reg: %x, msg: %x\n", address,msg);
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(msg);
  uint8_t error = Wire.endTransmission(true);
  Serial.printf("endTransmission: %u\n", error);
}


uint8_t readRegI2C (uint8_t device, uint8_t address){
  uint8_t bytesReceived=0;
  int temp=0;
  Serial.printf("Reading reg: %#02X\n", address);
  Wire.beginTransmission(device);
  Wire.write(address);
  uint8_t error = Wire.endTransmission(false);
  Serial.printf("endTransmission error: %u\n", error);
  //Read 16 bytes from the slave
  bytesReceived = Wire.requestFrom(device, 1);
  temp = Wire.read();
  Serial.printf("bytes received: %d\n", bytesReceived);
  Serial.printf("value: %x\n", temp);
  Serial.printf("value in decimal: %d\n", temp);
  return  temp;
}

void scanI2C (){
  byte error, address;
  int nDevices = 0;

  delay(5000);

  Serial.println("Scanning for I2C devices ...");
  for(address = 0x01; address < 0x7f; address++){
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0){
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if(error != 2){
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0){
    Serial.println("No I2C devices found");
  }
}



