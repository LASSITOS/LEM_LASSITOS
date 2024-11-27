
// /////////////////////////////////////////////
// ---------------------------------------------
//INS code
// ---------------------------------------------
// /////////////////////////////////////////////


void serialEventIMX5Task(void * params){
  unsigned int numBytes = 0;
  
  
  for(;;){
    if(IMX5.available()){
      while(IMX5.available()){
		  tempBufferIMX5_i++;
          tempBufferIMX5[tempBufferIMX5_i]=IMX5.read();
		  IMX_parser << tempBufferIMX5[tempBufferIMX5_i];
		  
		  
		  if(tempBufferIMX5_i==tempBufferIMX5_length-1){
			  if (measuring) { 
				writeToBuffer(tempBufferIMX5, tempBufferIMX5_i);}
			  tempBufferIMX5_i=-1;
		  }
      }
      
    }

    // }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}








// add checksum to asciiMessage
int FormatAsciiMessage(char *Message, int messageLength, char *outMessage) {
  int checkSum = 0;
  unsigned int ptr = 0;
  char buf[16];
  outMessage[0] = '\0';

  if (Message[0] == '$') {
    ptr++;
  } else {
    strcpy(outMessage, "$");
  }

  // concatenate Message to outMessage
  strcat(outMessage, Message);

  // compute checksum
  while (ptr < messageLength) {
    checkSum ^= Message[ptr];
    ptr++;
  }
  // Serial.print("Size of message: ");
  // Serial.println(messageLength);
  sprintf(buf, "*%.2x\r\n", checkSum);
  strcat(outMessage, buf);
  return sizeof(outMessage);
}


// setting up INS
void setupINS() {
  if (!FormatAsciiMessage(asciiMessage, sizeof(asciiMessage), asciiMessageformatted)) {
    Serial.println("Failed to encode ASCII get INS message\r\n");
  } else {
    Serial.println(asciiMessageformatted);
    IMX5.write(asciiMessageformatted);  //send instruction for sending ASCII messages
    Serial.println("Send instruction for sending ASCII messages to IMX5");
  }
  while (IMX5.read() >= 0)
    ;
  Serial.println(F("INS setting updated"));
  strcat(txString, "INS setting updated");

  // pulseStrobeIMX5();  
  // lastTime_STROBE = millis();
}


// setting up INS
void send_ASCB( char * Message, int messageLength) {
  if (!FormatAsciiMessage(Message,  messageLength, asciiMessageformatted)) {
    Serial.println("Failed to encode ASCII for INS message\r\n");
  } else {
    // Serial.println(asciiMessageformatted);
    IMX5.write(asciiMessageformatted);  //send instruction for sending ASCII messages
    // Serial.println("Pulled GPS messages");
  }
}




void getDateTime() {
  char msgOut[256];
  char msgStart[] = "$ASCB,0,,,,,,,,,,,6,";
  FormatAsciiMessage(msgStart, sizeof(msgStart), msgOut);
  String msg = "NotValidMessage";
  IMX5.write("$STPC*14\r\n");
  Serial.println(msgOut);
  IMX5.write(msgOut);
  lastTime = millis();
  validTime=false;
  char out;
  while (!validTime) {
    if (IMX5.available()) {  // Check Serial inputs
      while(IMX5.available()){
        out=IMX5.read();
        // Serial.print(out);
        IMX_parser << out;
      }
      // IMX_parser << IMX5.read();
    }

    if (millis() - lastTime > 5000) {
      Serial.print(F("No GPS. Setting date to default and random time!"));
      // out = parseDateTime("$GPZDA,001924,06,01,1980,00,00*41");
      parseDateTime("NotAValidMessage");
      break;
    }
    delay(50);
  }
  IMX5.write("$STPC*14\r\n");
  char msgStop[] = "$ASCB,0,,,,,,,,,,,0,";
  FormatAsciiMessage(msgStop, sizeof(msgStop), msgOut);
  Serial.println(msgOut);
  IMX5.write(msgOut);

  Serial.print(F("Current date:"));
  strcpy(txString, printDateTime().c_str());
  Serial.println(txString);
}



void getDateTime_old() {
  char msgOut[256];
  char msgStart[] = "$ASCB,0,,,,,,,,,,,6,";
  FormatAsciiMessage(msgStart, sizeof(msgStart), msgOut);
  int out = 0;
  String msg = "NotValidMessage";
  IMX5.write("$STPC*14\r\n");
  Serial.println(msgOut);
  IMX5.write(msgOut);
  lastTime = millis();
  while (!out) {
    if (IMX5.available()) {  // Check Serial inputs
      // String msg = Serial.readString();
      bitesToWrite = IMX5.available();
      if (tempBufferSize < bitesToWrite) {
        bitesToWrite = tempBufferSize;
      }
      IMX5.readBytes(tempBuffer, bitesToWrite);
      tempBuffer[bitesToWrite]='\0';
      String msg = (char*)tempBuffer;
      // Serial.println(msg);
      msg[bitesToWrite]='\0';
      Serial.printf("BitestoWrite: %d",bitesToWrite);
      out = parseDateTime(msg);
    }
    if (millis() - lastTime > 5000) {
      Serial.print(F("No GPS. Setting date to default and random time!"));
      // out = parseDateTime("$GPZDA,001924,06,01,1980,00,00*41");
      out = parseDateTime("NotAValidMessage");
      out = 1;
    }
    delay(10);
  }
  IMX5.write("$STPC*14\r\n");
  char msgStop[] = "$ASCB,0,,,,,,,,,,,0,";
  FormatAsciiMessage(msgStop, sizeof(msgStop), msgOut);
  Serial.println(msgOut);
  IMX5.write(msgOut);

  Serial.print(F("Current date:"));
  strcpy(txString, printDateTime().c_str());
  Serial.println(txString);
}

int parseDateTime(String GPDZA) {
  //$GPZDA,001924,06,01,1980,00,00*41\r\n
  //$GPZDA,032521,08,02,2023,00,00*46   
  int a=GPDZA.indexOf("$GPZDA");
  int b=GPDZA.indexOf("*",a+1);
  if (a != -1 and b!= -1 and a<b ) {
    Serial.print("Valid msg:");
    Serial.println(GPDZA.substring(a, b+3));
    int index[6];
    index[0] = a + 7;
    // Serial.print(index[0]);
    for (int i = 0; i < 5; i++) {
      index[i + 1] = GPDZA.indexOf(',', index[i]) + 1;
      // Serial.print(i+1);
      // Serial.println(index[i+1]);
    }
    // Serial.println(GPDZA.substring(index[1],index[2]-1));
    Day = GPDZA.substring(index[1], index[2] - 1).toInt();
    // Serial.print(Year);
    // Serial.println(GPDZA.substring(index[2],index[3]-1));
    Month = GPDZA.substring(index[2], index[3] - 1).toInt();
    // Serial.print(Month);
    // Serial.println(GPDZA.substring(index[3],index[4]-1));
    Year = GPDZA.substring(index[3], index[4] - 1).toInt();
    // Serial.print(Day);
    // Serial.println(GPDZA.substring(index[0],index[0]+2));
    Hour = GPDZA.substring(index[0], index[0] + 2).toInt();
    // Serial.print(Hour);
    // Serial.println(GPDZA.substring(index[0]+2,index[0]+4));
    Minute = GPDZA.substring(index[0] + 2, index[0] + 4).toInt();
    // Serial.print(Minute);
    // Serial.println(GPDZA.substring(index[0]+4,index[0]+6));
    Second = GPDZA.substring(index[0] + 4, index[0] + 6).toInt();
    // Serial.print(Second);
    // Serial.println("Date parsed");
    return 1;
  } else {
    Year = 2000;
    Month = 01;
    Day = 01;
    Hour = random(23);
    Minute = random(59);
    Second = random(59);

    Serial.print("Msg not valid:");
    Serial.println(GPDZA);
    
    return 0;
  }
}


String printDateTime() {
  sprintf(subString, "%d-%d-%d %d:%d:%d", Year, Month, Day, Hour, Minute, Second);
  Serial.println(subString);
  return subString;
}


void setIMX5message(){
	int IMXrate2=int( round(IMXrate/IMUdataRate));
	// sprintf(asciiMessage, "$ASCB,512,,,%d,,,,,,,,,",IMXrate2);
 	sprintf(asciiMessage, "$ASCE,0,3,%d,",IMXrate2); 
}


void pulseStrobeIMX5(){
 digitalWrite(IMX5_strobe, HIGH);
 delay(1);
 digitalWrite(IMX5_strobe, LOW);
}



// Get Laser data for 1 second and send it over BLE and serial
void check_PINS1() {
  strcpy(txString, "");
  lastTime = millis();
  if (!measuring and (lastTime-PINS1_data.timestp>PINS1_data.timevalid)) {
    
    PINS1_data.valid=false;
    send_ASCB(asciiMessage,sizeof(asciiMessage));
    while (!PINS1_data.valid and (millis() - lastTime < 1000 )){
      delay(10);
    }
  }
  PINS1_status_msg(subString);
  strcat(subString, "\n");
  Send_tx_String(subString);
}



void PINS1_status_msg(char *msg) {
  if(PINS1_data.valid){
    sprintf(msg, "Fix: %d, Roll:%.2f,Pitch:%.2f,Yaw:%.2f,Lat:%.5f,Lon:%.5f,H:%.2f",PINS1_data.fix, PINS1_data.roll,PINS1_data.pitch,PINS1_data.yaw,PINS1_data.lat,PINS1_data.lon,PINS1_data.el_HAE);
  } else { 
    strcpy(msg, "PINS1:notvalid");
  }
}

void PINS1_status_msg_short(char *msg) {
  if(PINS1_data.valid){
    // sprintf(msg, "F%d,H%.2f",PINS1_data.fix, PINS1_data.roll,PINS1_data.pitch,PINS1_data.yaw,PINS1_data.lat,PINS1_data.lon,PINS1_data.el_HAE);
    Serial.print("PINSvalid");
    sprintf(msg, "Fix%d",PINS1_data.fix);

  } else { 
    strcpy(msg, "PINS1:notvalid");
  }
}

// Get INS data for 1 second and send it over BLE and serial
void check_INS() {
  Send_tx_String(txString);
  strcpy(txString, "");
  if (!measuring) {
    strcat(txString, "\nIMX5 data \n# --------------------------------------\n");
    Send_tx_String(txString);
	strcpy(txString, "");
    while (IMX5.read() >= 0)
      ;  // flush the receive buffer.
    send_ASCB(asciiMessage2,sizeof(asciiMessage2));
    delay(1000);
    int availableBytes = IMX5.available();
	
	if (CharBufferSize < availableBytes) {
        availableBytes = CharBufferSize;
		  Serial.println("# Buffer full. Data were cut.");
    }
	
    IMX5.readBytes(myBuffer_Char, availableBytes);
    Serial.write(myBuffer_Char, availableBytes);
    if (   availableBytes>511){
      availableBytes=511;
    }
    myBuffer_Char[availableBytes]='\0';
    strcpy(txString, myBuffer_Char);
    sendToBase(txString);


    strcpy(txString, "# --------------------------------------\n");
    Send_tx_String(txString);
    
    strcat(txString, "\n# --------------------------------------\n");
    Send_tx_String(txString);
  } else {
    strcpy(txString, "Datalogging running. Can't run INS_check now!");
  }
  Send_tx_String(txString);
}



// Get INS data for 1 second and send it over BLE and serial
void check_INS2() {
  PGPSP_data.valid=false;
  GGA_data.valid=false;

  Send_tx_String(txString);
  strcpy(txString, "");
  if (!measuring) {
    strcat(txString, "\nIMX5 data \n# --------------------------------------\n");
    Send_tx_String(txString);
	  strcpy(txString, "");
    while (IMX5.read() >= 0)
      ;  // flush the receive buffer.
    send_ASCB(asciiMessage3,sizeof(asciiMessage3));
    delay(1000);
    int availableBytes = IMX5.available();
	
	  if (CharBufferSize < availableBytes) {
        availableBytes = CharBufferSize;
		  Serial.println("# Buffer full. Data were cut.");
    }
	
    IMX5.readBytes(myBuffer_Char, availableBytes);
    Serial.write(myBuffer_Char, availableBytes);
    if (   availableBytes>511){
      availableBytes=511;
    }
    myBuffer_Char[availableBytes]='\0';
    strcpy(txString, myBuffer_Char);
    sendToBase(txString);
    for(int i=0; i<availableBytes; i++ ){
        IMX_parser << myBuffer_Char[i];
    }
    delay(100);
    strcpy(txString, "# --------------------------------------\n");
    Send_tx_String(txString);
    delay(100);
    if (PGPSP_data.valid){
      msg_GPSStatus(txString); 
    } else {
      strcpy(txString, "No INS data");
    }
    Send_tx_String(txString);
    strcat(txString, "\n# --------------------------------------\n");
    Send_tx_String(txString);
  } else {
    strcpy(txString, "Datalogging running. Can't run INS_check now!");
  }
  Send_tx_String(txString);
}


// Get INS data for 1 second and send it over BLE and serial
bool check_GPSstatus() {
  Send_tx_String(txString);
  strcpy(txString, "");
  if (update_GPSstatus() ){
	msg_GPSStatus(txString);
  }else{
	strcpy(txString, "no GPS,Status");
  }
}



// Get INS data for 1 second and send it over BLE and serial
bool update_GPSstatus() {
  lastTime = millis();
  if (!measuring and (lastTime-PGPSP_data.timestp>PGPSP_data.timevalid)) {
    
    PGPSP_data.valid=false;
    send_ASCB(asciiMessage3,sizeof(asciiMessage3));
    while (!PGPSP_data.valid and (millis() - lastTime < 1000 )){
      delay(10);
    }
    return PGPSP_data.valid;

  } else if(lastTime-PGPSP_data.timestp<PGPSP_data.timevalid){
	return PGPSP_data.valid;
  }else {
    Serial.print("Datalogging running. Can't run INS_check now!");
    return false;
  }
}



void msg_GPSStatus_short(char *msg) {
  int fix=0;
  int NSat=0;
  char fixMSG[16];
  if (PGPSP_data.valid){
  // sprintf(msg, "GPS fix: %d, NSat:%d", PGPSP_data.fix, PGPSP_data.NSat);
  fix=PGPSP_data.fix;
  NSat=PGPSP_data.NSat;
  } else {
  // sprintf(msg, "GGA GPS fix: %d, NSat:%d", GGA_data.fix, GGA_data.NSat);
  fix=GGA_data.fix;
  NSat=GGA_data.NSat;
  }
  
    switch (fix){
    case(1):{
        strcat(fixMSG, "dead rec.");
        break;
    }
    case(2):{
        strcat(fixMSG, "2D");
        break;
    }
    case(3):{
        strcat(fixMSG, "3D");
        break;
    }
    case(4):{
        strcat(fixMSG, "GPS + dead rec.");
        break;
    }
    case(5):{
        strcat(fixMSG, "time only");
        break;
    }
    case(9):{
        strcat(fixMSG, "SBAS");
        break;
    }
    case(10):{
        strcat(fixMSG, "SINGLE");
        break;
    }
    case(11):{
        strcat(fixMSG, "FLOAT");
        break;
    }
    case(12):{
        strcat(fixMSG, "FIX");
        break;
    }
  }

  sprintf(msg, "GPS fix: %s, NSat:%d", fixMSG, NSat);

}

void msg_GPSStatus(char * msg) {
  sprintf(msg, "GPS: lat: %.5f, lon: %.5f, el.: %.5f, hAcc.: %.5f, vAcc.: %.5, NSat:%d,", PGPSP_data.lat, PGPSP_data.lon, PGPSP_data.MSL,PGPSP_data.hAcc,PGPSP_data.vAcc,PGPSP_data.NSat);
  strcat(msg, ", fix: ");
  switch (PGPSP_data.fix){
    case(1):{
        strcat(msg, " dead rekoning.");
        break;
    }
    case(2):{
        strcat(msg, "2D ");
        break;
    }
    case(3):{
        strcat(msg, "3D ");
        break;
    }
    case(4):{
        strcat(msg, "GPS + dead reckoning ");
        break;
    }
    case(5):{
        strcat(msg, "time only ");
        break;
    }
    case(9):{
        strcat(msg, " SBAS ");
        break;
    }
    case(10):{
        strcat(msg, "RTK single ");
        break;
    }
    case(11):{
        strcat(msg, "RTK float ");
    }
    case(12):{
        strcat(msg, "RTK fix ");
        break;
    }
  }
}
