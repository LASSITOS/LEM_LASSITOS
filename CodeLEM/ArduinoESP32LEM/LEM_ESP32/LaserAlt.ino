void serialEventLaserTask(void * params){
  unsigned int numBytes = 0;
  
  
  for(;;){
    if(RS232.available()){
      while(RS232.available()){
		  parserLaser << RS232.read();
		  }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}





void Laser_Handler(){
  // Serial.print(" ");
  Laser_data.LaserError=parserLaser.getLaserError( );
  Laser_data.valid=true;
  // Laser_data.dT=1000.0/(float)(millis()-Laser_data.timestp);
  Laser_data.timestp=millis();
  Laser_data.Dist=parserLaser.getDist();
  Laser_data.Temp=parserLaser.getTemp();  
  Laser_data.SigQual=parserLaser.getSigQual();

  // Laser_status_msg(subString);
  // Serial.println(subString);

  int bitesToWrite=parserLaser.getBuffer( tempBuffer);
  writeToBuffer(tempBuffer, bitesToWrite);
   
  return;
}


void errorHandlerLaser()
{
  if (parserLaser.error()>1){
    Serial.print("*** Error: ");
    Serial.println(parserLaser.error()); 
  }
}


void Laser_status_msg_short(char *msg) {
  if (Laser_data.LaserError==0) {
  // sprintf(msg, "Laser dist: %.3f m, SigQ: %.1f , Temp:%.1f degC, dRate: %.3f s-1", Laser_data.Dist, Laser_data.SigQual,Laser_data.Temp,Laser_data.dT);
  sprintf(msg, "LD %.3fm, SQ %.1f, T%.1fC", Laser_data.Dist, Laser_data.SigQual,Laser_data.Temp);

  } else{
    sprintf(msg,"DE%d:",Laser_data.LaserError);
    
    switch(Laser_data.LaserError){
      case 2:
        strcat(msg,"No distance");
        break;
      case 4:
        strcat(msg,"Device error ");
        break;
      case 6:
        strcat(msg,"T out of range ");
        break;
      case 10:
        strcat(msg,"laser voltage low");
        break;
    }
  }
}

void Laser_status_msg(char *msg) {
  if (Laser_data.LaserError==0) {
  // sprintf(msg, "Laser dist: %.3f m, SigQ: %.1f , Temp:%.1f degC, dRate: %.3f s-1", Laser_data.Dist, Laser_data.SigQual,Laser_data.Temp,Laser_data.dT);
  sprintf(msg, "Laser dist: %.3f m, SigQ: %.1f , Temp:%.1f degC", Laser_data.Dist, Laser_data.SigQual,Laser_data.Temp);

  } else{
    sprintf(msg,"Laser error %d:",Laser_data.LaserError);
    
    switch(Laser_data.LaserError){
      case 2:
        strcat(msg,": No distance identified ");
        break;
      case 4:
        strcat(msg,": Device error ");
        break;
      case 6:
        strcat(msg,": Temperature out of range ");
        break;
      case 10:
        strcat(msg,": laser voltage lower than min. voltage");
        break;
    }
  }
}



// Get Laser data for 1 second and send it over BLE and serial
void check_laser() {
  Send_tx_String(txString);
  strcpy(txString, "");
  
  // if (!measuring) {
    // lastTime = millis();
    // Laser_data.valid=false;
    // while ((!Laser_data.valid) ){
      // while (RS232.available() ) {
        // parserLaser << RS232.read() ;
      // }
      // delay(5);
      // if(millis() - lastTime < 1000) 
        // break;
    // }
  // }
  Laser_status_msg(subString);
  Send_tx_String(subString);
}



// Get Laser data for 1 second and send it over BLE and serial
void check_laser_old() {
  Send_tx_String(txString);
  strcpy(txString, "");
  if (!measuring) {
    strcat(txString, "\nLaser data \n# --------------------------------------\n");
    Send_tx_String(txString);
	  strcpy(txString, "");
    lastTime = millis();
    while (RS232.read() >= 0)
      ;  // flush the receive buffer.
    while (millis() - lastTime < 300)
      ;
    int availableBytes = RS232.available();
    if (CharBufferSize < availableBytes) {
        availableBytes = CharBufferSize;
		    strcat(txString, "# Buffer full. Data were cut.\n");
    }
	
	  RS232.readBytes(myBuffer_Char, availableBytes);
    Serial.write(myBuffer_Char, availableBytes);

    if (   availableBytes>511){
      availableBytes=511;
    }
    myBuffer_Char[availableBytes]='\0';
    strcpy(txString, myBuffer_Char);
    sendToBase(txString);


    strcpy(txString, "# --------------------------------------\n");
    Send_tx_String(txString);
  } else {
    strcpy(txString, "Datalogging running. Can't run LASER_check now!");
  }
  Send_tx_String(txString);
}