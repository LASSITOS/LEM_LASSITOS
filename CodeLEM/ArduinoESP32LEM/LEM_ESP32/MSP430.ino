
// /////////////////////////////////////////////
// ---------------------------------------------
// Communication to Microcontroller
// ---------------------------------------------
// /////////////////////////////////////////////



void setADCGain(){
  char timestr[16];
	sprintf(timestr, "G%01d", GAIN_ADC); 
	UART_MSP430.print( timestr );
  Serial.println(timestr);
}

void startMicro(){
  char timestr[16];
	sprintf(timestr, "S%02d%02d%02d%02d%02d%02d", Year % 100, Month, Day, Hour, Minute, Second); 
  Serial.println(timestr);
	UART_MSP430.print( timestr );
}


void startMicro2(){
  //start Microcontroller data logger for testing
  char status_out[64];
  int wait=0;
  #define waitmax 300   //300 ms
  int i=0;
  Hour = random(23);
  Minute = random(59);
  Second = random(59);
  printDateTime();
  resetMicro();
  setADCGain();
  delay(500);
  startMicro();	 
  startTime=millis();   
  delay(3000);  
  statusMicro(status_out);
  
  Serial.print("Microcontroller output:");
  Serial.println(status_out);
}


void stopMicro(){ 
    UART_MSP430.print( CDM_stop);
	  delay(500)   ;  // Check that this is enought time fot the microcontroller stopping the measurements
}

void statusMicro(char *status_out){   
  int wait=1;
  #define waitmax 50 //5 ms
  int i=0;
  UART_MSP430.listen();
  while (UART_MSP430.read() >= 0){
    ;  // flush the receive buffer.
  }
  startTime=millis();  
	UART_MSP430.print( CDM_status );
  
	// delay(50);
  while(wait){
    while(UART_MSP430.available()){
        status_out[i] = UART_MSP430.read();
        if ( i>=62){
          // if (status_out[i]=='\n' || i>=62){
          wait=0;
          status_out[i+1]='\0';
          Serial.println(i);
          break;
        }
        i++;
    }
    if (millis()-startTime>waitmax){
      wait=0;
      Serial.print(i);
      status_out[i]='\0'; 
      Serial.print("Timeout");
    }
  }
  Serial.print("Microcontroller status:");
  Serial.println(status_out);
}



void ParseStatus(char *status_out){   	
  Serial.print("Parsing status message:");
  Serial.println(status_out);
	String out=status_out;
  
  MSP430_CMD_OK =0;
	MSP430_measuring  =0;
  MSP430_SD_ERR = 0;
  MSP430_CALIBRATING = 0;
	MSP430_STARTING = 0;
  MSP430_SD_FULL = 0;
  
	if(out.indexOf(str_RUNNING) != -1){
		MSP430_measuring = 1;
	}else if(out.indexOf(str_SD_ERR) != -1){
		MSP430_SD_ERR = 1;
		Serial.print("MSP430 SD not running!");
	}else if(out.indexOf(str_SD_FULL) != -1){
		MSP430_SD_FULL = 1;
		Serial.print("SD is full");
  }else if(out.indexOf(str_CALIBRATING) != -1){
		MSP430_CALIBRATING = 1;
		Serial.print("MSP430 is calibrating!");
  }else if(out.indexOf(str_STARTING) != -1){
		MSP430_STARTING = 1;
		Serial.print("MSP430 is starting!");
  }else if(out.indexOf(str_READY) != -1){
		Serial.print("MSP430 is READY!");
  }
}


void resetMicro(){
  UART_MSP430.print( CDM_reset );
	delay(500)   ;  // Check that this is enought time for the microcontroller stopping the measurements
	Serial.print("Sent reset to Microcontroller!");
}




//start Microcontroller data logger for LEM measurement
//----------------------------------
void startMSP430() {
  resetMicro();
  setADCGain();
  delay(500);

  startMicro();	  
  delay(3000)   ;  // Wait until microcontroller started the measurement
  statusMicro(status_out);
  Serial.print("\r\nMicrocontroller status: ");
  Serial.println(status_out);

  sprintf(subString,"\r\nStatus Micro: %s,",status_out);
  strcat(txString, subString);
  
  ParseStatus(status_out );
  if(ignore_MSP430_status){
    MSP430_measuring=1;   // Uncomment for eactivating status check !!!!!!!!!  Danger!!!!!!
  }
  
  if(!MSP430_measuring){
        strcat(txString, "\r\nMeasurement on Microcontroller not started!");
    if (MSP430_SD_ERR){
      strcat(txString, " SD not working");
	  if (SD_filecreated){
		dataFile.println(F("# SD card MSP430 not working"));
	  }
	}
    if (MSP430_SD_FULL){
      strcat(txString, " SD card Full");
	  if (SD_filecreated){
		dataFile.println(F("#MSP430 SD card Full"));
	  }
    }
    Serial.println(txString);  
    interrupt_Measuring();  
    vTaskDelete(TaskStart);
  }
}