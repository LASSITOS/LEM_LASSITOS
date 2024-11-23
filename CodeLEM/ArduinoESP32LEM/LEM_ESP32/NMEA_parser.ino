
// Parsing NMEA messages
//-------------------------
void LEM_Handler(){
  Serial.print("Got $LEM message with ");
  Serial.print(LEM_parser.argCount());
  Serial.print(" arguments.  ");
  char arg0[16];
  char msgOut[64];
  int arg;
  char valStr[16];

  if (LEM_parser.getArg(0,arg0)) {
    Serial.print(" Message handle: ");
    Serial.println(arg0);
    strcpy(msgOut,arg0);
  }
  else{
    Serial.print("Coud not get messge handle! ");
    return;
  }


  
  if (LEM_parser.argCount()>1){
    strcat(msgOut,":");
    for (int i=1;i<LEM_parser.argCount();i++){
      if (LEM_parser.getArg(i,arg)){
        sprintf(valStr,"%d:",arg);
        strcat(msgOut,valStr);
      }
    }
  } 
  // Serial.print(" LEM command: ");
  // Serial.println(msgOut);
  // sendToBase(msgOut);
  parse( msgOut);
  
}



// ------------------------------------------------------------------------
// NMEA parser definition and handlers
// ------------------------------------------------------------------------


void PINS1_Handler(){
  N_PINS1++;
  int arg;
  float arg2;
  char valStr[16];

  if (IMX_parser.getArg(0,arg2)) { PINS1_data.TOW = arg2;  }
  if (IMX_parser.getArg(1,arg)) { PINS1_data.GPSWeek = arg; }
  if (IMX_parser.getArg(2,arg)) { 
    PINS1_data.INSstatus = arg;
    PINS1_data.fix  = (arg & INS_STATUS_GPS_NAV_FIX_MASK) >> 24 ;
    PINS1_data.Base  = (arg & INS_STATUS_RTK_ERR_BASE_MASK) >> 28 ;
  }
  if (IMX_parser.getArg(3,arg)) {     PINS1_data.Hardwarestatus = arg;}

  if (IMX_parser.getArg(4,arg2)) { PINS1_data.roll = arg2; }
  if (IMX_parser.getArg(5,arg2)) { PINS1_data.pitch= arg2; }
  if (IMX_parser.getArg(6,arg2)) { PINS1_data.yaw = arg2; }

  if (IMX_parser.getArg(7,arg2)) { PINS1_data.Vel_X = arg2; }
  if (IMX_parser.getArg(8,arg2)) { PINS1_data.Vel_Y = arg2; }
  if (IMX_parser.getArg(9,arg2)) { PINS1_data.Vel_Z = arg2; }
  if (IMX_parser.getArg(10,arg2)) { PINS1_data.lat = arg2; }
  if (IMX_parser.getArg(11,arg2)) { PINS1_data.lon = arg2; }
  if (IMX_parser.getArg(12,arg2)) { PINS1_data.el_HAE = arg2; }

  PINS1_data.valid=true;
  PINS1_data.timestp=millis();

  // Print for testing
  // Serial.print("roll:");
  // Serial.println(PINS1_data.roll);
  
  if (measuring) { 
				writeToBuffer(tempBufferIMX5, tempBufferIMX5_i);}
  tempBufferIMX5_i=-1;
  
  return;
}

void PGPSP_Handler(){
  int arg;
  float arg2;

  if (IMX_parser.getArg(0,arg)) { PGPSP_data.TOW = arg;  }
  if (IMX_parser.getArg(1,arg)) { PGPSP_data.GPSWeek = arg; }
  if (IMX_parser.getArg(2,arg)) { 
    PGPSP_data.status = arg;
    PGPSP_data.fix  = (arg & GPS_STATUS_FIX_MASK) >> 8 ;
    PGPSP_data.NSat  = (arg & GPS_STATUS_NUM_SATS_USED_MASK)  ;
    PGPSP_data.BasePos  = (arg & GPS_STATUS_FLAGS_GPS1_RTK_BASE_POSITION_MASK) >> 24 ;
    PGPSP_data.flags  = (arg & GPS_STATUS_FLAGS_MASK) >> 16 ;
    PGPSP_data.fixOK  = (arg & GPS_STATUS_FLAGS_FIX_OK) >0 ;
  }
  if (IMX_parser.getArg(3,arg2)) { PGPSP_data.lat = arg2; }
  if (IMX_parser.getArg(4,arg2)) { PGPSP_data.lon = arg2; }
  if (IMX_parser.getArg(5,arg2)) { PGPSP_data.el_HAE = arg2; }
  if (IMX_parser.getArg(6,arg2)) { PGPSP_data.MSL = arg2; }
  if (IMX_parser.getArg(7,arg2)) { PGPSP_data.pDOP = arg2; }
  if (IMX_parser.getArg(8,arg2)) { PGPSP_data.hAcc = arg2; }
  if (IMX_parser.getArg(9,arg2)) { PGPSP_data.vAcc = arg2; }
  if (IMX_parser.getArg(10,arg2)) { PGPSP_data.Vel_X = arg2; }
  if (IMX_parser.getArg(11,arg2)) { PGPSP_data.Vel_Y = arg2; }
  if (IMX_parser.getArg(12,arg2)) { PGPSP_data.Vel_Z = arg2; }
  if (IMX_parser.getArg(13,arg2)) { PGPSP_data.sAcc = arg2; }
  if (IMX_parser.getArg(14,arg2)) { PGPSP_data.cnoMean = arg2; }
  if (IMX_parser.getArg(15,arg2)) { PGPSP_data.towOffset = arg2; }
  if (IMX_parser.getArg(16,arg2)) { PGPSP_data.leapS = arg2; }

  PGPSP_data.valid=true;
  PGPSP_data.timestp=millis();

  if (measuring) { 
				writeToBuffer(tempBufferIMX5, tempBufferIMX5_i);}
  tempBufferIMX5_i=-1;


  return ;
}




void GGA_Handler(){
  int arg;
  float arg2;
  char valStr[16];

  if (IMX_parser.getArg(0,arg2)) {
    GGA_data.Hour = (int)arg2/10000;
    GGA_data.Minute = (int)arg2/100%100;
    GGA_data.Second = (int)arg2%100;
  }
  if (IMX_parser.getArg(5,arg)) { GGA_data.fix = arg; }

  if (IMX_parser.getArg(1,arg)) { GGA_data.lat = arg; }
  if (IMX_parser.getArg(4,arg)) { GGA_data.lon = arg; }
  if (IMX_parser.getArg(8,arg)) { GGA_data.MSL = arg; }
  
  if (IMX_parser.getArg(7,arg)) { GGA_data.hDop = arg; }
  if (IMX_parser.getArg(6,arg)) { GGA_data.NSat = arg; }

  GGA_data.valid=true;

  GGA_data.timestp=millis();

  if (measuring) { 
				writeToBuffer(tempBufferIMX5, tempBufferIMX5_i);}
  tempBufferIMX5_i=-1;


  return ;
}

void ZDA_Handler(){
  int arg;
  float arg2;
  char valStr[16];

  if (IMX_parser.getArg(0,arg2)) {
    Hour = (int)arg2/10000;
    Minute = (int)arg2/100%100;
    Second = (int)arg2%100;
  }
  else{
    Serial.print("Coud not get messge handle! ");
    return ;
  }

  if (IMX_parser.getArg(1,arg)) {
    Day = arg;
  }
  if (IMX_parser.getArg(2,arg)) {
    Month = arg;
  }
  if (IMX_parser.getArg(3,arg)) {
    Year = arg;
  }

  validTime=true;
  return ;
}


void unknownCommand()
{
  // Serial.print("*** Unkown command : ");
  // char buf[6];
  // parser.getType(buf);
  // Serial.println(buf);
}

void errorHandler()
{
  if (IMX_parser.error()>1){
    Serial.print("***NMEA parser Error: ");
    Serial.println(IMX_parser.error()); 
  }
}

void errorHandler2()
{
  if (LEM_parser.error()>1){
    Serial.print("*** Error: ");
    Serial.println(LEM_parser.error()); 
  }

}





