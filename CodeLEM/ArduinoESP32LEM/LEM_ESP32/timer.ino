// Functions used to get GPS TOW in ESP32. TOW form NMEA messages is used + millis() for the remaining time inteval. If GPS is not available uses only millis().


unsigned long myTime(){
	unsigned long TOW=get_TOW_PINS1();
	if(TOW==0){
		TOW=get_TOW_PGPSP();
	}
	if(TOW==0){
		TOW=millis()-startTime;
	}
	return TOW;
}


unsigned long  get_TOW_PINS1(){
	if( PINS1_data.valid){
		return PINS1_data.TOW*1000+millis()-PINS1_data.timestp;
	} else{
		return 0;
	}
}


unsigned long  get_TOW_PGPSP(){
	if( PGPSP_data.valid){
		return PGPSP_data.TOW*1000+millis()-PGPSP_data.timestp;
	} else{
		return 0;
	}
}