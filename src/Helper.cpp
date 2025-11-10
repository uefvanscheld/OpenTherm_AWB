/*
Edit history of Helper.ino


V 0.21:		- moved some functions and procedures from main.ino into here (Helper.ino)
			

*/

/*
*********************************************************************
*********************************************************************
 some service & setup functions
*********************************************************************
*********************************************************************
*/ 


/* 
********************************************************************************
 initialize interrupts for master frame send (1 Hz) and master bit send (2 kHz)
********************************************************************************
*/ 
void initialize_interrupts ()
{
    // setting is for MEGA2560
    // Timer3 Settings:
        // prepare timer 2 for 2 kHz timer frequency: bits with 1kbaud, but 1 interrupt for each half bit using the Manchester code
        // use ISR(TIMER3_COMPA_vect) as the related interrupt service routine
        TCNT3  = 0;     // initialize timer/counter value to 0
        TCCR3B = 0x00;      // Output Compare Register B: Disable Timer3 while we set it up
        TCCR3A = 0x00;    // Output Compare Register A:
        TIMSK3 = 0x00;    // set timer interrupt mask register to: disable/block all interrupts

        //   Clear Timer on Compare Match (CTC), Mode=4: WGMx3 = 0, WGMn2 = 1, WGMn1 = 0, WGMn0 = 0
        TCCR3B |= (1 << WGM32);
        // set top value for counter to INT3_INTERVAL_SHORT; together with prescaler=256 this results in 2000 interrupts per second
        OCR3A = INT3_INTERVAL_SHORT;
        //   set Timer 3 prescaler in Output Compare Register B to 256: CS22=1, CS21=0, CS20=0
        TCCR3B |= (1 << CS32);
        //Timer3 INT Reg: Timer3 Overflow Interrupt Enable
        //TIMSK3 |= (1 << OCIE3A);      
        
    // Interrupt Timer4 Settings
    // a) one interrupt each second to trigger launching a new frame to be sent to slave
    // use ISR(TIMER4_COMPA_vect) as the related interrupt service routine
        TCCR4A = 0;// set entire TCCR4A register to 0 to remove older configuration
        TCCR4B = 0;// same for TCCR4B; disable Timer 4 during configuration
        TCNT4  = 0;//initialize counter value to 0
        // Clear the timer 4 interrupt flag
        TIFR4  |=  (1 << OCF4A);
        
        //   Clear Timer on Compare Match (CTC), Mode=4: WGMx3 = 0, WGMn2 = 1, WGMn1 = 0, WGMn0 = 0
        TCCR4B |= (1 << WGM42);
        // Set CS42 bit for prescaler = 256
        TCCR4B |= (1 << CS42);  
        // set timer4 to interrupts at 1Hz 
        // start with a long interrupt interval when entering the main loop
        OCR4A = INT4_INTERVAL_LONG;// = (16*10^6) / (1*256) - 1 (must be <65536)
        // enable timer compare interrupt on match with output compare register A
        // TIMSK4 |= (1 << OCIE4A);     
}

void EnableMasterBitInterrupt() 
{
  // reset timer 3 counter to 0
  TCNT3  = 0;     // initialize timer/counter value to 0     
  // allow timer 3 to cause interrupts
  // timer3 INT Reg: Timer3 Overflow Interrupt Enable
  TIMSK3 |= (1 << OCIE3A);   
}

void DisableMasterBitInterrupt() 
{
  // prevent timer 3 to cause interrupts
  // timer3 INT Reg: Timer3 Overflow Interrupt disable
  TIMSK3 &= ~(1 << OCIE3A);
}


void EnableMasterFrameInterrupt() 
{
  // allow timer 4 to cause interrupts
  // timer4 INT Reg: Timer3 Overflow Interrupt Enable
  TIMSK4 |= (1 << OCIE4A);   
  // reset timer 4 counter to 0
  TCNT4  = 0;     // initialize timer/counter value to 0     
}

void DisableMasterFrameInterrupt() 
{
  // prevent timer 4 to cause interrupts
  // timer4 INT Reg: Timer3 Overflow Interrupt disable
  TIMSK4 &= ~(1 << OCIE4A);   
}


void EnableSlaveReceiveInterrupt()
{
  // start sensing the receiver pin and let it raise an interrupt
  attachInterrupt(digitalPinToInterrupt(RxPin), Rx_trigger, CHANGE); 
}

void DisableSlaveReceiveInterrupt()
{
  // disable interrupts on receiver pin to avoid interferences between sending an receiving
  detachInterrupt(digitalPinToInterrupt(RxPin)); 
}


boolean mqtt_build_next_message(char *topic, char *payload)			// return value: true: new message built to be published; 	false: nothing new found  
{	
	for (uint8_t i=0; i < MB_length; i++)		// search whole message buffer (for max 1 loop) for an updated OpenTherm parameter; start with entry following the latest one .....
	{
		MQTT_MB_pointer = ++MQTT_MB_pointer % MB_length;	// move pointer on buffer to next parameter
		if (MonitorBuffer[MQTT_MB_pointer].changed) 		// check if value was updated lately ...
		{
			memset(topic, 0, sizeof(topic));    // clear text buffer
			memset(payload, 0, sizeof(payload));    // clear text buffer
			memset(fl_buffer, 0, sizeof(fl_buffer));    // clear text buffer
			// build complete topic including label 
			sprintf(topic,"%s%s%s",MQTT_pub_topic_root, MonitorBuffer[MQTT_MB_pointer].topic, MonitorBuffer[MQTT_MB_pointer].par_name);   // new version with V 0.202: payload only consists of value, label is part of topic
			// build payload
			//dtostrf((float)MonitorBuffer[MQTT_MB_pointer].r_value / 1, 1, 3, (char *)payload[sizeof(payload)-1]);    // use dtostrf function to get better formatting options
			
			// payload (format) depends on message id;  use dtostrf function to get better formatting options
			switch (MonitorBuffer[MQTT_MB_pointer].r_id) {		// check id from slave's response and adapt payload format accordingly
				case 0:
					dtostrf(getValueLB(MonitorBuffer[MQTT_MB_pointer].r_value), 1, 0, fl_buffer);    // only send slave's status (lower byte)
				break;
				case 3:
					dtostrf(getValueHB(MonitorBuffer[MQTT_MB_pointer].r_value), 1, 0, fl_buffer);    // only send slave's configuration/capabilities (upper byte)
				break;
				case 5:
					dtostrf(getValueHB(MonitorBuffer[MQTT_MB_pointer].r_value), 1, 0, fl_buffer);    // only send application/Boiler fault flags (upper byte)
				break;
				case 17:	// modulation level
				case 18:	// CH water pressure
				case 19:	// DHW flow rate
				case 25:	// boiler water temparature
				case 26:	// DHW water temperature
				case 28:	// CH return water temperature
				case 125:	// slave's OT software version 
					dtostrf((float)MonitorBuffer[MQTT_MB_pointer].r_value / 256, 1, 2, fl_buffer);    // send decimal value with 2 digits of precision
				break;
				default:
					strncpy(fl_buffer, "ERROR: unkown ID", sizeof(fl_buffer));
				break;
			}
			sprintf(payload,"%s",fl_buffer);					// new version with V 0.202: payload only consists of value, label is part of topic
			MonitorBuffer[MQTT_MB_pointer].changed = false;		// track that value was published
			return true;										// and leave function indicating that a new MQTT has to be published
		}		// end of if
	}		// end of for loop
	return false ;		// default return value: nothing to be published via MQTT
} 



// compute the even parity of a word value
// returns 1 
byte parity_even_calc(unsigned long v)
{
  // code taken from http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel
  v ^= v >> 16;
  v ^= v >> 8;
  v ^= v >> 4;
  v &= 0xf;
  return (0x6996 >> v) & 1;
}

// convert 16bit data value to float
float DataID2Float(signed int data_val)
{
 return ((float) ((float) data_val / 256.00));  
}


void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
     frac = (val - int(val)) * precision;
   else
      frac = (int(val)- val ) * precision;
   int frac1 = frac;
   while( frac1 /= 10 )
       precision /= 10;
   precision /= 10;
   while(  precision /= 10)
       Serial.print("0");

   Serial.print(frac,DEC) ;
}


// extract higher 8 bits from value in OT frame
byte getValueHB(unsigned int value){
	return (byte) (value >> 8);
}

// extract lower 8 bits from value in OT frame
byte getValueLB(unsigned int value){
	return (byte) (value);
}


// next routines are used to print binary values with leading zeros
// code is taken from http://stackoverflow.com/questions/699968/display-the-binary-representation-of-a-number-in-c
// Code to use this: 
//      void test()
//      {
//       char so[width+1]; // working buffer for pBin
//       long int   val=1;
//       do
//       {
//         printf("%ld =\t\t%#lx =\t\t0b%s\n",val,val,pBinFill(val,so,0));
//         val*=11; // generate test data
//       } while (val < 100000000);
//      }

char* pBin(unsigned long long int x,char *so)
{
 char s[width+1];		// width defines maximum # of bits that can be handled (64 bits here)
 int    i=width;
 s[i--]=0x00;   // terminate string
 do
 { // fill in array from right to left
  s[i--]=(x & 1) ? '1':'0';  // determine bit
  x>>=1;  // shift right 1 bit
 } while( x > 0);
 i++;   // point to last valid character
 sprintf(so,"%s",s+i); // stick it in the temp string string
 return so;
}

char* pBinFill(unsigned long long int x,char *so, char fillChar)
{ // fill in array from right to left
 char s[width+1];
 int    i=width;
 s[i--]=0x00;   // terminate string
 do
 {
  s[i--]=(x & 1) ? '1':'0';
  x>>=1;  // shift right 1 bit
 } while( x > 0);
 while(i>=0) s[i--]='0';    // fill with fillChar 
 sprintf(so,"%s",s);
 return so;
}

char* pBinFill8(unsigned long long int x,char *so, char fillChar)
{ // fill in array from right to left
 byte w = 8;	// can handle 8 bits
 char s[w+1];
 int    i=w;
 s[i--]=0x00;   // terminate string
 do
 {
  s[i--]=(x & 1) ? '1':'0';
  x>>=1;  // shift right 1 bit
 } while( x > 0);
 while(i>=0) s[i--]='0';    // fill with fillChar 
 sprintf(so,"%s",s);
 return so;
}

char* pBinFill16(unsigned long long int x,char *so, char fillChar)
{ // fill in array from right to left
 byte w = 16;	// can handle 16 bits
 char s[w+1];
 int    i=w;
 s[i--]=0x00;   // terminate string
 do
 {
  s[i--]=(x & 1) ? '1':'0';
  x>>=1;  // shift right 1 bit
 } while( x > 0);
 while(i>=0) s[i--]='0';    // fill with fillChar 
 sprintf(so,"%s",s);
 return so;
}


void LogFSM() {
	// *LogDataStringp = LogDataString;	// Pointer auf Anfang des Arrays setzen

	sprintf(LogDataString, "%ld", millis());
	//printf("%'8u\n", x)
	//printf("%'d\n", 1123456789);
	Serial.print (LogDataString);
	Serial.print("\tFSM:");
	Serial.print(FSM_State);
    Serial.print(";\tMB_pointer:");
    Serial.print(MB_pointer);
    Serial.print(";\tRx_Bit_Cnt:");
    Serial.print(Rx_Bit_Cnt);
    Serial.print(";\tRx_Intrpt_Cnt:");
    Serial.print(Rx_Intrpt_Cnt);
    Serial.print(";\tRx_Timing_Viol_Cnt:");
    Serial.print(Rx_Timing_Viol_Cnt);
	Serial.println("");

/*  
	Serial.print("Bit_Fence_Max:");
    Serial.print(Rx_Timing_Bit_Fence_Max);
    Serial.print(";\tBit_Fence_Min:");
    Serial.print(Rx_Timing_Bit_Fence_Min);
    Serial.print(";\tHlfBit_Fence_Max:");
    Serial.print(Rx_Timing_HlfBit_Fence_Max);
    Serial.print(";\tHlfBit_Fence_Min:");
    Serial.print(Rx_Timing_HlfBit_Fence_Min);
    Serial.print(";\t Rx_Duration_IntService:");
    Serial.print(Rx_Duration_IntService);
	Serial.println("");
	Serial.println("");
 */
	FSM_State_old = FSM_State;

}

void LogFrameDebug() {
	Serial.print(millis());
	Serial.print(":  MB_pointer:");
	Serial.print(MB_pointer);
	Serial.print(";  FSM_State:");
	Serial.print(FSM_State);
	Serial.print(";   MST_Bit_Int_Cnt:");
	Serial.print(MST_Bit_Int_Cnt);
	Serial.print("; Master_Frame_Cnt:");
	Serial.print(Master_Frame_Cnt);
	Serial.print("; Slave_Frame_Cnt:");
	Serial.print(Slave_Frame_Cnt);
	Serial.print(";  Err_State:");
	Serial.print(Err_State);
	Serial.print(";  Rx_Bit_Cnt:");
	Serial.print(Rx_Bit_Cnt);
	Serial.print(";  Rx_Buffer:");
	Serial.print( (unsigned long) Rx_Buffer, BIN);
	Serial.print(";  Type:");
	Serial.print(MonitorBuffer[MB_pointer].r_type, BIN);
	Serial.print(";  Data-ID:");
	Serial.print(MonitorBuffer[MB_pointer].r_id);
	Serial.print(";  Value:");
	Serial.print(MonitorBuffer[MB_pointer].r_value, BIN);
	Serial.print(";  Master frame:0b");
	Serial.print(pBinFill(MSTR_FRM_Frame_Copy,so,0));
	Serial.println("");
}

void LogAppLayer(byte id, byte type, unsigned int value) {
	Serial.print(millis());
	// log message type
	Serial.print("\tType=");
	Serial.print(type);
	Serial.print("\t[");
	Serial.print(MsgTypeArray[type]);
	Serial.print("]\t");
	// each DATA-ID has to be handled specifically because value field has to interpreted in different ways 
	switch (id) {
		case 0:			// Status of master and slave
			Serial.print((char *)MonitorBuffer[0].par_name);
			Serial.print(":\t");
			Serial.print(pBinFill8(getValueHB(value),so,0));
			Serial.print(" ");
			Serial.print(pBinFill8(getValueLB(value),so,0));
		break;
		case 3:			// Configuration of master and slave
			Serial.print((char *)MonitorBuffer[2].par_name);
			Serial.print(":\tSlave config: ");
			Serial.print(pBinFill8(getValueHB(value),so,0));
			Serial.print("\tSlave ID: ");
			Serial.print(pBinFill8(getValueLB(value),so,0));
		break;
		case 5:			// Fault flags
			Serial.print((char *)MonitorBuffer[1].par_name);
			Serial.print(":\tFault flags: ");
			Serial.print(pBinFill8(getValueHB(value),so,0));
			Serial.print("\tOEM error code: ");
			Serial.print(getValueLB(value));
		break;
		case 17:		// Relative modulation level [0..100]
			Serial.print((char *)MonitorBuffer[3].par_name);
			Serial.print(": ");
			Serial.print(value);
		break;
		case 18:		// Domestic hot water pressure 
			Serial.print((char *)MonitorBuffer[4].par_name);
			Serial.print(": ");
			Serial.print(DataID2Float(value));
		break;
		case 19:		// Domestic hot water flow rate 
			Serial.print((char *)MonitorBuffer[5].par_name);
			Serial.print(": ");
			Serial.print(DataID2Float(value));
		break;
		case 25:		// Boiler water temp.   
			Serial.print((char *)MonitorBuffer[6].par_name);
			Serial.print(": ");
			Serial.print(DataID2Float(value));
		break;
		case 26:		// DHW temperature  
			Serial.print((char *)MonitorBuffer[8].par_name);
			Serial.print(": ");
			Serial.print(DataID2Float(value));
		break;
		case 28:		// Return water temperature   
			Serial.print((char *)MonitorBuffer[7].par_name);
			Serial.print(": ");
			Serial.print(DataID2Float(value));
		break;
		case 125:		// OpenTherm version Slave 
			Serial.print((char *)MonitorBuffer[9].par_name);
			Serial.print(": ");
			Serial.print(DataID2Float(value));
		break;
	}
	Serial.println("");
}