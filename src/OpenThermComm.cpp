
/*
*********************************************************************
*********************************************************************
*
*    Code related to OpenTherm communication between master to slave
*
*********************************************************************
*********************************************************************
*/
/*
Edit history of OpenThermComm.ino


V 0.22:		- in ISR for tranitting new frame: change pattern to select next mesage to be sent: combined RoundRobin & minimal Interval (to avoid status request to block manual mode (heating enabled/disabled) change at boiler)
V 0.21:		- moved some functions and procedures from main.ino into here (OpenThermComm.ino)
			



Debugging of V 0.22 regading boiler control panel blocked for changing operation mode
From manual: message ID 0 provide desired operation mode to boiler via bit mapping in Baster byte (HB) of the message content.
Remember to reset boiler once a new firmware version was built.
Here're the things tried so far:
- Master requested DHW for all tests mentioned below.
- after reset boiler showed op mode "dhw only" on control panel
1) introduced min sending interval for message ID 0
	result: didn't solve the problem; even with minimum interval set to 5 minutes
2) disabled sending of message ID 0 at all: (launched 11:30h, Aug. 9th)
	result:	operation mode was changeable and remained with DHW only --> so, message ID 0 is root cause for boiler control panel behavior
3) changed op mode requested by master to 0x0 (= NO CH and NO DHW); interval: 60 sec
	result:
	
	



*/

#include "OpenTherm.hpp"        // add OpenTherm support
#include "Includes.hpp"         // add some general stuff
#include "Helper.hpp"         // add some general stuff

/*
----------------------------------------------------------------------------
  Handler routine used to trigger new OT frame to be sent from master to slave
 ----------------------------------------------------------------------------
*/
ISR(TIMER4_COMPA_vect) {
	// do some cleanup before starting new dialogue
	MST_Bit_Int_Cnt = 0;
    SLV_FRM_Frame = 0;		// clear receive buffer for reply expected from slave
    Rx_Buffer = 0;			// clear receive buffer for reply expected from slave
    Rx_Bit_Cnt = 0;			// clear Rx bit counter
    Rx_Intrpt_Cnt = 0;		// clear interrupt counter
	Rx_Bit_One_Cnt = 0;		// clear counter for '1' bits (--> parity check) 
    //Rx_Edge_Cnt = 0;		// dlear counter for interupts on Rx line
	//Rx_Timing_Viol_Cnt = 0;	// reset counter for timing violations
    UnevaluatedReply = false;           // no uncomplete message from slave available
	//Rx_Timing_Bit_Fence_Max = 1000;
	//Rx_Timing_Bit_Fence_Min = 1000;
	//Rx_Timing_HlfBit_Fence_Max = 500;
	//Rx_Timing_HlfBit_Fence_Min = 500;
	
	/* prepare next message to be sent to slave */
	unsigned long frame = 0;  // 32bit buffer to build the frame to be sent ....
	short par = 0;        // initialize parity bit to 0
	int j;
	MST_Frame_Int_Cnt++;		// track # of frame interrupts
	Master_Frame_Cnt++;
	
	// MTIntCount++;
	
	// disable hardware interrupts to prevent interferences 
	// prevent signals on receiver pin to cause an interrupt while not waiting for an slave reply 
	DisableSlaveReceiveInterrupt();
	
	// find next message to be transmitted based on mixed approach with RoundRobin and minimal interval
	while (1)	{	
		// before here MB_pointer points to last message sent
		MB_pointer = (MB_pointer + 1) % MB_length;							// move pointer to next message and prevent overflow

		/*
		if (MonitorBuffer[MB_pointer].s_skip == true) {							// if this message is disabled ...
			continue;    														// ... go to next buffer entry
		}
		else if (MonitorBuffer[MB_pointer].trnsmt_int == 0) {					// check if this message is controlled by interval for minimum transmit frequency
			break;    															// this message has no limitiations regarding transmit frequency, so use it for this transmission
		}
		else if (FSM_timestamp >= MonitorBuffer[MB_pointer].trnsmt_next) {		// if blackout period is over ....
			MonitorBuffer[MB_pointer].trnsmt_next = FSM_timestamp + MonitorBuffer[MB_pointer].trnsmt_int;		// move timestamp forward by adding interval ....
			break;																// ... send this message
		}
		// current message doesn't match any criteria (pure RoundRobin neither interval controlled)
		// so try next one
		*/
	}
  
	LastDialogueIsError = 0;  // reset communication status for this frame transmission
	
	// first build frame from data provided
	frame = (((unsigned long)MonitorBuffer[MB_pointer].s_type) << 27)  |    // move bits for msg_type by additional 3 bits to consider the OpenTherm spare bits
			(((unsigned long)MonitorBuffer[MB_pointer].s_id) << 16) |
			((unsigned long)MonitorBuffer[MB_pointer].s_value);
	
	// calculate parity to achieve an even number of bits
	par = parity_even_calc(frame);
	// set parity bit in frame if required
	if (par) frame |= Tx_frame_PAR_mask;   // gleich bedeutend mit frame = frame | 2147483648; 2147483648 is the MSB bit of the 32 unsigned long frame
	//  Serial.print("frame: ");
	//  Serial.println(frame, BIN);
	
	// add start and stop bit     
	MSTR_FRM_Frame =  (unsigned long long)frame << 1 | MSTR_FRM_STRTSTP_Bit_Mask;
	MSTR_FRM_Frame_Copy = MSTR_FRM_Frame;        
	// prepare sending 
	MSTR_FRM_HalfBitCount = MSTR_FRM_HalfBitLength;	// set counter for tracking half bits
	// FSM_State = Tx_Trig_Mstr_Frame;
	OCR3A = INT3_INTERVAL_SHORT;          			// ... and preload timer with 2kHz interrupt value for master frame bits
	EnableMasterBitInterrupt();						// enable the bit sending interrupt
	
	tstMSTRStartTX = micros();						// keep timestamp for start of new dialogue (before master start to transmit it's frame)
}


/* 
// for later usage

// use the master frame components, build the 32bit word to be sent
// and add to transmit queue (uses QueueArray Library For Arduino); see http://playground.arduino.cc/Code/QueueArray
// check for maximum size of queue before adding new item
byte Tx_queue_frame(byte msg_type, byte data_id, unsigned int data_val)
{
  unsigned long frame = 0;  // initialize frame to 0
  byte par = 0;				// initialize parity bit to 0
  
  // first build frame from data provided
  frame = ((unsigned long)(msg_type) << 27)      // move bits for msg_type by additional 3 bits to consider the OpenTherm spare bits
            | ((unsigned long)(data_id) << 16)
            | data_val;

  // calculate parity to achieve an even number of bits
  par = parity_even_calc(frame);
  // set parity bit in frame if required
  if (par) frame |= Tx_frame_PAR_mask;   // gleich bedeutend mit frame = frame | 2147483648; 2147483648 is the MSB bit of the 32 unsigned long frame
  //  Serial.print("frame: ");
  //  Serial.println(frame, BIN);
  
  // if possible add additional item to queue	
  if ( (!(Tx_queue.isFull ())) && (Tx_queue.count () < Tx_queue_elements_MAX))		// check if queue if full or max size of queue is reached 
  {
	Tx_queue.push (frame);															// if not add new item
	return (0);
  }
  else																				// otherwise return with error
  {
	return (-1);
  }
}
*/

/*
*********************************************************************
*
*   Master transmission
*
*   transmit frame bits from master to slave
*	this is the interrupt service routine for timer 3 in CTC mode
*	
*	this routine handles two types of interrupts:
*	1) timeout for receiving a response from slave
*	2) trigger to send new bit of frame from master to slave
*	
*
*********************************************************************
*/
ISR(TIMER3_COMPA_vect)
{

  /* =========================
  *	check if slave's response timed out since master frame transmission was completed (>800 ms)
  *	this only applies while waiting for start bit of slave's reply to be fully camptured
  * otherwise we assume slave replies within defined timing (validation of data received happen somewhere else)
  *  =========================  */
  // if ((FSM_State == Rx_Wait_StrtBt_Lead) || (FSM_State == Rx_Wait_StrtBt_Trns))
  if ((FSM_State >= Rx_Wait_StrtBt_Lead) && (FSM_State < Rx_Frame_Cmplt))
  {
    FSM_State = Rx_Err_Tmout_StrtBt;      // update status
    MonitorBuffer[MB_pointer].err_cnt++;  // increment error counter for this dialogue
    DisableMasterBitInterrupt();          // disable  trigger
    return;   // leave ISR
  }
  
  
  /* =========================
  *
  *	process next (half)bit of frame to be sent to slave
  *
  *  =========================
  */
  MST_Bit_Int_Cnt++;		// track # of bit interrupts

  // if a new bit begins get the bit value 
  if (!(MSTR_FRM_HalfBitCount & 1)) {         // if counter is even get next bit value
        // read next bit to be sent from frame buffer starting from the MSB end of the frame
        if (MSTR_FRM_Frame & MSTR_FRM_SendBit_Mask) {
            MSTR_FRM_SendBit = 1;
		// IMPORTANT: INVERTING behaviour of hardware interface between Arduino and OT's side !!!
            // TxSetHigh;      			// in case of a '1' to send start new bit with high voltage level 
			digitalWrite(TxPin, LOW);	// in case of a '1' to send start new bit with high voltage level and bring OT voltage to high level (> 15V)
        }
        else {
            MSTR_FRM_SendBit = 0;
            // TxSetLow;				// in case of a '0' to send start new bit with low voltage level 
			digitalWrite(TxPin, HIGH);	// in case of a '0' to send start new bit with low voltage level and bring OT voltage to idle level (< 7V)
        }   // end of if for first half of Manchester coded bit
  }
  else     {    						// process second half of Manchester coded bit
        // TxToggle;     							// for second half of the bit just toggle the output voltage level
		digitalWrite(TxPin, !digitalRead(TxPin));	// just toggle transmit pin (Manchester coding)
        MSTR_FRM_Frame = MSTR_FRM_Frame << 1;		// when bit is done move next bit into MSB position
  } // end of if for second half of Manchester coded bit
  
  MSTR_FRM_HalfBitCount--;    // update counter for frame half bits
  
  // in case we are in the middle of stop bit, disable bit trigger and prepare for receiving slave response
  if (MSTR_FRM_HalfBitCount == 0) {
	// bring OT line to low voltage level - end of transmission
	// since stop bit ends on low voltage it's ok to already end tramission now !! 
    digitalWrite(TxPin, HIGH);			
    
    // prepare for slave response timeout interrupt
    DisableMasterBitInterrupt();	// disable 2kHz transmit trigger
    OCR3A = INT3_INTERVAL_LONG;		// load T3 timer with 800ms count down value
    EnableMasterBitInterrupt();		// re-enable interrupt againg
    
    SLV_FRM_Frame = 0;		// clear receive buffer for reply expected from slave
    Rx_Buffer = 0;			// clear receive buffer for reply expected from slave
    Rx_Bit_Cnt = 0;			// clear Rx bit counter
    Rx_Intrpt_Cnt = 0;		// clear interrupt counter
	  Rx_Bit_One_Cnt = 0;	// clear counter for '1' bits (--> parity check) 
    // Rx_Edge_Cnt = 0;		
	// clear Rx edge counter
    FSM_State = Rx_Wait_StrtBt_Lead;    // FSM state: waiting for start bit leading/rising edge
    UnevaluatedReply = false;           // no uncomplete message from slave available
    // RxStopwatch.reset();    			// initialize time to monitor signal timing
    // RxStopwatch.start();
    EnableSlaveReceiveInterrupt();		// enable interupt on receiving changes on Rx pin 
	//tstMSTRFinishTX = micros();			// keep timestamp when master finished to transmit it's frame
  }
 
}	// **** end if bit transmitting interrupt service routine *****


/*
*********************************************************************
*********************************************************************
*
* Interrupt routine
* to receive replies from OpenTherm slave
*
* This interrupt is triggered by a transition on an input pin 
*
* the frame received is collected in 32bit Rx_buffer
* parity is tracked during receiving process (start and stop bit excluded)
*
*********************************************************************
*********************************************************************
*/ 
void Rx_trigger()
{
  timestamp = micros();                 // capture timestamp for further checks
  RxPin_Val = digitalRead(RxPin);       // capture signal level (=level following the transition that triggered the interrupt)  
                                        //       pin at low level: received a falling edge 
                                        //       pin at high level: received a rising edge
  Rx_TmStmp_IntSrv_Start = timestamp;
  Rx_Intrpt_Cnt++;                      // keep track on # of signal transitions   
  Rx_Timing_temp = (timestamp - Tx_TmStmp_Last_Int);
  if (Rx_Timing_temp > Rx_Timing_Bit_HlfBit_Fence) {
	  // verifing last interrupt was  about 1 lenth of a full bit ago
	  if (Rx_Intrpt_Cnt > 1)  Rx_Timing_Bit_Fence_Max = max(Rx_Timing_temp, Rx_Timing_Bit_Fence_Max);
	  Rx_Timing_Bit_Fence_Min = min(Rx_Timing_temp, Rx_Timing_Bit_Fence_Min);
  }
  else  {
	  // assuming last interrupt was  about 1 lenth of a full bit ago
	  Rx_Timing_HlfBit_Fence_Max = max(Rx_Timing_temp, Rx_Timing_HlfBit_Fence_Max);
	  Rx_Timing_HlfBit_Fence_Min = min(Rx_Timing_temp, Rx_Timing_HlfBit_Fence_Min);
  }
	  
  Tx_TmStmp_Last_Int = timestamp;		// keep timestamp of interupt (edge transition) to verify slave's timing

  switch (FSM_State) {
      case Rx_Wait_StrtBt_Lead:			// waiting for a start bit to begin and receiving a rising edge ---> start bit began 
        if (RxPin_Val == HIGH) 
        {
          FSM_State = Rx_Wait_StrtBt_Trns;	// now wait for mid start bit transition
		  Rx_Lst_Edge = timestamp;			// keep timestamp of leading edge start bit transition as reference for detecting mid-bit transition of startbit
		  Rx_StrtBit_Lead_Edge = timestamp;			// keep timestamp of leading edge start bit transition as reference for detecting mid-bit transition of startbit
		  // tstSLVStartTX = timestamp;		// keep timestamp when slave started to transmit it's reply

        }
        break;
      case Rx_Wait_StrtBt_Trns:          // if waiting for mid start bit transition and receiving a falling edge ---> start bit complete 
	  // check time passed since leading rising edge 
        if (RxPin_Val == LOW && ((timestamp - Rx_StrtBit_Lead_Edge) < Rx_Tmout_Slv_Hlf_Bit_Max) && ((timestamp - Rx_StrtBit_Lead_Edge) > Rx_Tmout_Slv_Hlf_Bit_Min) )
        {
          FSM_State = Rx_Wait_Rcv_Nxt_Edge;		// set status
          Rx_Lst_Bt_Trns = timestamp;			// keep timestamp of mid start bit transition; used as timing reference to detect next mid-bit transition
          //Rx_Buffer << 1;                 // add start bit to receive buffer
          Rx_Buffer = Rx_Buffer | (unsigned long) 1;      // add a "1" as LSB to receive buffer
          // Serial.print("Interrupt start bit transition edge received; timestamp: ");
          // Serial.println(timestamp);
        }
        break;          // end of case Rx_Wait_StrtBt_Trns:
      case Rx_Wait_Rcv_Nxt_Edge:          // waiting for next edge/bit transition 
/*
		// next is for debugging / statistics:
		// collect some information on variations within slave's timing
		if((timestamp - Rx_Lst_Edge) >= Rx_Timing_Bit_HlfBit_Fence) {
			// we assume this to have 
		}
*/		
		// filter for mid-bit transitions only and ignore transitions between bits
        if (((timestamp - Rx_Lst_Bt_Trns) < Rx_Tmout_Slv_Bit_Max) && ((timestamp - Rx_Lst_Bt_Trns) > Rx_Tmout_Slv_Bit_Min))            //  check for time since last bit transition
        {
  		  // check for time passed since last mid bit transition
		  // in case it's a valid mid bit transition store equivalent bit value in receiving buffer
  		  // otherwise forget the transition because it might be a tranission on bit border
          Rx_Bit_Cnt++;                          	// increment bit count; start bit is ignored here
          if (Rx_Bit_Cnt <= 32)          			// in case of any bit before the stop bit: add it to the receive buffer
          {
            FSM_State = Rx_Wait_Rcv_Nxt_Edge;      	// move bit into receive buffer
            Rx_Lst_Bt_Trns = timestamp;            	// update timestamp for last bit transition; used as timing reference to detect next mid-bit transition
            Rx_Buffer = Rx_Buffer << 1;            	// shift receive buffer 1 bit to the left
            if (RxPin_Val == LOW) 
      			{
      				Rx_Buffer = Rx_Buffer | 1;      // in case second half of bit is low voltage: add a "1" as LSB to receive buffer
      				Rx_Bit_One_Cnt++;				// count number of '1's for parity check
      			}													  // otherwise: leave it a "0" 
          }
          else
          {  
			// now we should be in the middle of the stop bit
			FSM_State = Rx_Frame_Cmplt;      		// all 34 bits received from slave 
			//tstSLVFinishTX = timestamp;			// keep timestamp when slave finished it's transmission
     		// check parity
   			if ( !(Rx_Bit_One_Cnt & (byte)1))	// check frame for an even number of '1's
   			{	// parity is even --> OK
   				FSM_State = Rx_Frame_Par_OK;
   				UnevaluatedReply = true;        // a complete message from slave available and worth to be evaluated
   			}
			else 
			{
				FSM_State = Rx_Err_Rspns_Parity;  
			}
            // prevent timer 3 to cause an slave response timeout interrupt (>800ms)
            DisableMasterBitInterrupt();
      		// deattach external interrupt
      		detachInterrupt(digitalPinToInterrupt(RxPin)); 
          }  // end of if (Rx_Bit_Cnt < 33) 
        }  // end of if: check timestamp
		else 
		{
			// signal timing (mid-bit -> mid-bit) violation
			// ignore half-bit transitions
			if (Rx_Timing_temp > Rx_Timing_Bit_HlfBit_Fence) Rx_Timing_Viol_Cnt++;
		}
        break;      // end of case Rx_Wait_Rcv_Nxt_Edge:
      default:
        break;     // end of default:
  }    // end of switch
  Rx_TmStmp_IntSrv_End = micros();
  Rx_Duration_IntService = (Rx_TmStmp_IntSrv_Start - Rx_TmStmp_IntSrv_End);
}  // end of interrupt routine




/*----------------------------------------------------------------------------
  Function to analyze frame received from OT slave device
  output: return code of the analysis
  0: OK (reply valid)
  1: invalid or uncomplete reply (e.g. wrong parity, incomplete # of bits, timeout etc.)
  2: Slave did not accept read command
  3: Slave did not accept write command
  4: invalid data
  5: unknown command (=data ID)
 *----------------------------------------------------------------------------*/
int Analyze_Frame() {
unsigned int  _msg = 0;       // Message type
unsigned int _data_id = 0;    // Data ID
unsigned int _value = 0;      // Value field (16bit)
unsigned int _value_HB = 0; // value field, high byte
unsigned int _value_LB = 0; // value field, low byte
int retval = 0;				// temp var to hold return value
  
  // start extracting frame components
  _msg      = (Rx_Buffer & OT_FRM_MSGTYPE_MASK) >> OT_FRM_MSGTYPE_MASK_SHIFT;
  _data_id  = (Rx_Buffer & OT_FRM_DATAID_MASK) >> OT_FRM_DATAID_MASK_SHIFT;
  _value    = (Rx_Buffer & OT_FRM_VALUE_MASK) >> OT_FRM_VALUE_MASK_SHIFT;
  // pc.printf("Analyzed frame: msg-type:%d, data-id:%d, value:%d\n", _msg, _data_id, _value );
  
  // check slave's reply 
  switch (_msg) {
    case SLV_RD_ACK :           // read command accepted
		// in case data were read successfully update buffer with new value ...
		MonitorBuffer[MB_pointer].r_type = _msg;
        MonitorBuffer[MB_pointer].r_id = _data_id;
		// check if value changed
		if (MonitorBuffer[MB_pointer].r_value != _value) MonitorBuffer[MB_pointer].changed = true;
        MonitorBuffer[MB_pointer].r_value = _value;
		// TODO: micros() have to be replaced by real time stamp (e.g. from RTC or NTP server)
		MonitorBuffer[MB_pointer].tmstp_rec = micros();
		// ... and reset error counter
		MonitorBuffer[MB_pointer].err_cnt = 0;
		retval = 0;
		LastDialogueIsError = false;
		break;		// end of case SLV_RD_ACK
    case SLV_WRT_ACK :            // write command accepted
  		LastDialogueIsError = false;
		  retval = 0;
		break;
    case SLV_ID_INVLD :            // slave received invalid message ID
  		// ... and increment error counter
  		MonitorBuffer[MB_pointer].err_cnt++;
  		Serial.print("ERROR: SLV_ID_INVLD; _msg_type=");
  		Serial.print(_msg,BIN);
  		Serial.print("; _data_id=");
  		Serial.print(_data_id,BIN);
  		Serial.print("; _value=");
  		Serial.print(_value,BIN);
  		Serial.print("; err_cnt=");
  		Serial.print(MonitorBuffer[MB_pointer].err_cnt);
  		Serial.print("; MB_pointer=");
  		Serial.println(MB_pointer);
  		LastDialogueIsError = true;
  		retval = 0;
		break;
    case SLV_DTA_INVLD :            // slave received invalid data
		// ... and increment error counter
      MonitorBuffer[MB_pointer].err_cnt++;
  		Serial.print("ERROR: SLV_DTA_INVLD; _msg_type=");
  		Serial.print(_msg,BIN);
  		Serial.print("; _data_id=");
  		Serial.print(_data_id,BIN);
  		Serial.print("; _value=");
  		Serial.print(_value,BIN);
  		Serial.print("; err_cnt=");
  		Serial.print(MonitorBuffer[MB_pointer].err_cnt);
  		Serial.print("; MB_pointer=");
  		Serial.println(MB_pointer);
  		LastDialogueIsError = true;
  		retval = 0;
		break;
    default :
      
    break;
  }		// end of switch (_msg)
  
  return(LastDialogueIsError);
}

void printOTValues() 
{
	for (int i=0; i < MB_length; i++)
	{
		Serial.print(i);
		Serial.print(": ");
		Serial.print((char *)MonitorBuffer[i].par_name);
		Serial.print(": \t");
		if (i < 3) 
		{
			Serial.print(MonitorBuffer[i].r_value, BIN);
		}
		else
		{
			printDouble( DataID2Float(MonitorBuffer[i].r_value), 100);			
		}
		Serial.print("; \terr_cnt=");
		Serial.print(MonitorBuffer[i].err_cnt);
		Serial.print("; \ttime=");
		Serial.print(MonitorBuffer[i].tmstp_rec);

		Serial.println("");
	}
}


