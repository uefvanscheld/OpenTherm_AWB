/*
Edit history:
V 0.204:  changed pin activation sequence in setup();
          for temp debugging polling of a control pin was added to stop Arduino from sending (used to learn about behaviour of OT heating unit)
V 0.202:  Extended status definitions for Master to be sent to slave
          Modified MQTT topic structure: moved label into topic, 
V 0.201:  Added timeout for reply from slave
          Added initial basic MQTT support (just just connect and publish)
V 0.200:  New start based on MEGA256 and interrupts; some code taken from Freescale board project;
          MEGA is used to have more ressources for an integration with FHEM (protocols are still tbd)
V 0.104:  additional code rewriting to use just one time interrupt
V 0.103:  restarted coding to get rid of old code fragments no longer needed
V 0.102:  started adding OpenTherm.h 
V 0.101:  modified to support interrupts for master sending messages to slave
V 0.100:  added code for send and receive buffer
V 0.82: code hanged with first interrupt so first step is to remove all serial.print from interrupt routine
V 0.8:  started implementing receive routines; increased speed of serial communication for better debugging
V 0.7:  transmitting a standard frame seems to work; not always receiving a reply from heating
V 0.6:  added parity bit routine and improve transmit routine

*/

#include <Arduino.h>

// next is for Ethernet support
#include <SPI.h>
#include <Ethernet.h>

#include <PubSubClient.h>     // add MQTT support

#include "OpenTherm.h"        // add OpenTherm support
#include "Includes.h"         // add some general stuff

//#include <QueueArray.h>		  // DISBALED FOR NOW: include queue library header
#include <avr/interrupt.h> 	  // used for interrupt programming
#include <avr/io.h> 		      // used for interrupt programming

// next is required to print binary values with leading zeroes using specific routines
// char* pBinFill(long int x,char *so, char fillChar); // version with fill
// char* pBin(long int x, char *so);                    // version without fill
#define width 64
char so[width+1]; // working buffer for pBin

// Update these with values suitable for your hardware/network.
byte mac[]    = {  0x90, 0xA2, 0xDA, 0x0E, 0xBE, 0x99 };    // MAC address of this Arduino Ethernet client (see ethernet shield)
IPAddress ip(192, 168, 0, 9);                               // IP address assigned to this Arduino Ethernet client
IPAddress mqtt_server(192, 168, 0, 8);                           // IP address of MQTT broker in your LAN


// define a buffer for the message currently sent by the master
// may be also used for resending a command in case an error occurred (e.g. no reply, error message, incomplete reply)
//OT_message Tx_message;
// also define a buffer for the message received from the slave
// keeps data until is interpreted by the master; protected against updates by a semaphore
//OT_message Rx_message;
//boolean Rx_message_locked = false;	// received message can be updated

// configure the transmit queue
//volatile QueueArray <OT_message> Tx_queue;			// create a queue of OT_messages for master to transmit; use volatile to make accessible for an interrupt routine
//volatile int Tx_queue_elements = 0;					// contains the number of items in the transmit queue
//volatile int Tx_queue_elements_MAX = 25;			// contains the MAXIMUM number of items in the transmit queue; value may be subject to change 

// initalize Ethernet client
EthernetClient ethClient;
PubSubClient mqtt_client(ethClient);

long now = 0;

boolean reconnect() {
	Serial.print("MQTT: TRY reconnect .....");
	Serial.println("");

  if (mqtt_client.connect("OpenThermClient")) {
    // Once connected, publish an announcement...
	Serial.print("MQTT: RECONNECTED .....");
	Serial.println("");
    mqtt_client.publish("outTopic","hello world");
    // ... and resubscribe
    // mqtt_client.subscribe("inTopic");
  }
  return mqtt_client.connected();
}

void setup()
{
  pinMode(LED, OUTPUT);

  
  mqtt_client.setServer(mqtt_server, 1883);	
  // mqtt_client.setCallback(callback);				// callback not needed so far

  Ethernet.begin(mac, ip);
  delay(1500);
  mqtt_lastReconnectAttempt = 0;
  
  // initialize the serial communications:
  Serial.begin(115200);
  
  // next pin is used to control the behaviour of the transmit line
  pinMode(CtrlPin, INPUT_PULLUP);        // define pin 5 as input

  pinMode(CH_Relais, OUTPUT);
  digitalWrite(CH_Relais, LOW);     // turn relais off --> disconnect interface from CH
	Serial.print(millis());
	Serial.println(": Relais off .6..");
  delay(1000);                  	// wait for a second
  
  pinMode(TxPin, OUTPUT);       // transmit pin works inverted
	Serial.print(millis());
  digitalWrite(TxPin, HIGH);    // bring OT voltage to idle level (< 7V)
	Serial.println(": TxPin is Output ...");
  delay(1000);             		// waits for a second
  
  pinMode(RxPin, INPUT);        // define receive pin as input
	Serial.print(millis());
	Serial.println(": RxPin is Input ..10.");
  delay(1000);           		// waits for half a second
  
  digitalWrite(CH_Relais, HIGH);    // turn relais on --> connect interface to CH
	Serial.print(millis());
	Serial.println(": Relais on ...");
  delay(1000);                  	// wait for 2 second2

  while (digitalRead(CtrlPin) == LOW) {     // stop here in case control pin is low (has internal PULL_UP ;chance to observe OT heating unit)
  }


  // prepare for loop to start with initial master request
  Rx_TmStmp_FrmStrt = micros();
  FSM_State = Rx_Frame_Cmplt;      // assume last reply from slave was received successfully
  Err_State = 0;    
  Rx_Buffer = 0;
  
  // cli(); // disable all interrupts during setup
  initialize_interrupts ();			  // configure and initialize timer1 and timer2 interrupts for master transmissions 

  // prevent signals on receiver pin to cause an interrupt while not waiting for an slave reply 
  DisableSlaveReceiveInterrupt();

  // let timer start to send first frame to slave
  // this will cause the first timer interrupt to occur after 1 sec in main loop
  EnableMasterFrameInterrupt();   
  sei();    // enable global interrupts, use cli() to disable them
			
}

/*
*********************************************************************
*********************************************************************
*
*    MAIN LOOP STARTS HERE
*
*********************************************************************
*********************************************************************
*/
void loop()
{
	// just wait for replies from OT slave device
  timestamp2 = timestamp1;
  switch (FSM_State) {

      case Rx_Wait_StrtBt_Lead:               // when waitung for response from slave print frame sent by master
      Serial.print(millis());
      Serial.print(":  MB_pointer:");
      Serial.print(MB_pointer);
      Serial.print(";  FSM_State:");
      Serial.print(FSM_State);
      Serial.print(";  Master frame:0b");
//      Serial.print( (unsigned long) MSTR_FRM_Frame_Copy, BIN);
      Serial.print(pBinFill(MSTR_FRM_Frame_Copy,so,0));
      Serial.println("");
      break;
    
      case Rx_Err_Tmout_StrtBt:
      case Rx_Err_Tmout_StrtBt_Trns:
      case Rx_Err_Tmout_Bit:
			Serial.print(millis());
			Serial.print(":  TIMEOUT ERROR - FSM_State:");
			Serial.print(FSM_State);
			Serial.print("; Master_Frame_Cnt:");
			Serial.print(Master_Frame_Cnt);
			Serial.print(":  MB_pointer:");
			Serial.print(MB_pointer);
			Serial.print(";  Rx_Bit_Cnt:");
			Serial.print(Rx_Bit_Cnt);
			Serial.print(";  Rx_Buffer:");
			Serial.print( (unsigned long) Rx_Buffer, BIN);
      Serial.println("");
      break;
      
      case Rx_Err_Rspns_Parity:
          Serial.print("PARITY ERROR - FSM_State:");
          Serial.print(FSM_State);
			Serial.print(";  Rx_Bit_Cnt:");
			Serial.print(Rx_Bit_Cnt);
          Serial.println("");
      break;
      
      case Rx_Frame_Par_OK:          // if slave sent a VALID complete answer (=34bits and parity=OK) ....
    		if (UnevaluatedReply) 			// in case the reply was not analyzed yet
    		{
			ReplyState = Analyze_Frame();		// check reply
			UnevaluatedReply = false;
			if (ReplyState) 					// in case of an error within slave's response:
			{
				FSM_State = Rx_Err_Rspns_Invalid;	// update status
			}
			else Slave_Frame_Cnt++;				// otherwise count successful slave messages
			
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
			
			printOTValues();					// log current values
			Serial.println("");
		}
        break;    // else do nothing and wait
      
      case Rx_Frame_Cmplt:          // if slave sent a complete (!) answer (=34bits) ....
		// FSM_State = Rx_Err_Rspns_Invalid;	
      Serial.print(millis());
      Serial.print(":  Slave reply OK - FSM_State:");
      Serial.print(FSM_State);
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
      Serial.println("");
        break;    // else do nothing and wait
      
      case Rx_Err_Rspns_Invalid:    // in case of an error output error condition and some other information and reset FSM_State
          Serial.print("ERROR - FSM_State:");
          Serial.print(FSM_State);
          Serial.print(";  Err_State:");
          Serial.print(Err_State);
          Serial.print(";  Rx_Buffer:");
          Serial.print((unsigned long) Rx_Buffer, BIN);
          Serial.print(";  Rx_Bit_Cnt:");
          Serial.println(Rx_Bit_Cnt);
          // FSM_State = Rx_Frame_Cmplt;  // and reset FSM state
          Err_State = 0;    
          break;    // else do nothing and wait      

      default:
        break;
    }   // end of switch
	
	/*
	Serial.print(millis());
	Serial.print(":  MB_pointer:");
	Serial.print(MB_pointer);
	Serial.print(";  FSM_State:");
	Serial.print(FSM_State);
	Serial.print(";   MST_Bit_Int_Cnt:");
	Serial.print(MST_Bit_Int_Cnt);
	Serial.print("; MST_Frame_Int_Cnt:");
	Serial.print(MST_Frame_Int_Cnt);
	Serial.print(";  Err_State:");
	Serial.print(Err_State);
	Serial.print(";  Rx_Bit_Cnt:");
	Serial.print(Rx_Bit_Cnt);
	Serial.print(";  Rx_Buffer:");
	Serial.print( (unsigned long) Rx_Buffer, BIN);
	Serial.println("");
	*/

	
	// now let's handle MQTT stuff
	now = millis();
	if (!mqtt_client.connected()) 		// check if client is still connected to broker
	{
		if (now - mqtt_lastReconnectAttempt > 5000) 			// try reconnect every 5 secs
		{
			Serial.print("MQTT: NOT connected .....");			
			Serial.println("");
			mqtt_lastReconnectAttempt = now;					// update timestamp for reconnect try
			// Attempt to reconnect
			if (reconnect()) 									// try to reconnect
			{
				mqtt_lastReconnectAttempt = 0;
			}
		}
	} 
	else 
	{
    	// Client connected .... 
		// publish annother topic
		// Serial.print("MQTT: client connected .....");
		// Serial.println("");

		if (now - mqtt_lastPublishAttempt > 200) 	// publish max 5 messages / second
		{
			if (mqtt_build_next_message(MQTT_pub_topic_buffer, MQTT_pub_payload_buffer)) 	// in case an updated/changed parameter needs to get published
			{	
				mqtt_lastPublishAttempt = now;												// update publishing time stamp
				mqtt_client.publish(MQTT_pub_topic_buffer, MQTT_pub_payload_buffer);		// publish new value
				Serial.print("MQTT: published topic:<");
				Serial.print(MQTT_pub_topic_buffer);
				Serial.print(">\t  payload:<");
				Serial.print(MQTT_pub_payload_buffer);
				Serial.println(">");
			}	// end of if mqtt_build_next_message
		}	// end of if > 200ms
		// let him ask for new messages subscribed for ....
		mqtt_client.loop();
	}
}  // end of main loop



/*
*********************************************************************
*********************************************************************
*
*    Code related to send from master to slave
*
*********************************************************************
*********************************************************************
*/

/*
----------------------------------------------------------------------------
  Handler routine used to trigger new OT frame to be sent from master to contact slave
 ----------------------------------------------------------------------------
*/
ISR(TIMER4_COMPA_vect) {
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
	
	// find next parameter to be requested based on assigned priority
	prio_index = 0;		// in each call start with highest prio to see if there are parameters left to be requested 
	MB_pointer = -1;	//
  do 
  {
    if (prio_pointer[prio_index] == MB_length)   // first check and handle wrap around
    {
      prio_pointer[prio_index] = 0;
      j = 0;
    }
    else j = ++prio_pointer[prio_index];		// otherwise increment prio's index to next buffer's element
/*
	Serial.print(millis());
      Serial.print(":  x:");
      Serial.print(x);
      Serial.print(";  j:");
      Serial.print(j);
      Serial.print(";  MB_pointer:");
      Serial.print(MB_pointer);
      Serial.print(";  prio_index:");
      Serial.print(prio_index);
      Serial.print(";   prio_pointer[prio_index]:");
      Serial.print(prio_pointer[prio_index]);
      Serial.print("; MonitorBuffer[j].s_prio:");
      Serial.print(MonitorBuffer[j].s_prio);
      Serial.print("; MB_length:");
      Serial.print(MB_length);
      Serial.println("");
*/
    if (j == MB_length)         // search for entries of prio = prio_index reached end of buffer so all parameter of this priority have been requested
    {
      prio_index = (++prio_index) % prio_num;         // jump to next lower priority (or back to highest when end is reached)
/*      Serial.print("reached end of buffer");
      Serial.println("");
*/      continue;                       // continue with search within next priority
    }
    else
    {   // check for right priority
      if (MonitorBuffer[j].s_prio == prio_index)        // in case we found a matching entry ...
      {
        MB_pointer = j;         // ... hand it over to Master frame sending routine .....
/*
      Serial.print("MATCH @ ");
	  Serial.print(millis());
      Serial.print(":  x:");
      Serial.print(x);
      Serial.print(";  j:");
      Serial.print(j);
      Serial.print(";  MB_pointer:");
      Serial.print(MB_pointer);
      Serial.print(";  prio_index:");
      Serial.print(prio_index);
      Serial.print(";   prio_pointer[prio_index]:");
      Serial.print(prio_pointer[prio_index]);
      Serial.print("; MonitorBuffer[j].s_prio:");
      Serial.print(MonitorBuffer[j].s_prio);
      Serial.print("; MB_length:");
      Serial.print(MB_length);
      
      Serial.println("");
*/
      }
      prio_pointer[prio_index] = j;   // .. and update the related pointer
    }
  } 
  while (MB_pointer == -1);    // stop loop in case a pointer to next parameter was found
  
  
  LastDialogueIsError = 0;  // reset communication status for this try

  // pick up data for next frame from prepared message buffer.... let pointer work on RoundRobin
  MB_pointer = MB_pointer % MB_length;
  
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
  MSTR_FRM_HalfBitCount = MSTR_FRM_HalfBitLength;		// set counter for half bits
  FSM_State = 0;
  OCR3A = INT3_INTERVAL_SHORT;          // ... and preload timer again 2kHz interrupt value for next dialogue
  EnableMasterBitInterrupt();							// initialize the bit sending interrupt
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
*   transmit frame bits from master to slave
*	this is the interrupt service routine for timer 3 in CTC mode
*
*********************************************************************
*/
ISR(TIMER3_COMPA_vect)
{
	// this routine handles two types of interrupts:
	// a) trigger to send new bit of frame to slave
  // b) timeout for receiving a response from slave

  // first check for slave response timeout interrupt
  
  if (FSM_State >= Rx_Wait_StrtBt_Lead)
  {
    FSM_State = Rx_Err_Tmout_StrtBt;      // update status
    MonitorBuffer[MB_pointer].err_cnt++;  // increment error counter for this dialogue
    DisableMasterBitInterrupt();          // disable  trigger
    return;   // leave ISR
  }
  
  
  // 
  MST_Bit_Int_Cnt++;		// track # of bit interrupts

  // if a new bit begins get the bit value 
  if (!(MSTR_FRM_HalfBitCount & 1)) {         // if counter is even get new bit value
        // read next bit to be sent from frame buffer starting from the MSB end of the frame
        if (MSTR_FRM_Frame & MSTR_FRM_SendBit_Mask) {
            MSTR_FRM_SendBit = 1;
		// IMPORTANT: observe INVERTING behaviour of hardware interface between Arduino and OT's side !!!
            // TxSetHigh;      // in case of a '1' to send start new bit with high voltage level 
			digitalWrite(TxPin, LOW);        // in case of a '1' to send start new bit with high voltage level and bring OT voltage to high level (> 15V)
        }
        else {
            MSTR_FRM_SendBit = 0;
            // TxSetLow;     // in case of a '0' to send start new bit with low voltage level 
			digitalWrite(TxPin, HIGH);       // in case of a '0' to send start new bit with low voltage level and bring OT voltage to idle level (< 7V)
        }   // end of if for first half of Manchester coded bit
  }
  else     {    // // for second half of Manchester coded bit
        // TxToggle;     // for second half of the bit just toggle the output voltage level
		digitalWrite(TxPin, !digitalRead(TxPin));  // toggle transmit pin
        MSTR_FRM_Frame = MSTR_FRM_Frame << 1;  // when bit is done move next bit into MSB position
  } // end of if for second half of Manchester coded bit
  
  MSTR_FRM_HalfBitCount--;    // update counter for frame half bits
  
  // in case we are in the middle of stop bit, disable bit trigger and prepare receiving slave response
  if (MSTR_FRM_HalfBitCount == 0) {
	// bring OT line to low voltage level - end of transmission
	// since stop bit ends on low voltage it's ok to already end tramission now !! 
    digitalWrite(TxPin, HIGH);			
    
    // prepare for slave response timeout interrupt
    DisableMasterBitInterrupt();        // disable 2kHz transmit trigger
    OCR3A = INT3_INTERVAL_LONG;        // load timer with 800ms count down value
    EnableMasterBitInterrupt();        // re-enable interrupt againg
    
    SLV_FRM_Frame = 0;  // clear receive buffer for reply expected from slave
    Rx_Buffer = 0;  // clear receive buffer for reply expected from slave
    Rx_Bit_Cnt = 0;     // clear Rx bit counter
    Rx_Intrpt_Cnt = 0;	// clear interrupt counter
	  Rx_Bit_One_Cnt = 0;	// clear counter for '1' bits (--> parity check) 
    // Rx_Edge_Cnt = 0;	// clear Rx edge counter
    FSM_State = Rx_Wait_StrtBt_Lead;    // FSM state: waiting for start bit leading/rising edge
    UnevaluatedReply = false;           // no uncomplete message from slave available
    EDGE_FRM_Frame = 0;
    // RxStopwatch.reset();    // initialize time to monitor signal timing
    //RxStopwatch.start();
    EnableSlaveReceiveInterrupt();       // enable interupt on receiving changes on Rx pin 
  }

}	// **** end if bit transmitting interrupt service routine *****


/*
*********************************************************************
*********************************************************************
*
* Interrupt routine
* to receive replies from Open therm slave
*
* data received is collected in 32bit Rx_buffer
* parity is tracked during receiving process (start and stop bit excluded)
*
*********************************************************************
*********************************************************************
*/ 
void Rx_trigger()
{
  timestamp = micros();                  // capture timestamp for further checks
  RxPin_Val = digitalRead(RxPin);        // capture signal level that caused the interrupt
                                         //       pin at low level: received a falling edge 
                                         //       pin at high level: received a rising edge
  // RxPin_Val_Sum = RxPin_Val_Sum + RxPin_Val;	// count number of rising edges 
  Rx_Intrpt_Cnt++;                                         

  switch (FSM_State) {
      case Rx_Wait_StrtBt_Lead:          // if waiting for a start bit to begin and receiving a rising edge ---> start bit began 
        if (RxPin_Val == HIGH) 
        {
          FSM_State = Rx_Wait_StrtBt_Trns;    // now wait for mid start bit transition
		  Rx_Lst_Edge = timestamp;			// keep timestamp of leading edge transition
        }
        break;
      case Rx_Wait_StrtBt_Trns:          // if waiting for mid start bit transition and receiving a falling edge ---> start bit complete 
	  // check time passed since leading rising edge 
        if (RxPin_Val == LOW && ((timestamp - Rx_Lst_Edge) < Rx_Tmout_Slv_Hlf_Bit_Max) && ((timestamp - Rx_Lst_Edge) > Rx_Tmout_Slv_Hlf_Bit_Min) )
        {
          FSM_State = Rx_Wait_Rcv_Nxt_Edge;		// set status
          Rx_Lst_Bt_Trns = timestamp;			// keep timestamp of mid start bit transition
          //Rx_Buffer << 1;                 // add start bit to receive buffer
          Rx_Buffer = Rx_Buffer | (unsigned long) 1;      // add a "1" as LSB to receive buffer
          // Serial.print("Interrupt start bit transition edge received; timestamp: ");
          // Serial.println(timestamp);
        }
        break;          // end of case Rx_Wait_StrtBt_Lead:
      case Rx_Wait_Rcv_Nxt_Edge:          // waiting for next edge/bit transition 
        if (((timestamp - Rx_Lst_Bt_Trns) < Rx_Tmout_Slv_Bit_Max) && ((timestamp - Rx_Lst_Bt_Trns) > Rx_Tmout_Slv_Bit_Min))            //  check for time since last bit transition
        {
  		  // check for time passed since last mid bit transition
        // in case it's a valid mid bit transition store equivalent bit value in receiving buffer
  		  // otherwise forget the transition because it might be a tranission on bit border
          Rx_Bit_Cnt++;                          // increment bit count; start bit is ignored here
          if (Rx_Bit_Cnt <= 32)          // in case of any bit before the stop bit: add it to the receive buffer
          {
            FSM_State = Rx_Wait_Rcv_Nxt_Edge;      // move bit into receive buffer
            Rx_Lst_Bt_Trns = timestamp;            // update timestamp for last bit transition
            Rx_Buffer = Rx_Buffer << 1;                           // shift receive buffer 1 bit to the left
            if (RxPin_Val == LOW) 
      			{
      				Rx_Buffer = Rx_Buffer | 1;      // in case second half of bit is low voltage: add a "1" as LSB to receive buffer
      				Rx_Bit_One_Cnt++;				// count number of '1's for parity check
      			}													  // otherwise: leave it a "0" 
          }
          else
    		  // now we are in the middle of the stop bit
    		  // otherwise check for a '1' on the stop bit position and check parity
          {  
            FSM_State = Rx_Frame_Cmplt;      // all 34 bits received from slave 
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
        break;      // end of case Rx_Wait_Rcv_Nxt_Edge:
      default:
        break;     // end of default:
  }    // end of switch
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
  
  return(0);
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




/*
*********************************************************************
*********************************************************************
 some service  & setup functions
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
        //   set Timer Prescaler in Output Compare Register B to 64: CS22=1, CS21=0, CS20=0
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
			dtostrf((float)MonitorBuffer[MQTT_MB_pointer].r_value / 256, 1, 2, fl_buffer);    // use dtostrf function to get better formatting options

      // sprintf(payload,"%s: %s",MonitorBuffer[MQTT_MB_pointer].par_name, fl_buffer);    // old version building payload from label AND value
			sprintf(payload,"%s",fl_buffer);                                                    // new version with V 0.202: payload only consists of value, label is part of topic
			MonitorBuffer[MQTT_MB_pointer].changed = false;		// track that value was published
			return true;										// and leave function indicating that a new 
		}		// end of if
	}		// end of for loop
	return false ;
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
float DataID2Float(unsigned int data_val)
{
 return ((float) (data_val / 256));  
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
 char s[width+1];
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
