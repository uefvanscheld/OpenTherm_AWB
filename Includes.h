#ifndef Includes_h
#define Includes_h

#include <Arduino.h>		// required to make sure all data types are defined

//  define constants to control receive procedures
//  all numbers are defined in micro seconds
;
const unsigned long Tx_Tmout_Lst_Mstr_Rqst = 1100000;  // maximum time between two requests sent from Master is 1.1 sec  [us]

const unsigned long Rx_Dur_Bit = 1000;            		  // nominal time between two bit transitions [us]
const unsigned long Rx_Tmout_Slv_Rrpl_Min = 20000;        // minimum time within the slave have to start to reply to a masters request; measured since last bit sent by master
const unsigned long Rx_Tmout_Slv_Rrpl_Max = 800000;       // maximum time within the slave have to start to reply to a masters request; measured since last bit sent by master
const unsigned long Rx_Tmout_Slv_Rrpl_End = 800000;      // maximum time within the slave's response have to be completed [us]
const unsigned long Rx_Tmout_Slv_Bit_Max = 1150; 		// maximum time within the slave is expected to send next bit [us]
const unsigned long Rx_Tmout_Slv_Bit_Min = 900;       // minimum time within the slave is expected to send next bit
// next is only used for detemination of start bit
const unsigned long Rx_Tmout_Slv_Hlf_Bit_Max = 650; 		// maximum time within the slave is expected to send next mid bit transition after previous between bit transition  [us]
const unsigned long Rx_Tmout_Slv_Hlf_Bit_Min = 400;       // minimum time within the slave is expected to send next mid bit transition after previous between bit transition  [us]

// ********************************************
// define states for FSM
// ********************************************
const unsigned long Rx_Wait_StrtBt_Lead = 10;     // waiting for receiving leading rising edge of start bit from slave
const byte Rx_Wait_StrtBt_Trns = 20;     // waiting for falling transition edge in mid of start bit
const byte Rx_StrtBt_Cmplt = 30;         // start bit received successfully
const byte Rx_Wait_Rcv_Nxt_Edge = 40;     // waiting for receiving next edge/trigger/interrupt of next bit
const byte Rx_Edge_Rcvd = 50;             // edge received (somewhere within slave frame)
const byte Rx_Frame_Cmplt = 60;           // 32 bits (=frame) received completely
const byte Rx_Frame_Par_OK = 70;           // received frame passed parity check successfully

const byte Rx_Err_Tmout_StrtBt = 80;     	// waiting for start bit timed out
const byte Rx_Err_Tmout_StrtBt_Trns = 81; // waiting for slave response to begin falling transition edge of start bit
const byte Rx_Err_Tmout_Bit = 82;     		// waiting for next bit timed out

const byte Rx_Err_Rspns_Invalid = 90;		  // slave response invalid
const byte Rx_Err_Rspns_Parity = 91;      // slave response had parity error

// configure input and output pins
byte LED = 13; 					// The blue onboard LED
volatile byte CH_Relais = 7;  // output which controls the relais placed in boiler/central heating
volatile byte TxPin = 6;  		// output pin for transmit signal
volatile byte RxPin = 3;		// input pin for receive signal
byte CtrlPin = 5;    // input pin for receive signal

// next variables are used for debugging timing and FSM issues
// may be obsolete in future releases
byte tx_status = 0;
volatile byte FSM_State = 0;
volatile int FSM_State_10_Cnt_1 = 0;
volatile int FSM_State_10_Cnt_2 = 0;
volatile int FSM_State_20_Cnt_1 = 0;
volatile int FSM_State_20_Cnt_2 = 0;
volatile int FSM_State_40_Cnt_1 = 0;
volatile int FSM_State_40_Cnt_2 = 0;
volatile int FSM_State_40_Cnt_High = 0;
volatile int FSM_State_40_Cnt_Low = 0;
volatile byte Err_State = 0;


// ********************************************
// define variables for receiving procedures
// ********************************************
volatile int RxPin_Val;					// hold the level of the Rx pin when in ISR
//volatile int RxPin_Val_Sum;				
//volatile int state = LOW;
volatile unsigned long Rx_Buffer = 0;   //  32bit buffer for bits received
volatile byte  Rx_Bit_Cnt = 0;               //  counter for bits received
volatile byte  Rx_Bit_One_Cnt = 0;			// counter for '1's received from slave to keep track on parity
volatile byte  Rx_Parity_Bit = 0;			// hold received parity bit
volatile int   Rx_Intrpt_Cnt = 0;            //  counter for interrupts received
int ReplyState = 0;						// holds result of analysis of slave response

volatile boolean UnevaluatedReply = false;	// to control whether a new message has to be analyzed
boolean LastDialogueIsError = false;    	// indicates whether last communication between master and slave was successful
unsigned long LastDialogueErrorCnt = 0;		// count number of errors during communicaion with slave

volatile unsigned long timestamp = 0;		// used to track time since last signal edge received from slave
volatile unsigned long Rx_Lst_Bt_Trns = 0;   // time stamp when last bit transition happened; used to filter bit transitions from transitions happening between bits
volatile unsigned long Rx_Lst_Edge = 0;      // time stamp when last edge happened; used to filter bit transitions from transitions happening between bits
volatile unsigned long Rx_TmStmp_FrmStrt = 0;// time stamp when frame began (begin of start bit)
volatile unsigned long Tx_TmStmp_Mstr_Reqst_Snt = 0;// time stamp when sending last Master request was completed (end of stop bit)


unsigned long Tx_frame_MSB_mask = 2147483648;  // value to mask the MSB bit of the 32bit OT data frame during sending
unsigned long Tx_frame_PAR_mask = 2147483648;  // value to mask the parity bit of the 32bit OT data frame before sending



/*  --------------------------------------------------------------------------------
// some variables to serve the master sending service routines and to handle the frame to be sent
// count which bit of the frame is currently processed
 *----------------------------------------------------------------------------*/
volatile unsigned long MTIntCount = 0;					// Master Transmit interrupt count; tracks number of frames transmitted
volatile byte MSTR_FRM_HalfBitLength = 68;    // 32 data bits + 1 start + 1 stop = 34; multiply by 2 for Manchester coding      
volatile byte MSTR_FRM_HalfBitCount = 68;   // 32 data bits + 1 start + 1 stop = 34; multiply by 2 for Manchester coding      
// frame to be sent 
volatile unsigned long long MSTR_FRM_Frame = 0;
volatile unsigned long long MSTR_FRM_Frame_Copy = 0;
volatile unsigned long long SLV_FRM_Frame = 0;
volatile unsigned long long EDGE_FRM_Frame = 0;
// bit mask for start and stop bits
unsigned long long MSTR_FRM_STRTSTP_Bit_Mask = (((unsigned long long)1ull)<<33) | ((unsigned long long)1ull);
// the bit value to be sent
volatile byte MSTR_FRM_SendBit = 0;       
// the bit mask to extract the MSB bit of the frame to be sent
volatile unsigned long long MSTR_FRM_SendBit_Mask = ((long long)1ull) << 33;

// for the next we assume to deal just with the relevant 32 'data' bits (without start and stop bits) 
short             OT_FRM_MSGTYPE_MASK_SHIFT = 28;
unsigned long OT_FRM_MSGTYPE_MASK = ((unsigned long) 0x7) << OT_FRM_MSGTYPE_MASK_SHIFT;   // mask to get message type
short             OT_FRM_DATAID_MASK_SHIFT = 16;
unsigned long OT_FRM_DATAID_MASK = ((unsigned long) 0xFF) << OT_FRM_DATAID_MASK_SHIFT;    // mask to get data id
short             OT_FRM_VALUE_MASK_SHIFT = 0;
unsigned long OT_FRM_VALUE_MASK = ((unsigned long) 0xFFFF) << OT_FRM_VALUE_MASK_SHIFT;    // mask to get value field

/*  --------------------------------------------------------------------------------
// some variables to general debugging and statistics
 *----------------------------------------------------------------------------*/
// these variables are used for debugging timing issues with interrupts
// may be obsolete in future releases
volatile unsigned long timestamp0 = 0;
volatile unsigned long timestamp1 = 0;
volatile unsigned long timestamp2 = 0;
volatile unsigned long timestamp3 = 0;
volatile unsigned long timestamp4 = 0;
volatile unsigned long resp_delay = 0;

volatile unsigned long MST_Bit_Int_Cnt = 0;		// track number of Master bit interrupts
volatile unsigned long MST_Frame_Int_Cnt = 0;		// track number of Master frame interrupts

volatile unsigned long Master_Frame_Cnt = 0;		// counts number of messages Master sent to slave
volatile unsigned long Slave_Frame_Cnt = 0;			// counts number of successful messages Master received from slave (error statistics)

// some variables to serve the ISR and to handle the frame to be sent
volatile byte MSTR_FRM_BitCount = 0;			// count which bit of the frame is currently processed
volatile byte MSTR_FRM_BitISRCount = 0;			// track how often interrupt was called; purpose: track half bits for Manchester coding

/*  --------------------------------------------------------------------------------
// some definitions for MQTT
 *----------------------------------------------------------------------------*/
char	MQTT_pub_topic_root[] = "myhome/opentherm/";	// most upper level(s) of all topics published with this client; HAS TO END WITH A SLASH '/'
char	MQTT_sub_topic_root[] = "";						// most upper level(s) of all topics subscribed with this client; HAS TO END WITH A SLASH '/'
byte	MQTT_pub_qos =	1;		// Quality-of-service used for publishing
byte	MQTT_sub_qos =	1;		// Quality-of-service used for subscriptions

char	MQTT_pub_topic_buffer[128] = "";		// text buffer for topic to be published
char	MQTT_pub_payload_buffer[64] = "";		// text buffer for payload to be published
char	fl_buffer[20] = "";
uint8_t	MQTT_MB_pointer = 0;				// index to parameter to published next

long mqtt_lastReconnectAttempt = 0;			// timestamp of last reconnect attempt
long mqtt_lastPublishAttempt = 0;				// timestamp of last publish attempt


#endif

