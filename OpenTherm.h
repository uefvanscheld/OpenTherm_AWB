/*
**********************************************************
	INCLUDES FOR OPENTHERM
**********************************************************
*/

// define OpenTherm message structure 
typedef struct
{
	byte			id;		// message ID defines the request or command sent
	byte			type;	// type of message: Master <-/-> Slave or Read/Write; 3 bits used filled with spare bits
	unsigned int	value;	// data sent or received; may be interpreted in different ways depending on message ID (s. OpenTherm protocol reference)
} OT_message;

// define OpenTherm parameter data types
// typedef enum {flag8, u8, s8, f8_8, u16, s16} DataTypes;
typedef enum {both, highbyte, lowbyte} ByteType;


// define message types sent by master and slave
const byte	MSTR_RD_DTA	= B000;	// Master sends a read message to slave
const byte	MSTR_WRT_DTA = B001;	// Master sends a write message to slave
const byte	MSTR_INVLD 	= B010;		// Master sends a invalid data message to slave; not really used
const byte	SLV_RD_ACK	= B100;		// Slave sends a read acknowledge message to master
const byte	SLV_WRT_ACK = B101;		// Slave sends a write acknowledgemessage to master
const byte	SLV_DTA_INVLD = B110;	// Slave indicates that data sent by master was invalid
const byte	SLV_ID_INVLD = B111;	// Slave indicates that message ID sent by master was invalid or unknown

// STATUS BITS FOR MASTER
const byte  CH_Enabled  = B00000001;  // for message ID=0: set heating enabled
const byte  DHW_Enabled = B00000010;  // for message ID=0: set hot water enabled
byte  MSTR_STATUS = DHW_Enabled;     // Only enable domestic hot water

// STATUS BITS FOR SLAVE
const byte  CH_Active  = B00000010;  // for message ID=0: set heating enabled
const byte  DHW_Active = B00000100;  // for message ID=0: set hot water enabled

byte  SLV_STATUS = 0;

// define structure for message buffer for OpenTherm communication
typedef struct
{
	int8_t			s_prio;			// controls how often a specific request is sent 
	byte			s_id;			// message ID defines the request or command sent
	byte			s_type;			// type of message: Master <-/-> Slave or Read/Write; 3 bits used filled with spare bits
	unsigned int	s_value;		// data sent or received; may be interpreted in different ways depending on message ID (s. OpenTherm protocol reference)
	byte			r_id;			// message ID defines the request or command sent
	byte			r_type;			// type of message: Master <-/-> Slave or Read/Write; 3 bits used filled with spare bits
	unsigned int	r_value;		// data sent or received; may be interpreted in different ways depending on message ID (s. OpenTherm protocol reference)
	boolean			changed;		// 'true' indicates that parameter changed to a new value; introduced to reduce traffic load; set during frame analysis, reset by MQTT publish routine
	unsigned long   tmstp_rec;		// timestamp when received last valid/successful message of this type (=ID)
	unsigned int	err_cnt;		// errors occurred since last successfully received message of this ID; indicator that value might not be valid meanwhile
	char			topic[10];		// used for MQTT to assign parameter to topics
	char			par_name[25];		// name of this parameter within OpenTherm protocol and also used for MQTT to identify topics
	
} OT_Com_message;
	
volatile int16_t	MB_pointer = 0;						// index of the next message to be read or write from monitoring buffer
volatile byte		prio_num = 4;						// number of prioity levels used
volatile byte		prio_pointer[4] = {0, 0, 0, 0};		// vector of indices to MonitorBuffer one for each priority
volatile byte		prio_index = 0;						// index of prio_pointer holding current priority level
const int16_t		na = -1;							// to prevent this parameter to be requested explicitly

const byte			MB_length = 10;	// number of messages in the buffer of monitoring messages
// set prio to 'na' to prevent parameter to be requested from slave (used to keep e.g. fault flags in buffer)
volatile OT_Com_message MonitorBuffer[] = 
{  /*  
    |-------SEND-----------------------||---RECEIVE--------|
   Prio,ID,Type     ,Value      ,ID ,Type ,Value  ,Changed,timestamp  ,errors ,topic  ,label                    */

/*  
  {1  , 0 ,MSTR_RD_DTA  ,(((unsigned int) MSTR_STATUS)<<8)|SLV_STATUS,0  ,0    ,0    ,false  ,0      ,0    ,"status/"  ,"MasterSlaveStatusFlags"}  , // #00, ID=0: exchange status information
*/
  {1  , 0 ,MSTR_RD_DTA  ,0        ,0  ,0    ,0    ,false  ,0      ,0    ,"status/"  ,"MasterSlaveStatusFlags"}  , // #00, ID=0: exchange status information
  {0  , 5 ,MSTR_RD_DTA  ,0        ,0  ,0    ,0    ,false  ,0      ,0    ,"status/"  ,"FaultFlags"}        , // #01, ID=5: read slave's fault flags indicating functional error/problems e.g. water pressure 
  {3  , 3 ,MSTR_RD_DTA  ,0        ,0  ,0    ,0    ,false  ,0      ,0    ,"status/"  ,"SlaveConfiguration"}    , // #02, ID=3: read slave's functional capabilities  
  {1  ,17 ,MSTR_RD_DTA  ,0        ,0  ,0    ,0    ,false  ,0      ,0    ,"ch/"    ,"RelModulationLevel"}      , // #03, ID=17: read relative modulation of boiler
  {1  ,18 ,MSTR_RD_DTA  ,0        ,0  ,0    ,0    ,false  ,0      ,0    ,"ch/"    ,"CHWaterPressure"}       , // #04, ID=18: read water pressure in boiler
  {2  ,19 ,MSTR_RD_DTA  ,0        ,0  ,0    ,0    ,false  ,0      ,0    ,"dhw/"   ,"DHWFlowRate"}         , // #05, ID=19: read water flow rate for hot water (l/min)
  {1  ,25 ,MSTR_RD_DTA  ,0        ,0  ,0    ,0    ,false  ,0      ,0    ,"ch/"    ,"BoilerWaterTemperature"}    , // #06, ID=25: read flow water temperature from boiler (°C)
  {1  ,28 ,MSTR_RD_DTA  ,0        ,0  ,0    ,0    ,false  ,0      ,0    ,"ch/"    ,"ReturnWaterTemperature"}    , // #07, ID=28: read return water temperature to boiler (°C)
  {2  ,26 ,MSTR_RD_DTA  ,0        ,0  ,0    ,0    ,false  ,0      ,0    ,"dhw/"   ,"DHWTemperature"}         , // #08, ID=26: read hot  water temperature (°C)
  {3  ,125 ,MSTR_RD_DTA  ,0        ,0  ,0    ,0    ,false  ,0      ,0    ,"status/"   ,"OTVersionSlave"}          // #08, ID=26: read hot  water temperature (°C)
};

// timing for T1 interrupt
// these are the values the countdown timer has to be preloaded for next interrupt
// Prescaler is selected to 256 for 16Mhz; 
// prescaler 256 was preferred over 1024 since it allowed a more accurate value for the match value
unsigned int INT3_PRESCALER = 256;          // prescaler value for timer 3
unsigned int INT3_INTERVAL_SHORT = 31;	    // 16MHz / 256 / 31 = 2000Hz; frequency for master to send 1Khz with Manchester coding
unsigned int INT3_INTERVAL_LONG = 50000;    // 16MHz / 256 / 50000 = 800ms = 1,25Hz; timeout for slave to reply on master request
unsigned int INT4_PRESCALER = 256;          // prescaler value for timer 4
unsigned int INT4_INTERVAL_LONG = 62500;	  // 16MHz  / 256 = 62367 clock cycle to remind master for next telegram each second



/*
*********************************************************************
*
* definitions of central heating and hot water values
*
*********************************************************************
*/
// Central heating
volatile float OT_WaterPressure;        // 0..5 bar, f8.8
volatile float OT_RelModulationLevel;   // 0..100 %, f8.8
volatile float OT_BoilerWaterTemperature; // -40..127 °C, f8.8
volatile float OT_ReturnWaterTemperature; // -40..127 °C, f8.8
// Domestic hot water
volatile float OT_DHWFlowRate;          // 0..16 l/min, f8.8
volatile float OT_DHWTemperature;       // -40..127 °C, f8.8
// Configuration
volatile boolean  OT_CHEnabled;        // Central heating
volatile boolean  OT_DHWEnabled;       // Domestic hot water
volatile boolean  OT_CoolingEnabled;   // Cooling enabled ?
volatile boolean  OT_OTCEnabled;       // Outside temperature control
volatile boolean  OT_Fault;            // Fault indicator

volatile int      OT_DiagnosticCode;   // Diagnostic / service code
volatile boolean  OT_Fault_ServiceRequest;
uint8_t           OT_Fault_ServiceRequest_Mask = 0x01;
volatile boolean  OT_Fault_LockoutReset;
uint8_t           OT_Fault_LockoutReset_Mask = 0x02;
volatile boolean  OT_Fault_LowWaterPressure;
uint8_t           OT_Fault_LowWaterPressure_Mask = 0x04;
volatile boolean  OT_Fault_GasFlame;
uint8_t           OT_Fault_GasFlame_Mask = 0x08;
volatile boolean  OT_Fault_AirPressure;
uint8_t           OT_Fault_AirPressure_Mask = 0x10;
volatile boolean  OT_Fault_WaterOverTemperatur;
uint8_t           OT_Fault_WaterOverTemperatur_Mask = 0x20;


