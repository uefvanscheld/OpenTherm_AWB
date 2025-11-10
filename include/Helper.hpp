/*
Edit history of Helper.ino


V 0.21:		- moved some functions and procedures from main.ino into here (Helper.ino)
			

*/

#ifndef Helper_h
#define Helper_h

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
void initialize_interrupts ();


void EnableMasterBitInterrupt();
void DisableMasterBitInterrupt();

void EnableMasterFrameInterrupt();
void DisableMasterFrameInterrupt();

void EnableSlaveReceiveInterrupt();
void DisableSlaveReceiveInterrupt();

boolean mqtt_build_next_message(char , char );


// compute the even parity of a word value
// returns 1 
byte parity_even_calc(unsigned long);

// convert 16bit data value to float
float DataID2Float(signed int);

void printDouble( double , unsigned int );

// extract higher 8 bits from value in OT frame
byte getValueHB(unsigned int );

// extract lower 8 bits from value in OT frame
byte getValueLB(unsigned int );


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

char* pBin(unsigned long long int , char* );
char* pBinFill(unsigned long long int x, char*, char );
char* pBinFill8(unsigned long long int x, char*, char );
char* pBinFill16(unsigned long long int x, char*, char );

void LogFSM();
void LogFrameDebug();
void LogAppLayer(byte , byte, unsigned int);

#endif
