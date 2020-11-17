
#define DCC_PIN    4                   // Arduino pin for DCC out 
//Timer frequency is 2MHz for ( /8 prescale from 16MHz )
#define TIMER_SHORT 0x8D               // 58usec pulse length 141 255-141=114
#define TIMER_LONG  0x1B               // 116usec pulse length 27 255-27 =228

unsigned char last_timer=TIMER_SHORT;  // store last timer value
   
unsigned char flag=0;                  // used for short or long pulse
bool second_isr = false;               // pulse up or down

                         
#define PREAMBLE  0                    // definitions for state machine
#define SEPERATOR 1                    // definitions for state machine
#define SENDBYTE  2                    // definitions for state machine

unsigned char state = PREAMBLE;
unsigned char preamble_count = 16;
unsigned char outbyte = 0;
unsigned char cbit = 0x80;
                                         
unsigned char locoSpeed=0;             // variables for throttle
unsigned char dir=1;                   //forward
unsigned char locoAdr=8;              // this is the (fixed) address of the loco
unsigned char sound=0;

/* keaypad flags*/   
int speedFlag = 1;
int trackFlag = -1;

/*struct for evalutating trackCommand */
struct trackCommands{
  unsigned char onOff, trackAddr, trackDir;
};

typedef struct trackCommands trackCommandStruct;

/* train track */
unsigned char A;
unsigned char D;
#include <Keypad.h>

const byte ROWS = 4; 
const byte COLS = 3; 

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[ROWS] = {13, 12, 11, 10}; 
byte colPins[COLS] = {9, 8, 7}; 

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 



struct Message                         // buffer for command
{
   unsigned char data[7];
   unsigned char len;
};

#define MAXMSG  2

struct Message msg[MAXMSG] = 
{ 
    { { 0xFF,     0, 0xFF, 0, 0, 0, 0}, 3},   // idle msg
    { { locoAdr, 8,  0, 0, 0, 0, 0}, 3}       // locoMsg with 128 speed steps
}; 
                                
int msgIndex=0;  
int byteIndex=0;


//Setup Timer2.
//Configures the 8-Bit Timer2 to generate an interrupt at the specified frequency.
//Returns the time load value which must be loaded into TCNT2 inside your ISR routine.

void SetupTimer2()
{
  //Timer2 Settings: Timer Prescaler /8, mode 0
  //Timer clock = 16MHz/8 = 2MHz oder 0,5usec
  // 
  TCCR2A = 0; //page 203 - 206 ATmega328/P

  TCCR2B = 2; //Page 206
  
/*         bit 2     bit 1     bit0
            0         0         0       Timer/Counter stopped 
            0         0         1       No Prescaling
            0         1         0       Prescaling by 8
            0         0         0       Prescaling by 32
            1         0         0       Prescaling by 64
            1         0         1       Prescaling by 128
            1         1         0       Prescaling by 256
            1         1         1       Prescaling by 1024
*/
  TIMSK2 = 1<<TOIE2;   //Timer2 Overflow Interrupt Enable - page 211 ATmega328/P   
  TCNT2=TIMER_SHORT;   //load the timer for its first cycle
}

ISR(TIMER2_OVF_vect) //Timer2 overflow interrupt vector handler
{ 

   char customKey = customKeypad.getKey();
  
    if(customKey == '1'){
      speedFlag = 1;
    }
    if(customKey == '2'){
      speedFlag = 2;
    }
    if (customKey == '3'){
      trackFlag = 3;
    }
    if (customKey == '4'){
      trackFlag= 5;
    }
    if (customKey == '5'){
      trackFlag = 7;
    }
    if (customKey == '6'){
      trackFlag = 9;
    }
    
  //Capture the current timer value TCTN2. This is how much error we have
  //due to interrupt latency and the work in this function
  //Reload the timer and correct for latency.  
  unsigned char latency;
  
  if (second_isr) 
  {  // for every second interupt just toggle signal
     digitalWrite(DCC_PIN,1);
     second_isr = false;    
     latency=TCNT2;    // set timer to last value
     TCNT2=latency+last_timer; 
  }
  else  
  {  // != every second interrupt, advance bit or state
     digitalWrite(DCC_PIN,0);
     second_isr = true;
     switch(state)  
     {
       case PREAMBLE:
           flag=1; // short pulse
           preamble_count--;
           if (preamble_count == 0)  
           {  
              state = SEPERATOR; // advance to next state
              msgIndex++; // get next message
              if (msgIndex >= MAXMSG)  
              {  
                msgIndex = 0; 
              }  
              byteIndex = 0; //start msg with byte 0
           }
           break;
        case SEPERATOR:
           flag=0; // long pulse and then advance to next state
           state = SENDBYTE; // goto next byte ...
           outbyte = msg[msgIndex].data[byteIndex];
           cbit = 0x80;  // send this bit next time first         
           break;
        case SENDBYTE:
           if ((outbyte & cbit)!=0)  
           { 
              flag = 1;  // send short pulse
           }
           else  
           {
              flag = 0;  // send long pulse
           }
           cbit = cbit >> 1;
           if (cbit == 0)  
           {  // last bit sent 
              //Serial.print(" ");
              byteIndex++;
              if (byteIndex >= msg[msgIndex].len) // is there a next byte?  
              {  // this was already the XOR byte then advance to preamble
                 state = PREAMBLE;
                 preamble_count = 16;
                 //Serial.println();
              }
              else  
              {  // send separtor and advance to next byte
                 state = SEPERATOR ;
              }
           }
           break;
     }   
 
     if (flag)  
     {  // data = 1 short pulse
        latency=TCNT2;
        TCNT2=latency+TIMER_SHORT;
        last_timer=TIMER_SHORT;
        //Serial.print('1');
     }  
     else  
     {   // data = 0 long pulse
        latency=TCNT2;
        TCNT2=latency+TIMER_LONG; 
        last_timer=TIMER_LONG;
       // Serial.print('0');
     } 
  }
}



void setup(void) 
{
  Serial.begin(9600);
  pinMode(DCC_PIN,OUTPUT);              // pin 4 this is for the DCC Signal
  assemble_dcc_msg(); 
  SetupTimer2(); // Start the timer  
}

void loop(void) 
{      
  assemble_dcc_msg();
  delay(100);
}

void assemble_dcc_msg() 
{
  
   unsigned char dataRaw,addr, xdata;
   unsigned char addrAndCom[2];
   unsigned char *addrAndComP = addrAndCom;
   trackCommandStruct commandsResults;

   
   if (speedFlag == 1){
    dataRaw = 95;
    addr = 8;
   }else{
    dataRaw = 96;
    addr = 8;
   }

   if(trackFlag > -1){
      //onOff, trackAddr, trackDir
      commandsResults = evaluateTrackCommand(trackFlag);
      changeTrack(commandsResults.trackAddr, commandsResults.onOff, commandsResults.trackDir, addrAndComP); 
      addr = *addrAndComP;
      addrAndComP++;
      dataRaw = *addrAndComP;
      addrAndComP = addrAndCom;
      if( (trackFlag % 2) == 1){
        trackFlag++;
      }else{
        trackFlag = -1;
      }
      //Serial.print(addr);
      //Serial.print(dataRaw);
   }
   
  /*
   switch(trackFlag){
    case 3: //change track 102 to straight
      changeTrack(102, 1, 1, addrAndComP); //parameters are in order: unsigned char trackAddr, int number, int trackDirection, unsigned char *assignmentP
      addr = *addrAndComP;
      addrAndComP++;
      dataRaw = *addrAndComP;
      addrAndComP = addrAndCom;
      trackFlag = 4;
      Serial.print(addr);
      Serial.print(dataRaw);
      break;
    case 4: //change track 102 to straight second command
      changeTrack(102, 0, 1, addrAndComP);
      addr = *addrAndComP;
      addrAndComP++;
      dataRaw = *addrAndComP;
      addrAndComP = addrAndCom;
      trackFlag = -1;
      Serial.print(addr);
      Serial.print(dataRaw);
      break;
    case 5: //change track 102 to turn
      changeTrack(102, 1, 0, addrAndComP); 
      addr = *addrAndComP;
      addrAndComP++;
      dataRaw = *addrAndComP;
      addrAndComP = addrAndCom;
      trackFlag = 6;
      //Serial.print(addr);
      //Serial.print(dataRaw);
      break;
    case 6: //change track 102 to turn second command
      changeTrack(102, 0, 0, addrAndComP); 
      addr = *addrAndComP;
      addrAndComP++;
      dataRaw = *addrAndComP;
      addrAndComP = addrAndCom;
      trackFlag = -1;
      //Serial.print(addr);
      //Serial.print(dataRaw);
      break;
    case 7://change track 101 to stright
      changeTrack(101, 1, 1, addrAndComP); 
      addr = *addrAndComP;
      addrAndComP++;
      dataRaw = *addrAndComP;
      addrAndComP = addrAndCom;
      trackFlag = 8;
      //Serial.print(addr);
      //Serial.print(dataRaw);
      break;
    case 8://change track 101 to straight second command
      changeTrack(101, 0, 1, addrAndComP); 
      addr = *addrAndComP;
      addrAndComP++;
      dataRaw = *addrAndComP;
      addrAndComP = addrAndCom;
      trackFlag = -1;
      //Serial.print(addr);
      //Serial.print(dataRaw);
      break;
    case 9://change track 101 to turn 
      changeTrack(101, 1, 0, addrAndComP); 
      addr = *addrAndComP;
      addrAndComP++;
      dataRaw = *addrAndComP;
      addrAndComP = addrAndCom;
      trackFlag = 10;
      //Serial.print(addr);
      //Serial.print(dataRaw);
      break;
    case 10:
      changeTrack(101, 0, 0, addrAndComP); 
      addr = *addrAndComP;
      addrAndComP++;
      dataRaw = *addrAndComP;
      addrAndComP = addrAndCom;
      trackFlag = -1;
      //Serial.print(addr);
      //Serial.print(dataRaw);
      break;
   }//all this could be written with a single if statement or a function, but would be more convoluted.
  */
  
  /*
   if (trackFlag == 3){
      //changeTrack(102, 1, 1, addrAndComP);
      //dataRaw = *addrAndComp;
      //addrAndComp++;
      dataRaw = 249;
      addr = 154;
   }
   if (trackFlag == 4){
      dataRaw = 241;
      addr = 154;
      trackFlag = 0;
    }
   if(trackFlag == 3){
      trackFlag = 4;
   }
   
   
   if (trackFlag == 5){
      dataRaw = 248;
      addr = 154;
   }
   if (trackFlag == 6){
      dataRaw = 240;
      addr = 154;
      trackFlag = 0;
    }
   if(trackFlag == 5){
      trackFlag = 6;
   }
  */
   
   noInterrupts();  // make sure that only "matching" parts of the message are used in ISR  
   msg[1].data[0] = addr;
   msg[1].data[1] = dataRaw;
   xdata = msg[1].data[0] ^ msg[1].data[1];
   msg[1].data[2] = xdata;
   interrupts();
}

trackCommandStruct evaluateTrackCommand(int flagCommand){
  
  trackCommandStruct commands;

  //onOff, trackAddr, trackDir
  
  if(flagCommand == 3){
    commands.trackAddr = 102;
    commands.onOff = 1;
    commands.trackDir =  1;
  }
  else if(flagCommand == 4){
    commands.trackAddr = 102;
    commands.onOff = 0;
    commands.trackDir =  1;
  }
  else if(flagCommand == 5){
    commands.trackAddr = 102;
    commands.onOff = 1;
    commands.trackDir =  0;
  }
  else if(flagCommand == 6){
    commands.trackAddr = 102;
    commands.onOff = 0;
    commands.trackDir =  0;
  }
  else if(flagCommand == 7){
    commands.trackAddr = 101;
    commands.onOff = 1;
    commands.trackDir =  1;
  }
  else if(flagCommand == 8){
    commands.trackAddr = 101;
    commands.onOff = 0;
    commands.trackDir =  1;
  }
  else if(flagCommand == 9){
    commands.trackAddr = 101;
    commands.onOff = 1;
    commands.trackDir =  0;
  }
  else if(flagCommand == 10){
    commands.trackAddr = 101;
    commands.onOff = 0;
    commands.trackDir =  0;
  }
  return(commands);
  
}

void changeTrack(unsigned char trackAddr, unsigned char number, unsigned char trackDirection, unsigned char *assignmentP){
  unsigned char fixed = 63;
  unsigned char accAdr = (((trackAddr / 4) + 1) & fixed);
  unsigned char regAddr = ((trackAddr % 4) -1 );
  unsigned char addr;
  unsigned char com;
  
  
  for(int i = 7; i>=0; i--){
    if(((accAdr>>i) & 1) == 1){
      addr |= 1 << i;
      
    }else{
      addr &= ~(1 << i);
    }
      
  }  addr |= 1 << 7;
  //Serial.print(addr);
  
  
  /*com precdure*/
  
   com |= 1 << 7;
   
   
   for(int i = 6; i>=4; i--){
    if( ( (accAdr>>(i+1)) & 1) == 0 ){
      com |= 1 << i;
    }else{
      com &= ~(1 << i);
    }
   }
   
   if(number == 1){
    com |= 1 << 3;
   }else{
    com &= ~(1 << 3);
   }
   
   if(regAddr < 0){
    regAddr = 3;
   }
   for(int i = 1, z = 0; i >= 0; i--, z++){
    if( ((regAddr>>z) & 1) == 1){
      com |= 1 << (i+1);
    }else{
      com &= ~(1<<(i+1));
    }
   }
   
    if(trackDirection == 1){
      com |= 1<<0;
    }else{
      com &= ~(1<<0);
    }
    //Serial.print(com);
    
  /*com precdure*/
  
  *assignmentP = addr;
  assignmentP++;
  *assignmentP = com;
}
