
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

/*sensor vars */
const int trigPin=2;
const int echoPin=3;
long duration;
int distance;
int temp = 1;
int trackTemp = 1;
int tempLocoAddr = 7;

/* keaypad flags*/   
int speedFlag = 1;
int trackFlag = 3;

/* time */
unsigned long startTime = millis();

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
    { { locoAdr, 0,  0, 0, 0, 0, 0}, 3}       // locoMsg with 128 speed steps
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
    else if(customKey == '2'){
      speedFlag = 2;
    }
    else if (customKey == '3'){
      trackFlag = 3;
    }
    else if (customKey == '4'){
      trackFlag= 5;
    }
    else if (customKey == '5'){
      trackFlag = 7;
    }
    else if (customKey == '6'){
      trackFlag = 9;
    }
    else if(customKey == '7'){
      speedFlag = 3;
      Serial.print('3');
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

  //Distance
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  
  Serial.begin(9600);
  pinMode(DCC_PIN,OUTPUT);              // pin 4 this is for the DCC Signal
  assemble_dcc_msg(); 
  SetupTimer2(); // Start the timer  
}

void loop(void) 
{      
  assemble_dcc_msg();
  delay(200);
}

void assemble_dcc_msg() 
{  
   unsigned char dataRaw,addr, xdata;
   unsigned char addrAndCom[2];
   unsigned char *addrAndComP = addrAndCom;
   trackCommandStruct commandsResults;

   digitalWrite(trigPin, LOW);
   delayMicroseconds(2);
   digitalWrite(trigPin, HIGH);
   delayMicroseconds(10);
   duration = pulseIn(echoPin, HIGH);
   distance = duration*0.034/2;
   
   unsigned long currentTime = millis();
   unsigned long elapsedTime = currentTime - startTime;
   
   if(distance < 15 && elapsedTime > 3000){ //may read several times
      Serial.print(elapsedTime);
      startTime = millis();
    
      speedFlag = 2;
      temp = 2;
      
      if(trackFlag == 3){
        trackFlag = 5;
        trackTemp = trackFlag;
      }else{
        trackFlag = 3;
        trackTemp = trackFlag;
      }
      
      
   }else if(temp == 2){
        speedFlag = 0;
    
        if(trackFlag > -1){
        commandsResults = evaluateTrackCommand(trackFlag);
        changeTrack(commandsResults.trackAddr, commandsResults.onOff, commandsResults.trackDir, addrAndComP); 
        addr = *addrAndComP;
        addrAndComP++;
        dataRaw = *addrAndComP;
        addrAndComP = addrAndCom;
      }
      
      if( (trackFlag % 2) == 1){
        trackFlag++;
      }else{
        temp = 3;
        trackFlag = trackTemp;
      }
      
        
   }
   
   else if(temp == 3){
      speedFlag = 1;
      
      if(tempLocoAddr == 7){
        tempLocoAddr = 40;
      }else{
        tempLocoAddr = 7;
      }
      temp = 0;
      
   }

   if (speedFlag == 1){
    addr = tempLocoAddr;
    if(addr == 40){
      dataRaw = 120;
    }else{
      dataRaw = 123;
    }
   }
   else if(speedFlag == 2){
    dataRaw = 96;
   }
   else if(speedFlag == 3){
    dataRaw = 92;
   }

    
      

      
      /*
      speedFlag = 1;
      if(addr == 7){
      addr = 6;
      }else{
        addr = 7;
      }*/
    
    
  


  /*
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
      Serial.print(addr);
      Serial.print(dataRaw);
   }*/
   
   
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
    if( ((regAddr>>i) & z) == 1){
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
