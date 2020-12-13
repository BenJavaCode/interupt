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

unsigned char trainAddresses[] = {7,8,40};
int ini = 0;

unsigned long checkTime = millis();

unsigned char outList[10][5] = {8, 254, 254, 254, 254,
                               14, 254, 254, 254, 254,
                               13, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               };

unsigned long timeArr[10] = {0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             0
                            };

struct sensPoint{
  unsigned char pointAddr;
  unsigned char lights[2];
  unsigned char addrArr[4];
  unsigned char typeOff;
  unsigned char tracks[4];
};

unsigned char commandsS[9] = {0,0,0,0, 0,0,0,0,254}; //F1:0,F2:1,T1:2,T2:3,FYN1:4,FYN2:5,TYN1:6,TYN2:7
unsigned char commandsB[13] = {0,0,0,0, 0,0,0,0,0,0,0,0,254}; //F1:0,F2:1,T1:3,T2:4,T3:5,T4:6,FYN1:7,FYN2:8,TYN1:9,TYN2:10,TYN3:11,TYN4:11
unsigned char commandsType = 0;
unsigned char* commandsPointer = NULL;
unsigned char* commandsPointer2 = NULL;
unsigned char onOrOff = 1;

unsigned char *pToOutReceived = NULL;


unsigned char sensPins[] = {3,5,6,7,8,9,10,11};

typedef struct sensPoint sensPointStruct;

unsigned long quarantineArr[] = {66,66,66};
unsigned long quarantineTimes[3];

unsigned char valFlag = 0;

unsigned char inProgressFlag = 0;

// 1 means small, 2 means big
sensPointStruct aight = {
  8, {81, 82}, {9, 10}, 1 , {241, 242}
};

sensPointStruct nine = {
  9, {91, 92}, {8, 11, 12, 15}, 2, {241, 243, 249, 251}
};

sensPointStruct ten = {
  10, {101, 102}, {8, 11, 12, 15}, 2, {242, 244, 250, 252}
};

sensPointStruct eleven = {
  11, {111, 112}, {9, 10}, 1, {243, 244}
};

sensPointStruct twelve = {
  12, {121, 122}, {13, 14, 9, 10}, 2, {249, 250, 241, 242}
};

sensPointStruct thirteen = {
  13, {131, 132}, {12, 15}, 1, {249, 251}
};

sensPointStruct fourteen = {
  14, {141, 142}, {12, 15}, 1, {250, 252}
};

sensPointStruct fifteen = {
  15, {151, 152},{13, 14, 9, 10}, 2, {251, 252, 243, 244}
};


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
  for(int i = 0; i<=7; i++){
    pinMode(sensPins[i], INPUT_PULLUP);
  }
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


   /* set all the trains in motion at the begining*/
   if(ini <= 2){
    dataRaw = 99;
    addr = trainAddresses[ini]; 
    ini++;
   }else{
    dataRaw = 0;
    addr = 0;
   }
  

  if(ini > 2){
    unsigned char* fCommandsPointer = NULL;
    if(inProgressFlag == 0){
      unsigned char checkFlag = sensorDetectAndQuar();
      pToOutReceived = sensorDecision(checkFlag);
  
      unsigned long timeNow = millis();
      unsigned long currentCheckTime = timeNow - checkTime;
      unsigned long currentPointTime = timeNow - timeArr[0];
  
      if(currentCheckTime > 5000 && outList[0][0] != 254 && outList[0][1] != 254 && currentPointTime > 7000){
        //Serial.print(currentPointTime);
        unsigned char manipulatedPointAddr = outList[0][4] - 8; //here
        checkTime = millis();
        pToOutReceived = sensorDecision(manipulatedPointAddr);
        checkFlag = outList[0][4]; // for printing only
      }
  
      if(checkFlag != 66){
        commandsType = interAndSend(pToOutReceived);
        if(commandsType == 1){
          commandsPointer = commandsS;
          commandsPointer2 = commandsPointer;
          commandsPointer2 += 4;
        }else{
          commandsPointer = commandsB;
          commandsPointer2 = commandsPointer;
          commandsPointer2 += 6;
        }
        inProgressFlag = 1;
        onOrOff = 1;
      }
    }
    if(inProgressFlag == 1){
      /*for(int i = 0; i<=11; i++){
        if(*commandsPointer != 254){
          Serial.print(*commandsPointer);
          Serial.print(',');
          commandsPointer++;
        }
      }*/
      if(*commandsPointer2 != 254 && *commandsPointer > 80){
        changeTrack(*commandsPointer, onOrOff, *commandsPointer2, addrAndCom);
        addr = *addrAndComP;
        addrAndComP++;
        dataRaw = *addrAndComP;
        addrAndComP = addrAndCom;

        if(onOrOff == 0){
          commandsPointer++;
          commandsPointer2++;
        }
        onOrOff = (onOrOff == 1) ? 0: 1;
        
      }else{
        //break out of process
        inProgressFlag = 0;
      } 
    }

  }
   
   
   noInterrupts();  // make sure that only "matching" parts of the message are used in ISR  
   msg[1].data[0] = addr;
   msg[1].data[1] = dataRaw;
   if(addr != 0){
   Serial.print(msg[1].data[0]);
   Serial.print("-");
   Serial.print(msg[1].data[1]);
   Serial.print(" ");
   }
   xdata = msg[1].data[0] ^ msg[1].data[1];
   msg[1].data[2] = xdata;
   interrupts();
}



unsigned char* sensorDecision(unsigned char checkFlag){
  unsigned char *pToOut;
  
  if(checkFlag != 66){
    sensPointStruct temp = (checkFlag == 0) ? aight : (checkFlag == 1) ? nine : (checkFlag == 2) ? ten : (checkFlag == 3) ? eleven : (checkFlag == 4) ? twelve : (checkFlag == 5) ? thirteen : (checkFlag == 6) ? fourteen : fifteen;
    for(int i = 0; i<=9; i++){
      
      if(outList[i][0] == temp.pointAddr || outList[i][1] == temp.pointAddr || outList[i][2] == temp.pointAddr || outList[i][3] == temp.pointAddr || outList[i][4] == temp.pointAddr){
 
        for(int ql = i; ql<=9; ql++){
          if(outList[ql][0] == 254){
            break;
          }else if(ql < 9){
            outList[ql][0] = outList[ql+1][0];
            outList[ql][1] = outList[ql+1][1];
            outList[ql][2] = outList[ql+1][2];
            outList[ql][3] = outList[ql+1][3];
            outList[ql][4] = outList[ql+1][4]; 
            timeArr[ql] = timeArr[ql+1];
          }else{
            outList[ql][0] = 254;
            outList[ql][1] = 254;
            outList[ql][2] = 254;
            outList[ql][3] = 254;
            outList[ql][4] = 254;
            timeArr[ql] = 0; 
            break;
          }
        } 
      }
     
      for(int k = 0; k<=4; k++){
        
        if(outList[i][k] == 254 && k == 0 ){
          unsigned char indexes[] = {0, 0, 0, 0};
          int pp = 0;
          
          if(temp.addrArr[1] == 0 && temp.addrArr[2] == 0 && temp.addrArr[3] == 0 && temp.addrArr[4] == 0){
            outList[i][0] = 0;
            outList[i][1] = 0;
            outList[i][2] = 0;
            outList[i][3] = 0;
          }else{
            for(int c = 0; c<=3; c++){
              if(temp.addrArr[c] != 0){
                pp++;
                indexes[c] = 1;
              }
            }
          }
          if(pp > 2){  
            if(indexes[0] != 0 || indexes[1] != 0){ //changed from and to or, might cause problems
              outList[i][k] = temp.addrArr[0];
              outList[i][k+1] = temp.addrArr[1];
              outList[i][k+2] = 0;
              outList[i][k+3] = 0;
            }else if(indexes[2] != 0 || indexes[3] != 0){ //changed from and to or, might cause problems
              outList[i][k] = 0;
              outList[i][k+1] = 0;
              outList[i][k+2] = temp.addrArr[2];
              outList[i][k+3] = temp.addrArr[3];
            }
          }else{
              outList[i][k] = temp.addrArr[0];
              outList[i][k+1] = temp.addrArr[1];
              outList[i][k+2] = temp.addrArr[2];
              outList[i][k+3] = temp.addrArr[3];
          }
          outList[i][k+4] = temp.pointAddr;
          unsigned long tOut = millis();
          timeArr[i] = tOut;
          pToOut = &outList[i][0];
          Serial.print(" adresses ");
          Serial.print(outList[i][0]);
          Serial.print(outList[i][1]);
          Serial.print(outList[i][2]);
          Serial.print(outList[i][3]);
          Serial.print(outList[i][4]);
          Serial.print(" adresses ends ");
          return (pToOut);
        }

        if(temp.typeOff == 1){
          if(outList[i][k] == temp.addrArr[0]){
            temp.addrArr[0] = 0;
          }
          if(outList[i][k] == temp.addrArr[1]){
            temp.addrArr[1] = 0;
          }
          temp.addrArr[2] = 0;
          temp.addrArr[3] = 0;
          
        }else{
          if(outList[i][k] == temp.addrArr[0]){
            temp.addrArr[0] = 0;
          }
          if(outList[i][k] == temp.addrArr[1]){
            temp.addrArr[1] = 0;
          }
          if(outList[i][k] == temp.addrArr[2]){
            temp.addrArr[2] = 0;
          }
          if(outList[i][k] == temp.addrArr[3]){
            temp.addrArr[3] = 0;
          }
        }
      
      }
    }
  }
}

unsigned char sensorDetectAndQuar(){
  unsigned char checkFlag = 66;
  /*for checking sensorpins and doing timecheck, so we have no repeats */ 
  for(int i = 0; i<=7; i++){
    valFlag = digitalRead(sensPins[i]);
    
    if(valFlag == 0){

      if(i != quarantineArr[0] && i != quarantineArr[1] && i != quarantineArr[2]){
        
          for(int k = 0; k<=2; k++){

            if(quarantineArr[k] == 66){
              quarantineArr[k] = i;
              unsigned long startTime = millis();
              quarantineTimes[k] = startTime;
              checkFlag = i;
              break;
            }
          }
      } 
    }
  }/*for checking sensorpins and doing timecheck, so we have no repeats */ 

  /*removing sensorpins from quarintine lists if time is due */ 
  for(int k = 0; k<=2; k++){
    if(quarantineArr[k] != 66){
      unsigned long currentTime = millis();
      unsigned long elapsedTime = currentTime - quarantineTimes[k];
      if(elapsedTime > 4000){
       // Serial.print(elapsedTime);
        quarantineArr[k] = 66;
        quarantineTimes[k] = 0;
      }
    }
  }/*removing sensorpins from quarintine lists if time is due */ 

  return(checkFlag);
}

unsigned char interAndSend(unsigned char* pToOutReceived){

  unsigned char* dPToReceived = pToOutReceived;
  pToOutReceived+=4;
  unsigned char pointAddr = *pToOutReceived;
  pToOutReceived = dPToReceived;
  /*
  Serial.print(" here are the addresses out: ");
  for(int i = 0; i<=3; i++){
    Serial.print(*pToOutReceived);
    pToOutReceived++;
    Serial.print(',');
  }pToOutReceived = dPToReceived;
  Serial.print(" ");
  */
  
  sensPointStruct temp = (pointAddr == 8) ? aight : (pointAddr == 9) ? nine : (pointAddr == 10) ? ten : (pointAddr == 11) ? eleven : (pointAddr == 12) ? twelve : (pointAddr == 13) ? thirteen : (pointAddr == 14) ? fourteen : fifteen;
  
  if(temp.typeOff == 1){
 
    unsigned char* clearerPointer = commandsS;
    for(int i = 0; i<=6; i++){
      *clearerPointer = 0;
      clearerPointer++;
    }*clearerPointer = 254;
    

    commandsS[0] = temp.lights[0];
    commandsS[1] = temp.lights[1];
      
    if(*pToOutReceived != 0){
      commandsS[2] = temp.tracks[0];
      commandsS[4] = 1;
      commandsS[6] = 1;
     }
    pToOutReceived++;
    if(*pToOutReceived != 0){
      commandsS[3] = temp.tracks[1];
      commandsS[5] = 1;
      commandsS[7] = 1;
    }
    return(1);
  }else{
    unsigned char* clearerPointer = commandsB;
    for(int i = 0; i<=11; i++){
      *clearerPointer = 0;
      clearerPointer++;
    }*clearerPointer = 254;

    commandsB[0] = temp.lights[0];
    commandsB[1] = temp.lights[1];

    for(int i = 1; i<=4; i++){
      if( (i % 2) != 0 && *pToOutReceived != 0){
        commandsB[6] = 1;
      }else if( (i%2) == 0 && *pToOutReceived != 0){
        commandsB[7] = 1;
      }
      pToOutReceived++;
    }
    pToOutReceived = dPToReceived;
    
     //F1:0,F2:1,T1:2,T2:3,T3:4,T4:5,FYN1:6,FYN2:7,TYN1:8,TYN2:9,TYN3:10,TYN4:11
    for(int i = 0; i<=1; i++){
      if(*pToOutReceived != 0){
        commandsB[i+2] = temp.tracks[i];
        commandsB[i+8] = 1;
      }
      pToOutReceived++;
    }
    if(*pToOutReceived != 0){
      commandsB[2] = temp.tracks[0];
      commandsB[8] = 2;
      commandsB[4] = temp.tracks[2];
      commandsB[10] = 1;
    }
    pToOutReceived++;
    if(*pToOutReceived != 0){
      commandsB[3] = temp.tracks[1];
      commandsB[9] = 2;
      commandsB[5] = temp.tracks[3];
      commandsB[11] = 1;
    }
  }
  
  return(2);
  
}



void changeTrack(unsigned char trackAddr, int number, int trackDirection, unsigned char *assignmentP){
  unsigned char addr;
  unsigned char com;

  unsigned char fixed = 63;
  
  int regAddr = ((trackAddr % 4) -1 );


  
  unsigned char accAdr = ((trackAddr / 4) + 1);
   if(regAddr < 0){
    regAddr = 3;
    accAdr--;
  }
  accAdr = accAdr & fixed;


  
  for(int i = 8; i>=0; i--){
    if(((accAdr>>i) & 1) == 1){
      addr |= 1 << i;
      
    }else{
      addr &= ~(1 << i);
    }
      
  }  addr |= 1 << 7;

  
  /*com precdure*/
  
   com |= 1 << 7;
   
   for(int i = 6; i>=4; i--){
    com |= 1 << i;
   }
   
   if(number == 1){
    com |= 1 << 3;
   }else{
    com &= ~(1 << 3);
   }
   
   for(int i = 1, z = 0; i >= 0; i--, z++){ //here
    if( ((regAddr>>i) & 1) == 1){
      com |= 1 << (z+1);
    }else{
      com &= ~(1<<(z+1));
    }
   }
   
    if(trackDirection == 1){
      com |= 1<<0;
    }else{
      com &= ~(1<<0);
    }
    
  /*com precdure*/
  
  *assignmentP = addr;
  assignmentP++;
  *assignmentP = com;
}
