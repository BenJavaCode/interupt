/* place trains with two sensors in between and insert addresses of trains into trainAddresses */

//preprocessor command that includes Keypad.h from the C library before compiling 
//#include <Keypad.h>

//Today, #define is primarily used to handle compiler and platform differences. 
//#define should thus be limited unless absolutely necessary
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

//faster than functions as they don't involve function call overhead.
#define XOR(A,B) ((A)^(B)) //used to xor addr and com

/*
#ifdef MACRO     
   // conditional codes
#endif (This directive ends the scope of the #if , #ifdef , #ifndef , #else , or #elif directive.)

#if expression
   conditional codes if expression is non-zero
#elif expression1
    // conditional codes if expression is non-zero
#else
   conditional if expression is 0
#endif

#if defined BUFFER_SIZE 
  // codes

*/

unsigned char state = PREAMBLE;
unsigned char preamble_count = 16;
unsigned char outbyte = 0;
unsigned char cbit = 0x80;
                                         

unsigned char trainAddresses[] = {36,8,40}; //addresses of th trains used. Used for sending initial commands to trains
int ini = 0; // used for sending initial commands to trains/controling flow of assemble_dcc_msg

unsigned long checkTime = millis(); //for controling update of outlist points

unsigned char outList[10][6] = {254, 254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254, 254, 
                               254, 254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254, 254,
                               }; //for storing outlist points. Could be shortened with realloc according to number of trains in use
                                  // or just create the array according to number of trains :)

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
                            }; // for storing creation time of associated outlist points. Could be shortened according to number of trains aswell

struct sensPoint{
  unsigned char pointAddr;
  unsigned char lights[6];
  unsigned char addrArr[4];
  unsigned char typeOff;
  unsigned char tracks[4];
}; // struct containing all information needed to coordinate a point action

typedef struct sensPoint sensPointStruct; //type definiton for sensPoint structure 

unsigned char commandsS[13] = {0,0,0,0,0,0, 0,0,0,0,0,0, 254}; //F1:0,F2:1,F3:2,F4:3,T1:4,T2:5,FYN1:6,FYN2:7,FYN3:8,FYN4:9,TYN1:10,TYN2:11
                                                              //F1:0,F2:1,T1:2,T2:3,FYN1:4,FYN2:5,TYN1:6,TYN2:7 
                                                              //(this is coordinates of informtion in commandsS)
                                                              //Used by interAndSend to create commands based on outlist point returned from sensorDecision named pToOutReceived
//
unsigned char commandsB[21] = {0,0,0,0,0,0,0,0,0,0 ,0,0,0,0,0,0,0,0,0,0,254}; //F1:0,F2:1,F3:2,F4:3,F5:4,F6:5,T1:6,T2:7,T3:8,T4:9,
                                                                              //FYN1:10,FYN2:11,FYN3:12,FYN4:13,FYN5:14,FYN6:15,TYN1:16,TYN2:17,TYN3:18,TYN4:19
                                                                              //(this is coordinates of informtion in commandsB)
                                                                         //Used by interAndSend to create commands based on outlist point returned from sensorDecision named pToOutReceived
                                                                              
unsigned char commandsType = 0; //used in loop when inrogressFlag == 1. Used for determining pointtype that is proccesed. Returned from interAndSend
unsigned char* commandsPointer = NULL; // pointer to commands, used in assemble_dcc_msg for in progress part
unsigned char* commandsPointer2 = NULL; //pointer that points to commands, and i incremented according to commandsType, to point to part that contains direction/light on or off
                                        //fed to changeTrack as trackDirection
unsigned char onOrOff = 1; // used to controll flow (first/second message to unit) in assemble_dcc_msg in progrress part. fed to changeTrack as Number

unsigned char *pToOutReceived = NULL; // pointer to outList item created by sensorDecision. Used by interAndSend to fill in commands 

unsigned char sensPins[] = {3,5,6,7,8,9,10,11}; // pins used for sensors. pulled up in setup, and used in sensorDetectAndQuar to detect at train passing over a sensor.

unsigned long quarantineArr[] = {66,66,66}; // Used in sensorDetectAndQuar for quarintining sensorPoints, so we have no repetitions of detection in timeframe(4 seconds)
unsigned long quarantineTimes[3]; //Associated with quarantineArr by index, used in sensorDetectandQuat to remove point from quarantineArr if time is due.(stores time of detection)

unsigned char valFlag = 0; //Used in sensorDetectAndQuar for storing digitalRead of sensPins, (determins action based on value).

unsigned char inProgressFlag = 0; //Used to controll assemble_dcc_msg. When a proccess(a set of commands needs to be sent to units) this flag is used to make sure that these commands are sent
                                  //No other action can happen before this flag is set to 0 again(all commands are sent)

//these structs conatains all information associated with a point
//there are two types 1(small) and 2(big)
sensPointStruct aight = {
  8, {81, 82, 91, 101}, {9, 10}, 1 , {241, 242}
};

sensPointStruct nine = {
  9, {91, 92, 81, 111, 121, 151}, {8, 11, 12, 15}, 2, {241, 243, 249, 251}
};

sensPointStruct ten = {
  10, {101, 102, 82, 112, 122, 152}, {8, 11, 12, 15}, 2, {242, 244, 250, 252}
};

sensPointStruct eleven = {
  11, {111, 112, 92, 102}, {9, 10}, 1, {243, 244}
};

sensPointStruct twelve = {
  12, {121, 122, 131, 141, 91, 101}, {13, 14, 9, 10}, 2, {249, 250, 241, 242}
};

sensPointStruct thirteen = {
  13, {131, 132, 121, 151}, {12, 15}, 1, {249, 251}
};

sensPointStruct fourteen = {
  14, {141, 142, 122, 152 }, {12, 15}, 1, {250, 252}
};

sensPointStruct fifteen = {
  15, {151, 152, 132, 142, 92, 102},{13, 14, 9, 10}, 2, {251, 252, 243, 244}
};

/** Array of procedure pointers, and double pointer to that array **/
unsigned char sensorDetectAndQuar(); //function declaration
unsigned char interAndSend(); //function declaration

unsigned char (*fparr[2])(void) = {sensorDetectAndQuar, interAndSend}; // creates array with two function pointers. Return type is unisgned char, and there is no parameters(void)
unsigned char (**pToFuncArray)(void) = fparr; //double pointer to fpar

/** Array of procedure pointers, and double pointer to that array **/


struct Message                         // buffer for command
{
   unsigned char data[7];
   unsigned char len;
};

#define MAXMSG  2

struct Message msg[MAXMSG] = 
{ 
    { { 0xFF,     0, 0xFF, 0, 0, 0, 0}, 3},   // idle msg
    { { 0, 0,  0, 0, 0, 0, 0}, 3}       // locoMsg with 128 speed steps
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
  pinMode(DCC_PIN,OUTPUT); // pin 4 this is for the DCC Signal
  for(int i = 0; i<=7; i++){ //pullup of sensorPins, so we can read change when val is 0
    pinMode(sensPins[i], INPUT_PULLUP);
  }
  SetupTimer2(); // Start the timer  
}

void loop(void) 
{      
  assemble_dcc_msg();
}

void assemble_dcc_msg() 
{
  
   unsigned char dataRaw,addr, xdata;
   unsigned char addrAndCom[2]; //used by changetrack to input addr and com
   unsigned char *addrAndComP = addrAndCom;
   addr = 0; //set to 0, so that if not changed, no addr and com will be sent


   /* set all the trains in motion at the begining*/
  if(ini <= 2){ //UPDATE to sizeOf trainAddrArr
    addr = trainAddresses[ini];
    dataRaw = 104;
    ini++;
  }
  

  if(ini > 2){
    if(inProgressFlag == 0){ // only if no procces in proggress
      
      unsigned char checkFlag = (**pToFuncArray) (); //sensorDetectAndQuar
      pToOutReceived = sensorDecision(checkFlag); // pToOutReceived is a pointer to a outList point created by sensorDecision
  
      unsigned long timeNow = millis();
      unsigned long currentCheckTime = timeNow - checkTime;
      unsigned long currentPointTime = timeNow - timeArr[0]; // timeArr is used for storing times associated with outList point creation time(sensorDecision)
  
      if(currentCheckTime > 5000 && currentPointTime > 7000 && checkFlag == 66 && outList[0][0] != 254){ // if not a repeat sensordetect and time is due for update and outList[0][0] != 254
        unsigned char manipulatedPointAddr = outList[0][4] - 8; //pass first point in outList and decrement by 8 for intefacing with sensorDecision
        checkTime = millis();
        pToOutReceived = sensorDecision(manipulatedPointAddr);
        checkFlag = outList[0][4]; // new checkFlag is pointAddr of senspoint struct created by sensorDecision
      }
      
      if(checkFlag != 66){ // ignore if checkFlag is still 66(no point detect and outListPoint[0] timeArr associate <= 7 seconds)
        
        *pToFuncArray++; //increment to get second index(interandSend)
        commandsType = (**pToFuncArray) (); //interAndSend
        pToFuncArray = fparr; //set to begining
        
        if(commandsType == 1){ // commands small(commandsS)
          
          
          //F1:0,F2:1,F3:2,F4:3,F5:4,F6:5,T1:6,T2:7,T3:8,T4:9,
          //FYN1:10,FYN2:11,FYN3:12,FYN4:13,FYN5:14,FYN6:15,TYN1:16,TYN2:17,TYN3:18,TYN4:19
          
          commandsPointer = commandsS;
          commandsPointer2 = commandsPointer;
          commandsPointer2 += 6; // for getting trackDirection command(trackDirection parameter in changeTack)
        }else{
          commandsPointer = commandsB;
          commandsPointer2 = commandsPointer;
          commandsPointer2 += 10; // for getting trackDirection command(trackDirection parameter in changeTack)
        }
        inProgressFlag = 1; // proccess in proggress
        onOrOff = 1; // first command (Number in changeTrack)
      }
    } // inProgressFlag not 1
    
    if(inProgressFlag == 1){ // proccess in progress
      if(*commandsPointer2 != 254){ //if 254 there is no more commands to be sent
        if(*commandsPointer != 0){ //if 0 then skip and increment to next index
          changeTrack(*commandsPointer, onOrOff, *commandsPointer2, addrAndCom);

          //prepare commands to be sent
          addr = *addrAndComP;
          addrAndComP++;
          dataRaw = *addrAndComP;
          addrAndComP = addrAndCom;

          if(onOrOff == 0){ //increment to next command
            commandsPointer++;
            commandsPointer2++;
          }
          onOrOff = onOrOff == 1 ? 0: 1; //switch between first or last command(Number in changeTrack)
        }else{ // skip commands that is 0
          commandsPointer++;
          commandsPointer2++;
          onOrOff = 1;
        }
        
      }else{
        //break out of process
        inProgressFlag = 0;
      } 
    }

  }
   
   
   
  if(addr !=  0){
    noInterrupts();  // make sure that no interrupts occur 
    msg[1].data[0] = addr;
    msg[1].data[1] = dataRaw;
    Serial.print(msg[1].data[0]);
    Serial.print("-");
    Serial.print(msg[1].data[1]);
    Serial.print(" ");
    xdata = (msg[1].data[0]^ msg[1].data[1]);
    msg[1].data[2] = xdata;
    interrupts();
  }
   
}



unsigned char* sensorDecision(unsigned char checkFlag){ // this function creates a outList point, and also is responsible for shifting outlistArr when a update occurs
  unsigned char *pToOut; //pointer to the created outList point (will be returned and used by interAndSend)
  unsigned char lastPoint = 0; // the last point that was passed by a train
  unsigned char replacerPoint = 0; // used to replace lastPoint if new point does not contain lastpoint(two points passed)
  if(checkFlag != 66){// if checkFlag == 66 return NULL(realy does not matter, but saves time)

    sensPointStruct *temp; //pointer to struct
    sensPointStruct *tempCopy; //pointer to struct
    temp=(sensPointStruct*)malloc(sizeof(sensPointStruct)); // specifies type that will be stored, reserves memory and sets pointer to beginning
    tempCopy=(sensPointStruct*)malloc(sizeof(sensPointStruct)); // specifies type that will be stored, reserves memory and sets pointer to beginning
    if(temp==NULL || tempCopy == NULL){ // really this is kind of useless and there should be some machanism to default to annother behavior
      Serial.print("Some kind of malloc() error");
      exit(1);
    }
 
    *temp = (checkFlag == 0) ? aight : (checkFlag == 1) ? nine : (checkFlag == 2) ? ten : (checkFlag == 3) ? eleven : (checkFlag == 4) ? twelve : (checkFlag == 5) ? thirteen : (checkFlag == 6) ? fourteen : fifteen;
    *tempCopy = *temp; //used to check if new lasPoint should be pointAddr of this struct

    
    for(int i = 0; i<=9; i++){
      /** temp->pointAddr same as writing (*temp).pointAddr **/
      if(outList[i][0] == temp->pointAddr || outList[i][1] == temp->pointAddr || outList[i][2] == temp->pointAddr || outList[i][3] == temp->pointAddr || outList[i][4] == temp->pointAddr){
        //check if the right point has been found in outList and shift outList
        lastPoint = outList[i][5];
        replacerPoint = outList[i][4];
         for(int ql = i; ql<=9; ql++){
           if(outList[ql][0] == 254){
             break;
           }else if(ql < 9){
             outList[ql][0] = outList[ql+1][0];
             outList[ql][1] = outList[ql+1][1];
             outList[ql][2] = outList[ql+1][2];
             outList[ql][3] = outList[ql+1][3];
             outList[ql][4] = outList[ql+1][4];
             outList[ql][5] = outList[ql+1][5];
             timeArr[ql] = timeArr[ql+1];
           }else{
             outList[ql][0] = 254;
             outList[ql][1] = 254;
             outList[ql][2] = 254;
             outList[ql][3] = 254;
             outList[ql][4] = 254;
             outList[ql][5] = 254;
             timeArr[ql] = 0;
             break;
           }
         } 
       }
      
      
     
      for(int k = 0; k<=4; k++){
        
        if(outList[i][k] == 254 && k == 0 ){ //this will be the place the new pont will be inserted in, so we are done checking outList
          unsigned char indexes[] = {0, 0, 0, 0};
          int pp = 0;
          
          if(temp->addrArr[1] == 0 && temp->addrArr[2] == 0 && temp->addrArr[3] == 0 && temp->addrArr[4] == 0){ //there are no possible actions
            outList[i][0] = 0;
            outList[i][1] = 0;
            outList[i][2] = 0;
            outList[i][3] = 0;
          }else{
            for(int c = 0; c<=3; c++){ //get indexes of commands that are not 0
              if(temp->addrArr[c] != 0){
                pp++;
                indexes[c] = 1;
              }
            }
          }
          if(pp > 2){  // we only need two commands one in each direction
            if(indexes[0] != 0 || indexes[1] != 0){ 
              outList[i][k] = temp->addrArr[0];
              outList[i][k+1] = temp->addrArr[1];
              outList[i][k+2] = 0;
              outList[i][k+3] = 0;
            }else if(indexes[2] != 0 || indexes[3] != 0){ // we only need two commands one in each direction
              outList[i][k] = 0;
              outList[i][k+1] = 0;
              outList[i][k+2] = temp->addrArr[2];
              outList[i][k+3] = temp->addrArr[3];
            }
          }else{ // in case the above can not be accomplished there will only be 1 or two actions
              outList[i][k] = temp->addrArr[0];
              outList[i][k+1] = temp->addrArr[1];
              outList[i][k+2] = temp->addrArr[2];
              outList[i][k+3] = temp->addrArr[3];
          }
          outList[i][k+5] = lastPoint; //set lastpoint of this to the last point of what this replaced
          if(outList[i][k+5] == 0){ //this will occur if this is the first time a train passes(will occur the same number of times as there are trains)
            outList[i][k+5] = temp->pointAddr;
          }else if(tempCopy->addrArr[0] != outList[i][k+5] &&  tempCopy->addrArr[1] != outList[i][k+5] && tempCopy->addrArr[2] != outList[i][k+5] && tempCopy->addrArr[3] != outList[i][k+5]){
            outList[i][k+5] = replacerPoint; //this happens if two points was passed from last update of lastpoint
          }
          
          if(outList[i][k] == outList[i][k+5]){ //this removes the option of going back to the point that a train came from
            outList[i][k] = 0;
          }else if(outList[i][k+1] == outList[i][k+5]){
            outList[i][k+1] = 0;
          }else if(outList[i][k+2] == outList[i][k+5]){
            outList[i][k+2] = 0;
          }else if(outList[i][k+3] == outList[i][k+5]){
            outList[i][k+3] = 0;
          }
          Serial.print(" lastPoint: ");
          Serial.print(outList[i][k+5]);
          Serial.print(" ");
          outList[i][k+4] = temp->pointAddr; //sets the point addres in outList
          unsigned long tOut = millis();
          timeArr[i] = tOut; //create point of creation, so that we can update/check when time is due
          pToOut = &outList[i][0];
          Serial.print(" adresses ");
          Serial.print(outList[i][0]);
          Serial.print(outList[i][1]);
          Serial.print(outList[i][2]);
          Serial.print(outList[i][3]);
          Serial.print(outList[i][4]);
          Serial.print(" adresses ends ");
          /** Release memory **/
          free(temp), free(tempCopy); //frees memory occupied(the deallocation of memory when a process terminates is system dependent, and all memory should therefore be released like this)
          /** Release memory **/
          return (pToOut); //returns pointer to outList where this point was created
        }
        
        // check if poit is occupied and zero if so(based on senspoint type)
        if(temp->typeOff == 1){ 
          if(outList[i][k] == temp->addrArr[0]){
            temp->addrArr[0] = 0;
          }
          if(outList[i][k] == temp->addrArr[1]){
            temp->addrArr[1] = 0;
          }
          temp->addrArr[2] = 0;
          temp->addrArr[3] = 0;
          
        }else{
          if(outList[i][k] == temp->addrArr[0]){
            temp->addrArr[0] = 0;
          }
          if(outList[i][k] == temp->addrArr[1]){
            temp->addrArr[1] = 0;
          }
          if(outList[i][k] == temp->addrArr[2]){
            temp->addrArr[2] = 0;
          }
          if(outList[i][k] == temp->addrArr[3]){
            temp->addrArr[3] = 0;
          }
        }
      
      }
    }
  }
}

unsigned char sensorDetectAndQuar(){ // checking sensorpins, make sure no repeats in timeframe, remove point from quarantine if time is due
  unsigned char checkFlag = 66; // ussed to controll many procceses. Returnes the sensorpin if a detect was made
  /*for checking sensorpins and doing timecheck, so we have no repeats */ 
  for(int i = 0; i<=7; i++){
    valFlag = digitalRead(sensPins[i]); //read pins
    
    if(valFlag == 0){ //if sensor is activated

      if(i != quarantineArr[0] && i != quarantineArr[1] && i != quarantineArr[2]){ // if sensor not in quaranrinArr
        
          for(int k = 0; k<=2; k++){

            if(quarantineArr[k] == 66){ //upadte quarantineArr and its quarantineTimes associate, and return the senspin number
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
      if(elapsedTime > 4000){ // if 4 seconds or more passed from last detect, remove from quranrantineArr and quarantineTimes
        quarantineArr[k] = 66;
        quarantineTimes[k] = 0;
      }
    }
  }/*removing sensorpins from quarintine lists if time is due */ 
  return(checkFlag); // return checkflag that is used by sensorDecision and controll in assemble_dcc_msg
}

unsigned char interAndSend(){ // interprets the outList point that was created by sensorDecision based on pointer returned by sensorDecision.

  unsigned char* dPToReceived = pToOutReceived;
  pToOutReceived+=4;
  unsigned char pointAddr = *pToOutReceived; // get pointAddr
  pToOutReceived = dPToReceived;
  
  //create sensPointstruct based on pointAddr
  sensPointStruct temp = (pointAddr == 8) ? aight : (pointAddr == 9) ? nine : (pointAddr == 10) ? ten : (pointAddr == 11) ? eleven : (pointAddr == 12) ? twelve : (pointAddr == 13) ? thirteen : (pointAddr == 14) ? fourteen : fifteen;
  
  if(temp.typeOff == 1){ // this is the procedure for points of type small(1)
    
    //F1:0,F2:1,F3:2,F4:3,T1:4,T2:5,FYN1:6,FYN2:7,FYN3:8,FYN4:9,TYN1:10,TYN2:11(structure of commandsS)
    unsigned char* clearerPointer = commandsS;
    for(int i = 0; i<=11; i++){//zero everything
      *clearerPointer = 0;
      clearerPointer++;
    }*clearerPointer = 254;

    //set inner lights(as information always will be needed to be communicated to theese)
    commandsS[0] = temp.lights[0];
    commandsS[1] = temp.lights[1];
    
    for(int i = 0; i<=1; i++){
      if(*pToOutReceived != 0 && i == 0){ // free pass to one side(set outer light of this side)
        commandsS[2] = temp.lights[2];
      }else if(*pToOutReceived != 0 && i == 1){ // free pass to one side(set outer light of this side)
        commandsS[3] = temp.lights[3];
      }
      pToOutReceived++;
    }pToOutReceived = dPToReceived;
      
    if(*pToOutReceived != 0){ //free pass to one side, set track of this side, and set its "trackDirection" to 1. also set associted light on this side to 1(trackDirection)
      commandsS[4] = temp.tracks[0];
      commandsS[10] = 1;
      
      commandsS[6] = 1;
      commandsS[8] = 1;
     }
    pToOutReceived++;
    if(*pToOutReceived != 0){//free pass to one side, set track of this side, and set its "trackDirection" to 1. also set associted light on this side to 1(trackDirection)
      commandsS[5] = temp.tracks[1];
      commandsS[11] = 1;
      
      commandsS[7] = 1;
      commandsS[9] = 1;
    }
    pToOutReceived = NULL; //just making sure here, but probably not needed
    return(1); //return the type of senspointstruct (small, 1)
  }else{ //sensePointType big(2)
    
    unsigned char* clearerPointer = commandsB;
    for(int i = 0; i<=19; i++){ //zero everything
      *clearerPointer = 0;
      clearerPointer++;
    }*clearerPointer = 254;
    
          
    //set inner lights
    commandsB[0] = temp.lights[0];
    commandsB[1] = temp.lights[1];
    

    for(int i = 0; i<=3; i++){//set lights based on commands in outList
      if(*pToOutReceived != 0 && i == 0){
        commandsB[2] = temp.lights[2];
      }else if(*pToOutReceived != 0 && i == 1){
        commandsB[3] = temp.lights[3];
      }else if(*pToOutReceived != 0 && i == 2){
        commandsB[4] = temp.lights[4];
      }else if(*pToOutReceived != 0 && i == 3){
        commandsB[5] = temp.lights[5];
      }
      pToOutReceived++;
    }pToOutReceived = dPToReceived;
    
    //Structure of commandsB
    //F1:0,F2:1,F3:2,F4:3,F5:4,F6:5,T1:6,T2:7,T3:8,T4:9,
    //FYN1:10,FYN2:11,FYN3:12,FYN4:13,FYN5:14,FYN6:15,TYN1:16,TYN2:17,TYN3:18,TYN4:19
    for(int i = 0; i<=3; i++){//set lights to on according to outList com data
      if(i == 0 && *pToOutReceived != 0){
        commandsB[10] = 1;
        commandsB[12] = 1;
      }else if(i == 1 && *pToOutReceived != 0){
        commandsB[11] = 1;
        commandsB[13] = 1;
      }else if(i == 2 && *pToOutReceived != 0){
        commandsB[10] = 1;
        commandsB[14] = 1;
      }else if(i == 3 && *pToOutReceived != 0){
        commandsB[11] = 1;
        commandsB[15] = 1;
      }
      pToOutReceived++;
    }
    pToOutReceived = dPToReceived;
    
    
    //F1:0,F2:1,F3:2,F4:3,F5:4,F6:5,T1:6,T2:7,T3:8,T4:9,
    //FYN1:10,FYN2:11,FYN3:12,FYN4:13,FYN5:14,FYN6:15,TYN1:16,TYN2:17,TYN3:18,TYN4:19
    
    for(int i = 0; i<=1; i++){ // set inner tracks based on outList data
      if(*pToOutReceived != 0){
        commandsB[i+6] = temp.tracks[i];
        commandsB[i+16] = 1;
      }
      pToOutReceived++;
    }
    
    if(*pToOutReceived != 0){ //set outer tracks and reset inner tracks, based on outList data
      commandsB[6] = temp.tracks[0];
      commandsB[16] = 2;
      
      commandsB[8] = temp.tracks[2];
      commandsB[18] = 1;
    }
    pToOutReceived++;
    if(*pToOutReceived != 0){ //set outer tracks and reset inner tracks, based on outList data
      
      commandsB[7] = temp.tracks[1];
      commandsB[17] = 2;
      
      commandsB[9] = temp.tracks[3];
      commandsB[19] = 1;
    }
  }
  
  pToOutReceived = NULL;
  return(2); //return type of command
  
}



void changeTrack(unsigned char trackAddr, int number, int trackDirection, unsigned char *assignmentP){ //for creating data for trains, lights and tracks
  unsigned char addr;
  unsigned char com;

  unsigned char fixed = 63;
  
  int regAddr = ((trackAddr % 4) -1);
  unsigned char accAdr = ((trackAddr / 4));

   if(regAddr < 0){
    regAddr = 3;
  }else{
    accAdr++;
  }
  accAdr = accAdr & fixed;

  //addr
  for(int i = 7; i>=0; i--){ 
    if(((accAdr>>i) & 1) == 1){
      addr |= 1 << i; //set bit of addr at position i to 1
      // this is the same as (addr = addr | 1 << i)
      // 1 is first shifted by i
      // then it is or'ed with addr
      // In general, (1 << i) | addr.
    }else{
      addr &= ~(1 << i); //set bit of addr at position i to 0
      //this is the same as addr = addr & ~(1 << i)
      // first 1 is shifted by i
      //then the result i not'ed
      // then the result is and'ed with addr
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

   if( (regAddr>>1 & 1) == 1 && (regAddr>>0 & 1) == 0 ){ //2
     com |= 1 << (2);
     com &= ~(1<<(1));
   }else if( (regAddr>>1 & 1) == 0 && (regAddr>>0 & 1) == 1 ){ //1
     com |= 1 << (1);
     com &= ~(1<<(2));
   }else if( (regAddr>>1 & 1) == 1 && (regAddr>>0 & 1) == 1){ // 3
     com |= 1 << (1);
     com |= 1 << (2);
   }else{ // 0
     com &= ~(1<<(1));
     com &= ~(1<<(2));
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

/* union */ 
/*

 The size of the union will be the size of the biggest member, as all the members will be stored in the same memory slot.

union Data {
   int i;
   float f;
   char str[20];
};
 
int main( ) {

   union Data data;        

   data.i = 10;
   data.f = 220.5;
   strcpy( data.str, "C Programming");

   printf( "data.i : %d\n", data.i);
   printf( "data.f : %f\n", data.f);
   printf( "data.str : %s\n", data.str);

   return 0;
}
When the above code is compiled and executed, it produces the following result −
data.i : 1917853763
data.f : 4122360580327794860452759994368.000000
data.str : C Programming


 
union Data {
   int i;
   float f;
   char str[20];
};
 
int main( ) {

   union Data data;        

   data.i = 10;
   printf( "data.i : %d\n", data.i);
   
   data.f = 220.5;
   printf( "data.f : %f\n", data.f);
   
   strcpy( data.str, "C Programming");
   printf( "data.str : %s\n", data.str);

   return 0;
}
When the above code is compiled and executed, it produces the following result −
data.i : 10
data.f : 220.500000
data.str : C Programming
*/
/* union */
