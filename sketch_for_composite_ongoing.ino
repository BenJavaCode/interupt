unsigned char trainAddresses[] = {7,8,40};
int ini = 0;


unsigned char outList[10][5] = {254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254,
                               254, 254, 254, 254, 254
                               };

struct sensPoint{
  unsigned char pointAddr;
  unsigned char lights[2];
  unsigned char addrArr[4];
  unsigned char typeOff;
  unsigned char tracks[4];
};


unsigned char sensPins[] = {3,5,6,7,8,9,10,11};

typedef struct sensPoint sensPointStruct;

unsigned long quarantineArr[] = {66,66,66};
unsigned long quarantineTimes[3];

unsigned char valFlag = 0;


sensPointStruct three = {
  3, {141, 142}, {5,7}, 's', {250, 252}
};

sensPointStruct five = {
  5, {122, 121}, {3,6,8,10}, 'b', {250, 249, 242, 241}
};

sensPointStruct six = {
  6, {131, 132}, {5,7}, 's', {249, 251}
};

sensPointStruct seven = {
  7, {152, 151}, {3,6,8,10}, 'b', {252, 251, 244, 243}
};

sensPointStruct aight = {
  8, {101, 102}, {5,7,9,11}, 'b', {250, 252, 242, 244}
};

sensPointStruct nine = {
  9, {82, 81}, {8,10}, 's', {242, 241}
};

sensPointStruct ten = {
  10, {91, 92}, {5,7,9,11}, 'b', {249, 251, 241, 243}
};

sensPointStruct eleven = {
  11, {112, 111},{8,10}, 's', {244, 243}
};


void setup() {
  Serial.begin(9600);

   for(int i = 0; i<=7; i++){
    pinMode(sensPins[i], INPUT_PULLUP);
  }
  
}

void loop() {

  unsigned char checkFlag = 66;


  /* set all the trains in motion at the begining*/
   if(ini <= 2){
    dataRaw = 95;
    addr = trainAddresses[ini]; 
    ini++;
   }
  
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

  if(checkFlag != 66){
    sensPointStruct temp = (checkFlag == 0) ? three : (checkFlag == 1) ? five : (checkFlag == 2) ? six : (checkFlag == 3) ? seven : (checkFlag == 4) ? aight : (checkFlag == 5) ? nine : (checkFlag == 6) ? ten : eleven;
    //Serial.print(temp.pointAddr);
    unsigned char outArr[5];

  
    for(int i = 0; i<=9; i++){
      for(int k = 0; k<=4; k++){
      
        if(outList[i][k] == 254 && i == 0 ){
          break;
        }

        if(temp.typeOff == 's'){
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
          if(outList[i][k] == temp.addrArr[0]){
            temp.addrArr[2] = 0;
          }
          if(outList[i][k] == temp.addrArr[1]){
            temp.addrArr[3] = 0;
          }
        }
      
      }
    }
    
    if(temp.addrArr[1] == 0 && temp.addrArr[2] == 0 && temp.addrArr[3] == 0 && temp.addrArr[4] == 0){
     outArr[0] = temp.pointAddr;
   }else{
      for(int i = 0; i<=3; i++){
        outArr[i] = temp.addrArr[i];
      }outArr[4] = temp.pointAddr;
    }
    for(int i = 0; i<=4; i++){
      Serial.print(outArr[i]);
      Serial.print(',');
    }
  }
  
  
  

  

  

}
