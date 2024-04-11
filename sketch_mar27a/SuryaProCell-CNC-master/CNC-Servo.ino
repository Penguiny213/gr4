#include <PID.h>
#include <encoder1.h>
#include <Servo.h>

#define spindle 13

encoder myencoderX(A0,A1);
encoder myencoderY(A2,A3);
encoder myencoderZ(A4,A5);

Servo myservoX;
Servo myservoY;
Servo myservoZ;

int centerServoX = 90;
int centerServoY = 90;
int centerServoZ = 90;

PID myPIDX(2,0,0); // KP, KI, KD
PID myPIDY(2,0,0); // KP, KI, KD
PID myPIDZ(2,0,0); // KP, KI, KD

byte c;

double setpointX = 0,
       setpointY = 0,
       setpointZ = 0,
       lastsetpointX = 0,
       lastsetpointY = 0,
       lastsetpointZ = 0,
       inputX = 0,
       inputY = 0,
       inputZ = 0,
       pwmX,
       pwmY,
       pwmZ;

void setup(){
  
  Serial.begin(115200);
  pinMode(spindle, OUTPUT);
  
  myservoX.attach(8);
  myservoY.attach(9);
  myservoZ.attach(10);
  
  myPIDX.LimitP(7,-7); //Pmax , Pmin
  myPIDX.LimitI(1,-1); //Imax , Imin
  
  myPIDY.LimitP(7,-7); //Pmax , Pmin
  myPIDY.LimitI(1,-1); //Imax , Imin
  
  myPIDZ.LimitP(7,-7); //Pmax , Pmin
  myPIDZ.LimitI(1,-1); //Imax , Imin
  
  myservoX.write(90);
  myservoY.write(90);
  myservoZ.write(90);
  
  delay(1000);
  Serial.write('R'); //send R to computer. Arduino Ready....
  
}

void loop(){
  
  cekSerial();
  
  inputX = myencoderX.baca();
  inputY = myencoderY.baca();
  inputZ = myencoderZ.baca();

  pwmX = centerServoX + myPIDX.Calculate(setpointX, inputX);
  pwmY = centerServoY + myPIDY.Calculate(setpointY, inputY);
  pwmZ = centerServoZ + myPIDZ.Calculate(setpointZ, inputZ);
  
  myservoX.write(pwmX);
  myservoY.write(pwmY);
  myservoZ.write(pwmZ);
  /*
  if(lastsetpointX != setpointX){
    if(inputX==setpointX){
      Serial.write('X');
      lastsetpointX = setpointX;
    }
  }
  if(lastsetpointY != setpointY){
    if(inputY==setpointY){
      Serial.write('Y');
      lastsetpointY = setpointY;
    }
  }
  if(lastsetpointZ != setpointZ){
    if(inputZ==setpointZ){
      Serial.write('Z');
      lastsetpointZ = setpointZ;
    }
  }
  */
}

void cekSerial(){
  
  if(Serial.available()>0){
    c = Serial.read();
    
    if(c == 1){ //if processing send 1 spindle ON
      digitalWrite(spindle, HIGH);
      Serial.write('1');
    }//end if c = 1
    
    if(c == 2){ //if processing send 2 spindle OFF
      digitalWrite(spindle, LOW);
      Serial.write('2');
    }//end if c = 2
    
    if(c == 4){ //if computer send 4 SetpointX + 1;
      setpointX++;
    }//end if c = 3
    
    if(c == 8){ //if computer send 8 SetpointX - 1;
      setpointX--;
    }//end if c = 4
    
    if(c == 16){ //if computer send 16 SetpointY + 1;
      setpointY++;
    }//end if c = 5
    
    if(c == 32){ //if computer send 32 SetpointY - 1;
      setpointY--;
    }//end if c = 6
    
    if(c == 64){ //if computer send 64 SetpointZ + 1;
      setpointZ++;
    }//end if c = 7
    
    if(c == 128){ //if computer send 128 SetpointZ - 1;
      setpointZ--;
    }//end if c = 8
    
  }//end serial available
}


