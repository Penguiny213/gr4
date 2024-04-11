#include <PID.h>
#include <encoder.h>

#define spindle 12
#define pinpwmX 3
#define motorX1 4
#define motorX2 5
#define pinpwmY 6
#define motorY1 7
#define motorY2 8
#define pinpwmZ 9
#define motorZ1 10
#define motorZ2 11

encoder myencoderX(A0,A1,INPUTPULLUP);
encoder myencoderY(A2,A3,INPUTPULLUP);
encoder myencoderZ(A4,A5,INPUTPULLUP);

PID myPIDX(16, 0, 0); // KP, KI, KD -- adjust the PID parameter with the DC motor you have
PID myPIDY(150, 0, 0); // KP, KI, KD -- adjust the PID parameter with the DC motor you have
PID myPIDZ(100, 0, 0); // KP, KI, KD -- adjust the PID parameter with the DC motor you have

byte c;

double setpointX = 0,
       setpointY = 0,
       setpointZ = 0,
       lastinputX = 0,
       lastinputY = 0,
       lastinputZ = 0,
       inputX = 0,
       inputY = 0,
       inputZ = 0,
       pwmX,
       pwmY,
       pwmZ;

void setup(){
  
  Serial.begin(115200);
  pinMode(spindle, OUTPUT);
  pinMode(pinpwmX, OUTPUT);
  pinMode(motorX1, OUTPUT);
  pinMode(motorX2, OUTPUT);
  pinMode(pinpwmY, OUTPUT);
  pinMode(motorY1, OUTPUT);
  pinMode(motorY2, OUTPUT);
  pinMode(pinpwmZ, OUTPUT);
  pinMode(motorZ1, OUTPUT);
  pinMode(motorZ2, OUTPUT);
  
  myPIDX.LimitP(255, -255); //Pmax , Pmin
  myPIDX.LimitI(127, -127); //Imax , Imin
  
  myPIDY.LimitP(255, -255); //Pmax , Pmin
  myPIDY.LimitI(127, -127); //Imax , Imin
  
  myPIDZ.LimitP(255, -255); //Pmax , Pmin
  myPIDZ.LimitI(127, -127); //Imax , Imin
  
  delay(1000);
  Serial.write('R'); //send R to computer. Arduino Ready....
  
}

void loop(){
  
  cekSerial();
  
  inputX = myencoderX.baca();
  inputY = myencoderY.baca();
  inputZ = myencoderZ.baca();

  pwmX = myPIDX.Calculate(setpointX, inputX);
  pwmY = myPIDY.Calculate(setpointY, inputY);
  pwmZ = myPIDZ.Calculate(setpointZ, inputZ);
  
  if(pwmX<0){
    analogWrite(pinpwmX, pwmX * -1);
    digitalWrite(motorX1, LOW);
    digitalWrite(motorX2, HIGH);
  }else{
    analogWrite(pinpwmX, pwmX);
    digitalWrite(motorX1, HIGH);
    digitalWrite(motorX2, LOW);
  }
  
  if(pwmY<0){
    analogWrite(pinpwmY, pwmY * -1);
    digitalWrite(motorY1, LOW);
    digitalWrite(motorY2, HIGH);
  }else{
    analogWrite(pinpwmY, pwmY);
    digitalWrite(motorY1, HIGH);
    digitalWrite(motorY2, LOW);
  }
  
  if(pwmZ<0){
    analogWrite(pinpwmZ, pwmZ * -1);
    digitalWrite(motorZ1, LOW);
    digitalWrite(motorZ2, HIGH);
  }else{
    analogWrite(pinpwmZ, pwmZ);
    digitalWrite(motorZ1, HIGH);
    digitalWrite(motorZ2, LOW);
  }

/*
  if(inputX != lastinputX){
    if(inputX > lastinputX){
      Serial.write('a');
    }else{
      Serial.write('b');
    }
    lastinputX = inputX;  
  }

  if(inputY != lastinputY){
    if(inputY > lastinputY){
      Serial.write('c');
    }else{
      Serial.write('d');
    }
    lastinputY = inputY;  
  }

  if(inputZ != lastinputZ){
    if(inputZ > lastinputZ){
      Serial.write('e');
    }else{
      Serial.write('f');
    }
    lastinputZ = inputZ;  
  }
  */
/*
 
  Serial.print("Setpoint: ");
  Serial.print(setpointX);
  Serial.print(" Input: ");
  Serial.print(inputX);
  Serial.print(" Pwm: ");
  Serial.println(pwmX);
*/

}

void cekSerial(){
  
  if(Serial.available()>0){
    c = Serial.read();
    
    if(c == 1){ //if Computer send 1 spindle ON
      digitalWrite(spindle, HIGH);
    }
    
    if(c == 2){ //if Computer send 2 spindle OFF
      digitalWrite(spindle, LOW);
    }
    
    if(c == 4){ //if Computer send 4 SetpointX + 1;
      setpointX++;
    }
    
    if(c == 8){ //if Computer send 8 SetpointX - 1;
      setpointX--;
    }
    
    if(c == 16){ //if Computer send 16 SetpointY + 1;
      setpointY++;
    }
    
    if(c == 32){ //if Computer send 32 SetpointY - 1;
      setpointY--;
    }
    
    if(c == 64){ //if Computer send 64 SetpointZ + 1;
      setpointZ++;
    }
    
    if(c == 128){ //if Computer send 128 SetpointZ - 1;
      setpointZ--;
    }
    
  }//end serial available
}

