arduino
#include <Servo.h>
#include <ArduinoOpenCV.h>

#define CAMERA_RX  3  // RX pin of the camera module
#define CAMERA_TX  4  // TX pin of the camera module

Servo servoX;
Servo servoY;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);  // Initialize serial communication with the camera module
  while (!Serial);      // Wait for serial port to connect

  servoX.attach(9);  // Attach servo to pin 9
  servoY.attach(10); // Attach servo to pin 10

  // Initialize ArduinoOpenCV library with camera pins
  ArduCAM myCAM(OV2640, CAMERA_RX, CAMERA_TX);
  myCAM.InitCAM();
}

void loop() {
  // Capture image from the camera module
  myCAM.ArduCamCapture();

  // Detect faces in the captured image
  FaceDetectionResult *fdResult = ArduCamFaceDetection();

  if (fdResult->faceDetected) {
    // If face is detected, move the servos
    int posX = fdResult->x;
    int posY = fdResult->y;
    
    // Map coordinates to servo angles (adjust as needed)
    int servoXAngle = map(posX, 0, 640, 0, 180); // Map X-coordinate to servo angle
    int servoYAngle = map(posY, 0, 480, 0, 180); // Map Y-coordinate to servo angle
    
    // Move the servos
    servoX.write(servoXAngle);
    servoY.write(servoYAngle);
  }
  else {
    // If no face is detected, stop the servos
    servoX.write(90); // Center position
    servoY.write(90); // Center position
  }
}