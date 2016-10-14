#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <Servo.h>
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

const int SFSensor = A0;  // Analog input pin that the potentiometer is attached to
const int SLSensor = A1;  // Analog input pin that the potentiometer is attached to
const int SRSensor = A2;  // Analog input pin that the potentiometer is attached to
const int LFSensor = A3;  // Analog input pin that the potentiometer is attached to

float SFValue = 0;    
float SLValue = 0;    
float SRValue = 0;    
float LFValue = 0;    

Servo LServo;
Servo RServo;
int startTime;
void setup() {
  startTime = millis();
  // initialize serial communications at 9600 bps:
  LServo.attach(2);
  RServo.attach(3);
  Serial.begin(9600);
  lcd.begin(16, 2);
}

void loop() {
  closeLoopCtrlPart1(5, 20);

  //closeLoopCtrlPart2(5, 5);
  
  //closeLoopCtrlPart3(5, 3);
}



void closeLoopCtrlPart1(float rt, float kp){
  float yt = sensorRead("front");
  int etl = rt - yt;
  int utl = kp * etl;

  int etr = yt - rt;
  int utr = kp * etr;
  
  int leftServoValue = saturationFunctionFront(utl);
  int rightServoValue = saturationFunctionFront(utr);

  Serial.println(leftServoValue);
  Serial.println(millis() - startTime);
  RServo.write(rightServoValue);
  LServo.write(leftServoValue);
}


void closeLoopCtrlPart2(float rt, float kp){
  //Control Loop for front sensor
  float ytf = sensorRead("front");

  //Check front distance and set left servo
  float etfl = rt - ytf;
  float utfl = kp * etfl;

  //Check front distance and set right servo
  float etfr = ytf - rt;
  float utfr = kp * etfr;

  float ytr = sensorRead("right");
  
  //Check right distance. Set left
  float etrl = rt - ytr;
  float utrl = kp * etrl;

  //Check right distance. Set Right
  float etrr = ytr - rt;
  float utrr = kp * etrr;

  int leftServoValue = saturationFunctionFront(utfl) + saturationFunctionRightLeft(utrl);
  int rightServoValue = saturationFunctionFront(utfr) - saturationFunctionRightLeft(utrr);

  if (leftServoValue <= 93 && leftServoValue >= 87){
    if (rightServoValue <= 93 && rightServoValue >= 87){
      rightServoValue = 80;
      leftServoValue = 80;
      RServo.write(rightServoValue);
      LServo.write(leftServoValue);
      delay(700);
    }
  }
  else{
    RServo.write(rightServoValue);
    LServo.write(leftServoValue); 
  }

}

void closeLoopCtrlPart3(float rt, float kp){
   //Control Loop for front sensor
  float ytf = sensorRead("front");

  //Check front distance and set left servo
  float etfl = rt - ytf;
  float utfl = kp * etfl;

  //Check front distance and set right servo
  float etfr = ytf - rt;
  float utfr = kp * etfr;




  float ytr = sensorRead("right");
  
  //Check right distance. Set left
  float etrl = rt - ytr;
  float utrl = kp * etrl;

  //Check right distance. Set Right
  float etrr = ytr - rt;
  float utrr = kp * etrr;



  float ytl = sensorRead("left");
  
  //Check left distance. Set Left
  float etll = rt - ytl;
  float utll = kp * etll;

  //Check left distance. Set Right
  float etlr = ytl - rt;
  float utlr = kp * etlr;

  int leftServoValue = saturationFunctionFront(utfl) + saturationFunctionRightLeft(utrl) - saturationFunctionRightLeft(utll);
  int rightServoValue = saturationFunctionFront(utfr) - saturationFunctionRightLeft(utrr) + saturationFunctionRightLeft(utlr);

  if (leftServoValue <= 93 && leftServoValue >= 87){
    if (rightServoValue <= 93 && rightServoValue >= 87){
      rightServoValue = 100;
      leftServoValue = 100;
      RServo.write(rightServoValue);
      LServo.write(leftServoValue);
      delay(200);
    }
  }
  else{
    RServo.write(rightServoValue);
    LServo.write(leftServoValue); 
  }
  
}





int saturationFunctionRightLeft(double val){
  int returnVal = 0;
  if (returnVal - val <= -8){
    return -8;
  }

  else if (returnVal - val >= 8){
    return 8;
  }

  else
    return returnVal - val;
}

int saturationFunctionFront(double val){
  int returnVal = 90;
  if (returnVal - val <= 80){
    return 80;
  }
  else if (returnVal - val >= 100){
    return 100;
  }
  else
    return returnVal - val;
}


float sensorRead(String sensorDirection){
  float sensorValue = 0;
  if (sensorDirection.equals("front")){
    sensorValue = 12.509 * pow(analogRead(SFSensor)*0.0048875855327468, -1.059) / 2.54;
    if (sensorValue > 11.80){
      sensorValue = 59.635 * pow(analogRead(LFSensor)*0.0048875855327468, -1.034) / 2.54;
      if (sensorValue > 59){
        sensorValue = 59;
      }
    }
  }

  else if (sensorDirection.equals("left")){
    sensorValue = 12.509 * pow(analogRead(SLSensor)*0.0048875855327468, -1.059) / 2.54;
    if (sensorValue > 11.80){
      sensorValue = 11.80;
    }
  }

   else if (sensorDirection.equals("right")){
    sensorValue = 12.509 * pow(analogRead(SRSensor)*0.0048875855327468, -1.059) / 2.54;
    if (sensorValue > 11.80){
      sensorValue = 11.80;
    }
  }
  else{
    sensorValue = -1;
  }

  
  return sensorValue;
}

