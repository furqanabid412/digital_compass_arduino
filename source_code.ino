#include <Wire.h>
#include <Servo.h>
#include <movingAvg.h> 

Servo myservo;

byte _ADDR1 = 0x0D;//address of compass
byte _ADDR2 = 0x68;//address of MPU6050

//MPU functions var
int16_t rawAccX, rawAccY, rawAccZ, rawTemp,rawGyroX, rawGyroY, rawGyroZ;
float gyroXoffset, gyroYoffset, gyroZoffset, gyroZ;;

//Servo speed control var
float servoSpeedZeroOffset = 90;
float servoControlOutput = 0;

//Loop var
float targetAngle = 0;
float azimuthalAngle;
float angularSpeed;
float angularAccelaration;

//PID variables
long lastTime;
double Input, Output, Setpoint;
double errSum,dErr, lastErr;
double error;
long now;
double timeChange;

float Kp = 0.4;  float Ki = 0; float Kd = 0.1;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  myservo.attach(9);
  
  //Device Check
  if (I2cCheck(_ADDR1)&I2cCheck(_ADDR2))
  Serial.print("Both Compass and GyroAccelerometer found");
  else
  {
    if (!I2cCheck(_ADDR1))
      Serial.print("Compass not found");
    if(!I2cCheck(_ADDR2))
      Serial.print("MPU6050 not found");
  }
   azimuthalAngle = 0;
   errSum=0;
   initialCompass();
   initialMPU(); 
}


void loop() {
    // Read values from two sensors
    angularSpeed = readvelocity();
  azimuthalAngle = readCompass();

//   if (azimuthalAngle==0){
//       myservo.write(servoSpeedZeroOffset);
//       errSum=0;
//   }
//    else
//    {
    //PID control begin
    now = millis();
    timeChange=now-lastTime;
    error=targetAngle - convertAngle(azimuthalAngle);
    errSum +=(error*timeChange);
    dErr=angularSpeed;
    servoControlOutput=Kp*error+Ki*errSum+Kd*dErr;
   Serial.println(servoSpeedZeroOffset-servoControlOutput);
   myservo.write(servoSpeedZeroOffset-servoControlOutput);  
   lastErr=error;
   lastTime=now;
//   }
}


////////////////////////////////////////////////////////////

// I2C check
bool I2cCheck(byte add){
   Wire.beginTransmission(add);
    byte error = Wire.endTransmission();
     if (error == 0)
      return 1;
     else
     return 0;
  }

//convert 0-360 to (-180)-180
  float convertAngle(float angle) {
  if (angle > 180) {
    return angle - 360;
  }
  return angle;
}
 ///////////////////////////////////////////////////////////

void initialMPU(){
  writeRegMPU(0x19,0x00);
  writeRegMPU(0x1a, 0x00);
  writeRegMPU(0x1b, 0x08);
  writeRegMPU(0x1c, 0x00);
  writeRegMPU(0x6b, 0x01);
  setGyroOffsets(-1.13, 1.13, 0.62);
  readvelocity();
}

float readvelocity(){
  Wire.beginTransmission(_ADDR2);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((int)_ADDR2, 14);
  
  rawAccX = Wire.read() << 8 | Wire.read();
  rawAccY = Wire.read() << 8 | Wire.read();
  rawAccZ = Wire.read() << 8 | Wire.read();
  rawTemp = Wire.read() << 8 | Wire.read();
  rawGyroX = Wire.read() << 8 | Wire.read();
  rawGyroY = Wire.read() << 8 | Wire.read();
  rawGyroZ = Wire.read() << 8 | Wire.read();
  
  gyroZ = ((float)rawGyroZ) / 65.5;
  gyroZ -= gyroZoffset;
  return gyroZ;
}

void setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}


//Write on register of MPU
void writeRegMPU(byte r, byte v){
  Wire.beginTransmission(_ADDR2);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission();
}

 ///////////////////////////////////////////////////////////
 
 int readCompass()
{
 int _vRaw[3] = {0,0,0};

  Wire.beginTransmission(_ADDR1);
  Wire.write(0x00);
  int err = Wire.endTransmission();
  if (!err) {
    Wire.requestFrom(_ADDR1, (byte)6);
    _vRaw[0] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    _vRaw[1] = (int)(int16_t)(Wire.read() | Wire.read() << 8);
    _vRaw[2] = (int)(int16_t)(Wire.read() | Wire.read() << 8); 
  }
  int a = atan2(_vRaw[1],_vRaw[0] ) * 180.0 / PI;
  return a < 0 ? 360 + a : a;
}

//Initializing the compass
void initialCompass(){
  writeRegCompass(0x0B,0x01);
  writeRegCompass(0x09,0x01|0x0C|0x10|0X00);//setmode
}

//Write on register of compass
void writeRegCompass(byte r, byte v){
  Wire.beginTransmission(_ADDR1);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission();
}
