//连线方法
//MPU-UNO
//VCC-VCC
//GND-GND
//SCL-A5
//SDA-A4
//INT-2 (Optional)
#include <Stepper.h>
#include <Kalman.h>
#include <Wire.h>
#include <Math.h>

float fRad2Deg = 57.295779513f; //将弧度转为角度的乘数
const int MPU = 0x68; //MPU-6050的I2C地址
const int nValCnt = 7; //一次读取寄存器的数量
//电机步进定义
const int stepsPerRevolution = 64;
//手动控制电机按钮定义
int pushButton1 = 2;
int pushButton2 = 3;
//LED控制相关定义
const int buttonPin = 12;
const int LEDPin = 13;
int LEDState = HIGH;
int buttonState;
int lastButtonState = LOW;
long lastDebounceTime = 0;
long debounceDelay = 50;
// 初始化步进电机要使用的Arduino的引脚编号
Stepper myStepper1(stepsPerRevolution, 4, 5, 6, 7);
Stepper myStepper2(stepsPerRevolution, 8, 9, 10, 11);
float deltapitch[3],mi;

const int nCalibTimes = 1000; //校准时读数的次数
int calibData[nValCnt]; //校准数据

unsigned long nLastTime = 0; //上一次读数的时间
float fLastRoll = 0.0f; //上一次滤波得到的Roll角
float fLastPitch = 0.0f; //上一次滤波得到的Pitch角
float fLastYaw = 0.0f;
Kalman kalmanRoll; //Roll角滤波器
Kalman kalmanPitch; //Pitch角滤波器

void setup() {
  Serial.begin(9600); //初始化串口，指定波特率
  Wire.begin(); //初始化Wire库
  WriteMPUReg(0x6B, 0); //启动MPU6050设备
  
  Calibration(); //执行校准
  nLastTime = micros(); //记录当前时间
  // 设置转速，步进/分钟
  myStepper1.setSpeed(256);
   // 设置转速，每分钟为128步
  myStepper2.setSpeed(128);
  //LED设定
  pinMode(buttonPin, INPUT);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LEDState);
}

void loop() {
  int readouts[nValCnt];
  ReadAccGyr(readouts); //读出测量值
  
  float realVals[7];
  Rectify(readouts, realVals); //根据校准的偏移量进行纠正

  //计算加速度向量的模长，均以g为单位
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  float fRoll = GetRoll(realVals, fNorm); //计算Roll角
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
  float fPitch = GetPitch(realVals, fNorm); //计算Pitch角
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }

  //计算两次测量的时间间隔dt，以秒为单位
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;
  //对Roll角和Pitch角进行卡尔曼滤波
  float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
  float fNewYaw = realVals[6] * dt; //计算Yaw角
  //跟据滤波值计算转过角度
  float deltaRoll = (fNewRoll - fLastRoll);
  float deltaPitch = (fNewPitch - fLastPitch);
  float deltaYaw = fNewYaw - fLastYaw ;
  deltapitch[0]=deltaPitch;
  deltapitch[1]=deltaPitch+360;
  deltapitch[2]=deltaPitch-360;
  if(abs(deltaPitch)>abs(deltapitch[1]))
      deltaPitch=deltapitch[1];
  if(abs(deltaPitch)>abs(deltapitch[2]))
      deltaPitch=deltapitch[2];
 //更新Roll角和Pitch角
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  fLastYaw = fNewYaw;
  //更新本次测的时间
  nLastTime = nCurTime;
  if( deltaPitch > 0.0878){
    myStepper1.step( -deltaPitch / 0.0878);
    delay(0);
  }
  if(  deltaPitch < -0.0878){
    myStepper1.step( -deltaPitch / 0.0878);
    delay(0);
  }
  //手控电机控制循环
  int buttonState1 = digitalRead(pushButton1);
  int buttonState2 = digitalRead(pushButton2);
    
  if(buttonState2 < 1){
       myStepper2.step( buttonState1 );
       delay(0);
    }
  if(buttonState1 < 1){
    //Serial.println("counterclockwise");
     myStepper2.step( -buttonState2 );
     delay(0);
    }
  //LED控制循环
  int reading = digitalRead(buttonPin);
  if(reading != lastButtonState){
    lastDebounceTime = millis();
  }
  if((millis() - lastDebounceTime) > debounceDelay){
    if(reading != buttonState){
      buttonState = reading;
      if(buttonState == HIGH){
        LEDState =! LEDState;
      }
    }
  }
  digitalWrite(LEDPin, LEDState);
  lastButtonState = reading;
  //向串口打印输出Roll角和Pitch角，运行时在Arduino的串口监视器中查看
  Serial.print("Roll:");
  Serial.print(fNewPitch); Serial.print('(');
  Serial.print(deltaPitch); Serial.print(")\n");
  delay(10);
}

//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void WriteMPUReg(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char ReadMPUReg(int nReg) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}

//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void ReadAccGyr(int *pVals) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

//对大量读数进行统计，校准平均偏移量
void Calibration()
{
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
  //先求和
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[nValCnt];
    ReadAccGyr(mpuVals);
    for (int j = 0; j < nValCnt; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  //再求平均
  for (int i = 0; i < nValCnt; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
  calibData[2] += 16384; //设芯片Z轴竖直向下，设定静态工作点。
}

//算得Roll角。算法见文档。
float GetRoll(float *pRealVals, float fNorm) {
  float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//算得Pitch角。算法见文档。
float GetPitch(float *pRealVals, float fNorm) {
  float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void Rectify(int *pReadout, float *pRealVals) {
  for (int i = 0; i < 3; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
  }
}
