#include<Stepper.h>
 
// 参考文件
/*
    28BYJ-48.pdf
 
    该参数根据电机每一转的步数来修改
*/
const int stepsPerRevolution = 64;
int pushButton1 = 2;
int pushButton2 = 3;
 
// 初始化步进电机要使用的Arduino的引脚编号
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);
int previous = 0; 
void setup()
{
    // 设置转速，每分钟为90步
    myStepper.setSpeed(90);
 
    // 初始化串口
    Serial.begin(9600);
}
 
void loop()
{
    int buttonState1 = digitalRead(pushButton1);
    int buttonState2 = digitalRead(pushButton2);
    
    if(buttonState2 < 1){
       myStepper.step( buttonState1 );
       delay(0);
    }
    if(buttonState1 < 1){
    //Serial.println("counterclockwise");
     myStepper.step( -buttonState2 );
     delay(0);
    }
    
}
