#include <math.h>
#include "gyro_serial_on_car.cpp"
/* 引脚定义 */
#define LEFT_A_PIN 9//开发板两个伸出的电机引脚指向前为正方向的左右5,6   ,9,10
#define LEFT_B_PIN 10
#define RIGHT_A_PIN 5
#define RIGHT_B_PIN 6

#define SERVO_N_PIN 11//舵机引脚3,2,11
#define SERVO_M_PIN 3
#define SERVO_F_PIN 12

#define INFRA A3//红外传感器引脚

#define LEFT_A_TRACK_PIN A4//寻迹引脚
#define MIDDLE_TRACK_PIN A2
#define RIGHT_A_TRACK_PIN A0

#define S2     7   //S2和S3的组合决定让红、绿、蓝哪种光线通过滤波器
#define S3     8
#define OUT    4  //输出信号输入到Arduino数字引脚,后续用于读取单个脉冲的时间

#define OE   13  //控制TCS3200颜色传感器是否enable
#define LED 2  //控制TCS3200颜色传感器的LED灯


/* 关键变量声明 */
#define SENSOR_NUM 3//寻迹传感器数量

Servo servoN;//基坐舵机
Servo servoM;//(越障状态下)俯仰舵机pitchServo
Servo servoF;//(越障状态下)滚转舵机rollServo

/*舵机的初始和工作位置*/
const uint16_t servoNinit = 2500;
const uint16_t servoMinit = 1500;
const uint16_t servoFinit = 1400;
const uint16_t servoNwork = 1500;
const uint16_t servoMwork = 1800;
const uint16_t servoFwork = 2300;

const int BLACK_VALUE = 250;//判定为黑的阈值
uint16_t sensor_values[SENSOR_NUM] = {0};//传感器状态储存
float error = 0;//寻迹误差
float previous_error = 0;//上一次误差
bool blackFlag = 0;//传感器全黑状态标记
bool infraFlag = 0;//红外状态标记
bool infra = 0;//存储滤波后红外状态,应用到滤波器上的结果就是,只有一直探测到物体,滤波器里全是0才会返回0,否则返回1
uint8_t allBlackCount = 0;//全黑计数.逻辑为上一次黑下一次不是,则计数
uint8_t targetColor = 255;//目标颜色,0为红,1为绿,2为蓝,255为未获取到颜色
uint8_t colorCount = 50; //单次调用时颜色采样次数
int16_t redStandard = 0;//颜色传感器白平衡标准
int16_t greenStandard = 0;
int16_t blueStandard = 0;
/* 传感器名称数组 */
const uint8_t SENSOR_NAMES[SENSOR_NUM] = {
  LEFT_A_TRACK_PIN, MIDDLE_TRACK_PIN, RIGHT_A_TRACK_PIN
};

/* 电机引脚数组 */
const uint8_t MOTOR_PINS[4] = {
  LEFT_A_PIN, LEFT_B_PIN, RIGHT_A_PIN, RIGHT_B_PIN
};


/* 函数声明 */
void read_sensor_values(void);//读取传感器值,并在满足条件时改变小车状态
void motor_control(void);//控制小车电机
void pin_init(uint8_t pin, uint8_t mode);//初始化引脚
void motor_pin_init(void);//初始化电机引脚
void track_pin_init(void);//初始化传感器引脚
void stop(void);//停车
uint32_t get_sensors_state(void);//将传感器状态数组转为二进制状态数
float calc_error();//计算误差
void tcl3200_pin_init(void);//颜色传感器针脚初始化
void colour_init(void);//颜色传感器白平衡初始化
uint8_t get_color(void);//获取颜色
void servo_init(void);//舵机初始化
void servo_control(bool state);//舵机控制,参数为0时旋转N舵机,参数为1时旋转M,F舵机

ServoController2 servoController = ServoController2(servoM, servoF);//云台舵机控制器
serialMPU6050 mpuProcessor = serialMPU6050();//陀螺仪解算
MovingAverageFilter infraFilter = MovingAverageFilter(20);//红外传感器结果滤波器,对于红外,LOW(0)为探测到物体,HIGH(1)为未探测到物体
/* 主程序 */
void setup() {
    Serial.begin(115200);
    track_pin_init();
    motor_pin_init();
    servo_init();
    tcl3200_pin_init();
    colour_init();
    pinMode(INFRA, INPUT);

}

void loop() {

    read_sensor_values();//读取传感器值
    motor_control();//依据小车状态控制电机
    //servoN.writeMicroseconds(mpuProcessor.getYawMicrosecond()+1000);//基坐舵机控制
    if (allBlackCount <= 0){
      servoController.update(mpuProcessor.getPitchMicrosecond(), mpuProcessor.getRollMicrosecond());//云台舵机控制
    }
    else {//如果全黑计数大于0,进行颜色识别&舵机控制.(只有在运行30秒之后&&偏航角在-30~30°时累加,具体见calc_error()函数)
      infra = infraFilter.filter(digitalRead(INFRA));
        if (targetColor == 255 && infra == LOW){//如果未获取到颜色,且红外传感(一直)探测到物体,就获取目标颜色,该分支只会执行一次
            targetColor = get_color();
            servo_control(0);
            infraFlag = 1;
            //delay(400);//避免第一个色卡多次识别
        }
        else{
            if (infra == LOW){//如果红外传感器(一直)探测到物体
                if (infraFlag == 0){//如果上一次没有探测到物体
                    if (get_color() == targetColor) servo_control(1);//说明遇到了色卡和盒子,进行颜色识别.如果颜色一致,则舵机转动
                }
                infraFlag = 1;//标记红外传感器已经探测到过物体,下次不再进入分支
            }
            else infraFlag = 0;
        }  
    }
}

void serialEvent() {
    mpuProcessor.update();//陀螺仪数据更新
}


/* 以下是函数实现部分 */

void pin_init(uint8_t pin, uint8_t mode) {
  pinMode(pin, mode);
}

void track_pin_init() {
  for (uint8_t i = 0; i < SENSOR_NUM; i++) {
    pin_init(SENSOR_NAMES[i], INPUT);
  }
}


void motor_pin_init() {
  for (uint8_t i = 0; i < 4; i++) {
    pin_init(MOTOR_PINS[i], OUTPUT);
  }
}

void servo_init(){
  servoN.attach(SERVO_N_PIN);
  servoM.attach(SERVO_M_PIN);
  servoF.attach(SERVO_F_PIN);
  servoN.writeMicroseconds(servoNinit);//舵机初始位置
  servoM.writeMicroseconds(servoMinit);
  servoF.writeMicroseconds(servoFinit);
}
void servo_control(bool state){
/*
舵机控制函数.参数state为0时为N舵机,1为F舵机
*/
  if (state == 0){

    servoN.writeMicroseconds(servoNwork);
    
  }
  else{
    servoM.writeMicroseconds(servoMwork);
    servoF.writeMicroseconds(servoFwork);
  }

}

void tcl3200_pin_init(){
  // 将传感器的OUT引脚设置为输入模式
    pinMode(OUT, INPUT);
    pinMode(LED, OUTPUT);
    pinMode(OE, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    digitalWrite(LED, LOW);
    digitalWrite(OE, HIGH);//初始禁用传感器输出
}
void colour_init(){//初始化颜色传感器,获取环境光情况.最好先对着白色的物体(对着虚空也行,效果差一些),随后再放到待测物体 
  int16_t red[colorCount] = {};
  int16_t green[colorCount] = {};
  int16_t blue[colorCount] = {};

  for (size_t i = 0; i < 4; i++) {//闪烁LED,提示用户进行白平衡
    digitalWrite(LED, HIGH);
    delay(250);
    digitalWrite(LED, LOW);
    delay(250);
  }
    digitalWrite(LED, HIGH);
  for (size_t i = 0; i < colorCount; i++) {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    digitalWrite(OE, LOW);
    red[i] = pulseIn(OUT, LOW);
    digitalWrite(OE, HIGH);

    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    digitalWrite(OE, LOW);
    green[i] = pulseIn(OUT, LOW);
    digitalWrite(OE, HIGH);

    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    digitalWrite(OE, LOW);
    blue[i] = pulseIn(OUT, LOW);
    digitalWrite(OE, HIGH);
  }

  for (size_t i = 0; i < colorCount; i++) {
    redStandard += red[i];
    greenStandard += green[i];
    blueStandard += blue[i];
  }
  digitalWrite(LED, LOW);
  Serial.print("redStandard: ");
  Serial.println(redStandard);
  Serial.print("greenStandard: ");
  Serial.println(greenStandard);
  Serial.print("blueStandard: ");
  Serial.println(blueStandard);
  Serial.println("----------WhiteBalanceComplete----------");
}

uint8_t get_color() {
  // 测量各种颜色方波的周期
  digitalWrite(LED, HIGH);
  int16_t red[colorCount] = {};
  int16_t green[colorCount] = {};
  int16_t blue[colorCount] = {};
    int16_t clear[colorCount] = {};
    for (size_t i = 0; i < colorCount; i++)
    {
        digitalWrite(S2, LOW);
        digitalWrite(S3, LOW);
        digitalWrite(OE, LOW);
        red[i] = pulseIn(OUT, LOW);
        digitalWrite(OE, HIGH);

        digitalWrite(S2, HIGH);
        digitalWrite(S3, HIGH);
        digitalWrite(OE, LOW);
        green[i] = pulseIn(OUT, LOW);
        digitalWrite(OE, HIGH);

        digitalWrite(S2, LOW);
        digitalWrite(S3, HIGH);
        digitalWrite(OE, LOW);
        blue[i] = pulseIn(OUT, LOW);
        digitalWrite(OE, HIGH); 
    }
    // 计算各种颜色方波的周期的和
    int16_t redPeriod = 0;
    int16_t greenPeriod = 0;
    int16_t bluePeriod = 0;
    for (size_t i = 0; i < colorCount; i++)
    {
        redPeriod += red[i];
        greenPeriod += green[i];
        bluePeriod += blue[i];
    }
    int16_t redDiff = redPeriod - redStandard;
    int16_t greenDiff = greenPeriod - greenStandard;
    int16_t blueDiff = bluePeriod - blueStandard;
    /*
    Serial.print("redDiff: ");
    Serial.println(redDiff);
    Serial.print("greenDiff: ");
    Serial.println(greenDiff);
    Serial.print("blueDiff: ");
    Serial.println(blueDiff);
    */
    digitalWrite(LED, LOW);
    //以差值判定颜色
    if (redDiff <= greenDiff && redDiff <= blueDiff) {
        return 0;
    } else if (greenDiff <= redDiff && greenDiff <= blueDiff) {
        return 1;
    } else if (blueDiff <= redDiff && blueDiff <= greenDiff) {
        return 2;
    } else {
        return 40;
    }
    
}

void stop() {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(MOTOR_PINS[i], LOW);
  }
}

/* 获取传感器值 */
void read_sensor_values(void) {
  for (uint8_t i = 0; i < SENSOR_NUM; i++) {
    sensor_values[i] = analogRead(SENSOR_NAMES[i]);
  }
}

/* 解算传感器状态值 */
uint32_t get_sensors_state(void) {
  uint32_t state = 0;
  for (uint8_t i = 0; i < SENSOR_NUM; i++) {
    state <<= 1;
    state |= (sensor_values[i] < BLACK_VALUE) ? 1 : 0;
  }
  return state;
}
/*@author MaoJiayang
*/
float calc_error() {
  uint8_t state = get_sensors_state();
  if (millis() > 15000) {//只有在30s外且偏航小于25度时才进行黑线检测 && abs(mpuProcessor.getYaw()) < 60
    if(state == 0b111){//如果探测到全黑
    blackFlag = 1;
    }else{   
        if (blackFlag == 1) allBlackCount++;//如果上一次是全黑,则计数    
        blackFlag = 0;
    }
  }   
  switch (state) {//根据传感器状态计算误差
    case 0b111:
      return 0.0;break;
    case 0b000:
      return 0.0;break;
    case 0b100:
      return -4.0;break;
    case 0b110:
      return -2.0;break;
    case 0b010:
      return 0.0;break;
    case 0b011:
      return 2.0;break;        
    case 0b001:
      return 4.0;break;
    default:
      return previous_error;
  }
  
}


void motor_control()//控制电机
{   
    error = calc_error();
    previous_error = error;

    if (error == 0)
    {//误差为0,前进
        digitalWrite(LEFT_A_PIN, HIGH);
        digitalWrite(LEFT_B_PIN, LOW);
        digitalWrite(RIGHT_A_PIN, HIGH);
        digitalWrite(RIGHT_B_PIN, LOW);
    }
    else if (error > 0)
    {//误差为正,右转
        if (error == 2)//误差为2,半幅右转
        {
            digitalWrite(LEFT_A_PIN, HIGH);
            digitalWrite(LEFT_B_PIN, LOW);
            digitalWrite(RIGHT_A_PIN, LOW);
            digitalWrite(RIGHT_B_PIN, LOW);
        }
        else//误差为4,全幅右转
        {
            digitalWrite(LEFT_A_PIN, HIGH);
            digitalWrite(LEFT_B_PIN, LOW);
            digitalWrite(RIGHT_A_PIN, LOW);
            digitalWrite(RIGHT_B_PIN, HIGH);
        }
    }
    else
    {//误差为负,左转
        if (error == -2)//误差为-2,半幅左转
        {
            digitalWrite(LEFT_A_PIN, LOW);
            digitalWrite(LEFT_B_PIN, LOW);
            digitalWrite(RIGHT_A_PIN, HIGH);
            digitalWrite(RIGHT_B_PIN, LOW);
        }
        else//误差为-4,全幅左转
        {
            digitalWrite(LEFT_A_PIN, LOW);
            digitalWrite(LEFT_B_PIN, HIGH);
            digitalWrite(RIGHT_A_PIN, HIGH);
            digitalWrite(RIGHT_B_PIN, LOW);
        }
    }
  
}