#include <math.h>

/* 引脚定义 */
#define LEFT_A_PIN 5//开发板两个伸出的电机引脚指向前为正方向的左右5,6   ,9,10
#define LEFT_B_PIN 6

#define RIGHT_A_PIN 10
#define RIGHT_B_PIN 9

#define SENSOR_NUM 3

#define LEFT_A_TRACK_PIN A0
#define MIDDLE_TRACK_PIN A2
#define RIGHT_A_TRACK_PIN A3

/* 关键变量声明 */
const float Kp = 19.0;//19
const float Ki = 125.0;//125
const float Kd = 1.3;//1.3
float delta_pid = 0.0;
const int BLACK_VALUE = 200;//判定为黑的阈值
const int INITIAL_MOTOR_SPEED = 255;//维持速度
uint16_t sensor_values[SENSOR_NUM] = {0};//传感器状态储存
//unsigned long start_time = 0;//计时器

/* 传感器名称数组 */
const uint8_t SENSOR_NAMES[SENSOR_NUM] = {
  LEFT_A_TRACK_PIN, MIDDLE_TRACK_PIN, RIGHT_A_TRACK_PIN
};

/* 电机引脚数组 */
const uint8_t MOTOR_PINS[4] = {
  LEFT_A_PIN, LEFT_B_PIN, RIGHT_A_PIN, RIGHT_B_PIN
};

/* 上一次误差值和上上次误差值 */
static float previous_error = 0.0;
static float previous2_error = 0.0;

/* 函数声明 */
void read_sensor_values(void);
void motor_control(void);
void pin_init(uint8_t pin, uint8_t mode);//初始化引脚
void motor_pin_init(void);//初始化电机引脚
void track_pin_init(void);//初始化传感器引脚
void stop(void);
uint32_t get_sensors_state(void);
float calc_error();
void calc_delta_pid(void);

/* 主程序 */
void setup() {
  Serial.begin(9600);
  track_pin_init();
  motor_pin_init();
  Serial.setTimeout(2);
  
}

void loop() {
    //start_time = micros();
  read_sensor_values();//读取传感器值
  calc_delta_pid();//计算增量式PID
  motor_control();//依据PID控制电机
    //Serial.println(micros() - start_time);
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

/* 获取传感器状态值 */
uint32_t get_sensors_state(void) {
  uint32_t state = 0;
  for (uint8_t i = 0; i < SENSOR_NUM; i++) {
    state <<= 1;
    state |= (sensor_values[i] < BLACK_VALUE) ? 1 : 0;
  }
  return state;
}

float calc_error() {
  uint32_t state = get_sensors_state();
  switch (state) {
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

void calc_delta_pid(void) {
  float error = calc_error();
  float delta_error = error - previous_error;
  float delta2_error = error - 2*previous_error + previous2_error;
  // 将delta_pid保存到全局变量中
  delta_pid = Kp * delta_error + Ki * error + Kd * delta2_error;

  previous2_error = previous_error;
  previous_error = error;


}

void motor_control()//控制电机,速度范围[-255,255]
{
  int left_motor_speed = INITIAL_MOTOR_SPEED + delta_pid;//可能需要速度补偿20
  int right_motor_speed = INITIAL_MOTOR_SPEED - delta_pid;

  if(left_motor_speed < -255)
  {
    left_motor_speed = -255;
  }
  
  if(left_motor_speed > 255)
  {
    left_motor_speed = 255;
  }
    if(right_motor_speed < -255)
  {
    right_motor_speed = -255;
  }
  
  if(right_motor_speed > 255)
  {
    right_motor_speed = 255;
  }
  /*上面是算法部分,下面是硬件部分*/

  if (right_motor_speed > 0) 
  {
    
    digitalWrite(RIGHT_B_PIN, LOW);//要把置低电平的语句放在前面防止烧电机!
    analogWrite(RIGHT_A_PIN,right_motor_speed);

  } 
  else 
  {
    digitalWrite(RIGHT_A_PIN, LOW);
    analogWrite(RIGHT_B_PIN,-right_motor_speed);
  }
 
  if (left_motor_speed > 0) 
  {
    digitalWrite(LEFT_B_PIN, LOW);
    analogWrite(LEFT_A_PIN,left_motor_speed);
  }
  else 
  {
    digitalWrite(LEFT_A_PIN, LOW);
    analogWrite(LEFT_B_PIN,-left_motor_speed);
  }
}