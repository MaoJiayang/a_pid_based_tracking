#include <Servo.h>
#include <Arduino.h>
class MovingAverageFilter {//使用滤波的代价是延迟
  private:
    float* buffer; // 缓存数组
    int bufferSize; // 缓存数组大小
    int currentIndex; // 当前缓存位置
    float sum; // 缓存数组元素和
  public:
    MovingAverageFilter(int size) {// 构造函数
      buffer = new float[size]{0.0};
      bufferSize = size;
      currentIndex = 0;
      sum = 0;
    }
    ~MovingAverageFilter() {
      delete[] buffer;
    }
    float filter(float value) {//通过循环队列实现窗口式移动平均
      sum -= buffer[currentIndex]; // 减去当前位置的元素
      buffer[currentIndex] = value; // 更新当前位置的元素
      sum += value; // 加上新的元素
      currentIndex = (currentIndex + 1) % bufferSize; // 更新当前位置
      return sum / bufferSize; // 返回平均值
    }
};
class ServoController2 {// 使用二阶插值的舵机控制器
  private:
    Servo pitchServo; // 定义俯仰舵机对象
    Servo rollServo; // 定义滚转舵机对象
    float targetPitchAngle = 1500;  // 目标俯仰角
    float currentPitchAngle = 1500; // 当前俯仰角
    float pitchVelocity = 0; // 当前俯仰速度
    float pitchAcceleration = 0; // 当前俯仰加速度
    float targetRollAngle = 1500;  // 目标滚转角度
    float currentRollAngle = 1500; // 当前滚转角度
    float rollVelocity = 0; // 当前滚转速度
    float rollAcceleration = 0; // 当前滚转加速度
    float maxAcceleration = 5000; // 最大加速度限制(5000=无限)
    float maxVelocity = 5000; // 最大速度限制(5000=无限)
    float dampingFactor = 0.5;// 阻尼系数,[0,1],数学上可以理解为(阻尼系数+1)为控制曲线的次数.阻尼系数大,响应速度慢但平滑(二次收敛),阻尼系数小,响应速度快但可能出现震荡(一次收敛)

  public:
      ServoController2(Servo pitchServo0, Servo rollServo0) {
        pitchServo = pitchServo0;
        rollServo = rollServo0;
    }

    void update(int pitch, int roll) {
      // 更新pitch舵机位置
      targetPitchAngle = (pitch - 1500) + 1500; // 设置舵机的目标角度(1500 - pitch)+1500,如果感觉舵机反向,可以改为(pitch - 1500)+1500
      /*说明:
      *上面这段代码主要是将陀螺仪输入的测量角度(以舵机毫秒值表示的)转换为舵机需要的控制角度.算法为: 角度差 + 维持位置
      *角度差 = (陀螺仪测量角度 - 1500),或者在舵机反向时(1500 - 陀螺仪测量角度).这里的1500是舵机的中位值,如果舵机的中位值不是1500,需要修改这个值
      */
      
      // 更新舵机位置,使用二阶插值,误差error作为加速度绝对值
      float error = targetPitchAngle - currentPitchAngle;
      int8_t sign = error > 0 ? 1 : -1;
      pitchAcceleration = sign * min(maxAcceleration, abs(error));
      pitchVelocity = dampingFactor * pitchVelocity + pitchAcceleration;
      pitchVelocity = min(maxVelocity, max(-maxVelocity, pitchVelocity));
      currentPitchAngle += pitchVelocity;
      // 更新舵机位置
      pitchServo.writeMicroseconds(currentPitchAngle);
      
     //pitchServo.writeMicroseconds(targetPitchAngle);//直接写入目标位置

      // 更新roll舵机位置
      targetRollAngle = (1500 - roll) + 1400; // 设置目标角度
      
      error = targetRollAngle - currentRollAngle;
      sign = error > 0 ? 1 : -1;
      rollAcceleration = sign * min(maxAcceleration, abs(error));
      rollVelocity = dampingFactor * rollVelocity + rollAcceleration;
      rollVelocity = min(maxVelocity, max(-maxVelocity, rollVelocity));
      currentRollAngle += rollVelocity;
      // 更新舵机位置
      rollServo.writeMicroseconds(currentRollAngle);
      
      //rollServo.writeMicroseconds(targetRollAngle);//直接写入目标位置

    }
    void printCurrentData() {
      Serial.print("roll: ");
      Serial.print(currentRollAngle);
      Serial.print(" pitch: ");
      Serial.println(currentPitchAngle);
    }
    void printTargetData() {
      Serial.print("roll: ");
      Serial.print(targetRollAngle);
      Serial.print(" pitch: ");
      Serial.println(targetPitchAngle);
    }
};
class serialMPU6050{
  /*
  *该类用于读取MPU6050的数据,并进行姿态角度解算.
  *通过调用getPitch(),getRoll(),getYaw()函数获取姿态角.
  *该类的update()方法必须在SerialEvent()中调用.如果在主函数中调用,如果串口数据读取时间过长,可能会阻塞主函数.
  */
    private:
        float pitch,roll,yaw;
        float prevoiusPitch,prevoiusRoll,prevoiusYaw;
        float ax,ay,az;
        unsigned char Re_buf[11],counter=0;
        unsigned char sign=0;
        MovingAverageFilter rollFilter{15}; // 定义移动平均滤波器对象
        MovingAverageFilter pitchFilter{15}; // 定义移动平均滤波器对象
        MovingAverageFilter yawFilter{15}; // 定义移动平均滤波器对象
        void readBuffer(){

            while (Serial.available())//TODO:这里的条件有可能导致数据帧读取不完全.探索者套件中的陀螺仪每帧数据长11字节
            {
                Re_buf[counter]=(unsigned char)Serial.read();
                if(counter==0&&Re_buf[0]!=0x55) return;      //第0号数据不是帧头(考虑在这里加入一个或条件:||Serial.available() < 11来防止读取不完整的帧)              
                counter++;       
                if(counter==11)             //接收到11个数据
                {    
                counter=0;               //重新赋值，准备下一帧数据的接收 
                sign=1;
                }
            }
        
        }
        public:
        void update(){//该函数只能在SerialEvent中调用!!!
            readBuffer();
            if(sign){//如果有数据帧

                sign=0;//清零，准备下一帧数据的接收
                if(Re_buf[0]==0x55){      //检查帧头  
                    /*
                    if (Re_buf [1] == 0x51){
                        ax = (short(Re_buf [3]<<8| Re_buf [2]));//32768.0*16;
                        ay = (short(Re_buf [5]<<8| Re_buf [4]));//32768.0*16;
                        az = (short(Re_buf [7]<<8| Re_buf [6]));//32768.0*16;
                        pitch = pitchFilter.filter(atan2(ax,az)*180/PI);
                        roll = rollFilter.filter(atan2(ay,az)*180/PI);
                    }
                    */
                    //他的角度数据有很大飘变
                    if (Re_buf [1] == 0x53 ){
                        prevoiusPitch = pitch;
                        prevoiusRoll = roll;
                        prevoiusYaw = yaw;
                        pitch = pitchFilter.filter((short(Re_buf [3]<<8| Re_buf [2]))/32768.0*180);
                        roll = rollFilter.filter((short(Re_buf [5]<<8| Re_buf [4]))/32768.0*180);
                        yaw = yawFilter.filter((short(Re_buf [7]<<8| Re_buf [6]))/32768.0*180);
                    } 
                    
                }

            }

        }
        float getPitch(){
            return pitch;
        }
        float getRoll(){
            return roll;
        }
        float getYaw(){
            return yaw;
        }
        float getPreviousPitch(){
            return prevoiusPitch;
        }
        float getPreviousRoll(){
            return prevoiusRoll;
        }
        float getPreviousYaw(){
            return prevoiusYaw;
        }
        float getPitchMicrosecond(){//100/9*pitch + 1500(只考虑-90,90度的范围,将其映射到500,2500)
            return int(100/9*pitch + 1500);
        }
        float getRollMicrosecond(){
            return int(100/9*roll + 1500);
        }
        float getYawMicrosecond(){
            return int(100/9*yaw + 1500);
        }
        void print(){
            Serial.print("pitch:");Serial.print(pitch);
            Serial.print(" roll:");Serial.print(roll);
            Serial.print(" yaw:");Serial.println(yaw);

        }
        void printMicrosecond(){
            Serial.print("pitch:");Serial.print(getPitchMicrosecond());
            Serial.print(" roll:");Serial.print(getRollMicrosecond());
            Serial.print(" yaw:");Serial.println(getRollMicrosecond());

        }

};
/*@author MaoJiayang
*/

