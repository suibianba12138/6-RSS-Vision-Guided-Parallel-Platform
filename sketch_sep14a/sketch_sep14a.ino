#include <ESP32Servo.h>  // 使用ESP32专用的舵机库

// 定义舵机数量和串口波特率
#define NUM_SERVOS 6
#define BAUDRATE 115200

// 舵机角度范围配置（根据实际舵机参数修改）
#define MIN_ANGLE -90.0   // 最小角度限制
#define MAX_ANGLE 180.0  // 最大角度限制

// 舵机对象数组
Servo servos[NUM_SERVOS];

// 舵机控制引脚 (可根据实际接线修改，ESP32大多数引脚支持PWM)
const int servoPins[NUM_SERVOS] = {18, 5, 17, 16, 4, 15};

// 存储舵机角度
float servoAngles[NUM_SERVOS];

// 串口接收相关变量
String serialBuffer = "";
bool dataReceiving = false;

void setup() {
  // 初始化串口通信
  Serial.begin(BAUDRATE);
  
  // 初始化舵机到中间位置
  float midAngle = (MIN_ANGLE + MAX_ANGLE) / 2;
  for (int i = 0; i < NUM_SERVOS; i++) {
    servoAngles[i] = 0;
  }
  
  // 初始化舵机
  for (int i = 0; i < NUM_SERVOS; i++) {
    // 连接舵机到指定引脚，设置角度范围
    if (servos[i].attach(servoPins[i], 500, 2500)) {  // 500-2500us脉冲对应0-180度
      Serial.print("舵机 ");
      Serial.print(i + 1);
      Serial.print(" 已连接到引脚 ");
      Serial.println(servoPins[i]);
      
      // 初始化为中间角度
      servos[i].write(0);
    } else {
      Serial.print("舵机 ");
      Serial.print(i + 1);
      Serial.println(" 连接失败!");
    }
  }
  
  Serial.print("系统初始化完成，等待角度数据... 角度范围: ");
  Serial.print(MIN_ANGLE);
  Serial.print("-");
  Serial.println(MAX_ANGLE);
}

void loop() {
  // 处理串口数据
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // 检测帧起始符
    if (c == 'S') {
      serialBuffer = "";
      dataReceiving = true;
    }
    // 检测帧结束符
    else if (c == 'E' && dataReceiving) {
      dataReceiving = false;
      parseSerialData(serialBuffer);
    }
    // 接收数据内容
    else if (dataReceiving) {
      serialBuffer += c;
    }
  }
}

// 解析串口接收的数据
void parseSerialData(String data) {
  // 数据格式应为: ",angle1,angle2,angle3,angle4,angle5,angle6,"
  // 分割字符串提取角度值
  int index = 0;
  String anglesStr[NUM_SERVOS];
  
  // 分割数据
  for (int i = 0; i < NUM_SERVOS; i++) {
    int commaIndex = data.indexOf(',', index + 1);
    if (commaIndex == -1) break;
    
    anglesStr[i] = data.substring(index + 1, commaIndex);
    index = commaIndex;
  }
  
  // 转换为数值并更新舵机角度
  bool updateSuccess = true;
  for (int i = 0; i < NUM_SERVOS; i++) {
    float angle = anglesStr[i].toFloat();
    if (i%2==0)
    {
      angle=45-angle;
    }
    else
    {
      angle=135+angle;
    }
    

    
    // 检查角度是否有效（在设定的角度范围内）
    if (angle >= MIN_ANGLE && angle <= MAX_ANGLE) {
      servoAngles[i] = angle;
      servos[i].write(angle); // 控制舵机转动
    } else {
      Serial.print("舵机 ");
      Serial.print(i + 1);
      Serial.print(" 角度无效: ");
      Serial.print(angle);
      Serial.print("，有效范围: ");
      Serial.print(MIN_ANGLE);
      Serial.print("-");
      Serial.println(MAX_ANGLE);
      updateSuccess = false;
    }
  }
  
  // 发送确认信息
  if (updateSuccess) {
    Serial.print("角度已更新: ");
    for (int i = 0; i < NUM_SERVOS; i++) {
      Serial.print(servoAngles[i]);
      if (i < NUM_SERVOS - 1) Serial.print(",");
    }
    Serial.println();
  }
}
    