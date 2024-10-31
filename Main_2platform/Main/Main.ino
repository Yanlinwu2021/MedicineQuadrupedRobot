#include "./src/Qilin.h"

const int BUTTON_PIN = 2;
// 定義感測器引腳
const int WaterLevelsensorPin = 4; // 感測器的信號輸入接到D4
const int ledPin = 13;   // 測試用LED

// 用于存储上一个传感器状态
int lastSensorValue = HIGH; // 初始化为HIGH，假设初始水位正常


// 設置各腳的初始階段
int phase[4] = {0, 2, 1, 3};

/*
         , - ~ ~ ~ - ,
     , '               ' ,
    ,                     ,
    '----~----->-----~----'
  -1.0 -0.5   0.0   0.5  1.0
*/
// 設置每個階段的目標位置和腳的方向
double phase_targets[4] = {0.5, 0.0, 0.5, 1.0};
double leg_directions[4] = {PI / 4, (3 * PI) / 4, -(3 * PI)/4 , -PI / 4};




// 当前步态模式
enum GaitMode { REPTILES_WALK, STOP, MAMMALS_WALK };
GaitMode currentGaitMode = STOP;

// 用于防止快速切换的标志位
bool isTransitioning = false;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const double STEP_RADIUS = 30.0; // 步幅半径
double x_start[4] = {0, 0, 0, 0}; // 起始X位置
double y_start[4] = {0, 0, 0, 0}; // 起始Y位置
double x_target[4] = {0, 0, 0, 0}; // 目标X位置
double y_target[4] = {0, 0, 0, 0}; // 目标Y位置

void setup() {
  pinMode(WaterLevelsensorPin, INPUT);  // 設定水位感測器為輸入模式
  pinMode(ledPin, OUTPUT);    // 設定LED為輸出模式
  pinMode(BUTTON_PIN, INPUT); // 设置按钮引脚为输入
  Serial.begin(9600);

    pwm.begin();
    pwm.setPWMFreq(60);
    delay(10);

    for (int n = 0; n < 12; n++)
        setServoAngle(n, 90);

    for (int leg = 0; leg < 4; leg++) {
        if (leg == 0 || leg == 1) {
            move_xyz(leg, 30, 0, 0);
        } else {
            move_xyz(leg, -30, 0, 0);
        }
    }
    delay(1000);
}

void loop() {
    // // 检查按钮切换步态
    // if (digitalRead(BUTTON_PIN) == LOW && !isTransitioning) {
    //     isTransitioning = true; // 设置为正在切换
        
    //     // 等待马达稳定
    //     delay(200); // 短暂延迟以避免按钮反弹

    //     // 切换步态
    //     switchGaitMode();

    //     // 让马达有时间稳定
    //     delay(500); // 这里可以根据马达特性调整延迟时间

    //     isTransitioning = false; // 重置切换标志位
    // }
// 读取水位传感器状态
    int sensorValue = digitalRead(WaterLevelsensorPin);

    // 检查水位状态变化
    if (sensorValue != lastSensorValue) {
        if (!isTransitioning) { // 只有在未转变时才进行状态切换
            isTransitioning = true; // 设置为正在切换

            if (sensorValue == LOW) { // 水位高
                if (currentGaitMode != REPTILES_WALK) {
                    // 切换到 REPTILES_WALK
                    currentGaitMode = REPTILES_WALK;
                    Serial.println("Switched to REPTILES_WALK.");
                } 
            } else { // 水位低
                if (currentGaitMode != MAMMALS_WALK) {
                    // 切换到 MAMMALS_WALK
                    currentGaitMode = MAMMALS_WALK;
                    Serial.println("Switched to MAMMALS_WALK.");
                }
            }

            // 进行状态切换时可以添加平滑过渡的延迟
            delay(500); // 这里可以根据需要调整延迟时间

            isTransitioning = false; // 重置切换标志位
        }

        // 更新上一个传感器状态
        lastSensorValue = sensorValue;
    } else {
    isTransitioning = false; // 只有在传感器值没有变化时才重置
}

    // 根据当前模式执行不同的行为
    switch (currentGaitMode) {
        case REPTILES_WALK:
            executeLoongWalk();
            break;
        case MAMMALS_WALK:
            executeMammalWalk();
            break;
    }

    delay(300); // 每300毫秒检查一次水位
}

void switchGaitMode() {
    if (currentGaitMode == REPTILES_WALK) {
        currentGaitMode = STOP;
    } else {
        currentGaitMode = REPTILES_WALK;
    }
}

// 實現爬蟲行走的邏輯
void executeLoongWalk() {
    // 獲取目標位置
    for (int leg = 0; leg < 4; leg++) {
        double v_mag = 2 * STEP_RADIUS * phase_targets[phase[leg]]; // 计算移动的距离
        double v_dir = 0.0; // 设置方向为 0
        x_target[leg] = v_mag * sin(v_dir + leg_directions[leg]); // 计算目标 X 轴位置
        y_target[leg] = v_mag * cos(v_dir + leg_directions[leg]); // 计算目标 Y 轴位置
    }

    // 遍历时间范围进行平滑移动
    for (double t = 0.0; t < 1.0; t += 0.02) {
        for (int leg = 0; leg < 4; leg++) {
            double x = (1 - t) * x_start[leg] + t * x_target[leg]; // 计算当前 X 轴位置
            double y = (1 - t) * y_start[leg] + t * y_target[leg]; // 计算当前 Y 轴位置
            double z = (phase[leg] == 0) ? 3 * STEP_RADIUS * sqrt(1 - (1 - t) * (1 - t)) :
                       (phase[leg] == 1) ? 3 * STEP_RADIUS * sqrt(1 - (t * t)) : 0; // 根据阶段计算 Z 轴位置
            move_xyz(leg, x, y, z); // 设置伺服马达位置
        }
        delay(10); // 等待 10 毫秒
    }

    // 保存结束位置，更新阶段
    for (int leg = 0; leg < 4; leg++) {
        phase[leg] = (phase[leg] + 1) % 4; // 更新阶段
        x_start[leg] = x_target[leg]; // 保存当前 X 轴位置
        y_start[leg] = y_target[leg]; // 保存当前 Y 轴位置
    }
}

// 實現哺乳行走的邏輯
void  executeMammalWalk() {
    // 獲取目標位置
    for (int leg = 0; leg < 4; leg++) {
        double v_mag = 2 * STEP_RADIUS * phase_targets[phase[leg]]; // 计算移动的距离
        double v_dir = 0.0; // 设置方向为 0
        x_target[leg] = v_mag * sin(v_dir + leg_directions[leg]); // 计算目标 X 轴位置
        y_target[leg] = v_mag * cos(v_dir + leg_directions[leg]); // 计算目标 Y 轴位置
    }

    // 遍历时间范围进行平滑移动
    for (double t = 0.0; t < 1.0; t += 0.5) {
        for (int leg = 0; leg < 4; leg++) {
            double x = (1 - t) * x_start[leg] + t * x_target[leg]; // 计算当前 X 轴位置
            double y = (1 - t) * y_start[leg] + t * y_target[leg]; // 计算当前 Y 轴位置
            double z = (phase[leg] == 0) ? 3 * STEP_RADIUS * sqrt(1 - (1 - t) * (1 - t)) :
                       (phase[leg] == 1) ? 3 * STEP_RADIUS * sqrt(1 - (t * t)) : 0; // 根据阶段计算 Z 轴位置
            move_xyz(leg, x, y, z); // 设置伺服马达位置
        }
        delay(10); // 等待 10 毫秒
    }

    // 保存结束位置，更新阶段
    for (int leg = 0; leg < 4; leg++) {
        phase[leg] = (phase[leg] + 1) % 4; // 更新阶段
        x_start[leg] = x_target[leg]; // 保存当前 X 轴位置
        y_start[leg] = y_target[leg]; // 保存当前 Y 轴位置
    }
}




