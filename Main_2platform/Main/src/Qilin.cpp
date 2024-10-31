#include "Qilin.h"


void setServoAngle(uint8_t num, uint16_t angle) {
    uint16_t pulselength = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(num, 0, pulselength);
}

void move_joints(int leg, double j1, double j2, double j3)
{
  // 控制每個伺服馬達的角度
  if (j1 >= 0 && j1 <= 105 && j2 >= 0 && j2 <= 120 && j3 >= 0 && j3 <= 120)
  {
    setServoAngle(leg * 0, j1 + J1_OFFSET);  // 控制 J1 伺服

    // 根據腳的位置來判斷是否需要反向 J2 和 J3
    if (leg == 2 || leg == 3) { // 右側的腳 (右後腳、右前腳)
      setServoAngle(leg * 3 + 1, 180 - (j2 + J2_OFFSET));  // 反轉 J2
      setServoAngle(leg * 3 + 2, 180 - (j3 + J3_OFFSET));  // 反轉 J3
    } else { // 左側的腳 (左前腳、左後腳)
      setServoAngle(leg * 3 + 1, j2 + J2_OFFSET);  // 正常控制 J2
      setServoAngle(leg * 3 + 2, j3 + J3_OFFSET);  // 正常控制 J3
    }
  }
  else
  {
    Serial.print("Out of bounds servo command on leg ");
    Serial.print(leg);
    Serial.print(": ");
    Serial.print(j1);
    Serial.print(" ");
    Serial.print(j2);
    Serial.print(" ");
    Serial.println(j3);
  }
}

void move_xyz(int leg, double x, double y, double z)
{
  // Add input validation
  if (!isValidPosition(x, y, z)) {
    Serial.printf("Invalid position for leg %d: x=%.2f y=%.2f z=%.2f\n", leg, x, y, z);
    return;
  }

  y += Y_OFFSET;
  z += Z_OFFSET;

  double l = sqrt(x * x + y * y + z * z);
  
  // Check if length is within valid range
  if (l > (FEMUR_L + TIBIA_L)) {
    Serial.printf("Position out of reach for leg %d: length=%.2f\n", leg, l);
    return;
  }

  double j1 = J1_fixed_angles[leg];
  
  // Calculate angles with error checking
  double j2_cos = (FEMUR_L * FEMUR_L - TIBIA_L * TIBIA_L + l * l) / (2 * FEMUR_L * l);
  double j3_cos = (FEMUR_L * FEMUR_L + TIBIA_L * TIBIA_L - l * l) / (2 * FEMUR_L * TIBIA_L);
  
  if (abs(j2_cos) > 1.0 || abs(j3_cos) > 1.0) {
    Serial.printf("Invalid angle calculation for leg %d\n", leg);
    return;
  }

  double j2 = acos(j2_cos) + atan2(z, sqrt(x * x + y * y));
  double j3 = acos(j3_cos);

  j2 = 0 + j2 * (180 / PI); // 转换为度数
  j3 = 90 + 30 - TIBIA_ANGLE - j3 * (180 / PI); // 转换为度数

  move_joints(leg, j1, j2, j3); // 控制伺服馬達
}

// Helper function to validate input position
bool isValidPosition(double x, double y, double z) {
  // Add your position validation logic here
  const double MAX_DISTANCE = FEMUR_L + TIBIA_L;
  return (abs(x) < MAX_DISTANCE && 
          abs(y) < MAX_DISTANCE && 
          abs(z) < MAX_DISTANCE);
}