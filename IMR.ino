#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

// 互补滤波器的时间常数
float tau = 0.98;

// 上一次更新的时间
unsigned long last_update = 0;

// 当前的角度估计
float angle_x = 0;
float angle_y = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  if (!imu.init()) {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
}

void loop() {
  imu.read();

  // 计算加速度计的角度（单位：度）
  float accel_angle_x = atan2(imu.a.y, imu.a.z) * RAD_TO_DEG;
  float accel_angle_y = atan2(imu.a.x, imu.a.z) * RAD_TO_DEG;

  // 计算陀螺仪的角速度（单位：度/秒）
  float gyro_rate_x = imu.g.x / 32768.0 * 245; // Here 245 is the gyro's maximum range (in dps) when the FS is set to 00.
  float gyro_rate_y = imu.g.y / 32768.0 * 245;

  // 计算自上次更新以来经过的时间（单位：秒）
  float dt = (micros() - last_update) / 1000000.0;
  last_update = micros();

  // 使用互补滤波器更新角度估计
  angle_x = tau * (angle_x + gyro_rate_x * dt) + (1 - tau) * accel_angle_x;
  angle_y = tau * (angle_y + gyro_rate_y * dt) + (1 - tau) * accel_angle_y;

  // 打印角度估计
  Serial.print("Angle X: "); Serial.println(angle_x);
  Serial.print("Angle Y: "); Serial.println(angle_y);

  delay(100);
  // 角度估计是指机器人在X轴和Y轴上的倾斜角度。这个角度是通过结合加速度计和陀螺仪的数据来计算的
  // 加速度计可以测量重力的方向，从而得到机器人的倾斜角度。然而，加速度计的读数会受到机器人运动的影响，因此它们在动态条件下可能不准确
  // 陀螺仪可以测量机器人围绕各个轴的旋转速度。通过对这些速度进行积分，可以得到机器人的旋转角度。然而，陀螺仪的读数会随着时间的推移累积误差
  // 互补滤波器结合了加速度计和陀螺仪的优点，提供了一个更准确的角度估计。它主要使用陀螺仪的数据，但当检测到陀螺仪的误差增大时，会逐渐切换到使用加速度计的数据
}
