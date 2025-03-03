// #include <ICM20948.h>
// ICM20948 icm;

// float continuousYaw = 0;  // มุมสะสม
// float offset_yaw = 0;
// float read_Yaw = 0;
// float newYaw =0;

// float pitch = 0;
// float roll = 0;

// unsigned long lastUpdateTime = 0;
// const unsigned long updateInterval = 10;  // อ่านค่า IMU ทุก 10ms


// void resetBoard() {
//   Serial.println("Rebooting...");
//   while(1);
//   // delay(20000);              // หน่วงเวลาสั้น ๆ ให้ Serial ทันส่งข้อมูล
//   // watchdog_reboot(0, 0, 0);  // คำสั่ง Reset RP2040
// }
// void init_IMU() {
//   Wire.begin();
  
//   int code = icm.begin();
//   icm.setI2CAddress(0x69);
//   icm.setFusionMode(FUSION_6_AXIS);
//   Wire.setClock(400000);
//   switch (code) {
//     case ICM_SUCCESS:
//       Serial.println("Init ok");
//       break;
//     case ICM_BAD_WHOAMI:
//       Serial.println("Bad WHOAMI");
//       resetBoard();
//       break;
//     case ICM_DMP_ERROR:
//       Serial.println("Failed to initialize DMP");
//       resetBoard();
//       break;
//     case ICM_MAG_ERROR:
//       Serial.println("Magnetometer not found");
//       resetBoard();
//       break;
//     case ICM_SERIAL_ERROR:
//       Serial.println("Serial connection failure");
//       resetBoard();
//       break;
//     case ICM_SETUP_ERROR:
//       Serial.println("Device setup failure");
//       resetBoard();
//       break;
//   }
//   icm.startSensor(INV_SENSOR_TYPE_MAGNETOMETER, 16000);
//   icm.startSensor(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 16000);
//   icm.startSensor(INV_SENSOR_TYPE_ROTATION_VECTOR, 16000);
//   icm.setHighPowerMode(1);
// }



// void updateIMU() {
//   icm.update();
//   float qx = icm.x();
//   float qy = icm.y();
//   float qz = icm.z();
//   float qw = icm.w();
//   newYaw = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / PI;
//   float sinp = 2 * (qw * qy - qz * qx);
//   if (fabs(sinp) >= 1)
//     pitch = copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
//   else
//     pitch = asin(sinp);
//   pitch = pitch * 180.0 / M_PI;

//   float siny_cosp = 2 * (qw * qz + qx * qy);
//   float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
//   roll = atan2(siny_cosp, cosy_cosp);
//   roll = roll * 180.0 / M_PI;

//   //Serial.println(roll);

// }

// void updateIMU_fine_offset_Yaw(int round) {
//   for (int Rread = 0; Rread < round; Rread++) {
//     icm.update();
//     if (icm.available()) {
//       float qx = icm.x();
//       float qy = icm.y();
//       float qz = icm.z();
//       float qw = icm.w();
//       read_Yaw = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / PI;
//       if (read_Yaw < 0) { read_Yaw += 360; }
//       //Serial.print("  Yaw: "); Serial.println(Yaw);
//       icm.clearAvailable();
//     }
//     delay(10);
//   }
//   offset_yaw = read_Yaw;
// }



// float getYaw() {
//   updateIMU();
//   if (isnan(newYaw)) {
//     Serial.println("Error: Yaw is NaN! ใช้ค่าเดิม");
//     return read_Yaw;
//   }
//   if (newYaw < 0) newYaw += 360;
//   if (newYaw < 0 || newYaw > 360) {
//     Serial.println("Error: Yaw out of range! ใช้ค่าเดิม");
//     return read_Yaw;
//   }
//   float deltaYaw = newYaw - read_Yaw;
//   if (deltaYaw > 180) deltaYaw -= 360;   
//   if (deltaYaw < -180) deltaYaw += 360; 
//   if (abs(deltaYaw) > 30.0) {
//     Serial.println("Warning: Yaw changed too fast! ใช้ค่าเดิม");
//     return read_Yaw;
//   }
//   read_Yaw = newYaw;
//   if (read_Yaw < 0) read_Yaw += 360;
//   if (read_Yaw > 360) read_Yaw -= 360;
//   return read_Yaw;

// }
// float getPitch() {
//   updateIMU();
//   return pitch;
// }
// float getRoll(){
//   updateIMU();
//   return roll;
// }
// float getOffsetYaw() {
//   return offset_yaw;
// }


// void updateContinuousYaw() {
//   static float lastYaw = 0;
//   static bool firstRun = true;

//   updateIMU(); // อ่านค่าจาก IMU

//   float currentYaw = getYaw(); // อ่านค่า getYaw() แค่ครั้งเดียว

//   if (firstRun) {
//     continuousYaw = currentYaw;
//     lastYaw = currentYaw;
//     firstRun = false;
//     return;
//   }

//   // คำนวณ deltaYaw และ normalize ให้เป็นช่วง -180 ถึง 180 องศา
//   float deltaYaw = currentYaw - lastYaw;
//   if (deltaYaw > 180.0) deltaYaw -= 360.0;
//   if (deltaYaw < -180.0) deltaYaw += 360.0;

//   // อัปเดต continuousYaw
//   continuousYaw += deltaYaw;
  
//   // อัปเดต lastYaw
//   lastYaw = currentYaw;
// }

// float getContinuousYaw() {
//   updateContinuousYaw();
//   return continuousYaw;
// }
// void resetContinuousYaw(){
//   continuousYaw = getYaw();
// }
#ifndef _PUPPYBOT_IMU_H
#define _PUPPYBOT_IMU_H

#include <ICM20948.h>
#include <Wire.h>

namespace PuppyBotIMU {
    static ICM20948 icm;
    static float continuousYaw = 0;
    static float offset_yaw = 0;
    static float read_Yaw = 0;
    static float newYaw = 0;
    static float pitch = 0;
    static float roll = 0;

    void resetBoard() {
        Serial.println("Rebooting...");
        while (1);
    }

    void initIMU() {
        Wire.begin();
        int code = icm.begin();
        icm.setFusionMode(FUSION_6_AXIS);
        Wire.setClock(400000);

        switch (code) {
            case ICM_SUCCESS:
                Serial.println("Init ok");
                break;
            case ICM_BAD_WHOAMI:
            case ICM_DMP_ERROR:
            case ICM_MAG_ERROR:
            case ICM_SERIAL_ERROR:
            case ICM_SETUP_ERROR:
                Serial.println("ICM-20948 Initialization Failed");
                resetBoard();
                break;
        }
        icm.startSensor(INV_SENSOR_TYPE_MAGNETOMETER, 16000);
        icm.startSensor(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 16000);
        icm.startSensor(INV_SENSOR_TYPE_ROTATION_VECTOR, 16000);
        icm.setHighPowerMode(1);
    }
    void updateIMU_fine_offset_Yaw(int round) {
      for (int Rread = 0; Rread < round; Rread++) {
        icm.update();
        if (icm.available()) {
          float qx = icm.x();
          float qy = icm.y();
          float qz = icm.z();
          float qw = icm.w();
          read_Yaw = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / PI;
          if (read_Yaw < 0) { read_Yaw += 360; }
          //Serial.print("  Yaw: "); Serial.println(Yaw);
          icm.clearAvailable();
        }
        delay(10);
      }
      offset_yaw = read_Yaw;
    }

    void updateIMU() {
        icm.update();
        float qx = icm.x();
        float qy = icm.y();
        float qz = icm.z();
        float qw = icm.w();
        newYaw = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / PI;
        
        float sinp = 2 * (qw * qy - qz * qx);
        pitch = (fabs(sinp) >= 1) ? copysign(M_PI / 2, sinp) : asin(sinp);
        pitch *= 180.0 / M_PI;

        float siny_cosp = 2 * (qw * qz + qx * qy);
        float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        roll = atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
    }

    float getYaw() {
        updateIMU();
        if (isnan(newYaw)) return read_Yaw;
        if (newYaw < 0) newYaw += 360;
        if (newYaw > 360) newYaw -= 360;
        read_Yaw = newYaw;
        return read_Yaw;
    }
    void updateContinuousYaw() {
    static float lastYaw = 0;
    static bool firstRun = true;

    updateIMU(); // อ่านค่าจาก IMU

    float currentYaw = getYaw(); // อ่านค่า getYaw() แค่ครั้งเดียว

    if (firstRun) {
      continuousYaw = currentYaw;
      lastYaw = currentYaw;
      firstRun = false;
      return;
    }

    // คำนวณ deltaYaw และ normalize ให้เป็นช่วง -180 ถึง 180 องศา
    float deltaYaw = currentYaw - lastYaw;
    if (deltaYaw > 180.0) deltaYaw -= 360.0;
    if (deltaYaw < -180.0) deltaYaw += 360.0;

    // อัปเดต continuousYaw
    continuousYaw += deltaYaw;
    
    // อัปเดต lastYaw
    lastYaw = currentYaw;
  }

    float getPitch() { updateIMU(); return pitch; }
    float getRoll() { updateIMU(); return roll; }
    float getOffsetYaw() { return offset_yaw; }
    float getContinuousYaw() { updateContinuousYaw(); return continuousYaw; }
    void resetContinuousYaw() { continuousYaw = getYaw(); }
}

using namespace PuppyBotIMU;  // ทำให้เรียกใช้งานง่ายขึ้นโดยไม่ต้องพิมพ์ PuppyBotIMU::

#endif  // _PUPPYBOT_IMU_H
