#include "PuppyBotIMU.h"
enum Direction_turnPID { NORTH = 0,
                 EAST = 1,
                 SOUTH = 2,
                 WEST = 3 };

int Direction_turnPIDIndex = NORTH;  
float previousYaw_turnPID = 0;      

float preverror_turnPID = 0;
float integral_turn = 0;
float turnDirection_PID_KP=2,turnDirection_PID_KD=0;
int speedMin_turnDirection = 15,speedMax_turnDirection = 60;
int error_for_turnPID = 1;
int Set_Mode_Gyro = 0;


void set_data_for_turnDirection(int error_PID,int speedMin,int speedMax,float KP,float KD){
  turnDirection_PID_KP = KP;
  turnDirection_PID_KD = KD;
  speedMin_turnDirection = speedMin;
  speedMax_turnDirection = speedMax;
  error_for_turnPID = error_PID;
}

float shortestAngle(float currentAngle, float targetAngle) {
    float error = fmod((targetAngle - currentAngle + 540), 360) - 180;
    return error;
}
void turnPID(int condition, float targetYaw_turnPID, int speedTurn_min, int speedTurn_max, float kp_turnPID, float kd_turnPID) {
  unsigned long startTime = millis();
  integral_turn = 0;
  int stableCount = 0;
  int overshootCount = 0;
  bool hasOvershoot = false;
  
  float preverror_turnPID = shortestAngle(getContinuousYaw(), targetYaw_turnPID);

  while (true) {
    updateContinuousYaw();
    float current_Yaw = getContinuousYaw();

    float error_turnPID = shortestAngle(current_Yaw, targetYaw_turnPID);

    if ((preverror_turnPID > 0 && error_turnPID < 0) || (preverror_turnPID < 0 && error_turnPID > 0)) {
      hasOvershoot = true;
      overshootCount++;
    }

    unsigned long now = millis();
    float deltaTime = max((now - startTime) / 1000.0, 0.001);
    startTime = now;

    integral_turn += error_turnPID * deltaTime;
    integral_turn = constrain(integral_turn, -50, 50);
    float derivative = (error_turnPID - preverror_turnPID) / deltaTime;
    float output = kp_turnPID * error_turnPID + kd_turnPID * derivative;

    int speed_L = constrain(abs(output), speedTurn_min, speedTurn_max);
    int speed_R = constrain(abs(output), speedTurn_min, speedTurn_max);

    if (output < 0) {
      // เลี้ยวซ้าย
      if (condition == 1) {
        // 2 ล้อ (ซ้ายหยุด ขวาหมุนหน้า)
        motor(1, 0);
        motor(2, speed_R);
        if(Set_Mode_Gyro == 0){
          motor(3, 0);
          motor(4, speed_R);
        }
      } else if (condition == 0) {
        // 4 ล้อ (เลี้ยวรถถัง)
        motor(1, -speed_L);
        motor(2, speed_R);
        if(Set_Mode_Gyro == 0){
          motor(3, -speed_L);
          motor(4, speed_R);
        }
      }
    } else {
      // เลี้ยวขวา
      if (condition == 1) {
        // 2 ล้อ (ขวาหยุด ซ้ายหมุนหน้า)
        motor(1, speed_L);
        motor(2, 0);
        if(Set_Mode_Gyro == 0){
          motor(3, speed_L);
          motor(4, 0);
        }
      } else if (condition == 0) {
        // 4 ล้อ (เลี้ยวรถถัง)
        motor(1, speed_L);
        motor(2, -speed_R);
        if(Set_Mode_Gyro == 0){
          motor(3, speed_L);
          motor(4, -speed_R);
        }
      }
    }

    if (overshootCount > 5) {
      ao(); delay(10); ao();
      break;
    }

    if (abs(error_turnPID) < error_for_turnPID && hasOvershoot && overshootCount >= 2) {
      stableCount++;
      if (stableCount > 5) {
        ao(); delay(10); ao();
        break;
      }
    } else {
      stableCount = 0;
    }

    if (millis() - startTime > 5000) {
      Serial.println("Timeout! Exiting loop...");
      ao();
      break;
    }

    preverror_turnPID = error_turnPID;
  }
}

// void turnPID(int condition,float targetYaw_turnPID, int speedTurn_min, int speedTurn_max, float kp_turnPID, float kd_turnPID) {
//   unsigned long startTime = millis();
//   integral_turn = 0;
//   int stableCount = 0;
//   int overshootCount = 0;
//   bool hasOvershoot = false;
  
//   float preverror_turnPID = shortestAngle(getContinuousYaw(), targetYaw_turnPID);

//   while (true) {
//     updateContinuousYaw();
//     float current_Yaw = getContinuousYaw();

//     // เรียกใช้ shortestAngle อย่างถูกต้องเพื่อจัดการ wrap-around
//     float error_turnPID = shortestAngle(current_Yaw, targetYaw_turnPID);

//     if ((preverror_turnPID > 0 && error_turnPID < 0) || (preverror_turnPID < 0 && error_turnPID > 0)) {
//       hasOvershoot = true;
//       overshootCount++;
//     }

//     unsigned long now = millis();
//     float deltaTime = max((now - startTime) / 1000.0, 0.001);
//     startTime = now;

//     integral_turn += error_turnPID * deltaTime;
//     integral_turn = constrain(integral_turn, -50, 50);
//     float derivative = (error_turnPID - preverror_turnPID) / deltaTime;
//     float output = kp_turnPID * error_turnPID + kd_turnPID * derivative;

//     int speed_L = constrain(abs(output), speedTurn_min, speedTurn_max);
//     int speed_R = constrain(abs(output), speedTurn_min, speedTurn_max);
//     if (output < 0) {
//        motor(1, -speed_L);
//        motor(2, speed_R);
      
//       if(Set_Mode_Gyro == 0){
//         motor(3, -speed_L);
//         motor(4, speed_R);
//       }
     
//     } else {
//       motor(1, speed_L);
//       motor(2, -speed_R);

//       if(Set_Mode_Gyro == 0){
//         motor(3, speed_L);
//         motor(4, -speed_R);
//       }
//     }

//     if (overshootCount > 5) {
//       ao();
//       delay(10);
//       ao();
//       break;
//     }

//     if (abs(error_turnPID) < error_for_turnPID && hasOvershoot && overshootCount >= 2) {
//       stableCount++;
//       if (stableCount > 5) {
//         ao();
//         delay(10);
//         ao();
//         break;
//       }
//     } else {
//       stableCount = 0;
//     }

//     if (millis() - startTime > 5000) {
//       Serial.println("Timeout! Exiting loop...");
//       ao();
//       break;
//     }
//     preverror_turnPID = error_turnPID;
//   }
// }




void turn_45(int condition) {
  float targetYaw_turnPID = getOffsetYaw() - 135;
  targetYaw_turnPID = fmod(targetYaw_turnPID, 360);  //รักษาให้อยู่ใน 0-359°
  turnPID(condition,targetYaw_turnPID,speedMin_turnDirection,speedMax_turnDirection,turnDirection_PID_KP,turnDirection_PID_KD);
  resetContinuousYaw();
}
void turn_90(int condition) {
  float targetYaw_turnPID = getOffsetYaw() -90 ;
  targetYaw_turnPID = fmod(targetYaw_turnPID, 360);  //รักษาให้อยู่ใน 0-359°
  turnPID(condition,targetYaw_turnPID,speedMin_turnDirection,speedMax_turnDirection,turnDirection_PID_KP,turnDirection_PID_KD);
  resetContinuousYaw();
}
void turn_135(int condition) {
  float targetYaw_turnPID = getOffsetYaw() -45 ;
  targetYaw_turnPID = fmod(targetYaw_turnPID, 360);  //รักษาให้อยู่ใน 0-359°
  turnPID(condition,targetYaw_turnPID,speedMin_turnDirection,speedMax_turnDirection,turnDirection_PID_KP,turnDirection_PID_KD);
  resetContinuousYaw();
}
void turn_180(int condition) {
  float targetYaw_turnPID = getOffsetYaw() ;
  targetYaw_turnPID = fmod(targetYaw_turnPID, 360);  //รักษาให้อยู่ใน 0-359°
  turnPID(condition,targetYaw_turnPID,speedMin_turnDirection,speedMax_turnDirection,turnDirection_PID_KP,turnDirection_PID_KD);
  resetContinuousYaw();
}
void turn_225(int condition) {
  float targetYaw_turnPID = getOffsetYaw() + 45 ;
  targetYaw_turnPID = fmod(targetYaw_turnPID, 360);  //รักษาให้อยู่ใน 0-359°
  turnPID(condition,targetYaw_turnPID,speedMin_turnDirection,speedMax_turnDirection,turnDirection_PID_KP,turnDirection_PID_KD);
  resetContinuousYaw();
}
void turn_270(int condition) {
  float targetYaw_turnPID = getOffsetYaw() + 90 ;
  targetYaw_turnPID = fmod(targetYaw_turnPID, 360);  //รักษาให้อยู่ใน 0-359°
  turnPID(condition,targetYaw_turnPID,speedMin_turnDirection,speedMax_turnDirection,turnDirection_PID_KP,turnDirection_PID_KD);
  resetContinuousYaw();
}
void turn_315(int condition) {
  float targetYaw_turnPID = getOffsetYaw() +135 ;
  targetYaw_turnPID = fmod(targetYaw_turnPID, 360);  //รักษาให้อยู่ใน 0-359°
  turnPID(condition,targetYaw_turnPID,speedMin_turnDirection,speedMax_turnDirection,turnDirection_PID_KP,turnDirection_PID_KD);
  resetContinuousYaw();
}
void turn_360(int condition) {
  float targetYaw_turnPID = getOffsetYaw() +180 ;
  targetYaw_turnPID = fmod(targetYaw_turnPID, 360);  //รักษาให้อยู่ใน 0-359°
  turnPID(condition,targetYaw_turnPID,speedMin_turnDirection,speedMax_turnDirection,turnDirection_PID_KP,turnDirection_PID_KD);
  resetContinuousYaw();
}


void turnByAngle(int turnAngle,int condition) {
  updateContinuousYaw();
  float currentYaw = getContinuousYaw();  
  float targetYaw = currentYaw + turnAngle;

  targetYaw = fmod(targetYaw, 360);

  turnPID(condition,targetYaw, speedMin_turnDirection, speedMax_turnDirection, turnDirection_PID_KP, turnDirection_PID_KD);

  resetContinuousYaw();
}


float normalizeAngle(float angle) {
    while (angle > 180) angle -= 360;
    while (angle <= -180) angle += 360;
    return angle;
}

void moveStraightPID(int Movedirection, float targetYaw_straight, int speedBase, float duration, float kp_straight, float ki_straight, float kd_straight,int Slow) {
    unsigned long startTime = millis();
    unsigned long startTime_reduc = millis();
    unsigned long endTime = startTime + duration; 
    float integral_straight = 0, preverror_straight = 0;

    while (millis() < endTime) {  
        updateContinuousYaw();  
        float current_Yaw = getContinuousYaw();
        
        // 🔥 ใช้ normalizeAngle() เพื่อลดปัญหาการเปลี่ยนทิศทางผิดพลาด
        float error_straight = normalizeAngle(targetYaw_straight - current_Yaw);

        unsigned long now = millis();
        float deltaTime = max((now - startTime) / 1000.0, 0.001);
        startTime = now;

        integral_straight += error_straight * deltaTime;
        integral_straight = constrain(integral_straight, -50, 50);  
        float derivative = (error_straight - preverror_straight) / deltaTime;
        float output = kp_straight * error_straight + ki_straight * integral_straight + kd_straight * derivative;

        preverror_straight = error_straight;
        int rightSpeed;
        int leftSpeed;

        if(Slow == 1){
          unsigned long elapsedTime = now - startTime_reduc;
          long timeError = endTime - now;
          float speedReduction;
          if (timeError <= 500 && timeError > 0) {
            speedReduction = timeError * 0.01;
            speedReduction = constrain(speedReduction, 0, speedBase);
          } else if (timeError <= 0) {
            speedReduction = 0;  // หยุดเมื่อครบเวลา
          } else {
            speedReduction = speedBase; // ก่อนถึงช่วงชะลอ ให้เต็มที่
          }
          rightSpeed = constrain(speedReduction - output, 10, 100);
          leftSpeed = constrain(speedReduction + output, 10, 100);
        }
        else{
          rightSpeed = constrain(speedBase - output, 20, 100);
          leftSpeed = constrain(speedBase + output, 20, 100);
        }

    
        if (Movedirection == 0) {  // เดินหน้า
            motor(1, leftSpeed);
            motor(2, rightSpeed);
            if(Set_Mode_Gyro == 0){
              motor(3, leftSpeed);
              motor(4, rightSpeed);
            }
            
        } else {  // ถอยหลัง
            motor(1, -rightSpeed);
            motor(2, -leftSpeed);
            if(Set_Mode_Gyro == 0){
              motor(3, -rightSpeed);
              motor(4, -leftSpeed);
            } 
        }
    }

    ao(); delay(10);
    ao(); 
}


void MoveStraightDirection(int Movedirection, int targetYaw_straight, int speedBase, float duration, float kp_straight, float ki_straight, float kd_straight,int Slow) {
    float targetYaw_MovePID = 0;
    float currentYaw = getOffsetYaw(); // อ่านค่าปัจจุบัน

    if (getContinuousYaw() >= 360 || getContinuousYaw() <= 0) {
        resetContinuousYaw();
    }

    // คำนวณเป้าหมายตามทิศทาง
    switch (targetYaw_straight) {
        case 0: // N (North)
            targetYaw_MovePID = currentYaw;
            break;
        case 1: // E (East)
            targetYaw_MovePID = currentYaw + 90;
            break;
        case 2: // S (South)
            targetYaw_MovePID = currentYaw + 180;
            break;
        case 3: // W (West)
            targetYaw_MovePID = currentYaw - 90;
            break;
        case 4: // W (West)
            targetYaw_MovePID = currentYaw - 135;
            break;
        case 5: // W (West)
            targetYaw_MovePID = currentYaw - 45;
            break;
        case 6: // W (West)
            targetYaw_MovePID = currentYaw + 45;
            break;
        case 7: // W (West)
            targetYaw_MovePID = currentYaw + 135;
            break;
    }

    // 🔥 ใช้ normalizeAngle() ป้องกันมุมเกิน 360° หรือติดลบ
    targetYaw_MovePID = normalizeAngle(targetYaw_MovePID);

    Serial.print("Move Direction: "); Serial.print(Movedirection);
    Serial.print(" Target Yaw MovePID: "); Serial.println(targetYaw_MovePID);

    moveStraightPID(Movedirection, targetYaw_MovePID, speedBase, duration, kp_straight, ki_straight, kd_straight,Slow);
}







void moveStraightPID_Encoder(int Movedirection, float targetYaw_straight, int speedBase, int target_encoder, float kp_straight, float ki_straight, float kd_straight,int Slow) {
    long initialEncoderCount = get_pulse_Encoder();
    unsigned long startTime = millis();
    float integral_straight = 0, preverror_straight = 0;

    while (abs(get_pulse_Encoder() - initialEncoderCount) < target_encoder) { 
        updateContinuousYaw();  
        float current_Yaw = getContinuousYaw();
        float error_straight = normalizeAngle(targetYaw_straight - current_Yaw);

        unsigned long now = millis();
        float deltaTime = max((now - startTime) / 1000.0, 0.001);
        startTime = now;

        integral_straight += error_straight * deltaTime;
        integral_straight = constrain(integral_straight, -50, 50);  
        float derivative = (error_straight - preverror_straight) / deltaTime;
        float output = kp_straight * error_straight + ki_straight * integral_straight + kd_straight * derivative;

        preverror_straight = error_straight;

        int rightSpeed;
        int leftSpeed;

        if(Slow == 1){
          float kp_encoder = 0.05;  
          long encoderMoved = abs(get_pulse_Encoder() - initialEncoderCount);
          long encoderError = target_encoder - encoderMoved;
          float speedReduction = encoderError * kp_encoder;
          speedReduction = constrain(speedReduction, 10, speedBase);

          rightSpeed = constrain(speedReduction - output, 0, 100);
          leftSpeed = constrain(speedReduction + output, 0, 100);
        }
        else{
          rightSpeed = constrain(speedBase - output, 0, 100);
          leftSpeed = constrain(speedBase + output, 0, 100);

        }

        if (Movedirection == 0) {  // เดินหน้า
            motor(1, leftSpeed);
            motor(2, rightSpeed);
            if(Set_Mode_Gyro == 0){
              motor(3, leftSpeed);
              motor(4, rightSpeed);
            }
            
        } else {  // ถอยหลัง
            motor(1, -rightSpeed);
            motor(2, -leftSpeed);
            if(Set_Mode_Gyro == 0){
              motor(3, -rightSpeed);
              motor(4, -leftSpeed);
            } 
        }
    }

    ao(); delay(10);
    ao(); 
}

void MoveStraightDirection_Encoder(int Movedirection, int targetYaw_straight, int speedBase, float duration, float kp_straight, float ki_straight, float kd_straight,int Slow) {
    float targetYaw_MovePID = 0;
    float currentYaw = getOffsetYaw(); // อ่านค่าปัจจุบัน

    if (getContinuousYaw() >= 360 || getContinuousYaw() <= 0) {
        resetContinuousYaw();
    }
    switch (targetYaw_straight) {
        case 0: // N (North)
            targetYaw_MovePID = currentYaw;
            break;
        case 1: // E (East)
            targetYaw_MovePID = currentYaw + 90;
            break;
        case 2: // S (South)
            targetYaw_MovePID = currentYaw + 180;
            break;
        case 3: // W (West)
            targetYaw_MovePID = currentYaw - 90;
            break;
        case 4: // W (West)
            targetYaw_MovePID = currentYaw - 135;
            break;
        case 5: // W (West)
            targetYaw_MovePID = currentYaw - 45;
            break;
        case 6: // W (West)
            targetYaw_MovePID = currentYaw + 45;
            break;
        case 7: // W (West)
            targetYaw_MovePID = currentYaw + 135;
            break;
    }
    targetYaw_MovePID = normalizeAngle(targetYaw_MovePID);
    moveStraightPID_Encoder(Movedirection, targetYaw_MovePID, speedBase, duration, kp_straight, ki_straight, kd_straight,Slow);
}
void moveStraightPID_step(int Movedirection, float targetYaw_straight, int speedBase, float kp_straight, float kd_straight) {
    static float integral_straight = 0;
    static float preverror_straight = 0;
    static unsigned long lastTime = millis();

    updateContinuousYaw();  
    float current_Yaw = getContinuousYaw();
    float error_straight = normalizeAngle(targetYaw_straight - current_Yaw);

    unsigned long now = millis();
    float deltaTime = max((now - lastTime) / 1000.0, 0.001);
    lastTime = now;  // สำคัญ! ต้องใช้ static

    integral_straight += error_straight * deltaTime;
    integral_straight = constrain(integral_straight, -50, 50);
    float derivative = (error_straight - preverror_straight) / deltaTime;
    preverror_straight = error_straight;

    float output = kp_straight * error_straight + kd_straight * derivative;

    int rightSpeed = constrain(speedBase - output, 0, 100);
    int leftSpeed  = constrain(speedBase + output, 0, 100);

    if (Movedirection == 0) {
        motor(1, leftSpeed);
        motor(2, rightSpeed);
        if(Set_Mode_Gyro == 0){
            motor(3, leftSpeed);
            motor(4, rightSpeed);
        }
    } else {
        motor(1, -rightSpeed);
        motor(2, -leftSpeed);
        if(Set_Mode_Gyro == 0){
            motor(3, -rightSpeed);
            motor(4, -leftSpeed);
        } 
    }
}

void MoveStraightDirection_step(int Movedirection, int targetYaw_straight, int speedBase, float kp_straight, float kd_straight) {
    float targetYaw_MovePID = 0;
    float currentYaw = getOffsetYaw(); // อ่านค่าปัจจุบัน

    if (getContinuousYaw() >= 360 || getContinuousYaw() <= 0) {
        resetContinuousYaw();
    }
    switch (targetYaw_straight) {
        case 0: // N (North)
            targetYaw_MovePID = currentYaw;
            break;
        case 1: // E (East)
            targetYaw_MovePID = currentYaw + 90;
            break;
        case 2: // S (South)
            targetYaw_MovePID = currentYaw + 180;
            break;
        case 3: // W (West)
            targetYaw_MovePID = currentYaw - 90;
            break;
        case 4: // W (West)
            targetYaw_MovePID = currentYaw - 135;
            break;
        case 5: // W (West)
            targetYaw_MovePID = currentYaw - 45;
            break;
        case 6: // W (West)
            targetYaw_MovePID = currentYaw + 45;
            break;
        case 7: // W (West)
            targetYaw_MovePID = currentYaw + 135;
            break;
    }
    targetYaw_MovePID = normalizeAngle(targetYaw_MovePID);
    moveStraightPID_step(Movedirection, targetYaw_MovePID, speedBase, kp_straight, kd_straight);
}