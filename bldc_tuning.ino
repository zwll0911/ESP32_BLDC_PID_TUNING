#include <SPI.h>
#include <mcp2515.h>
#include <PS4Controller.h>

const int CS_Pin = 5; 

// --- MOTOR SETTINGS ---
const uint32_t RM_CMD_ID_1_4 = 0x200;
const float M3508_Current_Max = 16384.0f;
const float M2006_Current_Max = 10000.0f;
const float M3508_Gear_Reduction = 19.0f;
const float M2006_Gear_Reduction = 36.0f;

// --- TUNING VARIABLES ---
float TUNING_TARGET_RPM = 500.0f; 
bool enablePlotter = false;

// --- PID Structure ---
struct PID_Config {
  float Kp, Ki, Kd;       
  float integral, prevError;
};

// Array of 4 PIDs 
PID_Config pids[4] = {
  {20.0, 0.0, 0.0, 0.0, 0.0}, 
  {20.0, 0.0, 0.0, 0.0, 0.0}, 
  {20.0, 0.0, 0.0, 0.0, 0.0}, 
  {20.0, 0.0, 0.0, 0.0, 0.0}  
};

// --- Objects ---
struct can_frame canRead;
struct can_frame canWrite;
MCP2515 mcp2515(CS_Pin);

float currentRPMs[4] = {0, 0, 0, 0};

void onConnect() { Serial.println("PS4 Connected"); }
void onDisConnect() { Serial.println("PS4 Disconnected"); }

void PS4_Setup() {
  PS4.begin("84:1f:e8:68:f8:a2"); 
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
}

void ProcessMotorFeedback(uint32_t CAN_ID, uint8_t RPM_H, uint8_t RPM_L) { 
  int16_t RPM_Raw = (RPM_H << 8) | RPM_L; 
  float RPM_True = (float)RPM_Raw / M2006_Gear_Reduction;
  
  if (CAN_ID >= 0x201 && CAN_ID <= 0x204) {
    currentRPMs[CAN_ID - 0x201] = RPM_True;
  }
}

int16_t CalculatePID(float targetRPM, float actualRPM, PID_Config &pid) {
  float error = targetRPM - actualRPM;
  float P = pid.Kp * error;
  pid.integral += error;
  
  if (pid.integral > 3000) pid.integral = 3000;
  if (pid.integral < -3000) pid.integral = -3000;
  
  float I = pid.Ki * pid.integral;
  float D = pid.Kd * (error - pid.prevError);
  pid.prevError = error;
  
  float output = P + I + D;
  
  if (output > M2006_Current_Max) output = M2006_Current_Max;
  if (output < -M2006_Current_Max) output = -M2006_Current_Max;
  
  return (int16_t)output;
}

void CheckSerialTuning() {
  if (Serial.available() > 0) {
    char command = Serial.read(); 
    
    // --- COMMAND: VIEW SETTINGS ---
    if (command == '?') {
       Serial.printf("\n--- CURRENT SETTINGS (Target: %.0f) ---\n", TUNING_TARGET_RPM);
       for(int i=0; i<4; i++) {
         Serial.printf("M%d -> Kp:%.2f  Ki:%.4f  Kd:%.2f\n", 
                        i+1, pids[i].Kp, pids[i].Ki, pids[i].Kd);
       }
       Serial.println("---------------------------------------");
       while(Serial.available()) Serial.read(); 
       return; 
    }

    // --- COMMAND: TOGGLE PLOTTER ---
    if (command == 'v') {
       enablePlotter = !enablePlotter;
       if(enablePlotter) Serial.println(">> PLOTTER MODE: ON (Data will spam)");
       else Serial.println(">> PLOTTER MODE: OFF (Silent)");
       while(Serial.available()) Serial.read();
       return;
    }
    
    // --- COMMAND: SET SPEED ---
    if (command == 's') {
       float val = Serial.parseFloat();
       TUNING_TARGET_RPM = val;
       Serial.printf(">> Target Speed Set: %.0f RPM\n", TUNING_TARGET_RPM);
       while(Serial.available()) Serial.read();
       return;
    }

    // --- COMMAND: PID UPDATE ---
    int motorID = Serial.parseInt(); 
    char separator = Serial.read(); 
    float value = Serial.parseFloat();
    while(Serial.available()) Serial.read(); 

    int startIdx = (motorID == 0) ? 0 : motorID - 1;
    int endIdx   = (motorID == 0) ? 3 : motorID - 1;

    if (startIdx < 0) startIdx = 0;
    if (endIdx > 3) endIdx = 3;

    for(int i = startIdx; i <= endIdx; i++) {
        switch(command) {
          case 'p': pids[i].Kp = value; pids[i].integral = 0; break;
          case 'i': pids[i].Ki = value; pids[i].integral = 0; break;
          case 'd': pids[i].Kd = value; break;
        }
    }
    Serial.printf(">> Updated %c for Motor(s) %d to %.4f\n", command, motorID, value);
  }
}

void SendCANCommand(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
  canWrite.can_id = RM_CMD_ID_1_4; 
  canWrite.can_dlc = 8;            
  int16_t allMotors[4] = {m1, m2, m3, m4};
  for (int i = 0; i < 4; i++) {
    canWrite.data[i * 2]     = (allMotors[i] >> 8) & 0xFF; 
    canWrite.data[i * 2 + 1] = allMotors[i] & 0xFF;        
  }
  mcp2515.sendMessage(&canWrite);
}

void setup() {
  Serial.begin(115200);
  PS4_Setup();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); 
  mcp2515.setNormalMode();
  
  Serial.println("--- MULTI-MOTOR TUNING READY ---");
  Serial.println("  p1_5.0  -> Set M1 Kp to 5.0");
  Serial.println("  ?       -> View Settings");
  Serial.println("  v       -> Toggle Graph Data (On/Off)");
}

void loop() {
  CheckSerialTuning();
  
  while (mcp2515.readMessage(&canRead) == MCP2515::ERROR_OK) {
    ProcessMotorFeedback(canRead.can_id, canRead.data[2], canRead.data[3]);
  }

  int16_t outputM[4] = {0,0,0,0};
  float currentTarget = 0;

  if (PS4.isConnected()) {
    if (abs(PS4.LStickY()) > 10) {
      currentTarget = TUNING_TARGET_RPM;
    } else {
      for(int i=0; i<4; i++) {
          pids[i].integral = 0;
          pids[i].prevError = 0;
      }
    }
    for(int i=0; i<4; i++) {
       outputM[i] = CalculatePID(currentTarget, currentRPMs[i], pids[i]);
    }
  }

  SendCANCommand(outputM[0], outputM[1], outputM[2], outputM[3]);

  // Only print if 'v' was pressed to enable plotting
  if (enablePlotter) {
    Serial.printf("%.0f,%.0f,%.0f,%.0f,%.0f\n", 
                  currentTarget, currentRPMs[0], currentRPMs[1], currentRPMs[2], currentRPMs[3]);
  }

  delay(10);
}
