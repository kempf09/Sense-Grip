/*
  SENSE-GRIP Release 3.0.4-dev — Continuous position approach + 5% TOUCH window
  ESP32-S3-WROOM-2-N32R16V
  Hardware: 6x FSR402, 3x SEN0628, CF35-12 position servo

  Core 0: WebServer only. HTTP handlers queue commands; no ADC, I2C or motor UART access.
  Core 1: high-priority CF35-12 + FSR + FSM task; lower-priority SEN0628 I2C task.

  SEN0628 calibration storage and embedded 3D/calibration page are preserved from Release 2.2.1.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>
#include <DFRobot_MatrixLidar.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Forward declarations must be in a header: Arduino auto-generates function prototypes
// before the type definitions in this .ino file.
#include "SENSE_GRIP_Release_3_0_3_Types.h"

// ---------------- Hardware ----------------
enum State : uint8_t { IDLE, POSITIONING, GRASPING, HOLDING, RELEASING, PAUSED, FAULT };
static const char* stateName(State s) {
  static const char* names[] = {"IDLE","POSITIONING","GRASPING","HOLDING","RELEASING","PAUSED","FAULT"};
  return names[(uint8_t)s < 7 ? (uint8_t)s : 6];
}

static constexpr uint8_t FSR_COUNT = 6;
static const uint8_t FSR_PINS[FSR_COUNT] = {1,2,4,5,6,10};
static constexpr uint8_t SERVO_RX = 17;
static constexpr uint8_t SERVO_TX = 18;

static constexpr int I2C_SDA_PIN = 8;
static constexpr int I2C_SCL_PIN = 9;
static constexpr uint32_t I2C_FREQUENCY = 100000;
static constexpr uint8_t SENSOR_COUNT = 3;
static constexpr uint8_t SENSOR_ADDR[SENSOR_COUNT] = {0x33,0x30,0x31};
static const char* SENSOR_NAME[SENSOR_COUNT] = {"S1","S2","S3"};
static constexpr uint32_t FSR_PERIOD_MS = 20;
static constexpr uint32_t MOTOR_PERIOD_MS = 10;
static constexpr uint32_t MODE_CHECK_MS = 1500;
static constexpr uint32_t MOTOR_STALE_MS = 500;
static constexpr uint32_t POSITION_TIMEOUT_MS = 8000;
static constexpr uint16_t SETUP_STEP_COUNTS = 20;  // Controller-generated movement; no absolute position input.
static constexpr uint16_t DEFAULT_CONTACT_STEP_COUNTS = 20;
static constexpr uint16_t DEFAULT_GRASP_STEP_COUNTS = 10;

bool isAllowedSetupStep(long step) {
  return step==20 || step==50 || step==100 || step==250 || step==500 || step==1000;
}
// FSR array order follows FSR_PINS: index 0..5 = GPIO 1,2,4,5,6,10.
// User channel labels: FSR2=GPIO1, FSR1=GPIO2, FSR3=GPIO4,
// FSR4=GPIO5, FSR5=GPIO6, FSR6=GPIO10.
static constexpr uint8_t FSR_CHANNEL_NUMBER[FSR_COUNT] = {2,1,3,4,5,6};
// Finger 1: fingertip FSR6/GPIO10 + base FSR2/GPIO1.
// Finger 2: fingertip FSR4/GPIO5 + base FSR3/GPIO4.
// Finger 3: fingertip FSR5/GPIO6 + base FSR1/GPIO2.
// Values are FSR array indices, not printed channel numbers.
static constexpr uint8_t FINGER_FSR_MAP[3][2] = {{5,0},{3,2},{4,1}};

DFRobot_MatrixLidar_I2C lidar0(SENSOR_ADDR[0], &Wire);
DFRobot_MatrixLidar_I2C lidar1(SENSOR_ADDR[1], &Wire);
DFRobot_MatrixLidar_I2C lidar2(SENSOR_ADDR[2], &Wire);
DFRobot_MatrixLidar_I2C* const lidar[SENSOR_COUNT] = {&lidar0,&lidar1,&lidar2};

struct SensorState {
  bool found=false, initialized=false, configured=false, hasData=false, lastReadOK=false;
  uint16_t mm[64] = {0};
  uint32_t lastGoodMs=0, readUs=0, okCount=0, errorCount=0, frameSeq=0;
};
SensorState sensor[SENSOR_COUNT];
uint32_t lastScanUs=0;

struct SensorCalibration {
  uint8_t orientation;
  float rangeScale, rangeOffset, yawCorrectionDeg, pitchCorrectionDeg, rollCorrectionDeg;
  bool valid;
};
static constexpr uint8_t SENSOR_CAL_VERSION=1;
SensorCalibration sensorCal[SENSOR_COUNT] = {
  {4,1.0f,0.0f,0.0f,0.0f,0.0f,false},
  {4,1.0f,0.0f,0.0f,0.0f,0.0f,false},
  {4,1.0f,0.0f,0.0f,0.0f,0.0f,false}
};

// One short critical section protects snapshots shared by the web and controller cores.
portMUX_TYPE sharedMux = portMUX_INITIALIZER_UNLOCKED;
WebServer server(80);
HardwareSerial Bus(1);
uint8_t servoId=1;
uint32_t servoBaud=1000000;
static const uint32_t BAUDS[] = {1000000,500000,250000,128000,115200,76800,57600,38400};
enum { PING=1, READ=2, WRITE=3 };

// ---------------- FSR and compact FSM configuration ----------------
static const uint16_t ADC_SAMPLES=16;
static const float SUPPLY_MV=3300.0f;
static const float OPEN_OHMS=10000000.0f;
struct FsrConfig { float fixedOhms,f1,o1,f2,o2,zeroMv; };
struct FsrReading { uint16_t raw; float mv,ohms,force; };
FsrConfig fsrCfg[FSR_COUNT];
FsrReading fsr[FSR_COUNT];
bool fsrCalStored[FSR_COUNT]={false,false,false,false,false,false};
uint32_t fsrSampleSeq=0;
bool fingerTouchStable[3]={false,false,false};
uint8_t fingerTouchSamples[3]={0,0,0};

// Layout retained from Release 2.2.1 so existing saved motion references and settings remain readable.
struct FsmConfig {
  uint32_t magic;
  bool closePositive,autoStart,openZeroValid;
  int32_t openZero;
  uint16_t idleDetectMm,positionTargetMm,positionToleranceMm,maxFingerDeviationMm;
  float graspForceN,fsrSlipDropPct,contactForceN,maxForceN;
  uint16_t minimumActiveFsr,contactMinActiveFsr,graspSpeed,positionSpeed,releaseSpeed,closeCurrentLimit,openCurrentLimit,stallCurrentMa;
  uint8_t stallPercent,slipDeltaMm,slipConsecutive,maxRetries,contactConsecutive,forceConsecutive;
  uint16_t openPositionToleranceCounts;
  uint32_t positioningTimeoutMs,graspTimeoutMs,releaseTimeoutMs,stallDebounceMs;
  int32_t touchPosition;
  bool touchPositionValid;
  uint16_t travelSafetyMarginCounts,contactStepCounts,graspStepCounts,manualJogStepCounts;
};
static constexpr uint32_t FSM_MAGIC=0x46534D34;
static constexpr uint8_t FSM_CONFIG_VERSION=4;
FsmConfig fc;

void defaultFsm() {
  // Calibrated references confirmed by the user: OPEN=1968, three-finger TOUCH=19202.
  // These are fallback defaults only. Valid values already stored in NVS take precedence.
  fc = {};
  fc.magic = FSM_MAGIC;
  fc.closePositive = true;
  fc.autoStart = false;
  fc.openZeroValid = true;
  fc.openZero = 1968;
  fc.idleDetectMm = 150;                 // Retained legacy field; not used by this FSM.
  fc.positionTargetMm = 5;               // Retained legacy field; not used by this FSM.
  fc.positionToleranceMm = 3;             // Retained legacy field; not used by this FSM.
  fc.maxFingerDeviationMm = 8;            // Retained legacy field; not used by this FSM.
  fc.graspForceN = 1.00f;                 // Initial low-force grasp target.
  fc.fsrSlipDropPct = 25.0f;              // Warn after a 25% drop from HOLDING force.
  fc.contactForceN = 0.25f;               // Either FSR on each finger group can indicate contact.
  fc.maxForceN = 30.00f;                  // Per-channel software ceiling; FSR calibration range is 0–30 N.
  fc.minimumActiveFsr = 3;                // At least three individual FSR channels contribute.
  fc.contactMinActiveFsr = 1;             // Retained legacy field; finger contact is group-based.
  fc.graspSpeed = 10;                     // CF35-12 speed command units.
  fc.positionSpeed = 61;                  // Maximum documented CF35-12 speed command (~44.7 RPM).
  fc.releaseSpeed = 61;                   // Maximum documented speed; avoids unnecessarily slow full-travel release.
  fc.closeCurrentLimit = 200;             // Existing CF35-12 current-limit command value.
  fc.openCurrentLimit = 260;              // Existing CF35-12 current-limit command value.
  fc.stallCurrentMa = 2700;                // Retained legacy field; not used by this simple FSM.
  fc.stallPercent = 90;
  fc.slipDeltaMm = 10;                     // Retained legacy field; no ToF logic is used.
  fc.slipConsecutive = 3;
  fc.maxRetries = 0;                       // No automatic retry cycle.
  fc.contactConsecutive = 3;               // 3 samples at 20 ms = approximately 60 ms.
  fc.forceConsecutive = 3;                 // 3 samples at 20 ms = approximately 60 ms.
  fc.openPositionToleranceCounts = 8;
  fc.positioningTimeoutMs = 8000;          // Continuous approach timeout; full calibrated travel is about 5.6 s at max no-load speed.
  fc.graspTimeoutMs = 15000;
  fc.releaseTimeoutMs = 30000;             // 30 s allowance for loaded return to saved OPEN zero.
  fc.stallDebounceMs = 150;                // Retained legacy field.
  fc.touchPosition = 19202;
  fc.touchPositionValid = true;
  fc.travelSafetyMarginCounts = 100;
  fc.contactStepCounts = 50;               // Coarse approach steps; stop on 3-finger contact.
  fc.graspStepCounts = 10;                 // Smaller force-build steps.
  fc.manualJogStepCounts = SETUP_STEP_COUNTS;
}

static constexpr uint16_t MAXD=128;
struct MotorReading {
  bool ok=false, moving=false, targetActive=false;
  int16_t position=0,targetPosition=0,speed=0,current=0;
  float degrees=0,mA=0,load=0,voltage=0;
  uint8_t temp=0,status=0,mode=255;
  bool modeKnown=false;
};
MotorReading motor;
struct Cf35DiagSnapshot {
  bool infoOk=false,limitsOk=false,phaseOk=false,modeOk=false,feedbackOk=false;
  uint8_t info[5]={0},limits[4]={0},phase=0,mode=255,feedback[18]={0};
  uint32_t capturedMs=0;
};
Cf35DiagSnapshot cf35Diag;
volatile bool cf35DiagRequested=false;
uint32_t motorLastGoodMs=0;
bool lastMotorCommandOk=true;
char lastMotorCommandError[96]="";
volatile bool estopRequested=false; // Core-0 emergency stop bypasses the ordinary command queue.

State state=IDLE, previousState=IDLE;
uint32_t stateSince=0;
bool fsmEnabled=false;
bool forceLossWarning=false;
float holdForceReference=0;
uint8_t contactStableSamples=0,graspStableSamples=0,forceLossSamples=0;
char transitionReason[144]="Booting";
char lastAction[144]="Controller starting";
bool lastActionOk=true;

// One motor target can be active at a time. This is separate from the FSM state.
enum MotionOwner : uint8_t { MOTION_NONE, MOTION_SETUP_STEP, MOTION_SETUP_OPEN, MOTION_FSM_CONTACT, MOTION_FSM_GRASP, MOTION_FSM_RELEASE };
MotionOwner motionOwner=MOTION_NONE;
bool motionActive=false;
bool setupStepClosing=false;
int32_t motionTarget=0,motionStartPosition=0;
uint32_t motionStartedMs=0;
uint32_t motionTimeoutMs=POSITION_TIMEOUT_MS;

// ---------------- FreeRTOS command hand-off ----------------
enum ControlCommandType : uint8_t {
  CMD_START, CMD_RELEASE, CMD_PAUSE, CMD_RESET, CMD_ESTOP,
  CMD_STEP_OPEN, CMD_STEP_CLOSE, CMD_SET_OPEN_ZERO, CMD_SET_TOUCH, CMD_MOVE_OPEN
};
struct ControlCommand { ControlCommandType type; uint16_t step; };
TaskHandle_t webTaskHandle=nullptr, controlTaskHandle=nullptr, sensorTaskHandle=nullptr;
QueueHandle_t controlQueue=nullptr;
volatile bool controllerTaskReady=false;

void setReason(const char* text) {
  portENTER_CRITICAL(&sharedMux);
  strncpy(transitionReason,text,sizeof(transitionReason)-1);
  transitionReason[sizeof(transitionReason)-1]='\0';
  portEXIT_CRITICAL(&sharedMux);
}
void setAction(const char* text,bool ok=true) {
  portENTER_CRITICAL(&sharedMux);
  strncpy(lastAction,text,sizeof(lastAction)-1);
  lastAction[sizeof(lastAction)-1]='\0';
  lastActionOk=ok;
  portEXIT_CRITICAL(&sharedMux);
}
void setState(State next,const char* why) {
  portENTER_CRITICAL(&sharedMux);
  previousState=state;
  state=next;
  stateSince=millis();
  strncpy(transitionReason,why,sizeof(transitionReason)-1);
  transitionReason[sizeof(transitionReason)-1]='\0';
  portEXIT_CRITICAL(&sharedMux);
}
void setMotionActive(bool active) {
  portENTER_CRITICAL(&sharedMux); motionActive=active; portEXIT_CRITICAL(&sharedMux);
}
void setFsmEnabled(bool enabled) {
  portENTER_CRITICAL(&sharedMux); fsmEnabled=enabled; portEXIT_CRITICAL(&sharedMux);
}
void setForceLossWarning(bool warning) {
  portENTER_CRITICAL(&sharedMux); forceLossWarning=warning; portEXIT_CRITICAL(&sharedMux);
}

// ---------------- Preferences ----------------
void loadAll() {
  Preferences p;
  if(p.begin("fsr402",true)) {
    for(uint8_t i=0;i<FSR_COUNT;i++) {
      fsrCfg[i]={2000,1,30000,10,3000,0};
      String k="s"+String(i);
      fsrCalStored[i]=p.isKey((k+"r").c_str())&&p.isKey((k+"f1").c_str())&&p.isKey((k+"o1").c_str())&&p.isKey((k+"f2").c_str())&&p.isKey((k+"o2").c_str());
      fsrCfg[i].fixedOhms=p.getFloat((k+"r").c_str(),fsrCfg[i].fixedOhms);
      fsrCfg[i].f1=p.getFloat((k+"f1").c_str(),fsrCfg[i].f1); fsrCfg[i].o1=p.getFloat((k+"o1").c_str(),fsrCfg[i].o1);
      fsrCfg[i].f2=p.getFloat((k+"f2").c_str(),fsrCfg[i].f2); fsrCfg[i].o2=p.getFloat((k+"o2").c_str(),fsrCfg[i].o2);
      fsrCfg[i].zeroMv=p.getFloat((k+"z").c_str(),0);
    }
    p.end();
  } else {
    for(uint8_t i=0;i<FSR_COUNT;i++) fsrCfg[i]={2000,1,30000,10,3000,0};
  }

  if(p.begin("senCal2",true)) {
    uint8_t v=p.getUChar("ver",0);
    for(uint8_t i=0;i<SENSOR_COUNT;i++) {
      String k=String(i);
      sensorCal[i].orientation=(v==SENSOR_CAL_VERSION)?p.getUChar(("o"+k).c_str(),4):4;
      sensorCal[i].rangeScale=(v==SENSOR_CAL_VERSION)?p.getFloat(("s"+k).c_str(),1.0f):1.0f;
      sensorCal[i].rangeOffset=(v==SENSOR_CAL_VERSION)?p.getFloat(("b"+k).c_str(),0.0f):0.0f;
      sensorCal[i].yawCorrectionDeg=(v==SENSOR_CAL_VERSION)?p.getFloat(("y"+k).c_str(),0.0f):0.0f;
      sensorCal[i].pitchCorrectionDeg=(v==SENSOR_CAL_VERSION)?p.getFloat(("p"+k).c_str(),0.0f):0.0f;
      sensorCal[i].rollCorrectionDeg=(v==SENSOR_CAL_VERSION)?p.getFloat(("r"+k).c_str(),0.0f):0.0f;
      sensorCal[i].valid=(v==SENSOR_CAL_VERSION)&&p.getBool(("v"+k).c_str(),false);
      if(sensorCal[i].orientation>7||!isfinite(sensorCal[i].rangeScale)||sensorCal[i].rangeScale<0.8f||sensorCal[i].rangeScale>1.2f||
         !isfinite(sensorCal[i].rangeOffset)||fabsf(sensorCal[i].rangeOffset)>40.0f||
         !isfinite(sensorCal[i].yawCorrectionDeg)||fabsf(sensorCal[i].yawCorrectionDeg)>8.0f||
         !isfinite(sensorCal[i].pitchCorrectionDeg)||fabsf(sensorCal[i].pitchCorrectionDeg)>8.0f||
         !isfinite(sensorCal[i].rollCorrectionDeg)||fabsf(sensorCal[i].rollCorrectionDeg)>8.0f)
        sensorCal[i]={4,1.0f,0.0f,0.0f,0.0f,0.0f,false};
    }
    p.end();
  }

  defaultFsm();
  bool migrateFsmConfig=false;
  if(p.begin("fsm",true)) {
    if(p.getBytesLength("cfg")==sizeof(fc)) {
      FsmConfig stored{};
      if(p.getBytes("cfg",&stored,sizeof(fc))==sizeof(fc)&&stored.magic==FSM_MAGIC) {
        fc=stored;
        const uint8_t configVersion=p.getUChar("cfgv",0);
        if(configVersion<FSM_CONFIG_VERSION) {
          // Replace only old factory defaults. Preserve settings that the user changed.
          if(fabsf(fc.contactForceN-0.20f)<0.001f) fc.contactForceN=0.25f;
          if(fabsf(fc.graspForceN-2.00f)<0.001f) fc.graspForceN=1.00f;
          if(fabsf(fc.maxForceN-5.00f)<0.001f) fc.maxForceN=3.00f;
          if(configVersion<4 && fabsf(fc.maxForceN-3.00f)<0.001f) fc.maxForceN=30.00f;
          if(configVersion<4 && fc.releaseSpeed==20) fc.releaseSpeed=61;
          if(configVersion<4 && fc.releaseTimeoutMs==20000) fc.releaseTimeoutMs=30000;
          if(fc.contactStepCounts==20) fc.contactStepCounts=50;
          if(fc.positionSpeed==20) fc.positionSpeed=15;
          if(fc.positionSpeed==15) fc.positionSpeed=61;
          if(fc.positioningTimeoutMs==30000) fc.positioningTimeoutMs=8000;
          if(fc.graspSpeed==30) fc.graspSpeed=10;
          if(fc.releaseSpeed==30) fc.releaseSpeed=20;
          if(fc.travelSafetyMarginCounts==20) fc.travelSafetyMarginCounts=100;
          if(fc.positioningTimeoutMs==5000) fc.positioningTimeoutMs=30000;
          if(fc.graspTimeoutMs==7000) fc.graspTimeoutMs=15000;
          if(fc.releaseTimeoutMs==7000) fc.releaseTimeoutMs=20000;
          // Apply the user-confirmed reference values only when no references were saved.
          if(!fc.openZeroValid && fc.openZero==0) { fc.openZero=1968; fc.openZeroValid=true; }
          if(!fc.touchPositionValid && fc.touchPosition==0) { fc.touchPosition=19202; fc.touchPositionValid=true; }
          migrateFsmConfig=true;
        }
      }
    }
    p.end();
  }
  if(migrateFsmConfig && p.begin("fsm",false)) {
    p.putBytes("cfg",&fc,sizeof(fc));
    p.putUChar("cfgv",FSM_CONFIG_VERSION);
    p.end();
  }
  // Keep old fsmLog namespace untouched. Release 3.0 does not use persistent fault logs.
  if(p.begin("cf35",true)) {
    servoId=p.getUChar("id",1);
    servoBaud=p.getULong("baud",1000000);
    p.end();
  }
  // Enforce useful minimum increment sizes. Old Release 2.2.1 defaults were below gearbox backlash.
  if(fc.contactStepCounts<10)fc.contactStepCounts=DEFAULT_CONTACT_STEP_COUNTS;
  if(fc.graspStepCounts<10)fc.graspStepCounts=DEFAULT_GRASP_STEP_COUNTS;
  if(fc.manualJogStepCounts<10)fc.manualJogStepCounts=SETUP_STEP_COUNTS;
  // Clamp persisted CF35-12 speed commands to the documented 61-unit maximum.
  fc.positionSpeed=min((uint16_t)61,max((uint16_t)1,fc.positionSpeed));
  fc.graspSpeed=min((uint16_t)61,max((uint16_t)1,fc.graspSpeed));
  fc.releaseSpeed=min((uint16_t)61,max((uint16_t)1,fc.releaseSpeed));
}
bool saveFsm() {
  FsmConfig copy;
  portENTER_CRITICAL(&sharedMux); copy=fc; portEXIT_CRITICAL(&sharedMux);
  Preferences p;
  if(!p.begin("fsm",false)) return false;
  size_t n=p.putBytes("cfg",&copy,sizeof(copy));
  size_t v=p.putUChar("cfgv",FSM_CONFIG_VERSION);
  p.end();
  return n==sizeof(copy)&&v==sizeof(uint8_t);
}

// ---------------- CF35-12 serial protocol (Core 1 only) ----------------
struct Resp { bool ok=false; uint8_t id=0,status=0,n=0,data[MAXD]{}; char error[40]=""; };
void busBegin(uint32_t baud) {
  servoBaud=baud;
  Bus.end(); delay(15);
  Bus.begin(baud,SERIAL_8N1,SERVO_RX,SERVO_TX);
  delay(25);
  while(Bus.available())Bus.read();
}
bool getByte(uint8_t& v,uint32_t timeoutUs=25000) {
  uint32_t start=micros();
  while((uint32_t)(micros()-start)<timeoutUs) { if(Bus.available()){v=(uint8_t)Bus.read();return true;} taskYIELD(); }
  return false;
}
Resp receivePacket(uint8_t wantId,uint8_t wantLength=255) {
  Resp r; uint8_t a=0,b=0; uint32_t start=micros();
  while((uint32_t)(micros()-start)<25000) { if(Bus.available()){b=(uint8_t)Bus.read();if(a==255&&b==255)break;a=b;} taskYIELD(); }
  if(!(a==255&&b==255)){strcpy(r.error,"packet timeout");return r;}
  uint8_t len=0;
  if(!getByte(r.id)||!getByte(len)||!getByte(r.status)){strcpy(r.error,"short header");return r;}
  if(len<2||len-2>MAXD){strcpy(r.error,"bad length");return r;}
  r.n=len-2; uint8_t sum=r.id+len+r.status;
  for(uint8_t i=0;i<r.n;i++){if(!getByte(r.data[i])){strcpy(r.error,"short data");return r;}sum+=r.data[i];}
  uint8_t checksum=0;if(!getByte(checksum)){strcpy(r.error,"missing checksum");return r;}
  r.ok=(uint8_t)(~sum)==checksum&&r.id==wantId&&(wantLength==255||r.n==wantLength);
  if(!r.ok)strcpy(r.error,"checksum/id/length");
  return r;
}
Resp sendPacket(uint8_t id,uint8_t instruction,const uint8_t* payload,uint8_t n,bool reply=true,uint8_t replyLength=255) {
  while(Bus.available())Bus.read();
  uint8_t len=n+2,sum=id+len+instruction;
  Bus.write(255);Bus.write(255);Bus.write(id);Bus.write(len);Bus.write(instruction);
  for(uint8_t i=0;i<n;i++){Bus.write(payload[i]);sum+=payload[i];}
  Bus.write((uint8_t)(~sum));Bus.flush();
  if(!reply){Resp r;r.ok=true;return r;}
  return receivePacket(id,replyLength);
}
bool readBus(uint8_t id,uint8_t address,uint8_t n,uint8_t* out,char* error,size_t errorCap=64) {
  uint8_t q[2]={address,n};Resp r=sendPacket(id,READ,q,2,true,n);
  if(!r.ok){strncpy(error,r.error,errorCap-1);error[errorCap-1]='\0';return false;}
  memcpy(out,r.data,n);return true;
}
bool writeBus(uint8_t id,uint8_t address,const uint8_t* data,uint8_t n,char* error,size_t errorCap=64) {
  uint8_t q[MAXD];q[0]=address;memcpy(q+1,data,n);Resp r=sendPacket(id,WRITE,q,n+1,true,0);
  if(!r.ok){strncpy(error,r.error,errorCap-1);error[errorCap-1]='\0';return false;}
  return true;
}
uint16_t u16(const uint8_t* p){return p[0]|(uint16_t(p[1])<<8);}
void p16(uint16_t v,uint8_t* p){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
int16_t sm15(uint16_t v){return (v&0x8000)?-int16_t(v&0x7fff):int16_t(v);}
int16_t sm10(uint16_t v){return (v&0x400)?-int16_t(v&0x3ff):int16_t(v);}
void psm(long v,uint8_t* p){uint16_t x=(uint16_t)labs(v);if(v<0)x|=0x8000;p16(x,p);}
bool readMotorMode() {
  uint8_t mode=255;char e[64]="";
  if(!readBus(servoId,33,1,&mode,e,sizeof(e))||mode>3) {
    portENTER_CRITICAL(&sharedMux);motor.modeKnown=false;motor.mode=255;portEXIT_CRITICAL(&sharedMux);
    return false;
  }
  portENTER_CRITICAL(&sharedMux);motor.mode=mode;motor.modeKnown=true;portEXIT_CRITICAL(&sharedMux);
  return true;
}
bool motorModeReady() { return motor.modeKnown&&motor.mode==0; }
bool torque(uint8_t value) { char e[64]="";bool ok=writeBus(servoId,40,&value,1,e,sizeof(e));if(!ok){strncpy(lastMotorCommandError,e,sizeof(lastMotorCommandError)-1);lastMotorCommandError[sizeof(lastMotorCommandError)-1]='\0';}return ok; }
bool motorPosition(long pos,long speed,long acceleration,long currentLimit) {
  if(pos<-32767||pos>32767||speed<1||speed>32767||acceleration<0||acceleration>254||currentLimit<1||currentLimit>430)return false;
  uint8_t d[7];d[0]=(uint8_t)acceleration;psm(pos,d+1);p16((uint16_t)currentLimit,d+3);p16((uint16_t)speed,d+5);
  char e[64]="";bool ok=writeBus(servoId,41,d,7,e,sizeof(e));
  if(!ok){strncpy(lastMotorCommandError,e,sizeof(lastMotorCommandError)-1);lastMotorCommandError[sizeof(lastMotorCommandError)-1]='\0';}
  return ok;
}
bool motorStop() {
  uint8_t zero[2]={0,0},torqueOff=0;char e1[64]="",e2[64]="";
  bool currentOk=writeBus(servoId,44,zero,2,e1,sizeof(e1));
  bool torqueOk=writeBus(servoId,40,&torqueOff,1,e2,sizeof(e2));
  lastMotorCommandOk=currentOk&&torqueOk;
  if(!lastMotorCommandOk){const char* e=!currentOk?e1:e2;strncpy(lastMotorCommandError,e,sizeof(lastMotorCommandError)-1);lastMotorCommandError[sizeof(lastMotorCommandError)-1]='\0';}
  else lastMotorCommandError[0]='\0';
  return lastMotorCommandOk;
}
// In HOLDING, keep position mode and torque enabled. Re-command the current
// encoder position so the servo holds the grasp instead of becoming limp.
bool motorHoldAtCurrentPosition(uint16_t currentLimit) {
  if(!motor.ok||!motorModeReady())return false;
  if(!torque(1))return false;
  if(!motorPosition(motor.position,fc.graspSpeed,20,currentLimit)) {
    motorStop();
    return false;
  }
  return true;
}
bool pollMotor() {
  uint8_t b[18];char e[64]="";
  if(!readBus(servoId,56,18,b,e,sizeof(e))) {
    portENTER_CRITICAL(&sharedMux);motor.ok=false;portEXIT_CRITICAL(&sharedMux);
    return false;
  }
  MotorReading next{};
  portENTER_CRITICAL(&sharedMux);next.mode=motor.mode;next.modeKnown=motor.modeKnown;portEXIT_CRITICAL(&sharedMux);
  next.position=sm15(u16(b));next.speed=sm15(u16(b+2));next.load=sm10(u16(b+4))*0.001f;
  next.voltage=b[6]*0.1f;next.temp=b[7];next.status=b[9];
  next.moving=(b[10]&1)!=0;next.targetActive=(b[10]&2)!=0;next.targetPosition=sm15(u16(b+11));
  next.current=sm15(u16(b+13));next.mA=next.current*6.5f;next.degrees=next.position*0.087f;next.ok=true;
  portENTER_CRITICAL(&sharedMux);motor=next;motorLastGoodMs=millis();portEXIT_CRITICAL(&sharedMux);
  return true;
}

// Read-only register survey. Runs only in the Core-1 controller task.
// Registers 65 and 67-68 are included in the 56-73 feedback block.
void pollCf35Diagnostics() {
  Cf35DiagSnapshot next{};
  char e[64]="";
  next.infoOk=readBus(servoId,0,5,next.info,e,sizeof(e));
  next.limitsOk=readBus(servoId,9,4,next.limits,e,sizeof(e));
  next.phaseOk=readBus(servoId,18,1,&next.phase,e,sizeof(e));
  next.modeOk=readBus(servoId,33,1,&next.mode,e,sizeof(e));
  next.feedbackOk=readBus(servoId,56,18,next.feedback,e,sizeof(e));
  next.capturedMs=millis();
  portENTER_CRITICAL(&sharedMux);cf35Diag=next;portEXIT_CRITICAL(&sharedMux);
}

bool beginPositionMove(long target,uint16_t speed,uint16_t currentLimit,MotionOwner owner,uint32_t timeoutMs=POSITION_TIMEOUT_MS) {
  if(!motorModeReady()||!motor.ok||motionActive||state==FAULT)return false;
  if(target<-32767||target>32767)return false;
  if(!torque(1)||!motorPosition(target,speed,20,currentLimit))return false;
  motionTarget=target;motionStartPosition=motor.position;motionStartedMs=millis();motionTimeoutMs=timeoutMs;motionOwner=owner;setMotionActive(true);
  lastMotorCommandOk=true;lastMotorCommandError[0]='\0';
  return true;
}
bool positionReached(long target,uint16_t tolerance) {
  return motor.ok&&!motor.moving&&!motor.targetActive&&labs((long)motor.position-target)<=tolerance;
}

// ---------------- FSR acquisition and force ----------------
float resistanceToForce(uint8_t i,float ohms) {
  if(!isfinite(ohms)||ohms<=0||ohms>=OPEN_OHMS)return 0;
  FsrConfig c;portENTER_CRITICAL(&sharedMux);c=fsrCfg[i];portEXIT_CRITICAL(&sharedMux);
  if(c.o1<=0||c.o2<=0||c.f1<=0||c.f2<=0||c.o1==c.o2)return NAN;
  float b=logf(c.f2/c.f1)/logf(c.o2/c.o1);float a=c.f1/powf(c.o1,b);return a*powf(ohms,b);
}
void sampleFsr() {
  FsrConfig cfg[FSR_COUNT];float contactThreshold;uint8_t contactSamplesRequired;
  portENTER_CRITICAL(&sharedMux);
  memcpy(cfg,fsrCfg,sizeof(cfg));contactThreshold=fc.contactForceN;contactSamplesRequired=fc.contactConsecutive;
  portEXIT_CRITICAL(&sharedMux);
  FsrReading next[FSR_COUNT];
  for(uint8_t i=0;i<FSR_COUNT;i++) {
    uint32_t sum=0;for(uint8_t n=0;n<ADC_SAMPLES;n++)sum+=analogRead(FSR_PINS[i]);
    next[i].raw=sum/ADC_SAMPLES;
    float measured=analogReadMilliVolts(FSR_PINS[i]);next[i].mv=fmaxf(0,measured-cfg[i].zeroMv);
    if(next[i].mv<1)next[i].ohms=OPEN_OHMS;
    else if(next[i].mv>=SUPPLY_MV-1)next[i].ohms=0;
    else next[i].ohms=cfg[i].fixedOhms*(SUPPLY_MV/next[i].mv-1);
    if(!isfinite(next[i].ohms)||next[i].ohms<=0||next[i].ohms>=OPEN_OHMS)next[i].force=0;
    else if(cfg[i].o1<=0||cfg[i].o2<=0||cfg[i].f1<=0||cfg[i].f2<=0||cfg[i].o1==cfg[i].o2)next[i].force=NAN;
    else {float b=logf(cfg[i].f2/cfg[i].f1)/logf(cfg[i].o2/cfg[i].o1);float a=cfg[i].f1/powf(cfg[i].o1,b);next[i].force=a*powf(next[i].ohms,b);}
  }
  portENTER_CRITICAL(&sharedMux);
  memcpy(fsr,next,sizeof(next));++fsrSampleSeq;
  for(uint8_t f=0;f<3;f++) {
    float a=next[FINGER_FSR_MAP[f][0]].force,b=next[FINGER_FSR_MAP[f][1]].force;
    bool hit=(isfinite(a)&&a>=contactThreshold)||(isfinite(b)&&b>=contactThreshold);
    if(hit){if(fingerTouchSamples[f]<255)fingerTouchSamples[f]++;}else fingerTouchSamples[f]=0;
    fingerTouchStable[f]=fingerTouchSamples[f]>=contactSamplesRequired;
  }
  portEXIT_CRITICAL(&sharedMux);
}
float averageForce(uint8_t* activeOut=nullptr) {
  float sum=0;uint8_t n=0;
  for(uint8_t i=0;i<FSR_COUNT;i++)if(isfinite(fsr[i].force)&&fsr[i].force>0.05f){sum+=fsr[i].force;n++;}
  if(activeOut)*activeOut=n;return n?sum/n:0;
}
float maximumFsrForce() { float m=0;for(uint8_t i=0;i<FSR_COUNT;i++)if(isfinite(fsr[i].force)&&fsr[i].force>m)m=fsr[i].force;return m; }
float fingerForce(uint8_t f) { float a=fsr[FINGER_FSR_MAP[f][0]].force,b=fsr[FINGER_FSR_MAP[f][1]].force;if(!isfinite(a))a=0;if(!isfinite(b))b=0;return fmaxf(a,b); }
uint8_t activeFingerCount() { uint8_t n=0;for(uint8_t f=0;f<3;f++)if(fingerTouchStable[f])n++;return n; }
bool allThreeFingersTouching() { return fingerTouchStable[0]&&fingerTouchStable[1]&&fingerTouchStable[2]; }
bool fsrCalibrationValid(uint8_t i) {
  if(i>=FSR_COUNT)return false;FsrConfig c;bool stored;
  portENTER_CRITICAL(&sharedMux);c=fsrCfg[i];stored=fsrCalStored[i];portEXIT_CRITICAL(&sharedMux);
  return stored&&isfinite(c.fixedOhms)&&c.fixedOhms>0&&isfinite(c.zeroMv)&&c.zeroMv>=0&&c.zeroMv<=1000&&
    isfinite(c.f1)&&c.f1>=0.2f&&c.f1<=30.0f&&isfinite(c.f2)&&c.f2>=0.2f&&c.f2<=30.0f&&
    isfinite(c.o1)&&c.o1>0&&c.o1<OPEN_OHMS&&isfinite(c.o2)&&c.o2>0&&c.o2<OPEN_OHMS&&c.o1!=c.o2&&c.f1!=c.f2;
}
bool allFsrCalibrated() { for(uint8_t i=0;i<FSR_COUNT;i++)if(!fsrCalibrationValid(i))return false;return true; }

// ---------------- SEN0628 acquisition (Core 1 only) ----------------
void initializeSensors() {
  for(uint8_t i=0;i<SENSOR_COUNT;i++) {
    Wire.beginTransmission(SENSOR_ADDR[i]);bool found=(Wire.endTransmission()==0);
    bool initialized=found&&(lidar[i]->begin()==0);
    bool configured=initialized&&(lidar[i]->setRangingMode(eMatrix_8X8)==0);
    portENTER_CRITICAL(&sharedMux);sensor[i].found=found;sensor[i].initialized=initialized;sensor[i].configured=configured;portEXIT_CRITICAL(&sharedMux);
  }
}
void readOneSensor(uint8_t i) {
  if(i>=SENSOR_COUNT||!sensor[i].found||!sensor[i].initialized||!sensor[i].configured)return;
  uint16_t values[64]={0};uint32_t t0=micros();uint8_t result=lidar[i]->getAllData(values);uint32_t elapsed=micros()-t0;
  portENTER_CRITICAL(&sharedMux);
  sensor[i].readUs=elapsed;
  if(result==0){memcpy(sensor[i].mm,values,sizeof(values));sensor[i].hasData=true;sensor[i].lastReadOK=true;sensor[i].lastGoodMs=millis();sensor[i].okCount++;sensor[i].frameSeq++;}
  else {sensor[i].lastReadOK=false;sensor[i].errorCount++;}
  portEXIT_CRITICAL(&sharedMux);
}

// ---------------- Simple controller / FSM (Core 1 only) ----------------
int closeSign() { return fc.closePositive?1:-1; }
long safeClosedPosition() { return (long)fc.touchPosition-(long)closeSign()*fc.travelSafetyMarginCounts; }
long calibratedTravelCounts() { return fc.touchPositionValid&&fc.openZeroValid?labs((long)fc.touchPosition-fc.openZero):0; }
bool travelValid() { return fc.openZeroValid&&fc.touchPositionValid&&calibratedTravelCounts()>(2L*fc.travelSafetyMarginCounts+2); }
// Stop the continuous approach when the encoder enters the final 5% of the
// calibrated OPEN-to-TOUCH travel. The safety margin can make this window wider.
bool touchWindowReached() {
  const long travel=calibratedTravelCounts();
  if(travel<=0)return false;
  const long window=max((long)lroundf((float)travel*0.05f),(long)fc.travelSafetyMarginCounts);
  const long threshold=(long)fc.touchPosition-(long)closeSign()*window;
  return closeSign()*((long)motor.position-threshold)>=0;
}
void setFault(const char* why) {
  setFsmEnabled(false);setMotionActive(false);motionOwner=MOTION_NONE;setupStepClosing=false;motorStop();setState(FAULT,why);setAction(why,false);
}
void setIdle(const char* why) {
  setFsmEnabled(false);setMotionActive(false);motionOwner=MOTION_NONE;setupStepClosing=false;motorStop();setState(IDLE,why);setAction(why,true);
}
void commandNextFsmStep(bool graspStep) {
  if(!motor.ok||!motorModeReady()){setFault("Motor link or position mode is not ready");return;}
  uint16_t step=graspStep?max((uint16_t)10,fc.graspStepCounts):max((uint16_t)10,fc.contactStepCounts);
  long target=(long)motor.position+(long)closeSign()*step;
  long limit=safeClosedPosition();
  if(closeSign()*(target-limit)>0)target=limit;
  if(target==motor.position){setFault("Safe closed travel limit reached before the target force was reached");return;}
  MotionOwner owner=graspStep?MOTION_FSM_GRASP:MOTION_FSM_CONTACT;
  if(!beginPositionMove(target,graspStep?fc.graspSpeed:fc.positionSpeed,fc.closeCurrentLimit,owner,POSITION_TIMEOUT_MS))
    setFault("Could not send the next position step to CF35-12");
}
void beginRelease() {
  if(!fc.openZeroValid){setFault("OPEN zero is not saved");return;}
  setFsmEnabled(true);setState(RELEASING,"Releasing to saved OPEN zero");
  if(!beginPositionMove(fc.openZero,fc.releaseSpeed,fc.openCurrentLimit,MOTION_FSM_RELEASE,fc.releaseTimeoutMs))setFault("Could not command release to OPEN zero");
}
void updatePositionMotion() {
  if(!motionActive)return;
  if(!motor.ok||(uint32_t)(millis()-motorLastGoodMs)>MOTOR_STALE_MS){
    if(motionOwner==MOTION_SETUP_STEP||motionOwner==MOTION_SETUP_OPEN){setMotionActive(false);motionOwner=MOTION_NONE;motorStop();setAction("Servo step stopped: motor feedback is stale",false);}
    else setFault("Motor feedback lost during FSM movement");
    setupStepClosing=false;
    return;
  }
  // The FSM approach is one continuous encoder-position command. Stop at the
  // 5% calibrated TOUCH window and hold the measured position.
  if(motionOwner==MOTION_FSM_CONTACT && touchWindowReached()) {
    setMotionActive(false);motionOwner=MOTION_NONE;
    holdForceReference=averageForce();setForceLossWarning(false);forceLossSamples=0;
    if(!motorHoldAtCurrentPosition(fc.closeCurrentLimit)) {setFault("Could not hold position after reaching the TOUCH window");return;}
    setFsmEnabled(true);
    setState(HOLDING,"Saved TOUCH position window reached (within 5% of calibrated travel)");
    setAction("Continuous approach complete — position hold active inside the 5% TOUCH window",true);
    return;
  }
  // Large manual close increments remain supervised while they move.
  // Stop at three-finger contact or at the configured per-channel force ceiling.
  if(motionOwner==MOTION_SETUP_STEP && setupStepClosing) {
    if(maximumFsrForce()>=fc.maxForceN) {
      setMotionActive(false);motionOwner=MOTION_NONE;setupStepClosing=false;motorStop();
      setAction("Closing step stopped: maximum FSR force limit reached",false);return;
    }
    if(allThreeFingersTouching()) {
      setMotionActive(false);motionOwner=MOTION_NONE;setupStepClosing=false;
      if(motorHoldAtCurrentPosition(fc.closeCurrentLimit))
        setAction("Closing step stopped at three-finger contact. Save TOUCH when ready.",true);
      else {motorStop();setAction("Three-finger contact detected, but position hold failed",false);}
      return;
    }
  }
  uint16_t tol=fc.openPositionToleranceCounts?fc.openPositionToleranceCounts:8;
  if(!motor.moving&&!motor.targetActive&&labs((long)motor.position-motionTarget)<=tol) {
    MotionOwner completed=motionOwner;setMotionActive(false);motionOwner=MOTION_NONE;setupStepClosing=false;
    if(completed==MOTION_SETUP_STEP)setAction("Controller position step completed",true);
    else if(completed==MOTION_SETUP_OPEN)setAction("Reached saved OPEN zero",true);
    else if(completed==MOTION_FSM_RELEASE){motorStop();setFsmEnabled(false);setState(IDLE,"Release complete — OPEN zero reached");setAction("Release complete",true);}
    return;
  }
  if(millis()-motionStartedMs>=motionTimeoutMs) {
    MotionOwner failed=motionOwner;setMotionActive(false);motionOwner=MOTION_NONE;setupStepClosing=false;motorStop();
    if(failed==MOTION_SETUP_STEP||failed==MOTION_SETUP_OPEN){setAction("Servo move timed out. Check the mechanism and motor link.",false);}
    else setFault("FSM motor move timed out before reaching its encoder target");
  }
}
void updateFsm() {
  if(!fsmEnabled||state==IDLE||state==PAUSED||state==FAULT)return;
  if(!motor.ok||(uint32_t)(millis()-motorLastGoodMs)>MOTOR_STALE_MS){setFault("Motor link lost during FSM cycle");return;}
  if(!motorModeReady()){setFault("CF35-12 is not in verified position mode 0");return;}
  if(maximumFsrForce()>=fc.maxForceN){setFault("Maximum FSR force limit reached");return;}

  static uint32_t lastProcessedFsrSeq=0;
  if(fsrSampleSeq!=lastProcessedFsrSeq) {
    lastProcessedFsrSeq=fsrSampleSeq;
    uint8_t active=0;float avg=averageForce(&active);
    if(state==GRASPING) {
      if(avg>=fc.graspForceN&&active>=fc.minimumActiveFsr) {if(graspStableSamples<255)graspStableSamples++;}
      else graspStableSamples=0;
      if(graspStableSamples>=max((uint8_t)1,fc.forceConsecutive)) {
        setMotionActive(false);motionOwner=MOTION_NONE;
        if(!motorHoldAtCurrentPosition(fc.closeCurrentLimit)){setFault("Could not maintain servo torque at grasp force");return;}
        holdForceReference=avg;setForceLossWarning(false);forceLossSamples=0;
        setFsmEnabled(true);setState(HOLDING,"Target grasp force reached and stable");setAction("Grasp established — HOLDING with position torque enabled",true);return;
      }
    } else if(state==HOLDING) {
      if(holdForceReference>0&&avg<holdForceReference*(1.0f-fc.fsrSlipDropPct/100.0f)) {if(forceLossSamples<255)forceLossSamples++;}
      else forceLossSamples=0;
      setForceLossWarning(forceLossSamples>=max((uint8_t)1,fc.slipConsecutive));
    }
  }

  if(motionActive)return;
  uint32_t elapsed=millis()-stateSince;
  if(state==POSITIONING) {
    if(elapsed>=fc.positioningTimeoutMs) setFault("Continuous approach timed out before entering the 5% TOUCH window");
    else if(!motionActive) setFault("Continuous approach stopped before entering the 5% TOUCH window");
  } else if(state==GRASPING) {
    if(closeSign()*(safeClosedPosition()-(long)motor.position)<=0) {setFault("Safe closed limit reached before target grasp force");return;}
    if(elapsed>=fc.graspTimeoutMs){setFault("Grasp force target timed out");return;}
    commandNextFsmStep(true);
  } else if(state==RELEASING) {
    if(positionReached(fc.openZero,fc.openPositionToleranceCounts)){motorStop();setFsmEnabled(false);setState(IDLE,"Release complete — OPEN zero reached");}
    else if(elapsed>=fc.releaseTimeoutMs)setFault("Release to OPEN zero timed out");
  }
}
void processControlCommand(const ControlCommand& cmd) {
  if(cmd.type==CMD_ESTOP){setFault("Emergency stop requested from web interface");return;}
  if(cmd.type==CMD_RESET) {
    if(!motor.ok||!motorModeReady()){setAction("Reset blocked: motor link or position mode is not ready",false);return;}
    setIdle("Fault/reset acknowledged by operator");return;
  }
  if(cmd.type==CMD_PAUSE) {
    setFsmEnabled(false);setMotionActive(false);motionOwner=MOTION_NONE;setupStepClosing=false;motorStop();setState(PAUSED,"Paused by operator");setAction("FSM paused",true);return;
  }
  if(cmd.type==CMD_RELEASE) { if(state==FAULT){setAction("Release blocked while FAULT is active. Reset first.",false);return;}if(!motor.ok||!motorModeReady()){setAction("Release blocked: motor link or position mode is not ready",false);return;}beginRelease();return; }
  if(cmd.type==CMD_START) {
    if(state!=IDLE||motionActive){setAction("Start blocked: controller is not idle",false);return;}
    if(!motor.ok||!motorModeReady()){setAction("Start blocked: CF35-12 must be online in position mode 0",false);return;}
    if(!allFsrCalibrated()){setAction("Start blocked: calibrate all six FSR channels first",false);return;}
    if(!travelValid()){setAction("Start blocked: save OPEN zero and the three-finger TOUCH position first",false);return;}
    if(labs((long)motor.position-fc.openZero)>fc.openPositionToleranceCounts){setAction("Start blocked: move the gripper to saved OPEN zero first",false);return;}
    if(maximumFsrForce()>=fc.maxForceN){setAction("Start blocked: reduce FSR force before closing",false);return;}
    setFsmEnabled(true);contactStableSamples=0;graspStableSamples=0;setForceLossWarning(false);
    setState(POSITIONING,"Continuous approach to saved TOUCH position; stop inside final 5% of calibrated travel");
    if(!beginPositionMove(fc.touchPosition,min((uint16_t)61,max((uint16_t)1,fc.positionSpeed)),fc.closeCurrentLimit,MOTION_FSM_CONTACT,fc.positioningTimeoutMs)) {
      setFault("Could not start continuous approach to saved TOUCH position");return;
    }
    setAction("Continuous approach started toward saved TOUCH position",true);return;
  }
  if(cmd.type==CMD_STEP_OPEN||cmd.type==CMD_STEP_CLOSE) {
    if(state!=IDLE||fsmEnabled||motionActive||motor.moving||motor.targetActive){setAction("Servo setup step blocked: FSM must be IDLE and the servo must be stationary",false);return;}
    if(!motor.ok||!motorModeReady()){setAction("Servo setup step blocked: motor link or position mode is not ready",false);return;}
    if(cmd.type==CMD_STEP_CLOSE&&!allFsrCalibrated()){setAction("Closing step blocked: calibrate all six FSR channels first",false);return;}
    if(cmd.type==CMD_STEP_CLOSE&&allThreeFingersTouching()){setAction("All three finger groups already show contact. Save the TOUCH reference before closing further.",false);return;}
    if(cmd.type==CMD_STEP_CLOSE&&maximumFsrForce()>=fc.maxForceN){setAction("Closing step blocked: maximum FSR force limit reached",false);return;}
    uint16_t step=cmd.step;
    if(!isAllowedSetupStep(step)){setAction("Invalid setup step. Select 20, 50, 100, 250, 500 or 1000 counts.",false);return;}
    long target=(long)motor.position+(cmd.type==CMD_STEP_CLOSE?(long)closeSign()*step:-(long)closeSign()*step);
    if(cmd.type==CMD_STEP_OPEN&&fc.openZeroValid&&closeSign()*(target-(long)fc.openZero)<0)target=fc.openZero;
    if(cmd.type==CMD_STEP_CLOSE&&fc.touchPositionValid&&closeSign()*(target-safeClosedPosition())>0)target=safeClosedPosition();
    if(target==motor.position){setAction("Servo is already at the calibrated travel limit",false);return;}
    if(!beginPositionMove(target,fc.positionSpeed,cmd.type==CMD_STEP_CLOSE?fc.closeCurrentLimit:fc.openCurrentLimit,MOTION_SETUP_STEP)) {setupStepClosing=false;setAction("Could not send servo setup step",false);}
    else {setupStepClosing=(cmd.type==CMD_STEP_CLOSE);setAction(cmd.type==CMD_STEP_CLOSE?"Controller closing step started":"Controller opening step started",true);}
    return;
  }
  if(cmd.type==CMD_MOVE_OPEN) {
    if(state!=IDLE||motionActive||motor.moving||motor.targetActive||!fc.openZeroValid||!motor.ok||!motorModeReady()){setAction("Move OPEN blocked: require IDLE, a stationary servo, motor mode 0 and a saved OPEN zero",false);return;}
    if(!beginPositionMove(fc.openZero,fc.releaseSpeed,fc.openCurrentLimit,MOTION_SETUP_OPEN,fc.releaseTimeoutMs))setAction("Could not move to OPEN zero",false);
    else setAction("Moving to saved OPEN zero",true);
    return;
  }
  if(cmd.type==CMD_SET_OPEN_ZERO) {
    if(state!=IDLE||motionActive||!motor.ok||!motorModeReady()||motor.moving||motor.targetActive){setAction("OPEN zero can only be saved while the motor is stationary in IDLE",false);return;}
    FsmConfig old;portENTER_CRITICAL(&sharedMux);old=fc;fc.openZero=motor.position;fc.openZeroValid=true;fc.touchPosition=0;fc.touchPositionValid=false;portEXIT_CRITICAL(&sharedMux);
    if(!saveFsm()){portENTER_CRITICAL(&sharedMux);fc=old;portEXIT_CRITICAL(&sharedMux);setAction("Failed to save OPEN zero to NVS",false);return;}
    setAction("OPEN zero saved. Three-finger TOUCH reference must now be measured again.",true);return;
  }
  if(cmd.type==CMD_SET_TOUCH) {
    if(state!=IDLE||motionActive||!fc.openZeroValid||!motor.ok||!motorModeReady()||motor.moving||motor.targetActive){setAction("TOUCH reference requires IDLE, a saved OPEN zero and a stationary motor",false);return;}
    if(!allFsrCalibrated()){setAction("TOUCH reference blocked: all six FSR channels must be calibrated",false);return;}
    if(!allThreeFingersTouching()){setAction("TOUCH reference not saved: all three finger groups must show contact",false);return;}
    long travel=(long)closeSign()*((long)motor.position-fc.openZero);
    if(travel<=2L*fc.travelSafetyMarginCounts+2){setAction("TOUCH position is too close to OPEN zero; move further toward contact",false);return;}
    FsmConfig old;portENTER_CRITICAL(&sharedMux);old=fc;fc.touchPosition=motor.position;fc.touchPositionValid=true;portEXIT_CRITICAL(&sharedMux);
    if(!saveFsm()){portENTER_CRITICAL(&sharedMux);fc=old;portEXIT_CRITICAL(&sharedMux);setAction("Failed to save three-finger TOUCH position to NVS",false);return;}
    setAction("Three-finger TOUCH position saved",true);return;
  }
}
void processControlQueue() {
  if(!controlQueue)return;ControlCommand cmd;
  while(xQueueReceive(controlQueue,&cmd,0)==pdTRUE)processControlCommand(cmd);
}

// ---------------- JSON / snapshots / Web APIs (Core 0 only) ----------------
String esc(String s){s.replace("\\","\\\\");s.replace("\"","\\\"");s.replace("\n","\\n");return "\""+s+"\"";}
void json(bool ok,const String& body,int code=200){server.sendHeader("Cache-Control","no-store");server.send(code,"application/json",String("{\"ok\":")+(ok?"true":"false")+","+body+"}");}
long argL(const char* n,long d=0){return server.hasArg(n)?strtol(server.arg(n).c_str(),nullptr,10):d;}
float argF(const char* n,float d=0){return server.hasArg(n)?server.arg(n).toFloat():d;}
bool enqueueCommand(ControlCommandType type,uint16_t step=SETUP_STEP_COUNTS) {
  QueueHandle_t q;bool ready;
  portENTER_CRITICAL(&sharedMux);q=controlQueue;ready=controllerTaskReady;portEXIT_CRITICAL(&sharedMux);
  if(!q||!ready)return false;ControlCommand c{type,step};return xQueueSend(q,&c,0)==pdTRUE;
}
void queueCommandApi(ControlCommandType type) {
  if(type==CMD_ESTOP) {
    portENTER_CRITICAL(&sharedMux);bool ready=controllerTaskReady;if(ready)estopRequested=true;portEXIT_CRITICAL(&sharedMux);
    if(ready)json(true,"\"queued\":true,\"priority\":\"emergency stop\"");
    else json(false,"\"error\":\"controller task is not running\"",503);
    return;
  }
  uint16_t step=SETUP_STEP_COUNTS;
  if(type==CMD_STEP_OPEN||type==CMD_STEP_CLOSE) {
    long requested=argL("step",SETUP_STEP_COUNTS);
    if(!isAllowedSetupStep(requested)) {json(false,"\"error\":\"step must be one of 20, 50, 100, 250, 500 or 1000 counts\"",400);return;}
    step=(uint16_t)requested;
  }
  if(enqueueCommand(type,step))json(true,"\"queued\":true,\"step\":"+String(step));
  else json(false,"\"error\":\"controller command queue is full or unavailable\"",503);
}

float avgForceLocal(const FsrReading* readings,uint8_t* active) {
  float total=0;uint8_t n=0;for(uint8_t i=0;i<FSR_COUNT;i++)if(isfinite(readings[i].force)&&readings[i].force>0.05f){total+=readings[i].force;n++;}
  if(active)*active=n;return n?total/n:0;
}
float maxForceLocal(const FsrReading* readings) {float m=0;for(uint8_t i=0;i<FSR_COUNT;i++)if(isfinite(readings[i].force)&&readings[i].force>m)m=readings[i].force;return m;}
bool calValidLocal(uint8_t i,const FsrConfig& c,bool stored) {
  return stored&&isfinite(c.fixedOhms)&&c.fixedOhms>0&&isfinite(c.zeroMv)&&c.zeroMv>=0&&c.zeroMv<=1000&&
    isfinite(c.f1)&&c.f1>=0.2f&&c.f1<=20&&isfinite(c.f2)&&c.f2>=0.2f&&c.f2<=20&&
    isfinite(c.o1)&&c.o1>0&&c.o1<OPEN_OHMS&&isfinite(c.o2)&&c.o2>0&&c.o2<OPEN_OHMS&&c.o1!=c.o2&&c.f1!=c.f2;
}
void apiStatus() {
  MotorReading m;FsrReading fr[FSR_COUNT];SensorState ss[SENSOR_COUNT];FsmConfig c;FsrConfig cfg[FSR_COUNT];bool stored[FSR_COUNT];
  State st,prev;uint32_t stSince,lastGood;char reason[sizeof(transitionReason)],action[sizeof(lastAction)];bool actionOk,enabled,loss,activeMotion;
  bool touches[3];uint8_t touchCounts[3];
  portENTER_CRITICAL(&sharedMux);
  m=motor;memcpy(fr,fsr,sizeof(fr));memcpy(ss,sensor,sizeof(ss));c=fc;memcpy(cfg,fsrCfg,sizeof(cfg));memcpy(stored,fsrCalStored,sizeof(stored));
  st=state;prev=previousState;stSince=stateSince;lastGood=motorLastGoodMs;activeMotion=motionActive;
  strncpy(reason,transitionReason,sizeof(reason)-1);reason[sizeof(reason)-1]='\0';
  strncpy(action,lastAction,sizeof(action)-1);action[sizeof(action)-1]='\0';actionOk=lastActionOk;enabled=fsmEnabled;loss=forceLossWarning;
  for(uint8_t f=0;f<3;f++){touches[f]=fingerTouchStable[f];touchCounts[f]=fingerTouchSamples[f];}
  portEXIT_CRITICAL(&sharedMux);
  uint8_t active=0;float avg=avgForceLocal(fr,&active),maximum=maxForceLocal(fr);bool allCal=true;
  for(uint8_t i=0;i<FSR_COUNT;i++)if(!calValidLocal(i,cfg[i],stored[i])){allCal=false;break;}
  long travel=(c.touchPositionValid&&c.openZeroValid)?labs((long)c.touchPosition-c.openZero):0;
  bool travelOk=c.openZeroValid&&c.touchPositionValid&&travel>(2L*c.travelSafetyMarginCounts+2);
  uint8_t activeFinger=0;for(uint8_t f=0;f<3;f++)if(touches[f])activeFinger++;
  String j="\"state\":"+esc(stateName(st))+",\"previous\":"+esc(stateName(prev))+",\"reason\":"+esc(String(reason))+
    ",\"stateMs\":"+String(millis()-stSince)+",\"fsmEnabled\":"+(enabled?"true":"false")+
    ",\"lastAction\":"+esc(String(action))+",\"lastActionOk\":"+(actionOk?"true":"false")+
    ",\"openZero\":"+String(c.openZero)+",\"openZeroValid\":"+(c.openZeroValid?"true":"false")+
    ",\"touchPosition\":"+String(c.touchPosition)+",\"touchPositionValid\":"+(c.touchPositionValid?"true":"false")+
    ",\"travelValid\":"+(travelOk?"true":"false")+",\"travelCounts\":"+String(travel)+
    ",\"safeClosedPosition\":"+String(c.touchPosition-(long)(c.closePositive?1:-1)*c.travelSafetyMarginCounts)+
    ",\"travelMarginCounts\":"+String(c.travelSafetyMarginCounts)+
    ",\"fsrCalibrationOk\":"+(allCal?"true":"false")+",\"activeFsr\":"+String(active)+
    ",\"activeFingers\":"+String(activeFinger)+",\"allThreeTouching\":"+(activeFinger==3?"true":"false")+
    ",\"avgForce\":"+String(avg,3)+",\"maxFsrForce\":"+String(maximum,3)+",\"maxForceLimitN\":"+String(c.maxForceN,2)+
    ",\"forceLossWarning\":"+(loss?"true":"false")+
    ",\"motorMode\":"+String(m.mode)+",\"motorModeKnown\":"+(m.modeKnown?"true":"false")+
    ",\"motorModeReady\":"+((m.modeKnown&&m.mode==0)?"true":"false")+
    ",\"motorStatusAgeMs\":"+String(millis()-lastGood)+
    ",\"motionActive\":"+(activeMotion?"true":"false")+
    ",\"motor\":{\"online\":"+(m.ok?"true":"false")+",\"position\":"+String(m.position)+
    ",\"targetPosition\":"+String(m.targetPosition)+",\"degrees\":"+String(m.degrees,2)+
    ",\"speed\":"+String(m.speed)+",\"mA\":"+String(m.mA,1)+",\"temp\":"+String(m.temp)+
    ",\"voltage\":"+String(m.voltage,1)+",\"moving\":"+(m.moving?"true":"false")+",\"targetActive\":"+(m.targetActive?"true":"false")+"}";
  j+=",\"finger\":[";
  for(uint8_t f=0;f<3;f++){if(f)j+=',';uint8_t a=FINGER_FSR_MAP[f][0],b=FINGER_FSR_MAP[f][1];j+=String("{\"touching\":")+(touches[f]?"true":"false")+",\"forceN\":"+String(fmaxf(fr[a].force,fr[b].force),3)+",\"fsrA\":"+String(FSR_CHANNEL_NUMBER[a])+",\"gpioA\":"+String(FSR_PINS[a])+",\"fsrB\":"+String(FSR_CHANNEL_NUMBER[b])+",\"gpioB\":"+String(FSR_PINS[b])+"}";}
  j+="],\"fsr\":[";
  for(uint8_t i=0;i<FSR_COUNT;i++){if(i)j+=',';j+="{\"channel\":"+String(FSR_CHANNEL_NUMBER[i])+",\"gpio\":"+String(FSR_PINS[i])+",\"raw\":"+String(fr[i].raw)+",\"mv\":"+String(fr[i].mv,1)+",\"ohms\":"+String(fr[i].ohms,1)+",\"force\":"+(isfinite(fr[i].force)?String(fr[i].force,3):String("null"))+",\"calibrated\":"+(calValidLocal(i,cfg[i],stored[i])?"true":"false")+"}";}
  j+="],\"sen\":[";
  for(uint8_t i=0;i<SENSOR_COUNT;i++){if(i)j+=',';j+=String("{\"online\":")+(ss[i].lastReadOK?"true":"false")+",\"ok\":"+String(ss[i].okCount)+",\"err\":"+String(ss[i].errorCount)+"}";}
  j+="]";json(true,j);
}
void requestCf35Diagnostics() {
  if(!controllerTaskReady) { server.send(503,"application/json","{\\"ok\\":false,\\"error\\":\\"Controller not ready\\"}"); return; }
  portENTER_CRITICAL(&sharedMux);cf35DiagRequested=true;portEXIT_CRITICAL(&sharedMux);
  server.sendHeader("Cache-Control","no-store");server.send(202,"application/json","{\\"ok\\":true}");
}
void apiCf35Diagnostics() {
  Cf35DiagSnapshot d;portENTER_CRITICAL(&sharedMux);d=cf35Diag;portEXIT_CRITICAL(&sharedMux);
  String j="{\"ageMs\":"+String(d.capturedMs?millis()-d.capturedMs:0)+
    ",\"infoOk\":"+(d.infoOk?"true":"false")+",\"info\":["+String(d.info[0])+","+String(d.info[1])+","+String(d.info[2])+","+String(d.info[3])+","+String(d.info[4])+"]"+
    ",\"limitsOk\":"+(d.limitsOk?"true":"false")+",\"limits\":["+String(d.limits[0])+","+String(d.limits[1])+","+String(d.limits[2])+","+String(d.limits[3])+"]"+
    ",\"phaseOk\":"+(d.phaseOk?"true":"false")+",\"phase\":"+String(d.phase)+
    ",\"modeOk\":"+(d.modeOk?"true":"false")+",\"mode\":"+String(d.mode)+
    ",\"feedbackOk\":"+(d.feedbackOk?"true":"false")+",\"feedback\":[";
  for(uint8_t i=0;i<18;i++){if(i)j+=',';j+=String(d.feedback[i]);}
  j+="]}";server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",j);
}
void apiConfig() {
  FsmConfig c;portENTER_CRITICAL(&sharedMux);c=fc;portEXIT_CRITICAL(&sharedMux);
  String j=String("\"closePositive\":")+(c.closePositive?"true":"false")+",\"openZero\":"+String(c.openZero)+",\"openZeroValid\":"+(c.openZeroValid?"true":"false")+
    ",\"touchPosition\":"+String(c.touchPosition)+",\"touchPositionValid\":"+(c.touchPositionValid?"true":"false")+
    ",\"contactForceN\":"+String(c.contactForceN,2)+",\"graspForceN\":"+String(c.graspForceN,2)+",\"maxForceN\":"+String(c.maxForceN,2)+
    ",\"minimumActiveFsr\":"+String(c.minimumActiveFsr)+",\"contactConsecutive\":"+String(c.contactConsecutive)+",\"forceConsecutive\":"+String(c.forceConsecutive)+
    ",\"fsrSlipDropPct\":"+String(c.fsrSlipDropPct,1)+",\"slipConsecutive\":"+String(c.slipConsecutive)+
    ",\"contactStepCounts\":"+String(c.contactStepCounts)+",\"graspStepCounts\":"+String(c.graspStepCounts)+
    ",\"positionSpeed\":"+String(c.positionSpeed)+",\"graspSpeed\":"+String(c.graspSpeed)+",\"releaseSpeed\":"+String(c.releaseSpeed)+
    ",\"travelSafetyMarginCounts\":"+String(c.travelSafetyMarginCounts)+",\"positioningTimeoutMs\":"+String(c.positioningTimeoutMs)+
    ",\"graspTimeoutMs\":"+String(c.graspTimeoutMs)+",\"releaseTimeoutMs\":"+String(c.releaseTimeoutMs);
  json(true,j);
}
bool readFloatArg(const char* key,float current,float lo,float hi,float& out) {
  if(!server.hasArg(key)){out=current;return true;}String raw=server.arg(key);char* end=nullptr;float v=strtof(raw.c_str(),&end);
  if(end==raw.c_str()||*end!='\0'||!isfinite(v)||v<lo||v>hi)return false;out=v;return true;
}
bool readLongArg(const char* key,long current,long lo,long hi,long& out) {
  if(!server.hasArg(key)){out=current;return true;}String raw=server.arg(key);char* end=nullptr;long v=strtol(raw.c_str(),&end,10);
  if(end==raw.c_str()||*end!='\0'||v<lo||v>hi)return false;out=v;return true;
}
void saveConfigApi() {
  State st;bool moving;FsmConfig old;portENTER_CRITICAL(&sharedMux);st=state;old=fc;moving=motionActive;portEXIT_CRITICAL(&sharedMux);
  if(st!=IDLE||moving){json(false,"\"error\":\"settings can only be changed in IDLE with the servo stopped\"",409);return;}
  FsmConfig next=old;float contact,grasp,maximum,slipDrop;
  if(!readFloatArg("contactForceN",old.contactForceN,0.10f,2.00f,contact)||
     !readFloatArg("graspForceN",old.graspForceN,0.25f,5.00f,grasp)||
     !readFloatArg("maxForceN",old.maxForceN,1.00f,30.00f,maximum)||
     !readFloatArg("fsrSlipDropPct",old.fsrSlipDropPct,10.0f,80.0f,slipDrop)||
     !(contact<grasp&&grasp<maximum)) {
    json(false,"\"error\":\"force settings must satisfy contact < grasp < maximum; check all force ranges\"",400);return;
  }
  const char* keys[]={"minimumActiveFsr","contactConsecutive","forceConsecutive","contactStepCounts","graspStepCounts","positionSpeed","graspSpeed","releaseSpeed","travelSafetyMarginCounts","positioningTimeoutMs","graspTimeoutMs","releaseTimeoutMs","slipConsecutive"};
  const long mins[]={1,1,1,10,10,1,1,1,20,1000,1000,1000,1};
  const long maxs[]={6,10,10,250,100,61,61,61,500,60000,30000,30000,10};
  long current[]={old.minimumActiveFsr,old.contactConsecutive,old.forceConsecutive,old.contactStepCounts,old.graspStepCounts,
    old.positionSpeed,old.graspSpeed,old.releaseSpeed,old.travelSafetyMarginCounts,old.positioningTimeoutMs,old.graspTimeoutMs,old.releaseTimeoutMs,old.slipConsecutive};
  long vals[sizeof(keys)/sizeof(keys[0])];
  for(uint8_t i=0;i<sizeof(keys)/sizeof(keys[0]);i++)if(!readLongArg(keys[i],current[i],mins[i],maxs[i],vals[i])){
    json(false,String("\"error\":\"")+keys[i]+" is outside its allowed range\"",400);return;
  }
  next.contactForceN=contact;next.graspForceN=grasp;next.maxForceN=maximum;next.fsrSlipDropPct=slipDrop;
  next.minimumActiveFsr=(uint16_t)vals[0];next.contactConsecutive=(uint8_t)vals[1];next.forceConsecutive=(uint8_t)vals[2];
  next.contactStepCounts=(uint16_t)vals[3];next.graspStepCounts=(uint16_t)vals[4];
  next.positionSpeed=(uint16_t)vals[5];next.graspSpeed=(uint16_t)vals[6];next.releaseSpeed=(uint16_t)vals[7];
  next.travelSafetyMarginCounts=(uint16_t)vals[8];next.positioningTimeoutMs=(uint32_t)vals[9];next.graspTimeoutMs=(uint32_t)vals[10];next.releaseTimeoutMs=(uint32_t)vals[11];next.slipConsecutive=(uint8_t)vals[12];
  if(next.touchPositionValid&&2L*next.travelSafetyMarginCounts>=labs((long)next.touchPosition-next.openZero)){json(false,"\"error\":\"travel margin must be less than half of calibrated travel\"",400);return;}
  Preferences p;if(!p.begin("fsm",false)){json(false,"\"error\":\"could not open FSM preferences\"",500);return;}
  size_t n=p.putBytes("cfg",&next,sizeof(next));size_t v=p.putUChar("cfgv",FSM_CONFIG_VERSION);p.end();
  if(n!=sizeof(next)||v!=sizeof(uint8_t)){json(false,"\"error\":\"failed to save FSM settings\"",500);return;}
  portENTER_CRITICAL(&sharedMux);fc=next;portEXIT_CRITICAL(&sharedMux);json(true,"\"saved\":true");
}
void apiFsrConfig() {
  FsrConfig c[FSR_COUNT];portENTER_CRITICAL(&sharedMux);memcpy(c,fsrCfg,sizeof(c));portEXIT_CRITICAL(&sharedMux);
  String j="\"sensors\":[";
  for(uint8_t i=0;i<FSR_COUNT;i++){if(i)j+=',';j+="{\"channel\":"+String(FSR_CHANNEL_NUMBER[i])+",\"gpio\":"+String(FSR_PINS[i])+",\"r\":"+String(c[i].fixedOhms,1)+",\"z\":"+String(c[i].zeroMv,1)+",\"f1\":"+String(c[i].f1,4)+",\"o1\":"+String(c[i].o1,1)+",\"f2\":"+String(c[i].f2,4)+",\"o2\":"+String(c[i].o2,1)+"}";}
  j+="]";json(true,j);
}
bool saveOneFsr(uint8_t i,const FsrConfig& c) {
  Preferences p;if(!p.begin("fsr402",false))return false;String k="s"+String(i);
  bool ok=p.putFloat((k+"r").c_str(),c.fixedOhms)==sizeof(float)&&p.putFloat((k+"f1").c_str(),c.f1)==sizeof(float)&&p.putFloat((k+"o1").c_str(),c.o1)==sizeof(float)&&
          p.putFloat((k+"f2").c_str(),c.f2)==sizeof(float)&&p.putFloat((k+"o2").c_str(),c.o2)==sizeof(float)&&p.putFloat((k+"z").c_str(),c.zeroMv)==sizeof(float);
  p.end();return ok;
}
void apiFsrSave() {
  State st;bool motion;portENTER_CRITICAL(&sharedMux);st=state;motion=motionActive;portEXIT_CRITICAL(&sharedMux);
  if(st!=IDLE||motion){json(false,"\"error\":\"FSR calibration is only available while the controller is stopped in IDLE\"",409);return;}
  int i=argL("id",-1);if(i<0||i>=FSR_COUNT){json(false,"\"error\":\"invalid FSR channel\"",400);return;}
  FsrConfig c={argF("r"),argF("f1"),argF("o1"),argF("f2"),argF("o2"),argF("z")};
  if(!isfinite(c.fixedOhms)||!isfinite(c.zeroMv)||!isfinite(c.f1)||!isfinite(c.f2)||!isfinite(c.o1)||!isfinite(c.o2)||c.fixedOhms<=0||c.zeroMv<0||c.zeroMv>1000||c.f1<0.2f||c.f1>30||c.f2<0.2f||c.f2>30||c.o1<=0||c.o2<=0||c.o1>=OPEN_OHMS||c.o2>=OPEN_OHMS||c.o1==c.o2||c.f1==c.f2){json(false,"\"error\":\"invalid calibration values\"",400);return;}
  if(!saveOneFsr(i,c)){json(false,"\"error\":\"failed to save FSR calibration\"",500);return;}
  portENTER_CRITICAL(&sharedMux);fsrCfg[i]=c;fsrCalStored[i]=true;portEXIT_CRITICAL(&sharedMux);json(true,"\"saved\":"+String(i));
}
void apiFsrZero() {
  int i=argL("id",-1);
  if(i<0||i>=FSR_COUNT){json(false,"\"error\":\"invalid FSR channel\"",400);return;}
  State st;bool motion;FsrReading reading;FsrConfig c;
  portENTER_CRITICAL(&sharedMux);st=state;motion=motionActive;reading=fsr[i];c=fsrCfg[i];portEXIT_CRITICAL(&sharedMux);
  if(st!=IDLE||motion){json(false,"\"error\":\"FSR zero is only available while stopped in IDLE\"",409);return;}
  c.zeroMv+=reading.mv;
  if(!saveOneFsr((uint8_t)i,c)){json(false,"\"error\":\"failed to save FSR zero\"",500);return;}
  portENTER_CRITICAL(&sharedMux);fsrCfg[i]=c;portEXIT_CRITICAL(&sharedMux);
  json(true,"\"zeroMv\":"+String(c.zeroMv,1));
}

void handleApiData() {
  SensorCalibration cal[SENSOR_COUNT];SensorState ss[SENSOR_COUNT];uint32_t scanUs;
  portENTER_CRITICAL(&sharedMux);memcpy(cal,sensorCal,sizeof(cal));memcpy(ss,sensor,sizeof(ss));scanUs=lastScanUs;portEXIT_CRITICAL(&sharedMux);
  String out;out.reserve(5600);out="{\"calibration\":[";
  for(uint8_t i=0;i<SENSOR_COUNT;i++){
    if(i)out+=',';out+="{\"orientation\":"+String(cal[i].orientation)+",\"rangeScale\":"+String(cal[i].rangeScale,6)+",\"rangeOffset\":"+String(cal[i].rangeOffset,3)+
      ",\"yaw\":"+String(cal[i].yawCorrectionDeg,3)+",\"pitch\":"+String(cal[i].pitchCorrectionDeg,3)+",\"roll\":"+String(cal[i].rollCorrectionDeg,3)+",\"valid\":"+(cal[i].valid?"true":"false")+"}";
  }
  uint32_t frames=0;for(uint8_t i=0;i<SENSOR_COUNT;i++)frames+=ss[i].okCount;
  out+="],\"frames\":"+String(frames)+",\"scan_us\":"+String(scanUs)+",\"sensors\":[";
  for(uint8_t i=0;i<SENSOR_COUNT;i++){
    if(i)out+=',';out+="{\"address\":"+String(SENSOR_ADDR[i])+
      ",\"found\":"+(ss[i].found?"true":"false")+",\"initialized\":"+(ss[i].initialized?"true":"false")+",\"configured\":"+(ss[i].configured?"true":"false")+
      ",\"has_data\":"+(ss[i].hasData?"true":"false")+",\"last_ok\":"+(ss[i].lastReadOK?"true":"false")+
      ",\"frame_seq\":"+String(ss[i].frameSeq)+",\"age_ms\":"+(ss[i].hasData?String((uint32_t)(millis()-ss[i].lastGoodMs)):String("null"))+
      ",\"read_us\":"+String(ss[i].readUs)+",\"ok\":"+String(ss[i].okCount)+",\"err\":"+String(ss[i].errorCount)+",\"values\":";
    if(!ss[i].hasData)out+="null";else{out+='[';for(uint8_t k=0;k<64;k++){if(k)out+=',';out+=String(ss[i].mm[k]);}out+=']';}out+='}';
  }
  out+="]}";server.sendHeader("Cache-Control","no-store");server.send(200,"application/json",out);
}
void handleSaveSensorCalibration() {
  SensorCalibration candidate[SENSOR_COUNT];
  for(uint8_t i=0;i<SENSOR_COUNT;i++){
    String k=String(i);long orientation=argL(("o"+k).c_str(),-1);float scale=argF(("s"+k).c_str(),-1),offset=argF(("b"+k).c_str(),10000);
    float yaw=argF(("y"+k).c_str(),1000),pitch=argF(("p"+k).c_str(),1000),roll=argF(("r"+k).c_str(),1000);bool valid=argL(("v"+k).c_str(),0)!=0;
    if(orientation<0||orientation>7||!isfinite(scale)||scale<0.8f||scale>1.2f||!isfinite(offset)||fabsf(offset)>40||!isfinite(yaw)||fabsf(yaw)>8||!isfinite(pitch)||fabsf(pitch)>8||!isfinite(roll)||fabsf(roll)>8){json(false,"\"error\":\"calibration values outside allowed limits\"",400);return;}
    candidate[i]={(uint8_t)orientation,scale,offset,yaw,pitch,roll,valid};
  }
  Preferences p;if(!p.begin("senCal2",false)){json(false,"\"error\":\"cannot open SEN0628 calibration preferences\"",500);return;}
  bool ok=p.putUChar("ver",SENSOR_CAL_VERSION)==sizeof(uint8_t);
  for(uint8_t i=0;i<SENSOR_COUNT;i++){String k=String(i);ok&=p.putUChar(("o"+k).c_str(),candidate[i].orientation)==sizeof(uint8_t);ok&=p.putFloat(("s"+k).c_str(),candidate[i].rangeScale)==sizeof(float);ok&=p.putFloat(("b"+k).c_str(),candidate[i].rangeOffset)==sizeof(float);ok&=p.putFloat(("y"+k).c_str(),candidate[i].yawCorrectionDeg)==sizeof(float);ok&=p.putFloat(("p"+k).c_str(),candidate[i].pitchCorrectionDeg)==sizeof(float);ok&=p.putFloat(("r"+k).c_str(),candidate[i].rollCorrectionDeg)==sizeof(float);ok&=p.putBool(("v"+k).c_str(),candidate[i].valid)==sizeof(uint8_t);}
  p.end();if(!ok){json(false,"\"error\":\"failed to save SEN0628 calibration\"",500);return;}
  portENTER_CRITICAL(&sharedMux);memcpy(sensorCal,candidate,sizeof(sensorCal));portEXIT_CRITICAL(&sharedMux);json(true,"\"saved\":true,\"calibrationVersion\":"+String(SENSOR_CAL_VERSION));
}

// ---------------- Main web page: simple setup + FSM ----------------
const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>SENSE-GRIP Control</title>
<style>
:root{color-scheme:dark;--bg:#08111d;--card:#122036;--line:#2a405d;--text:#e8eef8;--muted:#98abc4;--blue:#45b9ff;--green:#36cf8c;--red:#ef6476;--amber:#efbf58}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px system-ui,Arial}.top{position:sticky;top:0;z-index:5;background:#0c1728f4;border-bottom:1px solid var(--line);padding:12px 16px;display:flex;gap:10px;align-items:center;flex-wrap:wrap}.brand{font-size:19px;font-weight:800}.pill{padding:6px 10px;border:1px solid var(--line);border-radius:20px;color:var(--muted)}.layout{display:grid;grid-template-columns:210px minmax(0,1fr);min-height:calc(100vh - 56px)}nav{padding:12px;background:#0c1728;border-right:1px solid var(--line)}nav button{display:block;width:100%;text-align:left;background:transparent;color:var(--text);border:0;border-radius:8px;padding:11px;cursor:pointer;margin-bottom:4px}nav button.sel,nav button:hover{background:#1a2c48;color:var(--blue)}main{padding:18px;max-width:1400px;width:100%}.page{display:none}.page.on{display:block}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:12px}.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:15px;min-width:0}.card h2,.card h3{margin:0 0 10px}.muted,.small{color:var(--muted);font-size:12px}.metric{display:flex;justify-content:space-between;gap:10px;padding:8px;background:#0b1626;border-radius:7px;margin:6px 0}.value{font-size:28px;font-weight:800;color:var(--blue)}.btn{border:0;border-radius:8px;padding:10px 14px;margin:3px;background:#246fd1;color:white;font-weight:650;cursor:pointer}.btn.green{background:#178b5a}.btn.red{background:#bb3d54}.btn.amber{background:#9b6a12}.btn.secondary{background:#34465e}.btn:disabled{opacity:.45;cursor:not-allowed}input,select{background:#091321;color:var(--text);border:1px solid #3b5372;border-radius:7px;padding:8px;width:100%;max-width:240px}label{display:block;margin:9px 0;color:var(--muted)}.formgrid{display:block}.setting-table{display:flex;flex-direction:column;border:1px solid var(--line);border-radius:9px;overflow:hidden}.setting-head,.setting-row{display:grid;grid-template-columns:minmax(175px,1.05fr) minmax(260px,2fr) minmax(120px,.8fr) 125px;gap:12px;align-items:center;padding:10px 12px}.setting-head{background:#0b1626;color:var(--muted);font-size:11px;font-weight:800;letter-spacing:.06em}.setting-row{border-top:1px solid var(--line);min-height:56px}.setting-row:nth-child(odd){background:#0e1b2d}.setting-name{font-weight:700}.setting-desc{color:var(--muted);font-size:12px;line-height:1.35}.setting-range{color:var(--muted);font-size:12px;font-variant-numeric:tabular-nums}.setting-row input{max-width:120px;margin:0}.cfg-actions{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin-top:12px}@media(max-width:900px){.setting-head{display:none}.setting-row{grid-template-columns:minmax(0,1fr) minmax(110px,130px);gap:5px 12px}.setting-name{grid-column:1}.setting-desc{grid-column:1;grid-row:2}.setting-range{grid-column:1;grid-row:3}.setting-row>div:last-child{grid-column:2;grid-row:1/4;justify-self:end}.setting-row input{max-width:112px}}.statebar{display:flex;gap:7px;flex-wrap:wrap;margin:10px 0}.state{border:1px solid var(--line);border-radius:8px;padding:9px 12px;color:var(--muted);background:#0b1626}.state.active{border-color:var(--green);color:var(--green);background:#0d3529}.good{color:var(--green)}.bad{color:var(--red)}.warn{color:var(--amber)}.log{white-space:pre-wrap;word-break:break-word;background:#07101b;border-radius:8px;padding:10px;min-height:44px}.fsrgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:9px}.fsr{background:#0b1626;border:1px solid var(--line);border-radius:8px;padding:10px}.fingerrow{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}.finger{padding:12px;border:1px solid var(--line);border-radius:9px;background:#0b1626;text-align:center}.finger.on{border-color:var(--green);background:#0d3529;color:var(--green)}.finger.off{color:var(--muted)}.dangerBox{padding:12px;border:1px solid #8c3b50;background:#351622;border-radius:9px;margin:10px 0}.note{line-height:1.5;color:var(--muted);font-size:12px}.divider{border:0;border-top:1px solid var(--line);margin:14px 0}@media(max-width:760px){.layout{grid-template-columns:1fr}nav{display:flex;overflow:auto;gap:4px}nav button{white-space:nowrap;width:auto;min-width:120px}main{padding:10px}.fingerrow{grid-template-columns:1fr}}
</style></head><body>
<header class="top"><span class="brand">SENSE-GRIP</span><span id="conn" class="pill bad">CONNECTING</span><span class="pill">State: <b id="topState">--</b></span><span class="pill">Encoder: <b id="topPos">--</b></span><span class="pill">Mode: <b id="topMode">--</b></span><button class="btn red" onclick="cmd('estop')">EMERGENCY STOP</button></header>
<div class="layout"><nav id="nav"></nav><main>
<section id="dash" class="page on"><h1>System overview</h1><div id="warning"></div><p class="note">Finger wiring: F1 = FSR6/GPIO10 + FSR2/GPIO1 · F2 = FSR4/GPIO5 + FSR3/GPIO4 · F3 = FSR5/GPIO6 + FSR1/GPIO2.</p><div class="grid"><div class="card"><h2>CF35-12 actuator</h2><div class="value" id="stateMain">--</div><div class="metric"><span>Encoder position</span><b id="pos">--</b></div><div class="metric"><span>Motor speed</span><b id="speed">--</b></div><div class="metric"><span>Current</span><b id="current">--</b></div><div class="metric"><span>Voltage</span><b id="voltage">--</b></div><div class="metric"><span>Temperature</span><b id="temperature">--</b></div></div><div class="card"><h2>Travel references</h2><div class="metric"><span>OPEN zero</span><b id="openZero">not set</b></div><div class="metric"><span>3-finger TOUCH</span><b id="touchPos">not set</b></div><div class="metric"><span>Available travel</span><b id="travel">--</b></div><button class="btn green" onclick="showPage('servo')">Servo setup</button></div></div><h2>Three-finger contact</h2><div id="fingerRow" class="fingerrow"></div><h2>FSR channels</h2><div id="fsrCards" class="fsrgrid"></div><h2>SEN0628 sensors</h2><div id="senCards" class="grid"></div></section>
<section id="servo" class="page"><h1>Servo setup</h1><p class="note">The ESP32 controls the CF35-12 in position mode 0. There is no absolute-position input. Select a finite increment; the ESP32 creates the relative position command.</p><div class="grid"><div class="card"><h2>Live servo</h2><div class="value" id="servoPos">--</div><div class="metric"><span>Target</span><b id="servoTarget">--</b></div><div class="metric"><span>Mode</span><b id="servoMode">--</b></div><div class="metric"><span>Motion</span><b id="servoMoving">--</b></div><button class="btn secondary" onclick="cmd('moveOpen')">Move to saved OPEN zero</button></div><div class="card"><h2>Controller movement</h2><label for="servoStep">Movement increment (encoder counts)<select id="servoStep" onchange="updateStepLabels()"><option value="20" selected>20</option><option value="50">50</option><option value="100">100</option><option value="250">250</option><option value="500">500</option><option value="1000">1000</option></select></label><p class="note">Each press makes one relative move. Large increments (250–1000) are for coarse positioning only. Keep fingers and the mechanism clear. Use 20 or 50 counts when approaching contact. Closing stops at three-finger contact or the configured maximum FSR force. A saved TOUCH position also limits closing travel.</p><button class="btn" id="stepOpenBtn" onclick="cmdStep('stepOpen')">Step OPEN −20 counts</button><button class="btn green" id="stepCloseBtn" onclick="cmdStep('stepClose')">Step CLOSE +20 counts</button><hr class="divider"><button class="btn green" onclick="cmd('setOpenZero')">SET current position as OPEN zero</button><button class="btn amber" id="setTouchBtn" onclick="cmd('setTouch')">SAVE three-finger TOUCH position</button><p class="note">Finger 1: FSR6/GPIO10 + FSR2/GPIO1. Finger 2: FSR4/GPIO5 + FSR3/GPIO4. Finger 3: FSR5/GPIO6 + FSR1/GPIO2. TOUCH is saved only when all three finger groups indicate contact.</p></div></div><div class="card"><h2>Calibration status</h2><div class="metric"><span>OPEN zero</span><b id="servoOpenZero">not set</b></div><div class="metric"><span>Three-finger TOUCH position</span><b id="servoTouch">not set</b></div><div id="servoMessage" class="log">Waiting for controller.</div></div></section>
<section id="fsm" class="page"><h1>Grasp state machine</h1><p class="note">Simple sequence: POSITIONING → GRASPING → HOLDING → RELEASING. SEN0628 acquisition is kept separate and is not used as an FSM trigger in this release.</p><div id="stateBar" class="statebar"></div><div class="grid"><div class="card"><h2>Cycle control</h2><button class="btn green" onclick="cmd('start')">START GRASP</button><button class="btn" onclick="cmd('release')">RELEASE</button><button class="btn amber" onclick="cmd('pause')">PAUSE</button><button class="btn secondary" onclick="cmd('reset')">RESET / ACKNOWLEDGE</button><button class="btn red" onclick="cmd('estop')">EMERGENCY STOP</button><div class="metric"><span>State time</span><b id="stateTime">--</b></div><div class="metric"><span>Force-loss warning</span><b id="forceLoss">--</b></div></div><div class="card"><h2>Current cycle</h2><div class="metric"><span>Average active FSR force</span><b id="avgForce">--</b></div><div class="metric"><span>Maximum channel force</span><b id="maxForce">--</b></div><div class="metric"><span>Active fingers</span><b id="activeFingers">--</b></div><div class="metric"><span>FSR calibration</span><b id="fsrCal">--</b></div><div class="metric"><span>Travel calibration</span><b id="travelCal">--</b></div></div></div><div class="card" style="margin-top:12px"><h2>Simple FSM settings</h2><p class="note">Settings apply to the automatic grasp cycle. Change them only when the controller is IDLE and the servo is stopped. Force values are based on your saved FSR calibration. Begin with the defaults below and change one parameter at a time.</p><form id="cfg" class="formgrid"></form><div class="cfg-actions"><button class="btn green" onclick="saveCfg()">Save settings</button><div id="cfgMsg" class="note">Changes are stored in ESP32 NVS.</div></div></div></section>
<section id="fsrPage" class="page"><h1>FSR calibration</h1><p class="note">Six independent FSR402 channels, calibrated over 0–30 N. Keep the sensor unloaded when setting zero. Use two non-zero known force points (0.2–30 N) for each channel.</p><div id="fsrCfg" class="grid"></div></section>
<section id="diag" class="page"><h1>Diagnostics</h1><div class="card"><div class="metric"><span>Web task core</span><b>0</b></div><div class="metric"><span>Sensor / motor controller core</span><b>1</b></div><div class="metric"><span>Three-sensor scan time</span><b id="scanTime">--</b></div><div id="lastAction" class="log">--</div></div><div class="card"><h2>CF35-12 read-only register survey</h2><p class="note">Read status only when requested. This reads the registers once. It does not write servo settings or command motor movement. Values are raw decimal register bytes.</p><button class="btn" id="cfDiagRead" type="button" onclick="readCf35Status()">Read status</button><div id="cfDiagMessage" class="note">No new diagnostic read requested.</div><div class="metric"><span>Capture age</span><b id="cfDiagAge">--</b></div><div class="metric"><span>Registers 0–4</span><b id="cfDiagInfo">--</b></div><div class="metric"><span>Registers 9–12</span><b id="cfDiagLimits">--</b></div><div class="metric"><span>Register 18 (phase)</span><b id="cfDiagPhase">--</b></div><div class="metric"><span>Register 33 (mode)</span><b id="cfDiagMode">--</b></div><div class="metric"><span>Registers 56–73</span><b id="cfDiagFeedback">--</b></div><p class="note">Register 65 is byte 9 of the feedback block. Registers 67–68 are bytes 11–12. Save the raw capture with the test notes.</p></div><div class="grid" id="diagCards"></div><p class="note">Wi-Fi and HTTP handling run separately from the core-1 controller loop. HTTP handlers only read snapshots or queue commands. They do not access I²C, ADC or motor UART.</p></section>
</main></div>
<script>
'use strict';const $=id=>document.getElementById(id);
const pages=[['dash','Dashboard'],['servo','Servo setup'],['fsm','FSM operation'],['fsrPage','FSR calibration'],['diag','Diagnostics']];
$('nav').innerHTML=pages.map(([id,n],i)=>`<button class="${i===0?'sel':''}" onclick="showPage('${id}',this)">${n}</button>`).join('')+`<button onclick="window.open('/sen0628','_blank')">SEN0628 3D view</button>`;
function showPage(id,b){document.querySelectorAll('.page').forEach(x=>x.classList.remove('on'));$(id).classList.add('on');document.querySelectorAll('nav button').forEach(x=>x.classList.remove('sel'));if(b)b.classList.add('sel');}
function req(url,opt={}){return fetch(url,{cache:'no-store',...opt}).then(async r=>{let j=await r.json();if(!r.ok||!j.ok)throw Error(j.error||'request failed');return j;});}
function post(url,body=''){return req(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});}
async function cmd(name){try{await post('/api/control/'+name);$('servoMessage').textContent='Command queued: '+name;}catch(e){$('servoMessage').textContent=e.message;}}
function updateStepLabels(){const n=Number($('servoStep').value);$('stepOpenBtn').textContent='Step OPEN −'+n+' counts';$('stepCloseBtn').textContent='Step CLOSE +'+n+' counts';}
async function cmdStep(name){const step=Number($('servoStep').value);try{await post('/api/control/'+name,'step='+encodeURIComponent(step));$('servoMessage').textContent='Command queued: '+name+' · '+step+' counts';}catch(e){$('servoMessage').textContent=e.message;}}
const stateNames=['IDLE','POSITIONING','GRASPING','HOLDING','RELEASING','PAUSED','FAULT'];
function renderState(s){$('stateBar').innerHTML=stateNames.map(n=>`<span class="state ${s.state===n?'active':''}">${n}</span>`).join('');}
function renderStatus(s){window.lastStatus=s;updateStepLabels();$('conn').textContent='ONLINE';$('conn').className='pill good';$('topState').textContent=s.state;$('topPos').textContent=s.motor.position;$('topMode').textContent=s.motorModeReady?'POSITION 0':'NOT READY';$('stateMain').textContent=s.state;$('pos').textContent=s.motor.position;$('speed').textContent=s.motor.speed;$('current').textContent=s.motor.mA.toFixed(0)+' mA';$('voltage').textContent=s.motor.voltage.toFixed(1)+' V';$('temperature').textContent=s.motor.temp+' °C';$('servoPos').textContent=s.motor.position;$('servoTarget').textContent=s.motor.targetPosition;$('servoMode').textContent=s.motorModeReady?'0 · verified':'unverified';$('servoMoving').textContent=(s.motionActive||s.motor.moving||s.motor.targetActive)?'MOVING':'STOPPED';$('openZero').textContent=s.openZeroValid?s.openZero:'not set';$('touchPos').textContent=s.touchPositionValid?s.touchPosition:'not set';$('travel').textContent=s.travelValid?s.travelCounts+' counts':'not calibrated';$('servoOpenZero').textContent=s.openZeroValid?s.openZero:'not set';$('servoTouch').textContent=s.touchPositionValid?s.touchPosition:'not set';$('setTouchBtn').disabled=!s.allThreeTouching;$('stepCloseBtn').disabled=s.allThreeTouching||!s.fsrCalibrationOk;$('servoMessage').textContent=s.lastAction;$('lastAction').textContent=s.lastAction;$('stateTime').textContent=s.stateMs+' ms';$('forceLoss').textContent=s.forceLossWarning?'FORCE LOSS':'--';$('avgForce').textContent=s.avgForce.toFixed(2)+' N';$('maxForce').textContent=s.maxFsrForce.toFixed(2)+' N';$('activeFingers').textContent=s.activeFingers+'/3';$('fsrCal').textContent=s.fsrCalibrationOk?'VALID':'INCOMPLETE';$('travelCal').textContent=s.travelValid?'VALID':'NOT SET';$('scanTime').textContent=window.lastData?.scan_us?((window.lastData.scan_us/1000).toFixed(1)+' ms'):'--';renderState(s);
$('fingerRow').innerHTML=s.finger.map((f,i)=>`<div class="finger ${f.touching?'on':'off'}"><b>FINGER ${i+1}</b><div>${f.touching?'TOUCH':'NO CONTACT'}</div><small>FSR${f.fsrA}/GPIO${f.gpioA} + FSR${f.fsrB}/GPIO${f.gpioB}</small><br><small>${f.forceN.toFixed(2)} N</small></div>`).join('');
$('fsrCards').innerHTML=s.fsr.map((f,i)=>`<div class="fsr"><b>FSR ${f.channel} · GPIO${f.gpio}</b><div class="value">${f.force===null?'--':f.force.toFixed(2)+' N'}</div><div class="small">${f.raw} raw · ${f.ohms.toFixed(0)} Ω · ${f.calibrated?'calibrated':'not calibrated'}</div></div>`).join('');
$('senCards').innerHTML=s.sen.map((v,i)=>`<div class="card"><b>SEN0628 ${i+1}</b><div class="${v.online?'good':'bad'}">${v.online?'ONLINE':'NO RECENT FRAME'}</div><small class="muted">${v.ok} successful · ${v.err} errors</small></div>`).join('');
$('diagCards').innerHTML=$('senCards').innerHTML;$('warning').innerHTML=!s.motorModeReady?'<div class="dangerBox">CF35-12 must report position mode 0. Motion is locked.</div>':!s.fsrCalibrationOk?'<div class="dangerBox">Calibrate all six FSR channels before grasping.</div>':!s.travelValid?'<div class="dangerBox">Set OPEN zero and three-finger TOUCH before starting the FSM.</div>':s.state==='FAULT'?`<div class="dangerBox">${s.reason} Use RESET only after checking the mechanism.</div>`:'';}
function renderCf35Diagnostics(d){const fmt=(ok,v)=>ok?v.join(', '):'READ FAILED';$('cfDiagAge').textContent=d.ageMs+' ms';$('cfDiagInfo').textContent=fmt(d.infoOk,d.info);$('cfDiagLimits').textContent=fmt(d.limitsOk,d.limits);$('cfDiagPhase').textContent=d.phaseOk?String(d.phase):'READ FAILED';$('cfDiagMode').textContent=d.modeOk?String(d.mode):'READ FAILED';$('cfDiagFeedback').textContent=fmt(d.feedbackOk,d.feedback);}
async function readCf35Status(){const b=$('cfDiagRead');b.disabled=true;$('cfDiagMessage').textContent='Reading CF35-12 registers…';try{await post('/api/cf35/read');await new Promise(resolve=>setTimeout(resolve,500));const d=await req('/api/cf35/diagnostics');renderCf35Diagnostics(d);$('cfDiagMessage').textContent='Read complete. Capture age: '+d.ageMs+' ms.';}catch(e){$('cfDiagMessage').textContent='Read failed: '+e.message;}finally{b.disabled=false;}}
async function poll(){try{renderStatus(await req('/api/status'));}catch(e){$('conn').textContent='OFFLINE';$('conn').className='pill bad';}setTimeout(poll,1000);}
const cfgFields=[
 ['contactForceN','Contact threshold','Either FSR in each finger group can reach this force to register that finger as touching.','0.10–2.00 N',0.10,2.00,0.05],
 ['contactConsecutive','Contact debounce','Consecutive FSR samples required before a finger is marked as touching. 3 samples are about 60 ms.','1–10 samples',1,10,1],
 ['minimumActiveFsr','Minimum active FSR channels','Minimum number of individual FSR channels above 0.05 N before the grasp-force target can be accepted.','1–6 channels',1,6,1],
 ['graspForceN','Grasp force target','Legacy setting for a future force-based grasp phase; this branch enters HOLDING at the TOUCH position window.','0.25–5.00 N',0.25,5.00,0.05],
 ['forceConsecutive','Grasp stability samples','Legacy setting for a future force-based grasp phase; not used by the position-window stop.','1–10 samples',1,10,1],
 ['maxForceN','Maximum force per FSR','Hard software ceiling for any single FSR channel. Reaching it stops the FSM with a fault.','1.00–30.00 N',1.00,30.00,0.10],
 ['contactStepCounts','Legacy contact step size','Not used by this branch. The FSM now sends one continuous position command.','10–250 counts',10,250,5],
 ['graspStepCounts','Legacy grasp step size','Not used by this branch. The FSM now stops at the saved TOUCH position window.','10–100 counts',10,100,5],
 ['positionSpeed','Continuous approach speed','CF35-12 speed command for the single move to saved TOUCH. 61 units is approximately 44.7 RPM.','1–61 units',1,61,1],
 ['graspSpeed','Position-hold speed','CF35-12 speed command used to hold the measured position after approach.','1–61 units',1,61,1],
 ['releaseSpeed','Release speed','CF35-12 speed command used to return to the saved OPEN position. Default 61 uses the maximum documented speed.','1–61 units',1,61,1],
 ['travelSafetyMarginCounts','TOUCH safety margin','Stops automatic closing this many encoder counts before the saved TOUCH reference.','20–500 counts',20,500,5],
 ['positioningTimeoutMs','Continuous approach timeout','Maximum time for the single move to enter the final 5% of the saved TOUCH travel.','1–60 seconds',1000,60000,1000],
 ['graspTimeoutMs','Grasp timeout','Maximum time allowed to reach the grasp-force target after contact.','1–30 seconds',1000,30000,1000],
 ['releaseTimeoutMs','Release timeout','Maximum time allowed to return to the saved OPEN zero.','1–30 seconds',1000,30000,1000],
 ['fsrSlipDropPct','Force-loss warning threshold','Percentage drop from the force recorded on entry to HOLDING that starts a force-loss warning. It does not tighten the grip.','10–80 %',10,80,1],
 ['slipConsecutive','Force-loss warning samples','Consecutive FSR samples below the force-loss threshold required to show the warning.','1–10 samples',1,10,1]
];
async function loadCfg(){let c=await req('/api/config');$('cfg').innerHTML='<div class="setting-table"><div class="setting-head"><span>SETTING</span><span>WHAT IT DOES</span><span>ALLOWED RANGE</span><span>VALUE</span></div>'+cfgFields.map(([k,l,d,r,min,max,step])=>`<div class="setting-row"><div class="setting-name">${l}</div><div class="setting-desc">${d}</div><div class="setting-range">${r}</div><div><input name="${k}" aria-label="${l}" type="number" min="${min}" max="${max}" step="${step}" value="${c[k]}"></div></div>`).join('')+'</div>';}
async function saveCfg(){try{let p=new URLSearchParams(new FormData($('cfg')));await post('/api/config',p);$('cfgMsg').textContent='Settings saved to ESP32 NVS.';await loadCfg();}catch(e){$('cfgMsg').textContent='Save failed: '+e.message;}}
async function initFsr(){try{let c=await req('/api/fsr/config');$('fsrCfg').innerHTML=c.sensors.map((s,i)=>`<div class="card"><h3>FSR ${s.channel} · GPIO${s.gpio}</h3><label>Fixed resistor (Ω)<input id="r${i}" type="number" value="${s.r}"></label><label>Zero offset (mV)<input id="z${i}" type="number" step="0.1" value="${s.z}"></label><button class="btn secondary" onclick="zeroFsr(${i})">Set current as zero</button><hr class="divider"><label>Point 1 force (N)<input id="f1${i}" type="number" step="0.01" value="${s.f1}"></label><label>Point 1 resistance (Ω)<input id="o1${i}" type="number" value="${s.o1}"></label><button class="btn secondary" onclick="captureOhm(${i},1)">Use live resistance</button><label>Point 2 force (N)<input id="f2${i}" type="number" step="0.01" value="${s.f2}"></label><label>Point 2 resistance (Ω)<input id="o2${i}" type="number" value="${s.o2}"></label><button class="btn secondary" onclick="captureOhm(${i},2)">Use live resistance</button><br><button class="btn green" onclick="saveFsr(${i})">Save FSR ${s.channel}</button></div>`).join('');}catch(e){$('fsrCfg').textContent=e.message;}}
function captureOhm(i,n){let v=window.lastStatus?.fsr?.[i];if(v&&Number.isFinite(v.ohms))$('o'+n+i).value=v.ohms.toFixed(1);}
async function zeroFsr(i){try{await post('/api/fsr/zero','id='+i);await initFsr();}catch(e){alert(e.message);}}
async function saveFsr(i){try{let p=new URLSearchParams({id:i,r:$('r'+i).value,z:$('z'+i).value,f1:$('f1'+i).value,o1:$('o1'+i).value,f2:$('f2'+i).value,o2:$('o2'+i).value});await post('/api/fsr/save',p);await initFsr();}catch(e){alert(e.message);}}
loadCfg().catch(e=>$('cfgMsg').textContent=e.message);initFsr();poll();
</script></body></html>)HTML";

const char SEN0628_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SEN0628 Calibration and 3D Workspace</title>
<style>
:root{color-scheme:dark;font-family:system-ui,Arial,sans-serif;background:#08111d;color:#e8eef8}
*{box-sizing:border-box}body{margin:0;padding:14px;max-width:1550px;margin-inline:auto}
header{display:flex;justify-content:space-between;align-items:center;gap:12px;flex-wrap:wrap;margin-bottom:12px}
h1{font-size:1.2rem;margin:0}.muted{color:#91a4bd;font-size:.8rem}.pill{border:1px solid #34506d;border-radius:20px;padding:6px 10px;font-size:.76rem}
.layout{display:grid;grid-template-columns:minmax(0,1fr) 330px;gap:12px}.panel{background:#111e2f;border:1px solid #263b55;border-radius:13px;padding:12px;min-width:0}.panel h2{font-size:.95rem;margin:0 0 8px}
.viewer{position:relative;background:radial-gradient(ellipse at 50% 42%,#14253b,#080f1a 75%);border-radius:9px;overflow:hidden}
canvas{width:100%;height:560px;display:block;touch-action:none;cursor:grab}canvas:active{cursor:grabbing}
.controls{display:flex;flex-wrap:wrap;align-items:center;gap:10px;margin-top:10px;font-size:.78rem;color:#aabbd0}
button,select{background:#1a2b40;color:#e8eef8;border:1px solid #3a526f;border-radius:7px;padding:7px 10px;font:inherit;cursor:pointer}
button:disabled{opacity:.45;cursor:not-allowed}input[type=number]{background:#0a1222;border:1px solid #3a4b6d;color:#e8eef8;border-radius:7px;padding:7px;width:90px}
input[type=range]{width:130px;vertical-align:middle}label{display:inline-flex;align-items:center;gap:5px}
.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:9px}.stat{background:#0a1524;border-radius:7px;padding:8px}.stat small{display:block;color:#91a4bd;font-size:.68rem}.stat b{font-size:.95rem;font-variant-numeric:tabular-nums}
.sensors{display:grid;gap:8px}.sensor{background:#0a1524;border-radius:8px;padding:9px;border-left:3px solid var(--c)}.sensorhead{display:flex;justify-content:space-between;gap:8px;font-weight:650;font-size:.82rem}.sensorinfo{color:#9aadc4;font-size:.7rem;margin:4px 0 7px}.matrix{display:grid;grid-template-columns:repeat(8,1fr);gap:2px}.cell{aspect-ratio:1;min-width:0;background:#34445b;border-radius:2px;display:flex;align-items:center;justify-content:center;font-size:clamp(.38rem,.65vw,.58rem);color:#06101c;font-weight:700;overflow:hidden}
.legend{display:flex;flex-wrap:wrap;gap:9px;margin-top:9px;font-size:.7rem;color:#aabbd0}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:4px}.note{margin-top:9px;color:#91a4bd;font-size:.73rem;line-height:1.45}.wide{grid-column:1/-1}
.formgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:8px}.field{display:flex;flex-direction:column;gap:4px;color:#aabbd0;font-size:.72rem}.field input,.field select{width:100%;max-width:none}
.actions{display:flex;flex-wrap:wrap;gap:7px;margin-top:9px}.result{white-space:pre-wrap;background:#07101c;padding:9px;border-radius:8px;min-height:44px;color:#b9cce2;font:12px ui-monospace,monospace;overflow:auto}
.calgrid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}.calcard{background:#0a1524;border-radius:8px;padding:9px}.calcard h3{font-size:.82rem;margin:0 0 7px}.calcard .field{margin:5px 0}
.capturePanel{margin-top:12px;padding:10px;background:#0a1524;border:1px solid #2b405b;border-radius:9px}.capturePanel h3{font-size:.84rem;margin:0 0 7px}.captureList{display:grid;gap:7px}.captureRow{display:grid;grid-template-columns:minmax(0,1fr) auto auto;align-items:center;gap:7px;padding:8px;background:#101e30;border-radius:7px}.captureMeta{font-size:.72rem;color:#b5c6da;line-height:1.45}.captureMeta strong{color:#edf4ff}.captureRow button{font-size:.7rem;padding:6px 8px}.captureEditor{margin-top:10px;padding:10px;background:#101e30;border:1px solid #385372;border-radius:8px}.captureEditor[hidden]{display:none}.captureEditor h4{margin:0 0 8px;font-size:.8rem}.captureMatrix{display:grid;grid-template-columns:repeat(8,minmax(0,1fr));gap:3px;margin-top:9px;max-width:540px}.sampleTile{aspect-ratio:1;min-width:0;padding:1px;border:1px solid #536b87;border-radius:4px;background:#26384e;color:#e8eef8;font-size:clamp(.48rem,.8vw,.72rem);display:flex;align-items:center;justify-content:center;flex-direction:column;line-height:1.15}.sampleTile small{font-size:.55em;opacity:.75}.sampleTile.isIncluded{background:#22634f;border-color:#49bb8e}.sampleTile.isExcluded{background:#763f45;border-color:#e17a83;text-decoration:line-through}.sampleTile.empty{background:#111e2f;border-color:#26384d;color:#53647a;cursor:default}.editHelp{font-size:.7rem;color:#9aadc4;line-height:1.4;margin:7px 0}.danger{border-color:#8c4c55;color:#ffc5c9}.presetInfo{grid-column:1/-1;padding:8px;background:#0a1524;border-radius:7px;font-size:.72rem;color:#b7c8dc;line-height:1.5}
.tubeInputs{margin:10px 0}.tubeInputs .field{min-width:110px}.tubeMasks{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:9px;margin-top:10px}.tubeMaskCard{min-width:0;padding:9px;background:#0a1524;border:1px solid #263b55;border-radius:9px}.tubeMaskHead{display:flex;align-items:center;justify-content:space-between;gap:6px;font-size:.78rem;font-weight:650}.tubeMaskMeta{font-size:.68rem;color:#9aadc4;margin:5px 0}.tubeMaskActions{display:flex;gap:5px;margin:5px 0}.tubeMaskActions button{font-size:.66rem;padding:4px 7px}.maskGrid{display:grid;grid-template-columns:repeat(8,minmax(0,1fr));gap:3px}.maskCell{min-width:0;aspect-ratio:1;padding:1px;font-size:clamp(.43rem,.7vw,.68rem);border-radius:3px;line-height:1.1;overflow:hidden}.maskCell.on{background:#22634f;border-color:#49bb8e;color:#eafff6}.maskCell.off{background:#763f45;border-color:#e17a83;color:#ffe7e9;text-decoration:line-through}.plotPanel{margin-top:12px;padding:10px;background:#0a1524;border:1px solid #2b405b;border-radius:9px}.plotPanel h3{font-size:.84rem;margin:0 0 7px}#tubePlot{width:100%;height:320px;display:block;background:#07101c;border-radius:8px;cursor:default;touch-action:auto}.goodButton{background:#168c57;border-color:#249d69}
@media(max-width:900px){.tubeMasks{grid-template-columns:1fr}.tubeMaskCard{max-width:600px}}
@media(max-width:1000px){.layout{grid-template-columns:1fr}canvas{height:460px}.calgrid{grid-template-columns:1fr}}
@media(max-width:600px){body{padding:8px}canvas{height:420px}.stats{grid-template-columns:repeat(2,1fr)}}
</style></head>
<body>
<header><div><h1>Three SEN0628 — Calibration and 3D Workspace</h1><div class="muted">Fixed mechanical pose · per-sensor calibration · palm coordinate frame · millimetres</div></div><div id="sys" class="pill">CONNECTING</div></header>
<div class="layout">
<section class="panel">
<h2>Combined 3D point cloud</h2>
<div class="viewer"><canvas id="view"></canvas></div>
<div class="controls">
<label>Max range <input id="range" type="range" min="60" max="500" step="10" value="240"><b id="rangeV">240 mm</b></label>
<label>Display <select id="display"><option value="raw">Raw points</option><option value="voxel">Voxel preview (15 mm)</option></select></label>
<label><input type="checkbox" id="fov" checked> FOV guides</label>
<label><input type="checkbox" id="fingers" checked> Finger guides</label>
<button id="reset">Reset view</button>
</div>
<div class="stats">
<div class="stat"><small>Valid 3D points</small><b id="pts">0 / 192</b></div>
<div class="stat"><small>Fused voxels</small><b id="vox">0</b></div>
<div class="stat"><small>Nearest measured range</small><b id="near">--</b></div>
<div class="stat"><small>Sensor frames</small><b id="frames">0</b></div>
<div class="stat"><small>3-sensor cycle</small><b id="scan">--</b></div>
<div class="stat"><small>Fresh sensors</small><b id="fresh">0 / 3</b></div>
</div>
<div class="legend"><span><i class="dot" style="background:#ff6969"></i>S1 · 0x33</span><span><i class="dot" style="background:#61d7a3"></i>S2 · 0x30</span><span><i class="dot" style="background:#68a9ff"></i>S3 · 0x31</span><span><i class="dot" style="background:#ffd166"></i>Voxel preview</span></div>
<div class="note">The fixed mounting geometry is 101 mm radius, Z=0 mm, inward-facing, +30° elevation. Calibration corrections rotate around each fixed optical centre.</div>
</section>
<aside class="panel"><h2>Sensor matrices and orientation</h2><div class="sensors" id="sensorCards"></div><div class="note">Select matrix orientation separately for each sensor. Changing orientation clears that sensor's captured plane samples.</div></aside>
<section class="panel wide">
<h2>Priority 2 — Manual Calibration & Planar Fitting</h2>
<p class="note">Directly enter manual range scale, offset, and optical angle corrections, or capture non-parallel target planes to perform an automated fit. Click <b>Save calibration to ESP32</b> to store parameters permanently.</p>
<div class="calgrid" id="calParams"></div>
<div class="actions" style="margin-top:12px;"><button id="saveCalibration" class="act green">Save calibration to ESP32</button></div>
<hr style="border:0;border-top:1px solid #263b55;margin:14px 0;">
<div class="formgrid">
<label class="field">Sensor<select id="calSensor"><option value="0">S1 · 0x33</option><option value="1">S2 · 0x30</option><option value="2">S3 · 0x31</option></select></label>
<label class="field">Panel pose preset<select id="planePreset"><option value="A">A — normal to optical axis</option><option value="B">B — 35° toward sensor-right</option><option value="C">C — 35° toward sensor-down</option><option value="MANUAL">Manual plane values</option></select></label>
<label class="field">Plane normal X<input id="planeNx" type="number" step="0.0001" value="0"></label>
<label class="field">Plane normal Y<input id="planeNy" type="number" step="0.0001" value="-0.8660"></label>
<label class="field">Plane normal Z<input id="planeNz" type="number" step="0.0001" value="0.5000"></label>
<label class="field">Plane D (mm)<input id="planeD" type="number" step="0.001" value="32.531"></label>
<div class="presetInfo" id="presetInfo">Preset values are calculated from the fixed sensor geometry. Select S1, S2 or S3 and pose A, B or C to fill the plane equation automatically.</div>
</div>
<div class="actions"><button id="capturePlane">Capture plane from latest frame</button><button id="clearPlanes" class="danger">Clear all selected sensor captures</button><button id="fitSensor">Fit selected sensor</button></div>
<div class="capturePanel">
<h3>Captured frame review</h3>
<p class="editHelp">Frames are stored in this browser page until you reload it. Review each frame. You can delete only one frame, edit its plane equation, or click individual matrix cells to exclude or restore outlier zones. Excluded zones are not used by the fit.</p>
<div id="captureList" class="captureList"><div class="editHelp">No captured frames for this sensor.</div></div>
<div id="captureEditor" class="captureEditor" hidden>
<h4 id="captureEditorTitle">Frame editor</h4>
<div class="formgrid">
<label class="field">Normal X<input id="editNx" type="number" step="0.0001"></label>
<label class="field">Normal Y<input id="editNy" type="number" step="0.0001"></label>
<label class="field">Normal Z<input id="editNz" type="number" step="0.0001"></label>
<label class="field">Plane D (mm)<input id="editD" type="number" step="0.001"></label>
</div>
<div class="actions"><button id="applyFramePlane">Apply edited plane values</button><button id="closeFrameEditor">Close editor</button></div>
<p class="editHelp">Matrix positions are shown as row 0 at the top and column 0 at the left. Green = included. Red = excluded. Click a measured cell to toggle it.</p>
<div id="captureMatrix" class="captureMatrix"></div>
<div id="captureEditStatus" class="editHelp"></div>
</div>
</div>
<div id="planeStatus" class="result">No plane captures yet. Capture three different known plane orientations per sensor.</div>
</section>
<section class="panel wide">
<h2>Release 1.2 — Cylinder validation</h2>
<p class="note">Validation only. This tool does not change sensor calibration, mechanical transforms, FSR settings, motor settings or the grasp controller. Select only matrix zones that see the paper-covered cylinder. Z limits are expressed in the palm coordinate frame.</p>
<div class="formgrid tubeInputs">
  <label class="field">Reference diameter (mm)<input id="tubeDiameter" type="number" min="5" max="500" step="0.1" value="41.7"></label>
  <label class="field">Object height (mm)<input id="tubeHeight" type="number" min="5" max="1000" step="1" value="200"></label>
  <label class="field">Z minimum (mm)<input id="tubeZMin" type="number" step="1" value="-20"></label>
  <label class="field">Z maximum (mm)<input id="tubeZMax" type="number" step="1" value="200"></label>
  <label class="field">Range minimum (mm)<input id="tubeRangeMin" type="number" min="1" max="3500" step="1" value="20"></label>
  <label class="field">Range maximum (mm)<input id="tubeRangeMax" type="number" min="1" max="3500" step="1" value="220"></label>
</div>
<div class="actions">
  <button id="validateTube" class="goodButton">Run cylinder validation</button>
  <button id="resetTubeMasks" class="danger">Reset all zone masks</button>
</div>
<p class="editHelp">Zone masks are temporary and reset when this page reloads. Green = included; red = excluded. Click a cell to toggle it. The displayed range is the latest sensor measurement.</p>
<div id="tubeMasks" class="tubeMasks"></div>
<div class="plotPanel">
  <h3>Top-down XY validation plot</h3>
  <canvas id="tubePlot" width="1000" height="420"></canvas>
  <div class="editHelp">White dashed circle = reference diameter centred at the palm origin. Yellow circle = fitted diameter and centre. Point colours identify S1, S2 and S3.</div>
</div>
<div id="tubeResult" class="result">Waiting for validation. Select the target zones, then run the test.</div>
</section>
<section class="panel wide">
<h2>Mechanical pose and transformation matrices</h2>
<div class="note" id="poseInfo">Building transforms…</div>
<details><summary class="muted">Show 4×4 sensor transformation matrices</summary><pre id="matrices" style="white-space:pre-wrap;font-size:.68rem;color:#b7c8dc"></pre></details>
</section>
</div>
<script>
'use strict';
const CFG={radius:101.0,z:0.0,pitchDeg:30,yawDeg:[90,210,330],fovDeg:60,zoneStepDeg:7.5,palmRadius:50,tubeRadius:35,fingerTipRadius:170.28,fingerYawDeg:[30,150,270],voxelMm:15,fovDrawRange:135};
const names=['S1 · 0x33','S2 · 0x30','S3 · 0x31'];
const colors=['#ff6969','#61d7a3','#68a9ff'];
const orientationNames=['Native','Rotate 90° CW','Rotate 180°','Rotate 270° CW','Mirror columns','Mirror rows','Transpose','Anti-transpose'];
const sensorCal=[0,1,2].map(()=>({orientation:4,rangeScale:1,rangeOffset:0,yaw:0,pitch:0,roll:0,valid:false}));
const planeCaptures=[[],[],[]],lastCapturedSeq=[-1,-1,-1];
const tubeMasks=[0,1,2].map(()=>Array(64).fill(true));
const $=id=>document.getElementById(id),rad=d=>d*Math.PI/180;
const add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]],sub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]],mul=(a,s)=>[a[0]*s,a[1]*s,a[2]*s];
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2],cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const norm=a=>{const n=Math.hypot(a[0],a[1],a[2])||1;return mul(a,1/n);};
function matrixSourceIndex(s,row,col){let r=row,c=col;switch(sensorCal[s].orientation){case 1:r=7-col;c=row;break;case 2:r=7-row;c=7-col;break;case 3:r=col;c=7-row;break;case 4:r=row;c=7-col;break;case 5:r=7-row;c=col;break;case 6:r=col;c=row;break;case 7:r=7-col;c=7-row;break;}return r*8+c;}
function makeCard(s){const card=document.createElement('div');card.className='sensor';card.style.setProperty('--c',colors[s]);card.innerHTML=`<div class="sensorhead"><span>${names[s]}</span><span id="st${s}">WAIT</span></div><div class="sensorinfo" id="si${s}">No frame</div><label class="field">Matrix orientation<select id="ori${s}">${orientationNames.map((n,i)=>`<option value="${i}">${i}: ${n}</option>`).join('')}</select></label><div class="matrix" id="mx${s}"></div>`;$('sensorCards').appendChild(card);const cells=[];const m=card.querySelector('.matrix');for(let k=0;k<64;k++){const e=document.createElement('div');e.className='cell';e.textContent='·';m.appendChild(e);cells.push(e);}card.querySelector(`#ori${s}`).value=String(sensorCal[s].orientation);card.querySelector(`#ori${s}`).addEventListener('change',e=>{sensorCal[s]={orientation:Number(e.target.value),rangeScale:1,rangeOffset:0,yaw:0,pitch:0,roll:0,valid:false};planeCaptures[s]=[];lastCapturedSeq[s]=-1;rebuildPoses();updateCalibrationUI();if(lastData){updateCards(lastData);buildCloud();draw();}});return cells;}
const matrices=[0,1,2].map(makeCard);
function localRay(row,col){const ax=rad((col-3.5)*CFG.zoneStepDeg),ay=rad((3.5-row)*CFG.zoneStepDeg),ca=Math.cos(ay);return norm([ca*Math.sin(ax),-Math.sin(ay),ca*Math.cos(ax)]);}
function mechanicalAxes(s){const a=rad(CFG.yawDeg[s]),b=rad(CFG.pitchDeg),er=[Math.cos(a),Math.sin(a),0],et=[-Math.sin(a),Math.cos(a),0],up=[0,0,1];return{right:et,down:add(mul(er,-Math.sin(b)),mul(up,-Math.cos(b))),forward:add(mul(er,-Math.cos(b)),mul(up,Math.sin(b))),origin:[CFG.radius*Math.cos(a),CFG.radius*Math.sin(a),CFG.z]};}
function rotateLocal(v,p){const y=rad(p.yaw),x=rad(p.pitch),z=rad(p.roll);let q=[v[0]*Math.cos(z)-v[1]*Math.sin(z),v[0]*Math.sin(z)+v[1]*Math.cos(z),v[2]];q=[q[0],q[1]*Math.cos(x)-q[2]*Math.sin(x),q[1]*Math.sin(x)+q[2]*Math.cos(x)];q=[q[0]*Math.cos(y)+q[2]*Math.sin(y),q[1],-q[0]*Math.sin(y)+q[2]*Math.cos(y)];return q;}
function localToWorld(s,v,p=sensorCal[s]){const a=mechanicalAxes(s),q=rotateLocal(v,p);return add(add(mul(a.right,q[0]),mul(a.down,q[1])),mul(a.forward,q[2]));}
function makePoseMatrix(s,p=sensorCal[s]){const ex=localToWorld(s,[1,0,0],p),ey=localToWorld(s,[0,1,0],p),ez=localToWorld(s,[0,0,1],p),o=mechanicalAxes(s).origin;return new Float64Array([ex[0],ex[1],ex[2],0,ey[0],ey[1],ey[2],0,ez[0],ez[1],ez[2],0,o[0],o[1],o[2],1]);}
function transformPoint(M,p){return[M[0]*p[0]+M[4]*p[1]+M[8]*p[2]+M[12],M[1]*p[0]+M[5]*p[1]+M[9]*p[2]+M[13],M[2]*p[0]+M[6]*p[1]+M[10]*p[2]+M[14]];}
function transformDirection(M,p){return norm([M[0]*p[0]+M[4]*p[1]+M[8]*p[2],M[1]*p[0]+M[5]*p[1]+M[9]*p[2],M[2]*p[0]+M[6]*p[1]+M[10]*p[2]]);}
let poses=[0,1,2].map(i=>makePoseMatrix(i));
function rebuildPoses(){poses=[0,1,2].map(i=>makePoseMatrix(i));$('poseInfo').textContent=poses.map((m,i)=>`${names[i]} origin=(${m[12].toFixed(2)}, ${m[13].toFixed(2)}, ${m[14].toFixed(2)}) mm · yaw/pitch/roll=${sensorCal[i].yaw.toFixed(2)}/${sensorCal[i].pitch.toFixed(2)}/${sensorCal[i].roll.toFixed(2)}°`).join(' | ');$('matrices').textContent=poses.map((m,i)=>{const rows=[[m[0],m[4],m[8],m[12]],[m[1],m[5],m[9],m[13]],[m[2],m[6],m[10],m[14]],[m[3],m[7],m[11],m[15]]];return names[i]+'\n'+rows.map(r=>'['+r.map(v=>v.toFixed(4).padStart(9)).join(', ')+']').join('\n');}).join('\n\n');}
function pointFor(s,row,col,raw){const d=raw*sensorCal[s].rangeScale+sensorCal[s].rangeOffset;if(!Number.isFinite(d)||d<1)return null;const M=poses[s],ray=localRay(row,col);return transformPoint(M,mul(ray,d));}

function onManualCalChange(s){
  const sc=Number($('calScale'+s).value);
  const off=Number($('calOffset'+s).value);
  const y=Number($('calYaw'+s).value);
  const p=Number($('calPitch'+s).value);
  const r=Number($('calRoll'+s).value);
  if(Number.isFinite(sc)&&sc>=0.8&&sc<=1.2) sensorCal[s].rangeScale=sc;
  if(Number.isFinite(off)&&Math.abs(off)<=40) sensorCal[s].rangeOffset=off;
  if(Number.isFinite(y)&&Math.abs(y)<=8) sensorCal[s].yaw=y;
  if(Number.isFinite(p)&&Math.abs(p)<=8) sensorCal[s].pitch=p;
  if(Number.isFinite(r)&&Math.abs(r)<=8) sensorCal[s].roll=r;
  sensorCal[s].valid=true;
  rebuildPoses();
  if(lastData){buildCloud();draw();}
}

function renderCalibrationCards(){
  let h='';
  for(let s=0;s<3;s++){
    const c=sensorCal[s],caps=planeCaptures[s],active=caps.reduce((n,cap)=>n+cap.samples.filter(q=>q.included!==false).length,0);
    h+=`<div class="calcard">
      <h3>${names[s]} · ${c.valid?'CALIBRATED':'NOT VALIDATED'}</h3>
      <div class="formgrid">
        <label class="field">Range scale<input id="calScale${s}" type="number" step="0.00001" value="${c.rangeScale}" onchange="onManualCalChange(${s})"></label>
        <label class="field">Range offset (mm)<input id="calOffset${s}" type="number" step="0.1" value="${c.rangeOffset}" onchange="onManualCalChange(${s})"></label>
        <label class="field">Yaw (°)<input id="calYaw${s}" type="number" step="0.01" value="${c.yaw}" onchange="onManualCalChange(${s})"></label>
        <label class="field">Pitch (°)<input id="calPitch${s}" type="number" step="0.01" value="${c.pitch}" onchange="onManualCalChange(${s})"></label>
        <label class="field">Roll (°)<input id="calRoll${s}" type="number" step="0.01" value="${c.roll}" onchange="onManualCalChange(${s})"></label>
      </div>
      <div class="sensorinfo" style="margin-top:6px">Captured planes: ${caps.length} · included zones: ${active}</div>
    </div>`;
  }
  $('calParams').innerHTML=h;
}

let editingCaptureIndex=-1;
function selectedCalSensor(){return Number($('calSensor').value);}
function applyPlanePreset(){const s=selectedCalSensor(),key=$('planePreset').value;if(key==='MANUAL'){$('presetInfo').textContent='Manual mode. Enter the measured plane normal and D value in the palm coordinate frame.';return;}const a=mechanicalAxes(s);let P=add(a.origin,mul(a.forward,120));let n;if(key==='A')n=a.forward;else if(key==='B')n=add(mul(a.forward,Math.cos(rad(35))),mul(a.right,Math.sin(rad(35))));else n=add(mul(a.forward,Math.cos(rad(35))),mul(a.down,Math.sin(rad(35))));n=norm(n);const D=dot(n,P),clean=v=>Math.abs(v)<0.00005?0:v;P=P.map(clean);n=n.map(clean);$('planeNx').value=n[0].toFixed(4);$('planeNy').value=n[1].toFixed(4);$('planeNz').value=n[2].toFixed(4);$('planeD').value=D.toFixed(3);$('presetInfo').textContent=`${names[s]} · Pose ${key}. Panel centre P=(${P.map(v=>v.toFixed(3)).join(', ')}) mm. Normal n=(${n.map(v=>v.toFixed(4)).join(', ')}). D=n·P=${D.toFixed(3)} mm. Keep the panel centre at this position while changing its orientation.`;}
function renderCaptureReview(){const s=selectedCalSensor(),caps=planeCaptures[s];$('captureList').innerHTML=caps.length?caps.map((cap,i)=>{const active=cap.samples.filter(q=>q.included!==false).length,excluded=cap.samples.length-active,label=cap.preset||'Manual';return `<div class="captureRow"><div class="captureMeta"><strong>Frame ${i+1} · seq ${cap.seq} · pose ${label}</strong><br>${active}/${cap.samples.length} zones included${excluded?` · ${excluded} excluded`:''}<br>n=(${cap.n.map(v=>v.toFixed(3)).join(', ')}) · D=${cap.D.toFixed(2)} mm</div><button data-edit-cap="${i}">Review / edit</button><button class="danger" data-delete-cap="${i}">Delete frame</button></div>`;}).join(''):'<div class="editHelp">No captured frames for this sensor.</div>';
$('captureList').querySelectorAll('[data-edit-cap]').forEach(b=>b.addEventListener('click',()=>openCaptureEditor(Number(b.dataset.editCap))));
$('captureList').querySelectorAll('[data-delete-cap]').forEach(b=>b.addEventListener('click',()=>deleteCapture(Number(b.dataset.deleteCap))));}
function openCaptureEditor(index){const s=selectedCalSensor(),cap=planeCaptures[s][index];if(!cap)return;editingCaptureIndex=index;$('captureEditor').hidden=false;$('captureEditorTitle').textContent=`${names[s]} · frame ${index+1} · sequence ${cap.seq}`;$('editNx').value=cap.n[0].toFixed(5);$('editNy').value=cap.n[1].toFixed(5);$('editNz').value=cap.n[2].toFixed(5);$('editD').value=cap.D.toFixed(3);renderCaptureMatrix();}
function renderCaptureMatrix(){const s=selectedCalSensor(),cap=planeCaptures[s][editingCaptureIndex];if(!cap){$('captureEditor').hidden=true;editingCaptureIndex=-1;return;}const byCell=new Map(cap.samples.map((q,i)=>[q.row*8+q.col,{q,i}]));let html='';for(let k=0;k<64;k++){const item=byCell.get(k);if(!item){html+=`<button class="sampleTile empty" disabled>·</button>`;continue;}const {q,i}=item,included=q.included!==false;html+=`<button class="sampleTile ${included?'isIncluded':'isExcluded'}" data-sample-index="${i}" title="row ${q.row}, column ${q.col}, ${q.raw} mm — ${included?'included':'excluded'}">${q.raw}<small>${q.row},${q.col}</small></button>`;} $('captureMatrix').innerHTML=html;
$('captureMatrix').querySelectorAll('[data-sample-index]').forEach(b=>b.addEventListener('click',()=>{const sample=cap.samples[Number(b.dataset.sampleIndex)];sample.included=sample.included===false;renderCaptureMatrix();renderCaptureReview();updateCalibrationUI(false);}));
const active=cap.samples.filter(q=>q.included!==false).length;$('captureEditStatus').textContent=`${active} included / ${cap.samples.length-active} excluded / ${cap.samples.length} captured zones. These changes affect the fit only; raw readings remain unchanged.`;}
function deleteCapture(index){const s=selectedCalSensor();if(index<0||index>=planeCaptures[s].length)return;planeCaptures[s].splice(index,1);editingCaptureIndex=-1;$('captureEditor').hidden=true;lastCapturedSeq[s]=lastData?.sensors?.[s]?.frame_seq??lastCapturedSeq[s];updateCalibrationUI();$('planeStatus').textContent=`Deleted one capture from ${names[s]}. Other captures were kept.`;}
function updateCalibrationUI(updateStatus=true){renderCalibrationCards();renderCaptureReview();const s=selectedCalSensor(),caps=planeCaptures[s],active=caps.reduce((n,c)=>n+c.samples.filter(q=>q.included!==false).length,0),excluded=caps.reduce((n,c)=>n+c.samples.filter(q=>q.included===false).length,0);if(updateStatus)$('planeStatus').textContent=`${names[s]}: ${caps.length} captured frames, ${active} included zones, ${excluded} excluded zones. Need 3 diverse planes and at least 30 included zones to fit.`;}
$('calSensor').addEventListener('change',()=>{editingCaptureIndex=-1;$('captureEditor').hidden=true;applyPlanePreset();updateCalibrationUI();});
$('planePreset').addEventListener('change',applyPlanePreset);
['planeNx','planeNy','planeNz','planeD'].forEach(id=>$(id).addEventListener('input',()=>{if($('planePreset').value!=='MANUAL'){$('planePreset').value='MANUAL';$('presetInfo').textContent='Manual mode. Plane values were edited by hand. Confirm the target plane normal and D before capture.';}}));
function capturePlane(){if(!lastData||!lastData.sensors)return;const s=selectedCalSensor(),d=lastData.sensors[s];if(!d||!Array.isArray(d.values)||!d.last_ok){$('planeStatus').textContent='Selected sensor has no successful frame.';return;}const seq=Number(d.frame_seq||0);if(seq===lastCapturedSeq[s]){$('planeStatus').textContent='No new successful frame yet. Wait for the next sensor frame and try again.';return;}let n=[Number($('planeNx').value),Number($('planeNy').value),Number($('planeNz').value)],len=Math.hypot(...n),D=Number($('planeD').value);if(!Number.isFinite(len)||len<1e-6||!Number.isFinite(D)){$('planeStatus').textContent='Invalid plane equation.';return;}n=mul(n,1/len);D/=len;const samples=[];for(let row=0;row<8;row++)for(let col=0;col<8;col++){const raw=Number(d.values[matrixSourceIndex(s,row,col)]);if(Number.isFinite(raw)&&raw>=20&&raw<3500)samples.push({row,col,raw,included:true});}if(samples.length<12){$('planeStatus').textContent=`Only ${samples.length} valid zones. Use a larger target plane or improve its visibility.`;return;}if(planeCaptures[s].length>=8){$('planeStatus').textContent='Maximum of 8 plane captures per sensor reached. Delete a frame or clear captures to make room.';return;}const preset=$('planePreset').value;planeCaptures[s].push({n,D,samples,seq,preset});lastCapturedSeq[s]=seq;sensorCal[s].valid=false;editingCaptureIndex=planeCaptures[s].length-1;updateCalibrationUI();openCaptureEditor(editingCaptureIndex);$('planeStatus').textContent=`Captured ${names[s]} frame ${seq}: ${samples.length} zones. Review the frame and exclude any obvious background or outlier zones before fitting.`;}
$('capturePlane').addEventListener('click',capturePlane);
$('clearPlanes').addEventListener('click',()=>{const s=selectedCalSensor();planeCaptures[s]=[];lastCapturedSeq[s]=-1;editingCaptureIndex=-1;$('captureEditor').hidden=true;updateCalibrationUI();});
$('applyFramePlane').addEventListener('click',()=>{const s=selectedCalSensor(),cap=planeCaptures[s][editingCaptureIndex];if(!cap)return;let n=[Number($('editNx').value),Number($('editNy').value),Number($('editNz').value)],len=Math.hypot(...n),D=Number($('editD').value);if(!Number.isFinite(len)||len<1e-6||!Number.isFinite(D)){$('captureEditStatus').textContent='Invalid plane equation. No changes applied.';return;}n=mul(n,1/len);cap.n=n;cap.D=D/len;cap.preset='Manual edit';renderCaptureReview();renderCaptureMatrix();$('captureEditStatus').textContent=`Plane equation updated. n=(${n.map(v=>v.toFixed(4)).join(', ')}), D=${cap.D.toFixed(3)} mm.`;updateCalibrationUI(false);});
$('closeFrameEditor').addEventListener('click',()=>{$('captureEditor').hidden=true;editingCaptureIndex=-1;});
function planeFitMetrics(s,p){const a=mechanicalAxes(s);let sumSq=0,robustLoss=0,absResiduals=[],used=0;const huberMm=12;for(const cap of planeCaptures[s])for(const sample of cap.samples){if(sample.included===false)continue;const dir=localToWorld(s,localRay(sample.row,sample.col),p),den=dot(cap.n,dir);if(Math.abs(den)<0.08)continue;const expected=(cap.D-dot(cap.n,a.origin))/den;if(!Number.isFinite(expected)||expected<20||expected>3500)continue;const measured=p.rangeScale*sample.raw+p.rangeOffset;if(!Number.isFinite(measured)||measured<1)continue;const e=measured-expected,ae=Math.abs(e);sumSq+=e*e;robustLoss+=ae<=huberMm?0.5*e*e:huberMm*(ae-0.5*huberMm);absResiduals.push(ae);used++;}return{rms:used?Math.sqrt(sumSq/used):Infinity,objective:used?robustLoss/used:Infinity,n:used,abs:absResiduals};}
function planeDiversity(caps){if(caps.length<3)return false;for(let i=0;i<caps.length;i++)for(let j=i+1;j<caps.length;j++){const d=Math.abs(dot(caps[i].n,caps[j].n));if(Math.acos(Math.min(1,d))*180/Math.PI<18)return false;}return true;}
function clampParam(p,k){if(k<3)return Math.max(-8,Math.min(8,p[k]));if(k===3)return Math.max(.8,Math.min(1.2,p[k]));return Math.max(-40,Math.min(40,p[k]));}
function packParams(c){return[c.yaw,c.pitch,c.roll,c.rangeScale,c.rangeOffset];}
function unpackParams(v){return{yaw:v[0],pitch:v[1],roll:v[2],rangeScale:v[3],rangeOffset:v[4]};}
function fitSelectedSensor(){const s=selectedCalSensor(),caps=planeCaptures[s].filter(c=>c.samples.some(q=>q.included!==false)),sampleCount=caps.reduce((n,c)=>n+c.samples.filter(q=>q.included!==false).length,0);if(caps.length<3||!planeDiversity(caps)||sampleCount<30){$('planeStatus').textContent='Fit refused: keep at least 3 non-parallel plane captures with at least 30 included zones total.';return;}let p=packParams(sensorCal[s]),before=planeFitMetrics(s,unpackParams(p));if(before.n<30||!Number.isFinite(before.rms)){$('planeStatus').textContent='Fit refused: fewer than 30 rays intersect the entered planes within the supported range. Check plane normals and D values.';return;}
const stages=[{span:[4,4,4,.06,18],step:[1,1,1,.01,3]},{span:[1,1,1,.02,4],step:[.2,.2,.2,.002,.5]},{span:[.2,.2,.2,.004,1],step:[.05,.05,.05,.001,.25]}];
for(const stage of stages){for(let pass=0;pass<2;pass++){for(let k=0;k<5;k++){let best=p[k],bestCost=planeFitMetrics(s,unpackParams(p)).objective;for(let v=-stage.span[k];v<=stage.span[k]+1e-9;v+=stage.step[k]){const trial=p.slice();trial[k]=clampParam(p[k]+v,k);const metric=planeFitMetrics(s,unpackParams(trial));if(metric.n>=30&&metric.objective<bestCost){bestCost=metric.objective;best=trial[k];}}p[k]=best;}}}
const after=planeFitMetrics(s,unpackParams(p));const accepted=after.n>=30&&after.rms<=18&&after.rms<=before.rms+0.25;if(accepted){Object.assign(sensorCal[s],unpackParams(p));sensorCal[s].valid=true;rebuildPoses();if(lastData){buildCloud();draw();}}renderCalibrationCards();$('planeStatus').textContent=`${names[s]} plane fit ${accepted?'PASSED':'NOT ACCEPTED'}\nSamples used: ${after.n}\nRMS before: ${before.rms.toFixed(2)} mm\nRMS after: ${after.rms.toFixed(2)} mm\nCandidate yaw: ${p[0].toFixed(3)}°\nCandidate pitch: ${p[1].toFixed(3)}°\nCandidate roll: ${p[2].toFixed(3)}°\nCandidate range scale: ${p[3].toFixed(6)}\nCandidate range offset: ${p[4].toFixed(3)} mm\n${accepted?'Candidate applied. Review the result, then save it.':'Candidate rejected; current calibration remains unchanged. Check plane measurements, target coverage and capture diversity.'}`;}
$('fitSensor').addEventListener('click',fitSelectedSensor);
$('saveCalibration').addEventListener('click',async()=>{const p=new URLSearchParams();for(let s=0;s<3;s++){const c=sensorCal[s];p.set('o'+s,String(c.orientation));p.set('s'+s,String(c.rangeScale));p.set('b'+s,String(c.rangeOffset));p.set('y'+s,String(c.yaw));p.set('p'+s,String(c.pitch));p.set('r'+s,String(c.roll));p.set('v'+s,c.valid?'1':'0');}try{const r=await fetch('/api/calibration/save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString(),cache:'no-store'});const j=await r.json();if(!r.ok||!j.ok)throw new Error(j.error||'save failed');$('planeStatus').textContent='Calibration saved to ESP32 Preferences. Controller and FSM settings were not changed.';}catch(e){$('planeStatus').textContent='Calibration save failed: '+e.message;}});
function solve3(A,b){A=A.map((r,i)=>r.concat([b[i]]));for(let c=0;c<3;c++){let p=c;for(let r=c+1;r<3;r++)if(Math.abs(A[r][c])>Math.abs(A[p][c]))p=r;if(Math.abs(A[p][c])<1e-9)return null;[A[p],A[c]]=[A[c],A[p]];const q=A[c][c];for(let j=c;j<4;j++)A[c][j]/=q;for(let r=0;r<3;r++)if(r!==c){const f=A[r][c];for(let j=c;j<4;j++)A[r][j]-=f*A[c][j];}}return[A[0][3],A[1][3],A[2][3]];}
function fitCircleBasic(pts){if(pts.length<6)return null;const A=[[0,0,0],[0,0,0],[0,0,0]],b=[0,0,0];for(const q of pts){const x=q.p[0],y=q.p[1],v=x*x+y*y,f=[x,y,1];for(let i=0;i<3;i++){b[i]+=f[i]*v;for(let j=0;j<3;j++)A[i][j]+=f[i]*f[j];}}const z=solve3(A,b);if(!z)return null;const cx=z[0]/2,cy=z[1]/2,r2=z[2]+cx*cx+cy*cy;if(!Number.isFinite(r2)||r2<=0)return null;return{cx,cy,radius:Math.sqrt(r2)};}
function median(a){if(!a.length)return NaN;const s=a.slice().sort((x,y)=>x-y),m=s.length>>1;return s.length%2?s[m]:(s[m-1]+s[m])/2;}
function fitCircleRobust(pts){
  if(pts.length<6)return null;
  let keep=pts.slice(),fit=null;
  for(let iter=0;iter<5;iter++){
    fit=fitCircleBasic(keep);
    if(!fit)return null;
    const residuals=keep.map(q=>Math.abs(Math.hypot(q.p[0]-fit.cx,q.p[1]-fit.cy)-fit.radius));
    const m=median(residuals),mad=median(residuals.map(x=>Math.abs(x-m)));
    const limit=Math.max(6,m+3.0*1.4826*mad);
    const next=keep.filter(q=>Math.abs(Math.hypot(q.p[0]-fit.cx,q.p[1]-fit.cy)-fit.radius)<=limit);
    if(next.length<6||next.length===keep.length)break;
    keep=next;
  }
  fit=fitCircleBasic(keep);
  if(!fit||keep.length<6)return null;
  const errors=keep.map(q=>Math.abs(Math.hypot(q.p[0]-fit.cx,q.p[1]-fit.cy)-fit.radius)).sort((a,b)=>a-b);
  const rms=Math.sqrt(errors.reduce((a,v)=>a+v*v,0)/errors.length);
  const angles=keep.map(q=>(Math.atan2(q.p[1]-fit.cy,q.p[0]-fit.cx)+Math.PI*2)%(Math.PI*2)).sort((a,b)=>a-b);
  let largestGap=0;
  for(let i=0;i<angles.length;i++){
    const next=i===angles.length-1?angles[0]+Math.PI*2:angles[i+1];
    largestGap=Math.max(largestGap,next-angles[i]);
  }
  const coverage=Math.max(0,360-largestGap*180/Math.PI);
  const sensorCounts=[0,0,0];
  for(const q of keep)sensorCounts[q.s]++;
  return{...fit,n:keep.length,rms,median:median(errors),p95:errors[Math.min(errors.length-1,Math.ceil(.95*errors.length)-1)],coverage,sensorCounts,points:keep};
}

function initTubeMasks(){
  const root=$('tubeMasks');
  root.innerHTML='';
  for(let s=0;s<3;s++){
    const card=document.createElement('div');
    card.className='tubeMaskCard';
    card.innerHTML=`<div class="tubeMaskHead"><span>${names[s]} · target-zone mask</span><span id="tubeMaskCount${s}">64/64</span></div>
      <div class="tubeMaskMeta" id="tubeMaskMeta${s}">Waiting for sensor frame</div>
      <div class="tubeMaskActions"><button type="button" data-mask-all="${s}">Include all</button><button type="button" data-mask-none="${s}">Exclude all</button></div>
      <div class="maskGrid" id="tubeMaskGrid${s}"></div>`;
    root.appendChild(card);
    const grid=card.querySelector('.maskGrid');
    for(let k=0;k<64;k++){
      const b=document.createElement('button');
      b.type='button';b.className='maskCell on';b.dataset.sensor=String(s);b.dataset.zone=String(k);b.textContent='·';
      b.title=`${names[s]} row ${Math.floor(k/8)}, column ${k%8}`;
      b.addEventListener('click',()=>{
        tubeMasks[s][k]=!tubeMasks[s][k];
        updateTubeMaskDisplays();
      });
      grid.appendChild(b);
    }
    card.querySelector(`[data-mask-all="${s}"]`).addEventListener('click',()=>{tubeMasks[s].fill(true);updateTubeMaskDisplays();});
    card.querySelector(`[data-mask-none="${s}"]`).addEventListener('click',()=>{tubeMasks[s].fill(false);updateTubeMaskDisplays();});
  }
  updateTubeMaskDisplays();
}

function updateTubeMaskDisplays(){
  for(let s=0;s<3;s++){
    const d=lastData?.sensors?.[s];
    let included=0;
    for(let k=0;k<64;k++){
      const row=Math.floor(k/8),col=k%8,raw=Array.isArray(d?.values)?Number(d.values[matrixSourceIndex(s,row,col)]):NaN;
      const b=$(`tubeMaskGrid${s}`)?.children?.[k];
      if(!b)continue;
      if(tubeMasks[s][k])included++;
      b.className=`maskCell ${tubeMasks[s][k]?'on':'off'}`;
      b.textContent=Number.isFinite(raw)&&raw>=20&&raw<4000?String(Math.round(raw)):'·';
      b.title=`${names[s]} row ${row}, column ${col}: ${Number.isFinite(raw)?raw+' mm':'no reading'} — ${tubeMasks[s][k]?'included':'excluded'}`;
    }
    const count=$(`tubeMaskCount${s}`);
    if(count)count.textContent=`${included}/64 selected`;
    const meta=$(`tubeMaskMeta${s}`);
    if(meta){
      const age=d?.age_ms;
      meta.textContent=!d?'No sensor data':!d.last_ok?'Last read failed':age===null||age===undefined?'No successful frame':`Frame ${d.frame_seq??0} · ${age} ms old · ${d.ok??0} OK / ${d.err??0} errors`;
    }
  }
}

function collectTubePoints(cfg){
  const pts=[],sensorInfo=[];
  for(let s=0;s<3;s++){
    const d=lastData?.sensors?.[s];
    let selected=0,used=0;
    if(d&&d.last_ok&&Array.isArray(d.values)&&d.age_ms!==null&&Number(d.age_ms)<=1000){
      for(let row=0;row<8;row++)for(let col=0;col<8;col++){
        const k=row*8+col;
        if(!tubeMasks[s][k])continue;
        selected++;
        const raw=Number(d.values[matrixSourceIndex(s,row,col)]);
        if(!Number.isFinite(raw)||raw<4000||raw<cfg.rangeMin||raw>cfg.rangeMax)continue;
        const p=pointFor(s,row,col,raw);
        if(!p||p[2]<cfg.zMin||p[2]>cfg.zMax)continue;
        pts.push({p,s,value:raw,row,col,frameSeq:Number(d.frame_seq||0),ageMs:Number(d.age_ms)});
        used++;
      }
    }
    sensorInfo.push({selected,used,age:d?.age_ms??null,online:!!(d?.last_ok&&d?.age_ms!==null&&Number(d.age_ms)<=1000)});
  }
  return{pts,sensorInfo};
}

function drawTubePlot(pts,fit,referenceDiameter){
  const canvas=$('tubePlot'),rect=canvas.getBoundingClientRect(),dpr=Math.min(2,window.devicePixelRatio||1);
  const w=Math.max(1,rect.width),h=Math.max(1,rect.height);
  canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  const refR=referenceDiameter/2,fitR=fit?fit.radius:refR;
  const xs=pts.map(q=>q.p[0]),ys=pts.map(q=>q.p[1]);
  xs.push(-refR,refR,fit?fit.cx-fitR:-refR,fit?fit.cx+fitR:refR);
  ys.push(-refR,refR,fit?fit.cy-fitR:-refR,fit?fit.cy+fitR:refR);
  const minX=Math.min(...xs)-12,maxX=Math.max(...xs)+12,minY=Math.min(...ys)-12,maxY=Math.max(...ys)+12;
  const spanX=Math.max(20,maxX-minX),spanY=Math.max(20,maxY-minY),pad=28;
  const scale=Math.min((w-pad*2)/spanX,(h-pad*2)/spanY);
  const cx=w/2-((minX+maxX)/2)*scale,cy=h/2+((minY+maxY)/2)*scale;
  const sx=x=>cx+x*scale,sy=y=>cy-y*scale;
  ctx.strokeStyle='#1b2b40';ctx.lineWidth=1;
  for(let v=Math.ceil(minX/20)*20;v<=maxX;v+=20){ctx.beginPath();ctx.moveTo(sx(v),pad/2);ctx.lineTo(sx(v),h-pad/2);ctx.stroke();}
  for(let v=Math.ceil(minY/20)*20;v<=maxY;v+=20){ctx.beginPath();ctx.moveTo(pad/2,sy(v));ctx.lineTo(w-pad/2,sy(v));ctx.stroke();}
  ctx.strokeStyle='#6b8099';ctx.lineWidth=1.2;ctx.beginPath();ctx.moveTo(sx(0),pad/2);ctx.lineTo(sx(0),h-pad/2);ctx.moveTo(pad/2,sy(0));ctx.lineTo(w-pad/2,sy(0));ctx.stroke();
  ctx.setLineDash([6,4]);ctx.strokeStyle='#d8e2ef';ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(sx(0),sy(0),refR*scale,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);
  if(fit){ctx.strokeStyle='#ffd166';ctx.lineWidth=2;ctx.beginPath();ctx.arc(sx(fit.cx),sy(fit.cy),fit.radius*scale,0,Math.PI*2);ctx.stroke();ctx.fillStyle='#ffd166';ctx.beginPath();ctx.arc(sx(fit.cx),sy(fit.cy),3,0,Math.PI*2);ctx.fill();}
  const pointColors=['#ff6969','#61d7a3','#68a9ff'];
  for(const q of pts){ctx.fillStyle=pointColors[q.s];ctx.beginPath();ctx.arc(sx(q.p[0]),sy(q.p[1]),3,0,Math.PI*2);ctx.fill();}
  ctx.fillStyle='#b9cce2';ctx.font='11px system-ui';ctx.fillText('X (mm)',w-52,h-8);ctx.fillText('Y (mm)',7,14);ctx.fillText('Reference Ø'+referenceDiameter.toFixed(1),8,h-9);
}

function validateTube(){
  const diameter=Number($('tubeDiameter').value),height=Number($('tubeHeight').value);
  const zMin=Number($('tubeZMin').value),zMax=Number($('tubeZMax').value);
  const rangeMin=Number($('tubeRangeMin').value),rangeMax=Number($('tubeRangeMax').value);
  const cfg={diameter,height,zMin,zMax,rangeMin,rangeMax};
  if(!Number.isFinite(diameter)||diameter<5||diameter>500||!Number.isFinite(height)||height<5||
     !Number.isFinite(zMin)||!Number.isFinite(zMax)||zMin>=zMax||
     !Number.isFinite(rangeMin)||!Number.isFinite(rangeMax)||rangeMin<1||rangeMin>=rangeMax||rangeMax>3500){
    $('tubeResult').textContent='Invalid settings. Check diameter, height, Z limits and range limits.';
    return;
  }
  if(!lastData?.sensors){$('tubeResult').textContent='No sensor data is available yet.';return;}
  const collected=collectTubePoints(cfg),pts=collected.pts;
  const all=fitCircleRobust(pts);
  const qualifiedSensors=all?all.sensorCounts.filter(n=>n>0).length:0;
  const enoughPoints=!!all&&all.n>=30;
  const enoughCoverage=!!all&&all.coverage>=180;
  const enoughSensors=qualifiedSensors>=2;
  const passed=enoughPoints&&enoughCoverage&&enoughSensors;
  const lines=[
    `Reference diameter: ${diameter.toFixed(2)} mm (comparison only; not used by the fit)`,
    `Reference object height: ${height.toFixed(1)} mm`,
    `Palm-frame Z window: ${zMin.toFixed(1)} to ${zMax.toFixed(1)} mm`,
    `Sensor range window: ${rangeMin.toFixed(1)} to ${rangeMax.toFixed(1)} mm`,
    `Maximum frame age: 1000 ms`,
    `Selected target-zone points after filters: ${pts.length}`
  ];
  for(let s=0;s<3;s++){
    const r=fitCircleRobust(pts.filter(q=>q.s===s)),info=collected.sensorInfo[s];
    const sourceStatus=info.online?`fresh frame (${info.age} ms old)`:'offline, failed or stale frame — skipped';
    if(r){
      lines.push(`${names[s]}: ${sourceStatus}; selected zones=${info.selected}, usable points=${info.used}, fit points=${r.n}, diameter=${(2*r.radius).toFixed(2)} mm, centre=(${r.cx.toFixed(2)}, ${r.cy.toFixed(2)}) mm, RMS=${r.rms.toFixed(2)} mm, median=${r.median.toFixed(2)} mm, P95=${r.p95.toFixed(2)} mm, arc coverage=${r.coverage.toFixed(1)}°`);
    }else{
      lines.push(`${names[s]}: ${sourceStatus}; selected zones=${info.selected}, usable points=${info.used}, insufficient geometry for an individual circle fit`);
    }
  }
  if(all){
    const fittedDiameter=2*all.radius,delta=fittedDiameter-diameter,pct=delta/diameter*100;
    lines.push(`COMBINED: fit points=${all.n}, diameter=${fittedDiameter.toFixed(2)} mm, diameter error=${delta>=0?'+':''}${delta.toFixed(2)} mm (${pct>=0?'+':''}${pct.toFixed(2)}%), centre=(${all.cx.toFixed(2)}, ${all.cy.toFixed(2)}) mm, centre offset=${Math.hypot(all.cx,all.cy).toFixed(2)} mm, RMS=${all.rms.toFixed(2)} mm, median=${all.median.toFixed(2)} mm, P95=${all.p95.toFixed(2)} mm, angular coverage=${all.coverage.toFixed(1)}°, contributing sensors=${qualifiedSensors}`);
  }else{
    lines.push('COMBINED: no numerically stable circle fit.');
  }
  lines.push(`Data sufficiency: ${passed?'PASS':'INSUFFICIENT'}`);
  lines.push(`  ≥30 robust fit points: ${enoughPoints?'PASS':'FAIL'}`);
  lines.push(`  ≥180° angular coverage: ${enoughCoverage?'PASS':'FAIL'}`);
  lines.push(`  ≥2 contributing sensors: ${enoughSensors?'PASS':'FAIL'}`);
  lines.push('A data-sufficiency PASS does not mean the measured diameter is accurate.');
  lines.push('Validation only: no calibration, FSR, motor or FSM values were changed.');
  lines.push('Manual zone masks are temporary and will reset when this page reloads.');
  $('tubeResult').textContent=lines.join('\n');
  drawTubePlot(pts,all,diameter);
}
$('validateTube').addEventListener('click',validateTube);
$('resetTubeMasks').addEventListener('click',()=>{
  for(let s=0;s<3;s++)tubeMasks[s].fill(true);
  updateTubeMaskDisplays();
});
function initCalibration(data){if(!data||!Array.isArray(data.calibration))return;for(let s=0;s<3;s++){const c=data.calibration[s];if(!c)continue;sensorCal[s]={orientation:Number(c.orientation??4),rangeScale:Number(c.rangeScale??1),rangeOffset:Number(c.rangeOffset??0),yaw:Number(c.yaw??0),pitch:Number(c.pitch??0),roll:Number(c.roll??0),valid:!!c.valid};$('ori'+s).value=String(sensorCal[s].orientation);}rebuildPoses();renderCalibrationCards();}
let lastData=null,allPoints=[],fusedPoints=[],firstLoad=true;
function buildCloud(){allPoints=[];fusedPoints=[];if(!lastData||!lastData.sensors)return;const maxR=Number($('range').value);for(let s=0;s<3;s++){const d=lastData.sensors[s];if(!d||!Array.isArray(d.values)||!d.last_ok)continue;for(let row=0;row<8;row++)for(let col=0;col<8;col++){const raw=Number(d.values[matrixSourceIndex(s,row,col)]);if(!Number.isFinite(raw)||raw<20||raw>=4000||raw>maxR)continue;const p=pointFor(s,row,col,raw);if(p)allPoints.push({p,s,value:raw,row,col,frameSeq:d.frame_seq});}}const map=new Map(),v=CFG.voxelMm;for(const q of allPoints){const k=[Math.floor(q.p[0]/v),Math.floor(q.p[1]/v),Math.floor(q.p[2]/v)].join(',');let b=map.get(k);if(!b){b={sum:[0,0,0],n:0,mask:0};map.set(k,b);}b.sum=add(b.sum,q.p);b.n++;b.mask|=(1<<q.s);}for(const b of map.values())fusedPoints.push({p:mul(b.sum,1/b.n),n:b.n,mask:b.mask});$('pts').textContent=allPoints.length+' / 192';$('vox').textContent=String(fusedPoints.length);$('near').textContent=allPoints.length?Math.min(...allPoints.map(p=>p.value))+' mm':'--';$('frames').textContent=lastData.frames||0;$('scan').textContent=lastData.scan_us?(lastData.scan_us/1000).toFixed(1)+' ms':'--';}
function updateCards(d){let fresh=0;d.sensors.forEach((s,i)=>{const state=!s.found?'NOT FOUND':!s.initialized?'INIT FAIL':!s.configured?'MODE FAIL':!s.last_ok?'READ ERROR':'ONLINE';$('st'+i).textContent=state;if(s.last_ok&&s.age_ms!==null&&s.age_ms<1000)fresh++;$('si'+i).textContent=`${s.age_ms===null?'no data':s.age_ms+' ms old'} · read ${(s.read_us/1000).toFixed(1)} ms · OK ${s.ok} / ERR ${s.err} · frame ${s.frame_seq||0}`;for(let row=0;row<8;row++)for(let col=0;col<8;col++){const raw=s.values?s.values[matrixSourceIndex(i,row,col)]:null,e=matrices[i][row*8+col];e.textContent=(raw===null||raw===undefined||raw>=4000)?'·':raw;e.style.background=raw===null||raw===undefined||raw>=4000?'#34445b':raw<250?'#ee765b':raw<600?'#e6b85b':'#65a9d8';}});$('sys').textContent=fresh+'/3 sensors fresh';$('fresh').textContent=fresh+' / 3';}
const canvas=$('view'),ctx=canvas.getContext('2d');let camYaw=rad(42),camElev=rad(34),camDist=440;
function resize(){const r=canvas.getBoundingClientRect(),dpr=Math.min(2,window.devicePixelRatio||1);canvas.width=Math.max(1,Math.floor(r.width*dpr));canvas.height=Math.max(1,Math.floor(r.height*dpr));ctx.setTransform(dpr,0,0,dpr,0,0);draw();}
function cameraBasis(){const target=[0,0,10],ce=Math.cos(camElev),se=Math.sin(camElev),cy=Math.cos(camYaw),sy=Math.sin(camYaw),pos=[target[0]+camDist*ce*cy,target[1]+camDist*ce*sy,target[2]+camDist*se],forward=norm(sub(target,pos));let right=norm(cross(forward,[0,0,1]));if(Math.hypot(...right)<.001)right=[1,0,0];return{pos,forward,right,up:norm(cross(right,forward))};}
function project(p,b,w,h){const q=sub(p,b.pos),depth=dot(q,b.forward);if(depth<5)return null;const focal=(h*.9)/(2*Math.tan(rad(35)/2));return{x:w/2+dot(q,b.right)*focal/depth,y:h/2-dot(q,b.up)*focal/depth,z:depth};}
function line3(a,b,color,width=1,dash=[]){const basis=cameraBasis(),A=project(a,basis,canvas.clientWidth,canvas.clientHeight),B=project(b,basis,canvas.clientWidth,canvas.clientHeight);if(!A||!B)return;ctx.save();ctx.strokeStyle=color;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.beginPath();ctx.moveTo(A.x,A.y);ctx.lineTo(B.x,B.y);ctx.stroke();ctx.restore();}
function label3(p,t,color){const q=project(p,cameraBasis(),canvas.clientWidth,canvas.clientHeight);if(!q)return;ctx.fillStyle=color;ctx.font='12px system-ui';ctx.fillText(t,q.x+5,q.y-5);}
function drawGrid(){for(let k=-200;k<=200;k+=25){const c=k===0?'#526984':'#1b2b40',w=k===0?1.3:.65;line3([k,-200,-1],[k,200,-1],c,w);line3([-200,k,-1],[200,k,-1],c,w);}}
function drawCircle(r,z,color){let prev=null;for(let i=0;i<=96;i++){const a=i*Math.PI*2/96,p=[r*Math.cos(a),r*Math.sin(a),z];if(prev)line3(prev,p,color,1.5);prev=p;}}
function drawGripper(){drawGrid();drawCircle(CFG.palmRadius,0,'#8092a9');drawCircle(CFG.tubeRadius,0,'#44ff44');if($('fingers').checked)CFG.fingerYawDeg.forEach((a,i)=>{const t=rad(a),p0=[CFG.palmRadius*Math.cos(t),CFG.palmRadius*Math.sin(t),0],p1=[CFG.fingerTipRadius*Math.cos(t),CFG.fingerTipRadius*Math.sin(t),0];line3(p0,p1,'#7b8da5',5);label3(p1,'F'+(i+1),'#aebed2');});line3([0,0,0],[40,0,0],'#ef7777',2);label3([42,0,0],'X','#ef7777');line3([0,0,0],[0,40,0],'#6ed8a5',2);label3([0,42,0],'Y','#6ed8a5');line3([0,0,0],[0,0,40],'#74adff',2);label3([0,0,42],'Z','#74adff');}
function drawSensorsAndFov(){for(let i=0;i<3;i++){const o=mechanicalAxes(i).origin,color=colors[i],q=project(o,cameraBasis(),canvas.clientWidth,canvas.clientHeight);if(q){ctx.beginPath();ctx.fillStyle=color;ctx.arc(q.x,q.y,5,0,Math.PI*2);ctx.fill();}label3(o,'S'+(i+1),color);if($('fov').checked){const d=CFG.fovDrawRange,h=d*Math.tan(rad(CFG.fovDeg/2)),corners=[[-h,h,d],[h,h,d],[h,-h,d],[-h,-h,d]].map(p=>transformPoint(poses[i],p));corners.forEach(p=>line3(o,p,color,1,[4,4]));for(let j=0;j<4;j++)line3(corners[j],corners[(j+1)%4],color,1,[3,4]);}}}
function drawPoint(p,color,radius){const q=project(p,cameraBasis(),canvas.clientWidth,canvas.clientHeight);if(!q)return;const r=Math.max(1.5,Math.min(6,radius*420/(q.z+200)));ctx.beginPath();ctx.fillStyle=color;ctx.globalAlpha=.9;ctx.arc(q.x,q.y,r,0,Math.PI*2);ctx.fill();ctx.globalAlpha=1;}
function draw(){const w=canvas.clientWidth,h=canvas.clientHeight;if(!w||!h)return;ctx.clearRect(0,0,w,h);drawGripper();drawSensorsAndFov();if($('display').value==='voxel')fusedPoints.forEach(p=>drawPoint(p.p,'#ffd166',2.8+Math.min(2,p.n)));else allPoints.slice().sort((a,b)=>b.value-a.value).forEach(p=>drawPoint(p.p,colors[p.s],2.8));}
async function refresh(){try{const r=await fetch('/api/data',{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);lastData=await r.json();if(firstLoad){initCalibration(lastData);firstLoad=false;}updateCards(lastData);updateTubeMaskDisplays();buildCloud();draw();}catch(e){$('sys').textContent='WEB/API ERROR';}}
$('range').addEventListener('input',e=>{$('rangeV').textContent=e.target.value+' mm';buildCloud();draw();});$('display').addEventListener('change',draw);$('fov').addEventListener('change',draw);$('fingers').addEventListener('change',draw);$('reset').addEventListener('click',()=>{camYaw=rad(42);camElev=rad(34);camDist=440;draw();});
let drag=null;canvas.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY};canvas.setPointerCapture(e.pointerId);});canvas.addEventListener('pointermove',e=>{if(!drag)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;drag={x:e.clientX,y:e.clientY};camYaw+=dx*.008;camElev=Math.max(rad(5),Math.min(rad(85),camElev-dy*.008));draw();});canvas.addEventListener('pointerup',()=>drag=null);canvas.addEventListener('pointercancel',()=>drag=null);canvas.addEventListener('wheel',e=>{e.preventDefault();camDist=Math.max(210,Math.min(1000,camDist+e.deltaY*.45));draw();},{passive:false});window.addEventListener('resize',resize);
rebuildPoses();applyPlanePreset();renderCalibrationCards();updateCalibrationUI();initTubeMasks();resize();refresh();setInterval(refresh,250);
</script></body></html>)HTML";

void setupRoutes() {
  server.on("/",HTTP_GET,[]{server.send_P(200,"text/html; charset=utf-8",PAGE);});
  server.on("/sen0628",HTTP_GET,[]{server.send_P(200,"text/html; charset=utf-8",SEN0628_PAGE);});
  server.on("/api/status",HTTP_GET,apiStatus);
  server.on("/api/cf35/diagnostics",HTTP_GET,apiCf35Diagnostics);
  server.on("/api/cf35/read",HTTP_POST,requestCf35Diagnostics);
  server.on("/api/config",HTTP_GET,apiConfig);
  server.on("/api/config",HTTP_POST,saveConfigApi);
  server.on("/api/fsr/config",HTTP_GET,apiFsrConfig);
  server.on("/api/fsr/save",HTTP_POST,apiFsrSave);
  server.on("/api/fsr/zero",HTTP_POST,apiFsrZero);
  server.on("/api/data",HTTP_GET,handleApiData);
  server.on("/api/calibration/save",HTTP_POST,handleSaveSensorCalibration);
  server.on("/api/control/start",HTTP_POST,[]{queueCommandApi(CMD_START);});
  server.on("/api/control/release",HTTP_POST,[]{queueCommandApi(CMD_RELEASE);});
  server.on("/api/control/pause",HTTP_POST,[]{queueCommandApi(CMD_PAUSE);});
  server.on("/api/control/reset",HTTP_POST,[]{queueCommandApi(CMD_RESET);});
  server.on("/api/control/estop",HTTP_POST,[]{queueCommandApi(CMD_ESTOP);});
  server.on("/api/control/stepOpen",HTTP_POST,[]{queueCommandApi(CMD_STEP_OPEN);});
  server.on("/api/control/stepClose",HTTP_POST,[]{queueCommandApi(CMD_STEP_CLOSE);});
  server.on("/api/control/setOpenZero",HTTP_POST,[]{queueCommandApi(CMD_SET_OPEN_ZERO);});
  server.on("/api/control/setTouch",HTTP_POST,[]{queueCommandApi(CMD_SET_TOUCH);});
  server.on("/api/control/moveOpen",HTTP_POST,[]{queueCommandApi(CMD_MOVE_OPEN);});
  server.onNotFound([](){server.send(404,"text/plain","Not found");});
}

// Core 0 task: WebServer only. No Wire, ADC or motor UART calls are allowed here.
void webServerTask(void*) {
  for(;;) { server.handleClient(); vTaskDelay(pdMS_TO_TICKS(2)); }
}

// Core 1, lower priority: SEN0628 I2C only. The controller task can pre-empt
// this task while a sensor is between library calls. No motor or ADC access here.
void sensorTask(void*) {
  Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN,I2C_FREQUENCY);
  Wire.setTimeOut(120);
  initializeSensors();
  uint8_t sensorIndex=0;
  uint32_t scanStart=micros();
  for(;;) {
    if(sensorIndex==0)scanStart=micros();
    readOneSensor(sensorIndex);
    sensorIndex=(sensorIndex+1)%SENSOR_COUNT;
    if(sensorIndex==0) {
      const uint32_t duration=micros()-scanStart;
      portENTER_CRITICAL(&sharedMux);lastScanUs=duration;portEXIT_CRITICAL(&sharedMux);
    }
    // Give the higher-priority motor/FSR controller the CPU between sensors.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// Core 1, high priority: motor UART, FSR ADC, servo setup and FSM only.
void controllerTask(void*) {
  analogReadResolution(12);
  for(uint8_t i=0;i<FSR_COUNT;i++){pinMode(FSR_PINS[i],INPUT);analogSetPinAttenuation(FSR_PINS[i],ADC_11db);}
  busBegin(servoBaud);
  bool stopOk=motorStop();
  pollMotor();readMotorMode();sampleFsr();
  if(!stopOk||!motor.ok)setFault("Boot motor stop or link check failed");
  else if(!motorModeReady())setAction("Motor is online but not in position mode 0. Motion is locked.",false);
  else setAction("Controller ready. Configure OPEN and three-finger TOUCH references.",true);
  uint32_t lastFsr=millis(),lastMotor=millis(),lastMode=millis();
  for(;;) {
    uint32_t now=millis();
    bool stopNow=false;portENTER_CRITICAL(&sharedMux);stopNow=estopRequested;estopRequested=false;portEXIT_CRITICAL(&sharedMux);
    if(stopNow) processControlCommand({CMD_ESTOP,0});
    processControlQueue();
    if(now-lastFsr>=FSR_PERIOD_MS){lastFsr=now;sampleFsr();}
    if(now-lastMotor>=MOTOR_PERIOD_MS){lastMotor=now;pollMotor();}
    bool readDiag=false;portENTER_CRITICAL(&sharedMux);readDiag=cf35DiagRequested;cf35DiagRequested=false;portEXIT_CRITICAL(&sharedMux);if(readDiag)pollCf35Diagnostics();
    if(now-lastMode>=MODE_CHECK_MS){lastMode=now;if(!readMotorMode()&&fsmEnabled)setFault("CF35-12 mode register read failed during FSM motion");}
    updatePositionMotion();updateFsm();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  Serial.begin(115200);delay(250);loadAll();
  controlQueue=xQueueCreate(8,sizeof(ControlCommand));
  WiFi.mode(WIFI_AP);WiFi.softAP("SENSE-GRIP","12341234");
  setupRoutes();server.begin();
  Serial.print("SENSE-GRIP Release 3.0.4-dev · http://");Serial.println(WiFi.softAPIP());
  BaseType_t controlOk=xTaskCreatePinnedToCore(controllerTask,"controlCore1",8192,nullptr,3,&controlTaskHandle,1);
  BaseType_t sensorOk=xTaskCreatePinnedToCore(sensorTask,"sen0628Core1",8192,nullptr,1,&sensorTaskHandle,1);
  BaseType_t webOk=xTaskCreatePinnedToCore(webServerTask,"webCore0",12288,nullptr,1,&webTaskHandle,0);
  if(!controlQueue||controlOk!=pdPASS)Serial.println("ERROR: controller task/queue unavailable; motor motion must not be used");
  if(sensorOk!=pdPASS)Serial.println("WARNING: SEN0628 task unavailable; LiDAR page will show sensors offline");
  if(webOk!=pdPASS)Serial.println("ERROR: could not start web task on core 0");
  portENTER_CRITICAL(&sharedMux);controllerTaskReady=(controlQueue!=nullptr&&controlOk==pdPASS);portEXIT_CRITICAL(&sharedMux);
}
void loop() {
  // Work is executed by explicit pinned tasks: web on Core 0, hardware controller on Core 1.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
