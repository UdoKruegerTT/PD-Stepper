/*
 * PD Stepper Webpage Example:
 * 
 *********  Software Version 1.0 ******************
 * 
 *    How to Use:
 * 1. Connect to created WiFi Network named "PD Stepper"
 * 2. On a browser visit 192.168.4.1
 * 
 *  For more info and to purchase PD Stepper kits visit:
 *  https://thingsbyjosh.com
 * 
 * TODO:
 * - More TMC error conditions (including power bad?)
 * - Stall guard disabled at lower speeds
 * - Fix non smooth position control (due to webserver interuptions)
*/




#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h> // https://github.com/ESP32Async/ESPAsyncWebServer (ENSURE THIS VERSION IS USED)
#include <AsyncTCP.h>  //https://github.com/ESP32Async/AsyncTCP/tree/main (ENSURE THIS VERSION IS USED)
#include <TMC2209.h> //  https://github.com/janelia-arduino/TMC2209/tree/main
#include <Preferences.h> //for saving to flash (instead of old EEPROM lib)
#include <ArduinoOTA.h> // wireless firmware upload via PlatformIO `espota`
#include "index_html.h"

Preferences preferences;

//access point SSID and password (password = "" for no password)
const char *ssid = "PD Stepper";
const char *password = "";

AsyncWebServer server(80);

//TMC2209 setup
TMC2209 stepper_driver;
HardwareSerial & serial_stream = Serial2;
const long SERIAL_BAUD_RATE = 115200;

const uint8_t RUN_CURRENT_PERCENT = 100; //how much current to run at (0-100%)

//TMC2209 Stepper Driver
#define TMC_EN  21
#define STEP    5
#define DIR     6
#define MS1     1
#define MS2     2
#define SPREAD  7
#define TMC_TX  17
#define TMC_RX  18
#define DIAG    16
#define INDEX   11

//PD Trigger (CH224K)
#define PG      15  //power good singnal (dont enable stepper untill this is good)
#define CFG1    38
#define CFG2    48
#define CFG3    47

//Other
#define VBUS    4
#define NTC     7
#define LED1    10
#define LED2    12
#define SW1     35
#define SW2     36
#define SW3     37
#define AUX1    14
#define AUX2    13


//Global variables
int set_speed = 0;
bool PGState = 0; //state of the power good signal from PD sink IC
bool enabledState = 0;
bool state = 0; //step state

//AS5600 Hall Effect Encoder
#include <Wire.h> //For I2C for encoder
#define AS5600_ADDRESS 0x36 // I2C address of the AS5600 sensor
signed long total_encoder_counts = 0;
unsigned long lastEncRead = 0;

int mainFreq = 10; //Scheduled frequency = 100hz (for slower tasks, encoder reading etc)

//button read and debounce
bool incButtonState = HIGH;
bool decButtonState = HIGH;
bool resetButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

int buttonSpeed = 0;

//Voltage reading and calc
float VBusVoltage = 0;
float VREF = 3.3;
const float DIV_RATIO = 0.1189427313;  //20k&2.7K Voltage Divider

//Values received from websever save command
String enabled1 = "enabled";
String setVoltage = "12";
String microsteps = "32";
String current = "30";
String stallThreshold = "10";
String standstillMode = "NORMAL";
String homeAngle = "0";   // filter wheel home angle in deg, persisted to flash

//variable updated in callback
volatile bool speedUpdatePending = false;
volatile int pendingSpeed = 0;
volatile bool posUpdatePending = false;
volatile int pendingPosMode = 0;

//Varaiables for position control (open loop)
signed long setPoint = 0;
signed long CurrentPosition = 0;
unsigned long lastStep = 0;

// Filter wheel state — closed-loop around the AS5600 encoder via VACTUAL.
// Empirical TMC2209 sweep on this board shows the motor stalls outside a
// narrow VACTUAL "sweet spot" (good: |slider|=30..50 i.e. VACTUAL ~1000..1600
// at microsteps=32; worse above and below). We pick speeds inside that band.
static const int   FILTER_NUM_POSITIONS   = 6;
static const float FILTER_STOP_TOLERANCE  = 0.5f;    // deg
// Two-phase velocity profile: cruise fast, then slow down for final approach.
// FAST ≈ 365 deg/s — roughly 5× the previous 2000 setting.
// SLOW ≈ 36 deg/s — used inside DECEL_RANGE for precise braking.
// At SLOW + 5 ms poll cadence one step ≈ 0.18 deg < 0.5 deg stop tolerance.
static const int32_t FILTER_VELOCITY_FAST  = 4000;   // ~320 deg/s — stalls at ≥4800
static const int32_t FILTER_VELOCITY_SLOW  = 1000;
static const float   FILTER_DECEL_RANGE    = 4.0f;   // deg
// Sign of motor motion relative to encoder angle.
// Empirically confirmed: positive VACTUAL *decreases* encoder angle on this
// board (motor phase order inverted). Setting -1 corrects the closed-loop
// direction so the wheel moves toward the target rather than away from it.
static const int   FILTER_MOTOR_ENC_SIGN  = -1;

volatile bool   filterMotionActive  = false;
volatile float  filterTargetAngle   = 0.0f;

// Shortest signed angular difference (target - current) wrapped to (-180, +180].
// Handles the 0/360 boundary regardless of how many turns apart the two values
// nominally are.
static inline float shortestAngleDeltaDeg(float target, float current){
  float d = fmodf(target - current, 360.0f);
  if (d > 180.0f)  d -= 360.0f;
  if (d <= -180.0f) d += 360.0f;
  return d;
}
signed long filterEncoderZero = 0;      // encoder counts that correspond to 0 deg
signed long filterHomeSetPoint = 0;     // CurrentPosition value at filter position 0

volatile bool  filterUpdatePending = false;
volatile int   filterAction = 0;   // 1=goto_pos, 2=step, 3=set_zero, 4=goto_angle, 5=goto_home, 6=rt_start
volatile int   filterParamInt = 0;
volatile float filterParamFloat = 0.0f;

// Random position test state machine
static const int RT_MOVES = 8;
struct RtResult {
  int8_t pos;
  float  target_deg, actual_deg, error_deg, settle_s;
  bool   timeout;
};
static RtResult  rtResults[RT_MOVES];
volatile int     rtStep    = 0;    // written by loop(), read by web callback
volatile bool    rtRunning = false;
volatile bool    rtDone    = false;
static int       rtPrevPos = -1;
static bool      rtWaiting = false;
static uint32_t  rtStart   = 0;
static uint32_t  rtRandSeed = 12345;

// Forward declarations needed when building as a regular C++ translation unit.
String readPGState();
String readVoltage();
String readEncoderPos();
String readTMCStatus();
String readStallStatus();
String readFilterAngle();
String readFilterPosition();
float  getFilterAngleDeg();
int    getFilterPositionN();
void   handleFilterAction();
String processor(const String& var);
void readEncoder();
void configureSettings();
void readSettings();
void writeSettings();
void serviceRandomTest();

//read state of PG pin to display on webpage
String readPGState(){
  PGState = digitalRead(PG);
  if (PGState == 0){
    return ("Power Good");
  } else {
    return ("Power Bad");
  }
}

// read VBUS voltage to display on webpage
String readVoltage() {
  uint32_t mvSum = 0;
  int samples = 10; // Average 10 readings to smooth out electrical noise

  for (int i = 0; i < samples; i++) {
    mvSum += analogReadMilliVolts(VBUS);
  }

  float avgMilliVolts = (float)mvSum / (float)samples;
  
  // avgMilliVolts / 1000.0 = Actual voltage at the ESP32 pin
  // Dividing by DIV_RATIO scales it back up to the battery/VBUS voltage
  VBusVoltage = (avgMilliVolts / 1000.0) / DIV_RATIO;

  // Returning with 2 decimal places for better readability
  return String(VBusVoltage, 2) + "V";
}

//read encoder pos as angle (deg) to display on webpage
String readEncoderPos(){
  readEncoder();
  return String(getFilterAngleDeg(), 1) + "&deg;";
}

// Filter wheel: current angle in [0, 360) relative to the stored zero
float getFilterAngleDeg(){
  long c = total_encoder_counts - filterEncoderZero;
  long m = c % 4096;
  if (m < 0) m += 4096;
  return (float)m * 360.0f / 4096.0f;
}

// Raw encoder angle [0, 360) — independent of the Set-Zero offset.
// Used by Go-Home so the stored home position survives Set-Zero Here.
static float getRawEncoderAngleDeg(){
  long m = total_encoder_counts % 4096;
  if (m < 0) m += 4096;
  return (float)m * 360.0f / 4096.0f;
}

// Convert a RAW encoder angle to the currently-displayed frame
// (so we can reuse filterTargetAngle + shortestAngleDelta in displayed
// frame). displayed = raw - zeroAngle (mod 360).
static float rawAngleToDisplayed(float rawDeg){
  long zeroMod = filterEncoderZero % 4096;
  if (zeroMod < 0) zeroMod += 4096;
  float zeroDeg = (float)zeroMod * 360.0f / 4096.0f;
  float d = fmodf(rawDeg - zeroDeg, 360.0f);
  if (d < 0) d += 360.0f;
  return d;
}

// Derive filter slot (0..5) from the current angle
int getFilterPositionN(){
  float a = getFilterAngleDeg();
  int p = (int)((a / 60.0f) + 0.5f);
  if (p >= FILTER_NUM_POSITIONS) p -= FILTER_NUM_POSITIONS;
  if (p < 0) p += FILTER_NUM_POSITIONS;
  return p;
}

String readFilterAngle(){
  readEncoder();
  return String(getFilterAngleDeg(), 1);
}

String readFilterPosition(){
  readEncoder();
  return String(getFilterPositionN());
}

// Normalize an angle to [0, 360).
static inline float normalizeAngleDeg(float a){
  a = fmodf(a, 360.0f);
  if (a < 0) a += 360.0f;
  return a;
}

// Start a closed-loop move to filterTargetAngle at full speed (direction is
// picked as shortest path). Called from handleFilterAction after updating
// filterTargetAngle.
static int32_t filterCurrentVelocity = 0;
static unsigned long filterLastServiceMs = 0;

// Kick off closed-loop motion toward filterTargetAngle at FAST speed.
// serviceFilterMotion() downshifts to SLOW in the final FILTER_DECEL_RANGE
// degrees and stops when within FILTER_STOP_TOLERANCE.
static void filterStartMotionToTarget(){
  // Ensure the original step-pulse position control is quiescent.
  setPoint = CurrentPosition;
  readEncoder();
  float delta = shortestAngleDeltaDeg(filterTargetAngle, getFilterAngleDeg());
  float dist  = fabsf(delta);
  if (dist <= FILTER_STOP_TOLERANCE) {
    stepper_driver.moveAtVelocity(0);
    filterCurrentVelocity = 0;
    filterMotionActive = false;
    return;
  }
  int32_t spd = (dist <= FILTER_DECEL_RANGE) ? FILTER_VELOCITY_SLOW : FILTER_VELOCITY_FAST;
  int32_t v = spd * (int32_t)((delta > 0) ? 1 : -1) * FILTER_MOTOR_ENC_SIGN;
  stepper_driver.moveAtVelocity(v);
  filterCurrentVelocity = v;
  filterMotionActive = true;
}

// Executed from loop() in response to a webserver request.
void handleFilterAction(){
  switch (filterAction) {
    case 1: { // goto_pos: move to target slot (nearest instance via shortest path)
      int n = filterParamInt;
      if (n < 0) n = 0;
      if (n >= FILTER_NUM_POSITIONS) n = FILTER_NUM_POSITIONS - 1;
      filterTargetAngle = (float)n * (360.0f / FILTER_NUM_POSITIONS);
      filterStartMotionToTarget();
      break;
    }
    case 2: { // step +/- one filter slot (encoder-angle sense)
      int dir = (filterParamInt >= 0) ? 1 : -1;
      readEncoder();
      filterTargetAngle = normalizeAngleDeg(getFilterAngleDeg()
                                            + dir * (360.0f / FILTER_NUM_POSITIONS));
      filterStartMotionToTarget();
      break;
    }
    case 3: { // set_zero: stop motion, declare current mechanical position as 0 deg
      stepper_driver.moveAtVelocity(0);
      setPoint = CurrentPosition;
      filterMotionActive = false;
      readEncoder();
      filterEncoderZero = total_encoder_counts;
      filterHomeSetPoint = CurrentPosition;
      break;
    }
    case 4: { // goto_angle: drive to an arbitrary angle (DISPLAYED frame)
      filterTargetAngle = normalizeAngleDeg(filterParamFloat);
      filterStartMotionToTarget();
      break;
    }
    case 5: { // goto_home: drive to filterParamFloat in the RAW encoder frame
      // Storing home in raw frame means Set-Zero-Here can change the display
      // offset without invalidating the home position.
      float rawTarget = normalizeAngleDeg(filterParamFloat);
      filterTargetAngle = rawAngleToDisplayed(rawTarget);
      filterStartMotionToTarget();
      break;
    }
    case 6: { // rt_start: kick off the random position test
      if (!rtRunning) {
        rtRandSeed = (uint32_t)millis();
        rtStep     = 0;
        rtDone     = false;
        rtPrevPos  = -1;
        rtWaiting  = false;
        rtRunning  = true;
      }
      break;
    }
    default:
      break;
  }
}

// Closed-loop service for filter motion. Runs at ~50 Hz (UART writes to the
// TMC2209 are ~400 us each, so rewriting VACTUAL too often clogs the bus).
// Only rewrites VACTUAL when the commanded speed or direction actually
// changes.
static void serviceFilterMotion(){
  if (!filterMotionActive) return;
  unsigned long now = millis();
  if (now - filterLastServiceMs < 5) return;
  filterLastServiceMs = now;

  readEncoder();
  float delta = shortestAngleDeltaDeg(filterTargetAngle, getFilterAngleDeg());
  float dist  = fabsf(delta);
  if (dist <= FILTER_STOP_TOLERANCE) {
    if (filterCurrentVelocity != 0) {
      stepper_driver.moveAtVelocity(0);
      filterCurrentVelocity = 0;
    }
    filterMotionActive = false;
    return;
  }
  int32_t spd = (dist <= FILTER_DECEL_RANGE) ? FILTER_VELOCITY_SLOW : FILTER_VELOCITY_FAST;
  int32_t v = spd * (int32_t)((delta > 0) ? 1 : -1) * FILTER_MOTOR_ENC_SIGN;
  if (v != filterCurrentVelocity) {
    stepper_driver.moveAtVelocity(v);
    filterCurrentVelocity = v;
  }
}

// Simple LCG — avoids stdlib rand(), picks a slot different from prev.
static int rtRandNext(int prev) {
  rtRandSeed = rtRandSeed * 1664525UL + 1013904223UL;
  int p = (int)((rtRandSeed >> 16) % 6);
  if (p == prev) p = (p + 1) % 6;
  return p;
}

// Non-blocking random test state machine. Called every loop() after
// serviceFilterMotion() so filterMotionActive is already up to date.
void serviceRandomTest() {
  if (!rtRunning) return;

  if (rtWaiting) {
    float elapsed = (float)(millis() - rtStart) / 1000.0f;
    bool  timedOut = elapsed > 5.0f;
    if (filterMotionActive && !timedOut) return;   // still moving, not timed out

    if (timedOut && filterMotionActive) {
      stepper_driver.moveAtVelocity(0);
      filterMotionActive    = false;
      filterCurrentVelocity = 0;
    }
    readEncoder();
    float actual = getFilterAngleDeg();
    float target = rtResults[rtStep].target_deg;
    float err    = fabsf(shortestAngleDeltaDeg(target, actual));
    rtResults[rtStep].actual_deg = actual;
    rtResults[rtStep].error_deg  = err;
    rtResults[rtStep].settle_s   = elapsed;
    rtResults[rtStep].timeout    = timedOut;
    rtStep++;
    rtWaiting = false;
    if (rtStep >= RT_MOVES) { rtRunning = false; rtDone = true; return; }
  }

  // Kick off the next move.
  int pos = rtRandNext(rtPrevPos);
  rtPrevPos                = pos;
  rtResults[rtStep].pos        = (int8_t)pos;
  rtResults[rtStep].target_deg = (float)pos * 60.0f;
  filterTargetAngle        = rtResults[rtStep].target_deg;
  filterStartMotionToTarget();
  rtStart   = millis();
  rtWaiting = true;
}

String readTMCStatus(){
  bool hardware_disabled = stepper_driver.hardwareDisabled();
  if (hardware_disabled){
   return ("Hardware Disabled");
  }

  TMC2209::Status status = stepper_driver.getStatus();
  if (status.over_temperature_warning){
    return ("Over Temp Warning");
  }
  else if (status.over_temperature_shutdown){
    return ("Over Temp Shutdown");
  }
  return ("No Errors");
}

String readStallStatus(){
  return String(stepper_driver.getStallGuardResult());
//  if (digitalRead(DIAG) == HIGH){
//    return ("Stalled");
//  } else{
//    return ("Not Stalled");
//  }
}


//updates placeholder varibles in the HTML code //puts current arduino settings into webpage
//Puts stored values into the HTML value
String processor(const String& var)
{
  if(var == "enabled1"){
    if (enabled1 == "enabled"){
      return "checked";
    } else { return "";}
  }

  if (var == "microsteps"){
    return String(microsteps);
  }

  if(var == "voltage"){
     return String(setVoltage);
  }

  if(var == "current"){
     return String(current);
  }

  if(var == "stall_threshold"){
     return String(stallThreshold);
  }

  if(var == "standstill_mode"){
     return String(standstillMode);
  }

  if(var == "home_angle"){
     return String(homeAngle);
  }

  // Avoid undefined behavior if an unknown template token is requested.
  return String();
}


void setup() {
  delay(200); // Give USB serial time to settle after boot.
  Serial.begin(115200);
  Serial.println("Code Starting");

    //PD Trigger Setup
  pinMode(PG, INPUT);
  pinMode(CFG1, OUTPUT);
  pinMode(CFG2, OUTPUT);
  pinMode(CFG3, OUTPUT);
                            //  5V   9V   12V   15V   20V  (Can also be changed on the fly)
  digitalWrite(CFG1, LOW);  //  1    0     0     0     0
  digitalWrite(CFG2, LOW);  //  -    0     0     1     1
  digitalWrite(CFG3, HIGH); //  -    0     1     1     0

  //General
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  

  //Setup serial comms with TMC2209
  pinMode(MS1, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(TMC_EN, OUTPUT);
  pinMode(DIAG, INPUT);
  digitalWrite(TMC_EN, LOW); //Enabled here and later enabled/disabled over UART

  digitalWrite(MS2, LOW);

  //AS5600 Hall Encoder Setup
  Wire.begin(SDA, SCL);  //start wire with earlier defined pins

  //ADC Setup
  analogSetPinAttenuation(VBUS, ADC_11db);

  readSettings(); //get saved values from EEPROM
  
  stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.enableAutomaticCurrentScaling(); //current control mode
//  stepper_driver.enableCoolStep();
  stepper_driver.enableStealthChop(); //stealth chop needs to be enabled for stall detect
  stepper_driver.setCoolStepDurationThreshold(5000); //TCOOLTHRS (DIAG only enabled when TSTEP smaller than this)
  stepper_driver.disable();

  configureSettings(); //use saved settings

  // Set up ESP32 as an Access Point
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(ssid, password);
  if (!apStarted) {
    Serial.println("ERROR: Failed to start SoftAP");
  }

  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(ip);

  
  // Serve HTML page with JavaScript for updating values
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", index_html, processor);
  });
  server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", "{\"status\":\"ok\",\"service\":\"pd-stepper\"}");
  });
  server.on("/powergood", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readPGState());
  });
  server.on("/voltage", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readVoltage());
  });
  server.on("/position", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readEncoderPos());
  });
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readTMCStatus());
  });
  server.on("/stallguard", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readStallStatus());
  });
  // Soft-reboot. Handy when the motor gets stuck and the step/action state
  // needs resetting without cycling USB-C power. Accepts GET so you can just
  // paste http://192.168.4.1/reboot into a browser. Response is sent first so
  // the client sees a 200 before the MCU actually restarts.
  auto rebootHandler = [](AsyncWebServerRequest *request){
    request->send(200, "text/html",
      "<html><body style='font-family:sans-serif;background:#232324;color:#fff;text-align:center;padding:40px;'>"
      "<h2>Rebooting…</h2>"
      "<p>Give it ~5 seconds, then <a href='/' style='color:#fc4903;'>click here</a> to return.</p>"
      "</body></html>");
    delay(200);
    ESP.restart();
  };
  server.on("/reboot", HTTP_GET,  rebootHandler);
  server.on("/reboot", HTTP_POST, rebootHandler);
  server.on("/debug", HTTP_GET, [](AsyncWebServerRequest *request){
    String s = "setPoint=" + String(setPoint) +
               " CurrentPosition=" + String(CurrentPosition) +
               " diff=" + String(setPoint - CurrentPosition) +
               " filterAction=" + String(filterAction) +
               " filterParamFloat=" + String(filterParamFloat) +
               " filterParamInt=" + String(filterParamInt) +
               " microsteps=" + microsteps +
               " total_encoder_counts=" + String(total_encoder_counts) +
               " filterEncoderZero=" + String(filterEncoderZero) +
               " rtRunning=" + String((int)rtRunning) +
               " rtDone=" + String((int)rtDone) +
               " rtStep=" + String(rtStep) +
               " rtWaiting=" + String((int)rtWaiting) +
               " filterMotionActive=" + String((int)filterMotionActive);
    request->send(200, "text/plain", s);
  });
  server.on("/filter/angle", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readFilterAngle());
  });
  server.on("/filter/position", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readFilterPosition());
  });
  server.on("/filter/raw_angle", HTTP_GET, [](AsyncWebServerRequest *request){
    readEncoder();
    request->send(200, "text/plain", String(getRawEncoderAngleDeg(), 1));
  });

  // Register more-specific routes BEFORE the /filter catch-all POST so that
  // ESPAsyncWebServer's first-match routing selects the right handler.
  server.on("/filter/randomtest/start", HTTP_POST, [](AsyncWebServerRequest *request){
    filterAction        = 6;
    filterUpdatePending = true;
    request->send(200);
  });

  server.on("/filter", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("action", true)) {
      String action = request->getParam("action", true)->value();
      if (action == "goto_pos") {
        int pos = request->hasParam("pos", true) ? request->getParam("pos", true)->value().toInt() : 0;
        filterParamInt = pos;
        filterAction = 1;
        filterUpdatePending = true;
      } else if (action == "step") {
        int dir = request->hasParam("dir", true) ? request->getParam("dir", true)->value().toInt() : 1;
        filterParamInt = dir;
        filterAction = 2;
        filterUpdatePending = true;
      } else if (action == "set_zero") {
        filterAction = 3;
        filterUpdatePending = true;
      } else if (action == "goto_angle") {
        float a = request->hasParam("angle", true) ? request->getParam("angle", true)->value().toFloat() : 0.0f;
        filterParamFloat = a;
        filterAction = 4;
        filterUpdatePending = true;
      } else if (action == "goto_home") {
        // If no angle was given, fall back to the persisted home angle.
        float a = request->hasParam("angle", true)
                    ? request->getParam("angle", true)->value().toFloat()
                    : homeAngle.toFloat();
        filterParamFloat = a;
        filterAction = 5;
        filterUpdatePending = true;
      } else if (action == "randomtest_start") {
        filterAction = 6;
        filterUpdatePending = true;
      }
    }
    request->send(200);
  });

  server.on("/filter/randomtest/status", HTTP_GET, [](AsyncWebServerRequest *request){
    String j = "{\"running\":";
    j += rtRunning ? "true" : "false";
    j += ",\"done\":";
    j += rtDone ? "true" : "false";
    j += ",\"total\":";   j += String(RT_MOVES);
    j += ",\"count\":";   j += String(rtStep);
    j += ",\"results\":[";
    for (int i = 0; i < rtStep; i++) {
      if (i > 0) j += ",";
      j += "{\"pos\":"     + String(rtResults[i].pos)              +
           ",\"target\":"  + String(rtResults[i].target_deg, 1)    +
           ",\"actual\":"  + String(rtResults[i].actual_deg, 1)    +
           ",\"error\":"   + String(rtResults[i].error_deg,  2)    +
           ",\"settle\":"  + String(rtResults[i].settle_s,   2)    +
           ",\"timeout\":" + String(rtResults[i].timeout ? "true" : "false") +
           "}";
    }
    j += "]}";
    request->send(200, "application/json", j);
  });

  // Route to handle slider position update
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("slider", true)) {
      const AsyncWebParameter* p = request->getParam("slider", true);
      pendingSpeed = p->value().toInt();
      speedUpdatePending = true; // Signal the loop to update velocity
    }
  
    if (request->hasParam("positionControl", true)) {
      const AsyncWebParameter* p = request->getParam("positionControl", true);
      pendingPosMode = p->value().toInt();
      posUpdatePending = true; // Signal the loop to update position
    }
    request->send(200);
  });

  //handle post request of saving form
  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam("enabled1", true)) { //when checked "enabled1" variable is present
      enabled1 = "enabled";
    } else {
      enabled1 = "disabled";
    }
    if (request->hasParam("setvoltage", true)) {
      inputMessage = request->getParam("setvoltage", true)->value();
      setVoltage = inputMessage;     
      stepper_driver.moveAtVelocity(0); //stop when voltage changed
    }
    if (request->hasParam("microsteps", true)) {
      inputMessage = request->getParam("microsteps", true)->value();
      microsteps = inputMessage;

      stepper_driver.moveAtVelocity(0); //stop when microsteps changed
    }
    if (request->hasParam("current", true)) {
      inputMessage = request->getParam("current", true)->value();
      current = inputMessage;
    }
    if (request->hasParam("stall_threshold", true)) {
      inputMessage = request->getParam("stall_threshold", true)->value();
      stallThreshold = inputMessage;
    }
    if (request->hasParam("standstill_mode", true)) {
      inputMessage = request->getParam("standstill_mode", true)->value();
      standstillMode = inputMessage;
    }
    if (request->hasParam("home_angle", true)) {
      homeAngle = request->getParam("home_angle", true)->value();
    }
    writeSettings();  // persist all settings together
//    request->send(200, "text/html", "<h2>Saved</h2><br><a href=\"/\">Return to Home Page</a>"); //respond with save page
    request->redirect("/"); // Redirct back to main page to avaid having /save in URL
  });

  server.begin();
  Serial.println("HTTP server started");

  ArduinoOTA.setHostname("pd-stepper");
  ArduinoOTA.begin();
  Serial.println("ArduinoOTA started");

  digitalWrite(LED1, HIGH); //flash LED after setep complete
  delay(200);
  digitalWrite(LED1, LOW); 

}

void loop() {

  ArduinoOTA.handle();

  //Handle WebServer hardware requests safely in the main thread
  if (speedUpdatePending) {
    set_speed = pendingSpeed;
    stepper_driver.moveAtVelocity(set_speed * (microsteps.toInt()));
    speedUpdatePending = false;
  }

  if (posUpdatePending) {
    stepper_driver.moveAtVelocity(0); // Stop velocity mode
    if (pendingPosMode == 1)      setPoint -= 25600;
    else if (pendingPosMode == 2) setPoint -= 12800;
    else if (pendingPosMode == 3) setPoint += 12800;
    else if (pendingPosMode == 4) setPoint += 25600;
    posUpdatePending = false;
  }

  if (filterUpdatePending) {
    if (filterAction == 6 || !rtRunning) {
      handleFilterAction();
    }
    filterUpdatePending = false;
  }

  // Closed-loop stop-on-target for the filter wheel.
  serviceFilterMotion();
  serviceRandomTest();

  if (millis() - lastEncRead >= mainFreq){ //main loop
    lastEncRead = millis();
    //readEncoder(); //need to constantly read encoder in order to catch wrap around

    digitalWrite(LED2, digitalRead(DIAG)); //Stall detection

    PGState = digitalRead(PG);
    if (PGState == LOW and enabled1 == "enabled" and enabledState == 0){ //only enable if PD chip reports good
      stepper_driver.enable();
      enabledState = 1;
    } else if ((PGState == HIGH or enabled1 == "disabled") and enabledState == 1){ //Disable stepper if box unchecked or PG is high
      stepper_driver.disable();
      enabledState = 0;
    }
  }

  
  //position control done here (open-loop for now)
  int delaySpeed = 4500; //delay speed smaller value = faster
  int microSteps = microsteps.toInt();
  int delaySpeedAdjusted = delaySpeed/microSteps; //adjusting speed depending on microsteps
  
  if (setPoint > CurrentPosition){ //position control
    if (micros()-lastStep > delaySpeedAdjusted){ //always move same speed regardless of what microsteps set
      digitalWrite(DIR, LOW);
      digitalWrite(STEP, state);
      state = !state;
      CurrentPosition = CurrentPosition + (256/microSteps);  //update current position taking into account microsteps set
      lastStep = micros();
    }
  } else if (setPoint < CurrentPosition){ //position control
    if (micros()-lastStep > delaySpeedAdjusted){
      digitalWrite(DIR, HIGH);
      digitalWrite(STEP, state);
      state = !state;
      CurrentPosition = CurrentPosition - (256/microSteps);  //update current position taking into account microsteps set
      lastStep = micros();
    }
  }

  //Handle buttons inputs (seperate velocity control)
  if ((millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    bool currentIncButtonState = digitalRead(SW3);
    bool currentDecButtonState = digitalRead(SW1);
    bool currentResetButtonState = digitalRead(SW2);
  
    if (currentIncButtonState != incButtonState) {
      incButtonState = currentIncButtonState;
      if (incButtonState == LOW) {
        buttonSpeed = buttonSpeed + 30;
        if (buttonSpeed > 330){
          buttonSpeed = 330;
        }
        stepper_driver.moveAtVelocity(buttonSpeed*(microsteps.toInt()));
      }
    }
  
    if (currentDecButtonState != decButtonState) {
      decButtonState = currentDecButtonState;
      if (decButtonState == LOW) {
        buttonSpeed = buttonSpeed -30;
        if (buttonSpeed < -330){
          buttonSpeed = -330;
        }
        stepper_driver.moveAtVelocity(buttonSpeed*(microsteps.toInt()));
      }
    }
  
    if (currentResetButtonState != resetButtonState) {
      resetButtonState = currentResetButtonState;
      if (resetButtonState == LOW) {
        buttonSpeed = 0;
        stepper_driver.moveAtVelocity(0);
      }
    }
  }

  
}

void readEncoder(){
  int raw_counts;
  static int prev_raw_counts = 0;
  static signed long revolutions = 0;
//  float angle;
    // Request the raw encoder counts from the AS5600 sensor
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C); // Register address for raw angle output (0x0C)
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDRESS, 2); // Request 2 bytes of data
  if (Wire.available() >= 2) {
    raw_counts = Wire.read() << 8 | Wire.read(); // Combine two bytes to get the counts value
  }

  // Convert raw counts to angle in degrees if that is what's needed
//  angle = (raw_counts * 360.0) / 4096.0;

  // Check to see if it has gone past "home" over one full rotation
  if (prev_raw_counts > 3000 && raw_counts < 1000) {
    revolutions++;
  } else if (prev_raw_counts < 1000 && raw_counts > 3000) {
    revolutions--;
  }

  // Update the previous raw counts
  prev_raw_counts = raw_counts;
  total_encoder_counts = raw_counts + (4096 * revolutions);
}



//takes saved values impliments them (e.g after settings changed)
void configureSettings(){

  if (setVoltage == "5"){
                                 //  5V   9V   12V   15V   20V  (Can also be changed on the fly)
      digitalWrite(CFG1, HIGH);  //  1    0     0     0     0
  } else if (setVoltage == "9"){
                                //  5V   9V   12V   15V   20V  (Can also be changed on the fly)
      digitalWrite(CFG1, LOW);  //  1    0     0     0     0
      digitalWrite(CFG2, LOW);  //  -    0     0     1     1
      digitalWrite(CFG3, LOW);  //  -    0     1     1     0
  } else if (setVoltage == "12"){
                                //  5V   9V   12V   15V   20V  (Can also be changed on the fly)
      digitalWrite(CFG1, LOW);  //  1    0     0     0     0
      digitalWrite(CFG2, LOW);  //  -    0     0     1     1
      digitalWrite(CFG3, HIGH); //  -    0     1     1     0
  } else if (setVoltage == "15"){
                                //  5V   9V   12V   15V   20V  (Can also be changed on the fly)
      digitalWrite(CFG1, LOW);  //  1    0     0     0     0
      digitalWrite(CFG2, HIGH); //  -    0     0     1     1
      digitalWrite(CFG3, HIGH); //  -    0     1     1     0
  } else if (setVoltage == "20"){
                                //  5V   9V   12V   15V   20V  (Can also be changed on the fly)
      digitalWrite(CFG1, LOW);  //  1    0     0     0     0
      digitalWrite(CFG2, HIGH); //  -    0     0     1     1
      digitalWrite(CFG3, LOW);  //  -    0     1     1     0
  }

  stepper_driver.setRunCurrent(current.toInt());
  stepper_driver.setMicrostepsPerStep(microsteps.toInt());
  stepper_driver.setStallGuardThreshold(stallThreshold.toInt());

  if (standstillMode == "NORMAL"){ stepper_driver.setStandstillMode(stepper_driver.NORMAL);}
  else if (standstillMode == "FREEWHEELING"){ stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);}
  else if (standstillMode == "BRAKING"){ stepper_driver.setStandstillMode(stepper_driver.BRAKING);}
  else if (standstillMode == "STRONG_BRAKING"){ stepper_driver.setStandstillMode(stepper_driver.STRONG_BRAKING);}
}

//Read saved settings from EEPROM//
void readSettings(){ 

  preferences.begin("settings", false); //open the settings namespace

  enabled1 = preferences.getString("enable", ""); 
  
  if (enabled1 == ""){ //EEPROM has not been saved to before so save defaults
    preferences.end(); //close to write EEPROM can open again
    enabled1 = "enabled";
    setVoltage = "12";
    microsteps = "32";
    current = "30";
    stallThreshold = "10";
    standstillMode = "NORMAL";
    homeAngle = "0";
    writeSettings();
  } else {
    Serial.println("Settings found in EEPROM");
    setVoltage = preferences.getString("voltage", "");
    microsteps = preferences.getString("microsteps", "");
    current = preferences.getString("current", "");
    stallThreshold = preferences.getString("stallThreshold", "");
    standstillMode = preferences.getString("standstillMode", "");
    homeAngle = preferences.getString("homeAngle", "0");
    preferences.end();
  }

  // Keep the saved enable state but the actual driver.enable() is deferred
  // until the PG pin asserts in loop(), so the USB-PD negotiation always
  // completes before the motor draws current.
}



//save settings to flash
void writeSettings(){ //easiest (not best) is to write strings and then re-calculate values which use that

  preferences.begin("settings", false);

  preferences.putString("enable", enabled1);
  preferences.putString("voltage", setVoltage);
  preferences.putString("microsteps", microsteps);
  preferences.putString("current", current);
  preferences.putString("stallThreshold", stallThreshold);
  preferences.putString("standstillMode", standstillMode);
  preferences.putString("homeAngle", homeAngle);

  Serial.println("Saving settings to flash");

  preferences.end();
  
  configureSettings(); //use new settings
}
