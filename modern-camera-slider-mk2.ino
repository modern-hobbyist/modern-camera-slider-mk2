#define BLYNK_PRINT Serial

#include "credentials.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <Button2.h>

#define MS1 27       // Pin 25 connected to MS1 pin
#define MS2 14       // Pin 26 connected to MS2 pin
#define EN_PIN           33 // Enable
#define DIR_PIN          12 // Direction
#define STEP_PIN         13 // Step
#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define MIN_ENDSTOP_PIN 25
#define MAX_ENDSTOP_PIN 26

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

#define STEPS_PER_REVOLUTION 12800

#define TRAVEL_SPEED 10000 
#define MAX_SPEED 10000
#define MAX_TRAVEL_TIME 1000000.0

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

constexpr uint32_t steps_per_revolution = 12800;

AccelStepper myStepper = AccelStepper(myStepper.DRIVER, STEP_PIN, DIR_PIN);

Button2 minEndStop, maxEndStop;

long travelTimeInSecs = 0;
int distancePercentage = 100;
int playSpeed = 0;
bool stopped = true;
bool play = false;
bool reverseDirection = false;
bool durationSet = false;
bool distanceSet = true;
bool sendHome = false;

BLYNK_CONNECTED() {
  Blynk.virtualWrite(V0, 0);
  Blynk.virtualWrite(V1, 0);
  Blynk.virtualWrite(V2, 0);
  Blynk.virtualWrite(V3, 0);
}

BLYNK_WRITE(V0) {
  if(param[0]){
    Serial.println("Homing Gantry");
    homeGantry();
  }
}

BLYNK_WRITE(V1){
  if(!durationSet || !distanceSet){
    Blynk.virtualWrite(V1, 0);
  }

  int value = param[0]; 
  
  if(param[0] == 0){
    pauseSlider();
  }else{
    playSlider();
  }
}

BLYNK_WRITE(V2) {
  travelTimeInSecs = param[0].asLong();
  int value = round((travelTimeInSecs / MAX_TRAVEL_TIME) * 100);

  Blynk.virtualWrite(V3, value);
  
  Blynk.virtualWrite(V2, travelTimeInSecs);
  
  if(travelTimeInSecs > 0){
    durationSet = true;
  }
  calculateSpeed();
}

void setup() {
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MIN_ENDSTOP_PIN, INPUT_PULLUP);
  pinMode(MAX_ENDSTOP_PIN, INPUT_PULLUP);

  digitalWrite(MS1, LOW);
  digitalWrite(MS2, HIGH);

  Blynk.setProperty(V2, "min", 2); 
  Blynk.setProperty(V2, "max", 5);
  Blynk.setProperty(V2, "color", "#ED9D00");
  Blynk.setProperty(V2, "label", "Slide to Select Message");
  
  Blynk.begin(auth, ssid, pass);
  
  Serial.begin(115200);
  
  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(600);        // Set motor RMS current
  driver.microsteps(64);          // Set microsteps to 1/16th

  driver.pwm_autoscale(true);     // Needed for stealthChop

  myStepper.setMaxSpeed(steps_per_revolution); // 100mm/s @ 80 steps/mm
  myStepper.setAcceleration(80000); // 2000mm/s^2
  myStepper.setEnablePin(EN_PIN);
  myStepper.setPinsInverted(false, false, true);
  myStepper.enableOutputs();

  minEndStop.begin(MIN_ENDSTOP_PIN);
  minEndStop.setDebounceTime(50);
  minEndStop.setTapHandler(endstopClick);
  minEndStop.setPressedHandler(endstopPressed);
  minEndStop.setReleasedHandler(endstopReleased);
  
  maxEndStop.begin(MAX_ENDSTOP_PIN);
  maxEndStop.setDebounceTime(50);
  maxEndStop.setTapHandler(endstopClick);
  maxEndStop.setPressedHandler(endstopPressed);
  maxEndStop.setReleasedHandler(endstopReleased);
}

void loop()
{
  minEndStop.loop();
  maxEndStop.loop();
  
  Blynk.run();
  
  if(sendHome){
    myStepper.setSpeed(-TRAVEL_SPEED);
    myStepper.runSpeed();
  } else if(play){
    myStepper.runSpeed();
  }
  
}

void homeGantry(){ 
  if(!minEndStop.isPressed()){
    stopped = false;
    sendHome = true;
  }else {
    Blynk.virtualWrite(V0, 0);
  }
}

void playSlider(){
  if(!stopped){
    calculateSpeed();
    Serial.println("Playing");
    Blynk.virtualWrite(V1, 1);

    play = true;
  } else {
    Blynk.virtualWrite(V1, 0);
    homeGantry();
  }
}

void calculateSpeed() {  
  Serial.println("Calculating speed");
  playSpeed = MAX_SPEED * (travelTimeInSecs / MAX_TRAVEL_TIME);

  if(reverseDirection){
    Serial.println("Right to left");
    playSpeed = -1 * playSpeed;
  }

  myStepper.setSpeed(playSpeed);
  Serial.print("Speed: ");
  Serial.println(playSpeed);
}

void pauseSlider(){
  play = false;
  Blynk.virtualWrite(V1, 0);
}

void endstopClick(Button2& btn) {
  Serial.println("Button clicked");
  endstopPressed(btn);
}

void endstopPressed(Button2& btn) {
  if (btn == minEndStop) {
    //any necessary minstop actions
    sendHome = false;
    Blynk.virtualWrite(V0, 0);
    reverseDirection = false;
  } else if (btn == maxEndStop) {
    reverseDirection = true;
  }
  calculateSpeed();
}

void endstopReleased(Button2& btn) {
  if (btn == minEndStop) {
    //any necessary minstop actions
  } else if (btn == maxEndStop) {
    //any necessary maxstop actions
  }
}
