#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <math.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>

// Objects
Adafruit_SSD1306 display(128,64, &Wire, -1);
Adafruit_MPU6050 mpu;


int switchScreen = 5;
int buttonUp = 6;
int buttonDown = 7;
bool prevPressed = false;
unsigned long debounceDelay = 50;
unsigned long prevTime = 0; 
int length = 1.0;
char pout[30];
unsigned long doublePressTime = 500;
char myNewText[50];
int yoffset = 1.0;
int SDA_PIN = 9;
int SCL_PIN = 8;

// Stride and step variables
int steps = 0;
float lastAccelMag = 0;
unsigned long lastStepTime = 0;

enum PressType {
  NoPress, // 0
  SinglePress, //1 
};

// initialize state machine
enum MachineState {
  stepsTaken,
  distanceTraveled,
  strideLength,
  rawAcceleration
};

volatile PressType currentPress = NoPress;
volatile MachineState currentState = stepsTaken;


void buttonPress() {
  unsigned long currentTime = millis();
  if (currentTime - prevTime > debounceDelay) {
      currentPress = SinglePress;
    }
    prevTime = currentTime;

  }

void setup() {
  delay(1000);
  Serial.begin(9600); 
  while (!Serial) {
    yield();
  } 
  pinMode(switchScreen, INPUT_PULLDOWN);
  attachInterrupt(switchScreen, buttonPress, RISING); 

  pinMode(buttonUp, INPUT_PULLDOWN);
  attachInterrupt(buttonUp, buttonPress, RISING); 

  pinMode(buttonDown, INPUT_PULLDOWN);
  attachInterrupt(buttonDown, buttonPress, RISING); 

  currentState = stepsTaken;

  Wire.begin(SDA_PIN, SCL_PIN);
  // Start LED screen
  while (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    Serial.println("Screen not starting...");
    delay(1000);
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");


  Serial.println("Setup complete!");
  
}

sensors_event_t event;

float getAccelMagnitude() {
  mpu.getAccelerometerSensor()->getEvent(&event);
  return sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
}

bool detectStep(float accelMag) {
  const float threshold = 1.2; // tune this
  unsigned long now = millis();
  if (accelMag > threshold && (now - lastStepTime) > 300) { // 300 ms debounce
    lastStepTime = now;
    return true;
  }
  return false;
}


void loop() {

    if(digitalRead(switchScreen) && !prevPressed) {
        currentState = MachineState(((int)currentState + 1) % 4);
        Serial.println(currentState);
        
    }
    prevPressed = digitalRead(switchScreen);

  // erase screen
  display.clearDisplay(); 
  // get imu data
  mpu.getAccelerometerSensor()->getEvent(&event);

  float accelMag = getAccelMagnitude();
  if (detectStep(accelMag)) steps++;


  switch (currentState)
  {
  case stepsTaken:
      //sprintf(pout, "Steps: %d", steps);
      display.setTextSize(1);             // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);        // Draw white text
      display.setCursor(2,2);             // Start at top-left corner
      display.print(F("Steps: %d"));
      display.println(steps);
      Serial.println("steps taken");
    break;
  case distanceTraveled: 

      sprintf(pout, "Distance: %.2f m", steps * strideLength);
      //myOLED.text(0, yoffset, pout);
      Serial.println("distance traveled");
    break;

  case strideLength: 
     if(digitalRead(buttonUp) && !prevPressed) {
        length = length+ 1;
        prevPressed = digitalRead(buttonUp);
    }

    else if(digitalRead(buttonDown)&& !prevPressed){
      length = length-1;
      prevPressed = digitalRead(buttonDown);
    }

    // display on screen
    sprintf(myNewText, "Set Stride Length: %.1f", length);
    //myOLED.erase();
    //myOLED.text(3, yoffset, myNewText);
    Serial.println("stride length");
    break;

  case rawAcceleration: 
    sprintf(pout, "ax: %.2f", event.acceleration.x);
   // myOLED.text(0,0, pout);
    sprintf(pout, "ay: %.2f", event.acceleration.y);
   // myOLED.text(0,10, pout);
    sprintf(pout, "az: %.2f", event.acceleration.z);
   // myOLED.text(0,20, pout);
    Serial.println("raw");
    break;

  default:
    break;

  display.display();
}
}

