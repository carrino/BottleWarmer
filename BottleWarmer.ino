#include <Arduino.h>

// Define pins for 2 types of controllers
#if defined(ARDUINO_AVR_TRINKET3) || defined(ARDUINO_AVR_TRINKET5)
#define THERM_PIN 1 // this is analog 1 which is digial 2
#define RELAY_PIN 0
#define BUTTON_PIN 3 // This is a shared usb pin, but our button is open line until pressed
#define LED_PIN 1 // LED pin required PWM
#else
#define THERM_PIN A1
#define RELAY_PIN 9
#define BUTTON_PIN 10
#define LED_PIN 11
#endif

// Software constants for timing.
#define MOVING_AVERAGE_INTERVALS 32 // This should be a power of 2 for perf
#define PID_INTERVAL_MS 101 // prime
#define PID_INTERVAL_S (PID_INTERVAL_MS / 1000.0)
#define PID_INTERVAL_HZ (1000.0 / PID_INTERVAL_MS)
#define THERM_INTERVAL 11 // prime
#define WINDOW_SIZE 2999 // this is prime which gets us nice stuff
#define TARGET_TEMP 37.0 // body temp
#define RUN_TIME (15L*60*1000) // 15 min
#define LED_PWM_OUT 128

// Define Hardware contstants
#define RELAY_ON LOW  // our relay is backwards it seems
#define RELAY_OFF HIGH
#define MIN_WATTS 0.0
#define MAX_WATTS 300.0
#define KNOWN_RESISTOR 10000.0
#define THERMISTOR_BETA 3950.0
#define THERMISTOR_ROOM_TEMP_K 298.15
#define THERMISTOR_ROOM_RESIST 10000.0

// In testing we found that we can go full blast to about 10 degrees out from the target.
// We want to avoid overshoot.
// This means that KP should be about 1/10 of max watts to start backing off at 10degC out.
#define KP (MAX_WATTS / 10.0)  // This has units of watts/C.  Watts per degree error in C

// In testing we found that a rate of .3 degrees C per second was full blast with about 240cc of water
// We also found that a cutoff at full blast may still rise 5 degrees.
// To be on the safe side we want to full cut off heat at 5 degrees out when rising at 3 sec per C
// At 5C error we will be running at half power so we should cut 150W per .3C/s
#define KD (KP * 5.0 * 3.0) //This has units of watts/(C/s) or W * (s/C)

float lastTemp;
bool waitForButton = true;
long lastButtonPress;
int lastTempUpdate;
int lastPidUpdate;
int relayWindowStart;
float movingAvgTemp; // Run a simple exponential moving avg on temp to remove noise
int relayWindowOut = 0; // between [0, window size] used to control the relay

void setup() { 
  //Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  lastButtonPress = millis();
  lastPidUpdate = lastButtonPress;
  lastTempUpdate = lastButtonPress;
  relayWindowStart = lastButtonPress;
  lastButtonPress = lastButtonPress;
  lastTemp = getTemp();
  movingAvgTemp = lastTemp;
} 

void loop(void) {
  long now = millis();
  float temp = getTemp();
  if (now - lastTempUpdate > THERM_INTERVAL) {
    lastTempUpdate = now;
    // This is a modified moving average (MMA): https://en.wikipedia.org/wiki/Moving_average#Modified_moving_average
    // basically a exponential moving average with alpha set to 1/MOVING_AVERAGE_INTERVALS
    movingAvgTemp = (movingAvgTemp * (MOVING_AVERAGE_INTERVALS - 1) + temp) / MOVING_AVERAGE_INTERVALS;
    //Serial.print("temp time "); Serial.println(millis() - now);
    //Serial.print("temp: "); Serial.println(movingAvgTemp);
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    lastButtonPress = now;
    waitForButton = false;
    analogWrite(LED_PIN, LED_PWM_OUT);
  }

  if (now - lastButtonPress > RUN_TIME) {
    waitForButton = true;
  }

  if (waitForButton) {
    digitalWrite(RELAY_PIN, RELAY_OFF);
    digitalWrite(LED_PIN, LOW);
    return;
  }

  if (now - relayWindowStart >= WINDOW_SIZE) {
    relayWindowStart += WINDOW_SIZE;
  }

  if (now - lastPidUpdate > PID_INTERVAL_MS) {
    lastPidUpdate = now;
    relayWindowOut = updatePid(movingAvgTemp) * 10; 
    //Serial.print("window out "); Serial.println(relayWindowOut);
    //Serial.print("temp: "); Serial.println(movingAvgTemp);
  }

  if (now - relayWindowStart < relayWindowOut) {
    digitalWrite(RELAY_PIN, RELAY_ON);
  } else {
    digitalWrite(RELAY_PIN, RELAY_OFF);
  }
  //Serial.print("loop time "); Serial.println(millis() - now);
}

// The output of the method is watts between min and max
float updatePid(float temp) {
  float error = TARGET_TEMP - temp; // unit of deg C
  float delta = lastTemp - temp; // unit of deg C

  float p = KP * error; // unit of W
  float normP = norm(p); // unit of W

  float d = KD * delta * PID_INTERVAL_HZ; // unit of W
  if (d > 0) {
    // We only want our d term to slow us down when heating.  
    // It can be too unstable when temps fall due to amplification of small changes
    d = 0;
  }

  lastTemp = temp;
  return norm(p + d);
}

float norm(float f) {
  return max(MIN_WATTS, min(MAX_WATTS, f));
}

// return temp in C
float getTemp() {
  int val = analogRead(THERM_PIN);
  if (val == 0) {
    // Our math breaks down here.
    return 0;
  }

  if (val == 1023) {
    // Our math breaks down here.
    return 100;
  }

  // https://learn.adafruit.com/thermistor/using-a-thermistor
  float thermResistance = KNOWN_RESISTOR / (1023.0/val - 1);

  // https://en.wikipedia.org/wiki/Thermistor#B_or_.CE.B2_parameter_equation
  //  1/T = 1/T0 + 1/B * ln(R / R0)
  // We set R0 = 10K, T0 = 298.15 degK, B = 3950
  float inverseKelvin =  1 / THERMISTOR_ROOM_TEMP_K + log(thermResistance / THERMISTOR_ROOM_RESIST) / THERMISTOR_BETA;
  return (1.0 / inverseKelvin) - 273.15;
}
