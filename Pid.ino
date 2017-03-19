#include <Arduino.h>

#define MOVING_AVERAGE_INTERVALS 20
#define PID_INTERVAL_MS 100
#define PID_INTERVAL_S (PID_INTERVAL_MS / 1000.0)
#define PID_INTERVAL_HZ (1000.0 / PID_INTERVAL_MS)
#define THERM_INTERVAL 10
#define THERM_PIN 1
#define RELAY_PIN 0
#define WINDOW_SIZE 3001 // this is prime which gets us nice stuff
#define MIN_WATTS 0.0
#define MAX_WATTS 300.0
#define KP 1.0 // This has units of watts/C.  Watts per degree error in C
#define KI 1.0 // This has units of watts/C/s
#define KD 1.0 //This has units of watts/(C/s)
#define TARGET_TEMP 37.0 // body temp

// This is the internal state of our PID
float lastTemp;
float lastError = 0.0;
float accumulatedI = 0.0;

// Run a simple moving avg on temp to remove noise
int lastTempUpdate;
int lastPidUpdate;
int relayWindowStart;
float movingAvgTemp;
int relayOut = 0; // between [0, window size]

void setup() { 
  //Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(THERM_PIN, INPUT);

  lastPidUpdate = millis();
  lastTempUpdate = lastPidUpdate;
  relayWindowStart = lastPidUpdate;
  lastTemp = getTemp();
  movingAvgTemp = lastTemp;
} 

void loop(void) {
  int now = millis();
  float temp = getTemp();
  if (now - lastTempUpdate > THERM_INTERVAL) {
    lastTempUpdate = now;
    movingAvgTemp = (movingAvgTemp * (MOVING_AVERAGE_INTERVALS - 1) + temp) / MOVING_AVERAGE_INTERVALS;
  }

  if (now - lastPidUpdate > PID_INTERVAL_MS) {
    lastPidUpdate = now;
    // TODO: consider turning off the relay during the computation if it takes too long
    relayOut = updatePid(movingAvgTemp) * WINDOW_SIZE / MAX_WATTS; 
  }

  if (now - relayWindowStart >= WINDOW_SIZE) {
    relayWindowStart += WINDOW_SIZE;
  }

  if (now - relayWindowStart < relayOut) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }

}

// The output of the method is watts between 0 and 300W
float updatePid(float temp) {
  float error = TARGET_TEMP - temp; // unit of deg C
  float delta = lastTemp - temp; // unit of deg C

  float p = KP * error; // unit of W
  float normP = norm(p); // unit of W

  // We average lastError and error to use the trapezoidal approximation of the integral instead of the box one.
  accumulatedI += KI * (error + lastError) * 0.5 * PID_INTERVAL_S;
  float d = KD * delta * PID_INTERVAL_HZ; // unit of W

  // We don't want the accumulated integral term to be higher than what is needed to get up to the max output.
  // We don't need the integral to add more value if we are already at full blast.
  // This will reduce integral windup https://en.wikipedia.org/wiki/Integral_windup
  // This method is better at reducing windup than direct clamping at min and max.
  accumulatedI = min(accumulatedI, MAX_WATTS - normP);
  accumulatedI = max(accumulatedI, MIN_WATTS - normP);

  lastTemp = temp;
  lastError = error;
  return norm(p + accumulatedI + d);
}

float norm(float f) {
  return max(MIN_WATTS, min(MAX_WATTS, f));
}


float getTemp() {
  int val = analogRead(THERM_PIN);
  // convert to temp
  return 37;
}
