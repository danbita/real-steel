// uno_servo.ino - Atom servo bring-up firmware for an Uno R3 (Elegoo or Arduino).
//
// Lets the computer move PWM hobby servos (e.g. the DS51 neck servos) over USB.
// Pair with hardware/servo_test.py.
//
// WIRING
//   servo signal (yellow/orange/white) -> pin 9 (ch 0), pin 10 (ch 1)
//   servo + (red)                      -> EXTERNAL supply + (voltage on the servo's label)
//   servo - (brown/black)              -> EXTERNAL supply -  AND  an Uno GND pin
//   Do NOT power the servos from the Uno's 5V pin: a stalled servo pulls amps,
//   browns out the Uno and it resets mid-move. Grounds MUST be shared.
//
// PROTOCOL  (115200 baud, one command per line, replies one line)
//   ?            -> "ATOM-SERVO ready ch=2 travel=180"
//   M <ch> <deg> -> move channel to <deg> (0..TRAVEL_DEG); servo eases there
//   OFF [ch]     -> stop pulsing (servo goes limp); no ch = all
// Replies start with OK or ERR.

#include <Servo.h>

const int   N_CH         = 2;
const int   PINS[N_CH]   = {9, 10};
const float TRAVEL_DEG   = 180.0;   // set 270 if your servo is the 270-degree variant
const int   US_MIN       = 500;     // pulse at 0 deg
const int   US_MAX       = 2500;    // pulse at TRAVEL_DEG
const float RATE_DEG_S   = 90.0;    // easing speed - gentle for first tests
const unsigned long TICK_MS = 20;

Servo servo[N_CH];
float curUs[N_CH];
float tgtUs[N_CH];
bool  live[N_CH];

char  buf[32];
byte  len = 0;
unsigned long lastTick = 0;

float degToUs(float d) {
  return US_MIN + (US_MAX - US_MIN) * (d / TRAVEL_DEG);
}

void banner() {
  Serial.print(F("ATOM-SERVO ready ch="));
  Serial.print(N_CH);
  Serial.print(F(" travel="));
  Serial.println((int)TRAVEL_DEG);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < N_CH; i++) {
    curUs[i] = tgtUs[i] = degToUs(TRAVEL_DEG / 2);
    live[i] = false;            // not attached until first command: no jump at power-up
  }
  banner();
}

void handle(char *line) {
  char *cmd = strtok(line, " ");
  if (!cmd) return;

  if (strcmp(cmd, "?") == 0) { banner(); return; }

  if (strcmp(cmd, "M") == 0) {
    char *a = strtok(NULL, " ");
    char *b = strtok(NULL, " ");
    if (!a || !b) { Serial.println(F("ERR usage: M <ch> <deg>")); return; }
    int ch = atoi(a);
    float deg = atof(b);
    if (ch < 0 || ch >= N_CH)           { Serial.println(F("ERR bad channel")); return; }
    if (deg < 0 || deg > TRAVEL_DEG)    { Serial.println(F("ERR angle out of range")); return; }
    tgtUs[ch] = degToUs(deg);
    if (!live[ch]) {
      // First command: jump straight to target (we don't know where the horn is).
      curUs[ch] = tgtUs[ch];
      servo[ch].writeMicroseconds((int)curUs[ch]);
      servo[ch].attach(PINS[ch], US_MIN, US_MAX);
      live[ch] = true;
    }
    Serial.print(F("OK M "));
    Serial.print(ch);
    Serial.print(' ');
    Serial.print(deg, 1);
    Serial.print(F(" us="));
    Serial.println((int)tgtUs[ch]);
    return;
  }

  if (strcmp(cmd, "OFF") == 0) {
    char *a = strtok(NULL, " ");
    for (int i = 0; i < N_CH; i++) {
      if (a && atoi(a) != i) continue;
      servo[i].detach();
      live[i] = false;
    }
    Serial.println(F("OK OFF"));
    return;
  }

  Serial.println(F("ERR unknown command"));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = 0;
      handle(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }

  unsigned long now = millis();
  if (now - lastTick >= TICK_MS) {
    lastTick = now;
    float step = RATE_DEG_S * (TICK_MS / 1000.0) * (US_MAX - US_MIN) / TRAVEL_DEG;
    for (int i = 0; i < N_CH; i++) {
      if (!live[i]) continue;
      float d = tgtUs[i] - curUs[i];
      if (d > step) d = step;
      if (d < -step) d = -step;
      curUs[i] += d;
      servo[i].writeMicroseconds((int)curUs[i]);
    }
  }
}
