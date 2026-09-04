// ==================================================
// LINE FOLLOWER ROBOT
// ==================================================


// =========================
// HC4067
// =========================

int s0 = A0;
int s1 = A1;
int s2 = A2;
int s3 = A3;
int sig = A4;


// =========================
// START SWITCH
// =========================

int start = A5;

// Switch activated = LOW
int startState = LOW;


// =========================
// TB6612FNG
// =========================

// Left motor
int pwma = 5;
int ain1 = 10;
int ain2 = 11;

// Right motor
int pwmb = 6;
int bin1 = 8;
int bin2 = 7;

// Standby
int stby = 9;


// =========================
// SENSOR VALUES
// =========================

int sensor[7];


// =========================
// SENSOR PROCESSED VALUES
// =========================

int value[7];


// =========================
// LINE POSITION
// =========================

long position = 0;

int error = 0;
int lastError = 0;


// =========================
// PID
// =========================

float kp = 0.05;
float ki = 0.00;
float kd = 0.10;

float integral = 0;
float derivative = 0;

float output = 0;


// =========================
// MOTOR SPEED
// =========================

int baseSpeed = 120;

int leftSpeed = 0;
int rightSpeed = 0;


// =========================
// ROBOT STATE
// =========================

bool started = false;


// ==================================================
// SETUP
// ==================================================

void setup() {

  Serial.begin(9600);


  // --------------------------
  // HC4067
  // --------------------------

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);

  pinMode(sig, INPUT);


  // --------------------------
  // START SWITCH
  // --------------------------

  pinMode(start, INPUT_PULLUP);


  // --------------------------
  // TB6612FNG
  // --------------------------

  pinMode(pwma, OUTPUT);
  pinMode(pwmb, OUTPUT);

  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);

  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);

  pinMode(stby, OUTPUT);


  // --------------------------
  // MOTORS OFF
  // --------------------------

  analogWrite(pwma, 0);
  analogWrite(pwmb, 0);

  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);

  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);

  digitalWrite(stby, LOW);


  Serial.println("ROBOT READY");
}


// ==================================================
// MAIN LOOP
// ==================================================

void loop() {

  // --------------------------
  // CHECK START BUTTON
  // --------------------------

  startButton();


  // --------------------------
  // RUN ROBOT
  // --------------------------

  if (started == true) {

    runRobot();
  }
}


// ==================================================
// START BUTTON
// ==================================================

void startButton() {

  // Check A5

  if (digitalRead(start) == startState && started == false) {

    Serial.println("START DETECTED");


    // --------------------------
    // 5 SECOND DELAY
    // --------------------------

    Serial.println("5 SECOND DELAY");

    delay(5000);


    // --------------------------
    // RESET PID
    // --------------------------

    integral = 0;
    lastError = 0;


    // --------------------------
    // START ROBOT
    // --------------------------

    started = true;

    Serial.println("ROBOT START");
  }
}


// ==================================================
// RUN ROBOT
// ==================================================

void runRobot() {

  // Read sensors

  readSensors();


  // Find line position

  getPosition();


  // Calculate PID

  pid();


  // Control motors

  motor();


  delay(5);
}


// ==================================================
// SELECT HC4067 CHANNEL
// ==================================================

void select(int ch) {

  digitalWrite(s0, ch & 1);

  digitalWrite(s1, (ch >> 1) & 1);

  digitalWrite(s2, (ch >> 2) & 1);

  digitalWrite(s3, (ch >> 3) & 1);
}


// ==================================================
// READ 7 SENSORS
// ==================================================

void readSensors() {

  for (int i = 0; i < 7; i++) {

    // C1 → Sensor 1
    // C2 → Sensor 2
    // C3 → Sensor 3
    // C4 → Sensor 4
    // C5 → Sensor 5
    // C6 → Sensor 6
    // C7 → Sensor 7

    select(i + 1);

    delayMicroseconds(5);

    sensor[i] = analogRead(sig);
  }
}


// ==================================================
// SENSOR CALIBRATION
// ==================================================
//
// Calibration is NOT being used yet.
//
// We will determine the proper sensor
// thresholds from actual physical testing.
//
// ==================================================


// ==================================================
// GET LINE POSITION
// ==================================================

void getPosition() {

  long total = 0;
  long sum = 0;


  for (int i = 0; i < 7; i++) {

    // --------------------------------
    // TEMPORARY SENSOR VALUE
    // --------------------------------
    //
    // We will replace this with the
    // actual threshold/mapping later.
    //
    // --------------------------------

    value[i] = sensor[i];


    // Sensor positions:
    //
    // S1 = 0
    // S2 = 1000
    // S3 = 2000
    // S4 = 3000
    // S5 = 4000
    // S6 = 5000
    // S7 = 6000

    sum += (long)value[i] * (i * 1000);

    total += value[i];
  }


  // Calculate position

  if (total > 0) {

    position = sum / total;
  }


  // Center = 3000

  error = position - 3000;
}


// ==================================================
// PID
// ==================================================

void pid() {

  // Integral

  integral += error;


  // Derivative

  derivative = error - lastError;


  // PID calculation

  output =
    (kp * error) +
    (ki * integral) +
    (kd * derivative);


  // Save previous error

  lastError = error;
}


// ==================================================
// MOTOR CONTROL
// ==================================================

void motor() {

  // Apply PID correction

  leftSpeed = baseSpeed + output;

  rightSpeed = baseSpeed - output;


  // --------------------------
  // LEFT MOTOR LIMIT
  // --------------------------

  if (leftSpeed > 255) {
    leftSpeed = 255;
  }

  if (leftSpeed < 0) {
    leftSpeed = 0;
  }


  // --------------------------
  // RIGHT MOTOR LIMIT
  // --------------------------

  if (rightSpeed > 255) {
    rightSpeed = 255;
  }

  if (rightSpeed < 0) {
    rightSpeed = 0;
  }


  // --------------------------
  // ENABLE TB6612
  // --------------------------

  digitalWrite(stby, HIGH);


  // --------------------------
  // LEFT MOTOR
  // --------------------------

  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);

  analogWrite(pwma, leftSpeed);


  // --------------------------
  // RIGHT MOTOR
  // --------------------------

  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);

  analogWrite(pwmb, rightSpeed);
}