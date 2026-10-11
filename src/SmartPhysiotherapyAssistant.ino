#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>

// ======================================================
// SMART PHYSIOTHERAPY ASSISTANT
// ======================================================

// Hardware pins
#define LED_PIN 8
#define BUZZER_PIN 9
#define BUTTON_PIN 2

// Angle thresholds
#define EXTENDED_ANGLE 20.0
#define FLEXED_ANGLE 70.0
#define UNSAFE_ANGLE 120.0
#define START_ANGLE 8.0
#define MOVEMENT_DELTA 2.0

// Session settings
#define SESSION_TIMEOUT 5000UL
#define DEBOUNCE_DELAY 40UL

// Memory settings
#define BUFFER_SIZE 10
#define MAX_SESSIONS 10
#define PATIENT_ID_SIZE 10

// EEPROM layout
#define EEPROM_MAGIC 0x4A51
#define EEPROM_MAGIC_ADDR 0
#define EEPROM_COUNT_ADDR 2
#define EEPROM_REPS_ADDR 4
#define EEPROM_SESSION_ADDR 8

// Bitwise flags
#define CALIBRATED_FLAG 0x01
#define UNSAFE_FLAG 0x02
#define ACTIVE_FLAG 0x04
#define REP_FLAG 0x08
#define GOOD_ROM_FLAG 0x10

// ======================================================
// HARDWARE OBJECTS
// ======================================================

Adafruit_MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ======================================================
// ENUMS
// ======================================================

enum BodyPart {
  ELBOW,
  KNEE
};

enum SessionState {
  IDLE,
  ACTIVE
};

enum RepState {
  EXTENDED,
  FLEXED
};

enum FeedbackType {
  NORMAL,
  GOOD,
  INCOMPLETE,
  UNSAFE
};

// ======================================================
// SESSION STRUCTURE
// ======================================================

struct Session {
  uint16_t id;
  uint8_t bodyPart;
  uint8_t flags;
  uint16_t reps;
  float minAngle;
  float maxAngle;
  float average;
  uint32_t duration;
  char patient[PATIENT_ID_SIZE];
};

// ======================================================
// GLOBAL VARIABLES
// ======================================================

BodyPart bodyPart = ELBOW;
SessionState sessionState = IDLE;
RepState repState = EXTENDED;
FeedbackType feedback = NORMAL;

Session currentSession;

char patientID[PATIENT_ID_SIZE] = "PATIENT1";

byte statusFlags = 0;

// Circular buffer
float angleBuffer[BUFFER_SIZE];

uint8_t bufferIndex = 0;
uint8_t bufferCount = 0;

// Angle and statistics
float currentAngle = 0.0;
float previousAngle = 0.0;
float angleSum = 0.0;

float minAngle = 180.0;
float maxAngle = 0.0;

uint32_t sampleCount = 0;

// Repetition and session history
uint16_t repetitions = 0;
uint32_t lifetimeReps = 0;

uint16_t sessionCount = 0;
uint16_t nextSessionID = 1;

// Automatic start can be enabled or disabled
bool automaticStart = false;
bool unsafeDetected = false;

// Timers
unsigned long sessionStart = 0;
unsigned long lastMovement = 0;
unsigned long lastLCD = 0;
unsigned long lastSerial = 0;
unsigned long lastDebounce = 0;

// Button state
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;

// ======================================================
// FUNCTION DECLARATIONS
// ======================================================

float calculateAngle(sensors_event_t &acceleration);

void updateBuffer(float value);
float recursiveMin(float arr[], int index, int count);
float recursiveMax(float arr[], int index, int count);

void startSession();
void finishSession();
void updateStatistics();
void updateRepetition();
void checkSafety();
void virtualPhysio();

void updateLCD();
void monitorSerial();
void checkButton();
void checkSerial();

void saveSession();
void loadSessions();
void readSession(uint8_t index, Session *s);

void printHistory();
void printCSV();
void sortSessions();
void searchSession(uint16_t id);
void printSession(Session *s);
void clearHistory();

void showHelp();
void shortBeep();
void setPatientID();
void selectBodyPart();

// ======================================================
// SETUP
// ======================================================

void setup() {

  Serial.begin(115200);
  Wire.begin();

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(F("Smart Physio"));

  lcd.setCursor(0, 1);
  lcd.print(F("Initializing"));

  Serial.println(F("Starting Smart Physiotherapy Assistant"));

  delay(500);

  // Initialize MPU6050
  if (!mpu.begin()) {

    Serial.println(F("ERROR: MPU6050 not found!"));

    lcd.clear();
    lcd.print(F("MPU6050 ERROR"));

    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println(F("MPU6050 connected successfully."));

  // Load previously saved history
  loadSessions();

  statusFlags = CALIBRATED_FLAG;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("System Ready"));

  lcd.setCursor(0, 1);
  lcd.print(F("Press Button"));

  Serial.print(F("Patient ID: "));
  Serial.println(patientID);

  Serial.println(F("Body part: ELBOW"));
  Serial.println(F("Angle calculation: X/Z acceleration"));
  Serial.println(F("Automatic start: OFF"));
  Serial.println(F("Press button to start or finish."));
  Serial.println(F("Type H for commands."));

  showHelp();
}

// ======================================================
// MAIN LOOP
// ======================================================

void loop() {

  checkSerial();
  checkButton();

  // Read MPU6050
  sensors_event_t acceleration;
  sensors_event_t gyro;
  sensors_event_t temperature;

  mpu.getEvent(
    &acceleration,
    &gyro,
    &temperature
  );

  previousAngle = currentAngle;

  currentAngle = calculateAngle(acceleration);

  // Store angle in circular buffer
  updateBuffer(currentAngle);

  // Optional automatic session start
  if (automaticStart && sessionState == IDLE) {

    if (currentAngle >= START_ANGLE) {
      startSession();
    }
  }

  // Process active session
  if (sessionState == ACTIVE) {

    updateStatistics();
    updateRepetition();
    checkSafety();
    virtualPhysio();

    // Detect movement
    if (fabs(currentAngle - previousAngle) >= MOVEMENT_DELTA) {
      lastMovement = millis();
    }

    // End session after inactivity
    if (millis() - lastMovement >= SESSION_TIMEOUT) {
      finishSession();
    }

  } else {

    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
  }

  // Update LCD periodically
  if (millis() - lastLCD >= 300) {

    updateLCD();
    lastLCD = millis();
  }

  // Serial output
  if (millis() - lastSerial >= 500) {

    monitorSerial();
    lastSerial = millis();
  }

  delay(50);
}

// ======================================================
// ANGLE CALCULATION
// ======================================================

float calculateAngle(sensors_event_t &acceleration) {

  float x = acceleration.acceleration.x;
  float z = acceleration.acceleration.z;

  float angle = atan2(x, z) * 180.0 / PI;

  if (angle < 0) {
    angle = -angle;
  }

  if (angle > 180) {
    angle = 180;
  }

  return angle;
}

// ======================================================
// CIRCULAR BUFFER
// ======================================================

void updateBuffer(float value) {

  angleBuffer[bufferIndex] = value;

  bufferIndex++;

  if (bufferIndex >= BUFFER_SIZE) {
    bufferIndex = 0;
  }

  if (bufferCount < BUFFER_SIZE) {
    bufferCount++;
  }
}

// ======================================================
// RECURSIVE MINIMUM
// ======================================================

float recursiveMin(float arr[], int index, int count) {

  if (index == count - 1) {
    return arr[index];
  }

  float next = recursiveMin(arr, index + 1, count);

  if (arr[index] < next) {
    return arr[index];
  }

  return next;
}

// ======================================================
// RECURSIVE MAXIMUM
// ======================================================

float recursiveMax(float arr[], int index, int count) {

  if (index == count - 1) {
    return arr[index];
  }

  float next = recursiveMax(arr, index + 1, count);

  if (arr[index] > next) {
    return arr[index];
  }

  return next;
}

// ======================================================
// START SESSION
// ======================================================

void startSession() {

  if (sessionState == ACTIVE) {
    return;
  }

  sessionState = ACTIVE;

  statusFlags |= ACTIVE_FLAG;

  repetitions = 0;
  repState = EXTENDED;

  feedback = NORMAL;
  unsafeDetected = false;

  angleSum = 0;
  sampleCount = 0;

  minAngle = 180.0;
  maxAngle = 0.0;

  bufferCount = 0;
  bufferIndex = 0;

  sessionStart = millis();
  lastMovement = millis();

  memset(&currentSession, 0, sizeof(currentSession));

  currentSession.id = nextSessionID;
  currentSession.bodyPart = bodyPart;

  strncpy(
    currentSession.patient,
    patientID,
    PATIENT_ID_SIZE - 1
  );

  currentSession.patient[PATIENT_ID_SIZE - 1] = '\0';

  Serial.println();
  Serial.println(F("===== SESSION STARTED ====="));

  Serial.print(F("Patient: "));
  Serial.println(patientID);

  Serial.print(F("Body part: "));

  if (bodyPart == ELBOW) {
    Serial.println(F("ELBOW"));
  } else {
    Serial.println(F("KNEE"));
  }

  lcd.clear();
  lcd.print(F("Session Started"));
}

// ======================================================
// SESSION STATISTICS
// ======================================================

void updateStatistics() {

  angleSum += currentAngle;
  sampleCount++;

  if (currentAngle < minAngle) {
    minAngle = currentAngle;
  }

  if (currentAngle > maxAngle) {
    maxAngle = currentAngle;
  }

  currentSession.reps = repetitions;
  currentSession.minAngle = minAngle;
  currentSession.maxAngle = maxAngle;

  currentSession.average =
      sampleCount > 0 ? angleSum / sampleCount : 0;

  currentSession.duration = millis() - sessionStart;
}

// ======================================================
// REPETITION COUNTING
// ======================================================

void updateRepetition() {

  if (repState == EXTENDED) {

    if (currentAngle >= FLEXED_ANGLE) {

      repState = FLEXED;

      Serial.println(F("Flexion detected."));
    }

  } else {

    if (currentAngle <= EXTENDED_ANGLE) {

      repetitions++;
      lifetimeReps++;

      statusFlags |= REP_FLAG;

      repState = EXTENDED;

      shortBeep();

      Serial.print(F("REP COMPLETED: "));
      Serial.println(repetitions);

      Serial.print(F("Lifetime repetitions: "));
      Serial.println(lifetimeReps);
    }
  }
}

// ======================================================
// SAFETY ALERT
// ======================================================

void checkSafety() {

  if (currentAngle >= UNSAFE_ANGLE) {

    statusFlags |= UNSAFE_FLAG;

    unsafeDetected = true;
    feedback = UNSAFE;

    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 1000);

  } else {

    statusFlags &= ~UNSAFE_FLAG;

    digitalWrite(LED_PIN, LOW);
    noTone(BUZZER_PIN);
  }
}

// ======================================================
// VIRTUAL PHYSIOTHERAPIST
// ======================================================

void virtualPhysio() {

  if (currentAngle >= UNSAFE_ANGLE) {

    feedback = UNSAFE;
    return;
  }

  if (currentAngle >= FLEXED_ANGLE) {

    feedback = GOOD;

    statusFlags |= GOOD_ROM_FLAG;

    return;
  }

  if (
    repState == FLEXED &&
    currentAngle > EXTENDED_ANGLE
  ) {

    feedback = INCOMPLETE;
    return;
  }

  feedback = NORMAL;
}

// ======================================================
// FINISH SESSION
// ======================================================

void finishSession() {

  if (sessionState != ACTIVE) {
    return;
  }

  sessionState = IDLE;

  statusFlags &= ~ACTIVE_FLAG;

  digitalWrite(LED_PIN, LOW);
  noTone(BUZZER_PIN);

  currentSession.id = nextSessionID;
  currentSession.bodyPart = bodyPart;
  currentSession.flags = statusFlags;
  currentSession.reps = repetitions;

  currentSession.minAngle =
      sampleCount > 0 ? minAngle : 0;

  currentSession.maxAngle =
      sampleCount > 0 ? maxAngle : 0;

  currentSession.average =
      sampleCount > 0 ? angleSum / sampleCount : 0;

  currentSession.duration =
      (millis() - sessionStart) / 1000UL;

  strncpy(
    currentSession.patient,
    patientID,
    PATIENT_ID_SIZE - 1
  );

  currentSession.patient[PATIENT_ID_SIZE - 1] = '\0';

  // Analyze recent movement
  if (bufferCount > 0) {

    float recentMin = recursiveMin(
      angleBuffer, 0, bufferCount
    );

    float recentMax = recursiveMax(
      angleBuffer, 0, bufferCount
    );

    Serial.println();
    Serial.println(F("Recent movement analysis:"));

    Serial.print(F("Recent minimum: "));
    Serial.println(recentMin, 1);

    Serial.print(F("Recent maximum: "));
    Serial.println(recentMax, 1);
  }

  // Save to EEPROM
  saveSession();

  Serial.println();
  Serial.println(F("===== SESSION COMPLETE ====="));

  printSession(&currentSession);

  Serial.println(F("Virtual Physiotherapist feedback:"));

  if (repetitions == 0) {

    Serial.println(F("No complete repetitions recorded."));

  } else if (
    maxAngle >= FLEXED_ANGLE &&
    maxAngle < UNSAFE_ANGLE
  ) {

    Serial.println(F("Good range reached during session."));

  } else {

    Serial.println(F("Review the movement range."));
  }

  if (unsafeDetected) {
    Serial.println(F("Warning: unsafe threshold was crossed."));
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SESSION DONE"));

  lcd.setCursor(0, 1);
  lcd.print(F("Reps: "));
  lcd.print(repetitions);

  delay(1200);

  // Reset for next session
  statusFlags = CALIBRATED_FLAG;

  repetitions = 0;

  angleSum = 0;
  sampleCount = 0;

  minAngle = 180.0;
  maxAngle = 0.0;

  bufferCount = 0;
  bufferIndex = 0;

  feedback = NORMAL;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("System Ready"));

  lcd.setCursor(0, 1);
  lcd.print(F("Press Button"));
}

// ======================================================
// LCD DISPLAY
// ======================================================

void updateLCD() {

  if (sessionState == IDLE) {

    lcd.setCursor(0, 0);
    lcd.print(F("Ready           "));

    lcd.setCursor(0, 1);
    lcd.print(F("Press Button    "));

    return;
  }

  lcd.setCursor(0, 0);

  lcd.print(F("A:"));
  lcd.print(currentAngle, 1);
  lcd.print((char)223);

  lcd.print(F(" R:"));
  lcd.print(repetitions);
  lcd.print(F("   "));

  lcd.setCursor(0, 1);

  if (feedback == UNSAFE) {
    lcd.print(F("UNSAFE!         "));
  } else if (feedback == GOOD) {
    lcd.print(F("GOOD MOVEMENT   "));
  } else if (feedback == INCOMPLETE) {
    lcd.print(F("KEEP MOVING     "));
  } else {
    lcd.print(F("SAFE            "));
  }
}

// ======================================================
// SERIAL MONITOR STATUS
// ======================================================

void monitorSerial() {

  Serial.print(F("Angle: "));
  Serial.print(currentAngle, 1);

  Serial.print(F(" | Reps: "));
  Serial.print(repetitions);

  Serial.print(F(" | State: "));

  if (repState == EXTENDED) {
    Serial.print(F("EXTENDED"));
  } else {
    Serial.print(F("FLEXED"));
  }

  Serial.print(F(" | Session: "));

  if (sessionState == ACTIVE) {
    Serial.print(F("ACTIVE"));
  } else {
    Serial.print(F("IDLE"));
  }

  Serial.print(F(" | Coach: "));

  if (feedback == GOOD) {
    Serial.println(F("GOOD"));
  } else if (feedback == INCOMPLETE) {
    Serial.println(F("KEEP MOVING"));
  } else if (feedback == UNSAFE) {
    Serial.println(F("STOP"));
  } else {
    Serial.println(F("NORMAL"));
  }
}

// ======================================================
// BUTTON CONTROL
// ======================================================

void checkButton() {

  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounce = millis();
  }

  if (millis() - lastDebounce >= DEBOUNCE_DELAY) {

    if (reading != stableButtonState) {

      stableButtonState = reading;

      if (stableButtonState == LOW) {

        if (sessionState == IDLE) {
          startSession();
        } else {
          finishSession();
        }
      }
    }
  }

  lastButtonReading = reading;
}

// ======================================================
// BUZZER FEEDBACK
// ======================================================

void shortBeep() {
  tone(BUZZER_PIN, 1500, 80);
}

// ======================================================
// SAVE SESSION TO EEPROM
// ======================================================

void saveSession() {

  // Keep only the newest MAX_SESSIONS records
  if (sessionCount >= MAX_SESSIONS) {

    for (uint8_t i = 1; i < MAX_SESSIONS; i++) {

      Session temp;

      int sourceAddress =
          EEPROM_SESSION_ADDR + i * sizeof(Session);

      int destinationAddress =
          EEPROM_SESSION_ADDR + (i - 1) * sizeof(Session);

      EEPROM.get(sourceAddress, temp);
      EEPROM.put(destinationAddress, temp);
    }

    sessionCount = MAX_SESSIONS - 1;
  }

  int address =
      EEPROM_SESSION_ADDR + sessionCount * sizeof(Session);

  EEPROM.put(address, currentSession);

  sessionCount++;

  nextSessionID = currentSession.id + 1;

  EEPROM.put(
    EEPROM_MAGIC_ADDR,
    (uint16_t)EEPROM_MAGIC
  );

  EEPROM.put(EEPROM_COUNT_ADDR, sessionCount);
  EEPROM.put(EEPROM_REPS_ADDR, lifetimeReps);

  Serial.println(F("Session saved to EEPROM."));
}

// ======================================================
// LOAD SESSION HISTORY
// ======================================================

void loadSessions() {

  uint16_t magic = 0;

  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  if (magic != EEPROM_MAGIC) {

    sessionCount = 0;
    lifetimeReps = 0;
    nextSessionID = 1;

    EEPROM.put(
      EEPROM_MAGIC_ADDR,
      (uint16_t)EEPROM_MAGIC
    );

    EEPROM.put(
      EEPROM_COUNT_ADDR,
      (uint16_t)0
    );

    EEPROM.put(
      EEPROM_REPS_ADDR,
      (uint32_t)0
    );

    Serial.println(F("EEPROM initialized."));
    return;
  }

  EEPROM.get(EEPROM_COUNT_ADDR, sessionCount);
  EEPROM.get(EEPROM_REPS_ADDR, lifetimeReps);

  if (sessionCount > MAX_SESSIONS) {

    sessionCount = 0;
    lifetimeReps = 0;
    nextSessionID = 1;

    EEPROM.put(EEPROM_COUNT_ADDR, sessionCount);
    EEPROM.put(EEPROM_REPS_ADDR, lifetimeReps);

    Serial.println(F("Invalid history reset."));
    return;
  }

  nextSessionID = 1;

  if (sessionCount > 0) {

    Session last;

    readSession(sessionCount - 1, &last);

    nextSessionID = last.id + 1;
  }

  Serial.print(F("Stored sessions: "));
  Serial.println(sessionCount);

  Serial.print(F("Lifetime repetitions: "));
  Serial.println(lifetimeReps);
}

// ======================================================
// READ SESSION FROM EEPROM
// ======================================================

void readSession(uint8_t index, Session *s) {

  if (index >= sessionCount || index >= MAX_SESSIONS) {
    return;
  }

  int address =
      EEPROM_SESSION_ADDR + index * sizeof(Session);

  EEPROM.get(address, *s);
}

// ======================================================
// PRINT A SESSION
// ======================================================

void printSession(Session *s) {

  Serial.print(F("Session ID: "));
  Serial.println(s->id);

  Serial.print(F("Patient: "));
  Serial.println(s->patient);

  Serial.print(F("Body part: "));

  if (s->bodyPart == ELBOW) {
    Serial.println(F("ELBOW"));
  } else {
    Serial.println(F("KNEE"));
  }

  Serial.print(F("Repetitions: "));
  Serial.println(s->reps);

  Serial.print(F("Minimum angle: "));
  Serial.println(s->minAngle, 1);

  Serial.print(F("Maximum angle: "));
  Serial.println(s->maxAngle, 1);

  Serial.print(F("Average angle: "));
  Serial.println(s->average, 1);

  Serial.print(F("Duration: "));
  Serial.print(s->duration);
  Serial.println(F(" seconds"));

  Serial.print(F("Unsafe movement detected: "));
  Serial.println(s->flags & UNSAFE_FLAG ? F("YES") : F("NO"));

  Serial.println(F("-------------------------"));
}

// ======================================================
// VIEW HISTORY
// ======================================================

void printHistory() {

  Serial.println();
  Serial.println(F("===== SESSION HISTORY ====="));

  if (sessionCount == 0) {
    Serial.println(F("No saved sessions."));
    return;
  }

  for (uint8_t i = 0; i < sessionCount; i++) {

    Session s;

    readSession(i, &s);
    printSession(&s);
  }
}

// ======================================================
// CSV EXPORT
// ======================================================

void printCSV() {

  Serial.println(
    F("ID,PATIENT,BODY,REPS,MIN,MAX,AVERAGE,DURATION,UNSAFE")
  );

  for (uint8_t i = 0; i < sessionCount; i++) {

    Session s;

    readSession(i, &s);

    Serial.print(s.id);
    Serial.print(',');

    Serial.print(s.patient);
    Serial.print(',');

    Serial.print(s.bodyPart == ELBOW ? F("ELBOW") : F("KNEE"));
    Serial.print(',');

    Serial.print(s.reps);
    Serial.print(',');

    Serial.print(s.minAngle, 1);
    Serial.print(',');

    Serial.print(s.maxAngle, 1);
    Serial.print(',');

    Serial.print(s.average, 1);
    Serial.print(',');

    Serial.print(s.duration);
    Serial.print(',');

    Serial.println(
      s.flags & UNSAFE_FLAG ? 1 : 0
    );
  }
}

// ======================================================
// SORT HISTORY BY MAXIMUM ANGLE
// ======================================================

void sortSessions() {

  if (sessionCount < 2) {
    Serial.println(F("Not enough sessions to sort."));
    return;
  }

  // Use a small local array only during sorting
  Session data[MAX_SESSIONS];

  for (uint8_t i = 0; i < sessionCount; i++) {
    readSession(i, &data[i]);
  }

  // Bubble sort in descending order
  for (uint8_t i = 0; i < sessionCount; i++) {

    for (
      uint8_t j = 0;
      j + 1 < sessionCount - i;
      j++
    ) {

      if (data[j].maxAngle < data[j + 1].maxAngle) {

        Session temp = data[j];
        data[j] = data[j + 1];
        data[j + 1] = temp;
      }
    }
  }

  Serial.println(F("Sorted by maximum angle:"));

  for (uint8_t i = 0; i < sessionCount; i++) {
    printSession(&data[i]);
  }
}

// ======================================================
// SEARCH BY SESSION ID
// ======================================================

void searchSession(uint16_t id) {

  for (uint8_t i = 0; i < sessionCount; i++) {

    Session s;

    readSession(i, &s);

    if (s.id == id) {

      Serial.println(F("Session found."));
      printSession(&s);

      return;
    }
  }

  Serial.println(F("Session not found."));
}

// ======================================================
// CLEAR SESSION HISTORY
// ======================================================

void clearHistory() {

  sessionCount = 0;
  lifetimeReps = 0;
  nextSessionID = 1;

  EEPROM.put(EEPROM_MAGIC_ADDR, (uint16_t)0);
  EEPROM.put(EEPROM_COUNT_ADDR, (uint16_t)0);
  EEPROM.put(EEPROM_REPS_ADDR, (uint32_t)0);

  Serial.println(F("Session history cleared."));
}

// ======================================================
// CHANGE PATIENT ID
// ======================================================

void setPatientID() {

  Serial.println(F("Enter patient ID (max 9 characters):"));

  char newID[PATIENT_ID_SIZE];
  uint8_t index = 0;

  unsigned long startWait = millis();

  while (millis() - startWait < 15000UL) {

    if (Serial.available()) {

      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        break;
      }

      if (index < PATIENT_ID_SIZE - 1) {
        newID[index++] = c;
      }
    }
  }

  newID[index] = '\0';

  if (index > 0) {

    strncpy(patientID, newID, PATIENT_ID_SIZE - 1);
    patientID[PATIENT_ID_SIZE - 1] = '\0';

    Serial.print(F("Patient ID updated: "));
    Serial.println(patientID);

  } else {

    Serial.println(F("No ID entered."));
  }
}

// ======================================================
// SELECT BODY PART
// ======================================================

void selectBodyPart() {

  Serial.println(F("Select body part:"));
  Serial.println(F("1 - Elbow"));
  Serial.println(F("2 - Knee"));
  Serial.println(F("Enter selection:"));

  unsigned long startWait = millis();

  while (
    Serial.available() == 0 &&
    millis() - startWait < 10000UL
  ) {
    delay(10);
  }

  if (Serial.available() == 0) {
    Serial.println(F("Selection timed out."));
    return;
  }

  char selection = Serial.read();

  if (selection == '1') {

    bodyPart = ELBOW;
    Serial.println(F("Body part set to ELBOW."));

  } else if (selection == '2') {

    bodyPart = KNEE;
    Serial.println(F("Body part set to KNEE."));

  } else {

    Serial.println(F("Invalid selection."));
  }
}

// ======================================================
// SERIAL COMMANDS
// ======================================================

void checkSerial() {

  if (Serial.available() == 0) {
    return;
  }

  char command = Serial.read();

  if (command >= 'a' && command <= 'z') {
    command -= 32;
  }

  switch (command) {

    case 'H':
      showHelp();
      break;

    case 'V':
      printHistory();
      break;

    case 'C':
      printCSV();
      break;

    case 'S':
      sortSessions();
      break;

    case 'F': {

      Serial.println(F("Enter session ID:"));

      unsigned long startWait = millis();

      while (
        Serial.available() == 0 &&
        millis() - startWait < 10000UL
      ) {
        delay(10);
      }

      if (Serial.available() > 0) {
        uint16_t id = Serial.parseInt();
        searchSession(id);
      } else {
        Serial.println(F("Search timed out."));
      }

      break;
    }

    case 'X':
      clearHistory();
      break;

    case 'P':
      setPatientID();
      break;

    case 'B':
      selectBodyPart();
      break;

    case 'A':

      automaticStart = !automaticStart;

      Serial.print(F("Automatic start: "));
      Serial.println(
        automaticStart ? F("ON") : F("OFF")
      );

      break;

    default:
      break;
  }
}

// ======================================================
// HELP MENU
// ======================================================

void showHelp() {

  Serial.println();
  Serial.println(F("===== SMART PHYSIO COMMANDS ====="));

  Serial.println(F("H - Show help"));
  Serial.println(F("V - View session history"));
  Serial.println(F("C - Export CSV"));
  Serial.println(F("S - Sort sessions by maximum angle"));
  Serial.println(F("F - Find session by ID"));
  Serial.println(F("X - Clear session history"));
  Serial.println(F("P - Change patient ID"));
  Serial.println(F("B - Select body part"));
  Serial.println(F("A - Toggle automatic session start"));

  Serial.println();
  Serial.println(F("Button: Start or finish a session."));
  Serial.println(F("Automatic start is OFF by default."));
  Serial.println(F("Movement is sensor driven."));
  Serial.println(F("No manual repetition input required."));
  Serial.println(F("================================="));
}