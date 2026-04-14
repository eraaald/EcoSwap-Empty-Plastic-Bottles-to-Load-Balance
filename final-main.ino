#include <Servo.h>
#include <HX711.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <math.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#define gsmSerial Serial

const char adminPhone[] = "+639762851032"; // Change to Admin's number

// ===== LCD I2C =====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===== Keypad =====
const byte ROWS = 4, COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {A0, A1, A2, A3};
byte colPins[COLS] = {10, 11, 12, 13};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Pins
const int TOP_SERVO_PIN = 9;
const int BOTTOM_SERVO_PIN = 3;   
const int TRIG_PIN  = 6;
const int ECHO_PIN  = 7;
const int CAP_PIN   = 8;
const int HX_DT_PIN  = 4;
const int HX_SCK_PIN = 5;

// Servo angles (calibrate)
int TOP_OPEN = 90;
int TOP_CLOSE = 0;

int BOT_OPEN = 90;
int BOT_CLOSE = 0;

Servo topServo;
Servo bottomServo;
HX711 scale;
float CAL = -104.72583333333f;

// --- Tuning values (adjust after testing) ---
const float PRESENT_CM = 12.0f;
const unsigned long CLOSE_DELAY_MS = 120;
const unsigned long SETTLE_MS = 1200;

// --- Configurable thresholds ---
const float DETECT_DISTANCE_CM = 15.0f;   // Object detection distance (cm)
const unsigned long STABILIZE_DELAY_MS = 1200; // Delay to stabilize weight

// Weight ranges for accepted bottles (adjust after testing)
const float MIN_500ML_G = 10.0f;   // lower bound for empty 500ml bottle
const float MAX_500ML_G = 30.0f;   // upper bound for empty 500ml bottle
const float MIN_2L_G    = 40.0f;   // lower bound for empty 2L bottle
const float MAX_2L_G    = 70.0f;   // upper bound for empty 2L bottle

// ===== USER LOGIN =====
char userPhone[12] = {0};   // 11 digits + null
uint8_t phoneLen = 0;
bool userLoggedIn = false;

uint32_t userAccepted = 0;
uint32_t userRejected = 0;
uint32_t userPoints   = 0;

// ===== Bin state machine =====
enum BinState {
  ST_LOGIN,
  ST_IDLE,
  ST_ENTRY,
  ST_INSPECT,
  ST_RELEASE,
  ST_SETTLE,
  ST_WEIGH,
  ST_RESET,
  ST_REDEEM
};
BinState st = ST_LOGIN;

unsigned long tState = 0;
float wBefore = 0;
bool plastic = false;
float lastDelta = 0;

uint8_t pendingSaves = 0;

// ===== EEPROM user storage =====
const int MAX_USERS = 40;
const uint8_t REC_VALID = 0xA6; // NEW marker (layout includes points)

struct UserRecord {
  char phone[12];           // "09xxxxxxxxx"
  uint32_t accepted;
  uint32_t rejected;
  uint32_t points;
  uint8_t valid;            // 0xA6 means used
};

int recordAddr(int idx) { return idx * sizeof(UserRecord); }

bool phoneEquals(const char* a, const char* b) {
  for (int i = 0; i < 12; i++) {
    if (a[i] != b[i]) return false;
    if (a[i] == '\0') return true;
  }
  return true;
}

int findEmptyIndex() {
  UserRecord rec;
  for (int i = 0; i < MAX_USERS; i++) {
    EEPROM.get(recordAddr(i), rec);
    if (rec.valid != REC_VALID) return i;
  }
  return -1;
}

int findUserIndex(const char* phone) {
  UserRecord rec;
  for (int i = 0; i < MAX_USERS; i++) {
    EEPROM.get(recordAddr(i), rec);
    if (rec.valid == REC_VALID && phoneEquals(rec.phone, phone)) return i;
  }
  return -1;
}

int currentUserIndex = -1;

void loadOrCreateUser(const char* phone) {
  int idx = findUserIndex(phone);
  if (idx >= 0) {
    UserRecord rec;
    EEPROM.get(recordAddr(idx), rec);
    userAccepted = rec.accepted;
    userRejected = rec.rejected;
    userPoints   = rec.points;
    currentUserIndex = idx;
  } else {
    int empty = findEmptyIndex();
    if (empty >= 0) {
      UserRecord rec = {};
      for (int i = 0; i < 12; i++) rec.phone[i] = phone[i];
      rec.accepted = 0;
      rec.rejected = 0;
      rec.points   = 0;
      rec.valid    = REC_VALID;
      EEPROM.put(recordAddr(empty), rec);

      userAccepted = 0;
      userRejected = 0;
      userPoints   = 0;
      currentUserIndex = empty;
    } else {
      // EEPROM full
      currentUserIndex = -1;
      userAccepted = userRejected = userPoints = 0;
    }
  }
}

void saveCurrentUser() {
  if (currentUserIndex < 0) return;
  UserRecord rec;
  EEPROM.get(recordAddr(currentUserIndex), rec);
  rec.accepted = userAccepted;
  rec.rejected = userRejected;
  rec.points   = userPoints;
  rec.valid    = REC_VALID;
  EEPROM.put(recordAddr(currentUserIndex), rec);
}

// ---------- Helpers ----------
void topOpen() {
  topServo.attach(TOP_SERVO_PIN);
  topServo.write(TOP_OPEN);
  delay(300);
  topServo.detach();
}

void topClose() {
  topServo.attach(TOP_SERVO_PIN);
  topServo.write(TOP_CLOSE);
  delay(300);
  topServo.detach();
}

void bottomOpen() {
  bottomServo.attach(BOTTOM_SERVO_PIN);
  bottomServo.write(BOT_OPEN);
  delay(300);
  bottomServo.detach();
}

void bottomClose() {
  bottomServo.attach(BOTTOM_SERVO_PIN);
  bottomServo.write(BOT_CLOSE);
  delay(300);
  bottomServo.detach();
}

void setState(BinState s) { st = s; tState = millis(); }

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long us = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (us == 0) return -1;
  return us * 0.0343f * 0.5f;
}

bool capDetected() { 
  int count = 0; 
  for (int i = 0; i < 20; i++) { 
    if (digitalRead(CAP_PIN) == HIGH) count++; // or LOW depending on sensor type 
    delay(5); 
    } 
    return (count > 15); 
  }

float weightG(uint8_t samples = 15) {
  if (!scale.is_ready()) return NAN;
  return scale.get_units(samples);
}

const char* stateName(BinState s) {
  switch (s) {
    case ST_LOGIN:   return "LOGIN";
    case ST_IDLE:    return "IDLE";
    case ST_ENTRY:   return "ENTRY";
    case ST_SETTLE:  return "SETTLE";
    case ST_WEIGH:   return "WEIGH";
    case ST_RESET:   return "RESET";
    case ST_REDEEM:  return "REDEEM";
    default:         return "UNK";
  }
}

// ===== LCD marquee (scroll) for BOTH rows =====
String scrollMsg[2] = {"", ""};
bool scrollActive[2] = {false, false};
uint16_t scrollPos[2] = {0, 0};
unsigned long scrollLastMs[2] = {0, 0};
const unsigned long SCROLL_INTERVAL_MS = 250;

void lcdStartScroll(uint8_t row, const String &msg) {
  if (row > 1) return;
  if (scrollMsg[row] == msg) return; // don't reset scroll unnecessarily

  scrollMsg[row] = msg;
  scrollPos[row] = 0;
  scrollLastMs[row] = 0;
  scrollActive[row] = (msg.length() > 16);

  if (!scrollActive[row]) {
    lcd.setCursor(0, row);
    lcd.print(msg);
    for (int i = msg.length(); i < 16; i++) lcd.print(' ');
  }
}

void lcdScrollTick() {
  for (uint8_t row = 0; row < 2; row++) {
    if (!scrollActive[row]) continue;
    
    // Smooth timing check
    if (millis() - scrollLastMs[row] < SCROLL_INTERVAL_MS) continue;
    scrollLastMs[row] = millis();

    String displayMsg = scrollMsg[row] + "    "; 
    lcd.setCursor(0, row);
    
    // Fix: Print as a single chunk if possible or just ensure no clearing happens
    for (int i = 0; i < 16; i++) {
      int charIdx = (scrollPos[row] + i) % displayMsg.length();
      lcd.print(displayMsg[charIdx]);
    }

    scrollPos[row] = (scrollPos[row] + 1) % displayMsg.length();
  }
}

// Add this global variable to track activity in the current session
uint32_t sessionActivity = 0;

// ===== Login helpers =====
bool isValidPhone() {
  if (phoneLen != 11) return false;
  if (userPhone[0] != '0' || userPhone[1] != '9') return false;
  return true;
}

// ===== Redeem rules =====
// 5 accepted bottles = 1 point
uint32_t convertiblePoints() { 
  return userAccepted / 5; 
}

// Convert bottles to points
void doConvert() {
  uint32_t ptsToAdd = convertiblePoints();
  if (ptsToAdd == 0) return;
  
  userAccepted -= (ptsToAdd * 5); // Deduct bottles in multiples of 5
  userPoints += ptsToAdd;         // Add 1 point per 5 bottles
  pendingSaves++;
}

// 1 point = 1 peso load
uint32_t claimablePesos() {
  return userPoints; // Each point is 1 peso
}

// Claim load
uint32_t doClaim() {
  uint32_t pesos = userPoints;
  if (pesos == 0) return 0;
  
  userPoints = 0; // Deduct all points upon claiming
  pendingSaves++;
  return pesos; 
}

// ===== LCD content update =====
void lcdUpdateScreen(bool force = false) {
  static BinState lastState = (BinState)255;
  static uint32_t lastA = 0, lastR = 0, lastPts = 0;
  static bool lastP = false;
  static float lastD = 999999;
  static String lastPhone = "";

  bool changed = force
    || st != lastState
    || userAccepted != lastA
    || userRejected != lastR
    || userPoints   != lastPts
    || plastic != lastP
    || fabs(lastDelta - lastD) > 0.2f
    || String(userPhone) != lastPhone;

  if (!changed) return;

  if (st == ST_LOGIN) {
    lcdStartScroll(0, "Enter 11-digit 09xx");
    // Only update row 1 if length changed, but don't force a scroll reset
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print(userPhone);
    for (int i = phoneLen; i < 16; i++) lcd.print(' '); 
    scrollActive[1] = false; // Disable scroll for phone input to stop glitching
  }
  else if (st == ST_REDEEM) { 
    uint32_t pesos = claimablePesos();
    String l0 = "Load: P" + String(pesos) + " (" + String(userPoints) + "pts)";
    lcd.setCursor(0, 0);
    lcd.print(l0 + " Press C");

    // Line 1: Pending Bottles to Points
    uint32_t pendingPts = convertiblePoints();
    String l1 = "Bottles:" + String(userAccepted) + "->+" + String(pendingPts) + "pts";
    lcd.setCursor(0, 1);
    lcd.print(l1);
  }
  else {
    String l0 = String(stateName(st)) + " A:" + userAccepted + " R:" + userRejected + " Pts:" + userPoints;
    String l1 = String("P: ") + (plastic ? "Yes" : "No") + ", W: " + String(lastDelta, 1) + "g";
    lcdStartScroll(0, l0);
    lcdStartScroll(1, l1);
  }


  lastState = st;
  lastA = userAccepted;
  lastR = userRejected;
  lastPts = userPoints;
  lastP = plastic;
  lastD = lastDelta;
  lastPhone = String(userPhone);
}

// List of Smart/TNT Prefixes
const char* smartPrefixes[] = {
  "0900", "0907", "0908", "0909", "0910", "0912", "0913", "0914", 
  "0918", "0919", "0920", "0921", "0928", "0929", "0930", "0938", 
  "0939", "0940", "0946", "0947", "0948", "0949", "0950", "0951", 
  "0960", "0961", "0963", "0964", "0968", "0970", "0981", "0982",
  "0985", "0989", "0998", "0999", 
};
const int numPrefixes = 36;

bool isSmartOrTNT() {
  if (phoneLen < 4) return false;
  
  char currentPrefix[5];
  strncpy(currentPrefix, userPhone, 4);
  currentPrefix[4] = '\0';

  for (int i = 0; i < numPrefixes; i++) {
    if (strcmp(currentPrefix, smartPrefixes[i]) == 0) {
      return true;
    }
  }
  return false;
}

void handleLoginKey(char k) {
  if (k >= '0' && k <= '9') {
    if (phoneLen < 11) {
      userPhone[phoneLen++] = k;
      userPhone[phoneLen] = '\0';
    }
  } else if (k == '#') {
    if (phoneLen > 0) {
      phoneLen--;
      userPhone[phoneLen] = '\0';
    }
  } else if (k == '*') {
    phoneLen = 0;
    userPhone[0] = '\0';
  } else if (k == 'A') {
    if (phoneLen == 11 && isSmartOrTNT()) {
      userLoggedIn = true;
      loadOrCreateUser(userPhone);
      sessionActivity = 0;
      pendingSaves = 0;
      setState(ST_IDLE);
      topOpen();
      bottomClose();
    } else {
      // 1. Show Error
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("INVALID NUMBER!");
      lcd.setCursor(0, 1);
      lcd.print("SMART/TNT ONLY");
      
      // 2. Clear the failed number so they can try again
      phoneLen = 0;
      userPhone[0] = '\0';
      
      // 3. Force the LCD to redraw the login prompt
      lcd.clear(); 
      lcdUpdateScreen(true); 
      return; // Exit function so we don't trigger the update at the bottom twice
    }
  }
  lcdUpdateScreen(true);
}


void notifyAdmin(String msg) {
  Serial.write('S'); // Send 'S' for SMS
  // Or send a specific code like '1' for "Bottle Accepted"
}

void autoTare() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Auto taring...");

  unsigned long t0 = millis();
  while (!scale.is_ready()) {
    if (millis() - t0 > 3000) break; // timeout safety
    delay(10);
  }

  scale.set_scale(CAL);
  delay(300);
  scale.tare(30);  // average 30 samples

  lcd.setCursor(0, 1);
  lcd.print("Tare = 0 g");
  delay(800);
  lcd.clear();
}

void setup() {
  Serial.begin(115200);
  gsmSerial.begin(9600);
  lcd.init();
  lcd.backlight();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(CAP_PIN, INPUT);

  topClose();
  bottomClose();   // bottom starts closed

  scale.begin(HX_DT_PIN, HX_SCK_PIN);
  autoTare();
  scale.set_scale(CAL);

  // --- New initialization message ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ECOSWAP: Empty Plastic");
  lcd.setCursor(0, 1);
  lcd.print("Bottles to Load Balance");
  delay(3000); // Show message for 3 seconds

  setState(ST_LOGIN);
  lcdUpdateScreen(true);
}



void loop() {
  // Read keypad always
  char k = keypad.getKey();

  // Logout anytime
  if (k == 'D') {
    // CHECK FOR TRASH ACCOUNT
    // If they have 0 points, 0 accepted bottles, and did nothing this session
    if (userPoints == 0 && userAccepted == 0 && sessionActivity == 0) {
        if (currentUserIndex >= 0) {
            UserRecord blankRec = {}; 
            blankRec.valid = 0x00; // Invalidate the record
            EEPROM.put(recordAddr(currentUserIndex), blankRec);
            Serial.println("Trash account deleted to save space.");
        }
    } else {
        saveCurrentUser(); // Only save if they actually have data
    }

    userLoggedIn = false;
    phoneLen = 0;
    userPhone[0] = '\0';
    currentUserIndex = -1;
    sessionActivity = 0; 
    setState(ST_LOGIN);
    topClose();
    bottomClose();
    lcd.clear();
    lcdUpdateScreen(true);
  }

  // Enter redeem anytime (only if logged in)
  if (userLoggedIn && k == 'B' && st != ST_LOGIN) {
    setState(ST_REDEEM);
    topClose();
    bottomClose();          // keep hands safe while redeeming
    lcdUpdateScreen(true);
  }

  // Ultrasonic sampling (only relevant outside login/redeem)
  static unsigned long tPing = 0;
  static bool present = false;
  if (millis() - tPing >= 80) {
    tPing = millis();
    float d = readDistanceCm();
    if (d > 0) present = (d < PRESENT_CM);
  }

  switch (st) {
    case ST_LOGIN:
      if (k) handleLoginKey(k);
      break;

    case ST_REDEEM:
      if (k == 'C') {
        doConvert(); 
        uint32_t pesos = doClaim();
        
        if (pesos > 0) {
            // Send: 'C' (Command) + ':' (Separator) + phone number + '\n' (End)
            Serial.print('C');
            Serial.print(':');
            Serial.println(userPhone); 
            
            lcd.clear();
            lcd.print("CLAIMED P");
            lcd.print(pesos);
            delay(2000);
        }
        
        saveCurrentUser();
        lcdUpdateScreen(true);
      }

      // Press * to exit redeem back to idle
      if (k == '*') {
        setState(ST_IDLE);
        topOpen();
        lcdUpdateScreen(true);
      }
      break;

    case ST_IDLE:
      plastic = false;
      lastDelta = 0;

      if (present) {
        topClose();             // Trap bottle
        bottomClose();          // Ensure bottom closed
        setState(ST_ENTRY);
      }
      break;

    case ST_ENTRY:
      // Wait for bottle to fall and settle on bottom servo
      if (millis() - tState >= 500) {
        setState(ST_INSPECT);
      }
      break;

    case ST_INSPECT: {
      plastic = capDetected();

      if (plastic) {
        setState(ST_RELEASE);
      } else {
        userRejected++;
        pendingSaves++;
        bottomOpen();
        setState(ST_RESET);
      }
      break;
    }

    case ST_RELEASE:
      wBefore = weightG(20);    // Baseline weight
      bottomOpen();             // Release bottle
      setState(ST_SETTLE);
      break;

    case ST_SETTLE:
      if (millis() - tState >= 1200) {   // Allow to settle on load cell
        setState(ST_WEIGH);
      }
      break;

    case ST_WEIGH: {
      float wAfter = weightG(25);
      lastDelta = wAfter - wBefore;

      if (!isnan(wAfter)) {
        Serial.print("Weight (g): ");
        Serial.println(lastDelta, 2);

        // Show plastic detection + weight on line 1 only
        lcd.setCursor(0, 1);
        lcd.print("P: ");
        lcd.print(plastic ? "Yes" : "No");
        lcd.print(", W: ");
        lcd.print(lastDelta, 1);
        lcd.print("g   ");

        // Counters (optional – keep or remove)
        if (lastDelta >= 10.0 && lastDelta <= 70.0) {
          userAccepted++;
          sessionActivity++;
        } else {
          userRejected++;
        }

        pendingSaves++;
      } else {
        lcd.setCursor(0, 1);
        lcd.print("HX711 not ready  ");
        Serial.println("HX711 not ready");
      }

      if (pendingSaves >= 5) {
        saveCurrentUser();
        pendingSaves = 0;
      }

      setState(ST_RESET);
      break;
    }


    case ST_RESET:
      if (millis() - tState >= 800) {
        bottomClose();    // Close bottom again
        topOpen();        // Allow next bottle
        setState(ST_IDLE);
      }
      break;
  }

  // Update LCD + scroll
  lcdUpdateScreen(false);
  lcdScrollTick();
}