#include "Wire.h"
//****************Constants****************//
// PIN addres
#define analogPin A0
#define BTN_PIN 2

#define DOOR1_OPEN_SENZOR 4
#define DOOR2_OPEN_SENZOR 3

#define M1 1
#define M2 2

#define M2_DIR_OPEN_PIN 10
#define M2_DIR_CLOSE_PIN 11

#define M1_DIR_OPEN_PIN 12
#define M1_DIR_CLOSE_PIN 13

#define DARK_TRESHOLD 0

#define MAIN_LOOP_DELAY 300

// door commands
#define CMD_OPEN true
#define CMD_CLOSE false

#define DOOR1_CLOSING_TIME 48 // doba pro zavreni [s]
#define DOOR2_CLOSING_TIME 48 // doba pro zavreni [s]

#define DS3231_I2C_ADDRESS 0x68 // Adresa I2C modulu s RTC

// pokud je definovano pak je program v rezimu ladeni
#define DEBUG

// prog status
#define START 1
#define MEASURE_LIGHT 2
#define CHECK_TIMER 3
#define OPEN_DOOR1 4
#define DOOR1_OPENING 5
#define OPEN_DOOR2 6
#define DOOR2_OPENING 7
#define CLOSE_DOOR1 8
#define DOOR1_CLOSING 9
#define CLOSE_DOOR2 10
#define DOOR2_CLOSING 11



struct Time {
  byte h;
  byte m;
  byte s;
};

struct Date {
  byte d;
  byte m;
  int y;
  byte wd;
};

//****************Variables****************//
// vytvoření proměnných pro výsledky měření
int lightIntens;
int ProgState;
Time ClosingStart;
boolean bTodayOpen;
boolean bTodayClose;
volatile byte bDoorOpened; // 0 => closed, 1 => open
volatile int nLightSenzorDeactT; // 0 => closed, 1 =>
volatile boolean bChangeDoorStateByBtn;


//******************Init*******************//
void setup() {
  ProgState = START;
  bChangeDoorStateByBtn = false;
  nLightSenzorDeactT = 0;
  lightIntens = 0;
  bDoorOpened = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(M1_DIR_OPEN_PIN, OUTPUT);
  pinMode(M1_DIR_CLOSE_PIN, OUTPUT);
  pinMode(M2_DIR_OPEN_PIN, OUTPUT);
  pinMode(M2_DIR_CLOSE_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);
  pinMode(DOOR1_OPEN_SENZOR, INPUT);
  pinMode(DOOR2_OPEN_SENZOR, INPUT);
  calendar_setup();
  // Events setup
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnTouched, RISING);

#ifdef DEBUG
  Serial.begin(9600);
#endif
}

//****************Program******************//

bool IsTimeToOpenDoor()
{
  bool ItsTime = (lightIntens > DARK_TRESHOLD);
  const byte openTime = 7;
  const byte closeTime = 22;
  Time now = getTime();
  if ((now.h >= openTime) && (now.h <= closeTime)) ItsTime = true;

#ifdef DEBUG
  Serial.println("Now is: " + TimeToString(now));
#endif

  return ItsTime;
}

bool DayChanged(Time now)
{
  if(now.h = 0) {return true;}
  return false;
}

void loop() {
  Time now = getTime();
  
  switch (ProgState) {
  case START:
    ProgState = MEASURE_LIGHT;
    if(DayChanged(now)){
      bTodayClose = false;
      bTodayOpen = false;
    }
    if(bChangeDoorStateByBtn){
      bChangeDoorStateByBtn = false;
      if(bDoorOpened){
        ProgState = CLOSE_DOOR1;
      } else {
        ProgState = OPEN_DOOR1;
      }
    }
    break;
  case MEASURE_LIGHT:
    ProgState = CHECK_TIMER;
    break;
  case CHECK_TIMER:
    if(IsTimeToOpenDoor()&&!(bDoorOpened)&&!bTodayOpen){
      bTodayOpen = true;
      ProgState = OPEN_DOOR1;
    } else if (!IsTimeToOpenDoor()&&(bDoorOpened)&&!bTodayClose){
      bTodayClose = true;
      ProgState = CLOSE_DOOR1;
    } else {
      ProgState = START;
    }
    break;
  case OPEN_DOOR1:
    RunMotor(CMD_OPEN,M1);
    ProgState = DOOR1_OPENING;
    break;
  case DOOR1_OPENING:
    if(AreDoor1Open()){
      StopMotor(M1);
      ProgState = OPEN_DOOR2;
    }
    break;
  case OPEN_DOOR2:
    RunMotor(CMD_OPEN,M2);
    ProgState = DOOR2_OPENING;
    break;
  case DOOR2_OPENING:
    if(AreDoor2Open()){
      StopMotor(M2);
      bDoorOpened = true;
      ProgState = START;
    }
    break;
  case CLOSE_DOOR1:
    ClosingStart = getTime();
    RunMotor(CMD_CLOSE,M1);
    ProgState = DOOR1_CLOSING;
    break;
  case DOOR1_CLOSING:
    if((now.h*3600+now.m*60+now.s)-(ClosingStart.h*3600+ClosingStart.m*60+ClosingStart.s)>=DOOR1_CLOSING_TIME){
      StopMotor(M1);
      ProgState = CLOSE_DOOR2;
    }
    break;
  case CLOSE_DOOR2:
    ClosingStart = getTime();
    RunMotor(CMD_CLOSE,M2);
    ProgState = DOOR2_CLOSING;
    break;  
  case DOOR2_CLOSING:
    if((now.h*3600+now.m*60+now.s)-(ClosingStart.h*3600+ClosingStart.m*60+ClosingStart.s)>=DOOR1_CLOSING_TIME){
      StopMotor(M2);
      bDoorOpened = false;
      ProgState = START;
    }
    break;                             
  default:
    // statements
    Serial.print("Unexpected state of machine: " );
    Serial.print(ProgState);
    break;
  }
  //polling
  delay(MAIN_LOOP_DELAY);
}

bool AreDoor1Open() {
  bool doorStatus;
  doorStatus = digitalRead(DOOR1_OPEN_SENZOR);
#ifdef DEBUG
  Serial.print(doorStatus);
#endif
  return doorStatus;
}

bool AreDoor2Open() {
  bool doorStatus;
  doorStatus = digitalRead(DOOR2_OPEN_SENZOR);
#ifdef DEBUG
  Serial.print(doorStatus);
#endif
  return doorStatus;
}

//****************Events******************//

void btnTouched() {
#ifdef DEBUG
  Serial.print("btn");
#endif
  bChangeDoorStateByBtn = true;
  delay(200);
}

//*************************************** Light Sensor *********************************//
int ReadFromLightSenzor(int numOfMeasurement)
{
  int measuredVal = 0; // pracovni hodnota ze senzoru
  // načtení hodnoty z analogového pinu s průměrováním měření
  for (int i = 0; i < numOfMeasurement; i++) {
    measuredVal += analogRead(analogPin);
    delay(50);
  }
  measuredVal = round(measuredVal / numOfMeasurement);
  return measuredVal;
}
//*************************************** Light Sensor *********************************//

void UpdateLightIntensity()
{
  int refMeasuredVal = 0; // pracovni hodnota ze senzoru
  int measuredVal = 0; // pracovni hodnota ze senzoru
  int measSum = 0; // suma merenych hodnota
  int dev = 0; // deviace mereni
  const int devTreshold = 15; // deviace mereni
  const int numOfMeasurement = 3;

  // načtení hodnoty z analogového pinu s průměrováním 3 měření po 0,5s
  refMeasuredVal = ReadFromLightSenzor(10);
#ifdef DEBUG
  Serial.print("Measured light ref value: ");
  Serial.println(refMeasuredVal);
#endif
  for (int i = 0; i < numOfMeasurement; i++) {
    measuredVal = ReadFromLightSenzor(10);
#ifdef DEBUG
    Serial.print("Measured light value: ");
    Serial.println(measuredVal);
#endif
    measSum += measuredVal;
    dev = refMeasuredVal - measuredVal;
    if ((dev > devTreshold) || (dev < (-devTreshold))) {
      break;
    }
    delay(2000);
  }
  // osetreni spicek
  if ((dev < devTreshold) && (dev > (-devTreshold))) {
    lightIntens = round(measSum / numOfMeasurement);
  } else {
#ifdef DEBUG
    Serial.print("Incorrect measurement: ");
#endif
  }

#ifdef DEBUG
  Serial.print("Average light senzor value: ");
  Serial.print(lightIntens);
  Serial.print(", Deviation:  ");
  Serial.print(dev);
  Serial.print(", DevThreshold: ");
  Serial.println(devTreshold);
#endif

}

//****************************************** Motor ************************************//

void StopMotor(int motorID)
{
  if (motorID = M1) {
    digitalWrite(M1_DIR_OPEN_PIN, LOW);
    digitalWrite(M1_DIR_CLOSE_PIN, LOW);
  } else {
    digitalWrite(M2_DIR_OPEN_PIN, LOW);
    digitalWrite(M2_DIR_CLOSE_PIN, LOW);
  }
#ifdef DEBUG
  Serial.print("Motor ");
  Serial.print(motorID);
  Serial.println(" stopped. ");
#endif
}

void RunMotor(bool direct, int motorID) {
  // Stop Motor in case that its running
  StopMotor(motorID);
  
  // open or close it
  if (motorID = M1) {
    if (direct == CMD_OPEN) {
      digitalWrite(M1_DIR_OPEN_PIN, HIGH);
    } else {
      digitalWrite(M1_DIR_CLOSE_PIN, HIGH);
    }
  } else {
    if (direct == CMD_OPEN) {
      digitalWrite(M2_DIR_OPEN_PIN, HIGH);
    } else {
      digitalWrite(M2_DIR_CLOSE_PIN, HIGH);
    }
  }

  #ifdef DEBUG
  Serial.print("Motor running");
  Serial.print(motorID);
  Serial.println(" running.");
  #endif
}

//****************************************** Motor ************************************//

//****************************************** Kalendar *********************************//
//****************SuppFunc*****************//
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val) {
  return ( (val / 10 * 16) + (val % 10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val) {
  return ( (val / 16 * 10) + (val % 16) );
}

String TimeToString(Time t){
  String h = (String)t.h;
  String m = (String)t.m;
  String s = (String)t.s;
  if (t.h<10) h = "0"+(String)t.h;
  if (t.m<10) m = "0"+(String)t.m;
  if (t.s<10) s = "0"+(String)t.s;
  return h+":"+m+":"+s;
}
//******************Init*******************//
void calendar_setup() {
  Wire.begin();
  // set the initial time here:
  // DS3231 seconds, minutes, hours, day, date, month, year
  //setDS3231time(00,00,18,5,29,11,18);
}

//****************Program******************//
void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year) {
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}
void readDS3231time(byte *second, byte *minute, byte *hour, byte *dayOfWeek, byte *dayOfMonth, byte *month, byte *year) {
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

void resetRegisterTo(byte addr) {
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(addr); // set DS3231 register pointer to 00h
  Wire.endTransmission();
}

Time getTime() {
  Time now = {0, 0, 0};
  resetRegisterTo(0);
  Wire.requestFrom(DS3231_I2C_ADDRESS, 3);
  // request 3 bytes of data from DS3231 starting from register 00h
  now.s = bcdToDec(Wire.read() & 0x7f);
  now.m = bcdToDec(Wire.read());
  now.h = bcdToDec(Wire.read() & 0x3f);
  return now;
}

Date getDate() {
  Date today = {0, 0, 0, 0};
  resetRegisterTo(3);
  Wire.requestFrom(DS3231_I2C_ADDRESS, 4);
  // request 3 bytes of data from DS3231 starting from register 00h
  today.wd = bcdToDec(Wire.read());
  today.d = bcdToDec(Wire.read());
  today.m = bcdToDec(Wire.read());
  today.y = 2000 + bcdToDec(Wire.read());
  return today;
}

//****************************************** Kalendar *********************************//
