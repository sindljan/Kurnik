#include "Wire.h"
//****************Constants****************//
// PIN addres
#define analogPin A0
#define BTN_PIN 2

#define DOOR1_OPEN_SENZOR 3
#define DOOR2_OPEN_SENZOR 4

#define M1 1
#define M2 2

#define M2_DIR_OPEN_PIN 10
#define M2_DIR_CLOSE_PIN 11

#define M1_DIR_OPEN_PIN 12
#define M1_DIR_CLOSE_PIN 13

#define DARK_TRESHOLD 0

#define MAIN_LOOP_DELAY 500

// door commands
#define CMD_OPEN true
#define CMD_CLOSE false

// Komentár k motorovym konstantam
// Motor s jednou prevodovkou (cca 30 N/cm) ma zaviraci cas cca 190s a oteviraci cca 157s ne dverich c1
// Motor s jednou prevodovkou (cca 50 N/cm) ma zaviraci cas cca 65s a oteviraci cca 110s ne dverich c2

#define DOOR1_CLOSING_TIME 190 // doba pro zavreni [s]
#define DOOR2_CLOSING_TIME 65 // doba pro zavreni [s]
#define DOOR_OPENING_LIM_TIME 300 // limitni doba pro otevreni [s]

#define DS3231_I2C_ADDRESS 0x68 // Adresa I2C modulu s RTC

// pokud je definovano pak je program v rezimu ladeni
//
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

#define OPEN_DOOR_AT 7
#define CLOSE_DOOR_AT 21


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
Time OpeningStart;
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
  StopMotor(M1);
  StopMotor(M2);
  // Events setup
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnTouched, RISING);

#ifdef DEBUG
  Serial.begin(9600);
#endif
}

//****************Program******************//

bool IsTimeToOpenDoor()
{
  bool ItsTime = false;
  const byte openTime = OPEN_DOOR_AT;
  const byte closeTime = CLOSE_DOOR_AT;
  Time now = getTime();
  Date dNow = getDate();
  if ((now.h >= openTime) && (now.h < closeTime)) ItsTime = true;

#ifdef DEBUG
  Serial.println("Now is: " + TimeToString(now) + " " + DateToString(dNow));
  if(ItsTime){
    Serial.println("Door should be open until ("+(String)closeTime+"  Odpoledne)");
  }  else {
    Serial.println("Door should be closed until ("+(String)openTime+" Rano)");
  }
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
  int ClosingT;
  
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
    #ifdef DEBUG
    Serial.println("Door 1 are opening now.");
    #endif  
    OpeningStart = getTime();
    RunMotor(CMD_OPEN,M1);
    ProgState = DOOR1_OPENING;  
    break;
  case DOOR1_OPENING:
    if(AreDoor1Open()){
      StopMotor(M1);
      ProgState = OPEN_DOOR2;
      #ifdef DEBUG
      Serial.println("Door 1 opened.");
      #endif 
    }
    break;
  case OPEN_DOOR2:
    #ifdef DEBUG
    Serial.println("Door 2 are opening now.");
    #endif   
    OpeningStart = getTime();
    RunMotor(CMD_OPEN,M2);
    ProgState = DOOR2_OPENING;   
    break;
  case DOOR2_OPENING:
    if(AreDoor2Open()){
      StopMotor(M2);
      bDoorOpened = true;
      ProgState = START;
      #ifdef DEBUG
      Serial.println("Door 2 opened.");
      #endif 
    }
    break;
  case CLOSE_DOOR1:
    #ifdef DEBUG
    Serial.println("Door 1 are closing now.");
    #endif   
    ClosingStart = getTime();
    RunMotor(CMD_CLOSE,M1);
    ProgState = DOOR1_CLOSING; 
    break;
  case DOOR1_CLOSING:
    ClosingT = (now.h*3600+now.m*60+now.s)-(ClosingStart.h*3600+ClosingStart.m*60+ClosingStart.s);
    #ifdef DEBUG
    Serial.print("Door 1 closing time ");
    Serial.println(ClosingT);
    #endif 
    if(ClosingT >= DOOR1_CLOSING_TIME){
      StopMotor(M1);
      ProgState = CLOSE_DOOR2;
      #ifdef DEBUG
      Serial.println("Door 1 closed.");
      #endif 
    }
    break;
  case CLOSE_DOOR2:
    #ifdef DEBUG
    Serial.println("Door 2 are closing now.");
    #endif   
    ClosingStart = getTime();
    RunMotor(CMD_CLOSE,M2);
    ProgState = DOOR2_CLOSING;   
    break;  
  case DOOR2_CLOSING:
    ClosingT = (now.h*3600+now.m*60+now.s)-(ClosingStart.h*3600+ClosingStart.m*60+ClosingStart.s);
    #ifdef DEBUG
    Serial.print("Door 2 closing time ");
    Serial.println(ClosingT);
    #endif 
    if(ClosingT >= DOOR1_CLOSING_TIME){
      StopMotor(M2);
      bDoorOpened = false;
      ProgState = START;
      #ifdef DEBUG
      Serial.println("Door 2 closed.");
      #endif        
    }
    break;                             
  default:
    // statements
    Serial.print("Unexpected state of machine: " );
    Serial.print(ProgState);
    break;
  }
  
  #ifdef DEBUG
  Serial.println(ProgState);
  #endif
  //polling
  delay(MAIN_LOOP_DELAY);
}

bool AreDoor1Open() {
  bool doorStatus;
  Time now = getTime();
  int Opening = (now.h*3600+now.m*60+now.s)-(OpeningStart.h*3600+OpeningStart.m*60+OpeningStart.s);
  
  doorStatus = !digitalRead(DOOR1_OPEN_SENZOR);

  if(Opening >= DOOR_OPENING_LIM_TIME){
    doorStatus = true;
    Serial.println("Door 1 opening error: opening time limit exceeded.");
  }
  
#ifdef DEBUG
  Serial.print("Opening time: ");
  Serial.print(Opening);
  Serial.print(", Door 1 end sensor status: ");
  Serial.println(doorStatus);
#endif
  return doorStatus;
}

bool AreDoor2Open() {
  bool doorStatus;
  Time now = getTime();
  int Opening = (now.h*3600+now.m*60+now.s)-(OpeningStart.h*3600+OpeningStart.m*60+OpeningStart.s);
  
  doorStatus = !digitalRead(DOOR2_OPEN_SENZOR);
  
  if(Opening >= DOOR_OPENING_LIM_TIME){
    doorStatus = true;
    Serial.println("Door 2 opening error: opening time limit exceeded.");
  }
  
#ifdef DEBUG
  Serial.print("Opening time: ");
  Serial.print(Opening);
  Serial.print(", Door 2 end sensor status: ");
  Serial.println(doorStatus);
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
  if (motorID == M1) {
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
  if (motorID == M1) {
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
  Serial.print("Motor ");
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

String DateToString(Date now){
  String y = (String)now.y;
  String m = (String)now.m;
  String d = (String)now.d;
  String wd = (String)now.wd;
  
  return y+"."+m+"."+d+" ("+wd+")";
}
//******************Init*******************//
void calendar_setup() {
  Wire.begin();
  // set the initial time here:
  // DS3231 seconds, minutes, hours, day, date, month, year
  //setDS3231time(00,41,19,2,31,03,20);
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
