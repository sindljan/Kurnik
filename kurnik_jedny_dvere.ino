#include "Wire.h"
//****************Constants****************//
// PIN addres
#define analogPin A0
#define BTN_PIN 2

#define DOOR1_OPEN_SENZOR 3

#define M1 1


#define M1_DIR_OPEN_PIN 12
#define M1_DIR_CLOSE_PIN 13

#define MAIN_LOOP_DELAY 1000

// door commands
#define CMD_OPEN true
#define CMD_CLOSE false

// Komentar k motorovym konstantam
// Motor s jednou prevodovkou (cca 30 N/cm) ma zaviraci cas cca 190s a oteviraci cca 157s ne dverich c1

#define DOOR1_CLOSING_TIME 190 // doba pro zavreni [s]
#define DOOR_OPENING_LIM_TIME 300 // limitni doba pro otevreni [s]

#define DS3231_I2C_ADDRESS 0x68 // Adresa I2C modulu s RTC

// pokud je definovano pak je program v rezimu ladeni
//
#define DEBUG

// prog status
#define START 1
#define CHECK_TIMER 3
#define OPEN_DOOR1 4
#define DOOR1_OPENING 5
#define CLOSE_DOOR1 8
#define DOOR1_CLOSING 9

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

static int OpeningTimes[12] = {9,9,8,8,8,7,7,7,8,8,9,9};
static int ClosingTimes[12] = {17,18,19,21,21,22,22,22,21,20,18,17};

//****************Variables****************//
// vytvoření proměnných pro výsledky měření
int ProgState;
Time ClosingStart;
Time OpeningStart;
boolean bTodayOpen;
boolean bTodayClose;
volatile byte bDoorOpened; // 0 => closed, 1 => open
volatile int nLightSenzorDeactT; // 0 => closed, 1 =>
volatile boolean bChangeDoorStateByBtn;

//bool prepinac = false;


//******************Init*******************//
void setup() {
  ProgState = START;
  bChangeDoorStateByBtn = false;
  bDoorOpened = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(M1_DIR_OPEN_PIN, OUTPUT);
  pinMode(M1_DIR_CLOSE_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);
  pinMode(DOOR1_OPEN_SENZOR, INPUT);
  calendar_setup();
  StopMotor(M1);
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
  byte openTime;
  byte closeTime;
  Time now = getTime();
  Date dNow = getDate();
  openTime = OpeningTimes[dNow.m-1];
  closeTime = ClosingTimes[dNow.m-1];
  if ((now.h >= openTime) && (now.h < closeTime)) ItsTime = true;

#ifdef DEBUG
  Serial.println("Now is: " + TimeToString(now) + " " + DateToString(dNow));
  Serial.println("Open door after: " + (String)openTime + " Close door after: " + (String)closeTime);
  if(ItsTime){
    Serial.println("Door should be open until ("+(String)closeTime+":00)");
  }  else {
    Serial.println("Door should be closed until ("+(String)openTime+":00)");
  }
#endif

  return ItsTime;
}

bool DayChanged(Time now)
{
  if(now.h == 0) {return true;}
  return false;
}

void loop() {
  Time now = getTime();
  int ClosingT;
  
  switch (ProgState) {
  case START:
    ProgState = CHECK_TIMER;
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
      ProgState = START;
      #ifdef DEBUG
      Serial.println("Door 1 opened.");
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
      ProgState = START;
      #ifdef DEBUG
      Serial.println("Door 1 closed.");
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

//****************Events******************//

void btnTouched() {
#ifdef DEBUG
  Serial.print("btn");
#endif
  bChangeDoorStateByBtn = true;
  delay(200);
}

//****************************************** Motor ************************************//

void StopMotor(int motorID)
{
  if (motorID == M1) {
    digitalWrite(M1_DIR_OPEN_PIN, LOW);
    digitalWrite(M1_DIR_CLOSE_PIN, LOW);
  } else {
    
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
  // setDS3231time(00,22,15,3,01,06,26);
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
