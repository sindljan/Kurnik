#include "Wire.h"
//****************Constants****************//
// PIN addres
#define analogPin A0
#define PRX_PIN 5
#define BTN_PIN 2
#define DIR_OPEN_PIN 7
#define DIR_CLOSE_PIN 6

#define DARK_TRESHOLD 0

#define SENZOR_DEACT_TIME 3*60
#define MAIN_LOOP_DELAY 3000

#define CMD_OPEN true
#define CMD_CLOSE false

#define DOOR_OPENING_TIME 80 // doba pro otevreni [s]
#define DOOR_CLOSING_TIME 52 // doba pro zavreni [s]

#define DS3231_I2C_ADDRESS 0x68 // Adresa I2C modulu s RTC

// pokud je definovano pak je program v rezimu ladeni
#define DEBUG


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
volatile byte bDoorOpened; // 0 => closed, 1 => open
volatile int nLightSenzorDeactT; // 0 => closed, 1 =>
volatile boolean bChangeDoorStateByBtn;
volatile boolean bThreadSleep;  // rika ze hlavni vlakno je uspane
volatile int f_timer = 0;

//******************Init*******************//
void setup() {
  bChangeDoorStateByBtn = false;
  nLightSenzorDeactT = 0;
  lightIntens = 0;
  bDoorOpened = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DIR_OPEN_PIN, OUTPUT);
  pinMode(DIR_CLOSE_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);
  pinMode(PRX_PIN, INPUT);
  calendar_setup();
  // Events setup
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnTouched, RISING);

#ifdef DEBUG
  Serial.begin(9600);
#endif
}

//****************Program******************//

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

bool IsTimeToOpenDoor()
{
  bool ItsTime = (lightIntens > DARK_TRESHOLD);
  const byte openTime = 8;
  const byte closeTime = 18;
  Time now = getTime();
  if ((now.h >= openTime) && (now.h <= closeTime)) ItsTime = true;

#ifdef DEBUG
  Serial.println("Now is: " + TimeToString(now));
#endif

  return ItsTime;
}

void loop() {

  // door controled by senzor
  if (nLightSenzorDeactT == 0)
  {
    UpdateLightIntensity();

    if (IsTimeToOpenDoor()) {
      OpenDoor();
    }
    else {
      CloseDoor();
    }
  }

  // door controled by btn
  if (bChangeDoorStateByBtn) {
    if (bDoorOpened == 1) {
      CloseDoor();
    } else {
      OpenDoor();
    }
    bChangeDoorStateByBtn = false;
    nLightSenzorDeactT = SENZOR_DEACT_TIME;
  }

  if (nLightSenzorDeactT > 0) {
    nLightSenzorDeactT -= 1;
  }
#ifdef DEBUG
  Serial.print("Senzor deactivation time ");
  Serial.println(nLightSenzorDeactT);
#endif
  //polling
  delay(MAIN_LOOP_DELAY);

}

bool AreDoorOpen() {
  bool doorStatus;
  doorStatus = digitalRead(PRX_PIN);
#ifdef DEBUG
  Serial.print(doorStatus);
#endif
  return doorStatus;
}

void OpenDoor() {
  if (bDoorOpened == 0)
  {
#ifdef DEBUG
    Serial.println("Opening door");
#endif

    digitalWrite(LED_BUILTIN, HIGH); // zapne LED
    RunMotor(CMD_OPEN);
    delay(5000); //cas aby najel na senzor pokud ho prejel
    for (int i = 0; i <= (2 * DOOR_OPENING_TIME); i++) {
      delay(500);
      // otevreni je odchyceno senzorem
      if (AreDoorOpen() == true) {
        break;
      }

#ifdef DEBUG
      Serial.print("Motor run time[s]: ");
      Serial.print(i / 2);
      Serial.print(", goal[s]: ");
      Serial.println(DOOR_OPENING_TIME);
#endif
    }
    // Stop Motor
    StopMotor();

    bDoorOpened = 1;
  }
#ifdef DEBUG
  Serial.println("Door opened");
#endif
}

void CloseDoor() {
  if (bDoorOpened == 1)
  {
#ifdef DEBUG
    Serial.println("Closing door");
#endif

    digitalWrite(LED_BUILTIN, LOW); // vypne LED
    RunMotor(CMD_CLOSE);
    for (int i = 0; i <= DOOR_CLOSING_TIME; i++) {
      delay(1000);
#ifdef DEBUG
      Serial.print("Motor run time[s]: ");
      Serial.print(i);
      Serial.print(", goal[s]: ");
      Serial.println(DOOR_CLOSING_TIME);
#endif
    }
    // Stop Motor
    StopMotor();

    bDoorOpened = 0;

  }
#ifdef DEBUG
  Serial.println("Door closed");
#endif
}


//****************Events******************//

void btnTouched() {
#ifdef DEBUG
  Serial.print("btn ");
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
//****************************************** Motor ************************************//

void StopMotor()
{
  digitalWrite(DIR_OPEN_PIN, LOW);
  digitalWrite(DIR_CLOSE_PIN, LOW);
#ifdef DEBUG
  Serial.println("Stopped");
#endif
}

void RunMotor(bool direct) {
  // Stop Motor in case that its running
  StopMotor();
#ifdef DEBUG
  Serial.println("Motor running");
#endif

  // open/close it
  if (direct == CMD_OPEN) {
    digitalWrite(DIR_OPEN_PIN, HIGH);
  } else {
    digitalWrite(DIR_CLOSE_PIN, HIGH);
  }
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
