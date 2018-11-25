
//****************Constants****************//
// PIN addres
#define analogPin A0
#define BTN_PIN 2
#define DIR_OPEN_PIN 7
#define DIR_CLOSE_PIN 6

#define DARK_TRESHOLD 0

#define SENZOR_DEACT_TIME 3*60
#define MAIN_LOOP_DELAY 3000

// pokud je definovano pak je program v rezimu ladeni
#define DEBUG



//****************Variables****************//
// vytvoření proměnných pro výsledky měření
int lightIntens;
const int nMotorRunTime_Open = 42;  // doba pro otevreni [s]
const int nMotorRunTime_Close = 21; // doba pro zavreni [s]
volatile byte bDoorOpened; // 0 => closed, 1 => open
volatile int nLightSenzorDeactT; // 0 => closed, 1 => 
volatile boolean bChangeDoorStateByBtn;
volatile boolean bThreadSleep;  // rika ze hlavni vlakno je uspane
volatile int f_timer=0;

//******************Init*******************//
void setup() {
  bChangeDoorStateByBtn = false;
  nLightSenzorDeactT = 0;
  lightIntens = 0;
  bDoorOpened = 0;
  pinMode(LED_BUILTIN, OUTPUT);  // Use the led on pin 13 to indecate when Arduino is A sleep
  pinMode(DIR_OPEN_PIN, OUTPUT);
  pinMode(DIR_CLOSE_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT);
 
  // Events setup
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnTouched, RISING);

#ifdef DEBUG
  Serial.begin(9600);
#endif
}

//****************Program******************//
int ReadFromLightSenzor(int numOfMeasurement)
{
  int measuredVal = 0; // pracovni hodnota ze senzoru
  // načtení hodnoty z analogového pinu s průměrováním měření
  for (int i=0; i < numOfMeasurement; i++){
    measuredVal += analogRead(analogPin);
    delay(50);
  }
  measuredVal = round(measuredVal / numOfMeasurement);
  return measuredVal;
}


void UpdateLightIntensity()
{
  int refMeasuredVal = 0; // pracovni hodnota ze senzoru
  int measuredVal = 0; // pracovni hodnota ze senzoru
  int measSum = 0; // suma merenych hodnota
  int dev =0; // deviace mereni
  const int devTreshold = 15; // deviace mereni
  const int numOfMeasurement = 3;
  
  // načtení hodnoty z analogového pinu s průměrováním 3 měření po 0,5s
  refMeasuredVal = ReadFromLightSenzor(10);
  #ifdef DEBUG
    Serial.print("Measured light ref value: ");
    Serial.println(refMeasuredVal);
  #endif
  for (int i=0; i < numOfMeasurement; i++){
    measuredVal = ReadFromLightSenzor(10);
    #ifdef DEBUG
      Serial.print("Measured light value: ");
      Serial.println(measuredVal);
    #endif
    measSum += measuredVal;
    dev = refMeasuredVal - measuredVal;
    if((dev > devTreshold) || (dev < (-devTreshold))){
      break;
    }
    delay(2000);
  }
  // osetreni spicek
  if((dev < devTreshold) && (dev > (-devTreshold))){
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

void loop() {

  // door controled by senzor
  if(nLightSenzorDeactT == 0)
  {
    UpdateLightIntensity();
    
    if (lightIntens > DARK_TRESHOLD) {
      OpenDoor();
    }
    else {
      CloseDoor();
    }
  }

  // door controled by btn
  if (bChangeDoorStateByBtn){
    if (bDoorOpened == 1) {
      CloseDoor();
    } else {
      OpenDoor();
    }
    bChangeDoorStateByBtn = false;
    nLightSenzorDeactT = SENZOR_DEACT_TIME;
  }

  if (nLightSenzorDeactT > 0){
    nLightSenzorDeactT-=1;
  }
  #ifdef DEBUG
    Serial.print("Senzor deactivation time ");
    Serial.println(nLightSenzorDeactT);
  #endif
  //polling
  delay(MAIN_LOOP_DELAY);

}

void StopMotor()
{
  digitalWrite(DIR_OPEN_PIN, LOW);
  digitalWrite(DIR_CLOSE_PIN, LOW);
  #ifdef DEBUG
    Serial.println("Stopped");
  #endif
}

void RunMotor(int runTime, bool direct) {  
  // Stop Motor in case that its running
  StopMotor();
  #ifdef DEBUG
    Serial.println("Motor running");
  #endif
  
  // open/close it
  if (direct == true){
    digitalWrite(DIR_OPEN_PIN, HIGH);
  } else {
    digitalWrite(DIR_CLOSE_PIN, HIGH);
  }

  for(int i=0; i<=runTime; i++){
    delay(1000);
    #ifdef DEBUG
      Serial.print("Motor run time[s]: ");
      Serial.print(i);
      Serial.print(", goal[s]: ");
      Serial.println(runTime);
    #endif
  }
    
  // Stop Motor
  StopMotor();
  
}

void OpenDoor() {
  if(bDoorOpened == 0)
  {
    #ifdef DEBUG
      Serial.println("Opening door");
    #endif
    digitalWrite(LED_BUILTIN, HIGH); // zapne LED
    RunMotor(nMotorRunTime_Open, 1);
    bDoorOpened = 1;
  }
  #ifdef DEBUG
    Serial.println("Door opened");
  #endif
}

void CloseDoor() {
  if(bDoorOpened == 1)
  {
    #ifdef DEBUG
      Serial.println("Closing door");
    #endif
    digitalWrite(LED_BUILTIN, LOW); // zapne LED
    RunMotor(nMotorRunTime_Close, 0);
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
