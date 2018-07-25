
//****************Constants****************//
// PIN addres
#define analogPin A0
#define BTN_PIN 2
#define DIR_OPEN_PIN 6
#define DIR_CLOSE_PIN 7

#define DARK_TRESHOLD 2

#define SENZOR_DEACT_TIME 60;

// pokud je definovano pak je program v rezimu ladeni
//#define DEBUG



//****************Variables****************//
// vytvoření proměnných pro výsledky měření
int lightIntens;
volatile byte bDoorOpened; // 0 => closed, 1 => open
volatile int nLightSenzorDeactT; // 0 => closed, 1 => 
volatile boolean bChangeDoorStateByBtn;

//******************Init*******************//
void setup() {
  bChangeDoorStateByBtn = false;
  nLightSenzorDeactT = 0;
  lightIntens = 0;
  bDoorOpened = 1;
  pinMode(LED_BUILTIN, OUTPUT);
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
void ReadFromLightSenzor()
{
  int refMeasuredVal = 0; // pracovni hodnota ze senzoru
  int measuredVal = 0; // pracovni hodnota ze senzoru
  int measSum = 0; // suma merenych hodnota
  int dev =0; // deviace mereni
  const int devTreshold = 4; // deviace mereni
  const int numOfMeasurement = 3;
  
  // načtení hodnoty z analogového pinu s průměrováním 3 měření po 0,5s
  refMeasuredVal = analogRead(analogPin);
  for (int i=0; i < numOfMeasurement; i++){
    measuredVal = analogRead(analogPin);
    measSum += measuredVal;
    dev = refMeasuredVal - measuredVal;
    if((dev > devTreshold) || (dev < (-devTreshold))){
      break;
    }
    delay(500);
  }
  // osetreni spicek
  if((dev < devTreshold) && (dev > (-devTreshold))){
    lightIntens = measSum / numOfMeasurement;
  }
   
}

void loop() {

  // door control by senzor
  if(nLightSenzorDeactT == 0)
  {
    ReadFromLightSenzor();
  
    // put your main code here, to run repeatedly:
    if (lightIntens > DARK_TRESHOLD) {
      OpenDoor();
    }
    else {
      CloseDoor();
    }
  }

  // door control by btn
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
  delay(1000);

}

void RunMotor(int runTime, bool direct) {
  // Stop Motor in case that its running
  digitalWrite(DIR_OPEN_PIN, LOW);
  digitalWrite(DIR_CLOSE_PIN, LOW);
  
  // open/close it
  if (direct == true){
    digitalWrite(DIR_OPEN_PIN, HIGH);
  } else {
    digitalWrite(DIR_CLOSE_PIN, HIGH);
  }
  delay(runTime);
  
  // Stop Motor
  digitalWrite(DIR_OPEN_PIN, LOW);
  digitalWrite(DIR_CLOSE_PIN, LOW);
}

void OpenDoor() {
  if(bDoorOpened == 0)
  {
    #ifdef DEBUG
      Serial.println("Opening door");
    #endif
    digitalWrite(LED_BUILTIN, LOW); // zapne LED
    RunMotor(2000, 1);
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
    RunMotor(2000, 0);
    bDoorOpened = 0;
  }
  #ifdef DEBUG
    Serial.println("Door closed");
  #endif
}

//****************Events******************//

void btnTouched() {
  Serial.print("btn ");
  bChangeDoorStateByBtn = true;
  delay(200);
}


