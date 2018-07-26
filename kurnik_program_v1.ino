
#include <avr/sleep.h>//this AVR library contains the methods that controls the sleep modes
#include <avr/power.h>
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
volatile boolean bThreadSleep;  // rika ze hlavni vlakno je uspane
volatile int f_timer=0;

//******************Init*******************//
void setup() {
  bChangeDoorStateByBtn = false;
  nLightSenzorDeactT = 0;
  lightIntens = 0;
  bDoorOpened = 1;
  pinMode(LED_BUILTIN, OUTPUT);  // Use the led on pin 13 to indecate when Arduino is A sleep
  pinMode(DIR_OPEN_PIN, OUTPUT);
  pinMode(DIR_CLOSE_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);

  /*** Configure the timer.***/
  
  /* Normal timer operation.*/
  TCCR1A = 0x00; 
  
  /* Clear the timer counter register.
   * You can pre-load this register with a value in order to 
   * reduce the timeout period, say if you wanted to wake up
   * ever 4.0 seconds exactly.
   */
  TCNT1=0x0000; 
  
  /* Configure the prescaler for 1:1024, giving us a 
   * timeout of 4.09 seconds.
   */
  TCCR1B = 0x05;
  
  /* Enable the timer overlow interrupt. */
  TIMSK1=0x01;
  
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
  if(f_timer==1)
  {
    f_timer = 0;
    /* Toggle the LED */
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    
    // door controled by senzor
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
    delay(1000);
    /* Re-enter sleep mode. */
    enterSleep();
  }

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

// uspavani
/***************************************************
 *  Name:        ISR(TIMER1_OVF_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Timer1 Overflow interrupt.
 *
 ***************************************************/
ISR(TIMER1_OVF_vect)
{
  /* set the flag. */
   if(f_timer == 0)
   {
     f_timer = 1;
   }
}


/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_IDLE);
  
  sleep_enable();


  /* Disable all of the unused peripherals. This will reduce power
   * consumption further and, more importantly, some of these
   * peripherals may generate interrupts that will wake our Arduino from
   * sleep!
   */
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer2_disable();
  power_twi_disable();  

  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the timer timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();
}


//****************Events******************//

void btnTouched() {
  #ifdef DEBUG
  Serial.print("btn ");
  #endif
  bChangeDoorStateByBtn = true;
  delay(200);
}


