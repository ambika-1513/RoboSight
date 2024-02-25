#include <EEPROM.h>
#include <IRremote.hpp>
#include <LiquidCrystal.h>

#define PhotoResistor A3
unsigned long lastTimeLuminosity =millis();
unsigned long luminosityDelay =100;
#define IR 5
#define pinplay 0x43
#define pinprev 0x44
#define pinnext 0x40
#define pineq 0x9
#define pinch 0x46

#define DistanceINcm 0
#define DistanceINinches 1
#define cm_TO_inches 0.393701
#define EEPROM_address_distance_In_Unit 50

#define LCD_MODE_DISTANCE 0
#define  LCD_MODE_SETTINGS 1
#define LCD_MODE_LUMINOSITY 2
#define RS A5
#define E A4
#define d4 6
#define d5 7
#define d6 8
#define d7 9
LiquidCrystal lcd(RS, E, d4, d5, d6, d7);

#define echo 3
#define trigger 4
unsigned long lastTimeUltrasonicTrigger = millis();
unsigned long ultrasonicDelay = 60;
volatile bool newDistanceAvailable =false;
volatile unsigned long pulseTimeIN ;
volatile unsigned long pulseTimeOUT ;
double previousDistance = 400.0;

#define PushButton 2
unsigned long lastTimeButtonChanged = millis();
byte buttonState;
unsigned long debounceDelay = 50;

#define Error_LED_RED 12
unsigned long lastTimeError = millis();
unsigned long ErrorDelay =300;
#define Warning_LED_BLUE 11
unsigned long lastTimeWarning = millis();
unsigned long warningDelay =500;
byte Warning_LED_BLUE_state = LOW;
#define Light_LED_GREEN 10
byte Error_LED_RED_state =LOW;
int Light_LED_GREEN_state =LOW;

int lcdMODE = LCD_MODE_DISTANCE;
int distanceUNIT = DistanceINcm;
bool LOCK = false;

void triggerUltrasonic()
{
  digitalWrite( trigger, LOW);
  delayMicroseconds(2);
  digitalWrite( trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite( trigger , LOW);
}
void echoInterrupt()
{
  if(digitalRead(echo) == HIGH)
  {
   pulseTimeIN = micros();
  }
  else
  {
   pulseTimeOUT = micros();
   newDistanceAvailable = true; 
  }
}
double getDistance()
{
  double duration = pulseTimeOUT - pulseTimeIN;
  double dist = duration / 58.0;
  if(dist > 400.0)
  {
    return previousDistance;
  }
  dist =previousDistance *0.6 + dist*0.4;
  previousDistance = dist;
  return dist;
}
void printDistance()
{
  double dist = getDistance();
  if(LOCK)
  { 
  lcd.setCursor(0, 0);
  lcd.print("!!!!Obstacle!!!!");
  lcd.setCursor(0,1);
  lcd.print("Press to unlock ");
  }
  else if(lcdMODE == LCD_MODE_DISTANCE)
  {
    lcd.setCursor(0, 0);
    lcd.print("Distance=");
    if(distanceUNIT == DistanceINinches)
    {
        lcd.print(dist * cm_TO_inches);
        lcd.print("in         ");
    }
    else
    {
        lcd.print(dist);
    lcd.print("cm     ");
    }
    lcd.setCursor(0,1);
    if(dist <50)
    {
      lcd.print("--No Obstacle--");
    }
    else
    {
      lcd.print("!!!!WARNING!!!!");
    }
  }
}

void  toggleErrorLED()
{
  Error_LED_RED_state = (Error_LED_RED_state == LOW) ? HIGH : LOW;
  digitalWrite(Error_LED_RED , Error_LED_RED_state);
}

void toggleWarningLED()
{
  Warning_LED_BLUE_state =(Warning_LED_BLUE_state == LOW) ? HIGH : LOW;
  digitalWrite(Warning_LED_BLUE , Warning_LED_BLUE_state);
}

void BlinkRateOfWarningLED(double dist)
{
  warningDelay = dist*4;
}

void intiModes()
{
  pinMode(echo, INPUT);
  pinMode(trigger, OUTPUT);
  pinMode(PushButton, INPUT);
  pinMode(Error_LED_RED, OUTPUT);
  pinMode(Warning_LED_BLUE, OUTPUT);
  pinMode(Light_LED_GREEN, OUTPUT);
  pinMode(PhotoResistor, INPUT);
} 

void lock()
{
  if(!LOCK)
  {
    Serial.println("Locking");
    LOCK = true;
     Error_LED_RED_state = LOW;
    Warning_LED_BLUE_state = LOW;
  }
}

void unlock()
{
  if(LOCK)
  {
    Serial.println("Unlocking");
    LOCK = false;
    Error_LED_RED_state = LOW;
    digitalWrite(Error_LED_RED, Error_LED_RED_state);
    lcd.clear();
  }
}

void toggleDistanceUnit()
{
  if(distanceUNIT == DistanceINcm)
  {
    distanceUNIT = DistanceINinches;
  }
  else
  {
    distanceUNIT = DistanceINcm;
  }
  EEPROM.write(EEPROM_address_distance_In_Unit, distanceUNIT);
}

void toggleLCDScreen( bool next)
{
  switch(lcdMODE)
  {
    case LCD_MODE_DISTANCE:
    lcdMODE = (next) ? LCD_MODE_SETTINGS :LCD_MODE_LUMINOSITY;
    break;
    case LCD_MODE_SETTINGS:
    lcdMODE =(next) ? LCD_MODE_LUMINOSITY :LCD_MODE_DISTANCE;
    break;
    case LCD_MODE_LUMINOSITY:
    lcdMODE = (next) ? LCD_MODE_DISTANCE : LCD_MODE_SETTINGS;
    break;
    default:
    lcdMODE = LCD_MODE_DISTANCE;
  }
    lcd.clear();

  if(lcdMODE == LCD_MODE_SETTINGS);
  {
  
    lcd.setCursor(0, 0);
    lcd.print("Press on OFF to   ");
    lcd.setCursor(0,1);
    lcd.print("Reset settings.   ");
  }
}

void resetSettingsToDefault()
{
   if(lcdMODE == LCD_MODE_SETTINGS);
   {
     distanceUNIT = DistanceINcm;
     EEPROM.write(EEPROM_address_distance_In_Unit, distanceUNIT);
     lcd.clear();
      lcd.setCursor(0, 0);
     lcd.print("Settings have");
      lcd.setCursor(0, 1);
      lcd.print(" been RESET ");
   }
}

void setLEDLuminosity(int luminosity)
{
  byte brightness = map(luminosity, 0, 1023, 255, 0);
  analogWrite(Light_LED_GREEN, brightness);
}

void printLuminosity(int luminosity)
{
  if((!LOCK)&& (lcdMODE ==LCD_MODE_LUMINOSITY))
  {
   lcd.setCursor(0, 0);
   lcd.print("luminosity:");
   lcd.print(luminosity);
   lcd.print("          ");
  }
}

void setup() 
{
  Serial.begin(115200);
  lcd.begin(16, 2);
  intiModes();
  IrReceiver.begin(IR, ENABLE_LED_FEEDBACK); 
  buttonState = digitalRead(PushButton);
  attachInterrupt(digitalPinToInterrupt(echo), echoInterrupt , CHANGE);
  distanceUNIT = EEPROM.read(EEPROM_address_distance_In_Unit );
  if(distanceUNIT == 255)
  {
    distanceUNIT = DistanceINcm;
  }
  lcd.print("Intializing.....");
  delay(1000);
  lcd.clear();
}

void loop() 
{
  unsigned long timeNow = millis();
   // long luminosity = analogRead(PhotoResistor);
   // Map the LDR value to the LED brightness range
 //Light_LED_GREEN_state  = map(luminosity , 0, 1023, 255, 0);
 // analogWrite(Light_LED_GREEN , Light_LED_GREEN_state );
  
  if(LOCK)
  {
    if(timeNow- lastTimeError > ErrorDelay)
    {
      lastTimeError += ErrorDelay;
      toggleErrorLED();
      toggleWarningLED();
    }
    if(timeNow - lastTimeButtonChanged >debounceDelay)
    {
      byte newState = digitalRead(PushButton);
      if(newState != buttonState)
      {
         lastTimeButtonChanged = timeNow;
        buttonState = newState;
        if(buttonState == LOW)
           unlock();
      }
    }
  }
  else
  {
    if(timeNow- lastTimeWarning > warningDelay)
    {
      lastTimeWarning += warningDelay ;
      toggleWarningLED();
    }
  }
  if(IrReceiver.decode())
  {
    switch(IrReceiver.decodedIRData.command) 
    {
      case pinplay:
      unlock();
      break;
      case pinprev:
      toggleLCDScreen(false);
      break;
      case pinnext:
      toggleLCDScreen(true);
      break;
      case pinch:
      resetSettingsToDefault();
      break;
      case pineq:
      toggleDistanceUnit();
      break;
      default:
      ;
  }
  IrReceiver.resume();
}
  
  if(timeNow - lastTimeUltrasonicTrigger > ultrasonicDelay)
  {
    lastTimeUltrasonicTrigger += ultrasonicDelay;
    triggerUltrasonic();
  }
  if( newDistanceAvailable)
  {
    newDistanceAvailable = false;
    double dist =getDistance();
    Serial.println(dist);
    BlinkRateOfWarningLED(dist);
    if(dist <10 )
    {
      lock();
    }
  }
  printDistance();

  if(timeNow- lastTimeLuminosity > luminosityDelay)
  {
    lastTimeLuminosity += luminosityDelay;
    int luminosity =analogRead(PhotoResistor);
    setLEDLuminosity(luminosity);
    printLuminosity(luminosity);
  }
}
