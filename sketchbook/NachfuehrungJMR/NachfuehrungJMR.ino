// Steuerungssoftware zur Nachführung des Josef-Mosbach-Refraktors der Sternwarte Überlingen
// BW / 20.3.2018 V 1.01

#include <LiquidCrystal.h>               // Anzeigenbibliothek einbinden
//#include "DHT.h" //DHT Bibliothek laden  // Temperatur-/Feuchtigkeitssensor-Bibliothek einbinden
#include <EEPROM.h>
#include <TMC2208Stepper.h>

/*
  PIN-Belegungen
*/

#define PIN_DISPLAY_POTI A13
#define PIN_SCALE_POTI A14
#define PIN_TRACKING_SLIDER A15

#define PIN_HANDCONTROL_LEDBRIGHTNESS 4
#define PIN_SCALE_ILLU 5
#define PIN_DISPLAY_ILLU 3

// Schrittmotor Rektaszension
#define PIN_MOTOR_REK_DIR 8
#define PIN_MOTOR_REK_STEP 9
#define REK_ROT_CW LOW
#define REK_ROT_CCW HIGH

// Schrittmotor Deklination
#define PIN_MOTOR_DEK_DIR 10
#define PIN_MOTOR_DEK_STEP 11
#define DEK_ROT_CW LOW
#define DEK_ROT_CCW HIGH

// Temperatur-/Feuchtigkeitssensor
//#define PIN_DHT22 2 // pin 10 als Eingang DHT definieren

// LCD-PINs
#define PIN_LCD_RS 30
#define PIN_LCD_ENABLE 32
#define PIN_LCD_D4 34
#define PIN_LCD_D5 36
#define PIN_LCD_D6 38
#define PIN_LCD_D7 40

// Gehäuseknöpfe
#define PIN_BUTTON_SOLAR 22
#define PIN_BUTTON_SIDER 24
#define PIN_BUTTON_LUNAR 26
#define PIN_BUTTON_SCALE_ILLU 28

// Handbedienung
#define PIN_BUTTON_LEFT 23
#define PIN_BUTTON_RIGHT 25
#define PIN_BUTTON_UP 27
#define PIN_BUTTON_DOWN 29

#define PIN_MOTOR_REK_PDN 50
#define PIN_MOTOR_DEK_PDN 52

/*
  Zustandsdefinitionen 
*/

// Displayanzeigen
#define DISPLAY_IDLE 0
#define DISPLAY_SOLAR 1
#define DISPLAY_SIDER 2
#define DISPLAY_LUNAR 3
#define DISPLAY_LEFT 4
#define DISPLAY_RIGHT 5
#define DISPLAY_UP 6
#define DISPLAY_DOWN 7
#define DISPLAY_UNKNOWN 8
#define DISPLAY_TEMPHUM 9
#define DISPLAY_LED_BRIGHTNESS 10
#define DISPLAY_LEFT_UP 11
#define DISPLAY_LEFT_DOWN 12
#define DISPLAY_RIGHT_UP 13
#define DISPLAY_RIGHT_DOWN 14
#define DISPLAY_STATUS 15

// Definition der Modi für die Nachführung
#define TRACKING_IDLE 0
#define TRACKING_SOLAR 1
#define TRACKING_SIDER 2
#define TRACKING_LUNAR 3

/*
  Tastencodedefinitionen 
*/

// Nachführung
#define CODE_SOLAR 1
#define CODE_SIDER 4
#define CODE_LUNAR 16
#define CODE_SCALE_ILLU 64
#define CODE_STATUS 67
#define CODE_LEFT 2
#define CODE_RIGHT 8
#define CODE_UP 32
#define CODE_DOWN 128

// Schieber der Handbedienung
#define SLIDER_TRACKINGSPEED 0
#define SLIDER_LEDBRIGHTNESS 1

/*
  Sonstige Definitionen 
*/

// Es handelt sich um den DHT22 Sensor
//#define DHTTYPE DHT22

// Abfrageintervall Temperatur-/Feuchtigkeitssensor in ms
//#define DHT_INTERVAL_MS 30000
//#define DHT_DISPLAY_INTERVAL_MS 2000

// Zeitdefinitionen
#define ONE_MIN_IN_MS 60000
#define ONE_SEC_IN_MS 1000

// Timeout für Nachführung in Minuten
#define TRACKING_DURATION_MIN 179

// notwendige Differenz zwischen analogen Messwerten, um Status zu aktualisieren
#define ANALOG_IN_THRESHOLD 30

// Tastenentsprellzeit
#define BUTTON_DEBOUNCE_MS 100

// Zeitdauer in Sekunden in der die Skalenbeleuchtung angeschaltet bleibt
#define SCALE_ILLU_DURATION_S 300

// Intervall in Millisekunden in dem der analoge Eingang für die Skalenhelligkeit ausgelesen wird
#define SCALE_ILLU_INPUT_INTERVAL_MS 100

// minimal und maximal einstellbare Helligkeit für Skalenbeleuchtung
#define SCALE_MIN_BRIGHTNESS 5
#define SCALE_MAX_BRIGHTNESS 195

// minimal und maximal einstellbare Helligkeit für Displaybeleuchtung
#define DISPLAY_MIN_BRIGHTNESS 5
#define DISPLAY_MAX_BRIGHTNESS 195

// Intervall in Millisekunden in dem der analoge Eingang für die Displayhelligkeit ausgelesen wird
#define DISPLAY_ILLU_INPUT_INTERVAL_MS 100

// Intervall in Millisekunden in dem der analoge Eingang für die Handbedien-LED ausgelesen wird
#define LED_SLIDER_INPUT_INTERVAL_MS 100

// Intervall in Millisekunden in dem die Handbedien-LED blinken soll, während die Helligkeit eingestellt werden kann
#define LED_BLINK_OFF_MS 1000

// Umschaltdauer bei gedrückter Beleuchtungstaste in Modus für Helligkeitssteuerung der Handbedienungs-LED
#define HANDCONTROL_SWITCH_PERIOD_MS 3000

// Maximale Schrittmotorgeschwindigkeit = minimale Zeitdauer zwischen zwei Schritten
#define MIN_STEPPER_INTERVAL 500

// minimales Intervall bei maximalem Anlogwert um sicher den Maximalgeschwindigkeit zu erreichen, wird aber durch verherige Konstante begrenzt
#define MAX_ANALOG_SCALE_VALUE 400

// Adresse, an der die Helligkeit der Bedienteil-LED im EEPROM gespeichert wird
#define ADDR_LED_BRIGHTNESS 0

// Debuginformationen über die serielle Schnittstelle
#define DEBUG_SERIAL false

// Intervall in Millisekunden um die Motorgeschwindigkeit anzupassen
#define SLOW_START_UPDATE_INTERVAL_MS 2

// Intervall in Millisekunden um die Motorgeschwindigkeit anzupassen
#define SLOW_START_SPEED_DIFF 10

// Umlaufzeiten für die Nachführungsmodi
// Sekunden fuer einen Umlauf / Übersetzung Schrittmotor / Übersetzung Umlenkung / Übersetzung Schneckengetriebe
const double rotDurSolar = 86400.0 / 100.27 / 1.32 / 360.0;
const double rotDurSider = 86164.0 / 100.27 / 1.32 / 360.0;
const double rotDurLunar = 89441.0 / 100.27 / 1.32 / 360.0;

// = 16000000 MHz / 200 STEPS_PER_REVOLUTION / 16 Sechzehntelschritte / 2 Flankenwechsel
const double scaler = 2500.0;

// Array mit Umlaufzeiten, Index passend zu TRACKING_xyz Status
const unsigned int rotDuration[] = { 0, scaler * rotDurSolar, scaler * rotDurSider, scaler * rotDurLunar };

/*
  Variablendefinitionen 
*/

// Zustandsvariablen für Displayanzeige und Nachführung
volatile byte statusDisplay = DISPLAY_IDLE;
byte statusTracking = TRACKING_IDLE;

// Variablen für Tastenanschläge
unsigned int statusTasten = 0;
unsigned long timerTasten = 0;

// Variablen für sanften Anlauf
unsigned long timerSlowStart = 0;
int timerSlowStartValueRek = 0;
int timerSlowStartValueDek = 0;

// Variablen für automatische Skalenbeleuchtungsabschaltung
byte statusScaleIllu = 0;
unsigned long timerScaleIllu;
unsigned long timerReadScaleIlluInput;
unsigned long timerReadDisplayIlluInput = 0;

// Variablen für automatische Nachführungsabschaltung
unsigned long timerTracking = 0;
int durationNachf = -1;
unsigned long timerTrackingThreshold = 0;

// Variablen für blinkenden Doppelpunkt
byte statusTrackingColonState = 0;
unsigned long timerTrackingColonThreshold = 0;

// Variable für analogen Messwert für manuelle Nachführung
//unsigned int manualTrackingSpeed = 2000;
byte timerActiveRek = 0;
byte timerActiveDek = 0;

// Teilschritt für die Nachführung durch Timer gesteuert
volatile boolean stepRek = false;
volatile boolean stepDek = false;

// Status für Schieber der Handbedienung
byte statusSlider = SLIDER_TRACKINGSPEED;
unsigned long timerReadSliderInput;
unsigned long timerLEDblink = 0;

// Variable für Luftfeuchtigkeit-/ Temperaturmessung
//unsigned long timerDHT22 = 0;

// Displayanzeigen
// Zeile 1
char displayItems[][17] = { "Bereit          ", "Solar        :  ", "Siderisch    :  ", "Lunar        :  ", "Rektasz. -  ", "Rektasz. +  ", "Deklinat. - ", "Deklinat. + ", "unbek. Tastenk. ", "Temperatur", "LED-Hellig: ", "Rek-, Dek-  ", "Rek-, Dek+  ", "Rek+, Dek-  ", "Rek+, Dek+  ", "V1.01 20.03.2018"};

// Zeile 2
char trackingOnOff[][17] = { "Nachfuehrung aus", "Nachfuehrung ein" };

// Objekt für LCD-Anzeige
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_ENABLE, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);         

// Objekt für Temperatur-/Luftfeuchtigkeitssensor
//DHT dht(PIN_DHT22, DHTTYPE); //Der Sensor wird ab jetzt mit „dth“ angesprochen

//TMC2208Stepper driverRek = TMC2208Stepper(50, 51);
//TMC2208Stepper driverDek = TMC2208Stepper(52, 53);
/** Initialisierungen beim Programmstart */
void setup() {
//    Serial.begin(115200);
  if (DEBUG_SERIAL) {
    // Serielle Schnittstelle initialisieren
    Serial.begin(115200);
    Serial.println("Sternwarte Ueberlingen e.V.");
    Serial.print("Timer solar: ");
    Serial.println(rotDuration[TRACKING_SOLAR]);
    Serial.print("Timer siderisch: ");
    Serial.println(rotDuration[TRACKING_SIDER]);
    Serial.print("Timer lunar: ");
    Serial.println(rotDuration[TRACKING_LUNAR]);
  }
  
  // PINs initialisieren
  pinMode(PIN_HANDCONTROL_LEDBRIGHTNESS, OUTPUT);
  pinMode(PIN_SCALE_ILLU, OUTPUT);
  pinMode(PIN_DISPLAY_ILLU, OUTPUT);

  pinMode(PIN_SCALE_POTI, INPUT);
  pinMode(PIN_TRACKING_SLIDER, INPUT);
  pinMode(PIN_DISPLAY_POTI, INPUT);
  
  pinMode(PIN_BUTTON_SOLAR, INPUT);        // definiert pin 39 als Eingang (Taster "Solar")
  pinMode(PIN_BUTTON_SIDER, INPUT);        // definiert pin 41 als Eingang (Taster "SIDER")
  pinMode(PIN_BUTTON_LUNAR, INPUT);        // definiert pin 43 als Eingang (Taster "Lunar")
  pinMode(PIN_BUTTON_SCALE_ILLU, INPUT);        // definiert pin 45 als Eingang (Taster "LED Beleuchtung")
  
  pinMode(PIN_BUTTON_LEFT, INPUT);        // definiert pin 47 als Eingang (Taster 1 = links)
  pinMode(PIN_BUTTON_RIGHT, INPUT);        // definiert pin 49 als Eingang (Taster 2 = rechts)
  pinMode(PIN_BUTTON_UP, INPUT);        // definiert pin 51 als Eingang (Taster 3 = hoch)
  pinMode(PIN_BUTTON_DOWN, INPUT);        // definiert pin 53 als Eingang (Taster 4 = runter)

  digitalWrite(PIN_SCALE_ILLU, LOW);
  digitalWrite(PIN_HANDCONTROL_LEDBRIGHTNESS, LOW);

  // Display initialisieren  
  digitalWrite(PIN_DISPLAY_ILLU, HIGH);
  lcd.begin(16, 2);          // definiert die Anzahl der Zeichen und Zeilen des LCD-Displays
  lcd.clear();               // löscht die Anzeige bei bzw. vor Programmstart

  // für zwei Sekunden "Sternwarte Ueberlingen e.V." anzeigen
  lcd.setCursor(0, 0);
  lcd.print("Sternwarte");
  lcd.setCursor(0, 1);
  lcd.print("Ueberlingen e.V.");

/*delay(2000);

  driverRek.beginSerial(115200);

  // Push at the start of setting up the driver resets the register to default
  driverRek.push();

    uint8_t result = driverRek.test_connection();
    if (result) {
        Serial.println(F("failed!"));
        Serial.print(F("Likely cause: "));
        switch(result) {
            case 1: Serial.println(F("loose connection")); break;
            case 2: Serial.println(F("Likely cause: no power")); break;
        }
        Serial.println(F("Fix the problem and reset board."));
        //abort();
    }
    Serial.println(F("RekOK"));

  driverRek.pdn_disable(true);     // Use PDN/UART pin for communication
  driverRek.I_scale_analog(false); // Use internal voltage reference
  driverRek.rms_current(100);      // Set driver current = 500mA, 0.5 multiplier for hold current and RSENSE = 0.11.
  driverRek.toff(2);               // Enable driver in software

  driverDek.beginSerial(115200);

  // Push at the start of setting up the driver resets the register to default
  driverDek.push();

    result = driverDek.test_connection();
    if (result) {
        Serial.println(F("failed!"));
        Serial.print(F("Likely cause: "));
        switch(result) {
            case 1: Serial.println(F("loose connection")); break;
            case 2: Serial.println(F("Likely cause: no power")); break;
        }
        Serial.println(F("Fix the problem and reset board."));
        //abort();
    }
    Serial.println(F("DekOK"));

  driverDek.pdn_disable(true);     // Use PDN/UART pin for communication
  driverDek.I_scale_analog(false); // Use internal voltage reference
  driverDek.rms_current(100);      // Set driver current = 500mA, 0.5 multiplier for hold current and RSENSE = 0.11.
  driverDek.toff(2);               // Enable driver in software
*/

delay(2000);
  
  // Schrittmotorpins initialisieren;
  pinMode(PIN_MOTOR_REK_DIR, OUTPUT);
  pinMode(PIN_MOTOR_REK_STEP, OUTPUT);
  pinMode(PIN_MOTOR_DEK_DIR, OUTPUT);
  pinMode(PIN_MOTOR_DEK_STEP, OUTPUT);
  digitalWrite(PIN_MOTOR_REK_DIR, REK_ROT_CW);
  digitalWrite(PIN_MOTOR_REK_STEP, stepRek);
  digitalWrite(PIN_MOTOR_DEK_DIR, DEK_ROT_CW);
  digitalWrite(PIN_MOTOR_DEK_STEP, stepDek);
  
  delay(2000);
  pinMode(PIN_MOTOR_REK_PDN, OUTPUT);
  digitalWrite(PIN_MOTOR_REK_PDN, LOW);
  pinMode(PIN_MOTOR_DEK_PDN, OUTPUT);
  digitalWrite(PIN_MOTOR_DEK_PDN, LOW);

  //DHT22 Sensor initialisieren
//  pinMode(PIN_DHT22, INPUT);
//  dht.begin();
  
  // Bereitzustand anzeigen
  lcd.clear();
  setDisplayStatus(DISPLAY_IDLE);
  analogWrite(PIN_HANDCONTROL_LEDBRIGHTNESS, EEPROM.read(ADDR_LED_BRIGHTNESS));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
}

/** Nachführungsgeschwindigkeit anzeigen */
void displayTrackingSpeed(unsigned int speed) {
  char s[10];
  lcd.setCursor(11, 0);
  sprintf(s, "%4d%%", map(speed, 0, 1020, 8, 100));
  lcd.print(s);
}

/** Restzeit für die Nachführung anzeigen */
void displayRestzeit() {
  int hour = durationNachf / 60;
  int minute = durationNachf % 60;
  lcd.setCursor(12,0);
  lcd.print(hour);
  lcd.setCursor(14,0);
  if (minute < 10) lcd.print("0");
  lcd.print(minute);
}

/** Nachführungsmodi setzen */
void setTrackingStatus(byte newState) {
  if (statusDisplay != newState) {
    stopTimer();
    startTimer(rotDuration[newState]);
    if (statusTracking != newState) {
      timerTracking = millis();
      timerTrackingThreshold = timerTracking + ONE_MIN_IN_MS;
      statusTrackingColonState = 1;
      timerTrackingColonThreshold = timerTracking + ONE_SEC_IN_MS;
      durationNachf = TRACKING_DURATION_MIN;
    }
    statusTracking = newState;
    setDisplayStatus(newState);
    displayRestzeit();
  } else {
    stopTimer();
    timerTracking = 0;
    statusTracking = TRACKING_IDLE;
    setDisplayStatus(DISPLAY_IDLE);
  }
}

/** Skalenbeleuchtung ein/ausschalten */
void triggerScaleIllu(unsigned long milliSecs) {
  if (statusScaleIllu == 0) {
    // triggerScaleIllu einschalten und Timer zurücksetzen
    statusScaleIllu = 1;
    timerScaleIllu = milliSecs + (SCALE_ILLU_DURATION_S * 1000.0);
    timerReadScaleIlluInput = 0;
    if (DEBUG_SERIAL) Serial.println("triggerScaleIllu an");
  } else {
    statusScaleIllu = 0;
    digitalWrite(PIN_SCALE_ILLU, LOW);
    if (DEBUG_SERIAL) Serial.println("triggerScaleIllu aus");
  }
}

/** Status im Display anzeigen */
void setDisplayStatus(byte newState) {
  lcd.setCursor(0, 0);
  lcd.print(displayItems[newState]);
  statusDisplay = newState;
  lcd.setCursor(0, 1);
  lcd.print(trackingOnOff[statusTracking != TRACKING_IDLE]);
}

/** Leerlaufzustand */
void statusIdle() {
  stopTimer2();

  // falls Nachführung aktiv -> Display aktualisieren
  if (statusTracking == TRACKING_SOLAR) {
    if (statusDisplay != DISPLAY_SOLAR) setTrackingStatus(TRACKING_SOLAR);
    return;
  }
  if (statusTracking == TRACKING_SIDER) {
    if (statusDisplay != DISPLAY_SIDER) setTrackingStatus(TRACKING_SIDER);
    return;
  }
  if (statusTracking == TRACKING_LUNAR) {
    if (statusDisplay != DISPLAY_LUNAR) setTrackingStatus(TRACKING_LUNAR);
    return;
  }

  // keine Nachführung aktiv, Display löschen, Timer anhalten
  setDisplayStatus(DISPLAY_IDLE);
  stopTimer();
}

/** Timer für Nachführung starten */
void startTimer(unsigned int timer) {
  // see http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  
  // set TIMER 5 for interrupt frequency
  
  cli(); // stop interrupts
  TCCR5A = 0; // set entire TCCR1A register to 0
  TCCR5B = 0; // same for TCCR1B
  TCNT5  = 0; // initialize counter value to 0
  // set compare match register
  OCR5A = timer;
  // turn on CTC mode
  TCCR5B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for no prescaler
  TCCR5B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK5 |= (1 << OCIE5A);
  timerActiveRek = 1;
  sei(); // allow interrupts
}

/** Interruptvektor für Nachführung */
ISR(TIMER5_COMPA_vect){
  cli(); // stop interrupts
  // wenn der Timer für die Nachführung abgelaufen ist, Schrittmotor stellen
  stepRek = !stepRek;
  digitalWrite(PIN_MOTOR_REK_STEP, stepRek);
  sei(); // allow interrupts
}

/** Timer für Nachführung anhalten */
void stopTimer() {
  // erase clock source bits
  TCCR5B &= ~(1<< CS12 | 1<< CS11 | 1<< CS10);
  timerActiveRek = 0;
  timerSlowStartValueRek = 0;
}

/** Timer für Nachführung starten */
void startTimer2(unsigned int timer) {
  // see http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  
  // set TIMER 4 for interrupt frequency
  
  cli(); // stop interrupts
  TCCR4A = 0; // set entire TCCR1A register to 0
  TCCR4B = 0; // same for TCCR1B
  TCNT4  = 0; // initialize counter value to 0
  // set compare match register
  OCR4A = timer;
  // turn on CTC mode
  TCCR4B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for no prescaler
  TCCR4B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK4 |= (1 << OCIE4A);
  timerActiveDek = 1;
  sei(); // allow interrupts
}

/** Interruptvektor für Nachführung */
ISR(TIMER4_COMPA_vect){
  cli(); // stop interrupts
  // wenn der Timer für die Nachführung abgelaufen ist, Schrittmotor stellen
  stepDek = !stepDek;
  digitalWrite(PIN_MOTOR_DEK_STEP, stepDek);
  sei(); // allow interrupts
}

/** Timer für Nachführung anhalten */
void stopTimer2() {
  // erase clock source bits
  TCCR4B &= ~(1<< CS12 | 1<< CS11 | 1<< CS10);
  timerActiveDek = 0;
  timerSlowStartValueDek = 0;
}

/** Tasten auswerten */
void evalButtons(unsigned long milliSecs) {
  // Tasten entprellen
  if (timerTasten + BUTTON_DEBOUNCE_MS >= milliSecs) return;
  
  // Abfragen der gedrückten Taste(n)
  unsigned int tstat = PINA; // temporärer Tastenstatus
  
  // keine neue Taste gedrückt -> raus
  if (tstat == statusTasten) {
    // außer wenn die Reglerbleuchtung länger gedrückt wird, dann umstellen auf Helligkeitsanpassung für die Handbedienung
    if ((tstat == CODE_SCALE_ILLU) && (timerTasten + HANDCONTROL_SWITCH_PERIOD_MS <= milliSecs)) {
      if (statusSlider == SLIDER_LEDBRIGHTNESS) {
        statusSlider = SLIDER_TRACKINGSPEED;
        // Wert im EEPROM speichern
        unsigned int brightness = analogRead(PIN_TRACKING_SLIDER) >> 2;
        analogWrite(PIN_HANDCONTROL_LEDBRIGHTNESS, brightness);
        EEPROM.update(ADDR_LED_BRIGHTNESS, brightness);
        if (DEBUG_SERIAL) Serial.println("statusSlider TrackingSpeed");
        statusIdle();
      } else {
        statusSlider = SLIDER_LEDBRIGHTNESS;
        setDisplayStatus(DISPLAY_LED_BRIGHTNESS);
        if (DEBUG_SERIAL) Serial.println("statusSlider LED Brightness");
      }
      triggerScaleIllu(milliSecs);
    } else {
      return;
    }
  }
  
  timerTasten = milliSecs;
  statusTasten = tstat;

  if (DEBUG_SERIAL) {
    Serial.print("Tastencode: ");
    Serial.println(tstat);
  }
  if (statusSlider == SLIDER_LEDBRIGHTNESS) {
    return;
  }
  
  // Tastenkombination auswerten
  switch (tstat) {
    case 0:
      digitalWrite(PIN_MOTOR_REK_DIR, REK_ROT_CW);
      if (statusSlider != SLIDER_LEDBRIGHTNESS) statusIdle();
      break;
    case CODE_SOLAR:
      digitalWrite(PIN_MOTOR_REK_DIR, REK_ROT_CW);
      setTrackingStatus(TRACKING_SOLAR);
      break;
    case CODE_SIDER:
      digitalWrite(PIN_MOTOR_REK_DIR, REK_ROT_CW);
      setTrackingStatus(TRACKING_SIDER);
      break;
    case CODE_LUNAR:
      digitalWrite(PIN_MOTOR_REK_DIR, REK_ROT_CW);
      setTrackingStatus(TRACKING_LUNAR);
      break;
    case CODE_SCALE_ILLU:
      triggerScaleIllu(milliSecs);
      break;
    case CODE_STATUS:
      char line[17];
      lcd.setCursor(0, 0);
      lcd.print(displayItems[DISPLAY_STATUS]);
      EEPROM.get(ADDR_LED_BRIGHTNESS+18, line);
      lcd.setCursor(0, 1);
      lcd.print(line);
      delay(2000);
      break;
    case CODE_LEFT:
      digitalWrite(PIN_MOTOR_REK_DIR, REK_ROT_CCW);
      setDisplayStatus(DISPLAY_LEFT);
      break;
    case CODE_RIGHT:
      digitalWrite(PIN_MOTOR_REK_DIR, REK_ROT_CW);
      setDisplayStatus(DISPLAY_RIGHT);
      break;
    case CODE_UP:
      digitalWrite(PIN_MOTOR_DEK_DIR, DEK_ROT_CCW);
      setDisplayStatus(DISPLAY_UP);
      break;
    case CODE_DOWN:
      digitalWrite(PIN_MOTOR_DEK_DIR, DEK_ROT_CW);
      setDisplayStatus(DISPLAY_DOWN);
      break;
    case CODE_LEFT + CODE_UP:
      digitalWrite(PIN_MOTOR_REK_DIR, REK_ROT_CCW);
      digitalWrite(PIN_MOTOR_DEK_DIR, DEK_ROT_CCW);
      setDisplayStatus(DISPLAY_LEFT_UP);
      break;
    case CODE_LEFT + CODE_DOWN:
      digitalWrite(PIN_MOTOR_REK_DIR, REK_ROT_CCW);
      digitalWrite(PIN_MOTOR_DEK_DIR, DEK_ROT_CW);
      setDisplayStatus(DISPLAY_LEFT_DOWN);
      break;
    case CODE_RIGHT + CODE_UP:
      digitalWrite(PIN_MOTOR_REK_DIR, REK_ROT_CW);
      digitalWrite(PIN_MOTOR_DEK_DIR, DEK_ROT_CCW);
      setDisplayStatus(DISPLAY_RIGHT_UP);
      break;
    case CODE_RIGHT + CODE_DOWN:
      digitalWrite(PIN_MOTOR_REK_DIR, REK_ROT_CW);
      digitalWrite(PIN_MOTOR_DEK_DIR, DEK_ROT_CW);
      setDisplayStatus(DISPLAY_RIGHT_DOWN);
      break;
    default:
      setDisplayStatus(DISPLAY_UNKNOWN);
  }
}

/** Hauptroutine */
void loop() {
  char s[10];
  unsigned long milliSecs = millis();

  // gedrückte Tasten auswerten
  evalButtons(milliSecs);
  
  // Luftfeuchtigkeit und Temperatur regelmäßig auslesen
/*  if (timerDHT22 + DHT_INTERVAL_MS <= milliSecs) {
    // nur im Leerlauf messen, weil die Messung sonst die Nachführung ausbremst
    if (statusDisplay == DISPLAY_IDLE) {
      float Humidity = dht.readHumidity(); //die Luftfeuchtigkeit auslesen und unter „Humidity“ speichern
    
      float Temperature = dht.readTemperature();//die Temperatur auslesen und unter „Temperature“ speichern
    
      if (DEBUG_SERIAL) {
        Serial.print("Luftfeuchtigkeit: "); //Im seriellen Monitor den Text und 
        Serial.print(Humidity); //die dazugehörigen Werte anzeigen
        Serial.print(" % / Temperatur: ");
        Serial.print(Temperature);
        Serial.println(" C");
      }
    
      // Nachkommastellen abschneiden    
      int t = Temperature;
      int h = Humidity;  
    
      lcd.setCursor(0, 0);
      lcd.print("Temperatur: ");
      lcd.print(t);
      lcd.print("C  ");
      lcd.setCursor(0, 1);
      lcd.print("Luftfeucht: ");
      lcd.print(h);
      lcd.print("%  ");
      
      statusDisplay = DISPLAY_TEMPHUM;
    }
    
    timerDHT22 = milliSecs;
  }
  
  // nach kurzer Zeit Temperatur-/Feuchtigkeitsanzeige wieder löschen
  if ((statusDisplay == DISPLAY_TEMPHUM) && (timerDHT22 + DHT_DISPLAY_INTERVAL_MS <= milliSecs)) {
    setDisplayStatus(DISPLAY_IDLE);
  }*/
  
  // falls triggerScaleIllu eingeschaltet ist, regelmäßig Analogeingang auslesen und Helligkeit setzen
  if (statusScaleIllu == 1) {
    if (timerScaleIllu <= milliSecs) {
      statusScaleIllu = 0;
      digitalWrite(PIN_SCALE_ILLU, LOW);
      if (DEBUG_SERIAL) Serial.println("triggerScaleIllu aus");
    } else {
      if (timerReadScaleIlluInput <= milliSecs) {
        analogWrite(PIN_SCALE_ILLU, map(analogRead(PIN_SCALE_POTI), 0, 1023, SCALE_MIN_BRIGHTNESS, SCALE_MAX_BRIGHTNESS));
        if (DEBUG_SERIAL) {
          Serial.print("ScaleIllu: ");
          Serial.println(analogRead(PIN_SCALE_POTI) >> 2);
        }
        timerReadScaleIlluInput = milliSecs + SCALE_ILLU_INPUT_INTERVAL_MS;
      }
    }
  }

  // in regelmäßigen Intervallen Analogeingang für Displayhelligkeit auslesen und Ausgang neu setzen
  if (timerReadDisplayIlluInput <= milliSecs) {
    analogWrite(PIN_DISPLAY_ILLU, map(analogRead(PIN_DISPLAY_POTI), 0, 1023, DISPLAY_MIN_BRIGHTNESS, DISPLAY_MAX_BRIGHTNESS));
    if (DEBUG_SERIAL) {
      Serial.print("DisplayIllu: ");
      Serial.println(analogRead(PIN_DISPLAY_POTI) >> 2);
    }
    timerReadDisplayIlluInput = milliSecs + DISPLAY_ILLU_INPUT_INTERVAL_MS;
  }

  // falls sliderStatus auf SLIDER_LEDBRIGHTNESS steht, Analogeingang auslesen und Helligkeit an der Handbedienung setzen
  if (statusSlider == SLIDER_LEDBRIGHTNESS) {
    unsigned int brightness = analogRead(PIN_TRACKING_SLIDER) >> 2;
    // LED blinken lassen, so dass der Modus und das Helligkeitsverhältniss zuverlässig erkannt werden können
    if (timerLEDblink <= milliSecs) {
        if (brightness > 30)
          analogWrite(PIN_HANDCONTROL_LEDBRIGHTNESS, 0);
        else
          analogWrite(PIN_HANDCONTROL_LEDBRIGHTNESS, 255);
        timerLEDblink = milliSecs + LED_BLINK_OFF_MS;
    } else {
      if (timerReadSliderInput <= milliSecs) {
        unsigned int brightness = analogRead(PIN_TRACKING_SLIDER) >> 2;
        analogWrite(PIN_HANDCONTROL_LEDBRIGHTNESS, brightness);
        sprintf(s, "%3d", map(brightness, 0, 255, 0, 100));
        lcd.setCursor(12, 0);
        lcd.print(s);
        lcd.print("%");
        timerReadSliderInput = milliSecs + LED_SLIDER_INPUT_INTERVAL_MS;
      }
    }
  }

  // falls einer der Steuerungsknöpfe gedrückt ist, Analogeingang auslesen und Geschwindigkeit setzen
  if (statusDisplay >= DISPLAY_LEFT && statusDisplay <= DISPLAY_DOWN && timerSlowStart <= milliSecs) {
    // && abs(speed - timerSlowStartValueRek) > ANALOG_IN_THRESHOLD) 
    unsigned int speed = analogRead(PIN_TRACKING_SLIDER);
    // da Rauschen auf dem Analogeingang vorhanden: nur wenn sich der neue Wert von der aktuellen 
    // Geschwindigkeit signifikant unterscheidet, neue Geschwindigkeit berechnen
    if (statusDisplay == DISPLAY_LEFT || statusDisplay == DISPLAY_RIGHT) {
      if (timerActiveDek == 1) {
        // Deklinationsmotor anhalten
        stopTimer2();
      }
      
      // Geschwindigkeit langsam anpassen
      if (speed > timerSlowStartValueRek) {
        timerSlowStartValueRek += min(SLOW_START_SPEED_DIFF, speed - timerSlowStartValueRek);
      } else {
        timerSlowStartValueRek -= min(SLOW_START_SPEED_DIFF, timerSlowStartValueRek - speed);
      }
      int timer = timerSlowStartValueRek;
      if (timerActiveRek == 1) {
        stopTimer();
      }
      timerSlowStartValueRek = timer;
      // neue Geschwindigkeit setzen
      // Analogwert so skalieren, dass auf jeden Fall 100% erreicht werden, der Motor aber noch zuverlässig läuft
      startTimer(max(map(timerSlowStartValueRek, 0, 1024, rotDuration[1], MAX_ANALOG_SCALE_VALUE), MIN_STEPPER_INTERVAL));
      displayTrackingSpeed(timerSlowStartValueRek);
    }
    if (statusDisplay == DISPLAY_DOWN || statusDisplay == DISPLAY_UP) {
      if (timerActiveRek == 1) {
        // Rekaszensionsmotor anhalten
        stopTimer();
      }
      // Geschwindigkeit langsam anpassen
      if (speed > timerSlowStartValueDek) {
        timerSlowStartValueDek += min(SLOW_START_SPEED_DIFF, speed - timerSlowStartValueDek);
      } else {
        timerSlowStartValueDek -= min(SLOW_START_SPEED_DIFF, timerSlowStartValueDek - speed);
      }
      int timer = timerSlowStartValueDek;
      if (timerActiveDek == 1) {
        stopTimer2();
      }
      timerSlowStartValueDek = timer;
      // neue Geschwindigkeit setzen
      // Analogwert so skalieren, dass auf jeden Fall 100% erreicht werden, der Motor aber noch zuverlässig läuft
      startTimer2(max(map(timerSlowStartValueDek, 0, 1024, rotDuration[1], MAX_ANALOG_SCALE_VALUE), MIN_STEPPER_INTERVAL));
      displayTrackingSpeed(timerSlowStartValueDek);
    }
    timerSlowStart = milliSecs + SLOW_START_UPDATE_INTERVAL_MS;
  }
  
  // Doppelpunkt im Sekundentakt blinken lassen
  if ((statusDisplay >= DISPLAY_SOLAR) && (statusDisplay <= DISPLAY_LUNAR) && (timerTracking > 0) && (timerTrackingColonThreshold <= milliSecs)) {
    lcd.setCursor(13,0);
    if (statusTrackingColonState == 0) {
      statusTrackingColonState = 1;
      lcd.print(":");
    } else {
      statusTrackingColonState = 0;
      lcd.print(" ");
    }
    timerTrackingColonThreshold += ONE_SEC_IN_MS;  
  }

  // Herunterzählen der verbleibenden Nachführungsdauer
  if ((timerTracking > 0) && (timerTrackingThreshold <= milliSecs)) {
    durationNachf--;
    // wenn Timer abgelaufen ist, Nachführung ausschalten, ansonsten Restzeit anzeigen und Timer um eine Minute erhöhren
    if (durationNachf <= 0) {
      timerTracking = 0;
      statusTracking = TRACKING_IDLE;
      statusIdle();
    } else {  
      displayRestzeit();
      timerTrackingThreshold += ONE_MIN_IN_MS;
    }
  }
}
