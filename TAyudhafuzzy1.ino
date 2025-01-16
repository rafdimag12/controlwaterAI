#include <Arduino.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "DFRobot_PH.h"
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Fuzzy.h>

#define IN1 3 // deklarasi pin IN1
#define IN2 4  // deklarasi pin IN2
#define IN3 5  // deklarasi pin IN3
#define IN4 6  // deklarasi pin IN4
#define ENA A10 // deklarasi pin ENA
#define ENB A12  // deklarasi pin ENB

#define PH_PIN A2
#define DO_PIN A1
#define ONE_WIRE_BUS 8

#define VREF 5000    //VREF (mv)
#define ADC_RES 1024 //ADC Resolution

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 0

#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (1600) //mv
#define CAL1_T (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL2_V (1300) //mv
#define CAL2_T (15)   //℃



LiquidCrystal_I2C lcd(0x27, 16, 2);
const int pompaPh = 6;
const int PIN = 9;

const uint16_t DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
}

uint8_t Temperaturet;
uint16_t ADC_Raw;
uint16_t ADC_Voltage;
uint16_t DO;

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
#if TWO_POINT_CALIBRATION == 0
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#else
  uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL2_T) * ((uint16_t)CAL1_V - CAL2_V) / ((uint8_t)CAL1_T - CAL2_T) + CAL2_V;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
#endif
}

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorSuhu(&oneWire);
float voltage,phValue ;
DFRobot_PH ph;

int pwmasam;
int in1asam;
int pwmbasa;

// Objek fuzzy
Fuzzy* fuzzy = new Fuzzy();

// Definisi variabel
float pHLevel;
FuzzyInput* pHLevelInput = new FuzzyInput(1);
FuzzyOutput* enaOutput = new FuzzyOutput(1);
FuzzyOutput* enbOutput = new FuzzyOutput(1);

void setup()
{
  Serial.begin(115200);
  ph.begin();
  sensorSuhu.begin();
  pinMode(PIN, HIGH);
  pinMode(pompaPh, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  lcd.init(); // Inisialisasi LCD
  lcd.backlight(); // Nyalakan backlight LCD

//  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
 // digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

 // Input Fuzzy Sets Basa
FuzzySet *phbesar1 = new FuzzySet(8, 8.5, 9.5, 10);
FuzzySet *phbesar0 = new FuzzySet(7.5, 7.8 , 7.9, 7.9 );
FuzzySet *phnormalb = new FuzzySet(7, 7, 7, 7);

 // Input Fuzzy Sets asam
FuzzySet *phKecil = new FuzzySet(0, 1, 2, 3);
FuzzySet *phSedang = new FuzzySet(4, 5 , 6, 6.9 );
FuzzySet *phnormala = new FuzzySet(7, 7, 7, 7);

// Output Fuzzy Sets for PWM (Motor Speed)
FuzzySet *pwmHighasam = new FuzzySet(255, 255, 205, 160);
FuzzySet *pwmMediumasam = new FuzzySet(180, 130, 70, 10);
FuzzySet *pwmLowasam = new FuzzySet(0, 0, 0, 0);

FuzzySet *pwmHighbasa = new FuzzySet(255, 255, 205, 160);
FuzzySet *pwmMediumbasa = new FuzzySet(180, 130, 70, 10);
FuzzySet *pwmLowbasa = new FuzzySet(0, 0, 0, 0);


// Output Fuzzy Sets for another control (e.g., LED Brightness)
FuzzySet *in1low = new FuzzySet(HIGH, HIGH, HIGH, HIGH);
FuzzySet *in1med = new FuzzySet(HIGH, HIGH, HIGH, HIGH);
FuzzySet *in1high= new FuzzySet(HIGH, HIGH, HIGH, HIGH);

// Output Fuzzy Sets for another control (e.g., LED Brightness)
FuzzySet *in3low = new FuzzySet(HIGH, HIGH, HIGH, HIGH);
FuzzySet *in3med = new FuzzySet(HIGH, HIGH, HIGH, HIGH);
FuzzySet *in3high= new FuzzySet(HIGH, HIGH, HIGH, HIGH);

// Setup Fuzzy Input
FuzzyInput *acidity = new FuzzyInput(1);
acidity->addFuzzySet(phKecil);
acidity->addFuzzySet(phSedang);
acidity->addFuzzySet(phnormala);
acidity->addFuzzySet (phbesar1);
acidity->addFuzzySet (phbesar0);
acidity->addFuzzySet (phnormalb);
fuzzy->addFuzzyInput(acidity);

// Setup Fuzzy Outputs asam
FuzzyOutput *pwmOutputasam = new FuzzyOutput(1);
pwmOutputasam->addFuzzySet(pwmLowasam);
pwmOutputasam->addFuzzySet(pwmMediumasam);
pwmOutputasam->addFuzzySet(pwmHighasam);
fuzzy->addFuzzyOutput(pwmOutputasam);



FuzzyOutput *in1output = new FuzzyOutput(2);
in1output->addFuzzySet(in1low);
in1output->addFuzzySet(in1med);
in1output->addFuzzySet(in1high);
fuzzy->addFuzzyOutput(in1output);

// Setup Fuzzy Outputs asam
FuzzyOutput *pwmOutputbasa = new FuzzyOutput(3);
pwmOutputasam->addFuzzySet(pwmLowbasa);
pwmOutputasam->addFuzzySet(pwmMediumbasa);
pwmOutputasam->addFuzzySet(pwmHighbasa);
fuzzy->addFuzzyOutput(pwmOutputbasa);

FuzzyOutput *in3output = new FuzzyOutput(4);
in1output->addFuzzySet(in3low);
in1output->addFuzzySet(in3med);
in1output->addFuzzySet(in3high);
fuzzy->addFuzzyOutput(in3output);


// Fuzzy Rules
FuzzyRuleAntecedent *ifphKecil = new FuzzyRuleAntecedent();
ifphKecil->joinSingle(phKecil);
FuzzyRuleConsequent *thenPwmHighAndin1high = new FuzzyRuleConsequent();
thenPwmHighAndin1high->addOutput(pwmHighasam);
thenPwmHighAndin1high->addOutput(in1high);
FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifphKecil, thenPwmHighAndin1high);
fuzzy->addFuzzyRule(fuzzyRule1);

FuzzyRuleAntecedent *ifphSedang = new FuzzyRuleAntecedent();
ifphSedang->joinSingle(phSedang);
FuzzyRuleConsequent *thenPwmMediumAndin1med = new FuzzyRuleConsequent();
thenPwmMediumAndin1med->addOutput(pwmMediumasam);
thenPwmMediumAndin1med->addOutput(in1med);
FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifphSedang, thenPwmMediumAndin1med);
fuzzy->addFuzzyRule(fuzzyRule2);

FuzzyRuleAntecedent *ifphnormala = new FuzzyRuleAntecedent();
ifphnormala->joinSingle(phnormala);
FuzzyRuleConsequent *thenPwmLowAndin1low = new FuzzyRuleConsequent();
thenPwmLowAndin1low->addOutput(pwmLowasam);
FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifphnormala, thenPwmLowAndin1low);
fuzzy->addFuzzyRule(fuzzyRule3);

FuzzyRuleAntecedent *ifphbesar1 = new FuzzyRuleAntecedent();
ifphbesar1->joinSingle(phbesar1);
FuzzyRuleConsequent *thenPwmHighbasaAndin1low = new FuzzyRuleConsequent();
thenPwmHighbasaAndin1low->addOutput(pwmHighbasa);
thenPwmHighbasaAndin1low->addOutput(in3low);
FuzzyRule *fuzzyRule4 = new FuzzyRule(4, ifphbesar1, thenPwmHighbasaAndin1low);
fuzzy->addFuzzyRule(fuzzyRule4);

FuzzyRuleAntecedent *ifphbesar0 = new FuzzyRuleAntecedent();
ifphbesar0->joinSingle(phbesar0);
FuzzyRuleConsequent *thenPwmHighbasa0Andin1low = new FuzzyRuleConsequent();
thenPwmHighbasaAndin1low->addOutput(pwmMediumbasa);
thenPwmHighbasaAndin1low->addOutput(in3low);
FuzzyRule *fuzzyRule5 = new FuzzyRule(5, ifphbesar0, thenPwmHighbasaAndin1low);
fuzzy->addFuzzyRule(fuzzyRule5);

FuzzyRuleAntecedent *ifphnormalb = new FuzzyRuleAntecedent();
ifphnormalb->joinSingle(phnormalb);
FuzzyRuleConsequent *thenPwmLowbasaAndin1low = new FuzzyRuleConsequent();
thenPwmLowbasaAndin1low->addOutput(pwmLowbasa);

FuzzyRule *fuzzyRule6 = new FuzzyRule(6, ifphnormalb, thenPwmLowbasaAndin1low);
fuzzy->addFuzzyRule(fuzzyRule6);


}


void loop()
{
  sensorSuhu.requestTemperatures();
  float s = sensorSuhu.getTempCByIndex(0);
  float Do = readDO(ADC_Voltage, Temperaturet);
  float Oks = Do  / 1000 ;
 
  fuzzy->setInput(1, phValue);
  fuzzy->fuzzify();
  
  pwmasam = fuzzy->defuzzify(1);
  in1asam = fuzzy->defuzzify(2);
  pwmbasa = fuzzy->defuzzify(3);

  analogWrite(ENA, pwmasam);
  digitalWrite(IN1, in1asam);
  analogWrite(ENB,pwmbasa);

  Serial.print("ph: ");
  Serial.println(phValue);
  Serial.print("pwm asam: ");
  Serial.println(pwmasam);
  Serial.print("pwm basa: ");
  Serial.println(pwmbasa);

  Temperaturet = (uint8_t)READ_TEMP;
  ADC_Raw = analogRead(DO_PIN);
  ADC_Voltage = uint32_t(VREF) * ADC_Raw / ADC_RES;

  static unsigned long timepoint = millis();  
  
  ph.calibration(voltage,s);   
  float readTemperature();
  if(millis()-timepoint>1000U){                  //time interval: 1s
        timepoint = millis();
        //temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
        voltage = analogRead(PH_PIN)/1024.0*5000;  // read the voltage
        phValue = ph.readPH(voltage,s);  // convert voltage to pH with temperature compensation
        //Serial.print("temperature:");
      //  Serial.print(temperature,1);
        Serial.print("pH: ");
        Serial.println(phValue,2);
    }

  //Serial.print("Temperaturet:\t" + String(Temperaturet) + "\t");
  //Serial.print("ADC RAW:\t" + String(ADC_Raw) + "\t");
  //Serial.print("ADC Voltage:\t" + String(ADC_Voltage) + "\t");
  //Serial.print("Oksigen : ");
  //Serial.println(Oks);
  //Serial.print("Suhu : ");
  //Serial.println(s);
 
  lcd.clear();
  lcd.setCursor(0, 0); // Pindah ke baris pertama, kolom pertama
  lcd.print("Oks: ");
  lcd.print(Oks);
  
  lcd.setCursor(0, 9); // Pindah ke baris kedua, kolom pertama
  lcd.print("S: ");
  lcd.print(s);
  
  lcd.setCursor(8, 1); // Pindah ke baris kedua, kolom kedelapan
  lcd.print("Ph: ");
  lcd.print(phValue);

  delay(1000);
} 


/*
// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>
// #include <OneWire.h>
// #include <DallasTemperature.h>
// #include <eFLL.h>

// Definisi pin
#define ONE_WIRE_BUS 2
#define RELAY_PIN 3

// Objek LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Objek untuk sensor suhu
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Objek fuzzy
// Fuzzy* fuzzy = new Fuzzy();

// Definisi variabel
// float oxygenLevel, pHLevel, temperature;
// FuzzyInput* temperatureInput = new FuzzyInput(1);
// FuzzyOutput* aeratorOutput = new FuzzyOutput(1);

void setup() {
  // Inisialisasi komunikasi serial
  // Serial.begin(9600);

  // Inisialisasi LCD
  // lcd.init();
  // lcd.backlight();

  // Inisialisasi sensor suhu
  sensors.begin();

  // Inisialisasi relay
  pinMode(RELAY_PIN, OUTPUT);

  // Setup fuzzy logic
  // FuzzySet* cold = new FuzzySet(0, 0, 15, 25);
  // FuzzySet* warm = new FuzzySet(20, 25, 30, 35);
  // FuzzySet* hot = new FuzzySet(30, 35, 50, 50);
  // temperatureInput->addFuzzySet(cold);
  // temperatureInput->addFuzzySet(warm);
  // temperatureInput->addFuzzySet(hot);
  // fuzzy->addFuzzyInput(temperatureInput);

  // FuzzySet* low = new FuzzySet(0, 0, 25, 50);
  // FuzzySet* medium = new FuzzySet(25, 50, 50, 75);
  // FuzzySet* high = new FuzzySet(50, 75, 100, 100);
  // aeratorOutput->addFuzzySet(low);
  // aeratorOutput->addFuzzySet(medium);
  // aeratorOutput->addFuzzySet(high);
  // fuzzy->addFuzzyOutput(aeratorOutput);

  // FuzzyRuleAntecedent* ifCold = new FuzzyRuleAntecedent();
  // ifCold->joinSingle(cold);
  // FuzzyRuleConsequent* thenHigh = new FuzzyRuleConsequent();
  // thenHigh->addOutput(high);
  // FuzzyRule* fuzzyRule1 = new FuzzyRule(1, ifCold, thenHigh);
  // fuzzy->addFuzzyRule(fuzzyRule1);

  // FuzzyRuleAntecedent* ifWarm = new FuzzyRuleAntecedent();
  // ifWarm->joinSingle(warm);
  // FuzzyRuleConsequent* thenMedium = new FuzzyRuleConsequent();
  // thenMedium->addOutput(medium);
  // FuzzyRule* fuzzyRule2 = new FuzzyRule(2, ifWarm, thenMedium);
  // fuzzy->addFuzzyRule(fuzzyRule2);

  // FuzzyRuleAntecedent* ifHot = new FuzzyRuleAntecedent();
  // ifHot->joinSingle(hot);
  // FuzzyRuleConsequent* thenLow = new FuzzyRuleConsequent();
  // thenLow->addOutput(low);
  // FuzzyRule* fuzzyRule3 = new FuzzyRule(3, ifHot, thenLow);
  // fuzzy->addFuzzyRule(fuzzyRule3);
}

void loop() {
  // Pembacaan sensor
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);

  // Dummy values for oxygen and pH level
  oxygenLevel = 7.0; // Replace with actual sensor reading
  pHLevel = 7.5; // Replace with actual sensor reading

  // Display on LCD
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("O2: ");
  lcd.print(oxygenLevel);
  lcd.print(" pH: ");
  lcd.print(pHLevel);

  // Fuzzy logic
  fuzzy->setInput(1, temperature);
  fuzzy->fuzzify();

  float aeratorControl = fuzzy->defuzzify(1);

  // Actuator control
  if (aeratorControl > 50) {
    digitalWrite(RELAY_PIN, HIGH); // Turn on aerator
  } else {
    digitalWrite(RELAY_PIN, LOW); // Turn off aerator
  }

  // Delay
  delay(2000);
}
*/