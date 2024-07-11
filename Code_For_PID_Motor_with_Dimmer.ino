#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Key.h>
#include <Keypad.h>
#include <Keypad_I2C.h>

#define I2CADDR 0x20                // alamat i2c dari keypad
LiquidCrystal_I2C lcd(0x27, 20, 4); // alamat i2c dari lcd

#define DETECT 2           // pin zero cross
#define GATE A3            // pin TRIAC gate
#define TACHO 3            // pin tacho sinyal input
#define VIBRATOR 5         // Motor Vibrator
#define currentfeedback A0 // pin sensor arus
#define PULSE 2            // jumlah hitungan lebar pulsa trigger TRIAC. Satu hitungan 16 microsecond

double Input;             // input kontrol PID
double Output;            // output kontrol PID
double realRPM;           // pembacaan nilai real RPM sesuai tacho
double Kp =1.02 , Ki = 0.125 , Kd = 2.0; // parameter tuning PID
//tunning ziegler nichols
//double Kcr = 1.7;
//double Pcr = 16.0;
//double Kp = Kcr * 0.6;
//double Ki = (2.0 * Kp) / Pcr;
//double Kd = (0.125 * Kp * Pcr);
double previousError = 0, integral = 0;   // variabel deteksi waktu untuk derivative dan integral
unsigned long lastTime;   // variabel waktu PID
unsigned long timeSampling = 100; // time sampling dari kalkulasi PID
unsigned long x=0;
int a = 0;

double dimming = 540;             // variabel untuk mengontrol motor
const int mindimminglimit = 80;   // nilai minial delay motor, semakin kecil delay semakin cepat motor berputar
const int maxdimminglimit = 420;  // nilai maksimal delay motor, semakin kecil delay semakin cepat motor berputar

int Setpoint = 0;         // variabel set point perhitungan PID berdasarkan nilai yang diketik pada keypad
int maxSetpoint = 1500;   // maksimal nilai setpoint yang boleh dimasukkan pada keypad
int minSetpoint = 400;    // minimal nilai setpoint yang boleh dimasukkan pada keypad
int digit = 0;            // variabel untuk mendeteksi digit dari set point pada keypad
bool flag_run = false;          // penanda jalan program
bool flag_dimming = false;      // penanda motor
bool flag_vibration = false;    // penanda vibrator
String inputString = "";        // variabel untuk mengubah nilai integer terhadap nilai string pada keypad

unsigned long previousMillis_lcd = 0; // variabel untuk menyimpan nilai waktu sebelumnya pada lcd
const long interval_lcd = 1000;       // delay update masing-masing variabel pada lcd
unsigned long lastMillis_print = 0; // variabel untuk menyimpan nilai waktu sebelumnya pada sampling data

// variabel kalman filter
float Q = 0.1; // Process noise covariance
float R = 1.0; // Measurement noise covariance
float P = 1.0; // Estimation error covariance
float K = 0.0; // Kalman gain
float X = 0.0; // Estimated value

// variabel untuk keypad
const byte ROWS = 4;    // baris pada keypad
const byte COLS = 4;    // kolom pada keypad
char keys [ROWS] [COLS] = {
  {'D', '#', '0', '*'},
  {'C', '9', '8', '7'},
  {'B', '6', '5', '4'},
  {'A', '3', '2', '1'}
};
byte rowPins [ROWS] = {0, 1, 2, 3};
byte colPins [COLS] = {4, 5, 6, 7};
Keypad_I2C keypad (makeKeymap (keys), rowPins, colPins, ROWS, COLS, I2CADDR, PCF8574);

// variabel RPM
unsigned int RPM;                   // var RPM sebelum di kalman filter
unsigned int rawRPM;                // var RPM setelah di kalman filter
unsigned int count;                 // var untuk menghitung pembacaan RPM
unsigned int lastcount = 0;
unsigned long lastcounttime = 0;
unsigned long lastflash;
unsigned long lastpiddelay = 0;
unsigned long previousMillis = 0;
unsigned long lastDebounceTime = 0;
unsigned long lastMillis = 0;

double Current;                     // variabel pembacaan arus dari sensor

// perhitungan kalman filter untuk filter pada pembacaan RPM
float kalmanFilter(float measurement) {
  P = P + Q;
  K = P / (P + R);
  X = X + K * (measurement - X);
  P = (1 - K) * P;
  return X;
}

void setup() {
  Serial.begin(9600);
  Wire .begin ();
  keypad.begin (makeKeymap (keys));
  lcd.init();
  lcd.begin(20,4);
  lcd.backlight();

  pinMode(DETECT, INPUT);             // mengatur zero cross detect pin
  pinMode(TACHO, INPUT);              // mengatur tacho pulses detect pin
  pinMode(GATE, OUTPUT);              // mengatur TRIAC gate control pin
  pinMode(VIBRATOR, OUTPUT);          // mengatur VIBRATOR control pin
  digitalWrite(VIBRATOR, LOW);

  // set up Timer1
  OCR1A = 100;                        // inisialisasi the comparator
  TIMSK1 = 0x03;                      // mengaktifkan comparator A dan overflow interrupts
  TCCR1A = 0x00;                      // kontrol timer untuk mengatur registers
  TCCR1B = 0x00;                      // operasi normal, timer dimatikan

  // mengatur zero crossing interrupt IRQ0 pada pin 2
  attachInterrupt(digitalPinToInterrupt(DETECT), zeroCrossingInterrupt, RISING);

  // mengatur tacho sensor interrupt IRQ1 pada pin 3
  attachInterrupt(digitalPinToInterrupt(TACHO), tacho, FALLING);

  // Menampilkan Teaser pada LCD
  teaser();
  delay(3000);
  inputRPM();
}

// Interrupt Service
void zeroCrossingInterrupt() { // zero cross detect
  TCCR1B = 0x04;               // memulai timer, divide by 256 input
  TCNT1 = 0;                   // reset timer - menghitung dari nol
  if (dimming >= 420){
    OCR1A = 520;
  }
  else{
    OCR1A = int(dimming);           // mengatur komparasi register brightness desired
  }
}
ISR(TIMER1_COMPA_vect) {       // pencocokan comparator
  digitalWrite(GATE, HIGH);    // mengatur TRIAC gate ke high
  TCNT1 = 65536 - PULSE;       // trigger pulse width
}
ISR(TIMER1_OVF_vect) {         // timer1 overflow
  digitalWrite(GATE, LOW);     // mengatur TRIAC gate
  TCCR1B = 0x00;               // mematikan timer stops unintended triggers
}

// Tacho interrupt
void tacho() {
  count++;
}

void loop() {
  // sampling data
  unsigned long currentMillis_print = millis();
  const int interval_print = 100;
  if (currentMillis_print - lastMillis_print >= interval_print){
    lastMillis_print = currentMillis_print;
    Serial.println(int(realRPM));  
  }
  
  if(flag_run == true){

    // read Current
    const float sensitivity = 1.0; // sensitivitas sensor arus
    const float vRef = 5.0;         // tegangan referensi arduino
    const int adcMax = 1023;        // maksimal nilai ADC (10-bit)
    int sensorValue = analogRead(currentfeedback);
    float voltage = (sensorValue * vRef) / adcMax;
    Current = abs((voltage - (vRef / 2)) / sensitivity);

    // mematikan system
//    if(a==0){
//      x = millis();
//      a = 1; 
//    }
//    const float currentStop = 0.8;
//    unsigned long lastMillis_current = 0; // variabel untuk menyimpan nilai waktu sebelumnya pada sensor arus
//    unsigned long currentMillis_current = millis()-x;
//    const int interval_current = 20000;
//    Serial.println(currentMillis_current - lastMillis_current);
//    if (currentMillis_current - lastMillis_current > interval_current && Current <= currentStop){
//      lastMillis_current = 0;
//      currentMillis_current = 0;
//      a = 0;
//      finishSystem();
//      flag_run = false;
//      inputString = "";
//      Setpoint = 0;
//      digit = 0;
//      delay(2000);
//      inputRPM();
//    } 
    
    // read RPM
    unsigned long currentMillis = millis();
    const int interval = 100;
    if (currentMillis - lastMillis >= interval){
      lastMillis = currentMillis;

      RPM = (count * 6);          // nilai RPM yang tidak difilter
      rawRPM = (count * 6);       // nilai RPM untuk difilter
      float filteredRPM = kalmanFilter(rawRPM); // melakukan filter pada RPM
      Input = filteredRPM;        // menjadikan variabel RPM sebagai Input untuk kalkulasi pada PID
      // fungsi linearisasi
      const int max_Input = 600;
      const int min_Input = 0;
      const int max_Real = 1000;
      const int min_Real = 0;
      realRPM = (((Input - min_Input) / (max_Input - min_Input)) * (max_Real - min_Real)) + min_Real;   // rumus linierisasi (y-y1)/(y2-y2)=(x-x1)/(x2-x1)
      // realRPM = Input;
      // Input = RPM;
      count = 0;

      // calculation PID
      double error = Setpoint - realRPM;                // mendapatkan nilai error dari kalkulasi PID
      integral += (error);                              // mendapatkan nilai integral untuk dikalikan dengan Ki
      double derivative = (error - previousError);      // mendapatkan nilai derivative untuk dikalikan dengan Kd
      Output = (Kp * error) + (Ki * integral) + (Kd * derivative);  // kalkulasi PID mendapatkan nilai output untuk mengontrol dimming pada motor
      previousError = error;

      if(flag_dimming == false){
        dimming = map(Output, mindimminglimit, maxdimminglimit, maxdimminglimit, mindimminglimit);    // linearisasi nilai dimming dari output PID
        dimming = constrain(dimming, 200, 420);                                                       // filterasi variabel dimming supaya tetap dalam range nya

        if(dimming>420 || flag_vibration == true){
          digitalWrite(VIBRATOR, LOW);
        }
        else{
          digitalWrite(VIBRATOR, HIGH);
        }
      }
      else{
        dimming = 540;
      }
      if(flag_vibration == false){
        digitalWrite(VIBRATOR, HIGH);
      }
      else{
        digitalWrite(VIBRATOR, LOW); 
      }
    }
  }
  else{
    dimming = 540;
    digitalWrite(VIBRATOR, LOW);
  }

  // update LCD
  unsigned long currentMillis_lcd = millis();
  if (currentMillis_lcd - previousMillis_lcd >= interval_lcd) {
    previousMillis_lcd = currentMillis_lcd;
    if(flag_run == true){
      updateLCD();
    }
  }

  char key = keypad.getKey ();

  if(key){
    if (key >= '0' && key <= '9') {
      digit++;
      inputString += key;
      inputRPM();
      lcd.setCursor(13, 2);
      lcd.print(inputString);
      if(digit>5){
        maksDigit();
        delay(1500);
        inputRPM();
      }
    } 
    else if (key == '#') {
      Setpoint = inputString.toInt();
      inputString = "";
      digit = 0;
      if(Setpoint > maxSetpoint){
        maxSystem();
        delay(1000);
        runningSystem();
      }
      else if(Setpoint < minSetpoint && Setpoint != 0){
        minSystem();
        delay(1000);
        runningSystem();
      }
      else if(Setpoint == 0){
        errorSystem();
        delay(1000);
        inputRPM();
      }
      else{
        okSystem();
        delay(1000);
        runningSystem();
      }
    } 
    else if (key == '*') {
      flag_run = false;
      inputString = "";
      Setpoint = 0;
      digit = 0;
      inputRPM();
    }
    else if (key == 'A') {
      if(Setpoint>0){
        matikanMotor();
      }
      else{
        inputRPM();
      }
    }
    else if (key == 'B') {
      if(Setpoint>0){
        nyalakanMotor();
      }
      else{
        inputRPM();
      }
    }
    else if (key == 'C') {
      if(Setpoint>0){
        matikanVibrator();
      }
      else{
        inputRPM();
      }
    }
    else if (key == 'D') {
      if(Setpoint>0){
        nyalakanVibrator();
      }
      else{
        inputRPM();
      }
    }
  }
}

void teaser() {
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("SISTEM PENGGILING");
  lcd.setCursor(8, 1);
  lcd.print("JAHE");
  lcd.setCursor(5, 2);
  lcd.print("OLEH: MERY");
  lcd.setCursor(2, 3);
  lcd.print("NIM: 2041170135");
}

void inputRPM() {
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("Masukkan Nilai");
  lcd.setCursor(3, 1);
  lcd.print("Set Point RPM!");
  lcd.setCursor(2, 2);
  lcd.print("Set Point: ");
  lcd.setCursor(0, 3);
  lcd.print("* to clear |# to ok");
}

void maksDigit(){
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Maksimal 5 Digit!");
  digit = 0;
  Setpoint = 0;
  inputString = "";
}

void okSystem(){
  lcd.clear();
  lcd.setCursor(9, 0);
  lcd.print("OK!");
  lcd.setCursor(3, 1);
  lcd.print("System Running");
}

void errorSystem(){
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Masukkan Set Point");
  lcd.setCursor(2, 1);
  lcd.print("Terlebih Dahulu!");
}

void finishSystem(){
  lcd.clear();
  lcd.setCursor(7, 0);
  lcd.print("STOP!");
  lcd.setCursor(2, 1);
  lcd.print("System Finished");
}

void runningSystem(){
  flag_run = 1;
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Set point: ");
  lcd.setCursor(13, 0);
  lcd.print(Setpoint); 
  lcd.setCursor(5, 1);
  lcd.print("RPM: ");
  lcd.setCursor(10, 1);
  lcd.print(int(realRPM));
//  lcd.setCursor(3, 2);
//  lcd.print("Current: ");
//  lcd.setCursor(12, 2);
//  lcd.print(Current); 
  lcd.setCursor(0, 3);
  lcd.print("Press (*) to restart");
  delay(100);
}

void maxSystem(){
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("Maksimal");
  lcd.setCursor(2, 1);
  lcd.print("Set Point: ");
  lcd.setCursor(13, 1);
  lcd.print(maxSetpoint);
  Setpoint = maxSetpoint;
  delay(1000);
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("System Run to");
  lcd.setCursor(1, 1);
  lcd.print("Maksimal Set Point");
  delay(800);
  lcd.setCursor(7, 2);
  lcd.print(".      ");
  delay(500);
  lcd.setCursor(7, 2);
  lcd.print(". .    ");
  delay(500);
  lcd.setCursor(7, 2);
  lcd.print(". . .  ");
  delay(500);
  lcd.setCursor(7, 2);
  lcd.print(". . . .");
}

void minSystem(){
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("Minimum");
  lcd.setCursor(2, 1);
  lcd.print("Set Point: ");
  lcd.setCursor(13, 1);
  lcd.print(minSetpoint);
  Setpoint = minSetpoint;
  delay(1000);
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("System Run to");
  lcd.setCursor(1, 1);
  lcd.print("Minimum Set Point");
  delay(800);
  lcd.setCursor(7, 2);
  lcd.print(".      ");
  delay(500);
  lcd.setCursor(7, 2);
  lcd.print(". .    ");
  delay(500);
  lcd.setCursor(7, 2);
  lcd.print(". . .  ");
  delay(500);
  lcd.setCursor(7, 2);
  lcd.print(". . . .");
}

void matikanMotor(){
  lcd.clear();
  lcd.setCursor(5, 1);
  lcd.print("Motor OFF!");
  dimming = 540;
  flag_dimming = true;
  delay(1000);
  runningSystem();
}

void nyalakanMotor(){
  lcd.clear();
  lcd.setCursor(5, 1);
  lcd.print("Motor ON!");
  flag_dimming = false;
  delay(1000);
  runningSystem();
}

void matikanVibrator(){
  lcd.clear();
  lcd.setCursor(3, 1);
  lcd.print("Vibrator OFF!");
  digitalWrite(VIBRATOR, LOW);
  flag_vibration = true;
  delay(1000);
  runningSystem();
}

void nyalakanVibrator(){
  lcd.clear();
  lcd.setCursor(4, 1);
  lcd.print("Vibrator ON!");
  digitalWrite(VIBRATOR, HIGH);
  flag_vibration = false;
  delay(1000);
  runningSystem();
}

void updateLCD() {
  lcd.setCursor(10, 1);
  lcd.print("     ");
  lcd.setCursor(10, 1);
  lcd.print(int(realRPM));
//  lcd.setCursor(12, 2);
//  lcd.print("       ");
//  lcd.setCursor(12, 2);
//  lcd.print(Current);
}
