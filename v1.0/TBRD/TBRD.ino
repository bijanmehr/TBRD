//volt-amp meter
#include <Adafruit_INA219.h>
#include <Wire.h>

//SD card
#include <SPI.h>
#include <SD.h>

// RTC module
#include <DS3232RTC.h>

// LCD
#include <LiquidCrystal_I2C.h>

//timer
#include <timer.h>

//pin numbers
const int chipSelect = 53; //SD module
//driver motor pins
const int R_en = 9;
const int L_en = 8;
const int Rpwm = 11;
const int Lpwm = 10;
//switches
const int start_sw_pin = 2; //start position switch (interrupt pin)
const int end_sw_pin = 3; //end position switch (interrupt pin)
const int emr_sw_pin = 19; //emergency stop switch (interrupt pin)
const int rec_sw_pin = 18; //record switch (interrupt pin)


//variables
//volt-amp meter
  float shuntvoltage_mV = 0;
  float busvoltage_V = 0;
  float current_mA = 0;
  float loadvoltage_V = 0;
  float power_mW = 0;
//motor
int pwm = 0;
//tunable variables
float max_current_mA = 3000.0;
int start_pwm = 100;
int cruse_pwm = 80;
//others
int overload_rate = 0;
int optimal_overload_rate = 10;
int max_overload_rate = 50;
String pre_msg = "";    // for non-iterative printing
int pre_row = 10;    // for non-iterative lcd clearing
//logger
String data = "";

Adafruit_INA219 ina219(0x40); //volt-amp meter
File dataFile; //logger
LiquidCrystal_I2C lcd(0x27,16,2);


//timer
auto timer = timer_create_default(); // create a timer with default settings
bool refresh_lcd(void *) {
  lcd.clear();
  return true; // repeat? true
}

int motor_func(int dir = 0, int voltage = 0) { //driver motor_func function: dir={0:break,1:cw,-1:ccw}, voltage: pwm

  if (voltage <= 255 && voltage >= 0) {
    if (dir == 0) {
      digitalWrite(R_en, LOW);
      digitalWrite(L_en, LOW);
      //      digitalWrite(Rpwm, LOW);
      //      digitalWrite(Lpwm, LOW);
      return 0;
    }
    else if (dir == 1) {
      digitalWrite(R_en, HIGH);
      digitalWrite(L_en, HIGH);
      analogWrite(Rpwm, voltage);
      digitalWrite(Lpwm, LOW);
      return 1;
    }
    else if (dir == -1) {
      digitalWrite(R_en, HIGH);
      digitalWrite(L_en, HIGH);
      digitalWrite(Rpwm, LOW);
      analogWrite(Lpwm, voltage);
      return -1;
    }
    else {
      return 2; //error: invalid dir value
    }
  }
  else {
    return 3; //error: invalid pwm value
  }
}


void setup() {
  Serial.begin(115200);

  timer.every(300, refresh_lcd);

  //lcd
  lcd.init();
  lcd.backlight();
  lcdprinter(0,0,"TBRD");
  
  //motor_func
  pinMode(R_en, OUTPUT);
  pinMode(L_en, OUTPUT);
  pinMode(Rpwm, OUTPUT);
  pinMode(Lpwm, OUTPUT);
  digitalWrite(R_en, LOW);
  digitalWrite(L_en, LOW);
  digitalWrite(Rpwm, LOW);
  digitalWrite(Lpwm, LOW);


  pinMode(start_sw_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(start_sw_pin), start_sw, CHANGE);
  pinMode(end_sw_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(end_sw_pin), end_sw, CHANGE);
  pinMode(emr_sw_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(emr_sw_pin), emr_sw, CHANGE);
  pinMode(rec_sw_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rec_sw_pin), rec_sw, CHANGE);


  // the function to get the time from the RTC (PC)
  setSyncProvider(RTC.get);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }

  //initiate volt-amp meter
  ina219.begin();

}

void loop() {
  timer.tick(); // tick the timer
  shuntvoltage_mV = ina219.getShuntVoltage_mV();
  current_mA = ina219.getCurrent_mA();
  busvoltage_V = ina219.getBusVoltage_V();
  power_mW = ina219.getPower_mW();
  loadvoltage_V = busvoltage_V + (shuntvoltage_mV / 1000);

  String t = String(year()) + "/" + String(month()) + "/" + String(day()) + "," + String(hour()) + ":" + String(minute()) + ":" + String(second());
  String motor_spec = String(shuntvoltage_mV)+","+String(current_mA)+","+String(busvoltage_V)+","+String(power_mW)+","+String(loadvoltage_V);
  data =t+","+motor_spec;


  if (emr_sw() == 0) {  //emergency = off

    if (!SD.begin(chipSelect)) {
      printer("Card failed, or not present");
      // don't do anything more:
      lcdprinter(0,0,"SD: failed");
      while (1);
      
    }

    if (rec_sw() == 1) {  // recorder = on
      printer("recorder: ON!");
      lcdprinter(0,0,"REC: ON!");
      lcdprinter(1,0,String(shuntvoltage_mV/1000)+"v");
      lcdprinter(1,9,String(current_mA/1000)+"A");
      dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {   //log file is ok
        printer("log file: OK!");
        if (start_sw() == 1 && end_sw() == 0) {
          printer("stage 1");
          pwm = start_pwm;
          overload_rate = 0;
          dataFile.println("------------------------------------------------------");
          dataFile.println("YEAR/MONTH/DAY,HOUR:MINUTE:SECOND,shuntvoltage_mV,current_mA,busvoltage_V,power_mW,loadvoltage_V");
          dataFile.close();
          motor_func(-1, pwm);
        }
        else if (start_sw() == 0 && end_sw() == 1) {
          printer("stage 3");
          motor_func(0, 0);
          printer("test finished!");
        }
        else if (start_sw() == 0 && end_sw() == 0) {
          printer("stage 2");
          if (overload_rate < max_overload_rate) { //motor current is nominal
            motor_func(-1, pwm);
            dataFile.println(data);
            dataFile.close();
            if (current_mA > max_current_mA) {  //motor current pass the limit
              overload_rate += 1;
            }
            if (overload_rate > optimal_overload_rate) {  //increase motor pwm for nominal overload rate values
              if (pwm < 255 - optimal_overload_rate) {
                pwm = pwm + optimal_overload_rate;
              }
              else {
                pwm = 255;
              }
            }
          }
          else {  //motor overloaded
            //motor overload
            motor_func(0, 0);
            dataFile.println("motor overloaded!");
            dataFile.close();
            printer("ERROR: motor overloaded!");
            lcdprinter(0,0,"MOTOR OVERLOAD!");
          }
        }
        else {
          printer("ERROR: courses' switches malfunction!");
          lcdprinter(0,0,"switch problem");
          dataFile.close();
        }

      }
      else {  //can't ope log file
        printer("ERROR: problem with logfile");
        lcdprinter(0,0,"logfile error");
      }
    }
    else {    //recorder = off
      printer("recorder: OFF!");
      lcdprinter(0,0,"REC: OFF!");
      if (start_sw() == 0 && end_sw() == 1) {
        motor_func(1, cruse_pwm);
      }
      else if(start_sw() == 1 && end_sw() == 0){
        motor_func(0, 0);
      }
      else if(start_sw() == 0 && end_sw() == 0){
        motor_func(1, cruse_pwm);
      }
      else{
        printer("ERROR: courses' switches malfunction!");
        lcdprinter(0,0,"switch problem");
      }
      dataFile.close();
    }
  }
  else {  //emergency = on
    printer("ERROR: emergency switch is activated!");
    lcdprinter(0,0,"EMERGENCY!");
    motor_func(0, 0);
    dataFile.println("emergency switch is activated!");
    dataFile.close();
  }



}






int start_sw() { //start position switch function
  int start_sw_val = digitalRead(start_sw_pin);
  if (start_sw_val == 1) {
    return 1;
  }
  else {
    return 0;
  }
}


int end_sw() { //end position switch function
  int end_sw_val = digitalRead(end_sw_pin);
  if (end_sw_val == 1) {
    return 1;
  }
  else {
    return 0;
  }
}


int emr_sw() { //emergency stop switch function
  int emr_sw_val = digitalRead(emr_sw_pin);
  if (emr_sw_val == 1) {
    return 1;
  }
  else {
    return 0;
  }
}


int rec_sw() { //record switch function
  int rec_sw_val = digitalRead(rec_sw_pin);
  if (rec_sw_val == 1) {
    return 1;
  }
  else {
    return 0;
  }
}




void printer(String msg) {
  if (msg != pre_msg) {
    Serial.println(msg);
  }
  pre_msg = msg;
}




void lcdprinter(int row, int col, String lcd_msg) {

  lcd.setCursor(col,row);
  lcd.print(lcd_msg);
}
