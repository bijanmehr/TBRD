#include <Keypad.h>             //keypad
#include <LiquidCrystal_I2C.h>  //LCD
#include <Wire.h>               //VL53, RTC
#include <VL53L0X.h>            //tof sensor
#include <SPI.h>                //SD card
#include <SD.h>                 //SD card
//-------------------------------------------------------------------
//encoders, potentiometer

#define cnt_pot_pin A14
#define pwm_pot_pin A15

uint8_t cnt_val = 0;
uint8_t pwm = 0;
//-------------------------------------------------------------------
//switches
#define start_sw_pin 2            //start position switch (interrupt pin)
#define end_sw_pin 3              //end position switch (interrupt pin)
#define rec_sw_pin 19             //record switch (interrupt pin)
volatile int start_sw_val = 0;
volatile int end_sw_val = 0;
volatile int rec_sw_val = 0;
//-------------------------------------------------------------------
//keypad
const byte ROWS = 4;
const byte COLS = 3;
byte rowPins[ROWS] = {31, 33, 35, 37};
byte colPins[COLS] = {39, 41, 43};
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
String number = "";
String uid = "";
//-------------------------------------------------------------------
//driver motor
#define R_en_pin  6
#define L_en_pin  4
#define Rpwm_pin  7
#define Lpwm_pin  5
//-------------------------------------------------------------------
// SD module
#define SDpin 53
//-------------------------------------------------------------------
//I2C modules
//+-------+-------
//| parts |  I2C |
//+-------+------+
//|  RTC  | 0x68 |
//+-------+------+
//|  vl53 | 0x29 |
//+-------+------+
//|  lcd  | 0x27 |
//+-------+------+

//RTC
#define DS3231_ADDRESS 0x68
String rtc_time = "";
char date[8];
char rtime[8];

//vl53
String distance = "";

//lcd
#define LCD_ADDRESS 0x27
//-------------------------------------------------------------------
//logging
String directory = "";
String subdir = "";
String log_name = "";
String log_data = "";
String motor_spec = "";
int logger_stat = 0;
//-------------------------------------------------------------------
String pre_msg = "";
int pre_stage = 0;
//------------------------------------------------------------------
int keypad_flag = 1;
int pwm_flag = 1;
int cnt_flag = 1;
//------------------------------------------------------------------
int error_stat = 0;
int error_flag = 0;
int error_type = 0;
int pre_rec = 0;
int pre_pos = 1;
int header_flag = 1;
int num = 0;
int cnt = 0;
//------------------------------------------------------------------

VL53L0X tof;
LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
File dataFile; //logger
//------------------------------------------------------------------



int motor_func(int dir = 0, int voltage = 0) { //driver motor_func function: dir={0:break,1:cw,-1:ccw}, voltage: pwm

  if (voltage <= 255 && voltage >= 0) {
    if (dir == 0) {
      digitalWrite(R_en_pin, LOW);
      digitalWrite(L_en_pin, LOW);
      //      digitalWrite(Rpwm, LOW);
      //      digitalWrite(Lpwm, LOW);
      return 0;
    }
    else if (dir == -1) {
      digitalWrite(R_en_pin, HIGH);
      digitalWrite(L_en_pin, HIGH);
      analogWrite(Rpwm_pin, voltage);
      digitalWrite(Lpwm_pin, LOW);
      return -1;
    }
    else if (dir == 1) {
      digitalWrite(R_en_pin, HIGH);
      digitalWrite(L_en_pin, HIGH);
      digitalWrite(Rpwm_pin, LOW);
      analogWrite(Lpwm_pin, voltage);
      return 1;
    }
    else {
      return 2; //error: invalid dir value
    }
  }
  else {
    return 3; //error: invalid pwm value
  }
}
//------------------------------------------------------------------

void rec_sw() { //record switch function
  rec_sw_val = digitalRead(rec_sw_pin);
}

void start_sw() { //start position switch function
  start_sw_val = !digitalRead(start_sw_pin);
}


void end_sw() { //end position switch function
  end_sw_val = !digitalRead(end_sw_pin);
}
//------------------------------------------------------------------


int pos_check() {
  if (start_sw_val == 0 && end_sw_val == 0) {
    return 0; //slider in middle
  }
  else if (start_sw_val == 0 && end_sw_val == 1) {
    return 2; // slider in end position
  }
  else if (start_sw_val == 1 && end_sw_val == 0) {
    return 1; //slider in start position
  }
  else {
    return -1;  //switches malfunction
  }
}

//------------------------------------------------------------------
int ledState = LOW;
unsigned long previousMillis = 0;
const long interval = 300;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  //------------------------------------------------------------------
  Serial.begin(115200);
  Serial.setTimeout(50);
  //------------------------------------------------------------------
  //lcd
  lcd.init();
  lcd.backlight();
  //------------------------------------------------------------------
  //switches
  pinMode(start_sw_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(start_sw_pin), start_sw, RISING);
  pinMode(end_sw_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(end_sw_pin), end_sw, RISING);
  pinMode(rec_sw_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rec_sw_pin), rec_sw, CHANGE);

  start_sw_val = !digitalRead(start_sw_pin);
  end_sw_val = !digitalRead(end_sw_pin);
  //------------------------------------------------------------------
  //I2C modules
  Wire.begin();
  tof.init();
  tof.setTimeout(500);
  tof.startContinuous();
  //------------------------------------------------------------------
  //motor_func
  pinMode(R_en_pin, OUTPUT);
  pinMode(L_en_pin, OUTPUT);
  pinMode(Rpwm_pin, OUTPUT);
  pinMode(Lpwm_pin, OUTPUT);
  digitalWrite(R_en_pin, LOW);
  digitalWrite(L_en_pin, LOW);
  digitalWrite(Rpwm_pin, LOW);
  digitalWrite(Lpwm_pin, LOW);
  //------------------------------------------------------------------
}

void loop() {
  unsigned long currentMillis = millis();
  //  if (ledState == LOW) {
  //    ledState = HIGH;
  //  } else {
  //    ledState = LOW;
  //  }
  digitalWrite(LED_BUILTIN, !ledState);
  //------------------------------------------------------------------
  // debugging
  //      printer(String(pre_rec) + " " + String(rec_sw_val) + "  " + String(cnt) + " " + String(sd_stat));
  //    printer("start_sw: " + String(start_sw_val) + " " +
  //            "end_sw: " + String(end_sw_val) + "  " +
  //            "pos_check: " + String(pos_check()) + " " +
  //            "pre_pos: " + String(pre_pos) + " " +
  //            "pre_rec: " + String(pre_rec) + " " +
  //            "rec_sw_val: " + String(rec_sw_val));
  //printer(distance);
  //printer(String(pwm)+" "+String(cnt));
  //printer(String(date)+String(rtime));
  //  printer(directory+"|"+subdir+"|"+log_name+"|"+log_data+"|"+String(sd_stat));
  //  printer("shuntvoltage: " + String(shuntvoltage) + "  " +
  //          "current_mA: " + String(current_mA) + " " +
  //          "busvoltage: " + String(busvoltage) + " " +
  //          "power_mW: " + String(power_mW) + " " +
  //          "loadvoltage: " + String(loadvoltage));
  //printer(motor_spec);
  //  printer(String(sd_stat));
//    printer(log_data);
  //------------------------------------------------------------------
  //logging
    log_data = String(date) + String(rtime) + "," + distance + "," + String(cnt) + "," + String(pwm) + "," + uid;
  //------------------------------------------------------------------
  //RTC
  rtc_time = rtc();
  //------------------------------------------------------------------
  //tof
  distance = String(tof.readRangeContinuousMillimeters());
  //------------------------------------------------------------------
  if (pwm_flag == 1) {
    pwm = map(analogRead(pwm_pot_pin), 0, 1024, 10, 255);
    if (pwm > 150) {
      pwm = 150;
    }
  }
  cnt_val = map(analogRead(cnt_pot_pin), 0, 1023, 0, 100);
  if (cnt_flag == 1) {
    cnt = cnt_val + 1;
  }
  //------------------------------------------------------------------
  if (keypad_flag == 1) {
    char customKey = customKeypad.getKey();
    if (String(customKey) == "#") {
      if (number != "") {
        uid = number;
        number = "";
      }
    }
    else if (String(customKey) == "*") {
      lcd.clear();
      number = "";
      uid = "";
    }
    else {
      if (number.length() < 6) {
        number = number + String(customKey);
      }
      else {
        uid = number;
      }
    }
  }
  //------------------------------------------------------------------
  if (uid == "") {    //uid is empty
    keypad_flag = 1;  //enable keypad
    pwm_flag = 0;     //disable pwm
    cnt_flag = 0;     //disable cnt
    lcd_handler(0);   //request for UID
    pre_rec = 0;
    cnt = 0;
  }
  else {             //uid is NOT empty
    if (!SD.begin(SDpin)) { // SD card ERROR
      printer("Card failed, or not present");
      lcd_handler(5);   //SD error
      // don't do anything more
      while (1);
    }
    else { //SD card is okay

      //log data
      //open logger
      dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (!dataFile) { // error can not open datafile
        printer("Can not open the datafile");
        lcd_handler(10);   //logfile error
      }
      else { // logfile is okay
        dataFile.println(log_data);
        dataFile.close();
        if (error_flag == 0) {    // no error
          if (pre_rec == 0 && rec_sw_val == 0 && cnt == 0) {                //I:initial state
            keypad_flag = 1;  //enable keypad
            pwm_flag = 1;     //enable pwm
            cnt_flag = 1;     //enable cnt
            lcd_handler(1);   //main interface
            printer("I");
          }
          else if (pre_rec == 0 && rec_sw_val == 0 && cnt != 0) {           //II:ready to start process
            keypad_flag = 1;  //enable keypad
            pwm_flag = 1;     //enable pwm
            cnt_flag = 1;     //enable cnt
            lcd_handler(1);   //main interface
            printer("II");
          }
          else if (pre_rec == 0 && rec_sw_val == 1 && cnt == 0) {           //III:cnt error
            printer("CNT ERROR");
            keypad_flag = 0;  //disable keypad
            pwm_flag = 0;     //disable pwm
            cnt_flag = 0;     //disable cnt
            lcd_handler(8);   //cnt error
            printer("III");
          }
          else if (pre_rec == 0 && rec_sw_val == 1 && cnt != 0) {           //IV:start process
            keypad_flag = 0;  //disable keypad
            pwm_flag = 1;     //enable pwm
            cnt_flag = 0;     //disable cnt
            lcd_handler(2);   //recording interface
            printer("IV");

            if (pos_check() != 1) {
              error_flag = 1;
              error_type = 3;
            }

            pre_rec = 1;
          }
          else if (pre_rec == 1 && rec_sw_val == 0 && cnt == 0) {           //V:process finished, user turned off recorder
            keypad_flag = 1;  //enable keypad
            pwm_flag = 1;     //enable pwm
            cnt_flag = 1;     //enable cnt
            lcd_handler(1);   //main interface
            pre_rec = 0;
            motor_func(0, 0);
            printer("V");
          }
          else if (pre_rec == 1 && rec_sw_val == 0 && cnt != 0) {           //VI:user turned off recorder mid process
            keypad_flag = 1;  //enable keypad
            pwm_flag = 1;     //enable pwm
            cnt_flag = 1;     //enable cnt
            lcd_handler(1);   //main interface
            motor_func(0, 0);
            printer("VI");
          }
          else if (pre_rec == 1 && rec_sw_val == 1 && cnt == 0) {           //VII:process finished
            keypad_flag = 0;  //disable keypad
            pwm_flag = 0;     //disable pwm
            cnt_flag = 0;     //disable cnt
            lcd_handler(7);   //finish prompt
            motor_func(0, 0);
            printer("VII");
          }
          else if (pre_rec == 1 && rec_sw_val == 1 && cnt != 0) {           // VIII:mid process
            keypad_flag = 0;  //disable keypad
            pwm_flag = 1;     //enable pwm
            cnt_flag = 0;     //disable cnt
            lcd_handler(2);   //recording interface
            printer("VIII");


            //movment algorithm
            //pre_pos: start to end = 0
            //pre_pos: end to start = 1
            if (start_sw_val == 0 && end_sw_val == 0 && pre_pos == 0) {
              //midway from start to end
              motor_func(1, pwm);
            }
            else if (start_sw_val == 0 && end_sw_val == 0 && pre_pos == 1) {
              // midway from end to start
              motor_func(-1, pwm);
            }
            else if (start_sw_val == 0 && end_sw_val == 1 && pre_pos == 0) {
              //reach end from start
              pre_pos = 1;
              motor_func(0, 0);
            }
            else if (start_sw_val == 0 && end_sw_val == 1 && pre_pos == 1) {
              //ready to go to start
              motor_func(-1, pwm);
            }
            else if (start_sw_val == 1 && end_sw_val == 0 && pre_pos == 0) {
              //ready to go to end
              motor_func(1, pwm);
            }
            else if (start_sw_val == 1 && end_sw_val == 0 && pre_pos == 1) {
              //reach start from end
              pre_pos = 0;
              motor_func(0, 0);
              //one course completed!
              cnt--;
            }
            else if (start_sw_val == 1 && end_sw_val == 1 && pre_pos == 0) {
              //switch malfunction
              error_flag = 1;
              error_type = 4;
            }
            else if (start_sw_val == 1 && end_sw_val == 1 && pre_pos == 1) {
              //switch malfunction
              error_flag = 1;
              error_type = 4;
            }
            else {
              //impossible state
            }

          }
          else {    //impossible state return ERROR
            printer("STATE ERROR");
          }
        }
        else {                    // an error obsserved!
          printer("ERROR flag");
          error_stat = error_handler(error_type);
          if (error_stat == 0) {
            error_flag = 0;
          }
        }
      }

    }
  }


}

//------------------------------------------------------------------

void printer(String msg) {
  if (msg != pre_msg) {
    Serial.println(msg);
    pre_msg = msg;
  }
}
//------------------------------------------------------------------

void lcd_printer(int row, String data) {
  lcd.setCursor(0, row);
  lcd.print(data);
}

void lcd_handler(int stage) {
  if (stage != pre_stage) {
    lcd.clear();
  }
  switch (stage) {
    case 0:
      //request for UID
      lcd_printer(0, "TBRD2" );
      lcd_printer(1, "Enter UID & press # ");
      lcd_printer(2, "to clear, press *   ");
      lcd_printer(3, number);
      pre_stage = stage;
      break;
    case 1:
      // main interface
      lcd_printer(0, "TBRD2");
      lcd_printer(1, "UID: " + uid);
      char line22[20];
      sprintf(line22, "Rec: OFF    Cnt: %03d", cnt);
      lcd_printer(2, line22);
      char line23[20];
      sprintf(line23, "PWM: %03d            ", pwm);
      lcd_printer(3, line23);
      pre_stage = stage;
      break;
    case 2:
      // recording interface
      lcd_printer(0, "TBRD2");
      lcd_printer(1, "Recording...........");
      char line32[20];
      sprintf(line32, "Counter: %03d        ", cnt);
      lcd_printer(2, line32);
      char line33[20];
      sprintf(line33, "PWM: %03d            ", pwm);
      lcd_printer(3, line33);
      pre_stage = stage;
      break;
    case 3:
      // position error
      lcd_printer(0, "TBRD2         ");
      lcd_printer(1, "Position Error.");
      lcd_printer(2, "Turn off recorder &");
      lcd_printer(3, "move slider to start");
      pre_stage = stage;
      break;
    case 4:
      // movement error
      lcd_printer(0, "TBRD2         ");
      lcd_printer(1, "Movement Error.");
      lcd_printer(2, "Turn off recorder &");
      lcd_printer(3, "reboot the device");
      pre_stage = stage;
      break;
    case 5:
      // SD error
      lcd_printer(0, "SD failure!");
      lcd_printer(1, "reboot the device");
      lcd_printer(2, "                    ");
      lcd_printer(3, "                    ");
      pre_stage = stage;
      break;
    case 6:
      // logger error
      lcd_printer(0, "TBRD2         ");
      lcd_printer(1, "Logger Error.");
      lcd_printer(2, "Turn off recorder &");
      lcd_printer(3, "check SD card");
      pre_stage = stage;
      break;
    case 7:
      // finishing prompt
      lcd_printer(0, "TBRD2         ");
      lcd_printer(1, "process finished!");
      lcd_printer(2, "turn off recorder   ");
      lcd_printer(3, "                    ");
      pre_stage = stage;
      break;
    case 8:
      // cnt error
      lcd_printer(0, "TBRD2         ");
      lcd_printer(1, "counter Error.");
      lcd_printer(2, "turn off recorder   ");
      lcd_printer(3, "assignt value to cnt");
      pre_stage = stage;
      break;
    case 9:
      // switch malfunction error
      lcd_printer(0, "TBRD2         ");
      lcd_printer(1, "switch Error.");
      lcd_printer(2, "reboot the device");
      lcd_printer(3, "call support        ");
      pre_stage = stage;
      break;
    case 10:
      // open logfile error
      lcd_printer(0, "TBRD2         ");
      lcd_printer(1, "logfile Error.");
      lcd_printer(2, "reboot the device");
      lcd_printer(3, "call support        ");
      pre_stage = stage;
      break;
    default:
      //error
      lcd_printer(0, "ERROR               ");
      lcd_printer(1, "ERROR               ");
      lcd_printer(2, "ERROR               ");
      lcd_printer(3, "ERROR               ");
      break;
  }
}
//------------------------------------------------------------------

byte bcdToDec(byte val)  {
  // Convert binary coded decimal to normal decimal numbers
  return ( (val / 16 * 10) + (val % 16) );
}

String rtc() {

  // Reset the register pointer
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(DS3231_ADDRESS, 7);

  int second = bcdToDec(Wire.read());
  int minute = bcdToDec(Wire.read());
  int hour = bcdToDec(Wire.read() & 0b111111); //24 hour time
  int weekDay = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
  int monthDay = bcdToDec(Wire.read());
  int month = bcdToDec(Wire.read());
  int year = bcdToDec(Wire.read());

  sprintf(date, "%2d%02d%02d", year, month, monthDay);
  sprintf(rtime, "%02d%02d%02d", hour, minute, second);

  String rtc_data = String(year) + "." + String(month) + "." + String(monthDay) + "_" + String(hour) + ":" + String(minute) + ":" + String(second);
  return rtc_data;
}
//------------------------------------------------------------------
int error_handler(int mode) {
  switch (mode) {
    case 1:
      lcd_handler(5);
      return 1; // SD card problem
      break;
    case 2:
      lcd_handler(6);
      return 2; //can not open log file
      break;
    case 3:
      //position error
      lcd_handler(3);
      if (start_sw_val == 1 && end_sw_val == 0) {
        return 0;
      }
      else {
        return 3;
      }
      break;
    case 4:
      lcd_handler(9);
      return 4; //switch malfunction
      break;
    default:
      return 0;
      break;
  }
}
