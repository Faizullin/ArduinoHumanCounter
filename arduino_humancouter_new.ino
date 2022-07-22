#include <LiquidCrystal_I2C.h>
#include <iarduino_DHT.h>
#include <Servo.h>
#include <Wire.h>

//#define PRINT 1
//
#define WIRE_SLAVE_ADDRESS 8
#define WIRE_BUTTON_INPUT 11
#define DHT_INPUT 8
#define GAS_INPUT A0
#define DIOD_OUTPUT 12//9
#define WARN_DIOD_OUTPUT 10
#define TONED_OUTPUT 9 //11
#define SERVO_OUTPUT A2//12
#define TRIG1 4
#define TRIG2 2
#define ECHO1 5
#define ECHO2 3
#define COUNT_DISTANCE 6
#define DIST_DELAY_LIMIT 1000
#define MAX_COUNT 10
#define TONED_FREQUENCY 200
#define BUTTON_INPUT 6
#define DIOD_COMMAND 2
#define LOCALIP_COMMAND 3


Servo myservo;
LiquidCrystal_I2C lcd1(0x3F, 16, 2);// display 1
LiquidCrystal_I2C lcd2(0x27, 16, 2);
iarduino_DHT sensor(DHT_INPUT);
bool closed = false;


void sensors_update();
bool counting_update();
uint32_t timer1, timer2, timer;
int transferArray[4] = {0, 0, 0, 0};
bool SingleArd = true;
String localip;
class PressingButton {
    bool lastState = 0, State = 0;
    uint32_t timer;
    uint16_t _delay;
    int _pin;
  public:
    PressingButton(int _pin, uint16_t _delay = 500) {
      this->_pin = _pin;
      this->_delay = _delay;
    };
    bool update();
};

class TimerDiod {
    bool State = 0;
    uint8_t lastTimer = 0;
    int _pin; uint16_t _delay; uint32_t timer;
    uint8_t counter = 0;
  public:
    TimerDiod(int _pin, uint16_t _delay) {
      this->_pin = _pin;
      this->_delay = _delay;
    };
    bool update();
    void on();
    bool getState() {
      return this->State;
    };
    void off();
    bool isSec() {
      uint8_t t = millis() / 1000;
      if (!(t == this->lastTimer)) {
        this->lastTimer = t;
        return 1;
      }
      return 0;
    }
    int getTimer() {
      this->counter++;
      return this->_delay / 1000 - this->counter + 1;
    }
};

bool PressingButton::update() {
  this->State = !digitalRead(this->_pin);
#ifdef PRINT
  if (this->State) {
    Serial.print("click: ");
    Serial.print(this->State); Serial.println(this->lastState);
  }
#endif
  if (this->State != this->lastState) { // button state changed
    if (this->State) {

      this->timer = millis();
    }
    else if (millis() - this->timer > this->_delay) {
      this->lastState = this->State;
      return 1;
    }
  }
  this->lastState = this->State;
  return 0;
}
void TimerDiod::on() {
  digitalWrite(this->_pin, HIGH);
  this->timer = millis();
  this->lastTimer = this->timer / 1000;
  this->State = true;
  this->counter = 0;
}
void TimerDiod::off() {
  digitalWrite(this->_pin, LOW);
  this->State = false;
  this->counter = this->_delay / 1000;
}
bool TimerDiod::update() {
  if (this->State) {
    if (millis() - this->timer > this->_delay) {
      this->off();
    }
    return 1;
  }
  return 0;
}
PressingButton myButton1(BUTTON_INPUT);
PressingButton myButton2(WIRE_BUTTON_INPUT);
TimerDiod  myDiod(DIOD_OUTPUT, 10000);

bool connection_wait(unsigned long l) {
  bool res = false;
  uint16_t timer = millis();
  while (millis() - timer < l) {
    if (!res) {
      if (!digitalRead(WIRE_BUTTON_INPUT)) {
        res = true;
        digitalWrite(13, HIGH);
      }
    };
    delay(50);
  }
  return res;
}
void setup()
{

  pinMode(GAS_INPUT, INPUT); pinMode(BUTTON_INPUT, INPUT_PULLUP); pinMode(WIRE_BUTTON_INPUT, INPUT_PULLUP);
  pinMode(TONED_OUTPUT, OUTPUT); pinMode(13, OUTPUT); digitalWrite(13, LOW);
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT); pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);
  pinMode(DIOD_OUTPUT, OUTPUT);
  pinMode(WARN_DIOD_OUTPUT, OUTPUT);
  Wire.begin(WIRE_SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  myservo.attach(SERVO_OUTPUT); myservo.write(90);
  lcd1.init(); lcd2.init();
  lcd1.backlight(); lcd2.backlight();
  lcd1.print("START 1"); lcd2.print("START 2");
  tone(TONED_OUTPUT, TONED_FREQUENCY + 60);
  #ifdef PRINT
      Serial.begin(9600);
      Serial.print("start");
      SingleArd = false;
  #else
    if (connection_wait(3000)) {
      SingleArd = false;
      Serial.begin(9600);
      while (!Serial);
    }
  #endif
  digitalWrite(13, LOW);
  noTone(TONED_OUTPUT);
  lcd1.clear(); lcd2.clear();
  lcd1.print("People:" + (String)transferArray[3]) ;
  lcd2.print("T:" + (String)transferArray[0] + "*C  H:" + (String)transferArray[1] + "%");
  lcd2.setCursor(0, 1);
  lcd2.print("GAS:" + (String)transferArray[2]);
}

void loop() {
  if (SingleArd) {
    if (counting_update()) {
      lcd1.clear();
      lcd1.print("People:" + (String)transferArray[3]);
      if (transferArray[3] == MAX_COUNT) {
        tone(TONED_OUTPUT, TONED_FREQUENCY + 60, 1000);
        myservo.write(179);
        lcd1.setCursor(0, 1);
        lcd1.print("MAXIMUM COUNT");
        digitalWrite(WARN_DIOD_OUTPUT, HIGH);
      }
      else if (transferArray[3] == 0) {
        lcd1.setCursor(0, 1);
        lcd1.print("EMPTY");
        tone(TONED_OUTPUT, TONED_FREQUENCY, 300);
      } else {
        myservo.write(90);
        tone(TONED_OUTPUT, TONED_FREQUENCY, 300);
        digitalWrite(WARN_DIOD_OUTPUT, LOW);
      }
    }
  } else {
    if (Serial.available()) {
      String y = Serial.readString();
      int x = 0;
      #ifdef PRINT 
        Serial.print(y);
      #endif
      
      if (y[0] == 'h') {
        x = y.substring(1).toInt();
        transferArray[3] = x;
        lcd1.clear();
        lcd1.print("People:" + (String)transferArray[3]);
        if (transferArray[3] >= MAX_COUNT) {
          closed = true;
          tone(TONED_OUTPUT, TONED_FREQUENCY + 60, 1000);
          myservo.write(179);
          lcd1.setCursor(0, 1);
          lcd1.print("MAXIMUM COUNT");
          digitalWrite(WARN_DIOD_OUTPUT, HIGH);
        }
        else if (transferArray[3] == 0) {
          lcd1.setCursor(0, 1);
          lcd1.print("EMPTY");
          tone(TONED_OUTPUT, TONED_FREQUENCY, 300);
          closed = false;
        } else {
          myservo.write(90);
          tone(TONED_OUTPUT, TONED_FREQUENCY, 300);
          digitalWrite(WARN_DIOD_OUTPUT, LOW);
          closed = false;
        }
      } else {
        x = y.toInt();
        switch (x) {
          case 1:
            digitalWrite(WARN_DIOD_OUTPUT, HIGH); break;
          case 2:
            digitalWrite(WARN_DIOD_OUTPUT, LOW); break;
          case 3:
            digitalWrite(WARN_DIOD_OUTPUT, HIGH);
            delay(2000);
            digitalWrite(WARN_DIOD_OUTPUT, LOW); break;
          case 4:
            tone(TONED_OUTPUT, TONED_FREQUENCY + 60, 1000);
            myservo.write(179);
            lcd1.setCursor(0, 1);
            lcd1.print("MAXIMUM COUNT");
            digitalWrite(WARN_DIOD_OUTPUT, HIGH); break;
          case 5:
            myservo.write(90);
            tone(TONED_OUTPUT, TONED_FREQUENCY, 300);
            digitalWrite(WARN_DIOD_OUTPUT, LOW); break;
          default:
            break;
        }
      }
    }
    
    if (!closed) {
      if (counting_update()) {
        Serial.print(6);
        tone(TONED_OUTPUT, TONED_FREQUENCY, 300);
      }
    }

  }

  if (millis() - timer > 1000) {
    timer = millis();
    sensors_update();
    lcd2.clear();
    lcd2.print("T:" + (String)transferArray[0] + "*C  H:" + (String)transferArray[1] + "%");
    lcd2.setCursor(0, 1);
    lcd2.print("GAS:" + (String)transferArray[2]);
  }
  if (myButton2.update()) {
    lcd1.clear();
    lcd1.setCursor(0,0);
    lcd1.print("IP:");
    lcd1.print(localip);
    delay(1000);
  }
  if (myButton1.update()) {
#ifdef PRINT
    Serial.print("Update " + (String)myDiod.getState());
#endif
    if (myDiod.getState()) {
#ifdef PRINT
      Serial.println("OFF");
#endif
      myDiod.off();
    } else {
#ifdef PRINT
      Serial.println("ON");
#endif
      myDiod.on();
    }
  }
  if (myDiod.update()) {
    if (myDiod.isSec()) {
      lcd2.setCursor(14, 1);
      lcd2.print(myDiod.getTimer());
    }
  }
  delay(10);
}
void sensors_update() {
  if (sensor.read() == DHT_OK) {
    transferArray[0] = sensor.tem; transferArray[1] = sensor.hum;
  }
  transferArray[2] = analogRead(GAS_INPUT);
}
int dist(unsigned int trig, unsigned int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return (pulseIn(echo, HIGH) / 2) / 29.1;
}
bool counting_update() {
  if (millis() - timer1 > DIST_DELAY_LIMIT) {
    if (dist(TRIG1, ECHO1) < COUNT_DISTANCE && transferArray[3] < MAX_COUNT) {
      timer1 = millis();
      transferArray[3]++;
      return 1;
    }
  }
  if (millis() - timer2 > DIST_DELAY_LIMIT) {
    if (dist(TRIG2, ECHO2) < COUNT_DISTANCE && transferArray[3] > 0) {
      timer2 = millis();
      transferArray[3]--;
      return 1;
    }
  }
  return 0;
}
void receiveEvent(int howMany) {
  int command = Wire.read();
  #ifdef PRINT
   Serial.print(command);
  #endif
  if (command == DIOD_COMMAND) {
    int val = Wire.read();
    if (val) {
      myDiod.on();
    }
    else {
      myDiod.off();
    }
  } else if (command == LOCALIP_COMMAND) {
    int val;
    localip="";
    for(int i=0;i<3;i++){
      val = Wire.read();
      localip=localip+(String)val+".";
    }
    localip += (String)Wire.read();
    #ifdef PRINT
        Serial.println(localip);
    #endif
  }

}
void requestEvent() {
  for (int i = 0; i < (sizeof(transferArray)) / 2; i ++ ) {
    Wire.write(highByte(transferArray[i]));
    Wire.write(lowByte(transferArray[i]));
  }
}
