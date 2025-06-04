#include <LiquidCrystal_I2C.h>

constexpr uint8_t COLS = 20;
constexpr uint8_t ROWS = 4;

constexpr uint8_t ENA_PIN = 8; 
constexpr uint8_t DIR_PIN = 9;
constexpr uint8_t PUL_PIN = 10;

constexpr uint8_t BUT_PIN = 7;

bool enabled = false;
int offRange = 90;

LiquidCrystal_I2C lcd(0x27, COLS, ROWS);

void handleLCD();
void handleENA();
void handlePUL(int);


void setup() {

  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.home();
  lcd.print("Pantograph Walker");
  lcd.setCursor(0, 1);
  lcd.print("Trying to walk here");

  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(BUT_PIN, INPUT_PULLUP);

  digitalWrite(ENA_PIN, enabled);
  digitalWrite(DIR_PIN, LOW);
}

void loop() {
  int v = analogRead(A0);
  v -= 512;

  digitalWrite(DIR_PIN, v > 0 ? LOW : HIGH);

  handleENA();

  handlePUL(v);

  handleLCD();

}

void handleLCD(){
  char readMsg[COLS * ROWS] = "";
  int i = 0;

  while(Serial.available()){
    readMsg[i] = Serial.read();
  }

  if(strlen(readMsg)){
    lcd.clear();
    lcd.home();
    lcd.print(readMsg);
  }
}

void handleENA(){
    if(!digitalRead(BUT_PIN)){
    enabled = !enabled;
    digitalWrite(ENA_PIN, enabled);
    delay(300);
  }
}

void handlePUL(int v){
  v = abs(v);

  if(v > offRange){ 
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(5000 / (v - offRange));
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(5000 / (v - offRange));
  }
}