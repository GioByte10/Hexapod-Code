#include <LiquidCrystal_I2C.h>
#include <TM1637TinyDisplay.h>

#define END_CHAR          ']'
#define LCD_CHAR          '&'
#define TM_START_CHAR     '%'
#define TM_PAUSE_CHAR     '^'
#define TM_CONTINUE_CHAR  ';'

constexpr uint8_t COLS = 20;
constexpr uint8_t ROWS = 4;

constexpr uint8_t ENA_PIN = 8; 
constexpr uint8_t DIR_PIN = 9;
constexpr uint8_t PUL_PIN = 10;

constexpr uint8_t BUT_PIN = 7;

constexpr uint8_t CLK = 2;
constexpr uint8_t DIO = 3;

uint8_t dots = 0b01010000;
bool enabled = false;
int offRange = 90;
bool pausedTime = true;

unsigned long serialTimeStamp = 0;
unsigned long tmTimeStamp = 0;

int secs = 0;
int csecs = 0;

LiquidCrystal_I2C lcd(0x27, COLS, ROWS);
TM1637TinyDisplay tm(CLK, DIO);

void handleENA();
void handlePUL(int);
void printLine(char readMsg[]);
void readSerial();
void handleLCD(char readMsg[], int i = 1);
void displayTime();

void setup() {

  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.home();
  lcd.print("Pantograph Walker");
  lcd.setCursor(0, 1);
  lcd.print("Trying to walk here");

  tm.begin();
  tm.flipDisplay(true);

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

  displayTime();

  if(millis() - serialTimeStamp >= 150){
    readSerial();
    serialTimeStamp = millis();
  }

}

void readSerial(){
  char readMsg[COLS * ROWS + 4] = "";
  int i = 0;

  while(Serial.available() > 0){
    readMsg[i] = Serial.read();
    i++;
  }

  if(strlen(readMsg)){
    if(readMsg[0] == LCD_CHAR){
      handleLCD(readMsg);
    }

    else if (readMsg[0] == TM_START_CHAR || readMsg[0] == TM_PAUSE_CHAR || readMsg[0] == TM_CONTINUE_CHAR){
      tmTimeStamp = readMsg[0] == TM_START_CHAR ? millis() : tmTimeStamp;
      pausedTime = (readMsg[0] == TM_PAUSE_CHAR);

      if(strlen(readMsg) > 2){
        int i = 2;
        while(readMsg[i++] != LCD_CHAR);
        handleLCD(readMsg, i);
      }
    }
  }
}

void displayTime(){

  if(!pausedTime){
    secs = (millis() - tmTimeStamp) / 1000;
    csecs = (millis() - tmTimeStamp) / 10;
  }

  tm.showNumberDec(secs, dots, true, 2, 0);
  tm.showNumberDec(csecs, dots, true, 2, 2);
}

void handleLCD(char readMsg[], int i = 1){
  
  lcd.clear();

  /////////////////////////////

  lcd.home();
  printLine(i, readMsg);

  //////////////////////////////

  lcd.setCursor(0, 1);

  lcd.print("cycle_");
  printLine(i, readMsg);
  lcd.print(".mat");

  ////////////////////////////

  lcd.setCursor(0, 2);

  lcd.print("Control: ");
  printLine(i, readMsg);

  ///////////////////////////

  lcd.setCursor(0, 3);
  printLine(i, readMsg);

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

void printLine(int &i, char readMsg[]){
    while(readMsg[i] != END_CHAR)
    lcd.print(readMsg[i++]);

  i++;
}