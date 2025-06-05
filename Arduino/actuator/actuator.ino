#include <LiquidCrystal_I2C.h>

constexpr char END_CHAR = ']';

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

  delay(100);

}


void print_line(int &i, char readMsg[]){
    while(readMsg[i] != END_CHAR)
    lcd.print(readMsg[i++]);

  i++;
}

void handleLCD(){
  char readMsg[COLS * ROWS + 4] = "";
  int i = 0;

  while(Serial.available() > 0){
    readMsg[i] = Serial.read();
    i++;
  }

  if(strlen(readMsg)){
    lcd.clear();

    i = 0;

    /////////////////////////////

    lcd.home();
    print_line(i, readMsg);

    //////////////////////////////

    lcd.setCursor(0, 1);

    lcd.print("cycle_");
    print_line(i, readMsg);
    lcd.print(".mat");

    ////////////////////////////

    lcd.setCursor(0, 2);

    lcd.print("Control: ");
    print_line(i, readMsg);

    ///////////////////////////

    lcd.setCursor(0, 3);
    print_line(i, readMsg);
    
    // for(int row = 0; row < ROWS; row++){
    //   lcd.home();
    //   lcd.setCursor(0, row);

    //   for(int col = 0; col < COLS; col++){
    //     if(readMsg[i] == END_CHAR){
    //       i++;
    //       break;
    //     }

    //     lcd.print(readMsg[i++]);
    //   }
    // }
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