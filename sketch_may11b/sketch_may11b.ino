#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
int x =1;
#define volt 35
void setup() {
    lcd.init();
  lcd.backlight();
Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(volt, OUTPUT);
}

void loop() {
  digitalWrite(volt, HIGH);
  lcd.clear();
  lcd.print("hello");
   
  
  Serial.println("hello");
  // put your main code here, to run repeatedly:

}
