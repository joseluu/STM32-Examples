#include <LCD.h>
#include <LiquidCrystal_I2C.h>  // F Malpartida's NewLiquidCrystal library

/*-----( Declare Constants )-----*/
#define I2C_ADDR    0x47  // Define I2C Address for the PCF8574A 
//---(Following are the PCF8574 pin assignments to LCD connections )----
// This are different than earlier/different I2C LCD displays
#if 0
#define BACKLIGHT_PIN  7
#define En_pin  4
#define Rw_pin  5
#define Rs_pin  6
#define D4_pin  0
#define D5_pin  1
#define D6_pin  2
#define D7_pin  3
#else
#define BACKLIGHT_PIN  3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
#endif

#define  LED_OFF  HIGH
#define  LED_ON  LOW

/*-----( Declare objects )-----*/  
LiquidCrystal_I2C  lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);

void setupLCD()   /*----( SETUP: RUNS ONCE )----*/
{
	lcd.begin(16, 2);  // initialize the lcd 
	// Switch on the backlight
	lcd.setBacklightPin(BACKLIGHT_PIN, NEGATIVE);
	lcd.setBacklight(LED_ON);
}// END Setup

void testLCD()  
{

// Reset the display  
	lcd.clear();
	delay(1000);
	lcd.home();
  
	// Print our characters on the LCD
	  //lcd.backlight();  //Backlight ON if under program control
	lcd.setCursor(0, 0); //Start at character 0 on line 0
	lcd.print("Hello, world!");
	delay(1000);
	lcd.setCursor(0, 1); //Start at character 0 on line 1
	lcd.print("16 by 2 Display");
	delay(8000);
} // END Loop
