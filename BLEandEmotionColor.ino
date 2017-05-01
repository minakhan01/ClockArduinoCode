// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

// Neopixel
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// Redbear
//"RBL_nRF8001.h/spi.h/boards.h" is needed in every new project
#include <SPI.h>
#include <EEPROM.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include <Servo.h> 

// Neopixel
// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            9

//Redbear
#define DIGITAL_OUT_PIN    2
#define DIGITAL_IN_PIN     A4
#define PWM_PIN            3
#define SERVO_PIN          5
#define ANALOG_IN_PIN      A5

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      60

Servo myservo;

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
uint32_t magenta = pixels.Color(150, 0, 150);
uint32_t green = pixels.Color(0,100,0);
uint32_t red = pixels.Color(100, 0, 0);
uint32_t blue = pixels.Color(0, 0, 100);
uint32_t yellow = pixels.Color(150,150,0);
int delayval = 100; // delay for 1/10th of a second

void setup() {

  // RedBear start

  // Default pins set to 9 and 8 for REQN and RDYN
  // Set your REQN and RDYN here before ble_begin() if you need
  //ble_set_pins(3, 2);
  
  // Set your BLE Shield name here, max. length 10
  //ble_set_name("My Name");
  
  // Init. and start BLE library.
  ble_begin();
  
  // Enable serial debug
  Serial.begin(57600);
  
  pinMode(DIGITAL_OUT_PIN, OUTPUT);
  pinMode(DIGITAL_IN_PIN, INPUT);
  
  // Default to internally pull high, change it if you need
  digitalWrite(DIGITAL_IN_PIN, HIGH);
  //digitalWrite(DIGITAL_IN_PIN, LOW);
  
  myservo.attach(SERVO_PIN);
  //Redbear finish

  //Neopixel start
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  pixels.begin(); // This initializes the NeoPixel library.
  //Neopixel finish
}

void loop() {

  // RedBear start
  static boolean analog_enabled = false;
  static byte old_state = LOW;
  
  // If data is ready
  while(ble_available())
  {
    // read out command and data
    byte id_data = ble_read();
    byte data = ble_read();
    byte buffer_data = ble_read();
    Serial.println("id_data");
    Serial.println(id_data);
    Serial.println("data");
    Serial.println(data);
    Serial.println("buffer_data");
    Serial.println(buffer_data);

    //color
    if (id_data == 0x00) {
      Serial.print("colors of emotions: ");
      // fear
      if (data ==0x00) {
        Serial.println("fear");
        triggerFear();
        }
      // anger
      else if (data == 0x01) {
        Serial.println("anger");
        triggerAnger();
        }
      // disgust 
      else if (data == 0x02){
        Serial.println("disgust");
        triggerDisgust();
      }
      // happy 
      else if (data == 0x03){
        Serial.println("happy");
        triggerHappy();
      }
      // sad 
      else if (data == 0x04){
        Serial.println("sad");
        triggerSad();
      }
    }
    // speed
    else if (id_data == 0x01) {
      Serial.print("speed: ");
      Serial.println(data);
    }
  }
  if (analog_enabled)  // if analog reading enabled
  {
    // Read and send out
    uint16_t value = analogRead(ANALOG_IN_PIN); 
    ble_write(0x0B);
    ble_write(value >> 8);
    ble_write(value);
  }
  
  // If digital in changes, report the state
  if (digitalRead(DIGITAL_IN_PIN) != old_state)
  {
    old_state = digitalRead(DIGITAL_IN_PIN);
    
    if (digitalRead(DIGITAL_IN_PIN) == HIGH)
    {
      ble_write(0x0A);
      ble_write(0x01);
      ble_write(0x00);    
    }
    else
    {
      ble_write(0x0A);
      ble_write(0x00);
      ble_write(0x00);
    }
  }
  
  if (!ble_connected())
  {
    analog_enabled = false;
    digitalWrite(DIGITAL_OUT_PIN, LOW);
  }
  
  // Allow BLE Shield to send/receive data
  ble_do_events(); 
  // Redbear end
}

void testColors() {
  turnGreen();
  turnRed();
  turnMagenta();
  turnYellow();
  turnBlue();
}

void triggerAnger() {
  turnRed();
}

void triggerSad() {
  turnBlue();
}

void triggerDisgust() {
  turnGreen();
}

void triggerHappy() {
  turnYellow();
}

void triggerFear() {
  turnMagenta();
}

void turnGreen() {
  changeColor(green);
}

void turnRed() {
  changeColor(red);
}

void turnMagenta() {
  changeColor(magenta);
}

void turnYellow() {
  changeColor(yellow);
}

void turnBlue() {
  changeColor(blue);
}

void changeColor(uint32_t color) {

  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.

  for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, color); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).

  }
}
