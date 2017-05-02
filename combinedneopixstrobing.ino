// Stroboscopic Tachometer
// Ed Nisley - KE4ANU - December 2012
 
//----------
// Pin assignments
 
const byte PIN_KNOB_A = 2;          // knob A switch - must be on ext interrupt 2
const byte PIN_KNOB_B = 4;          //  .. B switch
const byte PIN_BUTTONS = A5;        //  .. push-close momentary switches
 
const byte PIN_STROBE = 9;          // LED drive, must be PWM9 = OCR1A using Timer1
 
const byte PIN_PWM10 = 10;          // drivers for LED strip, must turn these off...
const byte PIN_PWM11 = 11;
 
const byte PIN_SYNC = 13;           // scope sync
 
//----------
// Constants
 
const int UPDATEMS = 10;                // update LEDs only this many ms apart
 
#define TCCRxB_CS 0x03                  // Timer prescaler CS=3 -> 1:64 division
 
const float TICKPD = 64.0 * 62.5e-9;    // basic Timer1 tick rate: prescaler * clock
 
enum KNOB_STATES {KNOB_CLICK_0,KNOB_CLICK_1};
 
// ButtonThreshold must have N_BUTTONS elements, last = 1024
 
enum BUTTONS {SW_KNOB, B_1, B_2, B_3, B_4, N_BUTTONS};
const word ButtonThreshold[] = {265/2, (475+265)/2, (658+475)/2, (834+658)/2, (1023+834)/2, 1024};
 
//----------
// Globals
 
float FlashLength = 0.1e-3;                     // strobe flash duration in seconds
word FlashLengthCt = FlashLength / TICKPD;      //  ... in Timer1 ticks
 
float FlashFreq = 50.0;                         // strobe flash frequency in Hz
float FlashPd = 1.0 / FlashFreq;                //  ... period in sec
word FlashPdCt = FlashPd / TICKPD;              //  ... period in Timer1 ticks
 
float FreqIncr = 1.0;                           // default frequency increment
const float FreqMin = 4.0;
const float FreqMax = 1.0/(4.0*FlashLength);
 
volatile char KnobCounter = 0;
volatile char KnobState;
 
byte Button, PrevButton;
 
unsigned long MillisNow;
unsigned long MillisThen;
 
//-- Helper routine for printf()
 
int s_putc(char c, FILE *t) {
  Serial.write(c);
}
 
//-- Knob interrupt handler
 
void KnobHandler(void)
{
    byte Inputs;
    Inputs = digitalRead(PIN_KNOB_B) << 1 | digitalRead(PIN_KNOB_A);  // align raw inputs
//  Inputs ^= 0x02;                             // fix direction
 
    switch (KnobState << 2 | Inputs) {
    case 0x00 :                 // 0 00 - glitch
        break;
    case 0x01 :                  // 0 01 - UP to 1
        KnobCounter++;
        KnobState = KNOB_CLICK_1;
        break;
    case 0x03 :                  // 0 11 - DOWN to 1
        KnobCounter--;
        KnobState = KNOB_CLICK_1;
        break;
    case 0x02 :                  // 0 10 - glitch
        break;
    case 0x04 :                  // 1 00 - DOWN to 0
        KnobCounter--;
        KnobState = KNOB_CLICK_0;
        break;
    case 0x05 :                  // 1 01 - glitch
        break;
    case 0x07 :                  // 1 11 - glitch
        break;
    case 0x06 :                  // 1 10 - UP to 0
        KnobCounter++;
        KnobState = KNOB_CLICK_0;
        break;
    default :                   // something is broken!
        KnobCounter = 0;
        KnobState = KNOB_CLICK_0;
    }
}
 
//-- Read and decipher analog switch inputs
//      returns N_BUTTONS if no buttons pressed
 
byte ReadButtons(int PinNumber) {
 
word RawButton;
byte ButtonNum;
 
    RawButton = analogRead(PinNumber);
 
    for (ButtonNum = 0; ButtonNum <= N_BUTTONS; ButtonNum++){
        if (RawButton < ButtonThreshold[ButtonNum])
            break;
    }
 
    return ButtonNum;
 
}
 
//------------------
// Set things up
 

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
#define PIN            8

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
//#if defined (__AVR_ATtiny85__)
//  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
//#endif
  // End of trinket special code

  pixels.begin(); // This initializes the NeoPixel library.
  //Neopixel finish
    pinMode(PIN_SYNC,OUTPUT);
    digitalWrite(PIN_SYNC,LOW); // show we arrived
 
    analogWrite(PIN_PWM10,0);           // turn off other PWM outputs
    analogWrite(PIN_PWM11,0);
 
    analogWrite(PIN_STROBE,1);          // let Arduino set up default Timer1 PWM
    TCCR1B = 0;                         // turn off Timer1 for strobe setup
    TCCR1A = 0x82;                      // clear OCR1A on match, Fast PWM, lower WGM1x = 14
    ICR1 = FlashPdCt;
    OCR1A = FlashLengthCt;
    TCNT1 = FlashLengthCt - 1;
    TCCR1B = 0x18 | TCCRxB_CS;          // upper WGM1x = 14, Prescale 1:64, start Timer1
 
    pinMode(PIN_KNOB_B,INPUT_PULLUP);
    pinMode(PIN_KNOB_A,INPUT_PULLUP);
 
    KnobState = digitalRead(PIN_KNOB_A);
    Button = PrevButton = ReadButtons(PIN_BUTTONS);
 
    attachInterrupt((PIN_KNOB_A - 2),KnobHandler,CHANGE);
 
//    Serial.begin(9600);
    fdevopen(&s_putc,0);                // set up serial output for printf()
 
    printf("Stroboscope Tachometer\r\nEd Nisley - KE4ZNU - December 2012\r\n");
 
    printf("Frequency: %d.%02d\nPulse duration: %d us\n",
           (int)FlashFreq,(int)(100.0 * (FlashFreq - trunc(FlashFreq))),
           (int)(1e6 * FlashLength));
 
    MillisThen = millis();
 
}
 
//------------------
// Run the test loop
 
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
    
    MillisNow = millis();
 
    if ((MillisNow - MillisThen) > UPDATEMS) {
 
        digitalWrite(PIN_SYNC,HIGH);
 
        Button = ReadButtons(PIN_BUTTONS);
        if (PrevButton != Button) {
            if (Button == N_BUTTONS) {
//              printf("Button %d released\n",PrevButton);
                FreqIncr = 1.0;
            }
            else
//              printf("Button %d pressed\n",Button);
//              if (Button == SW_KNOB)
                    FreqIncr = 0.01;
            PrevButton = Button;
        }
 
        if (KnobCounter) {
            FlashFreq += (float)KnobCounter * FreqIncr;
            KnobCounter = 0;
 
            FlashFreq = constrain(FlashFreq,FreqMin,FreqMax);
            FlashFreq = round(100.0 * FlashFreq) / 100.0;
 
            FlashPd = 1.0 / FlashFreq;
            FlashPdCt = FlashPd / TICKPD;
 
            noInterrupts();
            TCCR1B &= 0xf8;             // stop Timer1
            ICR1 = FlashPdCt;           // set new period
            TCNT1 = FlashPdCt - 1;      // force immediate update
            TCCR1B |= TCCRxB_CS;        // start Timer1
            interrupts();
 
            printf("Frequency: %d.%02d\n",
                   (int)FlashFreq,(int)(100.0 * (FlashFreq - trunc(FlashFreq))));
        }
 
        digitalWrite(PIN_SYNC,LOW);
 
        MillisThen = MillisNow;
    }
 
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
