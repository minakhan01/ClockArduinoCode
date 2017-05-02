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
 
void setup() {
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
 
    Serial.begin(9600);
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
