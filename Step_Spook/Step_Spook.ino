/*
   Adapted from Low Power SLEEP modes for Arduino UNO/Nano
   using Atmel328P microcontroller chip video by Ralph S Bacon

   For full details see his videos #115, #159, #166, and all his other inspiring and insightful videos!
   at https://www.youtube.com/ralphbacon

   All details can be found at https://github.com/ralphbacon

   I've added the use of WS2812b LED's using the FastLED Arduino library:
   https://github.com/FastLED/FastLED
   (October 2019, Martin Bøgelund)
*/
#include "Arduino.h"
#include <avr/sleep.h>
#include <FastLED.h>

//                (Physical pin)
#define wakePin 3   // (5) when rising, makes 328P wake up, must be an interrupt pin (2 or 3 on ATMEGA328P)
#define beepPin 5   // (11) output pin for the LED (to show it is awake) PWM capable
#define ldrPIN A0   // (23) Determines light/dark state and prevent PIR from waking up unless dark
#define pwrLDR 7    // (13) A1 used as digital out
#define batIN A1    // (24) Used to read the battery voltage
#define batOUT 13   // (19) Closest physical pin to A1
#define spookPin 8  // (14) Data out pin for Step Spooks LED eyes (WS2812b LED's)
#define DEBUG TRUE
#define numSpookLeds 2   // We daisy chain 2 WS2812b LED's for the Spooks eyes

// Array of pins in use; all others get set to INPUT to reduce power
// char pinsInUse[] = {8, 3, 5, 14, 18};

unsigned long awakeTimeMs = 0;
unsigned long lastMillis = 0;
unsigned char prevPIRState = 0;

CRGB spookLeds[numSpookLeds];

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
  Serial.print("F_CPU: "); Serial.println(F_CPU); 
#endif

  // Set all [unused?] pin to INPUT mode to reduce power
  for (auto cnt = 0; cnt < 20; cnt++) {
    pinMode(cnt, INPUT_PULLUP);
  }

  // Wake pin will be brought HIGH/LOW by PIR
  pinMode(wakePin, INPUT_PULLUP);

  // Lighting Spook eyes just to show the µController is running
  FastLED.addLeds<WS2812, spookPin, GRB>(spookLeds, numSpookLeds);
  for(int loop = 0; loop < 4; loop++) {
    uint8_t bright = 255;
    spookLeds[0] = CRGB(bright, bright, bright);
    spookLeds[1] = CRGB(bright, bright, bright);
    FastLED.show();
    delay(40);

    spookLeds[0] = CRGB(0, 0, 0);
    spookLeds[1] = CRGB(0, 0, 0);
    FastLED.show();  
    delay(20);
  }
  
  // Power to potential divider with LDR. Ensures no power used until we read it.
  pinMode(pwrLDR, OUTPUT);
  digitalWrite(pwrLDR, LOW);

  // 1.1V internal reference voltage. Measure it on pin 21.
  analogReference(INTERNAL);

  /*
    If you are doing only ADC conversions on some or all of the analog inputs
    you should disable the digital buffers, to save power.

    Once disabled, a digitalRead on those pins will always read zero.
    http://www.gammon.com.au/adc
  */
  bitSet (DIDR0, ADC0D);  // disable digital buffer on A0
  bitSet (DIDR0, ADC1D);  // disable digital buffer on A1
  bitSet (DIDR0, ADC2D);  // disable digital buffer on A2
  bitSet (DIDR0, ADC3D);  // disable digital buffer on A3
  //bitSet (DIDR0, ADC4D);  // disable digital buffer on A4 <- we are using this pin as digital
  bitSet (DIDR0, ADC5D);  // disable digital buffer on A5

#ifdef DEBUG
  Serial.println("Setup completed.");
#endif
}

// The loop just blinks an LED when not in sleep mode
void loop() {
  #ifdef DEBUG
  Serial.println("Loop section started.");
#endif
  static int previousDark = 0;

  // Just blink LED/beep twice to show we're running. Note that after coming out of sleep
  // there is a deliberate small delay so we don't beep immediately in case it is light
  // when this should go back to sleep at once
  //greenBlink();
  redGlow();

  // How dark is it?
  digitalWrite(pwrLDR, HIGH);
  delay(800);

  // Keep track of ambient light level - read it twice as first read is corrupted
  auto darkness = analogRead(ldrPIN);
  darkness = analogRead(ldrPIN);
  if (darkness != previousDark) {
    previousDark = darkness;
#ifdef DEBUG
    Serial.print("Dark: ");
    Serial.println(darkness);
#endif
  }

  // Is the PIR is still active (LOW=detecting motion)?
  auto pirLevel = digitalRead(wakePin);
  if (pirLevel != prevPIRState) {
    prevPIRState = pirLevel;
#ifdef DEBUG
    Serial.print("PIR=");
    Serial.println(pirLevel == HIGH ? "HIGH" : "LOW");
#endif
  }

  // Is it light (> 400) then go to sleep at once
  // If it is dark AND it's been running for 5+ seconds AND PIR has not detected activity, go to sleep
  if (darkness > 500
      || (millis() > awakeTimeMs + 1000
          && pirLevel == HIGH))
  {
    // Turn off the LDR power
    digitalWrite(pwrLDR, LOW);

    // Disable the ADC (Analog to digital converter, pins A0 [14] to A5 [19])
    static byte prevADCSRA = ADCSRA;
    ADCSRA = 0;

    /* Set the type of sleep mode we want. Can be one of (in order of power saving):
        SLEEP_MODE_IDLE (Timer 0 will wake up every millisecond to keep millis running)
        SLEEP_MODE_ADC
        SLEEP_MODE_PWR_SAVE (TIMER 2 keeps running)
        SLEEP_MODE_EXT_STANDBY
        SLEEP_MODE_STANDBY (Oscillator keeps running, makes for faster wake-up)
        SLEEP_MODE_PWR_DOWN (Deep sleep)
    */
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    // Turn off Brown Out Detection (low voltage)
    // Thanks to Nick Gammon for how to do this (temporarily) in software rather than
    // permanently using an avrdude command line.
    //
    // NB Microchip state BODS and BODSE only available for picoPower devices ATmega48PA/88PA/168PA/328P
    //
    // BODS must be set to one and BODSE must be set to zero within four clock cycles. This sets
    // the MCU Control Register (MCUCR)
    MCUCR = bit (BODS) | bit (BODSE);

    // The BODS bit is automatically cleared after three clock cycles so we better get on with it
    MCUCR = bit (BODS);

    // Ensure we can wake up again by first disabling interupts (temporarily) so
    // the wakeISR does not run before we are asleep and then prevent interrupts,
    // and then defining the ISR (Interrupt Service Routine) to run when poked awake
    noInterrupts();
    attachInterrupt(digitalPinToInterrupt(wakePin), sleepISR, FALLING);

    // Send a message just to show we are about to sleep
#ifdef DEBUG
    Serial.println("Good night!");
    Serial.flush();
#endif

    // Allow interrupts now
    interrupts();

    // And enter sleep mode as set above
    sleep_cpu();

    // --------------------------------------------------------
    // µController is now asleep until woken up by an interrupt
    // --------------------------------------------------------

    // Wakes up at this point when wakePin is RISING - interrupt routine is run first
#ifdef DEBUG
    Serial.println("I'm awake!");
#endif

    // Reset the awake Counter so we stay awake for a minimum amount of time (unless it is light)
    awakeTimeMs = millis();
#ifdef DEBUG
    Serial.print("awakeTimeMs = ");
    Serial.println(awakeTimeMs);
#endif

    // Reset beeper timer so we beep almost immediately on wakeup (if movement and dark)
    lastMillis = millis() - 200;
    previousDark = 0;
#ifdef DEBUG
    Serial.print("lastMilliss = ");
    Serial.println(lastMillis);
#endif

    // Re-enable ADC if it was previously running
    ADCSRA = prevADCSRA;

    // Read the analog pin once here and discard to stabilize value
    darkness = analogRead(ldrPIN);

    // If the battery is below 3.29v (329) beep a longer series of beeps
    uint16_t batVolts = getBatteryVolts();
#ifdef DEBUG
    Serial.print("Bat: ");
    Serial.println(batVolts);
#endif

    if (batVolts < 329) {
#ifdef DEBUG
      Serial.println("Battery low");
#endif
      redBlink();
    }
  }
}


// When wakePin is brought LOW this interrupt is triggered FIRST (even in PWR_DOWN sleep)
void sleepISR() {
  // Prevent sleep mode, so we don't enter it again, except deliberately, by code
  sleep_disable();

  // Detach the interrupt that brought us out of sleep
  detachInterrupt(digitalPinToInterrupt(wakePin));

  // Now we continue running the main Loop() just after we went to sleep
}

// Slowly turn up eyes in purple, move over to a red glow, kep the red glow for a while
// and then turn down via purple
void redGlow() {
  uint8_t r_brgt = 0;
  uint8_t g_brgt = 0;
  uint8_t b_brgt = 0;
  if (millis() > lastMillis + 1000) {
    for(int i = 0; i < 256; i++) {
      r_brgt = i;
      if(i < 128) {
        b_brgt = i;
      } else {
        b_brgt = 255 - i;
      }
      for(int led_idx = 0; led_idx < numSpookLeds; led_idx++) {
        spookLeds[led_idx] = CRGB(g_brgt, r_brgt, b_brgt);
      }
      FastLED.show();
      delay(20);
    }

    delay(6000);
    for(int i = 255; i >= 0; i--) {
      r_brgt = i;
      if(i < 128) {
        b_brgt = i;
      } else {
        b_brgt = 255 - i;
      }
      for(int led_idx = 0; led_idx < numSpookLeds; led_idx++) {
        spookLeds[led_idx] = CRGB(g_brgt, r_brgt, b_brgt);
      }
      FastLED.show();
      delay(20);
    }
  }
}

// Double blink green just to show we are running. Note that we do NOT
// use the delay for final delay here, this is done by checking
// millis instead (non-blocking)
void greenBlink() {

  // One second must have elapased before we beep again
  if (millis() > lastMillis + 1000) {
    for(int i = 0; i < 2; i++){
      spookLeds[0] = CRGB(255, 0, 0);
      spookLeds[1] = CRGB(255, 0, 0);
      FastLED.show();
      delay(500);

      spookLeds[0] = CRGB(0, 0, 0);
      spookLeds[1] = CRGB(0, 0, 0);
      FastLED.show();
      delay(500);
    }
    lastMillis = millis();
  }
}

// Low battery warning - blink just one eye red
void redBlink() {
  for(int i = 0; i < 3; i++) {
    spookLeds[0] = CRGB(0, 80, 0);
    FastLED.show();
    delay(1000);
    spookLeds[0] = CRGB(0, 0, 0);
    FastLED.show();
    delay(1000);
  }

  // If we are consuming time here reset the blinker timer in case we should be sleeping
  lastMillis = millis() - 600;
}

unsigned int getBatteryVolts() {
  //http://www.gammon.com.au/adc

  // Adjust this value to your boards specific internal BG voltage x1000
  const long InternalReferenceVoltage = 1095L;

  // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
  // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);

  // Let mux settle a little to get a more stable A/D conversion
  delay(50);

  // Start a conversion
  ADCSRA |= _BV( ADSC );

  // Wait for conversion to complete
  while ( ( (ADCSRA & (1 << ADSC)) != 0 ) );

  // Scale the value - calculates for straight line value
  unsigned int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L;
  return results;
}
