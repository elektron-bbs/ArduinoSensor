/*  Demo Sketch for remote sensor lib for an ATTiny85 with 1Mhz
    RemoteSensor library v2.5 (201412) for Arduino
    Library to encode, encrypt and transmits data
    2014  N.Butzek, S.Butzek
    This library contains classes to perform encoding of radio signals in the 433MHz
    band, which are typical for home automation, while the intention was, to use
    them for various self made Arduino sensors to transmit temperatures, humidity,
    light, doorswitch info, ...
    As this is an early state of the developpment, two encoding approaches
    are realized. One uses manchester encoding and the other uses common pulse pause modulation.
    Additional information on manchester encoding can be found in
    https://github.com/mchr3k/arduino-libs-manchester

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    For compiling in Arduino-IDE use ATTinyCore:
    http://drazzy.com/package_drazzy.com_index.json
    select Attiny85
    
    Average current consumption 15 µA at 3,3 Volt / 1 MHz.
*/

#define PROGNAME               "Reedsensor"
#define PROGVERS               "0.3"
#define DeviceID                2
#define PWRDEV                  0   // Power Supply for DHT11
#define TRANSMITTER             3   // 433Mhz Transmitter DATA PIN
#define VCC 3600.0 //nominal voltage of the batteries, serves for battery check
//#define VCC 4500.0 //nominal voltage of the batteries, serves for battery check
#define POWERDOWN
//#define TEST
#define CALVCC // see below

//#define USE_MANCHESTER_CODING 1// Define to use Mancheser encoding, if not defined RTZ Coding will be used
#include "SensorTransmitter.h"
//#include <avr/interrupt.h>

//#include <Wire.h>
#include "SI7021.h"
SI7021 sensor;

#ifdef POWERDOWN
#include <avr/sleep.h>
#include <avr/wdt.h>
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#endif // POWERDOWN

void setup_watchdog(int ii);
void system_sleep();

//uint8_t deviceID  = 0;
// Initialization of all Transmitters on pin 9, with defines ID
/************************************
   Device Type
   0
   1
   2
   3  light hirange (outdoor)
   4	light hires (indoor)
   5	water
   6	temp
   7	reed gas
   8  voltage
   9  humidity
  10 raw 
   ..16
 ************************************/

asTransmitter temp(6, DeviceID, TRANSMITTER); // (DeviceType, DeviceID, OutputPin)
asTransmitter voltage(8, DeviceID, TRANSMITTER); // (DeviceType, DeviceID, OutputPin)
asTransmitter hum(9, DeviceID, TRANSMITTER); // (DeviceType, DeviceID, OutputPin)

const uint32_t reed_debounceDelay = 150; //minimu duration bewteen two low pulses

uint16_t vcc = 0; //supply voltage
byte battery = 0;
uint16_t tempVal = 0;
uint16_t humVal = 0;
uint8_t transmitCnt = 0;
bool trigger = 1; // manual send (via reset button)

/************************************
   Calibration of Vcc
   Calibration of the 1.1V reference requires an external measurement of Vcc with a voltmeter, for example.
   Follow these steps:
    1. Measure Vcc with the voltmeter => Vcc1
    2. Define CALVCC to let Arduino measure Vcc internal and print the result to Serial
    3. Enter the above measured values in the formula, overwriting the default values
       Make shure to use the same units.
 ************************************/
//internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
#ifndef CALVCC
//const int internalRef = 1.1 * 3160 / 3120; //with compensation
//const float internalRef = 0.98351648351;
const float internalRef = 0.96173822714;
const long scale_constant = internalRef * 1023 * 1000;
#else
// const float internalRef = 1.1 ; // for calibration no compensation is required
 const float internalRef = 1.1 * 3600 / 3550; // with compensation
// const float internalRef = 1.1;
const long scale_constant = internalRef * 1023 * 1000;
#endif

void setup() {
  srand (analogRead(PWRDEV));
  pinMode(TRANSMITTER, OUTPUT);

  if (MCUSR & _BV(WDRF)) {          // If a reset was caused by the Watchdog Timer...
    MCUSR &= ~_BV(WDRF);                 // Clear the WDT reset flag
    WDTCR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
    WDTCR = 0x00;                      // Disable the WDT
  }
  //  digitalWrite(PWRTRANSMITTER,1); //power supply for transmitter on digital pin
  checkBattery();// this will also measure and set vcc

#ifdef POWERDOWN
  setup_watchdog(WDTO_2S);                     // approximately 2 seconds sleep
#endif
  TinyWireM.begin();                    // initialize I2C lib
  //sensor.begin();
}

// possible sleep time: 15MS, 30MS, 60MS, 120MS, 250MS, 500MS, 1S, 2S, 4S, 8S
void loop() {
  aquire();
  transmit();
  sleep();
  trigger = 0; //after first run, this will remain 0 (automatic) => on reset button, trigger will show manual
  //wdt_reset();
}

uint16_t measureVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(MUX3) | _BV(MUX2);

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion

  while (bit_is_set(ADCSRA, ADSC)); // measuring
  //uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  //uint8_t high = ADCH; // unlocks both
  uint8_t oldSREG = SREG;
  uint16_t adc_result = ADC; //In one read?
  SREG = oldSREG;
  //long result = (high<<8) | low;
  long result = adc_result;
  result = scale_constant  / result; // Calculate Vcc (in mV); 1126400 = 1.1*1024*1000
  return (uint16_t)result; // Vcc in millivolts
}

byte checkBattery() {
  // use global vcc, battery
  vcc = measureVcc();
  battery=3; // optimal  
  if (vcc < 0.92 * VCC) battery = 0; // change
  else if (vcc < 0.94 * VCC) battery = 1; // good
  else if (vcc < 0.97 * VCC) battery = 2; // ok
}

void aquire() {
  ///////////////////// aquire data  ///////////////////////////////////////////
  if (!(transmitCnt % 5)) { // every 5 passes check the battery
    checkBattery(); // this also updates vcc
  }
  tempVal = sensor.getCelsiusHundredths();
  humVal = sensor.getHumidityPercent();
  tempVal = (uint16_t)(tempVal + 0x8000);
  humVal = (uint16_t)(humVal + 0x8000);
}

void transmit() {
  // Reset Timer and Prescaler to default
  /*
    TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
    TCCR0B = 0<<WGM02 | 1<<CS00;
    TCCR1 = 0<<PWM1A | 0<<COM1A0 | 1<<CS10;
    GTCCR = 1<<PWM1B | 2<<COM1B0;
  */
  static uint16_t old_tempVal;
  static uint16_t old_humVal;
  ////////////////// transmitt data  ///////////////////////////////////////////
  //  digitalWrite(PWRTRANSMITTER,1); //power supply for transmitter on digital pin
  // Send only if a sensor has updated Information, or every 5. transmission
  if ((old_tempVal != tempVal) || (transmitCnt % 3 == 0))
    temp.send(tempVal, battery, trigger); //(value[16bit], battery[2bit], trigger[1bit])
  old_tempVal = tempVal;
  if ((old_humVal != humVal) || (transmitCnt % 3 == 0))
    hum.send(humVal, battery, trigger); //(value[16bit], battery[2bit], trigger[1bit])
  old_humVal = humVal;
  // Update Battery every 5. Transmission because this will change always we do not check for changed value
  if (transmitCnt % 5 == 0)
    voltage.send(vcc, battery, trigger); //(value[16bit], battery[2bit], trigger[1bit])
  ++transmitCnt;
  //  digitalWrite(PWRTRANSMITTER,0); //power supply for transmitter on digital pin
}

void sleep() {
  ///////////////////////// go sleep  ///////////////////////////////////////////
#ifdef POWERDOWN
  setup_watchdog(WDTO_8S); // Take longest sleep mode (8 seconds sleep) to reduce wakeups
#endif
//  for (uint8_t i = 0; i < 6; i++) // => 48 S + random 30ms to 2 Seconds + 8 S = 58 S max
  for (uint8_t i = 0; i < 21; i++) // => 168 S + random 30ms to 2 Seconds + 8 S = 178 S max
  {
#ifdef POWERDOWN
    system_sleep();
#else
    delay(random(8000));
#endif
  }
  // generate some random time to avoid sending at fix intervals
#ifdef POWERDOWN
  setup_watchdog(random(1, 7)); // 30ms to 2 Seconds extra Delay to get some variation between transmissions
  system_sleep();
#endif
}

/*
  SLEEP FUNCTIONS
*/
#define BODS 7                   //BOD Sleep bit in MCUCR
#define BODSE 2                  //BOD Sleep enable bit in MCUCR
void system_sleep() {
#ifdef POWERDOWN
  pinMode(TRANSMITTER, INPUT);
  cbi(ADCSRA, ADEN);                   // switch Analog to Digitalconverter OFF
  cbi(ACSR, ACD);                      // disable the analog comparator
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  cli();
  uint8_t mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);  //turn off the brown-out detector
  uint8_t mcucr2 = mcucr1 & ~_BV(BODSE);
  MCUCR = mcucr1;
  MCUCR = mcucr2;
  sei();                               // ensure interrupts enabled so we can wake up again
  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out
  sbi(ADCSRA, ADEN);                   // switch Analog to Digitalconverter ON
  sbi(ACSR, ACD);                      // enable the analog comparator
  pinMode(TRANSMITTER, OUTPUT);
#endif
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {
  cli();
  //  wdt_reset();
  byte bb;
  // int ww;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  //  ww=bb;
  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
  sei();
}

ISR (WDT_vect)
{
  // WDIE & WDIF is cleared in hardware upon entering this ISR
  //wdt_disable();
}
