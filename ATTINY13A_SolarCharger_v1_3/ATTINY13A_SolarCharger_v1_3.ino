/*
   Program ATTINY13A to read battery voltage,
   turn on LED to bright and dim (PWM) mode,
   and respond to one user switch input that controls LED modes.

   ATTINY13A datasheet found here:
   http://www.atmel.com/images/doc8126.pdf
*/

/*
   PB5 will be used as digital output that turns on LED3
   PB4 will be used as an input that receives user input from a
   switch that controls LED mode
   PB3 will be used as digital output that opens and closes charging
   from solar panel to battery (protects from over charging)
   PB2 will be used as an ADC input to read battery voltage
   PB1 will be used as PWM and digital output that turns on LED2
   PB0 will be used as PWM and digital output that turns on LED1
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/cpufunc.h>

#define OFF 0
#define BRIGHT360 1
#define DIM360 2
#define BRIGHT180 3
#define POWERSAVE 4

boolean SHUTOFF = false;
boolean LOWBATT = false;
boolean HIGHBATT = false;


/*********************************************************
  Helper functions that setup pins and read/write to pins
 *********************************************************/

void PWM(int dutyCycle, int pin) {
  if (pin == PB0) {         //PB0 PWM mode
    //Set PB0 as "clear OC0A on compare match, set at TOP" (pg. 70)
    //Set PBO to fast-PWM mode
    TCCR0A |= (1 << COM0A1);
    TCCR0A |= (1 << WGM01) | (1 << WGM00);

    //Set Timer/Counter clock source to no prescalar
    TCCR0B |= (1 << CS01) | (1 << CS00);

    //Set output compare register to duty cycle value from 0 to 255
    OCR0A = dutyCycle;
  }
  else if (pin == PB1) {    //PB1 PWM mode
    //Set PB1 as "clear OC0B on compare match, set at TOP" (pg. 72)
    //Set PB1 to fast-PWM mode
    TCCR0A |= (1 << COM0B1);
    TCCR0A |= (1 << WGM01) | (1 << WGM00);

    //Set Timer/Counter clock source to no prescalar
    TCCR0B |= (1 << CS01) | (1 << CS00);

    //Set output compare register to duty cycle value from 0 to 255
    OCR0A = dutyCycle;
  }
  //  else if(mode == 4) {  //Blink mode
  //    //Set PB0 as "toggle OC0A on compare match" (pg. 70)
  //    //Set to CTC (clear timer on compare match) mode
  //    TCCR0A|= (1<<COM0A0);
  //    TCCR0A|= (1<<WGM01);
  //
  //    //Set Timer/Counter clock source to 1024 prescalar
  //    TCCR0B|= (1<<CS02)|(1<<CS00);
  //
  //    OCR0A = 255;
  //  }
  else {
    //Turn off PB0 and PB1 timer/counter
    TCCR0A &= ~(1 << COM0A1) & ~(1 << COM0B1);
    TCCR0A &= ~(1 << WGM01) & ~(1 << WGM00);
    TCCR0B &= ~(1 << CS02) & ~(1 << CS01) & ~(1 << CS00);
  }
}

void setupPin(int pin, boolean mode) {
  //Set pin as input or output (pg. 51)
  if (!mode)  {         //input mode
    DDRB &= ~(1 << pin);
    PORTB |= (1 << pin);
  }
  else                //output mode
    DDRB |= (1 << pin);
}

void digitalWrite(int pin, boolean mode) {
  //Set pin as HIGH (mode = 1) or LOW (mode = 0)
  if (mode)
    PORTB |= (1 << pin);
  else
    PORTB &= ~(1 << pin);
}

void pb2_ADC(void) {
  //Set PB2 as ADC input (pg. 92)
  //Set voltage reference as internal voltage reference (1.1V)
  ADMUX |= (1 << MUX0);                       //MUX[1:0] is 01 for PB2
  ADMUX |= (1 << REFS0);                      //1.1V reference
  ADCSRA |= (1 << ADEN);                      //Enables ADC
  ADCSRA |= (1 << ADPS1) | (1 << ADPS2) | (1 << ADPS0); //ADC prescalar set to 128 (i.e. 9.6MHz/128 = 75kHz)
  ADCSRA |= (1 << ADATE);                     //Allows free-running mode
  //start conversion
  ADCSRA |= (1 << ADSC);

  //Set up timing to come from Output Compare B
  //ADCSRB|= (1<<ADTS2)|(1<<ADTS0);             //Trigger source is Timer/Counter Compare Match B
  //TCCR0A|= (1<<COM0B1);                       //Set OC0B at TOP

  DIDR0 |= (1 << ADC1D);                      //Disables digital input to PB2, lower power consumption
}

int pb2_read(void) {
  //Vref = 1.087V
  int val = ADC; //10-bit conversion
  //double battVoltage = val*0.00433; //val/1023*Vref/[91/(91+270)] = .00413
  //value for 88.7 and 287kOhm, 0.00433
  //return battVoltage;    //returns 10-bit conversion
  return ADC;
}

int pb4_read(void) {
  //Reads PB4 bit from PINB register
  return (PINB & (1 << PINB4));
}

int ledTrigger(int modeCounter) {

  if (modeCounter == OFF) {     //Set PB0, PB1, and PB5 LOW (off)
    PWM(256, PB3);
    digitalWrite(PB0, LOW);
    digitalWrite(PB1, LOW);
    digitalWrite(PB5, LOW);
  }
  else if (modeCounter == BRIGHT360) { //Set PB0 HIGH (bright 360 deg)
    digitalWrite(PB0, HIGH);
    digitalWrite(PB1, HIGH);
  }
  else if (modeCounter == DIM360) { //Set PB0 and PB1 to 50% PWM (dim)
    digitalWrite(PB0, LOW);
    digitalWrite(PB1, LOW);
    PWM(127, PB0);
    PWM(127, PB1);
  }
  else if (modeCounter == BRIGHT180) { //Set PB1 and PB5 HIGH (bright 180 deg)
    PWM(0, PB3);                  //Turn off PB0 and PB1 PWM
    digitalWrite(PB1, HIGH);
    digitalWrite(PB5, HIGH);
    modeCounter = -1;
  }
  else if (modeCounter == POWERSAVE) { //Set PB0 to 25% PWM (power save)
    digitalWrite(PB0, LOW);
    PWM(63, PB0);
    modeCounter = -1;
  }
  return modeCounter;
}

void setPrescaler() {
  // Datasheet page 28

  // enable prescaler change
  // this must write a 1 to CLKPCE
  CLKPR = (1 << CLKPCE);

  // change prescaler
  // this must write a 0 to CLKPCE
  CLKPR = (1 << CLKPS2);
  // this sets the clock to 9.6/16 = 0.6MHz

}


/*************************************************
                Main function
**************************************************/

int main(void) {

  int modeCounter = OFF; // Counts switch mode (0 = OFF, 1 = LED1&LED2 ON,
  // 2 = LED1&LED2 DIM, 3 = LED2&LED3 ON, 4 = LED1&LED2 POWERSAVE)
  setPrescaler();

  setupPin(PB0, 1);
  setupPin(PB1, 1);
  pb2_ADC();
  setupPin(PB3, 1);
  setupPin(PB4, 0);
  setupPin(PB5, 1);

  while (1) {

    int batt = pb2_read();


    HIGHBATT = false;
    LOWBATT = false;
    SHUTOFF = false;
    
    if (batt > 1012)
      HIGHBATT = true;
    else if ((batt < 711) & (batt > 600))
      LOWBATT = true;
    else if (batt < 600)
      SHUTOFF = true;
    

    // Do not allow charging if battery voltage is too high (>4.2V)
    if (HIGHBATT)
      digitalWrite(PB3, HIGH);
    else
      digitalWrite(PB3, LOW);

    // Automatically set to off mode if on when below critical battery voltage (<2.8V)
    if (SHUTOFF)
      modeCounter = ledTrigger(OFF);
    // Automatically set to dim mode if in bright mode when low battery voltage (<3.0V)
    else if (LOWBATT && (modeCounter == BRIGHT360 | modeCounter == BRIGHT180))
      modeCounter = ledTrigger(POWERSAVE);

    // Read switch input
    int b4_input = pb4_read();

    // PINB4 reads LOW when switch is closed
    if (!b4_input) {
      modeCounter++;
      modeCounter = ledTrigger(modeCounter);

      int buttonDownTime = 0;
      //wait for user to release switch
      while (!b4_input && buttonDownTime < 1000) {
        delay(1);
        buttonDownTime++;
        b4_input = pb4_read();
      }
    }
  }
}
