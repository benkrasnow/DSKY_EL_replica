#include <SPI.h>

//Pin definitions
#define PIN_SPI_DAT 4  //Using hardware SPI.  Pins are defined here just for completeness
#define PIN_SPI_CLK 3
#define PIN_SPI_LE 0  //Latch enable on both the HV513 nad HV507
#define PIN_HV_CTRL PA16  // This constant is not used in the code, PA16 is accessed via registers directly
#define PIN_RED_LED 13

#define USE_HV507       //The original code worked with other high voltage switching ICs, so other definitions were possible, but this code only works with the HV507

#define SR_TIMER 18500   //  This value determines how often the EL display is refreshed.  The refresh rate is 12MHz/SR_TIMER.  Example:   12MHz/18500 = 648 Hz
                         //  Higher refresh rates will create a brighter, and more blue-colored display
                         //  The DSKY project needed a more-green phosphor, so I've adjusted the refresh rate much lower than used on other projects (8KHz) to make the display as green as possible

#define MAX_BRIGHTNESS 7   // Number of grayscale levels
                            // The effective frame rate is the refresh rate divided by MAX_BRIGHTNESS + 1.  Example:   648Hz / 8 = 81Hz
                            // This value detemines the tradeoff between number of grayscale levels, and smoothness.  Starting with a higher refresh rate provides more choices.

#define HV_ADC_SETPOINT 800   // ADC value to which the high voltage will be regulated.  ADC is set to 10 bits, reference is 3.3V, and HV divider is 100, so divide HV_ADC_SETPOINT by 3.1 to get voltage.  (2^10/(3.3*100)
                              //  600/3.1 = 193 V
                              //  850 / 3.1 = 274 V
                               //-------- BE CAREFUL! Setting this value too high can destroy hardware! ----------  


#ifdef USE_HV507
#define NUM_PIXELS 192
#define SR_BYTES 24
#endif

const uint8_t com_electrode_mask[SR_BYTES] = {  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                                0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
                                                0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x38};  // MSB first.  Each bit indicates if this electrode is common (1 = common, 0 = segment)


// 21 7-seg characters  3 plus/minus  and 1 special
const uint8_t seg_lookup[25][7] =
    {  //a, b, c, d, e, f, g
        {0,1,89,54,23,5,7},  //0  //prog digits
        {3,56,89,54,23,5,7},
        {3,56,89,54,23,5,7},      //verb digits
        {3,56,89,54,23,5,7},
        {3,56,89,54,23,5,7},  //4   //noun digits
        {3,56,89,54,23,5,7},
        {3,56,89,54,23,5,7},        //  plus/minus
        {3,56,89,54,23,5,7},         //top row of five digits
        {3,56,89,54,23,5,7},  //8
        {3,56,89,54,23,5,7},
        {3,56,89,54,23,5,7},
        {3,56,89,54,23,5,7},
        {3,56,89,54,23,5,7},  //12   plus/minus
        {3,56,89,54,23,5,7}, 
        {3,56,89,54,23,5,7},
        {3,56,89,54,23,5,7},
        {3,56,89,54,23,5,7},  //16
        {3,56,89,54,23,5,7},
        {3,56,89,54,23,5,7},         //plus /minus
        {3,56,89,54,23,5,7},
        {3,56,89,54,23,5,7},  //20
        {3,56,89,54,23,5,7},              
        {3,56,89,54,23,5,7},              
        {3,56,89,54,23,5,7},             
        {3,56,89,54,23,5,7}  //24           special character for boxes and bars on display
    
    };




//Global variables
int pixelvalues[NUM_PIXELS];  //Range is 0 (off) to MAX_BRIGHTNESS.  Originally, I used a floating point value (0 to 1.0), but this was too slow to process in the ISR.
SPISettings SPI_HV507(8000000, MSBFIRST, SPI_MODE0);  //8MHz limit for HV507



void setup() 
{ 
  //Clock distribution to the counters---------------------------------------------
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK0 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0
                     GCLK_CLKCTRL_ID_TCC0_TCC1 ;  // Feed GCLK0  TCC0, and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK0 to TCC2 and TC3
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0
                     GCLK_CLKCTRL_ID_TCC2_TC3 ;   // Feed GCLK0  TCC2, and TC3
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization





 //  The following counter will trigger an interrupt that initiates a SPI transaction with the shift register-----------------------------------------------
 //  In an alternative system, the frequency output was used external of the MCU, but it's not needed any longer, so the port setup is commented out.
 //  DMA could have been used, but we're only sending either one or 8 bytes at 8.5KHz, so the DMA overhead may cancel any benefit for such a small transaction. Most of the time is spent in computation, not SPI transmission
 //  PORT->Group[0].PINCFG[8].reg |= PORT_PINCFG_PMUXEN;
 //  PORT->Group[0].PMUX[8 >> 1].reg = PORT_PMUX_PMUXE_F;
  
  REG_TCC1_WAVE |=  TCC_WAVE_WAVEGEN_NPWM;    // Setup simple PWM
  while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a value set by the PER register
  REG_TCC1_PER = SR_TIMER;         // Set the frequency of the interrupt on TCC1  12MHz / 1200 = 10KHz ;  12MHz / 1400 = 8.57KHz;  12Mhz / 14000 = 857 Hz
  while (TCC1->SYNCBUSY.bit.PER);                // Wait for synchronization
  
  REG_TCC1_CC0 = SR_TIMER;         // TCC0 CC0
  while (TCC1->SYNCBUSY.bit.CC0);                // Wait for synchronization

  //Enable an interrupt on a counter (CNT) event
  REG_TCC1_INTENSET |= TCC_INTENSET_CNT;

  //Specify the interrupting counter event to be an END event
  //This could have been done with an overflow interrupt event as well, but this arangement was left over from an alterative control system, and there is nothing wrong with using a CNT event
  REG_TCC1_EVCTRL |= TCC_EVCTRL_CNTSEL_END;

  NVIC_EnableIRQ(TCC1_IRQn);
  NVIC_SetPriority(TCC1_IRQn, 0); 
  
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV4 |    // Divide GCLK4 by 4 48MHz/4 = 12MHz
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization





  //Setup PWM output for control of high voltage supply------------------------------------------
  
  PORT->Group[0].PINCFG[16].reg |= PORT_PINCFG_PMUXEN;    //Enable pin multiplexer
  PORT->Group[0].PINCFG[16].reg |= PORT_PINCFG_PULLEN;    //Enable pull-down resistor on this pin.  In timer one-shot mode, the pin appears to go high-Z after the timer ends, but cannot find this in the documentation
  PORT->Group[0].PMUX[16 >> 1].reg = PORT_PMUX_PMUXO_E | PORT_PMUX_PMUXE_E;   // PA16 is an even-numbered pin, so PMUXE_E  means set this even numbered pin to port function "E", which is TCC2
  
  REG_TCC2_WAVE |=  TCC_WAVE_WAVEGEN_NPWM  | TCC_WAVE_POL(0xF) ;    // Setup normal PWM and invert polarity on all 4 CC channels
  while (TCC2->SYNCBUSY.bit.WAVE);                                  // "invert" means the output is normally low.  The counter starts at 0, and counts up, turning on the ouput when it matches CCx
  
  REG_TCC2_PER = 2000;         // Period:  set the frequency of the PWM on TCC2  3MHz/2000 = 1.5KHz
  while (TCC2->SYNCBUSY.bit.PER);               
  
  REG_TCC2_CC0 = 2001;         // CC is initially set *above* PER, which means the output will never go high
  while (TCC2->SYNCBUSY.bit.CC0);    


  //The counter is configured as a one-shot, and PA16 goes low when CC0 is reached or the cycle ends.  The pin actually appears to go Hi-Z when the cycle ends -- not sure.
  //This was done so that if the MCU halts for some unforeseen reason, the high voltage supply is shut down by default (fail safe).
  //This arrangement requires the ISR to run periodically in order for high voltage to be generated
  REG_TCC2_CTRLBSET = TCC_CTRLBSET_ONESHOT;
  while (TCC2->SYNCBUSY.bit.CTRLB);

  //Enable an interrupt on an overflow (the counter reaches PER)
  TCC2->INTENSET.reg = TCC_INTENSET_OVF;
  
  NVIC_EnableIRQ(TCC2_IRQn);
  NVIC_SetPriority(TCC2_IRQn, 1);
  
  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV16 |    // Divide GCLK4 by 16  48MHz/16 = 3MHz
                    TCC_CTRLA_ENABLE;             // Enable the TCC2 output
  while (TCC2->SYNCBUSY.bit.ENABLE);              // Wait for synchronization





  //The default ADC setup for the M0 in Arduino is super slow -- over one millisecond per conversion, so we need to speed it up! -------------------------------------------------------------
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV32 |    // Divide Clock ADC GCLK by 32 (48MHz/32 = 1.5MHz)   Maximum allowed speed is about 2.1 MHz
                 ADC_CTRLB_RESSEL_10BIT;         // Set ADC resolution to 10 bits
  while(ADC->STATUS.bit.SYNCBUSY);                 // Wait for synchronization
  ADC->SAMPCTRL.reg = 0x00;                        // Set max Sampling Time Length to half divided ADC clock pulse


  pinMode(PIN_SPI_LE, OUTPUT);
  digitalWrite(PIN_SPI_LE, HIGH);

  pinMode(PIN_RED_LED, OUTPUT);
  digitalWrite(PIN_RED_LED, LOW);

  SPI.begin();

  Serial.begin(9600);
}






void loop() {
int i;





delay(20);


/*
for(i = 0; i<=MAX_BRIGHTNESS; i++)
  {
    pixelvalues[0] = i;
    delay(20);
  }
delay(500);
  for(i = MAX_BRIGHTNESS; i>=0; i--)
  {
    pixelvalues[0] = i;
    delay(20);
  }
delay(500);
*/

if(Serial.available())
  {
    DSKY_set_char(0,Serial.read(),MAX_BRIGHTNESS);
  }

}






//Don't forget that this MCU is also running a USB ISR if used normally through the Arduino IDE as an Adafruit Trinket M0.  This uses some CPU time, but also allows serial communication for new features or debugging.


//  This ISR will send data to the shift register.  Total duration measured at 250us for three HV507's. Runs at 648Hz, so 16% of CPU time. The SPI shiftout takes 63 us, the remainder is all of the bit shifting, which could be more efficient
//  The code in here has to be very efficient.  I originally used a floating point array to store the pixel brightness values, and did floating point math in the ISR, but this was *way* too slow.
void TCC1_Handler() 
{
  static uint8_t srvalue[SR_BYTES];
  static int fade_step;
  static int com_electrode_phase;
  int i;

 
   digitalWriteDirect(PIN_RED_LED, HIGH);
  //  __disable_irq();
 

  for(i = 0; i<SR_BYTES; i++)  //Initialize the shift register with the common electrode mask
        {
          srvalue[i] = (com_electrode_phase == 0) ? com_electrode_mask[i] : ~com_electrode_mask[i];
        }

  for(i =0; i< NUM_PIXELS; i++)  //Set all of the segment electrodes
        {    
          if (fade_step >= pixelvalues[i])   // this pixel should be dark, so set its state equal to the common electrode by flipping it with the xor-equals
            {                       
              srvalue[i>>3] ^= (1 << ((i%8))) & ~(com_electrode_mask[i>>3]);  // toggle the bit only if it is *not* in the common electrode mask.  i>>3 gets the byte position  and i%8 gets the bit position
            }
        }

  com_electrode_phase = (com_electrode_phase == 1) ? 0 : 1;
  fade_step = ++fade_step > MAX_BRIGHTNESS ? 0 : fade_step;


 #ifdef USE_HV507
  digitalWriteDirect(PIN_SPI_LE, LOW);  //Latch enable is brought low while shifting in new data
  SPI.beginTransaction(SPI_HV507);
  for(i = SR_BYTES; i>= 0; i--)   //Always most significant byte first, and most significant bit first
    {
      SPI.transfer(srvalue[i]);
    }
  
  SPI.endTransaction();
  digitalWriteDirect(PIN_SPI_LE, HIGH);
 #endif
  
 REG_TCC1_INTFLAG |= TCC_INTFLAG_CNT;
 NVIC_ClearPendingIRQ(TCC1_IRQn);

   digitalWriteDirect(PIN_RED_LED, LOW);
//   __enable_irq();
}





//  Voltage regulation interrupt handler.  Duration measured at 15us, runs at 1.5KHz, so about 2% of all CPU time
void TCC2_Handler() 
{ 
  __disable_irq();
   NVIC_ClearPendingIRQ(TCC2_IRQn);
  TCC2->INTFLAG.reg = TCC_INTFLAG_OVF;
  
  if(analogRead(A0) < HV_ADC_SETPOINT)  //Voltage is below setpoint, so set CC0 to a low value to make the output go high near the beginning of the timing cycle.
    {
      REG_TCC2_CC0 = 40;        // This must not be set to 0 or "too low" a value.  The LT3468 datasheet specs a minimum low time of 20us. The counter is 3MHz, so this delay of "40" is 13us, plus the ISR latency.   
      while (TCC2->SYNCBUSY.bit.CC0);   //I've destroyed an LT3468 by cycling the control line too rapidly.  Also, it might work to keep this line high continuously, but it's not easy to do that with this MCU timer config, and
    }                                   // sometimes the LT3468 decides it's done charging and ignores the input, so the control line must be cycled often to have decent voltage control.
  else
    {
      REG_TCC2_CC0 = 2001;        // Voltage is above setpoint, so set CC0 above PER, causing the output to always be low
      while (TCC2->SYNCBUSY.bit.CC0); 
    }
  
  TCC2->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;


  __enable_irq();
}




inline void digitalWriteDirect(int PIN, boolean val)
{
  if(val)  PORT->Group[g_APinDescription[PIN].ulPort].OUTSET.reg = (1ul << g_APinDescription[PIN].ulPin);
  else     PORT->Group[g_APinDescription[PIN].ulPort].OUTCLR.reg = (1ul << g_APinDescription[PIN].ulPin);
}






void DSKY_set_char(int char_position, char input_char, int br)
  {
    
    switch(toupper(input_char)) 
      {

         case ' ':
          pixelvalues[seg_lookup[char_position][0]] = 0;  //a
          pixelvalues[seg_lookup[char_position][1]] = 0;  //b
          pixelvalues[seg_lookup[char_position][2]] = 0;  //c
          pixelvalues[seg_lookup[char_position][3]] = 0;  //d
          pixelvalues[seg_lookup[char_position][4]] = 0;  //e
          pixelvalues[seg_lookup[char_position][5]] = 0;  //f
          pixelvalues[seg_lookup[char_position][6]] = 0;  //g
         break;
         
        case '0':
          pixelvalues[seg_lookup[char_position][0]] = br;  //a
          pixelvalues[seg_lookup[char_position][1]] = br;  //b
          pixelvalues[seg_lookup[char_position][2]] = br;  //c
          pixelvalues[seg_lookup[char_position][3]] = br;  //d
          pixelvalues[seg_lookup[char_position][4]] = br;  //e
          pixelvalues[seg_lookup[char_position][5]] = br;  //f
          pixelvalues[seg_lookup[char_position][6]] = 0;  //g
         break;

        case '1':
          pixelvalues[seg_lookup[char_position][0]] = 0;  //a
          pixelvalues[seg_lookup[char_position][1]] = br;  //b
          pixelvalues[seg_lookup[char_position][2]] = br;  //c
          pixelvalues[seg_lookup[char_position][3]] = 0;  //d
          pixelvalues[seg_lookup[char_position][4]] = 0;  //e
          pixelvalues[seg_lookup[char_position][5]] = 0;  //f
          pixelvalues[seg_lookup[char_position][6]] = 0;  //g
         break;

        case '2':
          pixelvalues[seg_lookup[char_position][0]] = br;  //a
          pixelvalues[seg_lookup[char_position][1]] = br;  //b
          pixelvalues[seg_lookup[char_position][2]] = 0;  //c
          pixelvalues[seg_lookup[char_position][3]] = br;  //d
          pixelvalues[seg_lookup[char_position][4]] = br;  //e
          pixelvalues[seg_lookup[char_position][5]] = 0;  //f
          pixelvalues[seg_lookup[char_position][6]] = br;  //g
         break;


         case '3':
          pixelvalues[seg_lookup[char_position][0]] = br;  //a
          pixelvalues[seg_lookup[char_position][1]] = br;  //b
          pixelvalues[seg_lookup[char_position][2]] = br;  //c
          pixelvalues[seg_lookup[char_position][3]] = br;  //d
          pixelvalues[seg_lookup[char_position][4]] = 0;  //e
          pixelvalues[seg_lookup[char_position][5]] = 0;  //f
          pixelvalues[seg_lookup[char_position][6]] = br;  //g
         break;

         case '4':
          pixelvalues[seg_lookup[char_position][0]] = 0;  //a
          pixelvalues[seg_lookup[char_position][1]] = br;  //b
          pixelvalues[seg_lookup[char_position][2]] = br;  //c
          pixelvalues[seg_lookup[char_position][3]] = 0;  //d
          pixelvalues[seg_lookup[char_position][4]] = 0;  //e
          pixelvalues[seg_lookup[char_position][5]] = br;  //f
          pixelvalues[seg_lookup[char_position][6]] = br;  //g
         break;

         case '5':
          pixelvalues[seg_lookup[char_position][0]] = br;  //a
          pixelvalues[seg_lookup[char_position][1]] = 0;  //b
          pixelvalues[seg_lookup[char_position][2]] = br;  //c
          pixelvalues[seg_lookup[char_position][3]] = br;  //d
          pixelvalues[seg_lookup[char_position][4]] = 0;  //e
          pixelvalues[seg_lookup[char_position][5]] = br;  //f
          pixelvalues[seg_lookup[char_position][6]] = br;  //g
         break;

         case '6':
          pixelvalues[seg_lookup[char_position][0]] = br;  //a
          pixelvalues[seg_lookup[char_position][1]] = 0;  //b
          pixelvalues[seg_lookup[char_position][2]] = br;  //c
          pixelvalues[seg_lookup[char_position][3]] = br;  //d
          pixelvalues[seg_lookup[char_position][4]] = br;  //e
          pixelvalues[seg_lookup[char_position][5]] = br;  //f
          pixelvalues[seg_lookup[char_position][6]] = br;  //g
         break;

         case '7':
          pixelvalues[seg_lookup[char_position][0]] = br;  //a
          pixelvalues[seg_lookup[char_position][1]] = br;  //b
          pixelvalues[seg_lookup[char_position][2]] = br;  //c
          pixelvalues[seg_lookup[char_position][3]] = 0;  //d
          pixelvalues[seg_lookup[char_position][4]] = 0;  //e
          pixelvalues[seg_lookup[char_position][5]] = 0;  //f
          pixelvalues[seg_lookup[char_position][6]] = 0;  //g
         break;

         case '8':
          pixelvalues[seg_lookup[char_position][0]] = br;  //a
          pixelvalues[seg_lookup[char_position][1]] = br;  //b
          pixelvalues[seg_lookup[char_position][2]] = br;  //c
          pixelvalues[seg_lookup[char_position][3]] = br;  //d
          pixelvalues[seg_lookup[char_position][4]] = br;  //e
          pixelvalues[seg_lookup[char_position][5]] = br;  //f
          pixelvalues[seg_lookup[char_position][6]] = br;  //g
         break;
         
        case '9':
          pixelvalues[seg_lookup[char_position][0]] = br;  //a
          pixelvalues[seg_lookup[char_position][1]] = br;  //b
          pixelvalues[seg_lookup[char_position][2]] = br;  //c
          pixelvalues[seg_lookup[char_position][3]] = 0;  //d
          pixelvalues[seg_lookup[char_position][4]] = 0;  //e
          pixelvalues[seg_lookup[char_position][5]] = br;  //f
          pixelvalues[seg_lookup[char_position][6]] = br;  //g
         break;




         
      }
      
  }
