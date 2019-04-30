#include <SPI.h>

//Pin definitions
#define PIN_SPI_DAT 4  //Using hardware SPI.  Pins are defined here just for completeness
#define PIN_SPI_CLK 3
#define PIN_SPI_LE 0  //Latch enable on both the HV513 nad HV507
#define PIN_HV_CTRL PA16  // This constant is not used in the code, PA16 is accessed via registers directly
#define PIN_RED_LED 13

//User control constants
#define USE_HV507       //The original code worked with other high voltage switching ICs, so other definitions were possible, but this code only works with the HV507

#define USE_UNIPOLAR_DRIVE


#define SR_TIMER 8000   //  This value determines how often the EL display is refreshed.  The refresh rate is 12MHz/SR_TIMER.  Example:   12MHz/18500 = 648 Hz
                         //  Higher refresh rates will create a brighter, and more blue-colored display
                         //  The DSKY project needed a more-green phosphor, so I've adjusted the refresh rate much lower than used on other projects (8KHz) to make the display as green as possible

#define MAX_BRIGHTNESS 7   // Number of grayscale levels
                            // The effective frame rate is the refresh rate divided by MAX_BRIGHTNESS + 1.  Example:   648Hz / 8 = 81Hz
                            // This value detemines the tradeoff between number of grayscale levels, and temporal smoothness.  Starting with a higher refresh rate provides more choices.

#define HV_ADC_SETPOINT 820   // ADC value to which the high voltage will be regulated.  ADC is set to 10 bits, reference is 3.3V, and HV divider is 100, so divide HV_ADC_SETPOINT by 3.1 to get voltage.  (2^10/(3.3*100)
                              //  600/3.1 = 193 V
                              //  850 / 3.1 = 274 V
                               //-------- BE CAREFUL! Setting this value too high can destroy hardware! ----------  

#define TOTAL_DISP_CHAR 25  // Number of characters on the display ( 21 7-seg characters,  3 plus/minus,  and 1 special )


// Three HV507 chips in series
#ifdef USE_HV507
#define NUM_PIXELS 192
#define SR_BYTES 24
#endif


// Each bit indicates if this electrode is common (1 = common, 0 = segment)   
//const uint8_t com_electrode_mask[SR_BYTES] = {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   
//                                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//                                                0x00, 0x00, 0x00, 0x00, 0x1C, 0xFF, 0xFF, 0x00}; 

const uint8_t com_electrode_mask[SR_BYTES] = {  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   
                                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00}; 



//158 is unused
// Mapping of shift register bits with display segment locations:  21 7-seg characters,  3 plus/minus,  and 1 special.  The 7 is hardcoded because they are seven-segment digits
const uint8_t seg_lookup[TOTAL_DISP_CHAR][7] =
    {  //a, b, c, d, e, f, g
        {187,128,130,190,189,188,191},  //0  //PROG two digits
        {131,132,136,137,134,133,135},
        {158,152,148,146,147,153,154},      //VERB  two digits
        {156,151,149,144,150,155,157},
        {141,116,113,112,143,142,111},  //4   //NOUN two  digits
        {139,122,121,119,118,140,120},
        {158,98,158,158,158,158,104},        //6 upper plus/minus  (it's not efficient to use a whole byte for just plus/minus, but creates character consistency and more readable code. a=minus, b=plus.  
        {107,103,101,99,105,106,102},         //upper-most row of five digits
        {108,110,82,86,90,100,83},  //8
        {117,123,69,77,78,114,76},
        {124,67,68,66,65,127,64},
        {125,72,73,74,70,126,71},
        {158,88,158,158,158,158,89},  //12   middle  plus/minus
        {85,84,81,92,91,93,87},           //middle row of five digits
        {79,35,36,37,165,80,38},
        {43,44,46,45,40,41,42},
        {48,52,51,55,49,47,50},  //16
        {58,184,185,56,53,54,186},
        {158,29,158,158,158,158,28},         //18  lower  plus /minus
        {39,25,27,22,23,24,26},           //lower row of five digits
        {18,15,16,19,21,20,17},  //20
        {57,60,10,12,14,13,11},              
        {63,1,7,9,8,59,2},              
        {62,61,4,5,6,0,3},             
        {166,129,167,138,109,75,34}  //24    special character for boxes and bars on display.  top-to-bottom, left-to-right:  a=COMP ACTY, b=PROG, c=VERB, d=NOUN, e=upper line, f=middle line, g=lower line
    
    };




//Global variables
int pixelvalues[NUM_PIXELS];  //Range is 0 (off) to MAX_BRIGHTNESS.  Originally, I used a floating point value (0 to 1.0), but this was too slow to process in the ISR.
                              //This variable is global, and is the only way pathway that graphics drawing functions can affect the shift register ISR
SPISettings SPI_HV507(6000000, MSBFIRST, SPI_MODE0);  //8MHz limit for HV507.  It's global so that it can be located at the top of the code here for easy access, but is only used in the shift register ISR.

int demo_mode = 0;  // Device boots in "attract mode" light show.  When it first receives a byte via USB serial interface, it cancels attract mode and is controlled only via USB.  Or, set this to zero to never use attract mode.

//-----------------------------------------End of user control constants and variables




void setup() 
{ 
  pinMode(PIN_SPI_LE, OUTPUT);
  digitalWrite(PIN_SPI_LE, HIGH);

  pinMode(PIN_RED_LED, OUTPUT);
  digitalWrite(PIN_RED_LED, LOW);
  
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


  

  SPI.begin();

  Serial.begin(9600);

  delay(100);
  
}






void loop() {
int i;
static int char_pos = 0;

//used only for demo mode
static int demo_pos = 0;
static int demo_val = MAX_BRIGHTNESS;






if (demo_mode == 1)
  {
    //test all segments
 //   pixelvalues[demo_pos] = demo_val;
 //   demo_pos = (++demo_pos == NUM_PIXELS) ? 0 : demo_pos;
//    if (demo_pos == 0)
 //     {
 //       demo_val = (demo_val == 0) ? MAX_BRIGHTNESS : 0;
  //    }
  //  Serial.println(demo_pos);
  //  delay(100);


for(i = 0; i< NUM_PIXELS; i++)
  {
    pixelvalues[i] = MAX_BRIGHTNESS;
    if(i > 0)
      {
        pixelvalues[i-1] = 0;
      }
      Serial.println(i);
      while(!Serial.available());
      Serial.read();     
  }

  
  
 }

else
  {
    while(Serial.available())
    {
      
      DSKY_set_char(char_pos,Serial.read(),MAX_BRIGHTNESS);
      char_pos = (++char_pos == TOTAL_DISP_CHAR) ? 0 : char_pos;
      demo_mode = 0;
    }
    delay(5);
  }




  
}






//Don't forget that this MCU is also running a USB ISR if used normally through the Arduino IDE as an Adafruit Trinket M0.  This uses some CPU time, but also allows serial communication for new features or debugging.


//  This ISR will send data to the shift register.  Total duration measured at 250us for three HV507's. Runs at 648Hz, so 16% of CPU time. The SPI shiftout takes 63 us, the remainder is all of the bit shifting, which could be more efficient
//  The code in here has to be very efficient.  I originally used a floating point array to store the pixel brightness values, and did floating point math in the ISR, but this was *way* too slow.
void TCC1_Handler() 
{
  static uint8_t srvalue[SR_BYTES];
  static int fade_step = 1;
  static int com_electrode_phase;
  int i;

 digitalWriteDirect(PIN_RED_LED, HIGH);

  #ifdef USE_UNIPOLAR_DRIVE  //Unipolar   When driving the display in unipolar mode, flipping the segment polarity while holding the common electrode constant will cause it emit light.
        for(i =0; i< NUM_PIXELS; i++)  //Set all of the segment electrodes
          {    
            if (fade_step <= pixelvalues[i])   // this pixel should be emit light on this cycle, so toggle the bit
              {                       
                srvalue[i>>3] ^= (1 << ((i%8))) & ~(com_electrode_mask[i>>3]);  // toggle the bit only if it is *not* in the common electrode mask.  i>>3 gets the byte position  and i%8 gets the bit position
              }
          }



 #else  // Bipolar   When driving the display in bipolar mode, flipping the segment polarity (so that it matches the common electrode's) will cause it to be dark.
        for(i = 0; i<SR_BYTES; i++)  //Initialize the shift register with the common electrode mask
              {
                srvalue[i] = (com_electrode_phase == 0) ? com_electrode_mask[i] : ~com_electrode_mask[i];
              }
      
      
        for(i =0; i< NUM_PIXELS; i++)  //Set all of the segment electrodes
              {    
                if (fade_step > pixelvalues[i])   // this pixel should be dark, so set its state equal to the common electrode by flipping it with the xor-equals
                  {                       
                    srvalue[i>>3] ^= (1 << ((i%8))) & ~(com_electrode_mask[i>>3]);  // toggle the bit only if it is *not* in the common electrode mask.  i>>3 gets the byte position  and i%8 gets the bit position
                  }
              }

  com_electrode_phase = (com_electrode_phase == 1) ? 0 : 1;
 
  #endif





  
  fade_step = (fade_step == MAX_BRIGHTNESS) ? 1 : fade_step + 1;


   __disable_irq();
   
 #ifdef USE_HV507

    SPI.beginTransaction(SPI_HV507);
    for(i = SR_BYTES-1; i>= 0; i--)   //Most significant byte first, and most significant bit first
      {
        SPI.transfer(srvalue[i]);
      }
    
    SPI.endTransaction();
  
    digitalWriteDirect(PIN_SPI_LE, LOW);  //Cycle the latch to move shift register data to the output transistors.  Keep the latch low normally to prevent accidential shift-ins.
    digitalWriteDirect(PIN_SPI_LE, HIGH);
    digitalWriteDirect(PIN_SPI_LE, LOW);
 #endif
  
 REG_TCC1_INTFLAG |= TCC_INTFLAG_CNT;
 NVIC_ClearPendingIRQ(TCC1_IRQn);

   digitalWriteDirect(PIN_RED_LED, LOW);
   __enable_irq();
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

    if( input_char & 0x80)  //most significant bit is one, so use the following seven bits to set the segments directly
      {
          pixelvalues[seg_lookup[char_position][0]] = (input_char & 0x01) ? br : 0;  //a
          pixelvalues[seg_lookup[char_position][1]] = (input_char & 0x02) ? br : 0;  //b
          pixelvalues[seg_lookup[char_position][2]] = (input_char & 0x04) ? br : 0;  //c
          pixelvalues[seg_lookup[char_position][3]] = (input_char & 0x08) ? br : 0;  //d
          pixelvalues[seg_lookup[char_position][4]] = (input_char & 0x10) ? br : 0;  //e
          pixelvalues[seg_lookup[char_position][5]] = (input_char & 0x20) ? br : 0;  //f
          pixelvalues[seg_lookup[char_position][6]] = (input_char & 0x40) ? br : 0;  //g
      }

    else  //If we received a standard ASCII character, parse it as a readable character
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
    
            case '+':
              pixelvalues[seg_lookup[char_position][0]] = 0;  //a
              pixelvalues[seg_lookup[char_position][1]] = br;  //b
              pixelvalues[seg_lookup[char_position][2]] = 0;  //c
              pixelvalues[seg_lookup[char_position][3]] = 0;  //d
              pixelvalues[seg_lookup[char_position][4]] = 0;  //e
              pixelvalues[seg_lookup[char_position][5]] = 0;  //f
              pixelvalues[seg_lookup[char_position][6]] = br;  //g
             break;
    
            case '-':
              pixelvalues[seg_lookup[char_position][0]] = 0;  //a
              pixelvalues[seg_lookup[char_position][1]] = 0;  //b
              pixelvalues[seg_lookup[char_position][2]] = 0;  //c
              pixelvalues[seg_lookup[char_position][3]] = 0;  //d
              pixelvalues[seg_lookup[char_position][4]] = 0;  //e
              pixelvalues[seg_lookup[char_position][5]] = 0;  //f
              pixelvalues[seg_lookup[char_position][6]] = br;  //g
             break;
    
           case 'a':  //pretty arbitrary.  This switch case could be extended to show lots of combinations other than numbers
              pixelvalues[seg_lookup[char_position][0]] = br;  //a
              pixelvalues[seg_lookup[char_position][1]] = br;  //b
              pixelvalues[seg_lookup[char_position][2]] = br;  //c
              pixelvalues[seg_lookup[char_position][3]] = br;  //d
              pixelvalues[seg_lookup[char_position][4]] = br;  //e
              pixelvalues[seg_lookup[char_position][5]] = br;  //f
              pixelvalues[seg_lookup[char_position][6]] = br;  //g
             break;
          }
     }
      
}
