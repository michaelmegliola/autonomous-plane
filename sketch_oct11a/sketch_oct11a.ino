

/////////////////////////////////////////////////////////////////////////
//
//
//   Real time clock with better than 0.5 µs accuracy
//
//
//
/////////////////////////////////////////////////////////////////////////

volatile uint32_t realTcount = 0x0 ;   // Counter for superticks (overflow interrupts)


void setup() {
Serial.begin(9600);
 
 setclockTC3();
 setCounterTC3();


}


/*
This is test code that counts to 10 s then resets and waits 3s to test 
reset, start and stop functions on TC3. 
*/
void loop() {
  
  delay(250);
  
    Serial.print(realTime(), 7);
    Serial.println();
    double val = realTime();
   
    if( int(val) >= 10)
    {
  resetStartTC3();  // reset so never gets above 10 s
      stopTC3();
    delay(3000); //wait 3 sec
    startTC3();
    }
    
}

void TC3_Handler()  // Interrupt on overflow
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
    realTcount++;                    // Increment the supertick register
    TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
//  }
}

/*
Get the real time in seconds.
*/
double realTime() 
{   
double  realTime =  (realTcount * 1.2288E-2) + (REG_TC3_COUNT16_COUNT * 1.875E-7) ;;
return realTime;
}


/*
  Setup the Generic clock register
*/

void setclockTC3()
{
  // Setup and enable clock for TC
    REG_GCLK_GENDIV = GCLK_GENDIV_DIV(9) |    // Divide the 48MHz system clock by 9 = 5.33333MHz
                    GCLK_GENDIV_ID(5);      // Set division on Generic Clock Generator (GCLK) 5
  while (GCLK->STATUS.bit.SYNCBUSY);        // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK 5
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the clock source to 48MHz
                     GCLK_GENCTRL_ID(5);          // Set clock source on GCLK 5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable the generic clock...
                     GCLK_CLKCTRL_GEN_GCLK5 |     // ....on GCLK5
                     // Hack to overcome define problem in next line!!!
                     0x1B;                        // Feed the GCLK5 to TCC2 and TC3
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
}

void setCounterTC3()
{
  // The type cast must fit with the selected timer mode
  TcCount16* TC = (TcCount16*) TC3; // get timer struct

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_NFRQ; // Set TC as normal Normal Frq
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set perscaler
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
 
  // Interrupts
  TC->INTENSET.reg = 0;              // disable all interrupts
  TC->INTENSET.bit.OVF = 1;          // enable overfollow interrup


  // Enable InterruptVector
  NVIC_EnableIRQ(TC3_IRQn);

  // Enable TC
  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

}



void resetStartTC3()
{
   TcCount16* TC = (TcCount16*) TC3; // get timer struct
   realTcount = 0x00;                // Zero superclicks
    TC->CTRLBSET.reg |= TC_CTRLBCLR_CMD_RETRIGGER;   // restart
}

void stopTC3()
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
    TC->CTRLBSET.reg |= TC_CTRLBSET_CMD_STOP;   // Stop counter
}


void startTC3()
{
  TcCount16* TC = (TcCount16*) TC3; // get timer struct
    TC->CTRLBSET.reg |= TC_CTRLBSET_CMD_RETRIGGER;   //  Start
} 
