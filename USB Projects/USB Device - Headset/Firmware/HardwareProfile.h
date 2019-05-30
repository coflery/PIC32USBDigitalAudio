/********************************************************************
 FileName:      HardwareProfile - PIC32MX460F512L PIM.h
 Dependencies:  See INCLUDES section
 Processor:     PIC32 USB Microcontrollers
 Hardware:      PIC32MX460F512L PIM
 Compiler:      Microchip C32 (for PIC32)
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                    coding style
  2.3   09/15/2008   Broke out each hardware platform into its own
                 "HardwareProfile - xxx.h" file
********************************************************************/


    /*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    //#define USE_SELF_POWER_SENSE_IO
    #define tris_self_power     TRISAbits.TRISA2    // Input
    #define self_power          1

    //#define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense  TRISBbits.TRISB5    // Input
    #define USB_BUS_SENSE       1

    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    /** Board definition ***********************************************/
    //These defintions will tell the main() function which board is
    //  currently selected.  This will allow the application to add
    //  the correct configuration bits as wells use the correct
    //  initialization functions for the board.  These defitions are only
    //  required in the stack provided demos.  They are not required in
    //  final application design.
    
    #define GetSystemClock() 		(40000000ul)
	#define GetPeripheralClock()    (GetSystemClock())//(GetSystemClock() / (1 << OSCCONbits.PBDIV))
	#define GetInstructionClock()   (GetSystemClock())
	
    #define CLOCK_FREQ 40000000
    /** LED ************************************************************/
    #define mInitAllLEDs()      LATA &= 0xFFC3; TRISBbits.TRISB0=0;TRISBbits.TRISB13=0;TRISBbits.TRISB15=0;TRISA &= 0xFFC3;

    #define mLED_1              LATBbits.LATB0
    #define mLED_2              LATBbits.LATB15
    #define mLED_3              LATBbits.LATB13
    #define mLED_4              LATBbits.LATB15

    #define mGetLED_1()         mLED_1
    #define mGetLED_2()         mLED_2
    #define mGetLED_3()         mLED_3
    #define mGetLED_4()         mLED_4

    #define mLED_1_On()         mLED_1 = 1;
    #define mLED_2_On()         mLED_2 = 1;
    #define mLED_3_On()         mLED_3 = 1;
    #define mLED_4_On()         mLED_4 = 1;

    #define mLED_1_Off()        mLED_1 = 0;
    #define mLED_2_Off()        mLED_2 = 0;
    #define mLED_3_Off()        mLED_3 = 0;
    #define mLED_4_Off()        mLED_4 = 0;

    #define mLED_1_Toggle()     mLED_1 = !mLED_1;
    #define mLED_2_Toggle()     mLED_2 = !mLED_2;
    #define mLED_3_Toggle()     mLED_3 = !mLED_3;
    #define mLED_4_Toggle()     mLED_4 = !mLED_4;

    /** SWITCH *********************************************************/
    #define mInitSwitch2()      TRISBbits.TRISB3=1;
    #define mInitSwitch3()      TRISBbits.TRISB7=1;
    #define mInitAllSwitches()  {CNPUBSET=0x8C; CNCONBbits.ON=1; mInitSwitch2();mInitSwitch3();}
    #define sw2                 PORTBbits.RB3
    #define sw3                 PORTBbits.RB7

    /** I/O pin definitions ********************************************/
    #define INPUT_PIN 1
    #define OUTPUT_PIN 0

    /*** Audio    ******************************************************/
    #ifdef AUDIO_SAMPLING_FREQUENCY_48000
        #define NO_OF_SAMPLES_IN_A_USB_FRAME 48
        #define PWM_PERIOD	(CLOCK_FREQ/48000)-1
    #elif defined AUDIO_SAMPLING_FREQUENCY_32000
        #define NO_OF_SAMPLES_IN_A_USB_FRAME 32
        #define PWM_PERIOD	(CLOCK_FREQ/32000)-1
    #elif defined AUDIO_SAMPLING_FREQUENCY_44100
        #define NO_OF_SAMPLES_IN_A_USB_FRAME 44
        #define PWM_PERIOD	(CLOCK_FREQ/44100)-1
    #endif

    /*** Intialize Audio Driver on the Speech Playback Card ************/
    #define mInitAudioDriver()  {TRISDCLR = 0x00000100; LATDSET = 0x00000100; }
    #define mAudioDriverON()	LATDCLR = 0x00000100;
    #define mAudioDriverOFF()	LATDSET = 0x00000100;

    /****** PWM Intialization*******************************************/
    // configure RD0 as output for PWM
    // PWM mode, Single output, Active High
    #define mInitPWM() 			 {  TRISDCLR = 0x00000001;\
    								OC1R 	 = 0;\
    								OC1CON   = 0x0000;\
    								OC1RS    = 0;\
    								OC1CON   = 0x0006;\
    								OC1CONSET = 0x8000;\
    							}

    #define DUTY_CYCLE OC1RS  // Duty Cycle register of the PWM Peripheral.
    #define MULTIPLY_FACTOR 4 // The Audio received from the USB host is 8 bits/Sample.
                              // However PIC32 has 32 bit Duty Cycle Register.
                              // Multiplying by 4 increases the signal strength.

    /****** Timer2 Intialization************************************/
    #define mInitTimerInterrupt() { INTEnableSystemMultiVectoredInt();\
    								INTEnableInterrupts;\
    								  }
    #define mInitTimer() 		  { OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1,\
                                    PWM_PERIOD);\
    								ConfigIntTimer2(T2_INT_OFF | T2_INT_PRIOR_7);\
    							  }

    /****** Unmask Timer2 Interrupt ************************************/
    #define mStartAudio()  {EnableIntT2;} // unmask timer 2 interrupt

    /******* Mask Timer2 Interrupt  ***********************************/
    #define mStopAudio()   {DisableIntT2;}// mask timer 2 interrupt







