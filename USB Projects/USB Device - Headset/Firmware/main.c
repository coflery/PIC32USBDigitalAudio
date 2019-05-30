/********************************************************************
 FileName:      main.c
 Dependencies:  See INCLUDES section
 Processor:		PIC32 USB Microcontrollers
 Hardware:		This demo is natively intended to be used on Microchip USB demo
 				boards supported by the MCHPFSUSB stack.  See release notes for
 				support matrix.  This demo can be modified for use on other hardware
 				platforms.
 Complier:  	Microchip C32 (for PIC32)
 Company:		Microchip Technology, Inc.

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
********************************************************************/

#ifndef MAIN_C
#define MAIN_C

/** INCLUDES *******************************************************/
#include "./USB/usb.h"
#include "USB/usb_function_audio.h"
#include "HardwareProfile.h"
#include "ak4645a.h"
#include "pps.h"
#include "Tick.h"

 /** CONFIGURATION **************************************************/ 
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_3, FPLLODIV = DIV_2
#pragma config POSCMOD = EC, FNOSC = PRIPLL, FPBDIV = DIV_1, FSOSCEN = OFF
#pragma config UPLLEN = ON, UPLLIDIV = DIV_3, FVBUSONIO = OFF, FUSBIDIO = OFF
#pragma config FWDTEN = OFF, JTAGEN = OFF    
#pragma config IESO = OFF, IOL1WAY = ON, PMDL1WAY = ON, OSCIOFNC = OFF       
#pragma config ICESEL = ICS_PGx1  

typedef struct
{
    INT16 left;
    INT16 right;
}
AUDIO_PLAY_SAMPLE;

#define AUDIO_MAX_FREQ              48000
#define AUDIO_MAX_SAMPLES           ( AUDIO_MAX_FREQ / 1000 )



/** VARIABLES ******************************************************/
AK4645AState* pCodecHandle=NULL;

TICK			timeLastms;
TICK                    timeLast10ms;
TICK                    timeLast100ms;
AUDIO_PLAY_SAMPLE ReceivedDataEvenBuffer[NO_OF_SAMPLES_IN_A_USB_FRAME];// Holds the Even Audio Data received from Host
AUDIO_PLAY_SAMPLE ReceivedDataOddBuffer[NO_OF_SAMPLES_IN_A_USB_FRAME]; // Holds the Odd  Audio Data received from Host

AUDIO_PLAY_SAMPLE TransmittedDataEvenBuffer[NO_OF_SAMPLES_IN_A_USB_FRAME];// Holds the Even Audio Data received from Host
AUDIO_PLAY_SAMPLE TransmittedDataOddBuffer[NO_OF_SAMPLES_IN_A_USB_FRAME]; // Holds the Odd  Audio Data received from Host

USB_HANDLE USBTxEvenHandle = 0;
USB_HANDLE USBTxOddHandle = 0;
USB_HANDLE USBRxEvenHandle = 0;
USB_HANDLE USBRxOddHandle = 0;
BOOL receivedDataEvenNeedsServicingNext = FALSE;	//TRUE means even need servicing next, FALSE means odd needs servicing next
BOOL transmittedDataEvenNeedsServicingNext = FALSE;	//TRUE means even need servicing next, FALSE means odd needs servicing next
UINT8 audioMute = 1; // Holds the status of Mute Control 
BOOL DACMute = TRUE; 

/** PROTOTYPES *********************************************/
void BlinkUSBStatus(void);
static void InitializeSystem(void);
void USBAudioTasks(void);
void UserInit(void);
void USBCBSendResume(void);
void CheckButtons(void);
void VolumeControlInit(void);
void VolumeControlTask(void);

/** DECLARATIONS ***************************************************/
#pragma code
UINT16 volumePot=0;
UINT16 prevvolumePot=-1;
BOOL input_enable=FALSE;



/** VARIABLES ******************************************************/
USB_HANDLE lastTransmission;
USB_HANDLE lastReception=0;


/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
int main(void)
{   
    InitializeSystem();
    USBDeviceAttach();
    		
    while(1)
    {		
        USBAudioTasks(); 
	CheckButtons();

	if ((TickGet() - timeLast10ms) > (300 * dwTicksPerMillisecond))
        {
            timeLast10ms = TickGet();
            VolumeControlTask ();
        }
    }
    
}


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{

	SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);	
    INTEnableSystemMultiVectoredInt();
        
//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//	to it in HardwareProfile.h.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif


    
    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
    					
    TickInit();
    
    //initialize the variable holding the handle for the last
    // transmission
    lastTransmission = 0;
    
    UserInit();   
}//end InitializeSystem



/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
    //Initialize all of the LED pins
    mInitAllLEDs();

    //Initialize all of the push buttons
    mInitAllSwitches();

	mInitTimerInterrupt();
	
	mInitTimer();

    //initialize the variable holding the handle for the last
    // transmission
    USBRxEvenHandle = NULL;
	USBRxOddHandle = NULL;
	USBTxEvenHandle = NULL;
	USBTxOddHandle = NULL;
	
	
       
	////////////////////////////////Init CODEC	

	ANSELA = ANSELB = 0;
	TRISBbits.TRISB0=0;
	TRISBbits.TRISB13=0;
	TRISBbits.TRISB15=0;
	LATBbits.LATB0=1;
	LATBbits.LATB13=1;
	LATBbits.LATB15=1;
	TRISAbits.TRISA3 = 0;	
		
	TRISAbits.TRISA0 = 0;
	TRISAbits.TRISA4 = 0;
	TRISAbits.TRISA1 = 1;	
	TRISBbits.TRISB5 = 0;
	TRISBbits.TRISB8 = 0;
	TRISBbits.TRISB9 = 0;
	LATAbits.LATA3 = 0;
	
	PPSOutput(3, RPA4, REFCLKO);	//REFCLK0: RPA4 - Out
	PPSInput(2, SDI1, RPA1);		//SDI: RPA1 - In
	PPSOutput(2, RPB5, SDO1);		//SDO: RPB5 - Out
	PPSInput(1, SS1, RPB4);			//SS: RPB4 - In

	// Initialize reference clock out module
	REFOCONbits.ROSEL = 6;
	REFOCONbits.RODIV = 4;
	REFOCONbits.OE = 1;
	REFOCONbits.ON = 1;
	
	LATAbits.LATA3 = 1;
		
	pCodecHandle=AK4645AOpen(O_RDWR);
	if(pCodecHandle==NULL)
		while(1);
	if(AK4645ASetSampleRate(pCodecHandle, SAMPLERATE_48000HZ)!=1)
		while(1);
	if(AK4645ASetADCDACOptions(pCodecHandle, TRUE)!=1)
		while(1);
	AK4645AStartAudio(pCodecHandle, TRUE);
	if(AK4645ASetDACVolume(pCodecHandle, 90)!=1)
		while(1);	
		
	VolumeControlInit();
}//end UserInit

/********************************************************************
 * Function:        void UsbAudioInputTerminalControlRequestsHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    If either the USBEP0SendRAMPtr() or USBEP0Receive()
 *                  functions are not called in this function then the 
 *                  requesting the Input Terminal request will be STALLed
 *
 * Overview:        This function is called by the Audio function driver
 *                  in response to a Input Terminal Request. 
 *
 * Note:            This function is called from the stack in
 *                  response of a EP0 packet.  The response to this
 *                  packet should be fast in order to clear EP0 for
 *                  any potential SETUP packets.  Do not call any
 *                  functions or run any algorithms that take a long time
 *                  to execute (>50uSec).  Have any data that would be
 *                  read using one of these commands pre-calculated
 *                  from the main line code and just use this function
 *                  to transfer the data.
 
 *                  Input Terminal Control request fields
 *                  SetupPkt.bRequest holds bRequest(request attribute) 
 *                  SetupPkt.W_Value holds CS(Control Selector)
 *                  SetupPkt.W_Index.byte.HB holds the Input Terminal ID
 *                  SetupPkt.W_Index.byte.LB holds the Interface number
 *                  SetupPkt.wLength holds length of the parameter block 
 *******************************************************************/
 #if defined USB_AUDIO_INPUT_TERMINAL_CONTROL_REQUESTS_HANDLER
 void UsbAudioInputTerminalControlRequestsHandler(void)
 {
     
     
 }    
 #endif 
/********************************************************************
 * Function:        void UsbAudioOutputTerminalControlRequestsHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    If either the USBEP0SendRAMPtr() or USBEP0Receive()
 *                  functions are not called in this function then the 
 *                  requesting the Output Terminal request will be STALLed
 *
 * Overview:        This function is called by the Audio function driver
 *                  in response to a Output Terminal Request. 
 *
 * Note:            This function is called from the stack in
 *                  response of a EP0 packet.  The response to this
 *                  packet should be fast in order to clear EP0 for
 *                  any potential SETUP packets.  Do not call any
 *                  functions or run any algorithms that take a long time
 *                  to execute (>50uSec).  Have any data that would be
 *                  read using one of these commands pre-calculated
 *                  from the main line code and just use this function
 *                  to transfer the data.
 
 *                  Output Terminal Control request fields
 *                  SetupPkt.bRequest holds bRequest(request attribute) 
 *                  SetupPkt.W_Value holds CS(Control Selector)
 *                  SetupPkt.W_Index.byte.HB holds the Output Terminal ID
 *                  SetupPkt.W_Index.byte.LB holds the interface number
 *                  SetupPkt.wLength holds length of the parameter block 
 *******************************************************************/
 #if defined USB_AUDIO_OUTPUT_TERMINAL_CONTROL_REQUESTS_HANDLER
 void UsbAudioOutputTerminalControlRequestsHandler(void)
 {
	 
     switch (SetupPkt.W_Value.byte.HB)
     {
        case MUTE_CONTROL:
            if (SetupPkt.bRequest == SET_CUR)
            {
                USBEP0Receive((BYTE*)&DACMute,SetupPkt.wLength,NULL);              
				AK4645ADACMute(pCodecHandle, !DACMute); 
            }
            else if (SetupPkt.bRequest == GET_CUR)
            {
                // Get Mute Control Status
                CtrlTrfData[0] = audioMute;
    			USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 1, USB_EP0_NO_OPTIONS);
            }
            break;     
        case VOLUME_CONTROL:
            switch(SetupPkt.bRequest)
            {
                case SET_CUR:
                    //Set Current Volume
                    break;
                case SET_MIN:
                    break;
                case SET_MAX:
                    break;
                case SET_RES:
                    break;
                case GET_CUR:
                    // up on this request user needs to send the current volume to the host. 
			        CtrlTrfData[0] = 0x43;
			        CtrlTrfData[1] = 0x00;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS);
                    break;
                case GET_MIN:
			        CtrlTrfData[0] = 0;
			        CtrlTrfData[1] = 0;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS); 
                    break;
                case GET_MAX:
			        CtrlTrfData[0] = 0xFF;
			        CtrlTrfData[1] = 0xFF;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS); 
                    break;
                case GET_RES:
			        CtrlTrfData[0] = 0xFF;
			        CtrlTrfData[1] = 0xFF;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS); 
                    break;
                default:
                    break;
                }// end of switch(SetupPkt.bRequest)	    
            break;
        case BASS_CONTROL:
             break;
        case MID_CONTROL:
             break;
        case TREBLE_CONTROL:
             break;
        case GRAPHIC_EQUALIZER_CONTROL:
             break;
        case AUTOMATIC_GAIN_CONTROL:
             break;
        case DELAY_CONTROL:
             break;
        case BASS_BOOST_CONTROL:
             break;
        case LOUDNESS_CONTROL:
             break;
        default:
             break;     
       }  
  

 }    
 #endif 
  /********************************************************************
 * Function:        void UsbAudioMixerUnitControlRequestsHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    If either the USBEP0SendRAMPtr() or USBEP0Receive()
 *                  functions are not called in this function then the 
 *                  requesting the Mixer unit request will be STALLed
 *
 * Overview:        This function is called by the Audio function driver
 *                  in response to a Mixer unit Request. 
 *
 * Note:            This function is called from the stack in
 *                  response of a EP0 packet.  The response to this
 *                  packet should be fast in order to clear EP0 for
 *                  any potential SETUP packets.  Do not call any
 *                  functions or run any algorithms that take a long time
 *                  to execute (>50uSec).  Have any data that would be
 *                  read using one of these commands pre-calculated
 *                  from the main line code and just use this function
 *                  to transfer the data.
 
 *                  Mixer Unit Control request fields
 *                  SetupPkt.bRequest holds bRequest(request attribute) 
 *                  SetupPkt.W_Value.byte.HB holds the ICN (Input Channel Number)
 *                  SetupPkt.W_Value.byte.LB holds the OCN (Output Channel Number)
 *                  SetupPkt.W_Index.byte.HB holds the Mixer Unit ID
 *                  SetupPkt.W_Index.byte.LB holds the interface number
 *                  SetupPkt.wLength holds length of the parameter block 
 *******************************************************************/
 #if defined USB_AUDIO_MIXER_UNIT_CONTROL_REQUESTS_HANDLER
 void UsbAudioMixerUnitControlRequestsHandler(void)
 {
     
     
 }    
 #endif 
  /********************************************************************
 * Function:        void UsbAudioSelectorUnitControlRequestsHabdler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    If either the USBEP0SendRAMPtr() or USBEP0Receive()
 *                  functions are not called in this function then the 
 *                  requesting the Selector unit request will be STALLed
 *
 * Overview:        This function is called by the Audio function driver
 *                  in response to a Selector unit Request. 
 *
 * Note:            This function is called from the stack in
 *                  response of a EP0 packet.  The response to this
 *                  packet should be fast in order to clear EP0 for
 *                  any potential SETUP packets.  Do not call any
 *                  functions or run any algorithms that take a long time
 *                  to execute (>50uSec).  Have any data that would be
 *                  read using one of these commands pre-calculated
 *                  from the main line code and just use this function
 *                  to transfer the data.
 
 *                  Selector Unit Control request fields
 *                  SetupPkt.bRequest holds bRequest(request attribute) 
 *                  SetupPkt.W_Index.byte.HB holds the Selector Unit ID
 *                  SetupPkt.W_Index.byte.LB holds the interface number
 *                  SetupPkt.wLength holds length of the parameter block 
 *******************************************************************/
 #if defined USB_AUDIO_SELECTOR_UNIT_CONTROL_REQUESTS_HANDLER
 void UsbAudioSelectorUnitControlRequestsHabdler(void)
 {
     
     
 }    
 #endif 


/********************************************************************
 * Function:        void UsbAudioFeatureUnitControlRequestsHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    If either the USBEP0SendRAMPtr() or USBEP0Receive()
 *                  functions are not called in this function then the 
 *                  requesting the feature unit request will be STALLed
 *
 * Overview:        This function is called by the Audio function driver
 *                  in response to a Feature unit Request. 
 *
 * Note:            This function is called from the stack in
 *                  response of a EP0 packet.  The response to this
 *                  packet should be fast in order to clear EP0 for
 *                  any potential SETUP packets.  Do not call any
 *                  functions or run any algorithms that take a long time
 *                  to execute (>50uSec).  Have any data that would be
 *                  read using one of these commands pre-calculated
 *                  from the main line code and just use this function
 *                  to transfer the data. 
 
 *                  Feature Unit Control request fields
 *                  SetupPkt.bRequest holds bRequest(request attribute) 
 *                  SetupPkt.W_Value.byte.HB holds the CS (Control Selector)
 *                  SetupPkt.W_Value.byte.LB holds the CN (Channel Number)
 *                  SetupPkt.W_Index.byte.HB holds the Feature Unit ID
 *                  SetupPkt.W_Index.byte.LB holds the Interface number
 *                  SetupPkt.wLength holds length of the parameter block                                        
 *******************************************************************/
 #if defined USB_AUDIO_FEATURE_UNIT_CONTROL_REQUESTS_HANDLER
 void UsbAudioFeatureUnitControlRequestsHandler(void)
 {
 
     switch (SetupPkt.W_Value.byte.HB)
     {
        case MUTE_CONTROL:
            if (SetupPkt.bRequest == SET_CUR)
            {
                USBEP0Receive((BYTE*)&DACMute,SetupPkt.wLength,NULL);               
				AK4645ADACMute(pCodecHandle, !DACMute);             
            }
            else if (SetupPkt.bRequest == GET_CUR)
            {
                // Get Mute Control Status
                CtrlTrfData[0] = audioMute;
    			USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 1, USB_EP0_NO_OPTIONS);
            }
            break;     
        case VOLUME_CONTROL:
            switch(SetupPkt.bRequest)
            {
                case SET_CUR:
                    //Set Current Volume
                    break;
                case SET_MIN:
                    break;
                case SET_MAX:
                    break;
                case SET_RES:
                    break;
                case GET_CUR:
                    // up on this request user needs to send the current volume to the host. 
			        CtrlTrfData[0] = 0x43;
			        CtrlTrfData[1] = 0x00;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS);
                    break;
                case GET_MIN:
			        CtrlTrfData[0] = 0;
			        CtrlTrfData[1] = 0;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS); 
                    break;
                case GET_MAX:
			        CtrlTrfData[0] = 0xFF;
			        CtrlTrfData[1] = 0xFF;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS); 
                    break;
                case GET_RES:
			        CtrlTrfData[0] = 0xFF;
			        CtrlTrfData[1] = 0xFF;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS); 
                    break;
                default:
                    break;
                }// end of switch(SetupPkt.bRequest)	    
            break;
        case BASS_CONTROL:
             break;
        case MID_CONTROL:
             break;
        case TREBLE_CONTROL:
             break;
        case GRAPHIC_EQUALIZER_CONTROL:
             break;
        case AUTOMATIC_GAIN_CONTROL:
            switch(SetupPkt.bRequest)
            {
                case GET_CUR:
                    // up on this request user needs to send the current volume to the host. 
			        CtrlTrfData[0] = 0x43;
			        CtrlTrfData[1] = 0x00;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS);
                    break;
                default:
                    break;
            }// end of switch(SetupPkt.bRequest)	    

             break;
        case DELAY_CONTROL:
             break;
        case BASS_BOOST_CONTROL:
             break;
        case LOUDNESS_CONTROL:
             break;
        default:
             break;     
       }  //end of switch (SetupPkt.W_Value.byte.HB)  
 
 }//end of void UsbFeatureUnitHandler(void)
 #endif 
 
 /********************************************************************
 * Function:        void UsbAudioProcessingUnitControlRequestsHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    If either the USBEP0SendRAMPtr() or USBEP0Receive()
 *                  functions are not called in this function then the 
 *                  requesting the Processing unit request will be STALLed
 *
 * Overview:        This function is called by the Audio function driver
 *                  in response to a Processing unit Request. 
 *
 * Note:            This function is called from the stack in
 *                  response of a EP0 packet.  The response to this
 *                  packet should be fast in order to clear EP0 for
 *                  any potential SETUP packets.  Do not call any
 *                  functions or run any algorithms that take a long time
 *                  to execute (>50uSec).  Have any data that would be
 *                  read using one of these commands pre-calculated
 *                  from the main line code and just use this function
 *                  to transfer the data.
 
 *                  Processing Unit Control request fields
 *                  SetupPkt.bRequest holds bRequest(request attribute) 
 *                  SetupPkt.W_Value holds CS(Control Selector)
 *                  SetupPkt.W_Index.byte.HB holds the Processing Unit ID
 *                  SetupPkt.W_Index.byte.LB holds the interface number
 *                  SetupPkt.wLength holds length of the parameter block 
 *******************************************************************/
 #if defined USB_AUDIO_PROCESSING_UNIT_CONTROL_REQUESTS_HANDLER
 void UsbAudioProcessingUnitControlRequestsHandler(void)
 {
 }
 #endif 
 /********************************************************************
 * Function:        void UsbAudioExtensionUnitControlRequestsHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    If either the USBEP0SendRAMPtr() or USBEP0Receive()
 *                  functions are not called in this function then the 
 *                  requesting the Extension unit request will be STALLed
 *
 * Overview:        This function is called by the Audio function driver
 *                  in response to a Extension unit Request. 
 *
 * Note:            This function is called from the stack in
 *                  response of a EP0 packet.  The response to this
 *                  packet should be fast in order to clear EP0 for
 *                  any potential SETUP packets.  Do not call any
 *                  functions or run any algorithms that take a long time
 *                  to execute (>50uSec).  Have any data that would be
 *                  read using one of these commands pre-calculated
 *                  from the main line code and just use this function
 *                  to transfer the data.
 
 *                  Extension Unit Control request fields
 *                  SetupPkt.bRequest holds bRequest(request attribute) 
 *                  SetupPkt.W_Value holds CS(Control Selector)
 *                  SetupPkt.W_Index.byte.HB holds the Extension Unit ID
 *                  SetupPkt.W_Index.byte.LB holds the interface number
 *                  SetupPkt.wLength holds length of the parameter block 
 *******************************************************************/
 #if defined USB_AUDIO_EXTENSION_UNIT_CONTROL_REQUESTS_HANDLER
 void UsbAudioExtensionUnitControlRequestsHandler(void)
 {
	 switch (SetupPkt.W_Value.byte.HB)
     {
        case MUTE_CONTROL:
            if (SetupPkt.bRequest == SET_CUR)
            {
                USBEP0Receive((BYTE*)&DACMute/*audioMute*/,SetupPkt.wLength,NULL);  
             
				AK4645ADACMute(pCodecHandle, !DACMute);
             
            }
            else if (SetupPkt.bRequest == GET_CUR)
            {
                // Get Mute Control Status
                CtrlTrfData[0] = audioMute;
    			//CtrlTrfData[1] = 0x00;
    			USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 1, USB_EP0_NO_OPTIONS);
            }
            break;     
        case VOLUME_CONTROL:
            switch(SetupPkt.bRequest)
            {
                case SET_CUR:
                    //Set Current Volume
                    break;
                case SET_MIN:
                    break;
                case SET_MAX:
                    break;
                case SET_RES:
                    break;
                case GET_CUR:
                    // up on this request user needs to send the current volume to the host. 
			        CtrlTrfData[0] = 0x43;
			        CtrlTrfData[1] = 0x00;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS);
                    break;
                case GET_MIN:
			        CtrlTrfData[0] = 0;
			        CtrlTrfData[1] = 0;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS); 
                    break;
                case GET_MAX:
			        CtrlTrfData[0] = 0xFF;
			        CtrlTrfData[1] = 0xFF;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS); 
                    break;
                case GET_RES:
			        CtrlTrfData[0] = 0xFF;
			        CtrlTrfData[1] = 0xFF;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS); 
                    break;
                default:
                    break;
                }// end of switch(SetupPkt.bRequest)	    
            break;
        case BASS_CONTROL:
             break;
        case MID_CONTROL:
             break;
        case TREBLE_CONTROL:
             break;
        case GRAPHIC_EQUALIZER_CONTROL:
             break;
        case AUTOMATIC_GAIN_CONTROL:
        Nop();
            switch(SetupPkt.bRequest)
            {
                case GET_CUR:
                    // up on this request user needs to send the current volume to the host. 
			        CtrlTrfData[0] = 0x43;
			        CtrlTrfData[1] = 0x00;
			        USBEP0SendRAMPtr((BYTE*)CtrlTrfData, 2, USB_EP0_NO_OPTIONS);
                    break;
                default:
                    break;
            }// end of switch(SetupPkt.bRequest)	    

             break;
        case DELAY_CONTROL:
             break;
        case BASS_BOOST_CONTROL:
             break;
        case LOUDNESS_CONTROL:
             break;
        default:
             break;     
       }  //end of switch (SetupPkt.W_Value.byte.HB)  
 
 }
 #endif 
 /********************************************************************
 * Function:        void UsbAudioInterfaceControlRequestsHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    If either the USBEP0SendRAMPtr() or USBEP0Receive()
 *                  functions are not called in this function then the 
 *                  requesting the feature unit request will be STALLed
 *
 * Overview:        This function is called by the Audio function driver
 *                  in response to a Feature unit Request. 
 *
 * Note:            This function is called from the stack in
 *                  response of a EP0 packet.  The response to this
 *                  packet should be fast in order to clear EP0 for
 *                  any potential SETUP packets.  Do not call any
 *                  functions or run any algorithms that take a long time
 *                  to execute (>50uSec).  Have any data that would be
 *                  read using one of these commands pre-calculated
 *                  from the main line code and just use this function
 *                  to transfer the data.
 *******************************************************************/
 #if defined USB_AUDIO_INTRFACE_CONTROL_REQUESTS_HANDLER
 void UsbAudioInterfaceControlRequestsHandler(void)
 {
 }
 #endif       
 /********************************************************************
 * Function:        void UsbAudioEndpointControlRequestsHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    If either the USBEP0SendRAMPtr() or USBEP0Receive()
 *                  functions are not called in this function then the 
 *                  requesting the feature unit request will be STALLed
 *
 * Overview:        This function is called by the Audio function driver
 *                  in response to a Feature unit Request. 
 *
 * Note:            This function is called from the stack in
 *                  response of a EP0 packet.  The response to this
 *                  packet should be fast in order to clear EP0 for
 *                  any potential SETUP packets.  Do not call any
 *                  functions or run any algorithms that take a long time
 *                  to execute (>50uSec).  Have any data that would be
 *                  read using one of these commands pre-calculated
 *                  from the main line code and just use this function
 *                  to transfer the data.
 
 *                  Endpoint Control request fields
 *                  SetupPkt.bRequest holds bRequest(request attribute) 
 *                  SetupPkt.W_Value holds CS(Control Selector)
 *                  SetupPkt.W_Index holds endpoint
 *                  SetupPkt.wLength holds length of the parameter block
 *******************************************************************/
 void UsbSampleRateControl(void){
	switch(pCodecHandle->samplingFreq){
	    case 48000:
			if(AK4645ASetSampleRate(pCodecHandle, SAMPLERATE_48000HZ)!=1)
				while(1);
			break;  
	    case 44100:
			if(AK4645ASetSampleRate(pCodecHandle, SAMPLERATE_44100HZ)!=1)
				while(1);
			break;    
	    case 32000:
			if(AK4645ASetSampleRate(pCodecHandle, SAMPLERATE_32000HZ)!=1)
				while(1);
			break;    
//	    case 24000:
//			if(AK4645ASetSampleRate(pCodecHandle, SAMPLERATE_24000HZ)!=1)
//				while(1);
//			break;        
//	    case 16000:
//			if(AK4645ASetSampleRate(pCodecHandle, SAMPLERATE_16000HZ)!=1)
//				while(1);
//			break;    
//	    case 8000:
//			if(AK4645ASetSampleRate(pCodecHandle, SAMPLERATE_8000HZ)!=1)
//				while(1);
//			break;				
        default:
             break; 				
	}	
}
 
 #if defined USB_AUDIO_ENDPOINT_CONTROL_REQUESTS_HANDLER
 void UsbAudioEndpointControlRequestsHandler(void)
 {
	 
	 switch (SetupPkt.W_Value.byte.HB)
     {
	     case EP_CONTROL_UNDEFINED:
	     	break;
	     case SAMPLING_FREQ_CONTROL:
            switch(SetupPkt.bRequest)
            {
                case SET_CUR:
	                USBEP0Receive ((BYTE*)&(pCodecHandle->samplingFreq),3,UsbSampleRateControl);                    
                    break;                   
                case SET_MIN:
                    break;
                case SET_MAX:
                    break;
                case SET_RES:
                    break;
                case GET_CUR:
                    break;
                case GET_MIN:
                    break;
                case GET_MAX:
                    break;
                case GET_RES:
                    break;
                default:
                    break;                    
            }// end of switch(SetupPkt.bRequest)	                        
            break;
         case PITCH_CONTROL:
         	break;
         default:
             break;     
     }
     Nop();
     
 }
 #endif 
 /********************************************************************
 * Function:        void UsbAudioMemoryRequestsHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    If either the USBEP0SendRAMPtr() or USBEP0Receive()
 *                  functions are not called in this function then the 
 *                  requesting the feature unit request will be STALLed
 *
 * Overview:        This function is called by the Audio function driver
 *                  in response to a Feature unit Request. 
 *
 * Note:            This function is called from the stack in
 *                  response of a EP0 packet.  The response to this
 *                  packet should be fast in order to clear EP0 for
 *                  any potential SETUP packets.  Do not call any
 *                  functions or run any algorithms that take a long time
 *                  to execute (>50uSec).  Have any data that would be
 *                  read using one of these commands pre-calculated
 *                  from the main line code and just use this function
 *                  to transfer the data.
 
 *                  Memory Requests fields
 *                  SetupPkt.bRequest holds bRequest(request attribute) 
 *                  SetupPkt.W_Value holds offset
 *                  SetupPkt.W_Index holds endpoint
 *                  SetupPkt.W_Index.byte.HB holds the Entity ID
 *                  SetupPkt.W_Index.byte.LB holds the interface number
 *                  SetupPkt.wLength holds length of the parameter block               
 *******************************************************************/
 #if defined USB_AUDIO_MEMORY_REQUESTS_HANDLER
 void UsbAudioMemoryRequestsHandler(void)
 {
 }
 #endif   
/********************************************************************
 * Function:        void UsbAudioStatusRequestsHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    If either the USBEP0SendRAMPtr() or USBEP0Receive()
 *                  functions are not called in this function then the 
 *                  requesting the feature unit request will be STALLed
 *
 * Overview:        This function is called by the Audio function driver
 *                  in response to a Feature unit Request. 
 *
 * Note:            This function is called from the stack in
 *                  response of a EP0 packet.  The response to this
 *                  packet should be fast in order to clear EP0 for
 *                  any potential SETUP packets.  Do not call any
 *                  functions or run any algorithms that take a long time
 *                  to execute (>50uSec).  Have any data that would be
 *                  read using one of these commands pre-calculated
 *                  from the main line code and just use this function
 *                  to transfer the data.
 
 *                  Get Status Requests fields
 *                  SetupPkt.bRequest holds bRequest(request attribute) 
 *                  SetupPkt.W_Index holds endpoint
 *                  SetupPkt.W_Index.byte.HB holds the Entity ID
 *                  SetupPkt.W_Index.byte.LB holds the interface number
 *                  SetupPkt.wLength holds length of the parameter block 
 *******************************************************************/
 #if defined USB_AUDIO_STATUS_REQUESTS_HANDLER
 void UsbAudioStatusRequestsHandler(void)
 {
 }
 #endif 
     

void USBAudioTasks(void)
{   
	int i;
	
    BlinkUSBStatus();
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;
    

 	if (receivedDataEvenNeedsServicingNext == TRUE) 
	{
    	if(!USBHandleBusy(USBRxEvenHandle))
    	{
			AK4645AWrite(pCodecHandle, ReceivedDataEvenBuffer, pCodecHandle->frameSize);
			AK4645AAdjustSampleRateTx(pCodecHandle);        	
        	USBRxEvenHandle = USBRxOnePacket(AS_EP_OUT,(BYTE*)&ReceivedDataEvenBuffer,sizeof(AUDIO_PLAY_SAMPLE)*pCodecHandle->frameSize);
			receivedDataEvenNeedsServicingNext = FALSE; 
		}
    }
	else 
	{
		if(!USBHandleBusy(USBRxOddHandle))
		{
			AK4645AWrite(pCodecHandle, ReceivedDataOddBuffer, pCodecHandle->frameSize);
			AK4645AAdjustSampleRateTx(pCodecHandle);
        	USBRxOddHandle = USBRxOnePacket(AS_EP_OUT,(BYTE*)&ReceivedDataOddBuffer,sizeof(AUDIO_PLAY_SAMPLE)*pCodecHandle->frameSize);
			receivedDataEvenNeedsServicingNext = TRUE; 	
		}
	}	

 	if (transmittedDataEvenNeedsServicingNext == TRUE) 
	{
    	if(!USBHandleBusy(USBTxEvenHandle))
    	{	
        	USBTxEvenHandle = USBTxOnePacket(AS_EP_IN,(BYTE*)&TransmittedDataEvenBuffer,(sizeof(AUDIO_PLAY_SAMPLE)*pCodecHandle->frameSize)/*>>1*/);
			AK4645ARead(pCodecHandle, TransmittedDataEvenBuffer, pCodecHandle->frameSize);     
			//AK4645AAdjustSampleRateRx(pCodecHandle);	
			transmittedDataEvenNeedsServicingNext = FALSE; 
		}
    }
	else 
	{
		if(!USBHandleBusy(USBTxOddHandle))
		{
        	USBTxOddHandle = USBTxOnePacket(AS_EP_IN,(BYTE*)&TransmittedDataOddBuffer,(sizeof(AUDIO_PLAY_SAMPLE)*pCodecHandle->frameSize)/*>>1*/);
	    	AK4645ARead(pCodecHandle, TransmittedDataOddBuffer, pCodecHandle->frameSize);     
			//AK4645AAdjustSampleRateRx(pCodecHandle);		
			transmittedDataEvenNeedsServicingNext = TRUE; 	
		}
	}	

}

/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs 
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
 
void BlinkUSBStatus(void)
{
    static WORD led_count=0;
    
    if(led_count == 0)led_count = 30000U;
    led_count--;

    #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
    #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
    #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
    #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if(USBSuspendControl == 1)
    {
        if(led_count==0)
        {
            mLED_1_Toggle();
            if(mGetLED_1())
            {
                mLED_2_On();
            }
            else
            {
                mLED_2_Off();
            }
        }//end if
    }
    else
    {
        if(USBDeviceState == DETACHED_STATE)
        {
            mLED_Both_Off();
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            mLED_Both_On();
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            mLED_Only_1_On();
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            mLED_Only_2_On();
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            if(led_count == 0)
            {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            if(led_count==0)
            {
                mLED_1_Toggle();
                if(mGetLED_1())
                {
                    mLED_2_Off();
                }
                else
                {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

}

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    	
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
	#ifdef USB_USE_AUDIO_CLASS
	USBCheckAudioRequest();
	#endif
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //Enable the Audio Streaming IN and OUT endpoints
    USBEnableEndpoint(AS_EP_IN,USB_OUT_ENABLED|USB_IN_ENABLED|USB_DISALLOW_SETUP);
    USBEnableEndpoint(AS_EP_OUT,USB_OUT_ENABLED|USB_IN_ENABLED|USB_DISALLOW_SETUP);

    //Prepare the IN and OUT endpoints to transfer the first packets from the host.
    USBRxEvenHandle = USBRxOnePacket(AS_EP_OUT,(BYTE*)&ReceivedDataEvenBuffer,sizeof(AUDIO_PLAY_SAMPLE)*pCodecHandle->frameSize); //First Audio Packet sent will arrive in the even buffer.
	USBRxOddHandle = USBRxOnePacket(AS_EP_OUT,(BYTE*)&ReceivedDataOddBuffer,sizeof(AUDIO_PLAY_SAMPLE)*pCodecHandle->frameSize); //Second Audio Packet sent will arrive in the odd buffer.
	receivedDataEvenNeedsServicingNext = TRUE;	//Used to keep track of which buffer will contain the next sequential data packet.
	
    USBTxEvenHandle = USBTxOnePacket(AS_EP_IN,(BYTE*)&TransmittedDataEvenBuffer,sizeof(AUDIO_PLAY_SAMPLE)*pCodecHandle->frameSize); //First Audio Packet sent will arrive in the even buffer.
	USBTxOddHandle = USBTxOnePacket(AS_EP_IN,(BYTE*)&TransmittedDataOddBuffer,sizeof(AUDIO_PLAY_SAMPLE)*pCodecHandle->frameSize); //Second Audio Packet sent will arrive in the odd buffer.
	transmittedDataEvenNeedsServicingNext = TRUE;	//Used to keep track of which buffer will contain the next sequential data packet.
	
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior, 
 *                  as a USB device that has not been armed to perform remote 
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *                  
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are 
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex: 
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup. 
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in 
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager 
    //properties page for the USB device, power management tab, the 
    //"Allow this device to bring the computer out of standby." checkbox 
    //should be checked).
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            
            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
                                        //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at 
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;        
            do
            {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
        	AK4645ADACMute(pCodecHandle, TRUE);
        	AK4645ABufferClear(pCodecHandle);
        	AK4645ADACMute(pCodecHandle, FALSE);
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was 
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was 
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT 
            //      endpoints).
            break;
        default:
            break;
    }      
    return TRUE; 
}




#define DEBOUNCE_TIME   (250 * (TICKS_PER_SECOND / 1000ull))
void CheckButtons( )
{
    // Process BUTTON_VOLUME_DOWN(S3) presses, including debouncing
    {
        static BOOL     switchBouncing = TRUE;
        static UINT32   switchTime = 0;
        if (switchBouncing)
        {
            if (sw3 == 0)
            {
                switchTime = TickGet();
            }
            else if (TickGet() - switchTime >= DEBOUNCE_TIME)
            {
                switchBouncing = FALSE;
            }
        }
        else if (sw3 == 0)
        {
        	mLED_3_Toggle();
	       	AK4645ASetInput(pCodecHandle, input_enable);
       		input_enable=!input_enable;
	        switchBouncing = TRUE;
	
	}
    }
}

/*** VOLUME INIT AND CONTROL ***/

void VolumeControlInit()
{
	TRISAbits.TRISA0=1;
	ANSELA = 1;
	AD1CHS = 0;
	AD1CON1 = 0x080E0;      
    AD1CON2 = 0;            
    AD1CON3 = 0x1FFF;       
    AD1CSSL = 0;            
}

void VolumeControlTask()
{
	AD1CON1bits.SAMP = 1;
	while(!AD1CON1bits.DONE);
	volumePot = ADC1BUF0;
	if(pCodecHandle != NULL){
		if((volumePot > (prevvolumePot+15)) || (volumePot < (prevvolumePot-15)))
			AK4645ASetDACVolume(pCodecHandle,(((UINT32)(volumePot*100))>>10));
		prevvolumePot = volumePot;
	}
}

/** EOF main.c *************************************************/
#endif
