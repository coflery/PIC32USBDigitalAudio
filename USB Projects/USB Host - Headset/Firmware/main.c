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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "usb_config.h"
#include "./USB/usb.h"
#include "USB/usb_host_audio_v1.h"
#include "ak4645a.h"
#include "Tick.h"


/** PROTOTYPES *********************************************/
static void InitializeSystem(void);
void BlinkUSBStatus(void);

/** VARIABLES ******************************************************/
volatile BOOL deviceAttached;
INT audioDataLength=0;
static UINT32_VAL currentSampleRate;
DWORD desiredFrequency = 48000; 

ISOCHRONOUS_DATA StreamingAudioIN;
ISOCHRONOUS_DATA StreamingAudioOUT;
static USB_AUDIO_V1_DEVICE_ID  deviceAudio;

AK4645AState* pCodecHandle=NULL;
UINT16 volumePot=0;
UINT16 prevvolumePot=-1;



enum SET_EVENT
{
	SET_INTERFACE,
	SET_FREQUENCY,
	SET_WRCOMMANDS
}; 
enum SET_EVENT eventState;


/** USB APPLICATION EVENT HANDLER **/
BOOL USB_ApplicationEventHandler( BYTE address, USB_EVENT event, void *data, DWORD size )
{
	switch( event )
    {
	    case EVENT_AUDIO_ATTACH:
            if (!USBHostIsochronousBuffersCreate( &StreamingAudioIN, USB_MAX_ISOCHRONOUS_DATA_BUFFERS, ((USB_AUDIO_V1_DEVICE_ID *)data)->audioDataPacketSize ))
                return TRUE;
            if (!USBHostIsochronousBuffersCreate( &StreamingAudioOUT, USB_MAX_ISOCHRONOUS_DATA_BUFFERS, ((USB_AUDIO_V1_DEVICE_ID *)data)->audioDataPacketSize ))
                return TRUE;
            StreamingAudioOUT.buffers[StreamingAudioOUT.currentBufferUSB].bfDataLengthValid=1;    
			
            audioDataLength=((USB_AUDIO_V1_DEVICE_ID *)data)->audioDataPacketSize;            
			deviceAudio = *(USB_AUDIO_V1_DEVICE_ID *)data;  		
		
    		
    		while( USBHostAudioV1SetInterfaceFullBandwidth( deviceAudio.deviceAddress, 1 ) != USB_SUCCESS );
			eventState = SET_INTERFACE;
            return TRUE;
            //break;	    
			    
        case EVENT_AUDIO_DETACH:
			if ( USBHostAudioV1SetInterfaceZeroBandwidth( deviceAudio.deviceAddress ) )	{}
            USBHostIsochronousBuffersDestroy( &StreamingAudioIN, USB_MAX_ISOCHRONOUS_DATA_BUFFERS );
            USBHostIsochronousBuffersDestroy( &StreamingAudioOUT, USB_MAX_ISOCHRONOUS_DATA_BUFFERS );
            deviceAudio.deviceAddress   = 0;
            return TRUE;            
            //break;  
			
       	case EVENT_AUDIO_INTERFACE_SET:
	       	switch(eventState)
		    {
				case SET_INTERFACE:
					while(( USBHostAudioV1SetInterfaceFullBandwidth( deviceAudio.deviceAddress, 2 ) != USB_SUCCESS ));
					eventState = SET_FREQUENCY;
					break;
					
				case SET_FREQUENCY:
				{
					BYTE  numFrequencies, rc;
					BYTE  *ptr;
					ptr = USBHostAudioV1SupportedFrequencies( deviceAudio.deviceAddress );
					if (ptr)
					{
						numFrequencies = *ptr;
						ptr++;
						if (numFrequencies == 0)
						{
							// Continuous sampling, minimum and maximum are specified.
							DWORD   minFrequency;
							DWORD   maxFrequency;
							
							minFrequency = *ptr + (*(ptr+1) << 8) + (*(ptr+2) << 16);
							ptr += 3;
							maxFrequency = *ptr + (*(ptr+1) << 8) + (*(ptr+2) << 16);
							if ((minFrequency <= desiredFrequency) && (desiredFrequency <= maxFrequency))
							{
								rc = USBHostAudioV1SetSamplingFrequency( deviceAudio.deviceAddress, (BYTE *)&desiredFrequency, 1 );
							}
							else
							{
								// Desired frequency out of range
							}
						}
						else
						{
							// Discrete sampling frequencies are specified.    
							DWORD frequency;
							
							while (numFrequencies)
							{
								frequency = *ptr + (*(ptr+1) << 8) + (*(ptr+2) << 16);
								if (frequency == desiredFrequency)
								{
									rc = USBHostAudioV1SetSamplingFrequency( deviceAudio.deviceAddress, ptr, 1 );
									//continue;
								}
								numFrequencies--;
								ptr += 3;
							}
							if (numFrequencies == 0)
							{
								// Desired frequency not found.
							}
						}
					}
					return TRUE;
				}
					//break;
			}		
			break;
 
		case EVENT_AUDIO_FREQUENCY_SET:
	       	switch(eventState)
		    {
				case SET_FREQUENCY:		
				{
					BYTE  numFrequencies, rc;
					BYTE  *ptr;
					
					ptr = USBHostAudioV1SupportedFrequencies( deviceAudio.deviceAddress );
					if (ptr)
					{
						numFrequencies = *ptr;
						ptr++;
						if (numFrequencies == 0)
						{
							// Continuous sampling, minimum and maximum are specified.
							DWORD   minFrequency;
							DWORD   maxFrequency;
							
							minFrequency = *ptr + (*(ptr+1) << 8) + (*(ptr+2) << 16);
							ptr += 3;
							maxFrequency = *ptr + (*(ptr+1) << 8) + (*(ptr+2) << 16);
							if ((minFrequency <= desiredFrequency) && (desiredFrequency <= maxFrequency))
							{
								rc = USBHostAudioV1SetSamplingFrequency( deviceAudio.deviceAddress, (BYTE *)&desiredFrequency, 0x82 );
							}
							else
							{
								// Desired frequency out of range
							}
						}
						else
						{
							// Discrete sampling frequencies are specified.    
							DWORD frequency;
							
							while (numFrequencies)
							{
								frequency = *ptr + (*(ptr+1) << 8) + (*(ptr+2) << 16);
								if (frequency == desiredFrequency)
								{
									while((rc = USBHostAudioV1SetSamplingFrequency( deviceAudio.deviceAddress, ptr, 0x82 )) 
				                	  == USB_ENDPOINT_BUSY);
								}
								numFrequencies--;
								ptr += 3;
							}
							if (numFrequencies == 0)
							{
								// Desired frequency not found.
							}
						}
					}   
					eventState = SET_WRCOMMANDS;    	
					return TRUE;
				}
				//	break;				
				
				case SET_WRCOMMANDS:				
				{
					 BYTE rc;				
					 	             
					 rc=USBHostAudioV1ReceiveAudioData( deviceAudio.deviceAddress,
					         &StreamingAudioIN );
					 if ( rc == USB_SUCCESS)
					 {
						//TODO: Add fail handle
						 Nop();
					 }
					 else 
					 {
						 Nop();
					 }	  
					 
					 rc=USBHostAudioV1TransmitAudioData( deviceAudio.deviceAddress,
							 &StreamingAudioOUT, audioDataLength  );        
					 if ( rc == USB_SUCCESS)
					 {
						//TODO: Add fail handle
						 Nop();
					 }
					 else 
					 {
						 Nop();
					 }				 
								  
				}           	
				return TRUE;
				
				//break;
			}
			break;
			
 		case EVENT_AUDIO_STREAM_RECEIVED:
 			{ 			
    		}         
            return TRUE;
            //break; 


 		case EVENT_AUDIO_STREAM_TRANSMITTED:
	 		{
 			}	        
            return TRUE;
            //break; 
                                	    
        case EVENT_VBUS_REQUEST_POWER:
            return TRUE;

        case EVENT_VBUS_RELEASE_POWER:
            deviceAttached = FALSE;
            return TRUE;
            break;

        case EVENT_HUB_ATTACH:
            return TRUE;
            break;

        case EVENT_UNSUPPORTED_DEVICE:
            return TRUE;
            break;

        case EVENT_CANNOT_ENUMERATE:
			Nop();
            return TRUE;
            break;

        case EVENT_CLIENT_INIT_ERROR:
            return TRUE;
            break;

        case EVENT_OUT_OF_MEMORY:
            return TRUE;
            break;

        case EVENT_UNSPECIFIED_ERROR:   
            return TRUE;
            break;

			
        default:
            break;
    }

    return FALSE;
}

/** USB APPLICATION DATA EVENT HANDLER **/
BOOL USB_ApplicationDataEventHandler ( UINT8 address, USB_EVENT event, VOID *data, UINT32 size )
{
	BlinkUSBStatus();
	switch(event){	
		case EVENT_AUDIO_STREAM_RECEIVED:
			AK4645AWrite(pCodecHandle, (AudioStereo*)data, size>>2);
			AK4645AAdjustSampleRateTx(pCodecHandle);
			break;
		
		case EVENT_AUDIO_STREAM_TRANSMITTED:
			AK4645ARead(pCodecHandle, (AudioStereo*)data, size>>2);
			break;
			
		default:
			break;
 	}   
    return TRUE;
}


 /** CONFIGURATION **************************************************/ 
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_3, FPLLODIV = DIV_2
#pragma config POSCMOD = EC, FNOSC = PRIPLL, FPBDIV = DIV_1, FSOSCEN = OFF
#pragma config UPLLEN = ON, UPLLIDIV = DIV_3, FUSBIDIO = OFF, FVBUSONIO = OFF
#pragma config FWDTEN = OFF, JTAGEN = OFF  
#pragma config IESO = OFF, IOL1WAY = ON, PMDL1WAY = ON, OSCIOFNC = OFF       
#pragma config ICESEL = ICS_PGx1  


int main(void)	
{
    InitializeSystem();
	USBInitialize( 0 );
				    
    while(1){
        USBTasks();        
    }    
        
}





static void InitializeSystem(void)
{
	SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);	
    INTEnableSystemMultiVectoredInt();   
    
    TickInit();
////////////////////////////////CODEC INIT

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
////////////////////////////////CODEC INIT END
	mInitAllSwitches(); 					
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
    
    if(led_count == 0)led_count = 35U;
    led_count--;

    #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
    #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
    #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
    #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}


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
    }

 
}



void VolumeControlInit(){
	TRISAbits.TRISA0=1;
	ANSELA = 1;
	AD1CHS = 0;
	AD1CON1 = 0x080E0;      // Turn on, auto-convert
    AD1CON2 = 0;            // AVdd, AVss, int every conversion, MUXA only
    AD1CON3 = 0x1FFF;       // 31 Tad auto-sample, Tad = 256*Tcy
    AD1CSSL = 0;            // No scanned inputs
}

void VolumeControlTask(){
	AD1CON1bits.SAMP = 1;
	while(!AD1CON1bits.DONE);
	volumePot = ADC1BUF0;
	if(pCodecHandle != NULL){
		if((volumePot > (prevvolumePot+100)) || (volumePot < (prevvolumePot-100)))
			AK4645ASetDACVolume(pCodecHandle,(((UINT32)(volumePot*100))>>10));
		prevvolumePot = volumePot;
	}
}



