/**********************************************************************
* ¬© 2007 Microchip Technology Inc.
*
* FileName:        main.c
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       dsPIC33Fxxxx/PIC24Hxxxx
* Compiler:        MPLAB¬Æ C30 v3.00 or higher
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in all
* derivatives hereto.  You may use this code, and any derivatives created by
* any person or entity by or on your behalf, exclusively with Microchip,s
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP,S
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY),
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL,
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* ADDITIONAL NOTES:
* Code Tested on:
* Explorer16 development board and ADC/PWM PICTail Board
**********************************************************************/

#include <p33FJ256GP710A.h>
#include "ADCChannelDrv.h"
#include "OCPWMDrv.h"
#include "G711.h"
#include "AES.h"

#define FRAME 20
_FGS(GWRP_OFF & GCP_OFF);
_FOSCSEL(FNOSC_FRC);
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
_FWDT(FWDTEN_OFF);

int adcBuffer		[ ADC_CHANNEL_DMA_BUFSIZE] __attribute__((space(dma)));
int ocPWMBuffer	[OCPWM_DMA_BUFSIZE]	__attribute__((space(dma)));

int 	inputFrame		[FRAME];
char encodedFrame	[FRAME];
int 	decodedFrame		[FRAME];

ADCChannelHandle 	adcChannelHandle;
OCPWMHandle 		ocPWMHandle;

ADCChannelHandle *pADCChannelHandle 	= &adcChannelHandle;
OCPWMHandle 	*pOCPWMHandle 		= &ocPWMHandle;


int main(void)
{

	CLKDIV = 0;
	PLLFBD = 38;

	__builtin_write_OSCCONH(0x03);		/*	Initiate Clock Switch to Primary Oscillator with PLL (NOSC=0b011)*/
	__builtin_write_OSCCONL(0x01);
	while (OSCCONbits.COSC != 0b011);	/*	Wait for Clock switch to occur	*/
	while(!OSCCONbits.LOCK);

	ADCChannelInit	(pADCChannelHandle,adcBuffer);	/*Initialize the ADC Channel	*/
	ADCChannelStart	(pADCChannelHandle);			/*Start acquiring samples	*/
	OCPWMInit		(pOCPWMHandle,ocPWMBuffer);	/*Initialize the OC module 	*/
	OCPWMStart		(pOCPWMHandle);				/*Start playback of samples	*/


       int key[8] = {0x0100,0x0302,0x0504,0x0706,0x0908,0x0B0A,0x0D0C,0x0F0E};


	while(1)
	{
		/* Wait till a frame of input data is available	*/
		while(ADCChannelIsBusy(pADCChannelHandle));

		/* Read the data into inputBuffer	*/
		ADCChannelRead	(pADCChannelHandle,inputFrame,FRAME);

		/*Encode the frame	*/
		G711Lin2Ulaw(inputFrame,encodedFrame,FRAME);

                AESEncrypt((int*)encodedFrame, key);
                AESCalcDecKey(key);
                AESDecrypt((int*)encodedFrame, key);

		/* Decode the frame	*/
		G711Ulaw2Lin(encodedFrame,decodedFrame,FRAME);

		/* Wait till the OC is available for new a frame	*/
		while(OCPWMIsBusy(pOCPWMHandle));

		/* Write the decoded frame to the output	*/
		OCPWMWrite (pOCPWMHandle,decodedFrame,FRAME);


	}

}

