/**********************************************************************
* © 2005 Microchip Technology Inc.
*
* FileName:        main.c
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       dsPIC33Fxxxx/PIC24Hxxxx
* Compiler:        MPLAB® C30 v3.00 or higher
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
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
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author            Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Settu D 			03/09/06  First release of source file
* MC				09/09/11  Updated auto sample time to 2*Tad
*							  Replaced pin toggle with C30 built in function
*							  Updated Configuration settings
*							  Other misc corrections
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~**
*
* ADDITIONAL NOTES:
* Code Tested on:
* Explorer16 Demo board with dsPIC33FJ256GP710A controller 
* The Processor starts with the Internal oscillator without PLL enabled and then the Clock is switched to PLL Mode.

**********************************************************************/

#if defined(__dsPIC33F__)
#include "p33Fxxxx.h"
#elif defined(__PIC24H__)
#include "p24Hxxxx.h"
#endif

#include "adcDrv2.h"

//Macros for Configuration Fuse Registers:
//Invoke macros to set up  device configuration fuse registers.
//The fuses will select the oscillator source, power-up timers, watch-dog
//timers etc. The macros are defined within the device
//header files. The configuration fuse registers reside in Flash memory.

_FGS(GWRP_OFF & GSS_OFF & GCP_OFF);
_FOSCSEL(FNOSC_FRC & IESO_OFF);
//_FOSC(POSCMD_XT & OSCIOFNC_OFF & FCKSM_CSECMD);
_FOSC(POSCMD_NONE & OSCIOFNC_ON & FCKSM_CSECMD);
_FWDT(FWDTEN_OFF);
_FICD(JTAGEN_OFF & ICS_PGD1);

#define PI2 6.2831853072
#define NFRAC 32
#define NFRACP 5
#include "math.h"

//audio FFT
#define NDIM 16
#define NDIMP 4
int iw0r[NDIM/2]; int iw0i[NDIM/2];
int ix1r[NDIM]; int ix1i[NDIM];
int ifr[NDIM]; int ifi[NDIM];
int n0, np0;

//disp FFT
#define NDDIM 128
#define NDDIMP 7
int idw0r[NDDIM/2]; int idw0i[NDDIM/2];
int idfr[NDDIM]; int idfi[NDDIM];
int idx1r[NDDIM]; int idx1i[NDDIM];
int nd0, ndp0;
int itraj[NDDIM];

#define AFOUT 3
#define FRQSTEP 4.167

#define NUMSAMP	NDDIM*2
int BufferA[NUMSAMP] __attribute__((space(dma)));
//int BufferB[NUMSAMP] __attribute__((space(dma)));

void initDma0(void)
{
	DMA0CONbits.SIZE = 0;//Data transfer size = word
	DMA0CONbits.DIR = 0;//Read from peripheral
	DMA0CONbits.AMODE = 0;			// Configure DMA for Register indirect with post increment
//	DMA0CONbits.MODE  = 2;			// Configure DMA for Continuous Ping-Pong mode
	DMA0CONbits.MODE  = 0;			// Configure DMA for Continuous Ping-Pong mode disabled

	DMA0PAD=(int)&ADC1BUF0;
	DMA0CNT=(NUMSAMP-1);				
	
	DMA0REQ=13;	
	
	DMA0STA = __builtin_dmaoffset(BufferA);		
//	DMA0STB = __builtin_dmaoffset(BufferB);

	IFS0bits.DMA0IF = 0;			//Clear the DMA interrupt flag bit
    IEC0bits.DMA0IE = 0;			//Clear the DMA interrupt enable bit
	
	DMA0CONbits.CHEN=1;


}

void wait(unsigned int imicrosec) {
	unsigned int i;
	unsigned int imax = (int)(22.69*imicrosec);
	for (i=0; i<imax; i++) {asm("NOP");}
}

void wait2ms(void) {//2.00 msec
	int i, j;
	for (j=0; j<10; j++) {
		for (i=0; i<4554; i++) {asm("NOP");}
	}
}

void FFTinit() {
	np0 = NDIMP;
	n0 = NDIM; 

	int i;
	for (i=0; i<(n0 / 2); i++) {
		iw0r[i] = (int)(cos(PI2 * i / n0) * NFRAC);
		iw0i[i] = (int)(sin(PI2 * i / n0) * NFRAC);
	}
	return;
}

void FFTDinit() {
	ndp0 = NDDIMP;
	nd0 = NDDIM; 

	int i;
	for (i=0; i<(nd0 / 2); i++) {
		idw0r[i] = (int)(cos(PI2 * i / nd0) * NFRAC);
		idw0i[i] = (int)(sin(PI2 * i / nd0) * NFRAC);
	}
	return;
}

void FFT(void) {
	const int n2 = n0 >> 1;
	int al = 1; int bt = n2; int jal, jal2, abinc, idx;
	int scrr, scri, sc2r, sc2i; 

	abinc = bt;
	int j, l, k;
	for (j=0; j<bt; j++) {
		scrr = ifr[j + abinc]; scri = ifi[j + abinc];
		sc2r = ifr[j]; sc2i = ifi[j];
		ix1r[j << 1] = (sc2r + scrr);// >> NFRACP2;
		ix1i[j << 1] = (sc2i + scri);// >> NFRACP2;
		ix1r[(j << 1) + 1] = (sc2r - scrr);// >> NFRACP2;
		ix1i[(j << 1) + 1] = (sc2i - scri);// >> NFRACP2;
	}
	al = al << 1; bt = bt >> 1;
	int jal3;
	for (l=1; l<(np0-1); l++) {//x[j,k]=x[j*alpha + k]
		if (l & 0x01) {
			for (j=0; j<bt; j++) {
				jal = j * al; jal2 = (jal << 1);
				for (k=0; k<abinc; k+=bt) {
					jal3 = jal + abinc;
					scrr = (ix1r[jal3] * iw0r[k] - ix1i[jal3] * iw0i[k]) >> NFRACP;
					scri = (ix1r[jal3] * iw0i[k] + ix1i[jal3] * iw0r[k]) >> NFRACP;
					ifr[jal2+al] = (ix1r[jal] - scrr);// >> NFRACP2;
					ifi[jal2+al] = (ix1i[jal] - scri);// >> NFRACP2;
					ifr[jal2] = (ix1r[jal] + scrr);// >> NFRACP2;
					ifi[jal2] = (ix1i[jal] + scri);// >> NFRACP2;
					jal++; jal2++;
				}
			}
		} else {
			for (j=0; j<bt; j++) {
				jal = j * al; jal2 = jal << 1;
				for (k=0; k<abinc; k+=bt) {
					jal3 = jal + abinc;
					scrr = (ifr[jal3] * iw0r[k] - ifi[jal3] * iw0i[k]) >> NFRACP;
					scri = (ifr[jal3] * iw0i[k] + ifi[jal3] * iw0r[k]) >> NFRACP;
					ix1r[jal2+al] = (ifr[jal] - scrr);// >> NFRACP2;
					ix1i[jal2+al] = (ifi[jal] - scri);// >> NFRACP2;
					ix1r[jal2] = (ifr[jal] + scrr);// >> NFRACP2;
					ix1i[jal2] = (ifi[jal] + scri);// >> NFRACP2;
					jal++; jal2++;
				}
			}
		}
		al = al << 1; bt = bt >> 1;
	}
	//always (al = n0/2, bt = 1) at here
	if (np0 & 0x01) {
		for (k=0; k<n0; k++) {ix1r[k] = ifr[k]; ix1i[k] = ifi[k];}
	}
	//l=np-1
	abinc = al;
		scrr = ix1r[n2]; scri = ix1i[n2];
		ifr[0] = ix1r[0] + scrr; ifi[0] = ix1i[0] + scri;
		ifr[abinc] = ix1r[0] - scrr; ifi[abinc] = ix1i[0] - scri;
	idx = abinc - 1;
	for (k=1; k<al; k++) {
		jal3 = n2+k; //kbt = k * bt;
		scrr = (ix1r[jal3] * iw0r[k] - ix1i[jal3] * iw0i[k]) >> NFRACP;
		scri = (ix1r[jal3] * iw0i[k] + ix1i[jal3] * iw0r[k]) >> NFRACP;
		ifr[idx+abinc] = ix1r[k] + scrr; ifi[idx+abinc] = ix1i[k] + scri;
		ifr[idx] = ix1r[k] - scrr; ifi[idx] = ix1i[k] - scri;
		idx--;
	}
	return;
}

void FFTD(void) {//fft for disp
	const int n2 = nd0 >> 1;
	int al = 1; int bt = n2; int jal, jal2, abinc, idx;
	int scrr, scri, sc2r, sc2i; 

	abinc = bt;
	int j, l, k;
	for (j=0; j<bt; j++) {
		scrr = idfr[j + abinc]; scri = idfi[j + abinc];
		sc2r = idfr[j]; sc2i = idfi[j];
		idx1r[j << 1] = (sc2r + scrr);// >> NFRACP2;
		idx1i[j << 1] = (sc2i + scri);// >> NFRACP2;
		idx1r[(j << 1) + 1] = (sc2r - scrr);// >> NFRACP2;
		idx1i[(j << 1) + 1] = (sc2i - scri);// >> NFRACP2;
	}
	al = al << 1; bt = bt >> 1;
	int jal3;
	for (l=1; l<(ndp0-1); l++) {//x[j,k]=x[j*alpha + k]
		if (l & 0x01) {
			for (j=0; j<bt; j++) {
				jal = j * al; jal2 = (jal << 1);
				for (k=0; k<abinc; k+=bt) {
					jal3 = jal + abinc;
					scrr = (idx1r[jal3] * idw0r[k] - idx1i[jal3] * idw0i[k]) >> NFRACP;
					scri = (idx1r[jal3] * idw0i[k] + idx1i[jal3] * idw0r[k]) >> NFRACP;
					idfr[jal2+al] = (idx1r[jal] - scrr);// >> NFRACP2;
					idfi[jal2+al] = (idx1i[jal] - scri);// >> NFRACP2;
					idfr[jal2] = (idx1r[jal] + scrr);// >> NFRACP2;
					idfi[jal2] = (idx1i[jal] + scri);// >> NFRACP2;
					jal++; jal2++;
				}
			}
		} else {
			for (j=0; j<bt; j++) {
				jal = j * al; jal2 = jal << 1;
				for (k=0; k<abinc; k+=bt) {
					jal3 = jal + abinc;
					scrr = (idfr[jal3] * idw0r[k] - idfi[jal3] * idw0i[k]) >> NFRACP;
					scri = (idfr[jal3] * idw0i[k] + idfi[jal3] * idw0r[k]) >> NFRACP;
					idx1r[jal2+al] = (idfr[jal] - scrr);// >> NFRACP2;
					idx1i[jal2+al] = (idfi[jal] - scri);// >> NFRACP2;
					idx1r[jal2] = (idfr[jal] + scrr);// >> NFRACP2;
					idx1i[jal2] = (idfi[jal] + scri);// >> NFRACP2;
					jal++; jal2++;
				}
			}
		}
		al = al << 1; bt = bt >> 1;
	}
	//always (al = n0/2, bt = 1) at here
	if (ndp0 & 0x01) {
		for (k=0; k<nd0; k++) {idx1r[k] = idfr[k]; idx1i[k] = idfi[k];}
	}
	//l=np-1
	abinc = al;
		scrr = idx1r[n2]; scri = idx1i[n2];
		idfr[0] = idx1r[0] + scrr; idfi[0] = idx1i[0] + scri;
		idfr[abinc] = idx1r[0] - scrr; idfi[abinc] = idx1i[0] - scri;
	idx = abinc - 1;
	for (k=1; k<al; k++) {
		jal3 = n2+k; //kbt = k * bt;
		scrr = (idx1r[jal3] * idw0r[k] - idx1i[jal3] * idw0i[k]) >> NFRACP;
		scri = (idx1r[jal3] * idw0i[k] + idx1i[jal3] * idw0r[k]) >> NFRACP;
		idfr[idx+abinc] = idx1r[k] + scrr; idfi[idx+abinc] = idx1i[k] + scri;
		idfr[idx] = idx1r[k] - scrr; idfi[idx] = idx1i[k] - scri;
		idx--;
	}
	return;
}

#define OSC_WCLK1_IO LATBbits.LATB6
#define OSC_WCLK2_IO LATBbits.LATB7
#define OSC_FQUD_IO LATBbits.LATB8
#define OSC_DATA_IO LATBbits.LATB9
#define OSC_RESET_IO LATBbits.LATB10
#define OSC_WAIT {}

void osc_reset(void) {
	//reset
	OSC_RESET_IO = 0;
	OSC_FQUD_IO = 0;
	OSC_DATA_IO = 0;
	OSC_WCLK1_IO = 0;
	OSC_WCLK2_IO = 0;
	OSC_RESET_IO = 1;
	wait(1);
	OSC_RESET_IO = 0;
	OSC_WAIT;
	//set as serial
	OSC_WCLK1_IO = 1;
	OSC_WCLK2_IO = 1;
	OSC_WAIT;
	OSC_WCLK1_IO = 0;
	OSC_WCLK2_IO = 0;
	OSC_WAIT;
	OSC_FQUD_IO = 1;
	OSC_WAIT;
	OSC_FQUD_IO = 0;
}

void osc_init(void) {
//	AD1PCFGL=0xFFFF;	// Set as digital
	LATBbits.LATB6 = 0;
	LATBbits.LATB7 = 0;
	LATBbits.LATB8 = 0;
	LATBbits.LATB9 = 0;
	LATBbits.LATB10 = 0;
//config RB6 as output
	ODCBbits.ODCB6 = 0;//disable open drain
	//ODCBbits.ODCB6 = 1;//enable open drain; this causes DDS errors
	TRISBbits.TRISB6 = 0;
//config RB7 as output
	ODCBbits.ODCB7 = 0;	
	TRISBbits.TRISB7 = 0;
//config RB8 as output
	ODCBbits.ODCB8 = 0;	
	TRISBbits.TRISB8 = 0;
//config RB9 as output
	ODCBbits.ODCB9 = 0;	
	TRISBbits.TRISB9 = 0;
//config RB10 as output
	ODCBbits.ODCB10 = 0;//disable open drain	
	TRISBbits.TRISB10 = 0;

	osc_reset();
}

int iphase_offset = 0;
void osc_set(double dfreq, int iphase) {
//AD9851 serial 
//freq = (lfreq / 2^32) * 180 MHz
//phase = iphase * 11.25 deg
//dfreq: frequency in kHz
	long lfreq = (long)(23860.92942222 * dfreq);
	iphase = (iphase + iphase_offset) & 0x1f;
	int i;
	//freq
	for(i=0; i<32; i++) {
		OSC_DATA_IO = ((lfreq & 0x01) != 0);
		OSC_WAIT;
    	OSC_WCLK1_IO = 1;
    	OSC_WCLK2_IO = 1;
		OSC_WAIT;
    	OSC_WCLK1_IO = 0;
    	OSC_WCLK2_IO = 0;
		OSC_WAIT;
		lfreq >>= 1;
	}
	//6x REFCLK
	OSC_DATA_IO = 1;
	OSC_WAIT;
   	OSC_WCLK1_IO = 1;
   	OSC_WCLK2_IO = 1;
	OSC_WAIT;
   	OSC_WCLK1_IO = 0;
   	OSC_WCLK2_IO = 0;
	OSC_WAIT;
	//reserved
	OSC_DATA_IO = 0;
	OSC_WAIT;
   	OSC_WCLK1_IO = 1;
   	OSC_WCLK2_IO = 1;
	OSC_WAIT;
   	OSC_WCLK1_IO = 0;
   	OSC_WCLK2_IO = 0;
	OSC_WAIT;
	//disable power-down mode
	OSC_DATA_IO = 0;
	OSC_WAIT;
   	OSC_WCLK1_IO = 1;
   	OSC_WCLK2_IO = 1;
	OSC_WAIT;
   	OSC_WCLK1_IO = 0;
   	OSC_WCLK2_IO = 0;
	OSC_WAIT;
	//phase
	for(i=0; i<5; i++) {
		OSC_DATA_IO = 0;
		OSC_WAIT;
    	OSC_WCLK1_IO = 1;
		OSC_WAIT;
    	OSC_WCLK1_IO = 0;
		OSC_WAIT;
		OSC_DATA_IO = ((iphase & 0x01) != 0);
		OSC_WAIT;
		OSC_WCLK2_IO = 1;
		OSC_WAIT;
    	OSC_WCLK2_IO = 0;
		OSC_WAIT;
		iphase >>= 1;
	}
	OSC_FQUD_IO = 1;
	OSC_WAIT;
	OSC_FQUD_IO = 0;
	OSC_WAIT;
}

void osc_initphase(void) {
	//exec after ADC and DMA init
	int i;
	float fsum[32];
	iphase_offset = 0;
	for (i=0; i<1024; i++) {
		int iphase = i & 0x1f;
		osc_set(100.0, iphase);
		wait(500);
		int j;
		fsum[iphase] = 0;
		for (j=0; j<NDDIM; j++) {
			unsigned long ldiff = BufferA[j*2+1] - BufferA[j*2];
			fsum[iphase] += (ldiff * ldiff);
		}
	}
	int imin = 0;
	float fmin;
	for (i=0; i<32; i++) {
		if (i == 0) {fmin = fsum[0]; imin = 0;}
		else if (fsum[i] < fmin) {fmin = fsum[i]; imin = i;}
	}
	iphase_offset = imin;
	osc_set(100.0, 8);
}

const unsigned char cchar[43][5] = {
	{0x7C,0xA2,0x92,0x8A,0x7C},		// 30	0
	{0x00,0x84,0xFE,0x80,0x00},		// 31	1
	{0xE4,0x92,0x92,0x92,0x8C},		// 32	2
	{0x44,0x82,0x92,0x92,0x6C},		// 33	3
	{0x30,0x28,0x24,0xFE,0x20},		// 34	4
	{0x4E,0x8A,0x8A,0x8A,0x72},		// 35	5
	{0x78,0x94,0x92,0x92,0x60},		// 36	6
	{0x02,0xE2,0x12,0x0A,0x06},		// 37	7
	{0x6C,0x92,0x92,0x92,0x6C},		// 38	8
	{0x0C,0x92,0x92,0x52,0x3C},		// 39	9
	{0x00,0x00,0x00,0x00,0x00},		// 3A	blank(20)
	{0xB8,0x64,0x5C,0x44,0x3A},		// 3B	phi	//instead of ";"
	{0x10,0x28,0x44,0x82,0x00},		// 3C	<
	{0x28,0x28,0x28,0x28,0x28},		// 3D	=
	{0x82,0x44,0x28,0x10,0x00},		// 3E	>
	{0x40,0x20,0x10,0x08,0x04},		// 3F	/	//instead of "?"
	{0x00,0xC0,0xC0,0x00,0x00},		// 40	.	//instaed of "@"
	{0xF8,0x24,0x22,0x24,0xF8},		// 41	A 
	{0x82,0xFE,0x92,0x92,0x6C},		// 42	B 
	{0x7C,0x82,0x82,0x82,0x44},		// 43	C 
	{0x82,0xFE,0x82,0x82,0x7C},		// 44	D 
	{0xFE,0x92,0x92,0x82,0x82},		// 45	E 
	{0xFE,0x12,0x12,0x02,0x02},		// 46	F 
	{0x7C,0x82,0x92,0x92,0x74},		// 37	G 
	{0xFE,0x10,0x10,0x10,0xFE},		// 48	H 
	{0x00,0x82,0xFE,0x82,0x00},		// 49	I
	{0x40,0x80,0x82,0x7E,0x02},		// 4A	J 
	{0xFE,0x10,0x28,0x44,0x82},		// 4B	K 
	{0xFE,0x80,0x80,0x80,0x80},		// 4C	L 
	{0xFE,0x04,0x18,0x04,0xFE},		// 4D	M 
	{0xFE,0x04,0x08,0x10,0xFE},		// 4E	N 
	{0x7C,0x82,0x82,0x82,0x7C},		// 4F	O 
	{0xFE,0x12,0x12,0x12,0x0C},		// 50	P 
	{0x7C,0x82,0xA2,0x42,0xBC},		// 51	Q
	{0xFE,0x12,0x32,0x52,0x8C},		// 52	R 
	{0x4C,0x92,0x92,0x92,0x64},		// 53	S 
	{0x02,0x02,0xFE,0x02,0x02},		// 54	T 
	{0x7E,0x80,0x80,0x80,0x7E},		// 55	U 
	{0x0E,0x30,0xC0,0x30,0x0E},		// 56	V 
	{0xFE,0x40,0x30,0x40,0xFE},		// 57	W 
	{0xC6,0x28,0x10,0x28,0xC6},		// 58	X 
	{0x06,0x08,0xF0,0x08,0x06},		// 59	Y 
	{0xC2,0xA2,0x92,0x8A,0x86}		// 5A	Z 
};

void lcd_init(void) {
//	AD1PCFGL=0xFFFF;	// Set as digital
	LATAbits.LATA4 = 0;
	LATBbits.LATB3 = 1;
	LATBbits.LATB4 = 1;
	LATBbits.LATB5 = 1;
//config RA4 as output
	ODCAbits.ODCA4 = 0;//disable open drain
	TRISAbits.TRISA4 = 0;//set as output
//config RB3 as output
	ODCBbits.ODCB3 = 0;	
	TRISBbits.TRISB3 = 0;
//config RB4 as output
	ODCBbits.ODCB4 = 0;	
	TRISBbits.TRISB4 = 0;
//config RB5 as output
	ODCBbits.ODCB5 = 0;	
	TRISBbits.TRISB5 = 0;
}

#define LCD_CS_IO LATBbits.LATB5
#define LCD_RS_IO LATAbits.LATA4
#define LCD_SDO_IO LATBbits.LATB3
#define LCD_SCK_IO LATBbits.LATB4
void lcd_write(int idata, int irs) {
    LCD_CS_IO = 0;
	if (irs) LCD_RS_IO = 1; else LCD_RS_IO = 0;//RS

	unsigned int i;
    for(i=0; i<8; i++) {
		asm("nop");
		LCD_SDO_IO = ((idata & 0x80) != 0);
		asm("nop");
		LCD_SCK_IO = 0;
		//asm("nop");
        LCD_SCK_IO = 1;
		asm("nop");
        idata <<= 1;
    }
    LCD_CS_IO = 1;
	return;
}

void lcd_pos(unsigned char xpos, unsigned char ypos){
    lcd_write(0xB0 | ypos, 0);
    lcd_write(0x10 | (xpos >> 4), 0);
    lcd_write(xpos & 0x0F, 0);
}

void lcd_char(char cdata, char cinv){
	int i;
	if (cdata == 0x20) cdata = 0x3a;
	cdata -= 0x30;
	if (cinv) {
		for(i=0; i<5; i++) {lcd_write(~(cchar[cdata][i]), 1);}
		lcd_write(0xff, 1);
	} else {
		for(i=0; i<5; i++) {lcd_write(cchar[cdata][i], 1);}
		lcd_write(0x00, 1);
	}
}

void lcd_string(char* pcstr, char cinv) {
    while (*pcstr) {lcd_char(*pcstr++, cinv);}
}

void lcd_printHex(unsigned int iout, char idigit) {
	char buf;
	signed char i;
	for (i=3; i>=0; i--) {
		buf = (iout >> (i * 4)) & 0x000f;
		if (buf < 10) buf += 0x30; else buf += 0x37;
		if (i < idigit) lcd_char(buf, 0);
	}
}

void lcd_printDec2(unsigned char cout, char cinv) {
	//char 8 bit ==> 2 decimal digits
	char buf0, buf1;
	buf0 = cout / 10;
	buf1 = cout - buf0 * 10;
	lcd_char(buf0 + 0x30, cinv);
	lcd_char(buf1 + 0x30, cinv);
}

void lcd_printDec5(unsigned int iout, int* pidigit, char csel) {
	//char 8 bit ==> 5 decimal digits
	int i;
	int idiv = 10000;
	int iomit = 1;
	*pidigit = 0;
	for (i=0; i<5; i++) {
		char buf0 = iout / idiv;
		char cinv  = (csel == i) ? 1 : 0;
		if (iomit && (i != 4)) {
			if (buf0) {lcd_char(buf0 + 0x30, cinv); iomit = 0; (*pidigit)++;}
			else lcd_char(' ', cinv);
		} else {
			lcd_char(buf0 + 0x30, cinv);
			(*pidigit)++;
		}
		if (i==4) break;
		iout = iout - buf0 * idiv;
		idiv /= 10;
	}
}

void lcd_clear(void){
	unsigned char i,j;
	wait2ms();
	for(j=0; j<8; j++) {
		lcd_pos(0, j);
		for(i=0; i<128; i++) {lcd_write(0, 1); wait(100);}
	}
}

void lcd_spectrum(unsigned char cDiv, unsigned char cTraj) {
	int ix = 0; int iy = 0;//disp origin
	unsigned char cdisp[NDDIM*3];
	int i, j, k;
	for (i=0; i<NDDIM*3; i++) {cdisp[i] = 0;}
	for (i=0; i<NDDIM; i++) {
		j = ((i - (NDDIM>>1)) & (NDDIM-1));
		//f: 0,1,2,...15 -16,-15,-14,...-1
		int iSignal = (abs(idfr[j]) + abs(idfi[j])) >> cDiv;
		if (cTraj) {
			itraj[i] = itraj[i] << 1;
			if (iSignal >= 8) {itraj[i] |= 0x01;}
		}
		for (k=2; k>=0; k--) {
			switch (iSignal) {
				case 0: {cdisp[i + k*NDDIM] = 0x00; break;}
				case 1: {cdisp[i + k*NDDIM] = 0x80; break;}
				case 2: {cdisp[i + k*NDDIM] = 0xC0; break;}
				case 3: {cdisp[i + k*NDDIM] = 0xE0; break;}
				case 4: {cdisp[i + k*NDDIM] = 0xF0; break;}
				case 5: {cdisp[i + k*NDDIM] = 0xF8; break;}
				case 6: {cdisp[i + k*NDDIM] = 0xFC; break;}
				case 7: {cdisp[i + k*NDDIM] = 0xFE; break;}
				default: {cdisp[i + k*NDDIM] = 0xFF; break;}
			}
			iSignal -= 8;
			if (iSignal < 0) break;
		}
	}
	for (k=0; k<=2; k++) {
		lcd_pos(ix, iy + k);
		for (i=0; i<NDDIM; i++) {
			int idata = cdisp[i + k * NDDIM];
			if (k == 0) {//disp arrow head
				switch (i) {
					case (62+AFOUT): {idata |= 0x01; break;}
					case (63+AFOUT): {idata |= 0x03; break;}
					case (64+AFOUT): {idata |= 0x07; break;}
					case (65+AFOUT): {idata |= 0x03; break;}
					case (66+AFOUT): {idata |= 0x01; break;}
				}
			}
			lcd_write(idata, 1);
		}
	}
}

void lcd_traj(void) {
	int ix = 0; int iy = 3;//disp origin
	int i;
	lcd_pos(ix, iy);
	for (i=0; i<NDDIM; i++) {
		unsigned char cbit = (((itraj[i] & 0x3f) << 2) | 0x01);
		lcd_write(cbit, 1);
		//lcd_write(cbit, 1);//2 pixels per div
	}
	lcd_pos(ix, iy+1);
	for (i=0; i<NDDIM; i++) {
		const unsigned char cbit = (itraj[i] >> 6) & 0xff;
		lcd_write(cbit, 1);
		//lcd_write(cbit, 1);//2 pixels per div
	}
}

#define VOLMAX 16
void dispParam(int iMode, int iAtt, char cDemod) {
	char cinv;
	//volume
	lcd_pos(0,5);
	lcd_string("VOL", 0);
	cinv = (iMode == 0) ? 1 : 0;
	lcd_printDec2(VOLMAX+1 - iAtt, cinv);
	//Demodulation
	lcd_pos(32,5);
	cinv = (iMode == 1) ? 1 : 0;
	switch (cDemod) {
		case 0: {lcd_string("AM", cinv); break;}
		case 1: {lcd_string("FM", cinv); break;}
	}
	lcd_pos(108,5);
	lcd_string(";", 0);//phi
	cinv = (iMode == 9) ? 1 : 0;
	lcd_printDec2(iphase_offset, cinv);
}

const unsigned char ccharKHz[12] = 
	{0xFE, 0x20, 0xD0, 0x00, 0xFE, 0x10, 0xFE, 0x00, 0x90, 0xD0, 0xB0, 0x90};

void dispFreq(double dfreq, char csel) {
	dfreq += AFOUT * FRQSTEP;
	int ifreq = (long)dfreq;
	lcd_pos(46,5);
	int idigit;
	lcd_printDec5(ifreq, &idigit, csel);
	lcd_char(0x40, 0);//decimal point
	char cinv = (csel == 5) ? 1 : 0;
	int ifrac1 = (int)((dfreq - ifreq) * 10);
	lcd_char(ifrac1 + 0x30, cinv);
	cinv = (csel == 6) ? 1 : 0;
	int ifrac2 = (int)((dfreq - ifreq - ifrac1*0.1) * 100);
	lcd_char(ifrac2 + 0x30, cinv);

	//disp "kHz"
	int i;
	for (i=0; i<12; i++) {lcd_write(ccharKHz[i], 1);}
}

unsigned int DACbuffer[1] __attribute__((space(dma)));

void initDAC(void) {
	DACbuffer[0] = 0x8000;
	//init DMA1
	DMA1CONbits.SIZE = 0;//Data transfer size = word
	DMA1CONbits.DIR = 1;//Write to peripheral
	DMA1CONbits.AMODE = 1;	// Configure DMA for Register indirect w/o post increment
	DMA1CONbits.MODE  = 0;	// Configure DMA for Continuous, Ping-Pong mode disabled
	DMA1PAD=(int)&DAC1LDAT;
	DMA1CNT=0;	
	DMA1REQ=79;
	DMA1STA = __builtin_dmaoffset(DACbuffer);
	IFS0bits.DMA1IF = 0;	//Clear the DMA interrupt flag bit
	IEC0bits.DMA1IE = 0;	//Disable DMA interrupt
	DMA1CONbits.CHEN=1;

	//set auxiliary clock for DAC operation
	ACLKCONbits.SELACLK = 0;	//PLL output ==> auxiliary clk source
	ACLKCONbits.APSTSCLR = 7; //auxiliary clk divider: divided by 1

	//init DAC
	DAC1STATbits.LOEN = 1;	//Left channel DAC output enabled
	DAC1STATbits.LITYPE = 1;	//Interrupt if FIFP is empty (interrupt is controlled with DMA1IE)
	DAC1CONbits.AMPON = 0;	//Amplifier is disabled during sleep/idle mode
	DAC1CONbits.DACFDIV = 31;	//Divide clock by 32 (19.52 kHz sampling if Fcy = 40 MHz)
	DAC1CONbits.FORM = 0;	//Data format is unsigned
	DAC1CONbits.DACEN = 1;	//DAC1 module enabled
	DAC1DFLT = 0x8000;
}

#define BIASR 0x0200
#define BIASI 0x0200
#define AVGDIM 256
#define AVGDIMP 8
int iavgr[AVGDIM]; int iavgi[AVGDIM];

#define DACSCL 6
#define DISP_RANGE 5
#define FRQMAX 65000
#define FRQMIN 7
#define NFASTDISP 100
#define DEMODMAX 0

int main (void)
{

// Configure Oscillator to operate the device at 40Mhz
// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
// Fosc= 6M*80/(2*3)=80Mhz for 6M input clock
//	PLLFBD=78;					// M=80
// Fosc= 7.3728M*65/(2*3)=79.872Mhz for FRC 1/1 input clock
	PLLFBD=63;					// M=80
	CLKDIVbits.PLLPOST=0;		// N1=2
	CLKDIVbits.PLLPRE=1;		// N2=3
	OSCTUN=0;					// Tune FRC oscillator, if FRC is used
	CLKDIVbits.FRCDIV = 0;	//FRC divided by 1

// Disable Watch Dog Timer
	RCONbits.SWDTEN=0;

// clock switching to incorporate PLL
	__builtin_write_OSCCONH(0x01);		// Initiate Clock Switch to FRC with PLL (NOSC=0x01)										
//	__builtin_write_OSCCONH(0x03);		// Initiate Clock Switch to Primary Oscillator with PLL (NOSC=0x03)										
	__builtin_write_OSCCONL(OSCCON || 0x01);		// Start clock switching
	while (OSCCONbits.COSC != 0x01);	// Wait for Clock switch to occur
//	while (OSCCONbits.COSC != 0x03);	// Wait for Clock switch to occur
	while(OSCCONbits.LOCK!=1);		// Wait for PLL to lock

//Set ports as digital
	AD1PCFGL=0xFFFF;

//wait for 200 ms
	int i, j;
	for (i=0; i<100; i++) {wait2ms();}

//switches
	TRISBbits.TRISB13 = 1;
	TRISBbits.TRISB12 = 1;
	TRISBbits.TRISB11 = 1;//push sw
	CNPU1bits.CN13PUE = 1;//enable pull ups
	CNPU1bits.CN14PUE = 1;
	CNPU1bits.CN15PUE = 1;

//lcd
	lcd_init();
	lcd_write(0xAE, 0);//LCD off
	lcd_write(0xA0, 0);//ADC normal
	lcd_write(0xC8, 0);//Common output 0xC8:reverse 0xC0:normal(upside down)
	lcd_write(0xA3, 0);//LCD bias 1/7
	lcd_write(0x2C, 0);//power ctrl mode=4
	wait2ms();
	lcd_write(0x2E, 0);//power ctrl mode=6
	wait2ms();
	lcd_write(0x2F, 0);//power ctrl mode=7
	lcd_write(0x23, 0);//resister ratio 3
	lcd_write(0x81, 0);//set Vo volume
	lcd_write(0x1C, 0);// volume=0x1c
	lcd_write(0xA4, 0);//A4: not all-on; A5: all dots on
	lcd_write(0x40, 0);//Display start line = 0
	lcd_write(0xA6, 0);//Display polarity = normal
	lcd_write(0xAF, 0);//LCD on
	wait2ms();
	lcd_clear();

//x axis
	lcd_pos(0,3);
	for (i=0; i<NDDIM; i++) {lcd_write(0x01, 1);}

//local osc
	osc_init();//init RB6-10 and reset DDS units
	osc_set(10.0, 0);

//disp
	unsigned int iMode = 0;
	char cDemod = 0;//AM
	int iAtt = 11;//11;
	double dfreq = 1100 - AFOUT * FRQSTEP;
	dispParam(iMode, iAtt, cDemod);
	dispFreq(dfreq, iMode-2);

	for (i=0; i<NDIM; i++) {itraj[i] = 0;}
	for (i=0; i<AVGDIM; i++) {iavgr[i] = BIASR; iavgi[i] = BIASI;}
	FFTinit();
	FFTDinit();

	unsigned int icyc = 0;
	unsigned int icyc2 = 0;
	unsigned int iPrevRB11 = 1;
	unsigned int iPrevRB12 = 1;
	int ibiasr = BIASR;
	int ibiasi = BIASI;
	int idisp = 0;

	initDma0();	// Initialise the DMA controller to buffer ADC data
   	initAdc1();	// Initialize the A/D converter
	initDAC();
	osc_initphase();
	osc_set(dfreq, 8);
	dispFreq(dfreq, iMode-2);
	dispParam(iMode, iAtt, cDemod);

	while (1) {
		icyc++;
		int iADCdiv = iAtt >> 1;

		//FFT
		i = DMA0CNT >> 1;
		j=NDIM-1;
		while (j >= 0) {
			int isumr = 0; int isumi = 0; int k;
			for (k=0; k<8; k++) {
				isumr += BufferA[i*2+1];
				isumi += BufferA[i*2];
				i--; if (i<0) i=NDDIM-1;
			}
			ifr[j] = ((isumr >> 3) - ibiasr) >> iADCdiv;
			ifi[j] = ((isumi >> 3) - ibiasi) >> iADCdiv;
			if (iAtt & 0x01) {
				ifr[j] = (ifr[j] * 11) >> 4;//*0.7
				ifi[j] = (ifi[j] * 11) >> 4;//*0.7
			}
			j--;
		}
		FFT();

		//audio
		int iamp = (int)sqrt((float)(ifr[AFOUT]) * ifr[AFOUT] + (float)ifi[AFOUT] * ifi[AFOUT]);
		DACbuffer[0] = iamp << DACSCL;
//		DACbuffer[0] = (icyc & 0xf) << 12;//test code to determine freq

		//switches
		if ((icyc & 0xff) == 0) {
			if (PORTBbits.RB11 == 0) {//push sw
				if (iPrevRB11) {
					idisp = 0;
					iMode++;
					if (iMode > 9) iMode = 0;
					dispParam(iMode, iAtt, cDemod);
					dispFreq(dfreq, iMode-2);
				}
				iPrevRB11 = 0;
			} else {
				iPrevRB11 = 1;
			}
		}
		if ((icyc & 0xf) == 0) {
			if (PORTBbits.RB12 == 0) {//rotary encoder
				if (iPrevRB12) {
					idisp = 0;
					if (PORTBbits.RB13) {
						switch (iMode) {
							case 0: {if (iAtt < VOLMAX) iAtt++; break;}
							case 1: {if (cDemod > 0) cDemod--; else cDemod=DEMODMAX; break;}
							case 2: {if (dfreq-10000 >= FRQMIN) dfreq-=10000; break;}
							case 3: {if (dfreq-1000 >= FRQMIN) dfreq-=1000; break;}
							case 4: {if (dfreq-100 >= FRQMIN) dfreq-=100; break;}
							case 5: {if (dfreq-10 >= FRQMIN) dfreq-=10; break;}
							case 6: {if (dfreq-1 >= FRQMIN) dfreq-=1; break;}
							case 7: {if (dfreq-0.1 >= FRQMIN) dfreq-=0.1; break;}
							case 8: {if (dfreq-0.01 >= FRQMIN) dfreq-=0.01; break;}
							case 9: {iphase_offset = (iphase_offset+1)&0x1f; break;}
						}
					} else {
						switch (iMode) {
							case 0: {if (iAtt > 0) iAtt--; break;}
							case 1: {if (cDemod < DEMODMAX) cDemod++; else cDemod=0; break;}
							case 2: {if (dfreq+10000 <= FRQMAX) dfreq+=10000; break;}
							case 3: {if (dfreq+1000 <= FRQMAX) dfreq+=1000; break;}
							case 4: {if (dfreq+100 <= FRQMAX) dfreq+=100; break;}
							case 5: {if (dfreq+10 <= FRQMAX) dfreq+=10; break;}
							case 6: {if (dfreq+1 <= FRQMAX) dfreq+=1; break;}
							case 7: {if (dfreq+0.1 <= FRQMAX) dfreq+=0.1; break;}
							case 8: {if (dfreq+0.01 <= FRQMAX) dfreq+=0.01; break;}
							case 9: {iphase_offset = (iphase_offset-1)&0x1f; break;}
						}
					}
					if (iMode <= 1) {dispParam(iMode, iAtt, cDemod);}
					else if (iMode == 9) {osc_set(dfreq, 8); dispParam(iMode, iAtt, cDemod);}
					else {osc_set(dfreq, 8); dispFreq(dfreq, iMode-2);}
				}
				iPrevRB12 = 0;
			} else {
				iPrevRB12 = 1;
			}
		}

		//disp interval
		if (idisp >= NFASTDISP) {
			if (icyc & 0xfff) continue;
		} else {
			if (icyc & 0x3ff) continue;//high disp freq
		}
		if (idisp < NFASTDISP) idisp++;

		//adjust DC level
		icyc2++;
		j = icyc2 & (AVGDIM - 1);
		long lavgr = 0; long lavgi = 0;
		for (i=0; i<NDDIM; i++) {
			lavgr += BufferA[i*2+1]; lavgi += BufferA[i*2];
		}
		iavgr[j] = lavgr >> NDDIMP; iavgi[j] = lavgi >> NDDIMP;
		long lbiasr = 0; long lbiasi = 0;
		for (i=0; i<AVGDIM; i++) {lbiasr += iavgr[i]; lbiasi += iavgi[i];}
		ibiasr = lbiasr >> AVGDIMP; ibiasi = lbiasi >> AVGDIMP;

		//show spectrum and trajectory
		j=NDDIM-1;
		i = DMA0CNT >> 1;
		while (j >= 0) {
			idfr[j] = (BufferA[i*2+1] - ibiasr) >> iADCdiv;
			idfi[j] = (BufferA[i*2] - ibiasi) >> iADCdiv;
			if (iAtt & 0x01) {
				idfr[j] = (idfr[j] * 11) >> 4;
				idfi[j] = (idfi[j] * 11) >> 4;
			}
			i--; if (i<0) i=NDDIM-1;
			j--;
		}
		FFTD();
		unsigned char ctraj = ((icyc & 0x03fff) == 0);
		lcd_spectrum(DISP_RANGE, ctraj);
		if (ctraj) {lcd_traj();}
	}
    return 0;
}

