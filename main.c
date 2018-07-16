/*
				DSP BASED DC MOTOR CONTROL
##############################################################################################################
Pin Configuration
1.Motor ON-OFF Switch is connected to GPIO31(CANTX-A)
2.Motor direction control Farward/Reverse is connected to GPIO-34(ECAP1/XREADY)
3.Motor Chopper PWM output HIGH side H1 gate G1 is connected to GPIO-00/EPWM1A
4.Motor Chopper PWM output HIGH side L1 gate G2 is connected to GPIO-00/EPWM1B
5.Motor Chopper PWM output HIGH side H2 gate G3 is connected to GPIO-00/EPWM2A
6.Motor Chopper PWM output HIGH side L2 gate G4 is connected to GPIO-00/EPWM2B
7.Incremental Encoder pulse A is connected to GPIO-20
8.Incremental Encoder Pulse B is connected to GPIO-21
9.Incremental Encoder Pulse Inder is connected to GPIO-23
10.Speed Control Knob
11.Armature Voltage Measurement Using Hall Effect Voltage Sensor
12.Feild Voltage Measurement Using Hall Effect Voltage Sensor
13.Armature Current Measurement Using Hall Effect Current Sensor
14.Field Current Measurement Using Hall Effect Current Sensor

###############################################################################################################
*/

#define DSP28_DIVSEL 	2
#define DSP28_PLLCR 	10
#define AVG        		1000
#define BUF_SIZE   		2048

#define EPWM1_TIMER_TBPRD  4710  // Period register
#define EPWM1_MAX_CMPA     4500
#define EPWM1_MIN_CMPA      400
#define EPWM1_MAX_CMPB     4239
#define EPWM1_MIN_CMPB      400

#define EPWM2_TIMER_TBPRD  4710  // Period register
#define EPWM2_MAX_CMPA     4500
#define EPWM2_MIN_CMPA      400
#define EPWM2_MAX_CMPB     4239
#define EPWM2_MIN_CMPB      400

#define ADC_MIN 			0
#define ADC_MAX				4095
#define TEN_PER_DUTY		0
#define NINTY_PER_DUTY		4239

#define FAR 				GpioDataRegs.GPADAT.bit.GPIO31
#define ON_OFF				GpioDataRegs.GPBDAT.bit.GPIO34

#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0


#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "Example_posspeed.h"
#include <IQmathLib.h>
#include <stdio.h>
#include <math.h>

typedef struct
{
   volatile struct EPWM_REGS *EPwmRegHandle;
   Uint16 EPwm_CMPA_Direction;
   Uint16 EPwm_CMPB_Direction;
   Uint16 EPwmTimerIntCount;
   Uint16 EPwmMaxCMPA;
   Uint16 EPwmMinCMPA;
   Uint16 EPwmMaxCMPB;
   Uint16 EPwmMinCMPB;
}EPWM_INFO;

POSSPEED qep_posspeed=POSSPEED_DEFAULTS;

__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);

interrupt void cpu_timer0_isr(void);
interrupt void prdTick(void);

void update_compare(EPWM_INFO*);

EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;

void periPheralModuleClocks(void);
void GPIOSettings();
void PWMSettings();
void Pll(int val, int divsel);
void disableWatchdog();
void delay_loop1();
void ADCsettings();
void ADCValue();
void delay_loop2();
void initEpwm();					//ENcoder
void init3Epwm();					//ENcoder


long map(long x, long in_min, long in_max, long out_min, long out_max);
float mapvoltage(float x, float in_min, float in_max, float out_min, float out_max);
long dutyPercentage(long x, long in_min, long in_max, long out_min, long out_max);
float mapcurrent(float x, float in_min, float in_max, float out_min, float out_max);

void LCD_init(void);
void LCD_cmd(unsigned char);
void LCD_dat(unsigned char);
void delay_ms(long end);

unsigned char i,temp;
const unsigned char Msg1[] = "16x2 LCD Test...";
const unsigned char Msg2[] = "F2812 EVB BOARD ";

Uint16  adc1=0,adc=0,adc2=0,adc3=0,adc4=0,adc5=0,duty=0,sec=0,en=0;
int16  fars=0,revs=0;
Uint16 t=0;
float Armature_Voltage,Field_voltage,Armature_current,Field_current;
int Duty_Percentage,RPM;

int main(void)
{
	disableWatchdog();																			//Disable Watchdog Timer
	Pll(DSP28_PLLCR,DSP28_DIVSEL);																//Initialize Phased Loced Loop
	periPheralModuleClocks();																	//Initaialze Peripheral Clocks
	GPIOSettings();
	InitEQep1Gpio();																			//For Encoder Initialise GPIO20,GPIO21,GPIO23 for Pulse A,Pulse B and Index signal respectively
	InitEPwm3Gpio();																			//For Encoder Initialise PWM3
	DINT;
	InitPieCtrl();																				//Initialize PieControl
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();																			//Initialize PieVectorControl
	EALLOW;  																					//This is needed to write to EALLOW protected registers
	PieVectTable.EPWM1_INT = &epwm1_isr;														//Map  epwm1_isr routine to EPWM1_INT epwm1 Interrupt
	PieVectTable.EPWM2_INT = &epwm2_isr;														//Map  epwm2_isr routine to EPWM2_INT epwm2 Interrupt
	PieVectTable.TINT0 = &cpu_timer0_isr;														//Map cpu_timer0_isr routine to TINT0 timer0 Interrupt
	PieVectTable.EPWM3_INT= &prdTick;															//Encoder	Map prdTick routine to EPWM3_INT;
	EDIS;
	InitCpuTimers();																			//Intialize CPU TIMERS
	init3Epwm();																				//Initilaze PWM3 for Encoder
	#if (CPU_FRQ_150MHZ)
    ConfigCpuTimer(&CpuTimer0, 150, 1000000);													//Initailize CPUTIMER0 as 150MHZ and period Register
	#endif
	#if (CPU_FRQ_100MHZ)
    ConfigCpuTimer(&CpuTimer0, 100, 500000);
	#endif
    CpuTimer0Regs.TCR.all = 0x4001;
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;														//Disable Timebaseclock Sync
	EDIS;
	PWMSettings();
	EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;														//enable Timebaseclock Sync
    EDIS;
	IER |= M_INT3;
	IER |= M_INT1;
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
	PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
	PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	EINT;   // Enable Global interrupt INTM
    ERTM;
	ADCsettings();
	qep_posspeed.init(&qep_posspeed);
 	for(;;)
	{
 		ADCValue();
 		duty			 = map(adc, ADC_MIN, ADC_MAX, TEN_PER_DUTY, NINTY_PER_DUTY);	//To Calculate the duty
 		Duty_Percentage	 = dutyPercentage(duty,471,4239,10,100);						//To Calculate the duty Percentage
 		Armature_Voltage = mapvoltage(adc1, 0.0, 1902.0, 0, 250);						//To Calculate the Armature Voltage
 		Armature_Voltage = floor(Armature_Voltage * 100) / 100;
 		Field_voltage	 = mapvoltage(adc2, 0.0, 2800.0, 0, 250);						//To Calculate the Field Voltage
 		Field_voltage    = floor(Field_voltage * 100) / 100;
 		Field_current	 = adc3/2;
 		Field_current    = floor(Field_current * 100) / 100;							//To Calculate the Field Current
 		Armature_current = adc4/2;
 		Armature_current    = floor(Armature_current * 100) / 100;
 		if(en>3)
 			{
 				RPM=qep_posspeed.SpeedRpm_fr;											//Load the speed of the motor in the variable  RPM
 			}
 		if(ON_OFF == 1)
 		{
			if(FAR == 0)
				{
					if(sec >= 1 && sec <= 10)
					{
						fars=0;
						EPwm1Regs.AQCSFRC.bit.CSFA = 0b10;								// MAke PWM1A as HIGH  so it will be low ***AFTER 15V CONVERSION**
						EPwm2Regs.AQCSFRC.bit.CSFB = 0b10;								// MAke PWM2B as HIGH  so it will be low ***AFTER 15V CONVERSION**
					}
					else
					{
						fars=1;
						EPwm1Regs.AQCSFRC.bit.CSFB = 0b00;								// MAke PWM1B as PWM  so it will be PWM ***AFTER 15V CONVERSION**
						EPwm2Regs.AQCSFRC.bit.CSFA = 0b00;								// MAke PWM1B as PWM  so it will be PWM ***AFTER 15V CONVERSION**
					}
				}
			else if(FAR == 1)
				{
					if(sec >= 1 && sec <= 10)
					{
						revs=0;
						EPwm1Regs.AQCSFRC.bit.CSFB = 0b10;								// MAke PWM1B as HIGH  so it will be low ***AFTER 15V CONVERSION**
						EPwm2Regs.AQCSFRC.bit.CSFA = 0b10;								// MAke PWM2A as HIGH  so it will be low ***AFTER 15V CONVERSION**
					}
					else
					{
						revs=1;
						EPwm1Regs.AQCSFRC.bit.CSFA = 0b00;								// MAke PWM1B as PWM  so it will be PWM ***AFTER 15V CONVERSION**
						EPwm2Regs.AQCSFRC.bit.CSFB = 0b00;								// MAke PWM1B as PWM  so it will be PWM ***AFTER 15V CONVERSION**
					}
				}
 		}
 		else if(ON_OFF == 0)
 		{
 			EPwm1Regs.AQCSFRC.bit.CSFA				=0b10;								// MAke PWM1A as HIGH  so it will be low ***AFTER 15V CONVERSION**
 			EPwm1Regs.AQCSFRC.bit.CSFB				=0b10;								// MAke PWM1B as HIGH  so it will be low ***AFTER 15V CONVERSION**
 			EPwm2Regs.AQCSFRC.bit.CSFA				=0b10;								// MAke PWM2A as HIGH  so it will be low ***AFTER 15V CONVERSION**
 			EPwm2Regs.AQCSFRC.bit.CSFB				=0b10;								// MAke PWM2B as HIGH  so it will be low ***AFTER 15V CONVERSION**
 		}
	}
}

void GPIOSettings()
{
	EALLOW;
	//****************PWM I0 SETTING*****************************//
	GpioCtrlRegs.GPAPUD.bit.GPIO0 		= 0;    // Enable pull-up on GPIO0 (EPWM1A)
	GpioCtrlRegs.GPAPUD.bit.GPIO1 		= 0;    // Enable pull-up on GPIO1 (EPWM1B)
	GpioCtrlRegs.GPAPUD.bit.GPIO2 		= 0;    // Enable pull-up on GPIO2 (EPWM2A)
	GpioCtrlRegs.GPAPUD.bit.GPIO3 		= 0;    // Enable pull-up on GPIO3 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 		= 1;    // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 		= 1;    // Configure GPIO1 as EPWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO2		= 1;    // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3		= 1;    // Configure GPIO3 as EPWM2B
	//****************END OF PWM I0 SETTING*****************************//

	//**   ON/OFF SWITCH & FARWARD REVERSE IO SWITCH      ******//
    GpioCtrlRegs.GPAPUD.bit.GPIO31		= 0;	// Enable pull-up on
    GpioCtrlRegs.GPBPUD.bit.GPIO34		= 0;  	// Enable pull-up on
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 	= 0;    // Configure GPIO31 as GPIO31
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 	= 0; 	// Configure GPIO34 as GPIO34
   // GpioCtrlRegs.GPADIR.bit.GPIO31		= 1;
   // GpioCtrlRegs.GPBDIR.bit.GPIO34		= 1;
	//**END OF ON/OFF SWITCH & FARWARD REVERSE IO SWITCH ******//


  //  GpioCtrlRegs.GPCMUX2.bit.GPIO84		= 0;
  //  GpioCtrlRegs.GPCMUX2.bit.GPIO86		= 0;
  //  GpioCtrlRegs.GPAMUX2.bit.GPIO31		= 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO16     = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO17		= 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO12		= 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO13		= 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14		= 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO15		= 0;

    GpioCtrlRegs.GPADIR.bit.GPIO16		= 1;
    GpioCtrlRegs.GPADIR.bit.GPIO17		= 1;
    GpioCtrlRegs.GPADIR.bit.GPIO12		= 1;
    GpioCtrlRegs.GPADIR.bit.GPIO13		= 1;
    GpioCtrlRegs.GPADIR.bit.GPIO14		= 1;
    GpioCtrlRegs.GPADIR.bit.GPIO15		= 1;

    //GpioDataRegs.GPADAT.bit.GPIO16		= 1;
   // GpioDataRegs.GPADAT.bit.GPIO17		= 1;

    EDIS;
}

void disableWatchdog()
{
  EALLOW;
  SysCtrlRegs.WDCR= 0x0068;
  EDIS;
}

void periPheralModuleClocks(void)
{
	EALLOW;
	/****Peripherl Clk Cont 0 REG ****/
	SysCtrlRegs.PCLKCR0.bit.ECANAENCLK		= 0;		//Disable the ECANAENCLK clock
	SysCtrlRegs.PCLKCR0.bit.ECANBENCLK		= 0;		//Disable the ECANBENCLK clock
	SysCtrlRegs.PCLKCR0.bit.MCBSPAENCLK		= 0;		//Disable the MCBSPAENCLK clock
	SysCtrlRegs.PCLKCR0.bit.MCBSPBENCLK		= 0;		//Disable the MCBSPBENCLK clock
	SysCtrlRegs.PCLKCR0.bit.SCIBENCLK		= 0;		//Disable the SCIBENCLK clock
	SysCtrlRegs.PCLKCR0.bit.SCIAENCLK		= 0;		//Disable the SCIAENCLK clock
	SysCtrlRegs.PCLKCR0.bit.SPIAENCLK		= 0;		//Disable the SPIAENCLK clock
	SysCtrlRegs.PCLKCR0.bit.SCICENCLK		= 0;		//Disable the SCICENCLK clock
	SysCtrlRegs.PCLKCR0.bit.I2CAENCLK		= 0;		//Disable the I2CAENCLKclock
	SysCtrlRegs.PCLKCR0.bit.ADCENCLK		= 1;		//Enable the clock for ADC Clock
	ADC_cal();
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC		= 0;		//Disable the Time base Clock Sync

	/****Peripherl Clk Cont 1 REG ****/
	SysCtrlRegs.PCLKCR1.bit.EQEP2ENCLK		= 1;		//Enable the EQEP2ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK		= 1;		//Enable the EQEP1ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.ECAP6ENCLK		= 1;		//Enable the ECAP6ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.ECAP5ENCLK		= 1;		//Enable the ECAP5ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.ECAP4ENCLK		= 1;		//Enable the ECAP4ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.ECAP3ENCLK		= 1;		//Enable the ECAP3ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.ECAP2ENCLK		= 1;		//Enable the ECAP2ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK		= 1;		//Enable the ECAP1ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK		= 1;		//Enable the EPWM6ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK		= 1;		//Enable the EPWM5ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK		= 1;		//Enable the EPWM4ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK		= 1;		//Enable the EPWM3ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK		= 1;		//Enable the EPWM2ENCLK Clock
	SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK		= 1;		//Enable the EPWM1ENCLK Clock

	/****Peripherl Clk Cont 1 REG ****/
	SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK		= 1;		//Enable the GPIOINENCLK Clock
	SysCtrlRegs.PCLKCR3.bit.XINTFENCLK		= 0;		//Disable the XINTFENCLK clock
	SysCtrlRegs.PCLKCR3.bit.DMAENCLK		= 0;		//Disable the DMAENCLK clock
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER2ENCLK	= 1;		//Enable the CPUTIMER2ENCLK Clock
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER1ENCLK	= 1;		//Enable the CPUTIMER1ENCLK Clock
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK	= 1;		//Enable the CPUTIMER0ENCLK Clock

	/****High Speed Peri Clk Prescaler ****/
	SysCtrlRegs.HISPCP.bit.HSPCLK			=0b011;  //High Speed Clock =SYSCLOCKOUT/6 => 150MHZ/6 => 25 MHZ

	/****Low Speed Peri Clk Prescaler ****/
	SysCtrlRegs.LOSPCP.bit.LSPCLK			=0b000;
	EDIS;
}

void Pll(int val, int divsel)
{
   if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)
   {
      asm("        ESTOP0");
   }
   if (SysCtrlRegs.PLLSTS.bit.DIVSEL != 0)
   {
       EALLOW;
       SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
       EDIS;
   }
   if (SysCtrlRegs.PLLCR.bit.DIV != val)
   {
      EALLOW;
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
      SysCtrlRegs.PLLCR.bit.DIV = val;
      EDIS;
      DisableDog();
      while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1)
      {
          // ServiceDog();
      }
      EALLOW;
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
      EDIS;
    }
	if((divsel == 1)||(divsel == 2))
	{
		EALLOW;
	    SysCtrlRegs.PLLSTS.bit.DIVSEL = divsel;
	    EDIS;
	}
	if(divsel == 3)
	{
		EALLOW;
	    SysCtrlRegs.PLLSTS.bit.DIVSEL = 2;
	    DELAY_US(50L);
	    SysCtrlRegs.PLLSTS.bit.DIVSEL = 3;
	    EDIS;
    }
}

void delay_loop1()
{
    volatile long i;
    for (i = 0; i < 100000000; i++) {}
}

void delay_loop2()
{
    volatile long i;
    for (i = 0; i < 149925000; i++) {}
}

void ADCsettings()
{
	AdcRegs.ADCTRL1.bit.RESET				= 0;
	AdcRegs.ADCTRL1.bit.SUSMOD				= 0; 		//*
	AdcRegs.ADCTRL1.bit.ACQ_PS				= 0xf;   	// S/H width in ADC module periods = 16 ADC clocks
	AdcRegs.ADCTRL1.bit.CPS					= 1;
	AdcRegs.ADCTRL1.bit.CONT_RUN			= 1;
	AdcRegs.ADCTRL1.bit.SEQ_OVRD			= 0; 		//*
	AdcRegs.ADCTRL1.bit.SEQ_CASC			= 1;

	AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ		= 0;
	AdcRegs.ADCTRL2.bit.RST_SEQ1			= 0;
	AdcRegs.ADCTRL2.bit.SOC_SEQ1			= 1;
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1		= 0;
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1		= 0;
	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1		= 0;
	AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1		= 0;
	AdcRegs.ADCTRL2.bit.RST_SEQ2			= 0;
	AdcRegs.ADCTRL2.bit.SOC_SEQ2			= 0;
	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ2		= 0;
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ2		= 0;
	AdcRegs.ADCTRL2.bit.EPWM_SOCB_SEQ2		= 0;

	AdcRegs.ADCTRL3.bit.ADCBGRFDN			= 0b11;
	AdcRegs.ADCTRL3.bit.ADCPWDN				= 1;
	AdcRegs.ADCTRL3.bit.ADCCLKPS			= 0b001;
	AdcRegs.ADCTRL3.bit.SMODE_SEL			= 0;

	AdcRegs.ADCCHSELSEQ1.bit.CONV00 		= 0x0;
	AdcRegs.ADCCHSELSEQ1.bit.CONV01 		= 0x1;
	AdcRegs.ADCCHSELSEQ1.bit.CONV02 		= 0x2;
	AdcRegs.ADCCHSELSEQ1.bit.CONV03 		= 0x3;
	AdcRegs.ADCCHSELSEQ2.bit.CONV04 		= 0x4;
	AdcRegs.ADCCHSELSEQ2.bit.CONV05			= 0x5;

//	AdcRegs.ADCMAXCONV.bit.MAX_CONV2		=
	AdcRegs.ADCMAXCONV.bit.MAX_CONV1		=0x5;

//	AdcRegs.ADCASEQSR.bit.SEQ1_STATE		=
//	AdcRegs.ADCASEQSR.bit.SEQ2_STATE		=
//	AdcRegs.ADCASEQSR.bit.SEQ_CNTR			=

	AdcRegs.ADCREFSEL.bit.REF_SEL			= 0b00;

//	AdcRegs.ADCOFFTRIM.bit.OFFSET_TRIM		=
}

void PWMSettings()
{
   EPwm1Regs.TBCTL.bit.CTRMODE 				= TB_COUNT_UP; 				  // Count up
   EPwm1Regs.TBPRD 							= EPWM1_TIMER_TBPRD;          // Set timer period
   EPwm1Regs.TBCTL.bit.PHSEN 				= TB_DISABLE;    			  // Disable phase loading
   EPwm1Regs.TBPHS.half.TBPHS 				= 0x0000;       			  // Phase is 0
   EPwm1Regs.TBCTR 							= 0x0000;                     // Clear counter
   EPwm1Regs.TBCTL.bit.HSPCLKDIV			= TB_DIV2;                    // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV 				= TB_DIV2;

   EPwm1Regs.CMPCTL.bit.SHDWAMODE			= CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE 			= CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE			= CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE			= CC_CTR_ZERO;

   EPwm1Regs.CMPA.half.CMPA					= EPWM1_MIN_CMPA;             // Set compare A value
   EPwm1Regs.CMPB 							= EPWM1_MIN_CMPB;             // Set Compare B value

   EPwm1Regs.AQCTLA.bit.ZRO 				= AQ_SET;                     // Set PWM1A on Zero
   EPwm1Regs.AQCTLA.bit.CAU 				= AQ_CLEAR;                   // Clear PWM1A on event A, up count

   EPwm1Regs.AQCTLB.bit.ZRO 				= AQ_SET;                     // Set PWM1B on Zero
   EPwm1Regs.AQCTLB.bit.CBU 				= AQ_CLEAR;                   // Clear PWM1B on event B, up count

   EPwm1Regs.ETSEL.bit.INTSEL 				= ET_CTR_ZERO;                // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN 				= 1;               			  // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD 				= ET_3RD;           		  // Generate INT on 3rd event

   epwm1_info.EPwm_CMPA_Direction 			= EPWM_CMP_UP; 				  // Start by increasing CMPA & CMPB
   epwm1_info.EPwm_CMPB_Direction 			= EPWM_CMP_UP;
   epwm1_info.EPwmTimerIntCount				= 0;             			  // Zero the interrupt counter
   epwm1_info.EPwmRegHandle 				= &EPwm1Regs;        		  // Set the pointer to the ePWM module
   epwm1_info.EPwmMaxCMPA		 			= EPWM1_MAX_CMPA;      		  // Setup min/max CMPA/CMPB values
   epwm1_info.EPwmMinCMPA 					= EPWM1_MIN_CMPA;
   epwm1_info.EPwmMaxCMPB 					= EPWM1_MAX_CMPB;
   epwm1_info.EPwmMinCMPB 					= EPWM1_MIN_CMPB;

   EPwm2Regs.TBCTL.bit.CTRMODE 				= TB_COUNT_UP;				   // Count up
   EPwm2Regs.TBPRD 							= EPWM2_TIMER_TBPRD;           // Set timer period
   EPwm2Regs.TBCTL.bit.PHSEN 				= TB_DISABLE;    			   // Disable phase loading
   EPwm2Regs.TBPHS.half.TBPHS 				= 0x0000;       			   // Phase is 0
   EPwm2Regs.TBCTR							= 0x0000;                      // Clear counter
   EPwm2Regs.TBCTL.bit.HSPCLKDIV 			= TB_DIV2;                     // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV 				= TB_DIV2;

   EPwm2Regs.CMPCTL.bit.SHDWAMODE			= CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE 			= CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE 			= CC_CTR_ZERO;
   EPwm2Regs.CMPCTL.bit.LOADBMODE 			= CC_CTR_ZERO;

   EPwm2Regs.CMPA.half.CMPA					= EPWM2_MIN_CMPA;               // Set compare A value
   EPwm2Regs.CMPB 							= EPWM2_MIN_CMPB;               // Set Compare B value

   EPwm2Regs.AQCTLA.bit.ZRO 				= AQ_SET;                       // Set PWM1A on Zero
   EPwm2Regs.AQCTLA.bit.CAU 				= AQ_CLEAR;                     // Clear PWM2A on event A, up count

   EPwm2Regs.AQCTLB.bit.ZRO 				= AQ_SET;            			// Set PWM2B on Zero
   EPwm2Regs.AQCTLB.bit.CBU 				= AQ_CLEAR;         			// Clear PWM1B on event B, up count

   EPwm2Regs.ETSEL.bit.INTSEL 				= ET_CTR_ZERO;       			// Select INT on Zero event
   EPwm2Regs.ETSEL.bit.INTEN 				= 1;                  			// Enable INT
   EPwm2Regs.ETPS.bit.INTPRD 				= ET_3RD;            			// Generate INT on 3rd event

   epwm2_info.EPwm_CMPA_Direction 			= EPWM_CMP_UP;  				// Start by increasing CMPA & CMPB
   epwm2_info.EPwm_CMPB_Direction 			= EPWM_CMP_UP; 					//
   epwm2_info.EPwmTimerIntCount 			= 0;                			// Zero the interrupt counter
   epwm2_info.EPwmRegHandle 				= &EPwm2Regs;          			// Set the pointer to the ePWM module
   epwm2_info.EPwmMaxCMPA 					= EPWM2_MAX_CMPA;         		// Setup min/max CMPA/CMPB values
   epwm2_info.EPwmMinCMPA 					= EPWM2_MIN_CMPA;
   epwm2_info.EPwmMaxCMPB 					= EPWM2_MAX_CMPB;
   epwm2_info.EPwmMinCMPB 					= EPWM2_MIN_CMPB;
}

void ADCValue()
{
	Uint16 i;
	for ( i=0; i<AVG; i++)
	 	 {
	 	   while (AdcRegs.ADCST.bit.INT_SEQ1== 0) {} 						// Wait for interrupt
	 	   AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;								// Clear interrupt to get more interrupt
	 	   adc  =((AdcRegs.ADCRESULT0>>4) );
	 	   adc1 =((AdcRegs.ADCRESULT1>>4));
	 	   adc2 =((AdcRegs.ADCRESULT2>>4));
	 	   adc3 =((AdcRegs.ADCRESULT3>>4));
	 	   adc4 =((AdcRegs.ADCRESULT4>>4));
	 	   adc5 =((AdcRegs.ADCRESULT5>>4));
	 	 }
}

__interrupt void epwm1_isr(void)
{
   update_compare(&epwm1_info);
   EPwm1Regs.ETCLR.bit.INT = 1;					//ACknowledge the interrupt to get more interrupts
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm2_isr(void)
{
   update_compare(&epwm2_info);
   EPwm2Regs.ETCLR.bit.INT = 1;					//ACknowledge the interrupt to get more interrupts
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

void update_compare(EPWM_INFO *epwm_info)
{
	epwm_info->EPwmRegHandle->CMPA.half.CMPA=duty;	//Load duty in CMPA and CMPB
	epwm_info->EPwmRegHandle->CMPB=duty;
}

interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
   sec++;
   t++;
   en++;
   if(sec > 10)
   	   {
	   sec=0;
   	   }
   if(en > 4)
      {
	   en=0;
      }
    if(FAR == 0 && fars == 0)
   		{
    		t=0;
   		}
    else if(FAR == 1 && revs == 0)
      	{
    		t=0;
      	}
    if(t >= 1000)
    	{
    		t=14;
    	}
}

interrupt void prdTick(void)
{

   qep_posspeed.calc(&qep_posspeed);
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;    //ACknowledge the interrupt to get more interrupts
   EPwm3Regs.ETCLR.bit.INT=1;
}


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapvoltage(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapcurrent(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long dutyPercentage(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


