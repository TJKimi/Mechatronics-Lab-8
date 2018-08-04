#include "msp.h"
#include "driverlib.h"
#include "timer_a.h"
#include "stdio.h"
//24 66
int s2;
volatile int PWM;
volatile int OnOffPWM1 = 0;
volatile int direction = 0;
int switchchecker = 0;
//Halt the watchdog timer
//WDT_A_holdTimer();
volatile  float voltage;
volatile float percentageOut;
//Configure timer A to interrupt every one second. Note the possibility of adding a lowpass filter later on in the code.
const Timer_A_UpModeConfig config =
{
 TIMER_A_CLOCKSOURCE_SMCLK,
 TIMER_A_CLOCKSOURCE_DIVIDER_64,
 10000,
 TIMER_A_TAIE_INTERRUPT_ENABLE,
 TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,
 TIMER_A_DO_CLEAR
};

//Configure the analog to digital converter. Note that the ADC should be scaled to (0,2.5), and that it should have 10 bits.

void main(void)
{
    //Initialize Clock
    CS_setDCOFrequency(3E+6);
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    //Initialize Timer A
    Timer_A_configureUpMode(TIMER_A1_BASE,&config);
    Interrupt_enableInterrupt(INT_TA1_N);
    Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_UP_MODE);
    Interrupt_enableMaster();

    //Configuring and running the ADC
    //Configure input pin
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN5,GPIO_TERTIARY_MODULE_FUNCTION);//p5.5 ADC

    //Begin by enabling the module
    ADC14_enableModule();
    //Set the resolution (For this lab, it should be 10 bit)
    ADC14_setResolution(ADC_10BIT);
    //Initiate the module (including clock source and clock rate. Note that I am running the clock quite slowly, to serve as an effective low pass filter...maybe)
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK,ADC_PREDIVIDER_1,ADC_DIVIDER_1,ADC_NOROUTE);
    //Configure Conversion Mode
    ADC14_configureSingleSampleMode(ADC_MEM0,0);
    //allocate register to hold results
    ADC14_configureConversionMemory(ADC_MEM0,ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A0,ADC_NONDIFFERENTIAL_INPUTS);
    //Enable Sample Timer
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    //Enable Conversion
    ADC14_enableConversion();
    //Enable Switch 1 Interrupt
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN1);//p1.1 button 1
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN4);//p1.4 button 2

    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN5);//p1.5
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN5);//1.5

    GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN1,GPIO_LOW_TO_HIGH_TRANSITION);//p1.1 edge detect
    //GPIO_interruptEdgeSelect(GPIO_PORT_P2,GPIO_PIN6,GPIO_LOW_TO_HIGH_TRANSITION);


    //Set Interrupt Priority

    GPIO_clearInterruptFlag(GPIO_PORT_P1,GPIO_PIN1);//1.1
    //GPIO_clearInterruptFlag(GPIO_PORT_P2,GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN1);//1.1
    //GPIO_enableInterrupt(GPIO_PORT_P2,GPIO_PIN6);
    Interrupt_setPriority(INT_PORT1,0);
    //Interrupt_setPriority(INT_PORT2,1);
    Interrupt_setPriority(INT_TA1_N,1);
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableInterrupt(INT_TA1_N);
    //Interrupt_enableInterrupt(INT_PORT2);
    Interrupt_enableMaster();
    P2SEL0 |= 0x10 ; // Set bit 4 of P2SEL0 to enable TA0.1 functionality on P2.4
    P2SEL1 &= ~0x10 ; // Clear bit 4 of P2SEL1 to enable TA0.1 functionality on P2.4
    P2DIR |= 0x10 ; // Set pin 2.4 as an output pin
    P6SEL0 |=0x40;
    P6SEL1 &= ~0x40;
    P6DIR |=0x40;
    // Set Timer A period (PWM signal period)
    TA0CCR0 = 3000 ;
    TA2CCR0 = 3000;
    // Set Duty cycle
    TA0CCR1 = 0;
    TA2CCR3 = 0;
    // Set output mode to Reset/Set
    TA0CCTL1 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
    TA2CCTL3 = OUTMOD_7;
    // Initialize Timer A
    TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0
    TA2CTL = TASSEL__SMCLK | MC__UP | TACLR ;
    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN2);//2.2
    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN1);//2.1
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);//1.0


    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);//2.2
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);//2.1
    GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);//2.0

    //In this lab, there is really no point in running the while loop, as we are not polling.
    while(1){
        s2 = GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN4);
        if(!GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN4)){
            if(!switchchecker){
                if(direction==2){
                    direction = 0;
                    switchchecker = 1;
                }else{
                    direction = 2;
                    switchchecker = 1;
                }
            }
        }else if(GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN4)){
                switchchecker = 0;
        }
        int x;
        for(x=0;x<5000;x++){}

        if(direction==2){
        TA0CCR1 = 0;
        TA2CCR3 = PWM;
        GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN2);//p2.2 turn on blue
        GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);//p2.1 turn off green
        GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);//p1.0 turn off red
        }else if(direction==1){
            TA0CCR1 = PWM;
            TA2CCR3 = 0;
            GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);//p2.2 turn off blue
            GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN1);//p2.1 turn on green
            GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);//p1.0 turn off red
        } else{
            TA0CCR1 = 0;
            TA2CCR3 = 0;
            GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN1);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
        }

    }




}


//Timer A 1 second interrupts
void TA1_N_IRQHandler(void){
    //Initialize/Clear Voltage and Temperature values


    float voltage = 0.0;

    //Clear Counter

    //Clear interrupt Flag
    Timer_A_clearInterruptFlag(TIMER_A1_BASE);

    //Begin the conversion
    ADC14_toggleConversionTrigger();
    //Get The Voltage From the Pot
    while(ADC14_isBusy()){}

    voltage = (float)ADC14_getResult(ADC_MEM0);
    percentageOut = voltage/752;
    PWM = (int)3000*percentageOut;
    //printf("PWM: %i\n", PWM);

}

void PORT1_IRQHandler(void){

    if(direction==1){
           direction = 0;
       }else{
           direction = 1;
       }
   GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1) ;
   volatile int z = 0;
   for(z=0;z<=1000;z++){}
}
