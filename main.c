/*
 * File:   main.c
 * Author: ADMIN
 *
 * Created on June 21, 2017, 12:04 AM
 */


#include "xc.h"
#include "p24FV32KA304.h"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#define FCY 16000000UL                      // FCY = FOSC/2
#include <libpic30.h>

_FICD( ICS_PGx1 )                           // keeping PGD1 and PGC1 as programming pins
_FOSC( POSCMOD_NONE )                       // Disabling Primary OScillator Mode
_FOSCSEL( FNOSC_FRCPLL )                    // using internal FRC oscillator + PLL for main clock
_FWDT( FWDTEN_OFF )                         // Disabling WDT

//Interrupt Control Definitions
#define disAnalogAtINT1 ANSBbits.ANSB14     // 0 to disable
#define Int1EdgeSelect INTCON2bits.INT1EP   // 0 for Positive Edge, 1 for Negative Edge
#define Int0EdgeSelect INTCON2bits.INT0EP   // 0 for Positive Edge, 1 for Negative Edge
#define Int1Flag IFS1bits.INT1IF            // Sets when Interrupt Occurs
#define Int0Flag IFS0bits.INT0IF            // Sets when Interrupt Occurs
#define Int1Enable IEC1bits.INT1IE          // 1 for Enable
#define Int0Enable IEC0bits.INT0IE          // 1 for Enable
#define Int0Priority IPC0bits.INT0IP        // 0b001 for Priority 1
#define Int1Priority IPC5bits.INT1IP        // 0b001 for Priority 1

//uart definitions
#define uartEnable U1MODEbits.UARTEN        // 1 = Enable Uart
#define uartIdleEnable U1MODEbits.USIDL     // 0 = continue running when device in Idle Mode
#define uartBRGH U1MODEbits.BRGH            // 0 for Normal Mode, 1 for High-Speed Mode
#define uartPDSEL U1MODEbits.PDSEL          // 0b00 for 8-bit data, No parity
#define uartStopSel U1MODEbits.STSEL        // 0 = 1 stop bits
#define uartTransmitEnable U1STAbits.UTXEN  // 1 for Enabling TX
#define uartTransBufFull U1STAbits.UTXBF    // 1 if Transmit Buffer is Full , 0 if Empty
#define uartTSREmpty U1STAbits.TRMT         // 1 if Transmit Shift Register is Empty and Transmit Buffer is Empty
#define uartBRGHValue U1BRG


//IO definitions
//OUTPUTS
#define LED LATBbits.LATB13
#define H4_NW LATAbits.LATA4
#define H2_NW LATBbits.LATB4
#define ManAutoMode LATCbits.LATC8
#define BoostFloatMode LATCbits.LATC9
#define SCR_CON1 LATCbits.LATC3
#define SCR_CON2 LATAbits.LATA9
#define UVLED LATBbits.LATB10
#define OVLED LATBbits.LATB11
#define OCLED LATBbits.LATB12
#define CONT1 LATAbits.LATA10
#define EN LATCbits.LATC4

//INPUTS
#define H8 PORTBbits.RB5
//#define LED PORTAbits.RA7
//#define H4_NW PORTAbits.RA4
//#define H2_NW PORTBbits.RB4
#define H4 PORTAbits.RA11
#define H2 PORTAbits.RA7
#define H1 PORTAbits.RA8
#define R_AUTO PORTBbits.RB6
#define R_BOOST PORTBbits.RB8
#define R_FLOAT PORTBbits.RB9

//ANALOG INPUTS
uint16_t V_BATT_PIN = 4;
uint16_t I_BATT_PIN = 5;
uint16_t I_LOAD_PIN = 6;
uint16_t BBC_PIN = 7;
uint16_t BVL_PIN = 9;
uint16_t FVL_PIN = 8;
uint16_t I_BATT_VREF_PIN = 13;
uint16_t I_LOAD_VREF_PIN = 14;


//TRISbits
#define LEDTRIS TRISBbits.TRISB13
#define ManAutoModeTRIS TRISCbits.TRISC8
#define BoostFloatModeTRIS TRISCbits.TRISC9
#define SCR_CON1TRIS TRISCbits.TRISC3
#define SCR_CON2TRIS TRISAbits.TRISA9
#define UVLEDTRIS TRISBbits.TRISB10
#define OVLEDTRIS TRISBbits.TRISB11
#define OCLEDTRIS TRISBbits.TRISB12
#define CONT1TRIS TRISAbits.TRISA10
#define ENTRIS TRISCbits.TRISC4
#define H8TRIS TRISBbits.TRISB5
#define H4TRIS_NW TRISAbits.TRISA4
#define H2TRIS_NW TRISBbits.TRISB4
#define H4TRIS TRISAbits.TRISA11
#define H2TRIS TRISAbits.TRISA7
#define H1TRIS TRISAbits.TRISA8
#define R_AUTOTRIS TRISBbits.TRISB6
#define R_BOOSTTRIS TRISBbits.TRISB8
#define R_FLOATTRIS TRISBbits.TRISB9

//GLOBAL VARIABLES
char res0[20]="";
uint16_t delayCount = 9000, Xcount = 0;
float FVL_sense, vref_sense, ibatt_sense, Scaling = 1.0;
float alphaMin = 20.0, alphaMax = 118.80;
float SUM = 0.0;
float potValue, V_BATT, I_BATT, SCALINGx = 33.0709, I_BATT_VREF, I_LOAD , I_LOAD_VREF;
uint16_t delayValue = 6000;
float alphaValue;
float FVL_REF, BVL_REF;
float alphaNew, alphaOld , alphaX = 0.5;
//float SCALINGx = 10.0;
float FVL_REF_CENTER = 121.0;
float FLOAT_IBATT_LIMIT = 3.0;
float BBC_LIMIT_MAX = 20.0;
float BOOST_VBATT_LIMIT =115.5;
float BOOST_VCHARGER_LIMIT = 129.25;

unsigned long pulseWidthUsec = 500;

uint16_t cycleCount=0;

//FLAGS
bool softStart_Flag = 0;
bool FirstIteration_Flag = 1;
bool ModeChange_Flag = 1;
bool TEST3_Flag = 0, TEST4_Flag= 0, TEST5_Flag = 0, TEST6_Flag = 0;

uint16_t ADCRead(uint16_t channel);
float GET_MEAN(uint16_t channel, uint16_t sampleSize);
void sysClock(void);
void Interrupt_Init(void);
void ADCInit(void);
void uart_Init(void);
void IO_Init(void);
void UART_print(char *var);
void floatToString_UART_print(float floatingNumber,char * floatString, int decimals);
uint16_t ADCRead(uint16_t channel);
void softStart(void);

int main(void) {
    
    sysClock();
    IO_Init();
    Interrupt_Init();
    ADCInit();
    uart_Init();
    
    while(1){
            
//           while(1){
//               EN=1;
//           if(H8){
//            SCR_CON1 = 1;
//            }
//            else SCR_CON1 = 0;
//            }
//        if(H2){
//            LED = 1;
//        }
//        else LED = 0;

//        I_BATT_VREF = GET_MEAN(I_BATT_VREF_PIN , 4);
//        I_BATT = GET_MEAN(I_BATT_PIN , 4);
//        //I_BATT = (I_BATT - I_BATT_VREF)/0.025;
//        I_LOAD_VREF = GET_MEAN(I_LOAD_VREF_PIN , 4);
//        I_LOAD = GET_MEAN(I_LOAD_PIN , 4);
//        
//        UART_print("I_BATT = ");
//        __delay_us(5);
//        floatToString_UART_print(I_BATT, res0, 4);
//        UART_print("\n\r");
//        __delay_us(5);
//        
//        
//        UART_print("I_BATT_VREF = ");
//        __delay_us(5);
//        floatToString_UART_print(I_BATT_VREF, res0, 4);
//        UART_print("\n\r");
//        __delay_us(5);
//        
//        
//        UART_print("I_LOAD = ");
//        __delay_us(5);
//        floatToString_UART_print(I_LOAD, res0, 4);
//        UART_print("\n\r");
//        __delay_us(5);
//        
//        
//        UART_print("I_LOAD_VREF = ");
//        __delay_us(5);
//        floatToString_UART_print(I_LOAD_VREF, res0, 4);
//        UART_print("\n\r");
//        __delay_us(5);
//        
//        
//        
//        __delay_ms(1000);
//        uint16_t OFFsetx = ADCRead(22);
//        __delay_us(100);
//        uint16_t read = ADCRead(I_BATT_PIN);
//        __delay_us(100);
//        
//        UART_print("Offset = ");
//        __delay_us(5);
//        floatToString_UART_print((float)OFFsetx, res0, 4);
//        UART_print("\n\r");
//        __delay_us(5);
//        
//        UART_print("I_BATT_read = ");
//        __delay_us(5);
//        floatToString_UART_print((float)read, res0, 4);
//        UART_print("\n\r");
//        
//        __delay_ms(500);
        
        
        while(H8 && H4 && H1){   // (OFF-OFF-OFF)
            
            if(ModeChange_Flag){
                Int1Enable = 0;
                Int0Enable = 0;
                softStart_Flag = 0;
                TEST3_Flag = 0;
                ModeChange_Flag = 0;
                TEST4_Flag = 0;
                TEST5_Flag = 0;
                TEST6_Flag = 0;
                UART_print("TEST1 : \n\r");
                __delay_ms(1);
            }
            
            ManAutoMode = 0;
            BoostFloatMode = 0;
            UVLED = 0;
            OVLED = 0;
            OCLED = 0;
            LED = 0;
            __delay_ms(500);
            ManAutoMode = 1;
            BoostFloatMode = 1;
            UVLED = 1;
            OVLED = 1;
            OCLED = 1;
            LED = 1;
            __delay_ms(500);
        }
        ModeChange_Flag = 1;
        
        while(H8 && H4 && !H1){     // (OFF-OFF-ON)
            
            if(ModeChange_Flag){
                Int1Enable = 0;
                Int0Enable = 0;
                softStart_Flag = 0;
                TEST3_Flag = 0;
                ModeChange_Flag = 0;
                TEST4_Flag = 0;
                TEST5_Flag = 0;
                TEST6_Flag = 0;
                UART_print("TEST2 : \n\r");
                __delay_ms(1);
            }
            
            if(!R_AUTO){
                ManAutoMode = 1;
                BoostFloatMode = 1;
                UVLED = 1;
                OVLED = 1;
                OCLED = 1;
                LED = 1;
            }
            else if(!R_BOOST){
                ManAutoMode = 0;
                BoostFloatMode = 1;
                UVLED = 1;
                OVLED = 1;
                OCLED = 1;
                LED = 1;
            }
            else if(!R_FLOAT){
                ManAutoMode = 0;
                BoostFloatMode = 0;
                UVLED = 1;
                OVLED = 1;
                OCLED = 1;
                LED = 1;
            }
        }
        ModeChange_Flag = 1;
        
        while(H8 && !H4 && !H1){     // (OFF-ON-ON)
            if(ModeChange_Flag){
                Int1Enable = 0;
                Int0Enable = 0;
                softStart_Flag = 0;
                TEST3_Flag = 1;
                TEST4_Flag = 0;
                TEST5_Flag = 0;
                TEST6_Flag = 0;
                UART_print("TEST3 : \n\r");
                __delay_ms(1);
                Int1Enable = 1;
                Int0Enable = 1;

                ModeChange_Flag = 0;
                
            }
            

            
            
        }
        ModeChange_Flag = 1;
        
        while(H8 && !H4 && H1){     // (OFF-ON-OFF)
         
        LED = 0;
            if(ModeChange_Flag){
                Int1Enable = 0;
                Int0Enable = 0;
                TEST3_Flag = 0;
                ModeChange_Flag = 0;
                softStart_Flag = 0;
                TEST4_Flag = 1;
                TEST5_Flag = 0;
                TEST6_Flag = 0;
                EN = 0;
                UART_print("TEST4 : \n\r");
                __delay_ms(1);
                Int1Enable = 1;
                Int0Enable = 1;
                cycleCount = 0;
            }
            
            Int1Enable = 1;
            Int0Enable = 1;
            potValue = GET_MEAN(FVL_PIN , 4);
            //float potScaled = potSense * Scaling;
            alphaValue = alphaMin + ((potValue/5.0)*(alphaMax - alphaMin));
            delayValue = (int)((alphaValue/180.0)*10000.0);
            
            //V_BATT = GET_MEAN(V_BATT_PIN , 4);
            //V_BATT = (V_BATT * SCALINGx) + 90.0;
            
            //I_BATT_VREF = GET_MEAN(I_BATT_VREF_PIN , 4);
            //I_BATT = GET_MEAN(I_BATT_PIN , 4);
            //I_BATT = (I_BATT - I_BATT_VREF)/0.025;
            //I_BATT = (I_BATT * 5.0);
            

            if(cycleCount > 25){
                
                //if(alphaValue <= 90.0){
                    
//                    potValue = GET_MEAN(FVL_PIN , 4);
                    
//                    floatToString_UART_print(potValue, res0, 4);
//                    UART_print("\n\r");
//                    __delay_us(5);
//                    
                    
//                    //float potScaled = potSense * Scaling;
//                    alphaValue = alphaMin + ((potValue/5.0)*(alphaMax - alphaMin));
//                    delayValue = (int)((alphaValue/180.0)*10000.0);
                    
//                    floatToString_UART_print((float)delayValue, res0, 4);
//                    UART_print("\n\r");
//                    __delay_us(5);
//                    
                    V_BATT = GET_MEAN(V_BATT_PIN , 4);
                    V_BATT = (V_BATT * SCALINGx);
            
                    I_BATT_VREF = GET_MEAN(I_BATT_VREF_PIN , 4);
                    I_BATT = GET_MEAN(I_BATT_PIN , 4);
                    I_BATT = (I_BATT - I_BATT_VREF)/0.02500;
                    //I_BATT = (I_BATT * 5.0);
                    
                    UART_print("V_BATT=");
                    __delay_us(5);
                    floatToString_UART_print(V_BATT, res0, 4);
                    UART_print("\n\r");
                    __delay_us(5);
                    
//                    UART_print("IBATT=");
//                    __delay_us(5);
//                    floatToString_UART_print(I_BATT, res0, 4);
//                    UART_print("\n\r");
//                    __delay_us(5);
                    
                    UART_print("D =");
                    __delay_us(5);
                    floatToString_UART_print(alphaValue, res0, 4);
                    UART_print("\n\r");
                    __delay_us(5);
                    
                   cycleCount = 0; 
                    
              //  }
                
            }
            
            
        }
        ModeChange_Flag = 1;
        
        while(!H8 && !H4 && H1){     // (ON-ON-OFF)
            
            if(ModeChange_Flag){
                Int1Enable = 0;
                Int0Enable = 0;
                softStart_Flag = 0;
                TEST3_Flag = 0;
                ModeChange_Flag = 0;
                TEST4_Flag = 0;
                TEST5_Flag = 1;
                TEST6_Flag = 0;
                
                
                UART_print("TEST5 : \n\r");
                for(Xcount = 0; Xcount<10; Xcount++){
                        ManAutoMode = 0;
                        BoostFloatMode = 0;
                        UVLED = 0;
                        OVLED = 0;
                        OCLED = 0;
                        LED = 0;
                        __delay_ms(500);
                        ManAutoMode = 1;
                        BoostFloatMode = 1;
                        UVLED = 1;
                        OVLED = 1;
                        OCLED = 1;
                        LED = 1;
                        __delay_ms(500);
                }
                softStart();
                cycleCount = 0;
            }
            
            Int1Enable = 1;
            Int0Enable = 1;
            
            if(cycleCount > 25){
                
                //if(alphaValue <= 90.0){
                    
                    V_BATT = GET_MEAN(V_BATT_PIN , 4);
                    V_BATT = (V_BATT * SCALINGx);
            
//                    //I_BATT_VREF = GET_MEAN(I_BATT_VREF_PIN , 4);
//                    I_BATT = GET_MEAN(I_BATT_PIN , 4);
//                    //I_BATT = (I_BATT - I_BATT_VREF)/0.025;
//                    I_BATT = (I_BATT * 5.0);
                    
                    UART_print("V_BATT=");
                    __delay_us(5);
                    floatToString_UART_print(V_BATT, res0, 4);
                    UART_print("\n\r");
                    __delay_us(5);
                    
//                    UART_print("IBATT=");
//                    __delay_us(5);
//                    floatToString_UART_print(I_BATT, res0, 4);
//                    UART_print("\n\r");
//                    __delay_us(5);
                    
//                    UART_print("D =");
//                    __delay_us(5);
//                    floatToString_UART_print(alphaValue, res0, 4);
//                    UART_print("\n\r");
//                    __delay_us(5);
                    
               // }
                cycleCount = 0;
            }
        }
        ModeChange_Flag = 1;
//        while(!H8 && H4 && H1){     // (ON-OFF-OFF)
//                
//                if(ModeChange_Flag){
//                Int1Enable = 0;
//                Int0Enable = 0;
//                TEST3_Flag = 0;
//                ModeChange_Flag = 0;
//                TEST4_Flag = 0;
//                TEST5_Flag = 0;
//                TEST6_Flag = 1;
//                
//                
//                UART_print("TEST6 : VMC\n\r");
//                for(Xcount = 0; Xcount<10; Xcount++){
//                        ManAutoMode = 0;
//                        BoostFloatMode = 0;
//                        UVLED = 0;
//                        OVLED = 0;
//                        OCLED = 0;
//                        LED = 0;
//                        __delay_ms(500);
//                        ManAutoMode = 1;
//                        BoostFloatMode = 1;
//                        UVLED = 1;
//                        OVLED = 1;
//                        OCLED = 1;
//                        LED = 1;
//                        __delay_ms(500);
//                }
//                softStart();
//            }
//            cycleCount = 0;
//            Int1Enable = 1;
//            Int0Enable = 1;
//            FVL_REF = GET_MEAN(FVL_PIN , 4);
//            FVL_REF = FVL_REF_CENTER + SCALING(FVL_REF , FVL_REF_CENTER);
//
//            BVL_REF = GET_MEAN(BVL_PIN , 4);
//            BVL_REF = BOOST_VBATT_LIMIT + SCALING(BVL_REF , BOOST_VBATT_LIMIT);
//            
//            V_BATT = GET_MEAN(V_BATT_PIN , 4);
//            V_BATT = (V_BATT * SCALINGx);
//
//            I_BATT_VREF = GET_MEAN(I_BATT_VREF_PIN , 4);
//            I_BATT = GET_MEAN(I_BATT_PIN , 4);
//            I_BATT = (I_BATT - I_BATT_VREF)/0.02500;
//            
//            if(V_BATT < FVL_REF*0.98){
//                alphaNew = alphaOld - alphaX;
//            }
//        
//            else if((V_BATT > FVL_REF*0.98) && (V_BATT < FVL_REF*1.02) ){
//                alphaNew = alphaOld;
//            }
//        
//            else if(V_BATT > FVL_REF*1.02){
//                alphaNew = alphaOld + alphaX;
//            }   
//        
//            if(alphaNew < 20.0){
//                alphaNew = 20.0;
//            }
//        
//            alphaOld = alphaNew;
//            updateDelay(alphaNew);
//            
//                
//        }
//        
//        ModeChange_Flag = 1;

        
        
        
    }
    
    
    return 0;
}


void sysClock(){
    OSCCONbits.COSC = 0b001;
    OSCCONbits.NOSC = 0b001;
    CLKDIVbits.RCDIV = 0b000;
}

void IO_Init(){
    LEDTRIS = 0;
    ManAutoModeTRIS = 0;
    BoostFloatModeTRIS = 0;
    SCR_CON1TRIS = 0;
    SCR_CON2TRIS = 0;
    UVLEDTRIS = 0;
    OVLEDTRIS = 0;
    OCLEDTRIS = 0;
    CONT1TRIS = 0;
    ENTRIS = 0;
    
    H8TRIS = 1;
    H4TRIS = 1;
    H2TRIS = 1;
    H1TRIS = 1;
    H4TRIS_NW = 0;
    H2TRIS_NW = 0;
    R_AUTOTRIS = 1;
    R_BOOSTTRIS = 1;
    R_FLOATTRIS = 1;
    
    SCR_CON1 = 0;
    SCR_CON2 = 0;
    EN = 0;
    H4_NW = 1;
    H2_NW = 1;
    UVLED = 1;
    OVLED = 1;
    OCLED = 1;
} 

void uart_Init(){
    uartIdleEnable = 0;
    uartBRGH = 0;
    uartPDSEL = 0b00;
    uartStopSel = 0;
    uartBRGHValue = 8;    //25 = 38400 Baudrate,  8 = 115200 Baudrate
    uartEnable = 1;
}

void UART_print(char *var) // for sending string
{
    uartTransmitEnable = 1;
	while(*var)
	{
		while (!uartTSREmpty);             // TX buffer ready?
		U1TXREG = *var++;
	}
    uartTransmitEnable= 0;
	return;
}

void floatToString_UART_print(float floatingNumber,char * floatString, int decimals)
{
    unsigned char tempFloatString[7];
    int tempIndex = 0;
    int index = 0;
    int tempValue = 0;
    unsigned char minusSign[20] = "-";
    
    
    float xfloatingNumber = floatingNumber;

    if (floatingNumber < 0.0)
    {
        //tempFloatString[0] = "-";
        floatingNumber = -floatingNumber;

    }
    
    tempValue = (int)floatingNumber;
    floatingNumber -= tempValue;
    
    
    do
    {
        tempFloatString[tempIndex++] = tempValue%10 + '0';
    }
    while( (tempValue /= 10) > 0);

    tempIndex--;

    while(tempIndex > -1)
    {
        floatString[index++] = tempFloatString[tempIndex--];
    }

    floatString[index] = '.';
    tempValue = (int)(floatingNumber * (float)pow(10,decimals));

    tempIndex = index;

    index += decimals;
    floatString[index + 1] = '\0';

    while(index > tempIndex)
    {
        floatString[index--] = tempValue % 10 + '0';
        tempValue /= 10;
    }
    
    if (xfloatingNumber < 0.0)
    {
        strcat(minusSign,floatString);
        floatString = minusSign;
        
        UART_print(minusSign);
        __delay_us(5);
        return;
    }
    UART_print(floatString);
    __delay_us(5);
    return;
}

void softStart(void){
    softStart_Flag = 1;
    delayCount = 9000;        // starts from 162degrees firing angle
    Int1Enable = 1;
    Int0Enable = 1;
    while(delayCount > 3428); // will end at 60 degree firing angle
    softStart_Flag = 0;
    alphaNew = ((float)delayCount/10000.0)*180.0;
    alphaOld = alphaNew;
}



void ADCInit() {
    AD1CON1bits.ADON = 1;
    AD1CON1bits.MODE12 = 1;
    AD1CON1bits.FORM = 0b00 ;
    AD1CON1bits.SSRC = 0x0 ;
    AD1CON1bits.ASAM = 0 ;
    AD1CON2bits.PVCFG = 0x01 ; // +Vref = External
    AD1CON2bits.NVCFG = 0x0 ;
    AD1CON2bits.SMPI = 0x0 ;
    AD1CON3bits.ADCS = 0x3F ;
    
    ANSBbits.ANSB2 = 1;
    ANSBbits.ANSB3 = 1;
    ANSBbits.ANSB4 = 0;
    ANSCbits.ANSC0 = 1;
    ANSCbits.ANSC1 = 1;
    ANSCbits.ANSC2 = 1;
    ANSBbits.ANSB15 = 1;
    ANSAbits.ANSA2 = 1;
    ANSAbits.ANSA3 = 1;
    
}

uint16_t ADCRead(uint16_t channel)
{
    uint16_t i ;

    AD1CHS = channel ;

    // Get an ADC sample
    AD1CON1bits.SAMP = 1 ;           //Start sampling
    for (i = 0 ; i < 1000 ; i++) ; //Sample delay, conversion start automatically

    AD1CON1bits.SAMP = 0 ;           //Start sampling
 //   for (i = 0 ; i < 1000 ; i++) ; //Sample delay, conversion start automatically

    while (!AD1CON1bits.DONE) ;       //Wait for conversion to complete

    return ADC1BUF0 ;
}

void Interrupt_Init(void){
    disAnalogAtINT1 = 0;
    Int1EdgeSelect = 0;
    Int1Flag = 0;
    Int1Priority = 0b001;
    Int1Enable = 0;
   
    Int0EdgeSelect = 0;
    Int0Flag = 0;
    Int0Priority = 0b001;
    Int0Enable = 0;
}

void __attribute__((__interrupt__, auto_psv )) _ISR _INT1Interrupt (void){ //ZCD+
    Int1Enable = 0;
    Int1Flag = 0;
    
    
    if(softStart_Flag){
        __delay_us(delayCount);
        EN = 1;
        SCR_CON1 = 1;
        __delay_us(pulseWidthUsec);
        SCR_CON1 = 0;
        EN = 0;
        delayCount -= 28;
        Int1Enable = 1;
        return;
    }
    cycleCount += 1;
    if(TEST3_Flag){
        __delay_us(8333);   // 8333uSec = 150degree angle delay
        EN = 1;
        SCR_CON1 = 1;
        __delay_us(pulseWidthUsec);
        SCR_CON1 = 0;
        EN = 0;
        Int1Enable = 1;
        return;
    }
    
    if(TEST4_Flag){
        __delay_us(delayValue); 
        EN = 1;
        SCR_CON1 = 1;
        __delay_us(pulseWidthUsec);
        SCR_CON1 = 0;
        EN = 0;
        Int1Enable = 1;
        return;
    }
    
    __delay_us(delayCount);
    EN = 1;
    SCR_CON1 = 1;
    __delay_us(pulseWidthUsec);
    SCR_CON1 = 0;
    EN = 0;
    //delayCount -= 33;
    Int1Enable = 1;
    return;

}

void __attribute__((__interrupt__, auto_psv )) _ISR _INT0Interrupt (void){ //ZCD-
    Int0Enable = 0;
    Int0Flag = 0;
    
        if(softStart_Flag){
        __delay_us(delayCount);
        EN = 1;
        SCR_CON2 = 1;
        __delay_us(pulseWidthUsec);
        SCR_CON2 = 0;
        EN = 0;
        delayCount -= 28;
        Int0Enable = 1;
        return;
        }
    cycleCount += 1;
    if(TEST3_Flag){
        __delay_us(8333);   // 8333uSec = 150degree angle delay
        EN = 1;
        SCR_CON2 = 1;
        __delay_us(pulseWidthUsec);
        SCR_CON2 = 0;
        EN = 0;
        Int0Enable = 1;
        return;
    }
    
    if(TEST4_Flag){
        __delay_us(delayValue); 
        EN = 1;
        SCR_CON2 = 1;
        __delay_us(pulseWidthUsec);
        SCR_CON2 = 0;
        EN = 0;
        Int0Enable = 1;
        return;
    }
    __delay_us(delayCount);
    EN = 1;
    SCR_CON2 = 1;
    __delay_us(pulseWidthUsec);
    SCR_CON2 = 0;
    EN = 0;
    //delayCount -= 33;
    Int0Enable = 1;
    return;
    
}

float GET_MEAN(uint16_t channel, uint16_t sampleSize){
    SUM = 0.0;
    
    for(Xcount = 1; Xcount <=sampleSize ;Xcount ++){
        uint16_t xvalue = ADCRead(channel);
        float xReal = ((float)xvalue/4095.0)*5.0010;
        SUM = xReal + SUM;
    }
    return (SUM/(float)sampleSize);
}

float SCALING(float REF , float Center){
    float returnValue, xyz;
    xyz = 2.500 - REF;
    
    if(xyz == 0.0)  return 0.0;
        
    else return (-1.00*Center*0.05*xyz/2.500);
    
}

void updateDelay(float alphaValue){
    delayCount = (int)((alphaValue/180.0)*10000.0);
    return;
}