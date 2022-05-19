/*
 * File:   Final_Control_Version_Final.c
 * Author: Trinidad Burs
 *
 * Created on 9 de febrero de 2021, 9:44
 */

#include <xc.h>
#include <time.h>
#include <stdio.h> //Para usar sprintf()
#include "config.h"
#define _XTAL_FREQ 16000000 //En Hz
#include "UART.h"
#define ADC_MODE 8 //El modo de mi ADC es de 8 bits
#define PWM_MODE 8 //El modo de mi PWM es de 8 bits
                   // Se eligieron estos modos ya que para completar el registro CCPR1L no se necesitara adaptar el valor
#define PWM_freq 10000  //en Hz
#define TMR2PRESCALE 4
#define REF_VALUE 51 // Al trabajar con el ADC en el modo de 8bits, 5V equivalen a 255 cuentas
                     // Luego si mi tension de referencia es 1V, esto equivale a 51 cuentas


//Variables para implementar mi compensador
volatile float X[2]       = {0}; // [ X(n) X(n-1)]
volatile float Y[2]       = {0}; // [ Y(n) Y(n-1)]
volatile float coef_ec[3] = { 4.3, -4.2269, 1 }; // [ coef1->X(n) coef2->X(n-1) coef3->Y(n-1) ]

//Variables auxiliares necesarias para la ejecuacion del programa
volatile char num_as_string[6] = { 0, 0, 0, '\r', '\n', '\0' };  //Esta variabl me servira para convertir un numero de 3 digitos en un string y asi imprimirlo por la UART
int start,stop,count;
char string[25];

// Declaracion funciones para configurar los registros del PIC18F4620
void PORT_configurar();
void ADC_configurar();
void turn_on_ADC();
void PWM_CCP1_configurar();
void turn_on_TMR2();
void PWM_CCP1_set_duty(float val);
void TMR0_configurar();
void initialize_TMR0();
void turn_on_TMR0();
void INTERRUPT_configurar();
void TMR1_configurar();


// Declaracion funciones complementarias
void descargar_Capacitores();
void int_2_str(int num);

//Implementacion del compensador digital
void compensador(int lectura_ADC){
    Y[0] = coef_ec[0] * X[0] + coef_ec[1] * X[1] + coef_ec[2] * Y[1];
    X[1] = X[0];
    X[0] = REF_VALUE - lectura_ADC;
    Y[1] = Y[0];   
}

void main(void) {
    PORT_configurar();
    UART_Init(9600);
    UART_Write_Text("Starting device... \r\n");
    TMR0_configurar();
    initialize_TMR0();
    ADC_configurar();
    turn_on_TMR2();
    PWM_CCP1_configurar();
    UART_Write_Text("Descargando capacitores...\r\n");
    descargar_Capacitores();
    UART_Write_Text("Termino la descarga de los capacitores!\r\n");
    turn_on_ADC();
    turn_on_TMR0();
    UART_Write_Text("Comienza la rutina de interrupcion...\r\n");
    INTERRUPT_configurar();
    while(1);
    return;
}


void __interrupt() ISR(){
    INTCONbits.GIE=0;
    initialize_TMR0();
    ADCON0bits.GO_nDONE=1;  //GO/DONE: A/D Conversion Status bit
                            //  1 = A/D conversion cycle in progress. Setting this bit starts an A/D conversion cycle.
                            //  This bit is automatically cleared by hardware when the A/D conversion has completed.
    while(ADCON0bits.GO_nDONE==1);
    compensador(ADRESH);
    PWM_CCP1_set_duty(Y[0]);
    int_2_str(ADRESH);
    UART_Write_Text(num_as_string);
    INTCONbits.TMR0IF = 0;
    INTCONbits.GIE = 1;
}

// Funciones complementarias
void descargar_Capacitores(){
    PWM_CCP1_set_duty(0);
    __delay_ms(10000);
}
void int_2_str(int num){
    /* Esta funcion convertira un numero de 3 digitos en un string para su impresion por la UART */
    /* La UART lo enviara y sera interpretado como codigo ASCII, luego por la posicion que ocupan los numeros en la tabla ASCII debo sumarle 48 */
    *(num_as_string+2) = ((num%10)+48);
    *(num_as_string+1) = (((num%100)/10)+48);
    *(num_as_string+0) = ((num/100)+48);   
}


// Funciones para configurar los registros del PIC18F4620
void PORT_configurar(){
    TRISAbits.RA0 = 1; //Pin que leera la salida de la planta
    TRISCbits.RC7 = 1; //El pin RX como entrada
    TRISCbits.RC6 = 0; //El pin TX como salida
    TRISCbits.RC2 = 0; //CCP1 as output to use PWM;
}
void ADC_configurar(){
    ADCON0bits.CHS   = 0; //Selecciono el canal AN0
    ADCON1bits.VCFG0 = 0; /*Voltage Reference Configuration bit (V REF + source)
                            1 = V REF + (AN3)
                            0 = V DD
                         */
    ADCON1bits.VCFG1 = 0; /*Voltage Reference Configuration bit (V REF - source)
                            1 = V REF - (AN2)
                            0 = V SS
                         */
    ADCON1bits.PCFG  = 0b1000; //PCFG3:PCFG0: A/D Port Configuration Control bits:
                            //Con ese numero que puse habilito la funcion analogica del AN6 y de todos los ANx anteriores.
    ADCON2bits.ADFM  = 0; //ADFM: A/D Conversion Result Format Select bit: 0 = Left justified
    ADCON2bits.ACQT  = 0b111; /*A/D Acquisition time select bits. Acquisition time is the duration that the A/D charge
                                holding capacitor remains connected to A/D channel from the instant the GO/DONE bit is set until
                                conversions begins.
                                    111 = 20 T AD
                            Por las dudas le doy el maximo */
    ADCON2bits.ADCS  = 0b101;  //Le puse que dependa de Fosc basandome en la Tabla 19-1
                              /* bit         2-0 ADCS<2:0>: A/D Conversion Clock Select bits
                                 */
}
void turn_on_ADC(){
    ADCON0bits.ADON = 1;
}
void PWM_CCP1_configurar(){
    CCP1CON=0b00001100; //PWM mode for CCP1
    T2CONbits.T2CKPS=0b01;; //Prescaler of TMR2 as 4
    PR2=((int)((_XTAL_FREQ/(PWM_freq*4.0*TMR2PRESCALE))-1)); //To make the PWM work at PWM_freq
}
void turn_on_TMR2(){
    T2CONbits.TMR2ON = 1; //Timer2 is on
}
void PWM_CCP1_set_duty(float val){
    CCPR1L = val * 99.0/255.0;// Se considera que el valor de entrada esta en el rango [0:255]
                              // Este valor sera la salida del compensador Y[0]
                              // PR2 tiene un valor de 99 luego debo adaptar el duty en funcion de este valor
}
void TMR0_configurar(){
    T0CONbits.T08BIT = 0; // 0 = Timer0 is configured as a 16-bit timer/counter
    T0CONbits.T0CS   = 0; // 0 = Internal instruction cycle clock (CLKO)
    T0CONbits.PSA    = 0; // 0 = Timer0 prescaler is assigned. Timer0 clock input comes from prescaler output.
    T0CONbits.T0PS   = 0; // 000 = 1:2 Prescale value
}
void initialize_TMR0(){
    /* Para obtener estos valores se utilizo la siguiente formula */
    /* Tdesado = 4/_XTAL_FREQ * PRESCALER_TMR0 * ((2^16)-1-TMR0) */
    /* Quiero que se genere una interrupcion cada 0.017 segundos, a su vez viendo la figura 11-2 del manual veo que el TMR me introduce un delay de 2 Tcy*/
    /* Entonces se deduce que Tdesado sera: 0.017 seg - 2* Tcy; Tcy = 4/_XTAL_FREQ*/
    /* Despejo y obtengo TMR0 = 31536*/
    TMR0L = 31536 & 255; //Esto truncara mi valor (en binario) y se quedara con los 8 bits menos significativos
    TMR0H = 31536 >> 8; //Con esto me quedo con los 8 bits mas significativos
}
void turn_on_TMR0(){
    T0CONbits.TMR0ON = 1;
}
void INTERRUPT_configurar(){
    INTCONbits.GIE    = 1;
    INTCONbits.PEIE   = 1;
    INTCONbits.TMR0IF = 0; //Por las dudas
    INTCONbits.TMR0IE = 1; // Enables the TMR0 overflow interrupt
    PIE1bits.ADIE     = 0; // Disables the A/D interrupt
    PIE1bits.RCIE     = 0; // Disables Receive Interrupt
    PIE1bits.TXIE     = 0; // Disables Transmit Interrupt
    PIE1bits.TMR2IE   = 0; // Disables TMR2 Interrupt
}
void TMR1_configurar(){
    //Esta funcion se hizo con el objetivo de medir cuanto tiempo tarda en ejecutarse el codigo dentro de la ISR
    // Si se quiere medir se debe realizar lo siguiente:
    /*
     1°) Agregar dentro del "main()", arriba de "INTERRUPT_configurar();" :
     *     TMR1_configurar();
     2°) Agrear como primera linea de la rutina de interrupcion:
     *     start = TMR1;
     3°) Agrear al final de la rutina de interrupcion:
     *     stop = TMR1;
     *     INTCONbits.GIE = 0;
     *     count = stop - start;
     *     sprintf(string,"Count:%d\n\r",count);
     *     UART_Write_Text(string);
     *     INTCONbits.GIE = 1;
     
     
     SE RECOMIENDA FERVIENTEMENTE NO PONER ESTAS LINEAS SALVO DE REALMENTE SE REQUIERA O SE DESEE HACER UNA PRUEBA
     YA QUE CON ELLAS EL SISTEMA NO FUNCIONARA COMO CORRESPONDE. SI SE QUIEREN DEJAR ESTAS LINEAS, SE RECOMIENDA
     NO UTILIZAR "SPRINTF" YA QUE ESTO HARA QUE EL CODIGO DENTRO DE LA RUTINA TARDE MAS EN EJECUTARSE
     QUE EL TIEMPO QUE TARDAN EN DISPARARSE LAS INTERRUPCIONES (POR ESO FUE NECESARIO INCLUIR: "INTCONbits.GIE = 0;"
     E "INTCONbits.GIE = 1;" EN EL PASO 3)
     
     */
    T1CONbits.RD16    = 0; // 0 = Enables register read/write of Timer1 in two 8-bit operations
    T1CONbits.T1CKPS  = 0b01; // 01 = 1:2 Prescale value
    T1CONbits.T1OSCEN = 0; // 0 = Timer1 oscillator is shut off
    T1CONbits.TMR1CS  = 0; // 0 = Internal clock (FOSC/4)
    PIE1bits.TMR1IE   = 0;
    T1CONbits.TMR1ON  = 1; 
    // A diferencia del TMR0, el TMR1 tiene la opcion de leer sus registros TMR1L y TMR1H al mismo tiempo (cuando RD16    = 0)
    // start = TMR1;
    // Por el contrario para leer el valor del TMR0 en su modo de 16 bits tengo que hacerlo de la siguiente manera asi me aseguro de leer primero el que corresponde
    // start = TMR0L; --> We must guarentee to read TMR0H after TMR0L (see datasheet)
    // start|= TMR0H << 8;
}