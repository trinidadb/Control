/*
 * File:   Final_Control_Discreto.c
 * Author: Trinidad Burs
 *
 * Created on 29 de Junio de 2021, 9:44
 */

#include <xc.h>
#include <time.h>
#include <stdio.h> //Para usar sprintf()
#include "config.h"
#define _XTAL_FREQ 16000000 //En Hz
#include "UART.h"

//Matrices para implementar la planta discreta (fast)
volatile double A[4][4]={ {1,0.006141536564009,0.000050448099760,0.000000103339552},
                          {0,0.998883310015838,0.016417874586211,0.000050448099760},
                          {0,-0.000008579608803,1.000588733396460,0.006146174035641},
                          {0,-0.002792155541873,0.191625950338727,1.000588733396460} };
volatile double B[4][1]={ {0.000034315789206},
                          {0.011166899841619},
                          {0.000085796088028},
                          {0.027921555418726} };
volatile double C[4][4]={ {1,0,0,0},
                          {0,1,0,0},
                          {0,0,1,0},
                          {0,0,0,1} };
volatile double D[4][1]={ {0},
                          {0},
                          {0},
                          {0} };

//Matrices para implementar el observador discreto
double Ao[4][4]={ {0.958494770240399,0.104557353964092,0,0},
                  {-0.104557353964092,0.958494770240399,0,0},
                  {0,0,0.946969026732359,0.033741740252987},
                  {0,0,-0.033741740252987,0.946969026732359} };
double Bo[4][5]={ {0.000591844723017,0.041505229759601,-0.098594051805029,0.000870011742836,0},
                  {0.010949975693391,0.104557353964092,0.040084717592603,0.016096464269285,0},
                  {0.000479857171282,0,-0.000047985717128,0.056322793462633,-0.027760556114307},
                  {0.027187200630366,0,-0.002718720063037,0.220245936577298,0.052925404689959} };
double Co[4][4]={ {1,0,0,0},
                  {0,1,0,0},
                  {0,0,1,0},
                  {0,0,0,1} };
double Do[4][5]={ {0,0,0,0,0},
                  {0,0,0,0,0},
                  {0,0,0,0,0},
                  {0,0,0,0,0} };

//Kd (fast)
volatile double Kd[1][4]={-3.294156674042878,-2.651861887633706,17.219724538737417,2.651034711766949};
volatile double nKd[1][4]={3.294156674042878,2.651861887633706,-17.219724538737417,-2.651034711766949}; //Hacer -Kd*Y[n] == +(-Kd)*Y[n]

//Declaraciones de valores de Referencia
double input=-0.658831334808570;//0.2*Nbard_fast
double ref_by_Nbard[1][1]={{-0.658831334808570}};

//X[n],X[n+1],Y[n] y U[n]. Condiciones Iniciales
volatile double X_0[4][1]={{0},{0},{0},{0}};//X[n]
volatile double X_1[4][1]={{0},{0},{0},{0}};//X[n+1]
volatile double Y[4][1]={{0},{0},{0},{0}};
volatile double U[1][1]={{-0.658831334808570}}; //Debo fijarla como matriz ya que debo multiplicarla
volatile double Xo_0[4][1]={{0},{0},{0},{0}};//Xo[n]
volatile double Xo_1[4][1]={{0},{0},{0},{0}};//Xo[n+1]
volatile double Yo[4][1]={{0},{0},{0},{0}};//Yo==X_hat

//Implementacion Sistema
void planta();
void plantaPLUSobservador();
void plantaPLUSobservadorPLUSperturbacion();

//Variables auxiliares necesarias para la ejecuacion del programa
volatile int fM1;
volatile double aux_1[4][1];   // A*X[n]
volatile double aux_2[4][1];   // B*U[n]
volatile double aux_3[1][1];   // -Kd*Y[n] (en el caso de planta()) o -Kd*Yo[n] (en el caso de plantaPLUSobservador())
volatile double inputObs[5][1];// Analogo MUX Simulink
volatile double aux_4[4][1];   // Ao*Xo[n]
volatile double aux_5[4][1];   // Bo*Uo[n]

// Declaracion funciones para configurar los registros del PIC18F4620
void PORT_configurar();
void INTERRUPT_configurar(); //Desactivo todas las interrupciones

// Declaracion funciones complementarias
void matrix_multiply_c1_c1(volatile double M1[][1],volatile double M2[][1],volatile double res[][1]);
void matrix_multiply_c4_c1(volatile double M1[][4],volatile double M2[][1],volatile double res[][1]);
void matrix_multiply_c5_c1(volatile double M1[][5],volatile double M2[][1],volatile double res[][1]);
void matrix_sum_c1(volatile double M1[][1],volatile double M2[][1],volatile double res[][1]);
void sendDataVisualizer();
void sendDataVisualizer24bits();

//Varaible para controlar el numero de muestras
volatile int count=0;

void main(void) {
    int muestras=1400;
    //int muestras=3000;
    PORT_configurar();
    UART_Init(9600);
    UART_Write_Text("Starting device... \r\n");
    UART_Write_Text("Comienza la rutina de interrupcion...\r\n");
    INTERRUPT_configurar();
    while(count<muestras){
        plantaPLUSobservador();
        //plantaPLUSobservadorPLUSperturbacion();
        sendDataVisualizer();
        //sendDataVisualizer24bits();
        count++;
    };
    return;
}


//Implementacion del sistema digital (fast). Planta realimentada
void planta(){
    
    //X[n+1]=A*X[n]+B*U[n]
    fM1=4;matrix_multiply_c4_c1(A,X_0,aux_1); //A*X[n]
    fM1=4;matrix_multiply_c1_c1(B,U,aux_2); //B*U[n]
    fM1=4;matrix_sum_c1(aux_1,aux_2,X_1);
    
    //Como C es la matriz identidad
    //Si no lo fuese: fM1=4;matrix_multiply_c4_c1(C,X_0,Y);
    Y[0][0]=X_0[0][0];
    Y[1][0]=X_0[1][0];
    Y[2][0]=X_0[2][0];
    Y[3][0]=X_0[3][0];
    
    //X[n]=X[n+1] (para la proxima vez que invoco a la planta)
    X_0[0][0]=X_1[0][0];
    X_0[1][0]=X_1[1][0];
    X_0[2][0]=X_1[2][0];
    X_0[3][0]=X_1[3][0];
    
    //U[n]=r[n]*Nbard-Kc*Y[n]=r[n]*Nbard+(-Kd)*Y[n]
    fM1=1;matrix_multiply_c4_c1(nKd,Y,aux_3); //-Kd*Y[n]
    fM1=1;matrix_sum_c1(ref_by_Nbard,aux_3,U);
   
}
void plantaPLUSobservador(){
    
//PLANTA    
    //X[n+1]=A*X[n]+B*U[n]
    fM1=4;matrix_multiply_c4_c1(A,X_0,aux_1); //A*X[n]
    fM1=4;matrix_multiply_c1_c1(B,U,aux_2); //B*U[n]
    fM1=4;matrix_sum_c1(aux_1,aux_2,X_1);
    
    //Como C es la matriz identidad se puede dar una asignacion directa (reduzco los tiempos de ejecutado)
    //Si no fuese la matriz identidad, debo reemplazar por: fM1=4;matrix_multiply_c4_c1(C,X_0,Y);
    Y[0][0]=X_0[0][0];
    Y[1][0]=X_0[1][0];
    Y[2][0]=X_0[2][0];
    Y[3][0]=X_0[3][0];
    
    //X[n]=X[n+1] (Para la proxima vez que se invoco a la planta)
    X_0[0][0]=X_1[0][0];
    X_0[1][0]=X_1[1][0];
    X_0[2][0]=X_1[2][0];
    X_0[3][0]=X_1[3][0];
 
//OBSERVADOR       
    //"Multiplexo" la entrada al observador para poder usar la representacion del espacio de estados usada en simulink
    inputObs[0][0]=U[0][0];
    inputObs[1][0]=Y[0][0];
    inputObs[2][0]=Y[1][0];
    inputObs[3][0]=Y[2][0];
    inputObs[4][0]=Y[3][0];

    //Xo[n+1]=A*Xo[n]+B*inputObs[n]
    fM1=4;matrix_multiply_c4_c1(Ao,Xo_0,aux_4); //Ao*Xo[n]
    fM1=4;matrix_multiply_c5_c1(Bo,inputObs,aux_5); //Bo*inputObs[n]
    fM1=4;matrix_sum_c1(aux_4,aux_5,Xo_1);
    
    //Como Co es la matriz identidad se puede dar una asignacion directa (reduzco los tiempos de ejecutado)
    //Si no fuese la matriz identidad, debo reemplazar por: fM1=4;matrix_multiply_c4_c1(Co,Xo_0,Yo);
    Yo[0][0]=Xo_0[0][0];
    Yo[1][0]=Xo_0[1][0];
    Yo[2][0]=Xo_0[2][0];
    Yo[3][0]=Xo_0[3][0];
    
    //Xo[n]=Xo[n+1] (Para la proxima vez que se invoco al observador)
    Xo_0[0][0]=Xo_1[0][0];
    Xo_0[1][0]=Xo_1[1][0];
    Xo_0[2][0]=Xo_1[2][0];
    Xo_0[3][0]=Xo_1[3][0];
    
//REALIMENTACION    
    //U[n]=r[n]*Nbard-Kd*Yo[n]=r[n]*Nbard+(-Kd)*Yo[n]
    fM1=1;matrix_multiply_c4_c1(nKd,Yo,aux_3); //-Kd*X[n]
    fM1=1;matrix_sum_c1(ref_by_Nbard,aux_3,U);
}
void plantaPLUSobservadorPLUSperturbacion(){
    
//PLANTA    
    //X[n+1]=A*X[n]+B*U[n]
    fM1=4;matrix_multiply_c4_c1(A,X_0,aux_1); //A*X[n]
    fM1=4;matrix_multiply_c1_c1(B,U,aux_2); //B*U[n]
    fM1=4;matrix_sum_c1(aux_1,aux_2,X_1);
    
    //Como C es la matriz identidad se puede dar una asignacion directa (reduzco los tiempos de ejecutado)
    //Si no fuese la matriz identidad, debo reemplazar por: fM1=4;matrix_multiply_c4_c1(C,X_0,Y);
    Y[0][0]=X_0[0][0];
    Y[1][0]=X_0[1][0];
    Y[2][0]=X_0[2][0];
    Y[3][0]=X_0[3][0];
    
    if (count>1400){
        Y[0][0]+=0.05;
    };
    
    //X[n]=X[n+1] (Para la proxima vez que se invoco a la planta)
    X_0[0][0]=X_1[0][0];
    X_0[1][0]=X_1[1][0];
    X_0[2][0]=X_1[2][0];
    X_0[3][0]=X_1[3][0];
 
//OBSERVADOR       
    //"Multiplexo" la entrada al observador para poder usar la representacion del espacio de estados usada en simulink
    inputObs[0][0]=U[0][0];
    inputObs[1][0]=Y[0][0];
    inputObs[2][0]=Y[1][0];
    inputObs[3][0]=Y[2][0];
    inputObs[4][0]=Y[3][0];

    //Xo[n+1]=A*Xo[n]+B*inputObs[n]
    fM1=4;matrix_multiply_c4_c1(Ao,Xo_0,aux_4); //Ao*Xo[n]
    fM1=4;matrix_multiply_c5_c1(Bo,inputObs,aux_5); //Bo*inputObs[n]
    fM1=4;matrix_sum_c1(aux_4,aux_5,Xo_1);
    
    //Como Co es la matriz identidad se puede dar una asignacion directa (reduzco los tiempos de ejecutado)
    //Si no fuese la matriz identidad, debo reemplazar por: fM1=4;matrix_multiply_c4_c1(Co,Xo_0,Yo);
    Yo[0][0]=Xo_0[0][0];
    Yo[1][0]=Xo_0[1][0];
    Yo[2][0]=Xo_0[2][0];
    Yo[3][0]=Xo_0[3][0];
    
    //Xo[n]=Xo[n+1] (Para la proxima vez que se invoco al observador)
    Xo_0[0][0]=Xo_1[0][0];
    Xo_0[1][0]=Xo_1[1][0];
    Xo_0[2][0]=Xo_1[2][0];
    Xo_0[3][0]=Xo_1[3][0];
    
//REALIMENTACION    
    //U[n]=r[n]*Nbard-Kd*Yo[n]=r[n]*Nbard+(-Kd)*Yo[n]
    fM1=1;matrix_multiply_c4_c1(nKd,Yo,aux_3); //-Kd*X[n]
    fM1=1;matrix_sum_c1(ref_by_Nbard,aux_3,U);
}

// Funciones para configurar los registros del PIC18F4620
void PORT_configurar(){
    TRISCbits.RC7 = 1; //El pin RX como entrada
    TRISCbits.RC6 = 0; //El pin TX como salida
}
void INTERRUPT_configurar(){
    INTCONbits.GIE    = 0;
    INTCONbits.PEIE   = 0;
}

// Operaciones de matrices
void matrix_multiply_c1_c1(volatile double M1[][1],volatile double M2[][1],volatile double res[][1]){
    //El numero de columnas de M1 debe coinicidir con el numero de filas de M2
    int i=0,j=0,k=0;
    for(i=0;i<fM1;i++){    
        for(j=0;j<1;j++){    
            res[i][j]=0;    
            for(k=0;k<1;k++){    
                res[i][j]+=M1[i][k]*M2[k][j];    
            }    
        }    
    }    
}
void matrix_multiply_c4_c1(volatile double M1[][4],volatile double M2[][1],volatile double res[][1]){
    //El numero de columnas de M1 debe coinicidir con el numero de filas de M2
    int i=0,j=0,k=0;
    for(i=0;i<fM1;i++){    
        for(j=0;j<1;j++){    
            res[i][j]=0;    
            for(k=0;k<4;k++){    
                res[i][j]+=M1[i][k]*M2[k][j];    
            }    
        }    
    }    
}
void matrix_multiply_c5_c1(volatile double M1[][5],volatile double M2[][1],volatile double res[][1]){
    //El numero de columnas de M1 debe coinicidir con el numero de filas de M2
    int i=0,j=0,k=0;
    for(i=0;i<fM1;i++){    
        for(j=0;j<1;j++){    
            res[i][j]=0;    
            for(k=0;k<5;k++){    
                res[i][j]+=M1[i][k]*M2[k][j];    
            }    
        }    
    }    
}
void matrix_sum_c1(volatile double M1[][1],volatile double M2[][1],volatile double res[][1]){
    //En la suma ambas matrices tienen las mismas dimensiones
    int i=0,j=0;
    for(i=0;i<fM1;i++){    
        for(j=0;j<1;j++){    
            res[i][j]=M1[i][j]+M2[i][j];        
        }    
    }
}

//Tansferencia de datos al Data Visualizer

void sendDataVisualizer(){
    char* ptrPlanta;
    char* ptrObs;
    ptrPlanta=(char *) & Y[0][0];
    ptrObs=(char *) & Yo[0][0];
    UART_Write(0x03);
    //SALIDA PLANTA
    for (int x=0;x<sizeof(Y[0][0]);x++){
        UART_Write(*(ptrPlanta++));
    };
    ptrPlanta=(char *) & Y[1][0];
    for (int x=0;x<sizeof(Y[1][0]);x++){
        UART_Write(*(ptrPlanta++));
    };
    ptrPlanta=(char *) & Y[2][0];
    for (int x=0;x<sizeof(Y[2][0]);x++){
        UART_Write(*(ptrPlanta++));
    };
    ptrPlanta=(char *) & Y[3][0];
    for (int x=0;x<sizeof(Y[3][0]);x++){
        UART_Write(*(ptrPlanta++));
    };
    //SALIDA OBSERVADOR
    for (int x=0;x<sizeof(Yo[0][0]);x++){
        UART_Write(*(ptrObs++));
    };
    ptrObs=(char *) & Yo[1][0];
    for (int x=0;x<sizeof(Yo[1][0]);x++){
        UART_Write(*(ptrObs++));
    };
    ptrObs=(char *) & Yo[2][0];
    for (int x=0;x<sizeof(Yo[2][0]);x++){
        UART_Write(*(ptrObs++));
    };
    ptrObs=(char *) & Yo[3][0];
    for (int x=0;x<sizeof(Yo[3][0]);x++){
        UART_Write(*(ptrObs++));
    };
    UART_Write(0xFC);
}
void sendDataVisualizer24bits(){
    char* ptrPlanta;
    char* ptrObs;
    int aux=sizeof(Y[0][0]);
    ptrPlanta=(char *) & Y[0][0];
    ptrObs=(char *) & Yo[0][0];
    UART_Write(0x03);
    //SALIDA PLANTA
    UART_Write(0b0);
    for (int x=0;x<sizeof(Y[0][0]);x++){
        UART_Write(*(ptrPlanta++));
    };
    UART_Write(0b0);
    ptrPlanta=(char *) & Y[1][0];
    for (int x=0;x<sizeof(Y[1][0]);x++){
        UART_Write(*(ptrPlanta++));
    };
    UART_Write(0b0);
    ptrPlanta=(char *) & Y[2][0];
    for (int x=0;x<sizeof(Y[2][0]);x++){
        UART_Write(*(ptrPlanta++));
    };
    UART_Write(0b0);
    ptrPlanta=(char *) & Y[3][0];
    for (int x=0;x<sizeof(Y[3][0]);x++){
        UART_Write(*(ptrPlanta++));
    };
    //SALIDA OBSERVADOR
    UART_Write(0b0);
    for (int x=0;x<sizeof(Yo[0][0]);x++){
        UART_Write(*(ptrObs++));
    };
    ptrObs=(char *) & Yo[1][0];
    UART_Write(0b0);
    for (int x=0;x<sizeof(Yo[1][0]);x++){
        UART_Write(*(ptrObs++));
    };
    ptrObs=(char *) & Yo[2][0];
    UART_Write(0b0);
    for (int x=0;x<sizeof(Yo[2][0]);x++){
        UART_Write(*(ptrObs++));
    };
    ptrObs=(char *) & Yo[3][0];
    UART_Write(0b0);
    for (int x=0;x<sizeof(Yo[3][0]);x++){
        UART_Write(*(ptrObs++));
    };
    UART_Write(0xFC);
}