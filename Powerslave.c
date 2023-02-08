// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)
 
// CONFIG1H
#pragma config FOSC = INTOSCIO_EC// Oscillator Selection bits (Internal oscillator, port function on RA6, EC used by USB (INTIO))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = ON        // Internal/External Oscillator Switchover bit (Oscillator Switchover mode enabled)
 
// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)
 
// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)
 
// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RE3 input pin enabled; MCLR pin disabled)
 
// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))
 
// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)
 
// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)
 
// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)
 
// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)
 
// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)
 
// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

#define _XTAL_FREQ 8000000L
#define BAUD 9600
#include    <xc.h>
#include    <string.h>

//Prototypes:
void C_Puertos(void);
void C_I2C(void);
void I2C(void);
void C(char);                   // Función para enviar un comando al LCD.
void D(char* str);              // Función para enviar datos en cadena al LCD.
void Dato(unsigned char);       // Función para enviar un dato al LCD.
void Delay(void);               // Función de retardo de 5ms.
void C_Timer1(void);            // Configuración de Timer1.
void Conversion(void);

//Variables Globales:
unsigned char address, DatoTx, PWM=0x30, Decena, Unidad;
int iPWM;

void main(void) {
    C_Puertos();
    C_I2C();

    C(0x80);
    Dato(0x55);
    __delay_ms(1000);
    
    while (1)
    {
        C(0x80);
        Dato(PWM);
    }
}

void C_Puertos()
{
    ADCON1bits.PCFG = 0b1001;   // AN6-AN12 Entradas Digitales.
    TRISB=0x03;                 // 0b00000011, RB0 y RB1 son entradas digitales (SDA y SCL, respectivamente).
    TRISC=0x00;                 // 0b0000 0000 | RS (LC1) y E (LC2).
    TRISD=0x00;                 // Comandos y Datos (RD0-RD3) para el LCD.
    C(0x28);
    C(0x0F);
    C(0x0C);
    C(0x01);
    C(0x80);
    return;
}

void C_I2C()
{
    INTCONbits.GIE=0;           // Inhabilita todas las interrupciones.
    SSPSTATbits.SMP=1;          // Slew rate control disabled for Standard Speed mode (100kHz).
    SSPCON1bits.WCOL=0;         // No hay colisión de datos.
    SSPCON1bits.SSPOV=0;        // No hubo sobreflujo de datos.
    SSPCON1bits.SSPEN=1;        // Master Synchronous Serial Port Enable bit.
    SSPCON1bits.CKP=1;          // Libera el reloj.
                                // Habilita el puerto serial y configura SDA y SCL como los pines del puerto serial.
    SSPCON1bits.SSPM=0b0110;    // I2C Slave mode, dirección de 7 bits.
    SSPCON2=0b00000000;         // Comunicación I2C no iniciada.
    SSPADD=0x48;             // 0b00100000 - Valor que representa la dirección del esclavo, recorrida un bit a la izquierda.
    PIR1bits.SSPIF=0;           // Esperando a transmitir/recibir.
    INTCONbits.PEIE=1;          // Enables all unmasked peripheral interrupts.
    INTCONbits.GIE=1;           // Enables all unmasked interrupts.
    PIE1bits.SSPIE=1;           // Habilita la interupción del MSSP.
    return;
}

void __interrupt () COM(void)
{
    if(INTCONbits.PEIE==1 && PIE1bits.SSPIE==1 && PIR1bits.SSPIF==1) I2C();
}

void I2C()
{
    if(!SSPSTATbits.D_nA)
    {
       address=SSPBUF;                    // Si el dato recibido es una dirección, guarda el valor en address y vacía SSPBUF.
       PIR1bits.SSPIF=0;                  // Esperando a tranmisión/recepción.
    }
    
    if(SSPSTATbits.R_nW)                  // El Maestro requiere leer un dato del Esclavo.
    {
        SSPBUF=DatoTx;
        SSPCON1bits.CKP=1;                // Libera el reloj.
        PIR1bits.SSPIF=0;                 // Esperando a tranmisión/recepción.
    }
    
    if(!SSPSTATbits.R_nW)                 // El Maestro requiere escribir un dato en el Esclavo. Recepción de PWM
    {
        PWM=SSPBUF;                       // En PWM se guarda el valor equivalente al PWM, en char, que tiene un valor de 00h a 64h.
        PIR1bits.SSPIF=0;                 // Esperando a tranmisión/recepción.
    }
    return;
}

void C_Timer1()//¿Por qué esta función hace su trabajo si no está llamada? D:
{
    INTCONbits.GIE=0;       // Inhabilita todas las interrupciones.
    T1CONbits.RD16=1;       // Habilita la lectura/escritura del registro Timer1 en una operación de 16 bits.
    T1CONbits.TMR1CS=0;     // Internal clock. Se selecciona que funcione como temporizador.
    T1CONbits.T1CKPS=0b10;  // Prescaler 1:4.
    TMR1H=0x63;             // Valor para un temporizador de 5ms.
    TMR1L=0xBF;
    T1CONbits.TMR1ON=1;     // Enables Timer1.
    PIR1bits.TMR1IF=0;      // El registro TMR1 no se desbordó.
    PIE1bits.TMR1IE=1;      // Habilita la interrupción por desbordamiento de TMR1.
    INTCONbits.PEIE=1;      // Enables all unmasked peripheral interrupts.
    INTCONbits.GIE=1;       // Enables all unmasked interrupts.
    return;
}

void Delay()//Función que supuestamente retarda 5ms, por alguna razón.
{
    while (PIE1bits.TMR1IE && PIR1bits.TMR1IF); 
    TMR1H=0x63;         // Valores para la duración de cada vuelta del conteo.
    TMR1L=0xBF;         // Valores para que el temporizador se desborde después de 5ms.
    PIR1bits.TMR1IF=0;  // el registro TMR1 no se desbordó. Se pone cada que se reinicie el contador.
    return;
}

void C(char HEX)
{
    if(HEX!=0x20&&HEX!=0x0F){
                        Delay();    
    LATCbits.LC1=0;     Delay();
    LATCbits.LC2=0;     Delay();
    LATCbits.LC2=1;     Delay();
    LATD=(HEX>>4)&0x0F; Delay();
    LATCbits.LC2=0;     Delay();}
     
    LATCbits.LC1=0;     Delay();
    LATCbits.LC2=0;     Delay();
    LATCbits.LC2=1;     Delay();
    LATD=HEX;           Delay();
    LATCbits.LC2=0;     Delay();
    return;
}

void D (char* Cadena)
{
    int l;
    l = strlen(Cadena);
    for (int i=0; i<l; i++)
    {
        
                                  Delay();
        LATCbits.LC1=1;           Delay();
        LATCbits.LC2=0;           Delay();
        LATCbits.LC2=1;           Delay();
        LATD=(Cadena[i]>>4)&0x0F; Delay();
        LATCbits.LC2=0;           Delay();
    
        LATCbits.LC2=0;           Delay();
        LATCbits.LC2=1;           Delay();
        LATD=Cadena[i]&0x0F;      Delay();
        LATCbits.LC2=0;           Delay();
    }
    return;
}

void Dato (unsigned char HEX)
{       
        
                            Delay();
        LATCbits.LC1=1;     Delay();
        LATCbits.LC2=0;     Delay();
        LATCbits.LC2=1;     Delay();
        LATD=(HEX>>4)&0x0F; Delay();
        LATCbits.LC2=0;     Delay();
    
        LATCbits.LC2=0;     Delay();
        LATCbits.LC2=1;     Delay();
        LATD=HEX&0x0F;      Delay();
        LATCbits.LC2=0;     Delay();
        return;
}

void Conversion()
{
    int i=0, n=0, b=0, ten, unit;
    iPWM=PWM;               // PWM es el valor de 1 byte que se recibe por I2C.
    n=iPWM;                 // iPWM es la conversión implícita de char a entero.
    for(i=0;i<=9;i++)
    {
        b=i*10; // b va de 0 a 90.
        n=iPWM-b;   // n = n - b
        if(n<10)
        {
            ten=i;
            break;
        }
    }
    iPWM=n;
    for(i=0;i<=9;i++)
    {
        n=iPWM-i;
        if(n==0)
        {
            unit=i;
            break;
        }
    }
    Decena=ten+0x30;
    Unidad=unit+0x30;
    return;
}