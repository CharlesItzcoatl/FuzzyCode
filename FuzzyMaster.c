#include <xc.h>
#include <string.h>
#include <math.h>
#include "Fuses.h"
//Prototypes:
void C_Puertos(void);               // Configuración de Puertos.
void C_ADC(void);                   // Configuración de ADC.
void C_Timer0(void);                // Configuración de Timer0.
void C_Timer1(void);                // Configuración de Timer1.
void C_EUSART(void);                // Configuración del módulo EUSART.
void C(unsigned char);              // Función para enviar un comando al LCD.
void D(unsigned char*);             // Función para enviar datos en cadena al LCD.
void Dato(unsigned char);           // Función para enviar un dato al LCD.
void Conversor (double, int);       // Adaptación de datos de Temperatura para mostrarse en LCD.
void Delay(void);                   // Función de retardo de 5ms.
void Timer(void);                   // Muestra la hora en el LCD.
void TACQ(void);                                            // Función para el proceso de lectura de mediciones de los sensores de Temperatura.
double Promedio(double*);                                   // Función para obtener un valor promedio para las Temperaturas medidas.
double Channel(unsigned char, int);                         // Función para la selección del canal analógico a convertir.
void MainDisplay(void);                                     // Estado normal del LCD; muestra las Temperaturas promediadas, RH y el Temporizador.
void RHControl(void);                                       // Función para el control ON-OFF de la Humedad Relativa.
int WaterLevel(void);                                       // Función para notificar si hay o no hay agua en el contenedor
void WeightACQ(void);                                       // Función pra obtener el valor del peso sobre el colchón.
void FuzzyControl(double, double, double, double); // Función para todo el Control Difuso.
void ErrorS(float, float*); // Función que recibe el valor de temperatura y lo asigna a las ecuaciones que describen a los conjuntos difusos de ErrorS.
void ErrorA(float, float*); // Función que recibe el valor de temperatura y lo asigna a las ecuaciones que describen a los conjuntos difusos de ErrorA.
void ErrorW(float, float*); // Función que recibe el valor de temperatura y lo asigna a las ecuaciones que describen a los conjuntos difusos de ErrorW.
void ErrorM(float, float*); // Función que recibe el valor de temperatura y lo asigna a las ecuaciones que describen a los conjuntos difusos de ErrorM.
void RuleEvaluation(float[], float[], float[], float[]);    // Selección de reglas activadas y determinación de mínimos para cada una.
float min(float, float, float, float);                      // Función de mínimos para el método de agregación.
float MAX(float, float);
float Centroid(int);                                        // Cálculo de centroide (criterio de combinaciones).
void Keyboard(void);                                        // Función que permite decidir las opciones del menú.
void Page2(void);                                           // Función que muestra los valores de Set Point y Potencia actuales.
void SetPointMod(void);                                     // Serie de funciones para configurar el Set Point de cualquier variable.
void SetPointKeyboard(void);
void ChangeSP(void);
void SPKeyboard(void);
double SPMod(int);
void LEDsMod(void);                                        // Serie de funciones para configurar el estado de la tira de LEDs.
void LEDsKeyboard(void);
void ForcePWM(void);                                       // Serie de funciones para forzar la actualización del valor de PWM.
void PWMMod(void);
void TimerMod(void);                                       // Serie de funciones para Reset del Timer.
void TimerKeyboard(void);
void ErrorDisplay(void);                                   // Función que muestra los Errores de las variables y el valor del PWM actuales.
void Alarm(int);                                           // Función que despliega las señales de alarma.
void AlarmMod(void);                                       // Serie de funciones para configurar el estado de la alarma.
void AlarmKeyboard(void);
void ForceHumidifier(void);                                // Serie de funciones para forzar la actualización del valor de PWM.
void HumidifierMod(void);
void DataTx(unsigned char);
void RHReceiver(void);
//Variables Globales:
float Y[4]={0.0, 0.0, 0.0, 0.0};		// Vector que representa la unión (nuevo conjunto difuso del cual se realizará el cálculo del centroide).
float Y2[4]={0.0, 0.0, 0.0, 0.0};
unsigned char ten=0x00, unit=0x00, tenth=0x00, hundredth=0x00, cPWM=0x00, Receiver=0x00, cFPWM=0x00;
int segundo=0, segundos=0, minuto=0, minutos=0, hora=0, RH=0, iPWM=0, iFPWM=0, op=0, Condicional=0, Counter=0, EH=0;
double TS=0.0, TA=0.0, TW=0.0, TM=0.0, PWM=0.0, FPWM=0.0, SPS=21.5, SPA=22.5, SPW=22.0, SPM=22.0, SPH=80.0, ES=0.0, EA=0.0, EW=0.0, EM=0.0, Voltage=0.0;
double Weight=0.0;
int TTS1=0, TTS2=0;
unsigned char TSV[2]={0,0}, TAV[2]={0,0};
void main(void) 
{
    C_ADC();        // Configuración del ADC y oscilador.
    C_Puertos();    // Configuración de los puertos.
    C_EUSART();     // Configuración del módulo EUSART.
    C_Timer0();     // Configuración del Temporizador.
    //C_Timer1();
    while (1) 
    {
        RHReceiver();
        WaterLevel();   // Medición de nivel de agua.
        MainDisplay();  // Pantalla principal.
        Keyboard();     // Menú del teclado matricial.
        TACQ();         // Registro de TS, TA, TW y TM medidas.
        //ES = SPS - TS;  // Obtención del Error para la Temperatura de la Piel.
        //EA = SPA - TA;  // Obtención del Error para la Temperatura del Aire.
        //EW = SPW - TW;  // Obtención del Error para la Temperatura de las Paredes.
        //EM = SPM - TM;  // Obtención del Error para la Temperatura del Colchón.
        //EH = SPH - RH;  // Obtención del Error para la Humedad Relativa.
        if((hora+minutos+minuto+segundos+segundo) != 0)     // Cambia el valor del PWM cada 5 minutos, excepto al inicializar.
        if((minuto==0 || minuto==5) && segundos==0 && segundo==0)
        {
            FuzzyControl(ES, EA, EW, EM);   // FuzzyControl ya recibe valores flotantes.
            RHControl();                    // Control de Humedad.
            DataTx(cPWM);                       // Transmisión de PWM de la resistencia calefactora.
            DataTx(cFPWM);                      // Transmisión de PWM del ventilador.
        }
        DataTx(TSV[0]);
        DataTx(TSV[1]);
        DataTx(TAV[0]);
        DataTx(TAV[1]);
    }
}

void C_Puertos()
{   
    ADCON1bits.PCFG=0b0011;   // Sólo AN12 es I/O digital, todas las demás, analógicas.
    TRISA=0x00;               // 0b0000 0000 | Para el LCD.
    TRISB=0x0E;               // 0b0000 1110, RB0 es salida digital (buzzer).
                              // RB2[AN8] (TM) y RB3[AN9] (Galgas) como entradas analógicas. RB4 para buzzer y RB5 para tira de LEDs.
    TRISC=0xC0;               // 0b1100 0000 | RS (LC1) y E (LC2).
                              // RC6 Tx || RC7 Rx
    TRISD=0x0F;               // 0b0000 1111 Teclado matricial, Nibble alto -> salidas | Nibble bajo -> entradas.
    TRISE=0x07;               // 0b0011Entradas analógicas.
    PORTEbits.RDPU=1;         // Resistencias de Pull-Up de PORTD habilitadas.
    C(0X28);                  // 4 bits 2 líneas
    C(0x0F);                  // Configuración del cursor.
    C(0x0C);                  // Inicialización 4 bits.
    C(0x01);                  // Clear Display       
    C(0X80);                  // Regreso a inicio.
    LATBbits.LB0=0;           // Buzzer normalmente apagado.
    LATBbits.LB5=0;           // Tira de LEDs normalmente apagada.
    LATCbits.LC0=0;           // Humidificador normalmente apagado.
    return;
}

void C_ADC()
{
    OSCCONbits.IRCF = 0b111; // Configura oscilador interno (FOSC = 8MHz) [p. 35]
    OSCCONbits.SCS = 0b10;   // Oscilador del sistema = Fosc interno
    ADCON1=0x05;             // Selecciona Voltajes de Referencia VDD y VSS. Configura AN10-AN12 como I/O digitales.
    ADCON2=0x8E;             // Tiempo de Adquisición = 2T_AD = 16us (TACQ>=2.45us). Tiempo de Conversión Fosc/64 T_AD = 8us. (T_AD min=0.8us). Justificación derecha (modo-10bits).
    return;
}

void C_Timer0 ()// Que reciba el valor entero que represente los milisegundos que serán insertados en la ecuación.
{
    INTCONbits.GIE=0;       // Inhabilita todas las interrupciones.
    T0CONbits.T0CS=0;       // Internal instruction cycle clock (CLKO). Se selecciona que funcione como temporizador.
    T0CONbits.PSA=0;        // Timer0 prescaler is assigned. Timer0 clock input comes from prescaler output.
    T0CONbits.T0PS=0b101;   // 1:64 Prescale value.
    T0CONbits.T08BIT=0;     // Timer0 está configurado como un temporizador de 16 bits.
    TMR0L=34286;            // Valores para la duración de cada vuelta del conteo.
    TMR0H=(34286)>>8;       // Valores para que el temporizador se desborde cada 1s.
    T0CONbits.TMR0ON=1;     // Habilitar Timer0, empiezan a ocurrir los ciclos de reloj.
    INTCONbits.TMR0IF=0;    // el registro TMR0 no se desbordó. Se pone cada que se reinicie el contador.
    INTCONbits.TMR0IE=1;    // Habilita la interrupción por desbordamiento de TMR0.
    INTCONbits.PEIE=1;      // Enables all unmasked peripheral interrupts. No es necesario si no se depende de un push button(?).
    INTCONbits.GIE=1;       // Enables all unmasked interrupts.
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

void C_EUSART()
{
    SPBRG=12;               // Valor para obtener 9600 baudios con un error de 0.15% (9615)
    TXSTAbits.BRGH=0;       // Baja velocidad de baudios.
    BAUDCONbits.BRG16=0;    // Baudios modo 8 bits.
    TXSTAbits.SYNC=0;       // Modo asíncrono (Tx y Rx trabajan con diferentes relojes, se envían bits
                            // Start y Stop con cada byte de datos para identificar la información).
    RCSTAbits.SPEN=1;       // Puerto Serial habilitado (configura los pines RX/DT y TX/CK como pines de puerto serial).
    RCSTAbits.RX9=0;        // Selecciona modo de recepción de 8 bits.
    RCSTAbits.CREN=1;       // Habilita el receptor.
    TXSTAbits.TX9=0;        // Transmisión de 8 bits habilitada.
    TXSTAbits.TXEN=1;       // Transmisión habilitada. Pone en nivel alto al bit TXIF,  
                            // que indica si TXREG está lleno (0) o vacío (1).
    PIR1bits.TXIF=1;        // El buffer de transmisión EUSART está vacío.
    return;   
}

void C(unsigned char HEX)
{
    if(HEX!=0x20&&HEX!=0x0F){
                        Delay();    
    LATCbits.LC1=0;     Delay();
    LATCbits.LC2=0;     Delay();
    LATCbits.LC2=1;     Delay();
    LATA=(HEX>>4)&0x0F; Delay();
    LATCbits.LC2=0;     Delay();}
     
    LATCbits.LC1=0;     Delay();
    LATCbits.LC2=0;     Delay();
    LATCbits.LC2=1;     Delay();
    LATA=HEX;           Delay();
    LATCbits.LC2=0;     Delay();
    return;
}

void D(unsigned char* Cadena)
{
    int l;
    l = strlen(Cadena);
    for (int i=0; i<l; i++)
    {
        
                                  Delay();
        LATCbits.LC1=1;           Delay();
        LATCbits.LC2=0;           Delay();
        LATCbits.LC2=1;           Delay();
        LATA=(Cadena[i]>>4)&0x0F; Delay();
        LATCbits.LC2=0;           Delay();
    
        LATCbits.LC2=0;           Delay();
        LATCbits.LC2=1;           Delay();
        LATA=Cadena[i]&0x0F;      Delay();
        LATCbits.LC2=0;           Delay();
    }
    return;
}

void Dato(unsigned char HEX)
{       
        
                            Delay();
        LATCbits.LC1=1;     Delay();
        LATCbits.LC2=0;     Delay();
        LATCbits.LC2=1;     Delay();
        LATA=(HEX>>4)&0x0F; Delay();
        LATCbits.LC2=0;     Delay();
    
        LATCbits.LC2=0;     Delay();
        LATCbits.LC2=1;     Delay();
        LATA=HEX&0x0F;      Delay();
        LATCbits.LC2=0;     Delay();
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

void interrupt OneSecond(void)// Interrupción para incrementar un segundo.
{
    if (INTCONbits.TMR0IE && INTCONbits.TMR0IF)
    {
        segundo++;
        TMR0L=34285;            // Valores para la duración de cada vuelta del conteo.
        TMR0H=(34285)>>8;       // Valores para que el temporizador se desborde cada 1s.
        INTCONbits.TMR0IF=0;    // El registro TMR0 no se desbordó. Se pone cada que se reinicie el contador.
    }
    if(segundo==10)
    {
        segundo=0;
        segundos++;
        if(segundos==6)
        {
            segundo=0;
            segundos=0;
            minuto++;
            if(minuto==10)
            {
                minuto=0;
                minutos++;
                if(minutos==6)
                {
                    minutos=0;
                    hora++;
                }
            }
        }
    }
}

void Timer()
{
    unsigned char second, seconds, minute, minutes, hour;
    second=segundo+0x30;
    seconds=segundos+0x30;
    minute=minuto+0x30;
    minutes=minutos+0x30;
    hour=hora+0x30;
    C(0x8D);
    Dato(hour);
    Dato(0x3A);
    Dato(minutes);
    Dato(minute);
    Dato(0x3A);
    Dato(seconds);
    Dato(second);
    return;
}

void Conversor(double n, int OP)
{
    int decena=0,unidad=0,decima=0,centesima=0;
    ten=0;
    unit=0;
    tenth=0;
    hundredth=0;
	double b=0.0,aux=0.0;
	int i=1,aux2=0,c=0;
	aux=n;
	for(i=0;i<=9;i++)
	{
		b=i*10;
		n-=b;
		if(n<10&&n>=0)
		{
			decena=i;
			break;
		}
		else n=aux;
	}
	aux=n;
	for(i=0;i<=9;i++)
	{
		n-=i;
		if(n<1)
		{
			unidad=i;
			break;
		}
		else n=aux;
	}
	n+=0.001;
	aux=n;
	for(i=0;i<=9;i++)
	{
		b=i*0.1;
		n-=b;
		if((n<=0.1)&&n>=0)
		{
			decima=i;
			break;
		}
		else n=aux;
	}
	if(n>0.009)
	{
		n*=100;
		c=(int)n;
		aux2=c;
		for(i=0;i<=9;i++)
		{
			c-=i;
			if(c==0)
			{
				centesima=i;
				break;
			}
			else c=aux2;
		}
	}
    ten=decena+0x30;
    unit=unidad+0x30;
    tenth=decima+0x30;
    hundredth=centesima+0x30;
    TTS1=(decena*10)+unidad;
    TTS2=(decima*100)+(centesima*100);
    switch(OP)
    {
        case 0:                 // Cuando es temperatura
            Dato(ten);
            Dato(unit);
            Dato(0x2E);
            Dato(tenth);
            Dato(0xDF);
            D("C");
            break;
        case 1:                 // Cuando es PWM
            Dato(ten);
            Dato(unit);
            Dato(0x2E);
            Dato(tenth);
            Dato(hundredth);
            Dato(0x25);
            break;
        case 2:                 // Cuando el PWM es de 100%
            Dato(PWM+0x30);
            Dato(ten);
            Dato(unit);
            Dato(0x25);
            break;
        case 3:                 // Cuando es RH
            Dato(ten);
            Dato(unit);
            Dato(0x25);
            break;
        case 4:                 // Cuando es conversión de peso
            Dato(unit);
            Dato(0x2E);
            Dato(tenth);
            Dato(hundredth);
            D(" kg");
            break;
        case 5:                 // Cuando es Error de T
            Dato(unit);
            Dato(0x2E);
            Dato(tenth);
            Dato(hundredth);
            Dato(0xDF);
            D("C");
            break;
    }
    return;
}

void TACQ()
{
    double TSkin[10];
    double TAir[10];
    double TWalls[10];
    double TMatt[10];
    
    // TS ACQUISITION.
    TSkin[Counter] = Channel(0x14,0);
    if(TSkin[9] != 0) TS=Promedio(TSkin);
    
    // TA ACQUISITION.
    TAir[Counter] = Channel(0x18,1);
    if(TAir[9] != 0) TA=Promedio(TAir);
    
    // TW ACQUISITION.
    TWalls[Counter] = Channel(0x1C,2);
    if(TWalls[9] != 0) TW=Promedio(TWalls);
    
    // TM ACQUISITION.
    TMatt[Counter] = Channel(0x20,3);
    if(TMatt[9] != 0) TM=Promedio(TMatt);
    
    if(Counter<9) Counter++;
    else 
    {
        Counter=0;
        for(int i=0;i<=9;i++)
        {
            TSkin[i]=0;
            TAir[i]=0;
            TWalls[i]=0;
            TMatt[i]=0;
        }
    } 
    
    return;
}

double Channel(unsigned char Selector, int Operator)
{
    float Rc=0.0, Rref=10000, R=0.0;
    unsigned int LecADC=0;
    double V0=0.0, E=0.0, Temperatura=0.0;
    ADCON0=Selector;                    // Deshabilita el Módulo AD, Selecciona el Canal Analógico (Channel 'Selector' [ANx] -> REx | RB2).
    ADCON0bits.ADON = 1;                // Habilita el Módulo AD.
    __delay_us(30);
    ADCON0bits.GO_DONE = 1;             // Inicia la Conversión AD.
    while (ADCON0bits.GO_DONE);
    LecADC = ADRESH;
    LecADC = (LecADC << 8) + ADRESL;    
    V0 = LecADC;
    V0 *= VCC / 1023.0;
    Voltage = V0;
    switch(Operator)
    {
        case 0:     // TS
            Rc=9960;
            break;
        case 1:     // TA
            Rc=6860;
            Rref=6666.667;
            break;
        case 2:     // TW
            Rc=10010;
            break;
        case 3:     // TM
            Rc=10020;
            break;
        default:
            Rc=0.0;
            break;
    }
    R = Rc/((VCC/V0)-1);
    E = log(R/Rref);
    Temperatura = pow(A1+(B1*E)+(C1*pow(E,2))+(D1*pow(E,3)),-1)-273.15;  // Temperatura en el Colchón.
    return Temperatura;
}

double Promedio(double* V)
{
	double Q=0.0, *pVector;
	pVector=&V[0];
	for(int i=0;i<=9;i++)
	{
		Q+=*pVector;
		pVector++;
	}
	return Q/10;
}

void MainDisplay()
{
    Timer();                            // Despliegue del Temporizador.
    
    C(0x80);                            // Primera línea del LCD.
    D("TS=");
    Conversor(TS, 0);
    TSV[0]=TTS1;
    TSV[1]=TTS2;
    
    C(0xA8);                            // Segunda línea del LCD.
    D("TA=");
    Conversor(TA, 0);
    TAV[0]=TTS1;
    TAV[1]=TTS2;
    
    C(0x94);                            // Tercera línea del LCD.
    D("TW=");
    Conversor(TW, 0);
    
    C(0xD4);                            // Cuarta línea del LCD.
    D("TM=");
    Conversor(TM, 0);
    
    C(0xCA);
    D("W:");
    Conversor(Weight, 4);
    
    C(0x9E);                            //  Segunda línea del LCD.
    D("PWM=");
    if(PWM==1) Conversor(PWM*100, 2);
    else Conversor (PWM*100, 1);
        
    C(0xDE);
    D("R.H=");
    Conversor(RH, 3); 
    
    return;
}

void RHReceiver()
{
    RCSTAbits.CREN=0;
    __delay_ms(10);
    RCSTAbits.CREN=1;
    __delay_ms(10);
    Receiver=RCREG; // Registro de R.H medidas (adquisición por EUSART).
    RH=Receiver;
    return;
}

void RHControl()
{
    
    if(WaterLevel())        // Si retorna un '1', quiere decir que el contenedor de agua tiene agua (protección del humidificador).
    {
        if(RH < SPH) LATCbits.LC0=1;       // Si la Humedad Relativa registrada es inferior al Set Point de Humedad, enciende el humidificador.
        else LATCbits.LC0=0;               // En caso contrario, apagar el humidificador.
    }
    else
    {
        LATCbits.LC0=0;
        C(0x01);
        __delay_ms(50);
        C(0x80);
        D("Low water level");
        C(0xA8);
        D("Fill container.");
        __delay_ms(500);
        C(0x01);
        __delay_ms(50);
    }
    return;
}

int WaterLevel()
{
    Channel(0x2C, 0);       // RB4 [AN11]
    if(Voltage>1.0) return 1;
    else return 0;
}

void WeightACQ()
{
    float Galga1=0.0, Galga2=0.0;
    Channel(0x24, 0); // Galga 1 RB3 [AN9]
    Galga1=Voltage;   // Guarda el valor actual del voltaje en el canal seleccionado (hacer ecuación de la forma y = mx + b).
    Channel(0x28, 0); // Galga 2 RB1 [AN10]
    Galga2=Voltage;   // Guarda el valor actual del voltaje en el canal seleccionado (hacer ecuación de la forma y = mx + b).
    // Aquí debe ir la ecuación del funcionamiento de las galgas.
    // Aquí debe ir el promedio del peso obtenido de las galgas.
    return;
}

// FUZZY CONTROL START.

void FuzzyControl(double ErrorSkin, double ErrorAir, double ErrorWalls, double ErrorMattress)
{
    float ESkin[3]={0.0, 0.0, 0.0};		   	// Vector que contiene el grado de pertenencia del error a cada uno de los conjuntos difusos creados para el ErrorS.
    float EAir[3]={0.0, 0.0, 0.0};			// Vector que contiene el grado de pertenencia del error a cada uno de los conjuntos difusos creados para el ErrorA.
    float EWalls[3]={0.0, 0.0, 0.0};		// Vector que contiene el grado de pertenencia del error a cada uno de los conjuntos difusos creados para el ErrorW.
    float EMattress[3]={0.0, 0.0, 0.0};		// Vector que contiene el grado de pertenencia del error a cada uno de los conjuntos difusos creados para el ErrorM.
	int o=0;
	
	// Creación de vectores para representar el grado de membresía a los conjuntos difusos de cada Error.
	ErrorS(ErrorSkin, ESkin);
	ErrorA(ErrorAir, EAir);
	ErrorW(ErrorWalls, EWalls);
	ErrorM(ErrorMattress, EMattress);
	
	// Proceso de activación de reglas y mínimos para cada una.
	RuleEvaluation(ESkin, EAir, EWalls, EMattress);
	
	// Combinaciones posibles entre los conjuntos difusos de la salida.
	if(Y[0]==0 && Y[1]==0 && Y[2]==0 && Y[3]!=0)  o=1;
	if(Y[0]==0 && Y[1]==0 && Y[2]!=0 && Y[3]==0)  o=2;
	if(Y[0]==0 && Y[1]==0 && Y[2]!=0 && Y[3]!=0)  o=3;
	if(Y[0]==0 && Y[1]!=0 && Y[2]==0 && Y[3]==0)  o=4;
	if(Y[0]==0 && Y[1]!=0 && Y[2]!=0 && Y[3]==0)  o=6;
	if(Y[0]==0 && Y[1]!=0 && Y[2]!=0 && Y[3]!=0)  o=7;
	if(Y[0]!=0 && Y[1]==0 && Y[2]==0 && Y[3]==0)  o=8;
	if(Y[0]!=0 && Y[1]!=0 && Y[2]==0 && Y[3]==0)  o=12;
	if(Y[0]!=0 && Y[1]!=0 && Y[2]!=0 && Y[3]==0)  o=14;
	if(Y[0]!=0 && Y[1]!=0 && Y[2]!=0 && Y[3]!=0)  o=15;
	
	// Proceso de cálculo de centroide para la potencia de la resistencia calefactora.
	PWM=Centroid(o);        
    
    if(Y2[0]!=0 && Y2[1]==0 && Y2[2]==0)  o=17;	// Conjunto Slow.
	if(Y2[0]==0 && Y2[1]!=0 && Y2[2]==0)  o=18;	// Conjunto Moderate.
	if(Y2[0]!=0 && Y2[1]!=0 && Y2[2]==0)  o=19;	// Conjunto Moderate & Slow.
	if(Y2[0]==0 && Y2[1]==0 && Y2[2]!=0)  o=20;	// Conjunto Fast.
	if(Y2[0]!=0 && Y2[1]==0 && Y2[2]!=0)  o=21;	// Conjunto Slow & Fast.
	if(Y2[0]==0 && Y2[1]!=0 && Y2[2]!=0)  o=22;	// Conjunto Moderate & Fast.
	if(Y2[0]!=0 && Y2[1]!=0 && Y2[2]!=0)  o=23;	// Conjunto Slow, Moderate & Fast.
    // Proceso de cálculo de centroide para la potencia del ventilador.
    if(o==0) o=16;
    FPWM=Centroid(o);
    
	iPWM=100*PWM;           // Conversión implícita de double a int.
    cPWM=iPWM;              // Conversión implícita de int a char.
    iFPWM=100*FPWM;       // Conversión implícita de double a int.
    cFPWM=iFPWM;             // Conversión implícita de int a char.
	return;
}

void ErrorS(float T, float* Vector)
{
	float *pError;
	pError=Vector; 							 // Apuntador a la dirección del vector ESkin.
	if(T<-0.5)
	{
        Alarm(0);   // Temperatura demasiado alta.
		return;
	}
	if(T>=-0.5 && T<-0.49) *pError=100*(T+0.5);  // Error para la pendiente positiva del conjunto difuso 'Negative'.
	if(T>=-0.49 && T<0)	   *pError=-2.04*T;		 // Error para la pendiente negativa del conjunto difuso 'Negative'.
	pError++;									 // Pasa a las ecuaciones del conjunto difuso 'Zero'.
	if(T>=-0.3 && T<0) *pError=(T+0.3)/0.3;		 //Error para el conjunto difuso 'Zero'.
	if(T>=0 && T<0.3) *pError=(0.3-T)/0.3;		 //Error para el conjunto difuso 'Zero'.
	pError++;									 // Pasa a las ecuaciones del conjunto difuso 'Positive'.
	if(T>=0 && T<0.49) 	 *pError=2.04*T;		 // Error para la pendiente positiva del conjunto difuso 'Positive'.
	if(T>=0.49 && T<0.5) *pError=-100*(T-0.5);   // Error para la pendiente negativa del conjunto difuso 'Positive'.
    if(T>=0.5)
    {
        Alarm(1);   // Temperatura demasiado baja.
        return;
    }
	pError=Vector;
	for (int i=0;i<=2;i++) 
	{
		if (*pError <= 0.01) *pError=0.0;
		pError++;
	}
	return;
}

void ErrorA(float T, float* Vector)
{
	float *pError;
	pError=Vector;							 // Apuntador a la dirección del vector EAir.
	if(T<-1)
	{
        Alarm(2);
		return;
	}
	if(T>=-1 && T<-0.99) *pError=100*(T+1);		 // Error para la pendiente positiva del conjunto difuso 'Negative'.
	if(T>=-0.99 && T<0)  *pError=-T/0.99;		 // Error para la pendiente negativa del conjunto difuso 'Negative'.
	pError++; 									 // Pasa a las ecuaciones del conjunto difuso 'Zero'.
	*pError=1/(1+pow(T/0.42,8)); 				 //Error para el conjunto difuso 'Zero'.
	pError++;									 // Pasa a las ecuaciones del conjunto difuso 'Positive'.
	if(T>=0 && T<0.99) *pError=T/0.99;			 // Error para la pendiente positiva del conjunto difuso 'Positive'.
	if(T>=0.99 && T<1) *pError=-100*(T-1);		 // Error para la pendiente negativa del conjunto difuso 'Positive'.
    if(T>=1)
    {
        Alarm(3);
        return;
    }
	pError=Vector;
	for (int i=0;i<=2;i++) 
	{
		if (*pError <= 0.01) *pError=0.0;
		pError++;
	}
	return;
}

void ErrorW(float T, float* Vector)
{
	float *pError;
	pError=Vector;							 // Apuntador a la dirección del vector EWalls.
	*pError=1/(1+exp(6*(T+1)));                  // Error para la ecuación del conjunto difuso 'Negative'.
	pError++;									 // Pasa a la ecuación del conjunto difuso 'Zero'.
	*pError=exp(-pow(T,2)/1.1);					 // Error para la ecuación del conjunto difuso 'Zero'.
	pError++;									 // Pasa a la ecuación del conjunto difuso 'Positive'.
	*pError=1/(1+exp(-6*(T-1)));				 // Error para la ecuación del conjunto difuso 'Zero'.
	pError=Vector;
	for (int i=0;i<=2;i++) 
	{
		if (*pError <= 0.01) *pError=0.0;
		pError++;
	}
	return;
}

void ErrorM(float T, float* Vector)
{
	float *pError;
	pError=Vector;
	*pError=1/(1+exp(7*(T+1.05)));				 // Ecuación para el Error del conjunto difuso 'Negative'.
	pError++;									 // Pasa a la ecuación del conjunto difuso 'Zero'.
	if(T>=-2 && T<-0.75) *pError=(T+2)/1.25;     // Pendiente positiva para el Error del conjunto difuso 'Zero'.
	if(T>=-0.75 && T<0.75) *pError=1;            // Absolutamente miembro del conjunto difuso Zero.
	if(T>=0.75 && T<2) *pError=-(T-2)/1.25;      // Pendiente negativa para el Error del conjunto difuso 'Zero'.
	pError++;									 // Pasa a las ecuaciones del conjunto difuso 'Positive'.
	*pError=1/(1+exp(-7*(T-1.05)));				 // Ecuación para el Error del conjunto difuso 'Positive'.
	pError=Vector;
	for (int i=0;i<=2;i++) 
	{
		if (*pError <= 0.01) *pError=0.0;
		pError++;
	}
	return;
}

void RuleEvaluation(float EPiel[], float EAire[], float EPared[], float ECol[])
{
    float VZ=0.0, VS=0.0, VM=0.0, VF=0.0, VFS=0.0, VFM=0.0, VFF=0.0;
	int i=0, j=0, k=0, l=0, contador=0;
	for(i=0;i<=2;i++) // 81 combinaciones posibles, que representan todas las reglas que se pueden crear para el control difuso.
		{
			for(j=0;j<=2;j++)
			{
				for(k=0;k<=2;k++)
				{
					for(l=0;l<=2;l++)
					{
						if (EPiel[i]>0 && EAire[j]>0 && EPared[k]>0 && ECol[l]>0)// Si existen valores en estas combinaciones...
						{	
							if(contador>=0 && contador<17) VZ=MAX(min(EPiel[i], EAire[j], EPared[k], ECol[l]), VZ);
							
							if(contador>=17 && contador<41) VS=MAX(min(EPiel[i], EAire[j], EPared[k], ECol[l]), VS);
							
							if(contador>=41 && contador<63) VM=MAX(min(EPiel[i], EAire[j], EPared[k], ECol[l]), VM);
							
							if(contador>=63 && contador<81) VF=MAX(min(EPiel[i], EAire[j], EPared[k], ECol[l]), VF);
							
							if((contador>=27 && contador<45)) VFS=MAX(min(EPiel[i], EAire[j], EPared[k], ECol[l]), VFS);
							
							if((contador>=18 && contador<27) || (contador>=45 && contador <63)) VFM=MAX(min(EPiel[i], EAire[j], EPared[k], ECol[l]), VFM);
							
							if((contador>=0 && contador <18) || (contador>=63 && contador <81)) VFF=MAX(min(EPiel[i], EAire[j], EPared[k], ECol[l]), VFF);
						}
						contador++; // Para ver el número total de reglas (mover el número de regla).
					}
				}
			}
		}
    
    // Proceso de obtención de nuevo conjunto difuso a partir de los grados de pertenencia a los conjuntos difusos de la salida.
    Y[0]=VZ;
	Y[1]=VS;
	Y[2]=VM;
	Y[3]=VF;
	Y2[0]=VFS;
	Y2[1]=VFM;
	Y2[2]=VFF;
	return;
}

float min(float a, float b, float c, float d)
{
	float X=0.0;
	if(a<=b) X=a;								 // Compara los dos primeros 
	else X=b;									 // Asigna el valor mínimo.
	if(X>=c) X=c;								 // Compara con el tercer elemento.
	if(X>=d) X=d;								 // Compara con el último elemento.
	return X;
}

float MAX(float a, float b)
{
	if(a>=b) 
	return a;
	else return b;
}

float Centroid(int Conjuntos)
{
	int i=0;
	float a=0.0, b=0.0, c=0.0, d=0.0;
	
	switch(Conjuntos)
	{
		case 0:
			if(ES >= 0.5 || EA >= 1.0) a=1.0;
			if(ES < -0.5 || EA < -1.0) a=0.0;
			return a;
		case 1:
			//La sumatoria va de 0.85 a 1, con 16 puntos:  Fast >= 0.85.
			for(i=85;i<=100;i++) a+=(Y[3]*i*0.01)/(Y[3]*16);
			return a;
		case 2:
			//La sumatoria va de 0.1 a 0.9, con 81 puntos: 0.1 <= Moderate <= 0.9.
			for(i=10;i<=90;i++) a+=(Y[2]*i*0.01)/(Y[2]*81);
			return a;
		case 3:
			//La sumatoria va de 0.1 a 1, con 91 puntos; Moderate va de 0.1 a 0.85, con 76 puntos y Fast va de 0.85 a 1, con 15 puntos. 0.1 <= Moderate <= 0.86 && 0.86 < Fast <=1.
			for(i=10;i<=85;i++) a+=(Y[2]*i*0.01);
			for(i=86;i<=100;i++) b+=(Y[3]*i*0.01);
			a+=b;
			a/=((Y[2]*76)+(Y[3]*15));
			return a;
		case 4:
			//La sumatoria va de 0.01 a 0.35, con 35 puntos: 0.01 <= Slow <= 0.35.
			for(i=1;i<=35;i++) a+=(Y[1]*i*0.01)/(Y[1]*35);
			return a;
		case 6:
			//La sumatoria va de 0.01 a 0.9, con 90 puntos; Slow va de 0.01 a 0.1, con 10 puntos y Moderate va de 0.1 a 0.9, con 80 puntos. 0.01 <= Slow <= 0.1 && 0.1 < Moderate <= 0.9.
			for(i=1;i<=10;i++) a+=(Y[1]*i*0.01);
			for(i=11;i<=90;i++) b+=(Y[2]*i*0.01);
			a+=b;
			a/=((Y[1]*10)+(Y[2]*80));
			return a;
		case 7:
			//La sumatoria va de 0.01 a 1, con 100 puntos; Slow va de 0.01 a 0.1, con 10 puntos; Moderate va de 0.1 a 0.85, con 75 puntos y Fast va de 0.85 a 1, con 15 puntos. 0.01 <= Slow <= 0.1 && 0.1 < Moderate <= 0.85 && 0.85 < Fast <= 1.
			for(i=1;i<=10;i++) a+=(Y[1]*i*0.01);
			for(i=11;i<=85;i++) b+=(Y[2]*i*0.01);
			for(i=86;i<=100;i++) c+=(Y[3]*i*0.01);
			a+=b+c;
			a/=((Y[1]*10)+(Y[2]*75)+(Y[3]*15));
			return a;
		case 8:
			//La sumatoria sólo consta del punto 0.01, que representará siempre el 1% de la señal PWM.
			a=0.01;
			return a;
		case 12:
			//La sumatoria va de 0.01 a 0.35, con 35 puntos; Zero consta de 0.01 y Slow va de 0.01 a 0.35, con 34 puntos. Zero == 0.01 && 0.01 < Slow <= 0.35.
			a=(0.01*Y[0]);
			for(i=2;i<=35;i++) b+=(Y[1]*i*0.01);
			a+=b;
			a/=(Y[0]+(Y[1]*34));
			return a;
		case 14:
			//La sumatoria va de 0.01 a 0.9, con 90 puntos; Zero consta de 0.01, Slow va de 0.01 a 0.1, con 9 puntos y Moderate va de 0.1 a 0.9, con 80 puntos. Zero == 0.01 && 0.01 < Slow <= 0.1 && 0.1 < Moderate <= 0.9.
			a=(0.01*Y[0]);
			for(i=2;i<=10;i++) b+=(Y[1]*i*0.01);
			for(i=11;i<=90;i++) c+=(Y[2]*i*0.01);
			a+=b+c;
			a/=(Y[0]+(Y[1]*9)+(Y[2]*80));
			return a;
		case 15:
			//La sumatoria va de 0.01 a 1, con 100 puntos. Zero consta de 0.01, Slow va de 0.01 a 0.1, con 9 puntos, Moderate va de 0.1 a 0.85, con 75 puntos y Fast va de 0.85 a 1, con 15 puntos. Zero == 0.01 && 0.01 < Slow <= 0.1 && 0.1 < Moderate <= 0.85 && 0.85 < Fast <= 1.
			a=(0.01*Y[0]);
			for(i=2;i<=10;i++) b+=(Y[1]*i*0.01);
			for(i=11;i<=85;i++) c+=(Y[2]*i*0.01);
			for(i=86;i<=100;i++) d+=(Y[3]*i*0.01);
			a+=b+c+d;
			a/=(Y[0]+(Y[1]*9)+(Y[2]*75)+(Y[3]*15));
			return a;
        //
        //
        //
        // Centroide Ventilador.
        case 16:
			a=1.0;
			return a;
		case 17:
			// La sumatoria va de 0 a 0.2, con 20 puntos: 0 <= Slow <= 0.2.
			for(i=1;i<=20;i++) a+=(Y2[0]*i*0.01)/(Y2[0]*20);
			return a;
		case 18:
			// La sumatoria va de 0.1 a 0.8, con 71 puntos: 0.1 <= Moderate <= 0.8.
			for(i=10;i<=80;i++) a+=(Y2[1]*i*0.01)/(Y2[1]*71);
			return a;
		case 19:
			// La sumatoria va de 0 a 0.8, con 80 puntos: Slow va de 0 a 0.1, con 10 puntos y Moderate va de 0.1 a 0.8, con 70 puntos. 0.01 <= Slow <= 0.1; 0.1 < Moderate <= 0.8.
			for(i=1;i<=10;i++) a+=(Y2[0]*i*0.01);
			for(i=11;i<=80;i++) b+=(Y2[1]*i*0.01);
			a+=b;
			a/=((Y2[0]*10)+(Y2[1]*70));
			return a;
		case 20:
			// La sumatoria va de 0.75 a 1, con 26 puntos: 0.75 <= Fast <= 1.
			for(i=75;i<=100;i++) a+=(Y2[2]*i*0.01)/(Y2[2]*26);
			return a;
		case 21:
			// La sumatoria va de 0 a 0.2 y de 0.75 a 1, con 46 puntos. 0 <= Slow <= 0.2; 0.75 <= Fast <= 1.
			for(i=1;i<=20;i++) a+=(Y2[0]*i*0.01);
			for(i=75;i<=100;i++) a+=(Y2[2]*i*0.01);
			a+=b;
			a/=((Y2[0]*20)+(Y2[2]*26));
			return a;
		case 22:
			// La sumatoria va de 0.1 a 1, con 91 puntos: Moderate va de 0.1 a 0.75, con 66 puntos y Fast va de 0.75 a 1, con 25 puntos. 0.1 <= Moderate <= 0.75; 0.75 < Fast <= 1.
			for(i=10;i<=75;i++) a+=(Y2[1]*i*0.01);
			for(i=76;i<=100;i++) a+=(Y2[2]*i*0.01);
			a+=b;
			a/=((Y2[1]*66)+(Y2[2]*25));
			return a;
		case 23:
			// La sumatoria va de 0 a 1, con 100 puntos: Slow va de 0 a 0.1, con 10 puntos; Moderate va de 0.1 a 0.75 con 65 puntos y Fast va de 0.75 a 1, con 25 puntos.
			// 0 < Slow <= 0.1, 0.1 < Moderate <= 0.75, 0.75 < Fast <= 1.
			for(i=1;i<=10;i++) a+=(Y2[0]*i*0.01);
			for(i=11;i<=75;i++) b+=(Y2[1]*i*0.01);
			for(i=76;i<=100;i++) c+=(Y2[2]*i*0.01);
			a+=b+c;
			a/=((Y2[0]*10)+(Y2[1]*65)+(Y2[2]*25));
			return a;
		default:
			break;
	}
}

// FUZZY CONTROL ENDING.

void Alarm(int In)
{
    Condicional=1;
    while(Condicional)
    {
        LATBbits.LB0=0; // Enciende buzzer.
        C(0x01);
        __delay_ms(50);
        D("WARNING!");
        // Añadir tira de LEDs rojos.
        C(0xA8);
        switch(In)
        {
            case 0:
                D("TS is too high");
                break;
            case 1:
                D("TS is too low");
                break;
            case 2:
                D("TA is too high");
                break;
            case 3:
                D("TA is too low");
                break;
        }
        LATBbits.LB0=1; // Enciende buzzer.
        __delay_ms(500);
        C(0x01);
        __delay_ms(50);
        AlarmMod();
    }
    C(0x01);
    __delay_ms(50);
}

void DataTx(unsigned char Variable)
{
    while(!TXSTAbits.TRMT);
    TXREG=Variable;
    __delay_ms(200);
    return;
}

void Keyboard()
{
    
    LATD=0xFF;
    int i=0;
    unsigned char control [4] = {0xEF, 0xDF, 0xBF, 0x7F}; 
    for (int o=0;o<4;o++)
    {
        LATD=control[o];
        // Cuando el circuito está cerrado (push button presionado), el pin está conectado a tierra y se lee como 0.
        // Cuando el circuito está abierto (push button no presionado), el pin está conectado a VDD por medio de la resistencia de pull-up.
        // Los if preguntan por la condición de '0' en las entradas (tecla presionada).
        if (!PORTDbits.RD3)                 // Si RD3 es 0 se recorre la primera columna.
        {
            C(0x01);
            __delay_ms(50);
            switch(o)
            {
                case 3:                     // TECLA 1 TECLADO MATRICIAL
                    Page2();
                    for (i=0;i<2;i++) __delay_ms(500);
                    break;
                case 2:                     // TECLA 4 TECLADO MATRICIAL
                    ForcePWM();
                    break;
                case 1:                     // TECLA 7 TECLADO MATRICIAL
                    ForceHumidifier();
                    break;
                default:
                    break;
            }
            C(0x01);
        }
        if (!PORTDbits.RD2)                 // Si RD2 es 0 se recorre la segunda columna.
        {
            C(0x01);
            __delay_ms(50);
            switch(o)
            {
                case 3:                     // TECLA 2 TECLADO MATRICIAL
                    SetPointMod();
                    break;
                case 2:                     // TECLA 5 TECLADO MATRICIAL
                    TimerMod();
                    break;
                default:
                    break;
            }
            C(0x01);
        }
        if (!PORTDbits.RD1)                 // Si RD1 es 0 se recorre la tercera columna.
        {
            C(0x01);
            __delay_ms(50);
            switch(o)
            {
                case 3:                     // TECLA 3 TECLADO MATRICIAL
                    LEDsMod();
                    break;
                case 2:                     // TECLA 6 TECLADO MATRICIAL
                    ErrorDisplay();
                    for (i=0;i<2;i++) __delay_ms(500);
                    break;
                default:
                    break;
            }
            C(0x01);
        }
    }
    return;
}

void Page2()
{
    C(0x80);
    D("SPS=");
    Conversor(SPS, 0);
       
    C(0xA8);
    D("SPA=");
    Conversor(SPA, 0);
      
    C(0x94);
    D("SPW=");
    Conversor(SPW, 0);
       
    C(0xD4);
    D("SPM=");
    Conversor(SPM, 0);

    C(0x8B);
    D("SPH=");
    Conversor(SPH, 3);
    
    return;
}

void SetPointMod(void)
{
    while(!op)
    {
        C(0x80);
        D("Modify Set Point?");
        C(0xA8);
        D("A) Y   B) N");
        SetPointKeyboard();
    }
    op=0;
    return;
}

void SetPointKeyboard()
{
    unsigned char control [4] = {0xEF, 0xDF, 0xBF, 0x7F}; 
    LATD=0xFF;
    for (int o=0;o<4;o++)
    {
        LATD=control[o];
        if (!PORTDbits.RD0)                 //Si RD0 es 0 se recorre la cuarta columna.
        {               
            C(0x01);
            __delay_ms(50);
            switch(o)
            {
                case 3:                     // TECLA A TECLADO MATRICIAL
                    ChangeSP();
                    op=1;
                    break;
                case 2:                     // TECLA B TECLADO MATRICIAL
                    op=2;
                    break;
                default:
                    break;
            }
            C(0x01);
        }
    }
    return;
}

void LEDsMod(void)
{
    while(!op)
    {
        C(0x80);
        D("LEDS: ");
        C(0xA8);
        D("A) ON   B) OFF");
        LEDsKeyboard();
    }
    op=0;
    return;
}

void LEDsKeyboard()
{
    unsigned char control [4] = {0xEF, 0xDF, 0xBF, 0x7F}; 
    LATD=0xFF;
    for (int o=0;o<4;o++)
    {
        LATD=control[o];
        if (!PORTDbits.RD0)                 //Si RD0 es 0 se recorre la cuarta columna.
        {        
            C(0x01);
            __delay_ms(50);
            switch(o)
            {
                case 3:                     // TECLA A TECLADO MATRICIAL
                    LATBbits.LB5=1;
                    D("LEDs ON.");
                    op=1;
                    break;
                case 2:                     // TECLA B TECLADO MATRICIAL
                    LATBbits.LB5=0;
                    D("LEDs OFF.");
                    op=2;
                    break;
                default:
                    break;
            }
            __delay_ms(500);
            C(0x01);
        }
    }
    return;
}

void AlarmMod(void)
{
    while(!op)
    {
        C(0x80);
        D("Turn alarm off?");
        C(0xA8);
        D("A) Y   B) N");
        AlarmKeyboard();
    }
    op=0;
    return;
}

void AlarmKeyboard()
{
    unsigned char control [4] = {0xEF, 0xDF, 0xBF, 0x7F}; 
    LATD=0xFF;
    for (int o=0;o<4;o++)
    {
        LATD=control[o];
        if (!PORTDbits.RD0)                 //Si RD0 es 0 se recorre la cuarta columna.
        {             
            C(0x01);
            __delay_ms(50);
            switch(o)
            {
                case 3:                     // TECLA A TECLADO MATRICIAL
                    LATBbits.LB0=0; // Apaga buzzer.
                    D("Alarm off.");
                    op=1;
                    Condicional=0;
                    break;
                case 2:                     // TECLA B TECLADO MATRICIAL
                    op=2;
                    break;
                default:
                    break;
            }
            __delay_ms(500);
            C(0x01);
        }
    }
    return;
}

void TimerMod(void)
{
    while(!op)
    {
        C(0x80);
        D("Reset Timer?");
        C(0xA8);
        D("A) Y   B) N");
        TimerKeyboard();
    }
    op=0;
    return;
}

void TimerKeyboard()
{
    unsigned char control [4] = {0xEF, 0xDF, 0xBF, 0x7F}; 
    LATD=0xFF;
    for (int o=0;o<4;o++)
    {
        LATD=control[o];
        if (!PORTDbits.RD0)                 //Si RD0 es 0 se recorre la cuarta columna.
        {   
            C(0x01);
            __delay_ms(50);
            switch(o)
            {
                case 3:                     // TECLA A TECLADO MATRICIAL
                    segundo=segundos=minuto=minutos=hora=0;
                    D("Timer Reset.");
                    op=1;
                    break;
                case 2:                     // TECLA B TECLADO MATRICIAL
                    op=2;
                    break;
                default:
                    break;
            }
            __delay_ms(500);
            C(0x01);
        }
    }
    return;
}

void ChangeSP()
{
    __delay_ms(100);
    while(!op)
    {
        C(0x80);
        D("A) TS   B) TA");
        C(0xA8);
        D("C) TW   D) TM");
        C(0x94);
        D("#) RH");
        SPKeyboard();
    }
    op=0;
    return;
}

void SPKeyboard()
{
    LATD=0xFF;
    int i=0;
    unsigned char control [4] = {0xEF, 0xDF, 0xBF, 0x7F}; 
    while(!op)
    {
    for (int o=0;o<4;o++)
    {
        LATD=control[o];
        if (!PORTDbits.RD0)                 //Si RD0 es 0 se recorre la cuarta columna.
        {                                   
            C(0x01);
            __delay_ms(100);
            switch(o)
            {
                case 3:                     // TECLA A TECLADO MATRICIAL
                    
                    D("Modify SPS");
                    __delay_ms(500);
                    SPS=SPMod(1);
                    while(SPS>37.0 || SPS<20.0)         // ¿Cuáles son los límites del Set Point?
                    {
                        C(0x01);
                        __delay_ms(50);
                        if (SPS>37.0)
                        {
                            D("Error! SPS can't");
                            C(0xA8);
                            D("be too high.");
                            C(0x94);
                            D("Change again");
                            for(i=0;i<2;i++) __delay_ms(500);
                            SPS=SPMod(1);
                        }
                        else
                        {
                            D("Error! SPS can't");
                            C(0xA8);
                            D("be too low.");
                            C(0x94);
                            D("Change again");
                            for(i=0;i<2;i++) __delay_ms(500);
                            SPS=SPMod(1);
                        }
                    }
                    op=1;
                    break;
                case 2:                     // TECLA B TECLADO MATRICIAL
                    D("Modify SPA");
                    __delay_ms(500);
                    SPA=SPMod(1);
                    while(SPA>30.0 || SPA<18.0)
                    {
                        C(0x01);
                        __delay_ms(50);
                        if (SPA>30.0)
                        {
                            D("Error! SPA can't");
                            C(0xA8);
                            D("be too high");
                            C(0x94);
                            D("Change again");
                            for(i=0;i<2;i++) __delay_ms(500);
                            SPA=SPMod(1);
                        }
                        else
                        {
                            D("Error! SPA can't");
                            C(0xA8);
                            D("be too low.");
                            C(0x94);
                            D("Change again");
                            for(i=0;i<2;i++) __delay_ms(500);
                            SPA=SPMod(1);
                        }
                    }
                    op=1;
                    break;
                case 1:                     // TECLA C TECLADO MATRICIAL
                    D("Modify SPW");
                    __delay_ms(500);
                    SPW=SPMod(1);
                    op=1;
                    break;
                case 0:                     // TECLA D TECLADO MATRICIAL
                    D("Modify SPM");
                    __delay_ms(500);
                    SPM=SPMod(1);
                    op=1;
                    break;
                default:
                    break;
            }
            C(0x01);
        }
        if (!PORTDbits.RD1)                 // Si RD1 es 0 se recorre la tercera columna.
        {
            C(0x01);
            __delay_ms(100);
            switch(o)
            {
                case 0:                     // TECLA # MATRICIAL
                    
                    D("Modify SPH");
                    __delay_ms(500);
                    SPH=SPMod(0);
                    C(0x01);
                    op=1;
                    break;
                default:
                    break;
            }
        }
    }
    }
    C(0x01);
    __delay_ms(50);
    D("SetPoint saved.");
    __delay_ms(250);
    return;
}

double SPMod(int Ent)
{
    double R=0.0, R1=0.0, R2=0.0, R3=0.0;
    int z=0;
    C(0x01);
    __delay_ms(50);
    D("Set new SP: ");
    C(0xD6);
    Dato(0x2E);
    C(0xD8);
    if(Ent)
    {
        Dato(0xDF);
        D("C");
    }
    else Dato(0x25);
    C(0xD4);
    while(z<3)
    {
        LATD=0xFF;
        unsigned char control [4] = {0xEF, 0xDF, 0xBF, 0x7F}; 
        for (int o=0;o<4;o++)
        {
            LATD=control[o];
            if (!PORTDbits.RD3)                 // Si RD3 es 0 se recorre la primera columna.
            {
                switch(o)
                {
                    case 3:                     // TECLA 1 TECLADO MATRICIAL
                        if(z==0) R1=10;
                        if(z==1) R2=1;
                        if(z==2) 
                        {
                            R3=0.1;
                            C(0xD7);
                        }
                        D("1");
                        z++;
                        break;
                    case 2:                     // TECLA 4 TECLADO MATRICIAL
                        if(z==0) R1=40;
                        if(z==1) R2=4;
                        if(z==2) 
                        {
                            R3=0.4;
                            C(0xD7);
                        }
                        D("4");
                        z++;
                        break;
                    case 1:                     // TECLA 7 TECLADO MATRICIAL
                        if(z==0) R1=70;
                        if(z==1) R2=7;
                        if(z==2) 
                        {
                            R3=0.7;
                            C(0xD7);
                        }
                        D("7");
                        z++;
                        break;
                }
            }
            if (!PORTDbits.RD2)                 // Si RD2 es 0 se recorre la segunda columna.
            {
                switch(o)
                {
                    case 3:                     // TECLA 2 TECLADO MATRICIAL
                        if(z==0) R1=20;
                        if(z==1) R2=2;
                        if(z==2) 
                        {
                            R3=0.2;
                            C(0xD7);
                        }
                        D("2");
                        z++;
                        break;
                    case 2:                     // TECLA 5 TECLADO MATRICIAL
                        if(z==0) R1=50;
                        if(z==1) R2=5;
                        if(z==2) 
                        {
                            R3=0.5;
                            C(0xD7);
                        }
                        D("5");
                        z++;
                        break;
                    case 1:                     // TECLA 8 TECLADO MATRICIAL
                        if(z==0) R1=80;
                        if(z==1) R2=8;
                        if(z==2) 
                        {
                            R3=0.8;
                            C(0xD7);
                        }
                        D("8");
                        z++;
                        break;
                    case 0:                     // TECLA 0 TECLADO MATRICIAL
                        if(z==0) R1=0;
                        if(z==1) R2=0;
                        if(z==2) 
                        {
                            R3=0;
                            C(0xD7);
                        }
                        D("0");
                        z++;
                        break;
                }
            }
            if (!PORTDbits.RD1)                 // Si RD1 es 0 se recorre la tercera columna.
            {
                switch(o)
                {
                    case 3:                     // TECLA 3 TECLADO MATRICIAL
                        if(z==0) R1=30;
                        if(z==1) R2=3;
                        if(z==2) 
                        {
                            R3=0.3;
                            C(0xD7);
                        }
                        D("3");
                        z++;
                        break;
                    case 2:                     // TECLA 6 TECLADO MATRICIAL
                        if(z==0) R1=60;
                        if(z==1) R2=6;
                        if(z==2) 
                        {
                            R3=0.6;
                            C(0xD7);
                        }
                        D("6");
                        z++;
                        break;
                    case 1:                     // TECLA 9 TECLADO MATRICIAL
                        if(z==0) R1=90;
                        if(z==1) R2=9;
                        if(z==2) 
                        {
                            R3=0.9;
                            C(0xD7);
                        }
                        D("9");
                        z++;
                        break;
                }
            }
            __delay_ms(75);
        }
    }
    R=R1+R2+R3;
    return R;
}

void ErrorDisplay()
{
    if(ES>=0)
    {
        C(0x80);
        D("ES=");
        Conversor(ES, 5);
    }
    else
    {
        C(0x80);
        D("ES=-");
        Conversor(-ES, 5); 
    }
    
    if(EA>=0)
    {
        C(0xA8);
        D("EA=");
        Conversor(EA, 5);
    }
    else
    {
        C(0xA8);
        D("EA=-");
        Conversor(-EA, 5);
    }
    
    if(EW>=0)
    {
        C(0x94);
        D("EW=");
        Conversor(EW, 5);
    }
    else
    {
        C(0x94);
        D("EW=-");
        Conversor(-EW, 5);
    }
    
    if(EM>=0)
    {
        C(0xD4);
        D("EM=");
        Conversor(EM, 5);
    }
    else
    {
        C(0xD4);
        D("EM=-");
        Conversor(-EM, 5);
    }
    
    if(EH>=0)
    {
        C(0x8B);
        D("EH=");
        Conversor(EH, 3);
    }
    else
    {
        C(0x8B);
        D("EH=");
        Conversor(-EH, 3);
    }
    
    C(0xCA);
    D("PWM=");
    if(PWM==1) Conversor(PWM*100, 2);
    else Conversor(PWM*100, 1);
    
    C(0x9E);                            //  Segunda línea del LCD.
    D("Fan=");
    Conversor (FPWM*100, 1);
    return;
}

void ForcePWM(void)
{
    while(!op)
    {
        C(0x80);
        D("Update PWM?");
        C(0xA8);
        D("A) Y   B) N");
        PWMMod();
    }
    op=0;
    return;
}

void PWMMod()
{
    unsigned char control [4] = {0xEF, 0xDF, 0xBF, 0x7F}; 
    LATD=0xFF;
    for (int o=0;o<4;o++)
    {
        LATD=control[o];
        if (!PORTDbits.RD0)                 //Si RD0 es 0 se recorre la cuarta columna.
        {            
            C(0x01);
            __delay_ms(50);
            switch(o)
            {
                case 3:                     // TECLA A TECLADO MATRICIAL
                    FuzzyControl(ES, EA, EW, EM);
                    RHControl();
                    DataTx(cPWM);                       // Transmisión de PWM de la resistencia calefactora.
                    DataTx(cFPWM);                      // Transmisión de PWM del ventilador.
                    D("PWM updated.");
                    op=1;
                    break;
                case 2:                     // TECLA B TECLADO MATRICIAL
                    op=2;
                    break;
                default:
                    break;
            }
            __delay_ms(500);
            C(0x01);
        }
    }
    return;
}

void ForceHumidifier(void)
{
    while(!op)
    {
        C(0x80);
        D("Turn humidifier on?");
        C(0xA8);
        D("A) Y   B) N");
        HumidifierMod();
    }
    op=0;
    return;
}

void HumidifierMod()        // Revisar cuánto tiempo debe permanecer encendido el humidifcador.
{
    unsigned char control [4] = {0xEF, 0xDF, 0xBF, 0x7F}; 
    LATD=0xFF;
    for (int o=0;o<4;o++)
    {
        LATD=control[o];
        if (!PORTDbits.RD0)                 //Si RD0 es 0 se recorre la cuarta columna.
        {          
            C(0x01);
            __delay_ms(50);
            switch(o)
            {
                case 3:                     // TECLA A TECLADO MATRICIAL
                    RHControl();
                    op=1;
                    break;
                case 2:                     // TECLA B TECLADO MATRICIAL
                    op=2;
                    break;
                default:
                    break;
            }
            __delay_ms(500);
            C(0x01);
        }
    }
    return;
}
