# Código Tesis

Código para programar dos microcontroladores PIC18F4550. 

El microcontrolador maestro ejecuta las siguientes funciones:

* Recolección y procesamiento de datos de los sensores de temperatura.
* Proceso de Control Lógico Difuso.
* Receptor de datos por comunicación UART.
* Envío de datos del PWM para resistencia y ventilador por UART.
* Interfaz del usuario.
* Administración del estado de la tira de LEDs y el humidificador.
* Administración de alarmas audiovisuales.

La configuración del PIC18F4550 es la siguiente:

* Configuración del Puerto A como Salidas Digitales para operar el LCD.
* Configuración del pin RB0 para encendido y apagado de buzzer.
* Configuración de los pines RB1 y RB3 para celdas de carga.
* Configuración de los pines RB4 y RB5 para sensor de nivel y tira de LEDs, respectivamente.
* Configuración del pin RC0 para encendido de humidificador.
Configuración de los pines RC1 y RC2 como salidas para RS y E, respectivamente, del LCD.
* Configuración de los pines RC6 y RC7 del Puerto C como I/O del módulo EUSART.
* Configuración del Puerto D para el teclado matricial. Nibble alto para salidas y nibble bajo para entradas.
Configuración del Puerto E y Puerto B para Entradas Analógicas AN5-AN9.
* Oscilador interno funcionando a 8MHz.
* Tiempo de Conversión $\mathrm{F_{OSC}}/64, T_{AD} = 8\mu s$
* Tiempo de Adquisición = $\mathrm{2T_{AD} = 16\mu s}$
* Timer0 como Temporizador para desbordamiento cada 1s. Este desbordamiento genera la interrupción encargadad del despliegue gráfico del Temporizador.
* Timer1 como Temporizador para desbordamiento cada 5ms, con el propósito de enviar datos y comandos al LCD.

El segundo PIC18F4550 tiene la función de adquisición de datos del sensor de humedad BME280 y transmisión de estos al otro PIC18F4550, por lo que se realizó la siguiente configuración:
* Configuración de los pines RB0 y RB1 para transmisión de datos por $ \mathrm{I²C} $
* Configuración de los pines RC6 y RC7 del Puerto C como I/O del módulo EUSART.