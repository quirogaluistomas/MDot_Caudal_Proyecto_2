// Software compatible con la aplicación Configuracion_V1 

// Realiza tantos intentos como se configure cada cierto tiempo random para evitar colisiones.
// Se pueden setear los tiempos random superior e inferior, entre ciertos límites. TIEMPO_RANDOM_SUP y TIEMPO_RANDOM_INF

// Se ingresa en modo confuguracion si se preciona S2 (boton incorporado en la placa) al alimentarlo
// la configuracion se guarda en la memoria no volatil

// Se incorporó los tiempo de reintento de enlace con el gateway (Tiempo_reintento_join y Tiempo_reintento_join_2)
// el Dot se considera desconectado despues de una cantidaad threshold_fallos de fallos en el envio de datos

// NOTA 1: una vez que se toma la muestra se intenta enviarlo hasta 6 veces, si falla en los 6 intentos el dato se pierde.
// NOTA 2: la funcion sleep_wake_rtc_only fue modificada para poder pasarle como parametro el tiempo de sleep
// NOTA 3: la funcion printf solo debe usarse para enviarle datos a la aplicacion. Para mostrar por serie utilizar 

#include "dot_util.h"
#include "RadioEvent.h"
#include "mbed.h" 
#include "mDot.h"



#define UMBRAL_SUP 1300 //El rango en cuentas de las AI es de 0 a 4096, si VDD=3.3V -> 1300 => 1.04V
#define UMBRAL_INF 1000 //El rango en cuentas de las AI es de 0 a 4096, si VDD=3.3V -> 1000 => 0.8V

#define ADC_RES_bits 12 //Resolución del ADC del mDot

// Se obtiene las cuentas corriendo binariamente la cantidad de bits de resolucion del ADC
#define ADC_RES_Cuentas ((1<<ADC_RES_bits)-1)


// Por defecto se coloca en banda australiana 915Mhz
#if !defined(CHANNEL_PLAN)
#define CHANNEL_PLAN CP_AU915
#endif





///*********DEFINICION DE VARIABLES Y PARAMETROS********////////

#define CLOCK 96    //frec del micro en MHZ

int cantidad_AI=8;

//Creo arreglos para guardar las frecuencias de los canales leidos, el original en flotante y otro en entero para la resolución
float frec_ch[8];
int frec_ch_int[8];

int flag_configuracion=0;
//int TIEMPOS[5];

int CONFIGURACION[15];
//[0] TIEMPO_SLEEP;
//[1] unidad_TIEMPO_SLEEP;
//[2] cantidad_REINTENTOS;
//[3] TIEMPO_REINTENTOS_CONSECUTIVOS;
//[4] TIEMPO_RANDOM_SUP;
//[5] TIEMPO_RANDOM_INF;
//[6] seleccion_frec;
//[7] modo_AI[0];
//[8] modo_AI[1];
//[9] modo_AI[2];
//[10] modo_AI[3];
//[11] modo_AI[4];
//[12] modo_AI[5];
//[13] modo_AI[6];
//[14] modo_AI[7];

//float *K = new float[cantidad_AI]; // Toma las constantes de cada caudalímetro.
float K[8]; // Toma las constantes de cada caudalímetro.

int TIEMPO_SLEEP; //En realidad es el tiempo entre envíos, no el tiempo que está durmiendo
int unidad_TIEMPO_SLEEP;
int cantidad_REINTENTOS;
int TIEMPO_RANDOM_SUP;
int TIEMPO_RANDOM_INF;

mDot::mdot_file nombre_file;

int frecSeleccionada;

//Se crea vector de estados de Analog Inputs
//int *modo_AI = new int[cantidad_AI]; // 0: Deshabilitada 1: Habilitada
int modo_AI[8];// 0: Deshabilitada 1: Habilitada

uint32_t Tiempo_reintento_join=360; // en seg no sé qué es
uint32_t Tiempo_reintento_join_2=600; // en seg no sé qué es

int cortePanel = 1; //Variable flag que indicará si se ha cortado la tensión de panel

/////////////////////////////////////////////////////////////
// * these options must match the settings on your gateway //
// * edit their values to match your configuration         //
// * frequency sub band is only relevant for the 915 bands //
// * either the network name and passphrase can be used or //
//     the network ID (8 bytes) and KEY (16 bytes)         //
/////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// * estas opciones deben coincidir con la configuración de su gateway  //
// * edite sus valores para que coincidan con su configuración          //
// * la sub banda de frecuencia solo es relevante para la banda 915     //
// * se puede usar el network name y la passphrase o                    //
//     la network ID (8 bytes) y KEY (16 bytes)                         //
//////////////////////////////////////////////////////////////////////////
static std::string network_name = "MultiTech";
static std::string network_passphrase = "MultiTech";
static uint8_t network_id[] = { 0x6C, 0x4E, 0xEF, 0x66, 0xF4, 0x79, 0x86, 0xA6 };
static uint8_t network_key[] = { 0x1F, 0x33, 0xA1, 0x70, 0xA5, 0xF1, 0xFD, 0xA0, 0xAB, 0x69, 0x7A, 0xAE, 0x2B, 0x95, 0x91, 0x6B };
static uint8_t frequency_sub_band = 1;
static lora::NetworkType network_type = lora::PUBLIC_LORAWAN;
static uint8_t join_delay = 5;
static uint8_t ack = 1; //0 if acks are disabled, otherwise retries (1 - 8)
static bool adr = true;



// en sleep mode, el estado IO es mantenido, la RAM es retenida , y la aplicación se reanudará después de despertarse
// en deepsleep mode, IOs flotantes, RAM se pierde, y la aplicación comenzará desde el principio después de despertarse
// si deep_sleep == true, el dispositivo ingresará en deepsleep mode
static bool sleepMode = false;
//static bool deep_sleep = true;

mDot* dot = NULL;
lora::ChannelPlan* plan = NULL;

//Se crea objeto para puerto serial

Serial pc(USBTX, USBRX);

// AI a utilizar
AnalogIn AI_1(PB_1);  // Pin 20
AnalogIn AI_2(PB_0);  // Pin 19
AnalogIn AI_3(PA_5);  // Pin 18
AnalogIn AI_4(PA_4);  // Pin 17
AnalogIn AI_5(PA_1);  // Pin 16
AnalogIn AI_6(PC_1);  // Pin 15
AnalogIn AI_7(PA_2);  // Pin 2
AnalogIn AI_8(PA_7);  // Pin 11


// DI a utilizar
DigitalIn DI_Config(PA_6, PullNone); // Pin 4
DigitalIn DI_Interruption(PA_0, PullNone); // Pin 12 Este pin es el único que me permite usarlo para que se despierte frente a interrupción

//Entrada de interrupción real a utilizar
InterruptIn DI_Panel(PA_3, PullNone); // Pin 3 Este es el que estará puenteado al PA_0 que es el que permite despertar.

// DO a utilizar
DigitalOut Led_Blue(PC_13); // Pin 13. Este será un diodo azul que indica que está en modo config
DigitalOut Led_Red(PC_9); // Pin 7. Este será un diodo rojo que indicará fallas en escritura
DigitalOut Led_Green(PA_8); // Pin 6. Este será el Led verde que indicará que está todo ok

Timer timer1;
Timer timer2;

/////*************Declaro las funciones a utilizar**********************////
void error_1(class mbed::DigitalOut led);

//void error_2(class mbed::DigitalOut led);

void entrada_modo_config(class mbed::DigitalOut led);

float funcion_calcularFrec (int);

void func_configuracion(void);

bool func_leer_config(void);

void interruptPanel_fall(void);

void interruptPanel_rise(void);

//******************DEFINICION DE FUNCIONES*******************////

void error_1(class mbed::DigitalOut led)
{
    // Led Rojo parpadeando indica error en escritura
    while (1) {
        led.write(1);
        wait(1);
        led.write(0);
        wait(1);
    }   
}

/*void error_2(class mbed::DigitalOut led)
{
    while(1)
    {
        //parpadeo 2: intermitente cada 1 seg
        // indica que fallo la lectura de la configuracion o que los datos no son correctos
        led.write(1);
        wait(1);
        led.write(0);
        wait(1);
        
    }
}*/

void entrada_modo_config(class mbed::DigitalOut led)
{
    // indica que se ingreso en el modo configuracion. Se prende 5 segundos y se apaga.
    led.write(1);
    wait(5);
    led.write(0);
   
}

float funcion_calcularFrec (int ADC_leer) //Lee 8 adc distintos. Se le indica cuál es el de interés
{

    uint16_t muestras; //Cantidad de muestras tomadas por el adc con la función read
    int tiempo_total=0, tiempo_inicio=0; 
    float frecHZ;
    int contador = 0;
    int flag = 0;
    float ventana;

    
    // Tomo las muestras y comparo con umbral hasta completar 40 ciclos
    // tambien tomo el tiempo 
    timer2.reset();
    timer2.start();
    while(contador<41)
    {
            switch(ADC_leer)
            {
                //Las muestras se guardan en cuentas -> rango de AI es 0 a 1.0 , Rango Cuentas es 0 a 4096
                case 1:
                        muestras=(uint16_t) (ADC_RES_Cuentas*AI_1.read());
                        break;
                case 2:
                        muestras=(uint16_t) (ADC_RES_Cuentas*AI_2.read());
                        break;
                case 3:
                        muestras=(uint16_t) (ADC_RES_Cuentas*AI_3.read());
                        break;
                case 4:
                        muestras=(uint16_t) (ADC_RES_Cuentas*AI_4.read());
                        break;
                case 5:
                        muestras=(uint16_t) (ADC_RES_Cuentas*AI_5.read());
                        break;
                case 6:
                        muestras=(uint16_t) (ADC_RES_Cuentas*AI_6.read());
                        break; 
                case 7:
                        muestras=(uint16_t) (ADC_RES_Cuentas*AI_7.read());
                        break;
                case 8:
                        muestras=(uint16_t) (ADC_RES_Cuentas*AI_8.read());
                        break;  
            }

            // Si la señal supera el umbral se activa el flag y cuenta el ascenso.
            // Es decir cuando por primera vez en el ciclo al umbral superior es superado.
            if ((muestras > UMBRAL_SUP) && (flag == 0))
            {
                contador++;
                flag = 1;
                if(contador==1)
                {
                    //tiempo desde que inicio desde que paso el umbral sup por primera vez
                    // se debe restar este tiempo para el calculo de frec
                    tiempo_inicio=timer2.read_us();
                }
            }
    
            // Si la señal es menor al umbral inferior
            if ((muestras < UMBRAL_SUP) && (muestras < UMBRAL_INF))
            {
                flag = 0;
            }
            
            // Vuelvo a leer el tiempo en us y me fijo si pasó 1 segundo y todavía no llegó ni a la mitad de ciclos que
            // tendría la mas baja frecuencia entonces corto porque o no hay señal o es una frecuencia no permitida
            // y salgo del while

            if((timer2.read_us() > 1000000) && (contador < 21))
            {   
                timer2.stop();           
                return 0;
            } 
    }
        
        
    // Tiempo final de la toma de muestras y conteo de 40 flancos
    tiempo_total=timer2.read_us();
    pc.printf("total_time [us]=%d \n\r\n",tiempo_total);
    timer2.stop();
    
    // Tiempo efectivo de los 40 ciclos desde que detecta el primero hasta el último.
    ventana= float(tiempo_total-tiempo_inicio);   
    pc.printf("ventana [us]=%f \n\r\n",ventana);
      
    //Al numero de umbrales obtenidos le resto 1 para que me de el numero de periodos correspondientes.
    if(contador!=0)
        contador = contador-1;

    //Paso a Hz debido a que la ventana de tiempo está en us.
    frecHZ = 1000000*float(contador)/ventana;
    pc.printf("frec=%f\n\r\n",frecHZ);
    
    return (frecHZ);

}

void func_configuracion()
{
        
        int indice_conf = 0; //Para ver qué quiere configurar.
        int tiempoSleep = 0; // Lo inicializo con un valor por si se entra al modo config y no se setea esto.
        //int aux = 0; // Esta variable se utilizará para guardar en archivo
        int flag_error_write=0; // Flag de error de lectura
        int cantReintentos = 0; //Variable que contiene cantidad de reintentos.
        int tiempoSuperior;
        int tiempoInferior;
        int configFrecuencia;
        int aux = 0; //Variable a utilizar en la toma de dato de habilitación de AI
        float aux2 = 0; // Variable a utilizar para los coeficientes K
        int valor_aux=0;
        int valor_aux_2=0;
        //char ok[3]="\0";
        int ok;
         
        // Existe un struct que tiene una variable fd con tipo de dato int16. Si ese valor es negativo implica que
        // hay error en la carga de archivo.
        if(nombre_file.fd<0)
        {
            printf("ERROR en el archivo\n\r");
        }

        entrada_modo_config(Led_Blue); //Acá debería parpadear el led azul que te indica que estás en el modo configuración.

        // Se queda esperando que el usuario ponga "ok" en el inicio.
        pc.printf("Usted se encuentra en el modo configuracion. Ingrese '0' para continuar.\n\r");
        pc.scanf("%d",&ok);

        if(ok == 0)
        {
                pc.printf("%d_\n\r", cantidad_AI);     //Calculo que esto es solo para printear cuantas entradas AI hay. Se podría omitir o agregar mas info         
        }

        

        while(1)
        {
            
            char cadena2[8]="\0";
            char cadena3[3]="\0";
            char cadena4[2]="\0";

            int modo_AI_conf[cantidad_AI];
            float K_conf[cantidad_AI];

            // Se queda esperando en consola a que llegue el indicador
            pc.printf("Indique el numero correspondiente de las siguientes opciones para configurar:\n\r");
            pc.printf("1 - Tiempo en el que permanecerá en Modo Sleep.\n\r"); //ünica opción será en sleep.
            pc.printf("2 - Cantidad de reintentos en caso de falla de envio de dato.\n\r"); //Serán 2 por defecto
            pc.printf("3 - Limites superior de intervalo de tiempo de reintentos.\n\r"); // Esto se hará de manera aleatoria.
            pc.printf("4 - Limites inferior de intervalo de tiempo de reintentos.\n\r"); // Esto se hará de manera aleatoria.
            pc.printf("5 - Banda de Frecuencia de Operacion\n\r");
            pc.printf("6 - Configuracion de Entradas Analogicas\n\r");
            pc.scanf("%d", &indice_conf);
            pc.printf("%d\n\r", indice_conf);

            switch(indice_conf)
            {
                case 1:
                        // TIEMPO_SLEEP
                        pc.printf("Ingrese el tiempo de sleep en segundos: \n\r");
                        pc.scanf("%d", &tiempoSleep);
                        pc.printf("%d\n\r", tiempoSleep);

                        //pc.scanf("%d",cadena2);
                        //pc.scanf("%s",cadena3);                   
                        //pc.scanf("%s",cadena4);
                        // msj 1 : digito1 y digito2
                        // msj 2 : digito3 
                        // msj 3 : unidad

                        //if(strcmp(cadena3, "_")!=0) //tres digitos
                        //    strcat(cadena2, cadena3); // dato de tiempo completo                         


                        // paso el tiempo a segundos
                        //valor_aux = std::atoi(cadena2);

                        //valor_aux_2 = valor_aux * int(std::pow(double(60),(std::atoi(cadena4)))); 
                        
                        
                        // Escribe valor temporal
                        dot->seekUserFile(nombre_file, 0, 0);
                        flag_error_write = dot->writeUserFile(nombre_file, &tiempoSleep, sizeof(tiempoSleep)); // Devuelve -1 si hay error. //Se guarda en memoria flash (no volátil)

                        // Escribe unidad de tiempo [hora min seg]
                        //valor_aux=std::atoi(cadena4) ; 
                        //dot->seekUserFile(nombre_file, sizeof(int), 0);
                        if(flag_error_write < 0) // Si falla la escritura
                            error_1(Led_Red);  //Enciende Led Rojo intermitente 
                        break;

                case 2:
                        // TIEMPO_REINTENTOS_CONSECUTIVOS
                        //char cadena_aux[2];
                        //pc.scanf("%s",cadena_aux);
                        // msj 1 : num_reintentos
                        //valor_aux= std::atoi(cadena_aux);// cantidad de reintentos
                        pc.printf("Ingrese la cantidad de reintentos en caso de que falle la comunicacion: \n\r");
                        pc.scanf("%d", &cantReintentos);
                        pc.printf("%d\n\r", cantReintentos);

                        if(cantReintentos!=0)
                        {
                            //pc.scanf("%s",cadena2);
                            //pc.scanf("%s",cadena3);
                            
                            // msj 2 : digito1 y digito2
                            // msj 3 : digito3
                       
                            //if(strcmp(cadena3, "_")!=0)
                            //    strcat(cadena2, cadena3);
                            
                            //valor_aux_2= std::atoi(cadena2);// tiempo en segundos

                        }else
                        {
                            //valor_aux_2=0; // si los reintentos son 0, el tiempo de reintento tambien es 0
                        }

                        //dot->seekUserFile(nombre_file, sizeof(int)*2, 0);
                        dot->seekUserFile(nombre_file, sizeof(int), 0);
                        flag_error_write = dot->writeUserFile(nombre_file, &cantReintentos, sizeof(cantReintentos)); //Se guarda en memoria flash (no volátil)

                        //dot->seekUserFile(nombre_file, sizeof(int)*3, 0);
                        if(flag_error_write<0) //Si falla la escritura
                            error_1(Led_Red); //Enciende Led Rojo intermitente
                        break;
                
                case 3:
                        // TIEMPO_RANDOM_SUP
                        pc.printf("Ingrese el limite de tiempo superior: \n\r");
                        pc.scanf("%d", &tiempoSuperior);
                        pc.printf("%d\n\r", tiempoSuperior);
                        // pc.scanf("%s",cadena2);
                        // pc.scanf("%s",cadena3);
                        // msj 1 : digito1 y digito2
                        // msj 2 : digito3

                        //if(strcmp(cadena3, "_")!=0)
                        //    strcat(cadena2, cadena3);

                        //valor_aux= std::atoi(cadena2);// tiempo en segundos
                        dot->seekUserFile(nombre_file, sizeof(int)*2, 0);
                        flag_error_write = dot->writeUserFile(nombre_file, &tiempoSuperior, sizeof(tiempoSuperior));
                        if( flag_error_write < 0)
                            error_1(Led_Red); //Enciende Led Rojo intermitente si falla la escritura
                        break;

                case 4:
                        //TIEMPO_RANDOM_INF
                        pc.printf("\n\rIngrese el limite de tiempo inferior: \n\r");
                        pc.scanf("%d", &tiempoInferior);
                        pc.printf("%d\n\r", tiempoInferior);
                        //pc.scanf("%s",cadena2);
                        //pc.scanf("%s",cadena3);
                        // msj 1 : digito1 y digito2
                        // msj 2 : digito3

                        //if(strcmp(cadena3, "_")!=0)
                        //    strcat(cadena2, cadena3);

                        //valor_aux= std::atoi(cadena2);// tiempo en segundos
                        dot->seekUserFile(nombre_file, sizeof(int)*3, 0);
                        flag_error_write = dot->writeUserFile(nombre_file, &tiempoInferior, sizeof(tiempoInferior));
                        if(flag_error_write < 0)
                            error_1(Led_Red); //Enciende Led Rojo intermitente si falla la escritura
                        break;

                case 5:
                        // Configuracion frec
                        //pc.scanf("%s",cadena2);
                        //valor_aux= std::atoi(cadena2);
                        pc.printf("1 - AU915 \n\r");
                        pc.printf("2 - US915 \n\r");
                        pc.scanf("%d", &configFrecuencia);
                        pc.printf("%d\n\r", configFrecuencia);

                        dot->seekUserFile(nombre_file, sizeof(int)*4, 0);
                        flag_error_write = dot->writeUserFile(nombre_file, &configFrecuencia, sizeof(configFrecuencia));

                        if(flag_error_write < 0) //se guarda en la memoria no volatil
                            error_1(Led_Red); //Enciende Led Rojo intermitente si falla la escritura
                        break;
                
                case 6:
                        // Configuracion AI
                        int i;
                        int j;
                        pc.printf("Ingrese 1 para habilitar, 0 para deshabilitar\n\r");
                        
                        // Configuración de AI --> habilitada o deshabilitada
                        for(i=0;i<cantidad_AI;i++)
                        {
                            pc.printf("Habilitar entrada de caudal %d \n\r", i+1);
                            pc.scanf("%d",&aux);
                            pc.printf("%d\n\r", aux);
                            //pc.scanf("%s",cadena2);
                            //modo_AI_aux[i] = std::atoi(cadena2);
                            modo_AI_conf[i] = aux;
                        }

                        // Pido los coeficientes K 
                        for(i=0;i<cantidad_AI;i++)
                        {
                            pc.printf("Ingrese coeficiente K para la entrada %d \n\r", i+1);
                            pc.scanf("%f",&aux2);
                            K_conf[i] = aux2;
                            pc.printf("%f\n\r", aux2);
                        }


                        dot->seekUserFile(nombre_file,  sizeof(int)*5, 0);
                        flag_error_write=dot->writeUserFile(nombre_file, &modo_AI_conf, sizeof(modo_AI_conf));
                        dot->seekUserFile(nombre_file,  sizeof(int)*13, 0); // Me paro luego de guardar los modos AI
                        
                        if(dot->writeUserFile(nombre_file, &K_conf, sizeof(K_conf))<0 || flag_error_write<0) //se guarda en la memoria no volatil
                            error_1(Led_Red); //Enciende Led Rojo intermitente si falla la escritura
                        break;

                //case 7:
                        //Envio de configuracion AppKey y DevKey
                //        int k;
                //        for(k=0;k<16; k++)
                //        {
                //            pc.printf("%X", network_key[k]);
                //        }
                //        pc.printf("_");

                //        for(k=0;k<8; k++)
                //        {
                //            pc.printf("%X", network_id[k]);
                //        }
                                 
                //        break;
            }
        }

}
        

bool func_leer_config(void)
{
    //Lectura de configuracion guardada

    bool error_config = false;
    int flag_leer = 0;

    dot->seekUserFile(nombre_file, 0, 0); // Me posiciono al principio del archivo
    flag_leer = dot->readUserFile(nombre_file, &CONFIGURACION, sizeof(CONFIGURACION));
    dot->seekUserFile(nombre_file, sizeof(int)*12, 0);// Me posiciono donde termina CONFIGURACION, que sólo mostrará hasta los modos, no los K
    if(dot->readUserFile(nombre_file, &K, sizeof(K))<0 || flag_leer<0)//leo todo
    {
        logInfo("ERROR: no se pudo leer la configuracion\n\r");
        error_config= true;
    }else
    {

        // Si se leyo correctamente analizo los datos
        TIEMPO_SLEEP = CONFIGURACION[0];
        //unidad_TIEMPO_SLEEP=CONFIGURACION[1];
        cantidad_REINTENTOS = CONFIGURACION[1];
        //TIEMPO_REINTENTOS_CONSECUTIVOS=CONFIGURACION[3];
        TIEMPO_RANDOM_SUP = CONFIGURACION[2];
        TIEMPO_RANDOM_INF = CONFIGURACION[3];
        frecSeleccionada = CONFIGURACION[4];
        modo_AI[0] = CONFIGURACION[5];
        modo_AI[1] = CONFIGURACION[6];
        modo_AI[2] = CONFIGURACION[7];
        modo_AI[3] = CONFIGURACION[8];  
        modo_AI[4] = CONFIGURACION[9];
        modo_AI[5] = CONFIGURACION[10];
        modo_AI[6] = CONFIGURACION[11];
        modo_AI[7] = CONFIGURACION[12];


        logInfo("TIEMPO_SLEEP = %d\n\r", TIEMPO_SLEEP); //Este en realidad es el tiempo entre envios
        //logInfo("unidad_TIEMPO_SLEEP = %d\n\r", unidad_TIEMPO_SLEEP);
        logInfo("cantidad_REINTENTOS = %d\n\r", cantidad_REINTENTOS);
        //logInfo("TIEMPO_REINTENTOS_CONSECUTIVOS = %d\n\r",  TIEMPO_REINTENTOS_CONSECUTIVOS);
        logInfo("TIEMPO_RANDOM_SUP = %d\n\r",  TIEMPO_RANDOM_SUP);
        logInfo("TIEMPO_RANDOM_INF = %d\n\r",  TIEMPO_RANDOM_INF);

        
        if(cantidad_REINTENTOS==0)
        {
            if(TIEMPO_SLEEP <= 0)
            {
                logInfo("ERROR en la configuracion \n\r");
                error_config= true;
            }

        }else
        {
            //if(TIEMPO_SLEEP <= (TIEMPO_RANDOM_SUP +  (cantidad_REINTENTOS + 1)))
            if(TIEMPO_SLEEP <= (TIEMPO_RANDOM_SUP *(cantidad_REINTENTOS + 1))) // Tomo el peor caso: TiempoRandomSup y pido que sea mayor a esta condición
            {
                logInfo("ERROR en la configuracion \n\r");
                error_config= true;
            }
            
            if(TIEMPO_RANDOM_SUP>300)// 5min
            {
                logInfo("ERROR en la configuracion \n\r");
                error_config= true;
            }

            if(TIEMPO_RANDOM_INF<30)// 30 seg
            {
                logInfo("ERROR en la configuracion\n\r");
                error_config= true;
            }
        }
          

        switch(frecSeleccionada)
        {
            // A
            case 1:
                            //CHANNEL_PLAN CP_US915;
                            plan = new lora::ChannelPlan_AU915();
                            logInfo("\n\rUsted ha seleccionado AU915\n\r");
                            break;
                    
            // B
            case 2:
                            //CHANNEL_PLAN CP_AU915;
                            plan = new lora::ChannelPlan_US915();
                            logInfo("\n\rUsted ha seleccionado US915\n\r");                           
                            break;
            /*        
            // C
            case 67:
                            //CHANNEL_PLAN CP_EU868;
                            plan = new lora::ChannelPlan_EU868();
                            logInfo("\n\rSe selecciono EU868\n\r");
                            break;
                    
            // D
            case 68:
                            //CHANNEL_PLAN CP_KR920;
                            plan = new lora::ChannelPlan_KR920();
                            logInfo("\n\rSe selecciono KR920\n\r");
                            break;
                    
            // E
            case 69:
                            //CHANNEL_PLAN CP_AS923;
                            plan = new lora::ChannelPlan_AS923();
                            logInfo("\n\rSe selecciono AS923\n\r");
                            break;
                    
            // F
            case 70:
                            //CHANNEL_PLAN CP_AS923_JAPAN;
                            plan = new lora::ChannelPlan_AS923_Japan();
                            logInfo("\n\rSe selecciono AS923_JAPAN\n\r");
                            break;
            // G
            case 71:
                            //CHANNEL_PLAN CP_IN865;
                            plan = new lora::ChannelPlan_IN865();
                            logInfo("\n\rSe selecciono IN865\n\r");
                            break;
            */                
            default: 
                            logInfo("\n\rEl valor ingresado no es valido\n\r");
                            logInfo("\n\rfrecSeleccionada= %d\n\r", frecSeleccionada);
                            error_config = true;
        
        }

        int i=0;
        // Verificacion de conf AI
        for(i=0;i<cantidad_AI;i++)
        {
            if( modo_AI[i]==1 ) 
            {
                logInfo("AI_%d : Habilitada\n\r",i+1);
            }
            else if (modo_AI[i]==0) 
            {
                logInfo("AI_%d : Desabilitada\n\r",i+1);

            }else {
                error_config = true;
            }
                
        }
        Tiempo_reintento_join_2=TIEMPO_SLEEP; //Evaluar que onda esto
    }
    
    return error_config;
}

void interruptPanel_fall(void)
{
    int leerPanel;
    leerPanel = DI_Panel.read();
    if (leerPanel == 0)
    {
        cortePanel = 0;
    }
    //cortePanel = 0;
}

void interruptPanel_rise(void)
{
    int leerPanel;
    leerPanel = DI_Panel.read();
    if (leerPanel == 1)
    {
        cortePanel = 1;
    }
    //cortePanel = 1;
}

//////////////////////////////////////////   
        
 int main() {
    
    pc.baud(115200);
    //int test = 0; //Eliminar esta linea y reemplazar en el if por DI_conf
    plan = new lora::ChannelPlan_US915();//por defecto para que arranque se modifica con la configuracion el valor de plan
    MBED_ASSERT(plan);

    dot = mDot::getInstance(plan);

    MBED_ASSERT(dot);

    nombre_file = dot->openUserFile("ArchivoConfig.txt", (1 << 3)|(1 << 4)); // Crea un archivo y setea los bit 3 y 4 en 1.

    if(!dot->seekUserFile(nombre_file, 0, 0))// Si es la primera vez que entre al if y cree el archivo
    {
        dot->saveUserFile("ArchivoConfig.txt", CONFIGURACION, sizeof(CONFIGURACION) + sizeof(float)*2);
        nombre_file = dot->openUserFile("ArchivoConfig.txt", (1 << 3)|(1 << 4));
    }
        
    //pc.printf("Ingrese 1 para cargar conf: \n\r"); // Hay que eliminar esta parte
    //pc.scanf("%d\n\r", &test); // Hay que eliminar esta parte, sólo es para prueba

    if(DI_Config)   
    {
        // Configuracion
        func_configuracion();     
        
    }else
    {
        
        // Controlador de eventos personalizado para mostrar automáticamente datos RX
        RadioEvent events;
        
        mts::MTSLog::setLogLevel(mts::MTSLog::TRACE_LEVEL); 

        if (func_leer_config())
        {
            //si la configuracion no se puedo leer correctamente
            logInfo("ERROR al leer la configuracion\n\r");
            error_1(Led_Red); //Enciende Led Rojo intermitente si falla la lectura
            
        }

        uint8_t threshold_fallos = 2*(cantidad_REINTENTOS+1); // Eso indica cuántos fallos son admisibles para considerarse desconectado
        // si reintentos es 0 threshold_fallos=2  lo que equivale a dos pasadas
        // si reintentos es 1 threshold_fallos=6  lo que equivale a dos pasadas completas de intentos
        // si reintentos es 2 threshold_fallos=10  lo que equivale a dos pasadas completas de intentos
        
        dot->setChannelPlan(plan); // Acá se setea el ChannelPlan en función de lo que se guarde en el archivo. AU, US, EU, etc..
        
        // Setea el controlador de eventos personalizados
        dot->setEvents(&events);
        
        if (!dot->getStandbyFlag()) {
            //logInfo("mbed-os library version: %d.%d.%d", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
        
            //Comenzar desde un estado conocido
            //logInfo("defaulting Dot configuration");
            dot->resetConfig();
            dot->resetNetworkSession();
        
            // Asegurarse de que el registro de la biblioteca esté activado
            dot->setLogLevel(mts::MTSLog::INFO_LEVEL);
        
            // Update configuration if necessary 
            if (dot->getJoinMode() != mDot::OTA) {
                //logInfo("changing network join mode to OTA"); Se une al modo de unión de enlace Over the Air
                if (dot->setJoinMode(mDot::OTA) != mDot::MDOT_OK) {
                    //logError("failed to set network join mode to OTA");
                }
            }
                
        
            // En los modos de unión OTA y AUTO_OTA, las credenciales se pueden pasar a la biblioteca como un nombre y contraseña o un ID y KEY
            // solo un metodo o el otro puede ser usado
            // network ID = crc64(network name)
            // network KEY = cmac(network passphrase)
            update_ota_config_id_key(network_id, network_key, frequency_sub_band, network_type, ack);
        
        
            // configurar comprobaciones de enlaces de red
            // las comprobaciones de enlaces de red son una buena alternativa para pedir al gateway que ACK cada paquete y debe permitir que un gateway maneje más Dots
            // comprobar el enlace con cada conteo de paquete
            // declara al Dot desconectado después de que el umbral haya fallado las comprobaciones de enlace
            // para el threshold_fallos = 11 el dot se considera desconectado al fallar mas de 11 veces en el envio de datos, luego de esto reintentar enlazarse 
            // el primer parametro creo que no tiene efecto cuando se utiliza ack
            update_network_link_check_config(3, threshold_fallos);
        
            // habilitar o deshabilitar Adaptive Data Rate
            dot->setAdr(adr);
        
            // Configurar el join delay
            dot->setJoinDelay(join_delay);
        
            // Guardar cambios en la configuracion
            // logInfo("Guardando configuracion");
            if (!dot->saveConfig()) {
                //logError("No se pudo guardar la configuracion");
            }
        
            // display configuracion
            display_config();

        } else {
            // restaurar la sesión guardada si el Dot se despertó del modo deepsleep
            // útil para usar con deepsleep porque la información de la sesión se pierde cuando el Dot entra en deepsleep
            //logInfo("restaurar sesion de red desde NVM");
            dot->restoreNetworkSession();
        }
            
        
        int j=0;
        int k=0;
        //int flag_error_envio=0; //Preguntar luego qué onda esta variable
        int num_intentos=0;
        int cantidad_envios= 1 + cantidad_REINTENTOS;
        int caudal[cantidad_AI];
            
        int TiempoRandom=0;
        int SumaTiempoRandom=0;
        
        DI_Panel.fall(&interruptPanel_fall);   //Entra a esta función si detecta que hay un flanco descendente en el pin de interrupt
        DI_Panel.rise(&interruptPanel_rise);   //Entra a esta función si solo si detecta que hay un flanco ascendente en el pin de interrupción
        
        // La primera vez que se enlaza con el Gateway
        Led_Green.write(1); // Pensar si conviene colocar LED Verde para dar que está ok
        join_network(Tiempo_reintento_join); // Se une/enlaza con la red (GW) luego del tiempo que le pasas por parámetro
        Led_Green.write(0);

        // Los primeros dos envios de prueba
        std::vector<uint8_t> tx_data;      
        // Armo la trama de datos       
        tx_data.push_back(0xAA);// Tipo de Frame
        tx_data.push_back(0xBB);// Primer Frame
        tx_data.push_back(0x00);// Dato auxiliar, podria ser el nivel de campo, tension, alarma, etc

        // Los dos primeros envios son con poca info xq el nodo envia mas info
        // Se envían 2 datos como para comprobar enlace y ver que llega todo ok, luego arma los datos reales
        send_data(tx_data);
        wait(4);
        send_data(tx_data);
        wait(4);

        
        std::vector<uint8_t> tx_data1; //Paquete 1 de envío de datos reales 8 bytes (separados de a 2 bytes) desde AI 1 a AI 4
        std::vector<uint8_t> tx_data2; //Paquete 2 de envío de datos reales 8 bytes (separados de a 2 bytes) desde AI 5 a AI 8
        std::vector<uint8_t> tx_data3; //Este tendrá sólo el dato de estado de panel

        while (true) 
        {                 
        
                // Unirse a la red si no se unió
                 if (!dot->getNetworkJoinStatus()) 
                {  
                    join_network(Tiempo_reintento_join_2); // Se une/enlaza con la red (GW) luego del tiempo que le pasas por parámetro

                    if (dot->getNetworkJoinStatus()) // Si se enlazó envía el paquete
                    {
                        // Cuando se vuelve a enlazar que envie los msjs cortos como en el inicio
                        send_data(tx_data);
                        send_data(tx_data);
                    }

                }
        
                //if(flag_error_envio==0)// Si no hay error de envío, realizo la medicion pues armo paquete nuevo
                //{
                    tx_data1.clear(); //Limpio el paquete 1 de datos para armar nuevo
                    tx_data2.clear(); //Limpio el paquete 2 de datos para armar nuevo
                    tx_data3.clear(); //Limpio el paquete 3 de datos para armar uno nuevo

                    // Calculo de la frecuencia de 8 AI 
                    for(j=0;j<cantidad_AI; j++)
                    {   
                        if(modo_AI[j]==1)// Chequeo que esté habilitada
                        {
                            frec_ch[j]=funcion_calcularFrec(j+1);
                            logInfo("Frecuencia AI_%d = %f \n\r\n",(j+1), frec_ch[j]);                         
                            
                        }else 
                        {
                            frec_ch[j]=0;
                        }
                        
                    }

                    // Armo la trama de datos 
                    // msj1=AABB00[AI1 (16 bits)][AI2 (16 bits)][AI3 (16 bits)][AI4 (16 bits)]
                    // msj2=AABB01[AI5 (16 bits)][AI6 (16 bits)][AI7 (16 bits)][AI8 (16 bits)]        
                    tx_data1.push_back(0xAA);// Tipo de Frame
                    tx_data1.push_back(0xBB);// Primer Frame
                    tx_data1.push_back(0x00);// Dato auxiliar, podria ser el nivel de campo, tension, alarma, etc

                    tx_data2.push_back(0xAA);// Tipo de Frame
                    tx_data2.push_back(0xBB);// Primer Frame
                    tx_data2.push_back(0x01);// Dato auxiliar, podria ser el nivel de campo, tension, alarma, etc
                    
                    tx_data3.push_back(0xAA);// Tipo de Frame
                    tx_data3.push_back(0xBB);// Primer Frame
                    tx_data3.push_back(0x02);// Dato auxiliar, podria ser el nivel de campo, tension, alarma, etc
                    tx_data3.push_back(cortePanel); //Meto dato de cómo se encuentra el estado de panel
                    send_data(tx_data3); //Esta línea quizás sea mejor ponerla en la condición del OR mas adelante

                    //Para los 8 AI
                    for(k=0;k<cantidad_AI; k++)
                    {
                        if(modo_AI[k]==1)// Chequeo que este habilitada
                        {
                            if(K[k]!=0 && K[k]!=1)// Chequeo si se desea calcular el caudal o solo mandar la frecuencia (caso K = 1) El otro caso se supone chequeado antes.
                            {
                                // Caudal en m3/h
                                caudal[k]=int( ( frec_ch[k] / K[k] ) * 3600/1000 *10); //El *10 es para tomar 1 decimal de resolución. Es un entero pero la frecuencia es float
                                // si K>6 se podria usar dos decimales en lugar de uno.

                                if(k<4)
                                {
                                    //Por lo que se entiende, a pesar de correr bits no toca el valor original caudal[k], por lo tanto no se pierde el dato completo.
                                    // Probar hacer and con 0xFF00 para parte alta y luego con 0x00FF para la parte baja
                                    tx_data1.push_back((caudal[k] >> 8) & 0xFF); // Armo parte alta del packet
                                    tx_data1.push_back(caudal[k] & 0xFF); // Armo parte baja del packet
                                }else 
                                {
                                    tx_data2.push_back((caudal[k] >> 8) & 0xFF);
                                    tx_data2.push_back(caudal[k] & 0xFF);
                                }
                                
                                //tx_data1.push_back((caudal[k] >> 8) & 0xFF);
                                //tx_data1.push_back(caudal[k] & 0xFF);
                                                                
                                logInfo("Caudal AI_%d = %d en m3/h\n\r\n",(k+1), caudal[k]);
                            
                            }else 
                            {
                                frec_ch_int[k]= int(frec_ch[k] * 10); //Acá guardas la frecuencia pero en su valor entero, es decir, lo multiplicas por 10 y casteas a entero.


                                if(k<4)
                                {
                                    tx_data1.push_back((frec_ch_int[k] >> 8) & 0xFF);
                                    tx_data1.push_back(frec_ch_int[k] & 0xFF);
                                }else 
                                {
                                    tx_data2.push_back((frec_ch_int[k] >> 8) & 0xFF);
                                    tx_data2.push_back(frec_ch_int[k] & 0xFF);
                                }
                               //tx_data1.push_back((frec_ch_int[k] >> 8) & 0xFF);
                               //tx_data1.push_back(frec_ch_int[k] & 0xFF);
                                
                                logInfo("Frecuencia entera con un decimal de AI_%d = %d \n\r\n",(k+1), frec_ch_int[k]);
                                
                            }

                        }else // Deshabilitada
                        {
                            
                            if(k<4)
                            {
                                tx_data1.push_back(0xFF);
                                tx_data1.push_back(0xFF);
                            }else 
                            {
                                tx_data2.push_back(0xFF);
                                tx_data2.push_back(0xFF);
                            }
                            //tx_data1.push_back(0xFF);
                            //tx_data1.push_back(0xFF);
                            
                        }                        
                        
                    } 
                    //timer1.stop();
                    //int tiempo= timer1.read_us();
                    //pc.printf("tiempo [us]=%d \n\r\n",tiempo);              
                //}
                
                //Hasta acá armé los paquetes nada mas, no los envié.

                num_intentos=0; // cuenta la cantidad de intentos de comunicación que se intentaron desde que falló
                SumaTiempoRandom = 0; //Suma el tiempo random total para luego restarlo a cuando debe ir a sleep
                
                //Transmision del primer msj      
                             
                //while(((cantidad_envios - flag_error_envio - num_intentos) != 0) && (send_data(tx_data1)!=0)  ) //si no recibio el ACK en ninguna de las dos ventanas y si no se realizaron todos los intentos
                while(((cantidad_envios - num_intentos) != 0) && ( (send_data(tx_data1)!=0) || (send_data(tx_data2)!=0)) || (send_data(tx_data3)!=0) ) //Si no recibio el ACK en alguna de las dos ventanas (distinto de cero) y si no se realizaron todos los intentos
                {
                    num_intentos++; //Va por el próximo intento pues falló alguna condición, por ej, no se recibió ACK de algun send_data   
                    logInfo("Intento numero: %d\n\r",num_intentos);    

                    //El if de ahora es por si no quiero reintentos que la condición de 0 por la resta y chau, que no intente enviar nuevamente, luego saldrá del while
                    if((cantidad_envios - num_intentos) != 0) // si en la siguiente comparacion salgo del while no hago el wait
                    {
                         
                        srand (time(NULL));
                        
                        // Obtengo el valor random que se debe esperar para enviar
                        TiempoRandom = rand() % (TIEMPO_RANDOM_SUP - TIEMPO_RANDOM_INF) + TIEMPO_RANDOM_INF;
                        //logInfo("tiempo_rand = %d\n\r\n", tiempo_rand);
                        logInfo("Tiempo random asignado  = %d\n\r\n",TiempoRandom);
                        // Tiempo random total que se consume. 
                        SumaTiempoRandom = SumaTiempoRandom + TiempoRandom;
                        logInfo("Tiempo random total  = %d\n\r\n",SumaTiempoRandom);
                        
                        logInfo("wait %d\n\r",TiempoRandom);
                        wait(TiempoRandom);
                    }   
        
                }                
        
                

                // Si entra en modo deepsleep, guarde la sesión para que no se necesite unirse nuevamente después de despertar
                // no es necesario si entra en modo sleep ya que se retiene la RAM, sirve para XDot
                //if (sleepMode) {
                    //logInfo("Guardar sesion de red en NVM");
                //    dot->saveNetworkSession();
                //}
                
                
                // El tiempo de sleep depende de las condiciones
                // esto se hace para intentar mantener constante el tiempo entre muestras, 
                // si ocurre una falla la muestra actual sufrira un desfasaje 
                // pero la proxima muestra no se encuentra afectada y continua con el tiempo original.
                /*if(cantidad_REINTENTOS==0) //Si no quiero que haya reintentos entonces que se despierte cada cierto TIEMPO_SLEEP
                {

                    //Se despierta solo con rtc       
                    sleep_wake_rtc_only(sleepMode,TIEMPO_SLEEP); //Se despierta para tomar muestras y enviarlas cada cierto periodo TIEMPO_SLEEP


                }else// cantidad_REINTENTOS es 1 o 2
                {
                    // Acá ya probó en numero de intentos correspondientes ya que salió del while anterior. Ahora se va a dormir un tiempo random para luego despertarse e intentar nuevamente
                    //una cantidad fija dada por cantidad_REINTENTOS
                    if(flag_error_envio==0 && (cantidad_envios - flag_error_envio - num_intentos) == 0) // Acá se generan los tiempos Random en el otro código
                    {*/
                        /*//T random
                        // initialize random seed: 
                        srand (time(NULL));
                        
                        // Defino el intervalo del valor random y obtengo un valor
                        TiempoRandom = rand() % (TIEMPO_RANDOM_SUP - TIEMPO_RANDOM_INF) + TIEMPO_RANDOM_INF;
                        //logInfo("tiempo_rand = %d\n\r\n", tiempo_rand);
                        logInfo("Tiempo random asignado  = %d\n\r\n",TiempoRandom);
                        // Tiempo random total que se consume
                        SumaTiempoRandom = SumaTiempoRandom + TiempoRandom;
                        logInfo("Tiempo random total  = %d\n\r\n",SumaTiempoRandom);


                        //Se despierta solo con rtc luego de un cierto tiempo random pues ya fallaron los intentos fijos    
                        sleep_wake_rtc_only(sleepMode,TiempoRandom);
                        */
                        /*flag_error_envio = 1; // cambio el valor del flag pues sí hubo error en el envío del paquete, sólo que se contabiliza recién ahora, luego de calcular cuando volverá a enviar
                        logInfo("flag_error_envio = %d\n\r\n",flag_error_envio);
                        
                    }else
                    {
                        //Acá entra cuando ya intentó, luego de dormir un tiempo random, la cantidad de veces seteada y todas fallaron, sólo resta irse a dormir.
                        if((cantidad_envios - flag_error_envio - num_intentos) == 0) // fallo en todos los envios (primera y segunda tanda) 
                        {
                            num_intentos=num_intentos-1;// resto para que de bien el t sleep. Cuenta los intervalos de tiempo (ver mi hoja)
                        }
                        // Sleep por Tsleep - T suma -T reintento * num reintentos      
                        //sleep_wake_rtc_only(sleepMode,TIEMPO_SLEEP - SumaTiempoRandom -(num_intentos)); // Acá es donde me voy a dormir el tiempo que queda hasta TIEMPO_SLEEP que debo despertar

                        // T suma=0
                        //SumaTiempoRandom=0;
                        //logInfo("Tiempo random total  = %d\n\r\n",SumaTiempoRandom);

                        flag_error_envio=0; // cambio el valor del flag para que pueda volver a enviar paquetes
                        logInfo("flag_error_envio = %d\n\r\n",flag_error_envio);
                    }

                    
                }*/               
                //sleep_wake_rtc_only(sleepMode,TIEMPO_SLEEP - SumaTiempoRandom);
                sleep_wake_rtc_or_interrupt(sleepMode, TIEMPO_SLEEP - SumaTiempoRandom); //Esta linea va cuando pueda probar en placa
        }
 }
}