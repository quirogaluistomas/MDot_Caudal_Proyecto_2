// Software compatible con la aplicación Configuracion_V1 

// Hace hasta 3 intentos de envio, esperando TIEMPO_REINTENTOS_CONSECUTIVOS seg entre ellos
// si falla en los 3 vuelve a intentar una vez mas despues de un tiempo random entre TIEMPO_RANDOM_SUP y TIEMPO_RANDOM_INF

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

#define ADC_RES_bits 12
// Se obtiene las cuentas corriendo binariamente la cantidad de bits de resolucion del ADC
#define ADC_RES_Cuentas ((1<<ADC_RES_bits)-1)

#if !defined(CHANNEL_PLAN)
#define CHANNEL_PLAN CP_AU915
#endif





///*********DEFINICION DE VARIABLES Y PARAMETROS********////////

#define CLOCK 32    //frec del micro en MHZ

int cantidad_AI=4;

//Creo un arreglo para guardar las frecuencias de los canales leidos
float frec_ch[4]={0,0,0,0};
int frec_ch_int[4];

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

float K[4];


int TIEMPO_SLEEP;
int unidad_TIEMPO_SLEEP;
int cantidad_REINTENTOS;
int TIEMPO_REINTENTOS_CONSECUTIVOS;
int TIEMPO_RANDOM_SUP;
int TIEMPO_RANDOM_INF;

mDot::mdot_file nombre_file;

//int seleccion_CP;
int seleccion_frec;

int modo_AI[4]; // 0: Deshabilitada 1: Habilitada

uint32_t Tiempo_reintento_join=60; // en seg
uint32_t Tiempo_reintento_join_2=60; // en seg



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


// deepsleep consume ligeramente menos corriente que sleep
// en sleep mode, el estado IO es mantenido, la RAM es retenida , y la aplicación se reanudará después de despertarse
// en deepsleep mode, IOs flotantes, RAM se pierde, y la aplicación comenzará desde el principio después de despertarse
// si deep_sleep == true, el dispositivo ingresará en deepsleep mode
static bool deep_sleep = false;
//static bool deep_sleep = true;

mDot* dot = NULL;
lora::ChannelPlan* plan = NULL;

Serial pc(USBTX, USBRX);

// AI a utilizar
AnalogIn AI_1(PB_1);  // Pin 20
AnalogIn AI_2(PB_0);  // Pin 19
AnalogIn AI_3(PA_5);  // Pin 18
AnalogIn AI_4(PA_4);  // Pin 17

// DI a utilizar
DigitalIn DI_modo(PA_6, PullDown); // Pin 4

// DO a utilizar
DigitalOut Led(PC_13); // Pin 13

Timer timer1;
Timer timer2;

/////*************Declaro las funciones a utilizar**********************////
void parpadeo_1(class mbed::DigitalOut led);

void parpadeo_2(class mbed::DigitalOut led);

void parpadeo_3(class mbed::DigitalOut led);

float funcion_calcularFrec (int);

void func_configuracion(void);

bool func_leer_config(void);


//******************DEFINICION DE FUNCIONES*******************////

void parpadeo_1(class mbed::DigitalOut led)
{
    //parpadeo 1: encendido durante 2 seg
    // indica error en escritura
    led.write(1);
    wait(2);
    led.write(0);
       
}

void parpadeo_2(class mbed::DigitalOut led)
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
}

void parpadeo_3(class mbed::DigitalOut led)
{
    //parpadeo 3: parpadea dos veces, duracion 1 seg cada destello
    // indica que se ingreso en el modo configuracion
    led.write(1);
    wait(1);
    led.write(0);
    wait(1);
    led.write(1);
    wait(1);
    led.write(0);
       
}

float funcion_calcularFrec (int ADC_leer) //lee 8 adc distintos
{
    
    uint16_t muestras;
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
                
                
            }

            // Si la señal supera el umbral viniendo de un valor menor a este.
            // Es decir cuando por primera vez en el ciclo al umbral superior es superado.
            if ((muestras > UMBRAL_SUP) && (flag == 0))
            {
                contador++;
                flag = 1;
                if(contador==1)
                {
                    //tiempo desde que inicio hasta que se paso el umbral sup por primera vez
                    // se debe restar este tiempo para el calculo de frec
                    tiempo_inicio=timer2.read_us();
                }
            }
    
            // Si la señal es menor al umbral inferior
            if ((muestras < UMBRAL_SUP) && (muestras < UMBRAL_INF))
            {
                flag = 0;
            }
            
            // En caso de que no haya señal, para que salga del while
            /*if((timer2.read_us() > 1000000) && (contador < 21))
            {              
                return 0;
            } */

            if((timer2.read_us() > 1000000) && (contador < 21))
            {
                ///////////////esto funciona
                tiempo_total=timer2.read_us();
                timer2.stop();
                pc.printf("total_time [us]=%d \n\r\n",tiempo_total); 
                ventana=float(tiempo_total);
                pc.printf("ventana [ms]=%f \n\r\n",ventana);
                pc.printf("contador =%d \n\r\n",contador);
                return 0;

            }

            /* if((timer2.read_us() > 1000000) && (contador < 21))
            {
                ///////////////esto funciona
                tiempo_total=timer2.read_us();
                timer2.stop();
                pc.printf("total_time [us]=%d \n\r\n",tiempo_total); 
                ventana=float(tiempo_total);
                pc.printf("ventana [ms]=%f \n\r\n",ventana);
                contador=20;
                pc.printf("contador =%d \n\r\n",contador);
                frecHZ = 1000000*float(contador)/ventana;
                pc.printf("frec=%f\n\r\n",frecHZ); 
                return 0;

            }*/
            
    }
        
        
    // Tiempo total de la toma de muestras
    tiempo_total=timer2.read_us();
    pc.printf("total_time [us]=%d \n\r\n",tiempo_total);
    timer2.stop();
    
    // Tiempo efectivo de los 40 ciclos
    ventana= float(tiempo_total-tiempo_inicio);   
    pc.printf("ventana [us]=%f \n\r\n",ventana);
      
    //Al numero de umbrales obtenidos le resto 1 para que me de el numero de periodos correspondientes.
    if(contador!=0)
        contador = contador-1;

    //la Ventana de tiempo esta en uS, para pasarla a Hz la multiplico por 1000000
    frecHZ = 1000000*float(contador)/ventana;
    pc.printf("frec=%f\n\r\n",frecHZ);
    
    return (frecHZ);

}

void func_configuracion()
{
        
        int indicador_conf=0;
        int flag_error_write=0;
        int valor_aux=0;
        int valor_aux_2=0;
        char ok[3]="\0";

         

        if(nombre_file.fd<0)
        {
            printf("ERROR en el archivo");
        }

        parpadeo_3(Led);

        // espero que llegue el "ok" de inicio
        pc.scanf("%s",ok);
        if(strcmp(ok, "ok")==0)
        {
                //wait(1);
                pc.printf("%d_", cantidad_AI);               
        }

        

        while(1)
        {
            char indicador_conf_cad[3]="\0";
            char cadena2[8]="\0";
            char cadena3[3]="\0";
            char cadena4[2]="\0";

            int modo_AI_aux[cantidad_AI];
            float K_aux[cantidad_AI];

            // espero que llegue el indicador
            pc.scanf("%s",indicador_conf_cad);
            indicador_conf= std::atoi(indicador_conf_cad);// indica el parametro a modificar


            switch(indicador_conf)
            {
                case 1:
                        // TIEMPO_SLEEP
                        pc.scanf("%s",cadena2);
                        pc.scanf("%s",cadena3);                   
                        pc.scanf("%s",cadena4);
                        // msj 1 : digito1 y digito2
                        // msj 2 : digito3 
                        // msj 3 : unidad

                        if(strcmp(cadena3, "_")!=0)//tres digitos
                            strcat(cadena2, cadena3);// dato de tiempo completo                         


                        // paso el tiempo a segundos
                        valor_aux= std::atoi(cadena2);

                        valor_aux_2=valor_aux * int(std::pow(double(60),(std::atoi(cadena4)))); 
                        
                        
                        // Escribe valor temporal
                        dot->seekUserFile(nombre_file, 0, 0);
                        flag_error_write =dot->writeUserFile(nombre_file, &valor_aux_2, sizeof(valor_aux_2));

                        // Escribe unidad de tiempo [hora min seg]
                        valor_aux=std::atoi(cadena4) ; 
                        dot->seekUserFile(nombre_file, sizeof(int), 0);
                        if( dot->writeUserFile(nombre_file, &valor_aux, sizeof(valor_aux))<0 || flag_error_write<0) //si falla alguna de las escrituras
                            parpadeo_1(Led);  
                        break;

                case 2:
                        // TIEMPO_REINTENTOS_CONSECUTIVOS
                        char cadena_aux[2];
                        pc.scanf("%s",cadena_aux);
                        // msj 1 : num_reintentos
                        valor_aux= std::atoi(cadena_aux);// cantidad de reintentos

                        if(valor_aux!=0)
                        {
                            pc.scanf("%s",cadena2);
                            pc.scanf("%s",cadena3);
                            
                            // msj 2 : digito1 y digito2
                            // msj 3 : digito3
                       
                            if(strcmp(cadena3, "_")!=0)
                                strcat(cadena2, cadena3);
                            
                            valor_aux_2= std::atoi(cadena2);// tiempo en segundos

                        }else
                        {
                            valor_aux_2=0; // si los reintentos son 0, el tiempo de reintento tambien es 0
                        }

                        dot->seekUserFile(nombre_file, sizeof(int)*2, 0);
                        flag_error_write = dot->writeUserFile(nombre_file, &valor_aux, sizeof(valor_aux));

                        dot->seekUserFile(nombre_file, sizeof(int)*3, 0);
                        if( dot->writeUserFile(nombre_file, &valor_aux_2, sizeof(valor_aux_2))<0 || flag_error_write<0)//se guarda en la memoria no volatil
                            parpadeo_1(Led);
                        break;
                
                case 3:
                        //TIEMPO_RANDOM_SUP
                        pc.scanf("%s",cadena2);
                        pc.scanf("%s",cadena3);
                        // msj 1 : digito1 y digito2
                        // msj 2 : digito3

                        if(strcmp(cadena3, "_")!=0)
                            strcat(cadena2, cadena3);

                        valor_aux= std::atoi(cadena2);// tiempo en segundos
                        dot->seekUserFile(nombre_file, sizeof(int)*4, 0);

                        if( dot->writeUserFile(nombre_file, &valor_aux, sizeof(valor_aux))<0)//se guarda en la memoria no volatil
                            parpadeo_1(Led);
                        break;

                case 4:
                        //TIEMPO_RANDOM_INF
                        pc.scanf("%s",cadena2);
                        pc.scanf("%s",cadena3);
                        // msj 1 : digito1 y digito2
                        // msj 2 : digito3

                        if(strcmp(cadena3, "_")!=0)
                            strcat(cadena2, cadena3);

                        valor_aux= std::atoi(cadena2);// tiempo en segundos
                        dot->seekUserFile(nombre_file,  sizeof(int)*5, 0);

                        if( dot->writeUserFile(nombre_file, &valor_aux, sizeof(valor_aux))<0)//se guarda en la memoria no volatil
                            parpadeo_1(Led);
                        break;

                case 5:
                        // configuracion frec
                        pc.scanf("%s",cadena2);
                        valor_aux= std::atoi(cadena2);
                        dot->seekUserFile(nombre_file, sizeof(int)*6, 0);
                        if(valor_aux>=65 && valor_aux<=71)
                        {
                            if( (dot->writeUserFile(nombre_file, &valor_aux, sizeof(valor_aux))<0))//se guarda en la memoria no volatil
                                parpadeo_1(Led);     
                        }    
                        break;

                case 6:
                        //Envio de configuracion tiempos
                        dot->seekUserFile(nombre_file, 0, 0);
                        dot->readUserFile(nombre_file, &CONFIGURACION, sizeof(int)*6);
                        pc.printf("%d_%d_%d_%d_%d_%d", CONFIGURACION[0]/int(std::pow(double(60),CONFIGURACION[1])), CONFIGURACION[1], CONFIGURACION[2], CONFIGURACION[3], CONFIGURACION[4], CONFIGURACION[5]);
                        break;     

                case 7:
                        //Envio de configuracion banda de frec
                        dot->seekUserFile(nombre_file, sizeof(int)*6, 0);
                        dot->readUserFile(nombre_file, &seleccion_frec, sizeof(seleccion_frec));
                        switch(seleccion_frec)
                        {
                            // A
                            case 65:        pc.printf("US915");                          
                                            break;
                                    
                            // B
                            case 66:        pc.printf("AU915");
                                            break;
                                    
                            // C
                            case 67:        pc.printf("EU868");
                                            break;
                                    
                            // D
                            case 68:        pc.printf("KR920");
                                            break;
                                    
                            // E
                            case 69:        pc.printf("AS923");
                                            break;
                                    
                            // F
                            case 70:        pc.printf("AS923_JAPAN");
                                            break;
                            // G
                            case 71:        pc.printf("IN865");
                                            break;
                        }
                        break;  
                
                case 8:
                        // configuracion AI
                        int i;
                        int j;
                        

                        // recibo conf de AI --> habilitada o deshabilitada
                        for(i=0;i<cantidad_AI;i++)
                        {
                            pc.scanf("%s",cadena2);
                            modo_AI_aux[i] = std::atoi(cadena2);
                        }

                        // recibo los coeficientes K
                        for(j=0;j<cantidad_AI;j++)
                        {
                            pc.scanf("%s",cadena2);
                            pc.scanf("%s",cadena3);
                            if(strcmp(cadena3, "_")!=0)
                                strcat(cadena2, cadena3);
                           
                            K_aux[j] = std::stof(cadena2);
                        }


                        dot->seekUserFile(nombre_file,  sizeof(int)*7, 0);
                        flag_error_write=dot->writeUserFile(nombre_file, &modo_AI_aux, sizeof(modo_AI_aux));
                        dot->seekUserFile(nombre_file,  sizeof(int)*11, 0);
                        if( dot->writeUserFile(nombre_file, &K_aux, sizeof(K_aux))<0 || flag_error_write<0) //se guarda en la memoria no volatil
                            parpadeo_1(Led);
                        break;
                            
                case 9:
                        //Envio de configuracion AI
                        dot->seekUserFile(nombre_file, sizeof(int)*7, 0);
                        dot->readUserFile(nombre_file, &modo_AI, sizeof(modo_AI));
                        dot->seekUserFile(nombre_file, sizeof(int)*11, 0);
                        dot->readUserFile(nombre_file, &K, sizeof(K));
                        pc.printf("%d%d%d%d", modo_AI[0], modo_AI[1], modo_AI[2], modo_AI[3]);
                        wait(0.05);
                        pc.printf("%.1f_%.1f_%.1f_%.1f", K[0], K[1], K[2], K[3]);
                        break;

                case 10:
                        //Envio de configuracion AppKey y DevKey
                        int k;
                        for(k=0;k<16; k++)
                        {
                            pc.printf("%X", network_key[k]);
                        }
                        pc.printf("_");

                        for(k=0;k<8; k++)
                        {
                            pc.printf("%X", network_id[k]);
                        }
                                 
                        break;
            }
        }

}
        

bool func_leer_config(void)
{
    //Leer configuracion guardada
    bool error_config = false;
    int flag_leer=0;

    dot->seekUserFile(nombre_file, 0, 0);//me posiciono al principio del archivo
    flag_leer=dot->readUserFile(nombre_file, &CONFIGURACION, sizeof(CONFIGURACION));
    dot->seekUserFile(nombre_file, sizeof(int)*11, 0);//me posiciono donde termina CONFIGURACION
    if(dot->readUserFile(nombre_file, &K, sizeof(K))<0 || flag_leer<0)//leo todo
    {
        logInfo("ERROR: no se pudo leer la configuracion");
        error_config= true;
    }else
    {

        // Si se leyo correctamente analizo los datos
        TIEMPO_SLEEP=CONFIGURACION[0];
        unidad_TIEMPO_SLEEP=CONFIGURACION[1];
        cantidad_REINTENTOS=CONFIGURACION[2];
        TIEMPO_REINTENTOS_CONSECUTIVOS=CONFIGURACION[3];
        TIEMPO_RANDOM_SUP=CONFIGURACION[4];
        TIEMPO_RANDOM_INF=CONFIGURACION[5];
        seleccion_frec=CONFIGURACION[6];
        modo_AI[0] =CONFIGURACION[7];
        modo_AI[1] =CONFIGURACION[8];
        modo_AI[2] =CONFIGURACION[9];
        modo_AI[3] =CONFIGURACION[10];  



        logInfo("TIEMPO_SLEEP = %d\n\r", TIEMPO_SLEEP);
        logInfo("unidad_TIEMPO_SLEEP = %d\n\r", unidad_TIEMPO_SLEEP);
        logInfo("cantidad_REINTENTOS = %d\n\r", cantidad_REINTENTOS);
        logInfo("TIEMPO_REINTENTOS_CONSECUTIVOS = %d\n\r",  TIEMPO_REINTENTOS_CONSECUTIVOS);
        logInfo("TIEMPO_RANDOM_SUP = %d\n\r",  TIEMPO_RANDOM_SUP);
        logInfo("TIEMPO_RANDOM_INF = %d\n\r",  TIEMPO_RANDOM_INF);

        
        if(cantidad_REINTENTOS==0)
        {
            if(TIEMPO_SLEEP<= 0)
            {
                logInfo("ERROR en la configuracion");
                error_config= true;
            }

        }else
        {
            if(TIEMPO_SLEEP<= (TIEMPO_RANDOM_SUP +  (cantidad_REINTENTOS + 1)*TIEMPO_REINTENTOS_CONSECUTIVOS))
            {
                logInfo("ERROR en la configuracion");
                error_config= true;
            }
            
            if(TIEMPO_RANDOM_SUP>300)// 5min
            {
                logInfo("ERROR en la configuracion");
                error_config= true;
            }

            if(TIEMPO_RANDOM_INF<30)// 30 seg
            {
                logInfo("ERROR en la configuracion");
                error_config= true;
            }
        }
          

        switch(seleccion_frec)
        {
            // A
            case 65:
                            //CHANNEL_PLAN CP_US915;
                            plan = new lora::ChannelPlan_US915();
                            logInfo("\n\rSe selecciono US915\n\r");                           
                            break;
                    
            // B
            case 66:
                            //CHANNEL_PLAN CP_AU915;
                            plan = new lora::ChannelPlan_AU915();
                            logInfo("\n\rSe selecciono AU915\n\r");
                            break;
                    
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
                            
            default: 
                            logInfo("\n\rEl valor ingresado no es valido\n\r");
                            logInfo("\n\rseleccion_frec= %d\n\r", seleccion_frec);
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
        Tiempo_reintento_join_2=TIEMPO_SLEEP;
    }
    
    return error_config;
}



//////////////////////////////////////////   
        
 int main() {
    
    pc.baud(115200);

    plan = new lora::ChannelPlan_US915();//por defecto para que arranque se modifica con la configuracion el valor de plan
    MBED_ASSERT(plan);

    dot = mDot::getInstance(plan);

    MBED_ASSERT(dot);

    nombre_file = dot->openUserFile("configuracion5.txt", (1 << 3)|(1 << 4));

    if(!dot->seekUserFile(nombre_file, 0, 0))// Si es la primera vez que entre al if y cree el archivo
    {
        dot->saveUserFile("configuracion5.txt", CONFIGURACION, sizeof(CONFIGURACION) + sizeof(float)*2);
        nombre_file = dot->openUserFile("configuracion5.txt", (1 << 3)|(1 << 4));
    }
        
    
    if(DI_modo)   
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
            logInfo("ERROR al leer la configuracion");
            parpadeo_2(Led);
            
        }

        uint8_t threshold_fallos = 2*(2*cantidad_REINTENTOS+1);
        // si reintentos es 0 threshold_fallos=2  lo que equivale a dos pasadas
        // si reintentos es 1 threshold_fallos=6  lo que equivale a dos pasadas completas de intentos
        // si reintentos es 2 threshold_fallos=10  lo que equivale a dos pasadas completas de intentos
        
        dot->setChannelPlan(plan); // con esto puedo cambiar el plan que necesariamente tuve que definir arriba
        
        // unir el controlador de eventos personalizados
        dot->setEvents(&events);
        
        if (!dot->getStandbyFlag()) {
            //logInfo("mbed-os library version: %d.%d.%d", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
        
            // comenzar desde un estado conocido
            //logInfo("defaulting Dot configuration");
            dot->resetConfig();
            dot->resetNetworkSession();
        
            // asegurarse de que el registro de la biblioteca esté activado
            dot->setLogLevel(mts::MTSLog::INFO_LEVEL);
        
            // es necesario actualizar la configuracion 
            if (dot->getJoinMode() != mDot::OTA) {
                //logInfo("changing network join mode to OTA");
                if (dot->setJoinMode(mDot::OTA) != mDot::MDOT_OK) {
                    //logError("failed to set network join mode to OTA");
                }
            }
                
        
            // en los modos de unión OTA y AUTO_OTA, las credenciales se pueden pasar a la biblioteca como un name y passphrase o un ID y KEY
            // solo un metodo o el otro puede ser usado
            // network ID = crc64(network name)
            // network KEY = cmac(network passphrase)
            update_ota_config_id_key(network_id, network_key, frequency_sub_band, network_type, ack);
        
        
            // configurar comprobaciones de enlaces de red
            // las comprobaciones de enlaces de red son una buena alternativa a la necesidad de que el gateway ACK en cada paquete y debe permitir que un gateway maneje más Dots
            // comprobar el enlace cada paquete de conteo
            // declare el Dot desconectado después de que el umbral haya fallado las comprobaciones de enlace
            // para el threshold_fallos = 11 el dot se considera desconectado al fallar mas de 11 veces en el envio de datos, luego de esto reintentar enlazarse 
            // el primer parametro creo que no tiene efecto cuando se utiliza ack
            update_network_link_check_config(3, threshold_fallos);
        
            // habilitar o deshabilitar Adaptive Data Rate
            dot->setAdr(adr);
        
            // Configurar el join delay
            dot->setJoinDelay(join_delay);
        
            // guardar cambios en la configuracion
            //logInfo("Guardando configuracion");
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
        int flag_error_envio=0;
        int num_intentos=0;
        int cantidad_envios= 1 + cantidad_REINTENTOS;
        int caudal[cantidad_AI];
            
        int tiempo_rand=0;
        int tiempo_suma=0;
        
            
        // la primera vez que se enlaza con el Gateway
        Led.write(1);
        join_network(Tiempo_reintento_join);
        Led.write(0);

        // los primeros dos envios
        std::vector<uint8_t> tx_data;      
        // Armo la trama de datos             
        tx_data.push_back(0xAA);// Tipo de Frame
        tx_data.push_back(0xBB);// Primer Frame
        tx_data.push_back(0x00);// Dato auxiliar, podria ser el nivel de campo, tension, alarma, etc

        // Los dos primeros envios son con poca info xq el nodo envia mas info
        send_data(tx_data);
        wait(4);
        send_data(tx_data);
        wait(4);

        
        std::vector<uint8_t> tx_data1;
        

        while (true) 
        {
                         
        
                // Unirse a la red si no se unió
                 if (!dot->getNetworkJoinStatus()) 
                {  
                    join_network(Tiempo_reintento_join_2);

                    if (dot->getNetworkJoinStatus()) // si se enlazo
                    {
                        // cuando se vuelve a enlazar que envie los msjs cortos como en el inicio
                        send_data(tx_data);
                        send_data(tx_data);
                    }

                }
        
                if(flag_error_envio==0)// Realizo la medicion si se trata de un paquete nuevo
                {
                    tx_data1.clear();

                    // Calculo de la frecuencia de 4 AI   
                    for(j=0;j<cantidad_AI; j++)
                    {   
                        if(modo_AI[j]==1)// Chequeo que este habilitada
                        {
                            frec_ch[j]=funcion_calcularFrec(j+1);  
                            logInfo("frecuencia AI_%d = %f \n\r\n",(j+1), frec_ch[j]);                         
                            
                        }else 
                        {
                            frec_ch[j]=0;
                        }
                        
                    }

                    // Armo la trama de datos 
                    // msj1=AABB00[AI1 (16 bits)][AI2 (16 bits)][AI3 (16 bits)][AI4 (16 bits)]          
                    tx_data1.push_back(0xAA);// Tipo de Frame
                    tx_data1.push_back(0xBB);// Primer Frame
                    tx_data1.push_back(0x00);// Dato auxiliar, podria ser el nivel de campo, tension, alarma, etc

                        
                    //Para los 4 AI
                    for(k=0;k<cantidad_AI; k++)
                    {
                        if(modo_AI[k]==1)// Chequeo que este habilitada
                        {
                            if(K[k]!=0 && K[k]!=1)// Chequeo si se desea calcular el caudal
                            {
                                // CAudal en m3/h
                                caudal[k]=int( ( frec_ch[k] / K[k] ) * 3600/1000 *10);
                                // si K>6 se podria usar dos decimales en lugar de uno.

                                tx_data1.push_back((caudal[k] >> 8) & 0xFF);
                                tx_data1.push_back(caudal[k] & 0xFF);
                                                                
                                logInfo("Caudal AI_%d = %d en m3/h\n\r\n",(k+1), caudal[k]);
                            
                            }else 
                            {
                                frec_ch_int[k]= int(frec_ch[k] * 10);

                                tx_data1.push_back((frec_ch_int[k] >> 8) & 0xFF);
                                tx_data1.push_back(frec_ch_int[k] & 0xFF);
                                
                                logInfo("Frecuencia int AI_%d = %d \n\r\n",(k+1), frec_ch_int[k]);
                                
                            }

                        }else // Deshabilitada
                        {

                            tx_data1.push_back(0xFF);
                            tx_data1.push_back(0xFF);
                            
                        }                        
                        
                    } 
                    timer1.stop();
                    int tiempo= timer1.read_us();
                    pc.printf("tiempo [us]=%d \n\r\n",tiempo);              
                }
                
                     
                num_intentos=0;
                
                
                //Transmision del primer msj                   
                while(((cantidad_envios - flag_error_envio - num_intentos) != 0) && (send_data(tx_data1)!=0)  ) //si no recibio el ACK en ninguna de las dos ventanas y si no se realizaron todos los intentos
                {
                    num_intentos++;   
                    logInfo("num_intentos %d\n\r",num_intentos);    

                    if((cantidad_envios - flag_error_envio - num_intentos) != 0) // si en la siguiente comparacion salgo del while no hago el wait
                    {
                        logInfo("wait %d\n\r",TIEMPO_REINTENTOS_CONSECUTIVOS);
                        wait(TIEMPO_REINTENTOS_CONSECUTIVOS);
                    }   
        
                }                
        
                
                // Si entra en modo deepsleep, guarde la sesión para que no se necesite unirse nuevamente después de despertar
                // no es necesario si entra en modo sleep ya que se retiene la RAM
                if (deep_sleep) {
                    //logInfo("Guardar sesion de red en NVM");
                    dot->saveNetworkSession();
                }
                
                
                /// El tiempo de sleep depende de las condiciones
                // esto se hace para intentar mantener constante el tiempo entre muestras, 
                // si ocurre una falla la muestra actual sufrira un desfasaje 
                // pero la proxima muestra no se encuentra afectada y continua con el tiempo original.
                if(cantidad_REINTENTOS==0)
                {

                    //Se despierta solo con rtc       
                    sleep_wake_rtc_only(deep_sleep,TIEMPO_SLEEP);


                }else// cantidad_REINTENTOS es 1 o 2
                {
                    if(flag_error_envio==0 && (cantidad_envios - flag_error_envio - num_intentos) == 0) //T random
                    {
                        //T random
                        // initialize random seed: 
                        srand (time(NULL));
                        
                        // Defino el intervalo del valor random y obtengo un valor
                        tiempo_rand = rand() % (TIEMPO_RANDOM_SUP - TIEMPO_RANDOM_INF) + TIEMPO_RANDOM_INF;
                        //logInfo("tiempo_rand = %d\n\r\n", tiempo_rand);
                        logInfo("tiempo_rand  = %d\n\r\n",tiempo_rand);
                        // T suma
                        tiempo_suma=tiempo_rand + ((num_intentos-1) * TIEMPO_REINTENTOS_CONSECUTIVOS);
                        logInfo("tiempo_suma  = %d\n\r\n",tiempo_suma);


                        //Se despierta solo con rtc       
                        sleep_wake_rtc_only(deep_sleep,tiempo_rand);
                        
                        flag_error_envio=1; // cambio el valor del flag
                        logInfo("flag_error_envio = %d\n\r\n",flag_error_envio);
                        
                    }else
                    {
                        if((cantidad_envios - flag_error_envio - num_intentos) == 0) // fallo en todos los envios (primera y segunda tanda) 
                        {
                            num_intentos=num_intentos-1;// resto para que de bien el t sleep 
                        }
                        // Sleep por Tsleep - T suma -T reintento * num reintentos      
                        sleep_wake_rtc_only(deep_sleep,TIEMPO_SLEEP - tiempo_suma -(num_intentos * TIEMPO_REINTENTOS_CONSECUTIVOS));

                        // T suma=0
                        tiempo_suma=0;
                        logInfo("tiempo_suma  = %d\n\r\n",tiempo_suma);

                        flag_error_envio=0; // cambio el valor del flag
                        logInfo("flag_error_envio = %d\n\r\n",flag_error_envio);
                    }

                }               
        
        }
 }
} 
/* void funcion(void){
    timer1.reset();
    timer1.start();
    pc.printf("Hola\n\r");
    wait(2);
    pc.printf("Chau\n\r");
    wait(1);
    timer1.stop();
    int tiempo=timer1.read_us();
    pc.printf("tiempo= %d\n\r",tiempo);
}
int main()
{
    pc.baud(115200);
    pc.printf("Main\n\r");
    while(1)
    {
        
        funcion();
        
        wait(3);
    }
    

}*/