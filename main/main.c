#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <math.h>
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <string.h>
#include "rom/ets_sys.h"

//================================================================
// CÓDIGO HTML Y FRONTEND
// Se envia como texto plano. Utiliza "Flexbox" para las barras y 
// transiciones CSS para suavizar los saltos (interpolación visual).
//================================================================
const char *index_html = 
    "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Genio Genio Genio</title>"
    "<style>"
    "  body { background-color: #111; color: white; text-align: center; font-family: Arial; }"
    "  #ecualizador { display: flex; align-items: flex-end; justify-content: center; gap: 15px; height: 300px; margin-top: 50px; padding: 20px; background: #222; border-radius: 10px;}"
    "  .barra { width: 40px; background-color: #00ffcc; border-radius: 5px 5px 0 0; transition: height 0.05s ease; height: 0%; }"
    "</style></head><body>"
    "<h1>Deeply enthralled in your gaze</h1>"
    "<div id='ecualizador'>"
    "  <div class='barra' id='b0'></div><div class='barra' id='b1'></div><div class='barra' id='b2'></div>"
    "  <div class='barra' id='b3'></div><div class='barra' id='b4'></div><div class='barra' id='b5'></div>"
    "  <div class='barra' id='b6'></div>"
    "</div>"
    "<script>"
    "  var ws = new WebSocket('ws://' + location.hostname + '/ws');"
    "  ws.onmessage = function(e) {"
    "    var datos = e.data.split(',');"
    "    for(var i=0; i<7; i++) { document.getElementById('b'+i).style.height = (datos[i] * 12.5) + '%'; }"
    "  };"
    "</script></body></html>";

//================================================================
// DEFINICIONES DE HARDWARE Y DSP
//================================================================
/* numMuestras 512: Requisito estricto Radix-2 para el algoritmo Cooley-Tukey (2^9).
 frecuenciaMuestreo 32000: Por Teorema de Nyquist, permite analizar hasta 16kHz de audio real.
 */
#define numMuestras 512
#define frecuenciaMuestreo 32000 
#define pinMicrofono ADC_CHANNEL_0  
#define pinClock 2
#define pinDatos 3
#define pinLatch 10

//================================================================
// VARIABLES GLOBALES (FFT Y SERVIDOR WEB)
//================================================================
// Arreglos globales para evitar Stack Overflow en la memoria
float varReal[numMuestras];
float varImaginaria[numMuestras];

// Descriptores para gestionar la conexión persistente del WebSocket
static int ws_client_fd=-1;
static httpd_handle_t global_server=NULL;

//================================================================
// FUNCIONES DEL SERVIDOR WEB (HTTP Y WEBSOCKET)
//================================================================

/* index_handler: Intercepta peticiones a la IP raíz y envía el código HTML */
esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* ws_handler: Sube la conexión de HTTP a WebSocket y guarda el descriptor para poder enviarle datos asíncronos en tiempo real desde el bucle principal.*/
esp_err_t ws_handler(httpd_req_t *req){
    ws_client_fd=httpd_req_to_sockfd(req);
    global_server=req->handle;
    if(req->method==HTTP_GET){
        printf("Conectado, lcdll\n");
        return ESP_OK;
    }
    return ESP_OK;
}

//================================================================
// FUNCIONES DE PROCESAMIENTO DIGITAL DE SEÑALES (DSP)
//================================================================

/* removerCC: Elimina el Offset de Voltaje Continuo del micrófono. 
 Si no se remueve, la FFT arrojaría un pico en 0Hz.*/
void removerCC(float *datos, uint16_t muestras){
    float media=0;
    for(uint16_t i=0; i<muestras; i++){
        media+=datos[i];
    }
    media/=muestras;
    for(uint16_t i=0; i<muestras; i++){
        datos[i]-=media;
    }
}

/* calcularFFT: Transformada Rápida de Fourier (Algoritmo Cooley-Tukey).
 Se elige por sobre la DFT porque reduce operaciones de O(N^2) a O(N log N).
 */
void calcularFFT(float *varReal, float *varImaginaria, uint16_t muestras){
    uint16_t j=0;
    // Etapa 1: Bit-Reversal (Reordenamiento del arreglo)
    for(uint16_t i=0; i<muestras-1; i++){
        if(i<j){
            float auxReal=varReal[j];
            float auxImaginaria=varImaginaria[j];
            varReal[j]=varReal[i];
            varImaginaria[j]=varImaginaria[i];
            varReal[i]=auxReal;
            varImaginaria[i]=auxImaginaria;
        }
        uint16_t k=muestras>>1;
        while(k>0&&k<=j){ // Validación estricta para evitar bucles infinitos
            j-=k;
            k>>=1;
        }
        j+=k;
    }

    // Etapa 2: Cálculo de Mariposas (Cooley-Tukey)
    float c1=-1.0f;
    float c2=0.0f;
    uint16_t l2=1;
    for(uint16_t l=0; (1<<l)<muestras; l++){
        uint16_t l1=l2;
        l2<<=1;
        float u1=1.0f;
        float u2=0.0f;
        for(j=0; j<l1; j++){
            for(uint16_t i=j; i<muestras; i+=l2){
                uint16_t i1=i+l1;
                float t1=u1*varReal[i1]-u2*varImaginaria[i1];
                float t2=u1*varImaginaria[i1]+u2*varReal[i1];
                varReal[i1]=varReal[i]-t1;
                varImaginaria[i1]=varImaginaria[i]-t2;
                varReal[i]+=t1;
                varImaginaria[i]+=t2;
            }
            float z=(u1*c1)-(u2*c2);
            u2=(u1*c2)+(u2*c1);
            u1=z;
        }
        c2=sqrtf((1.0f-c1)/2.0f);
        c1=sqrtf((1.0f+c1)/2.0f);
        c2=-c2;
    }
}

/* calcularMagnitud: Aplica Pitágoras para convertir los números complejos
 resultantes de la FFT en magnitudes absolutas (amplitud real de frecuencia). */
void calcularMagnitud(float *varReal, float *varImaginaria, uint16_t muestras){
    // Solo evalúa hasta muestras/2 por simetría de Nyquist
    for(uint16_t i=0; i<muestras/2; i++){
        varReal[i]=sqrtf((varReal[i]*varReal[i])+(varImaginaria[i]*varImaginaria[i]));
    }
}

//================================================================
// FUNCIONES DE CONTROL DE HARDWARE (GPIO Y DMA)
//================================================================

/* iniciarPinesLED: Declara explicitamente los GPIO como Salida 
 para evitar estados de alta impedancia.*/
void iniciarPinesLED(){
    gpio_reset_pin(pinClock);
    gpio_reset_pin(pinDatos);
    gpio_reset_pin(pinLatch);
    gpio_set_direction(pinClock, GPIO_MODE_OUTPUT);
    gpio_set_direction(pinDatos, GPIO_MODE_OUTPUT);
    gpio_set_direction(pinLatch, GPIO_MODE_OUTPUT);
}

/* enviar74HC595: Implementa comunicación Serial Bit-Banging.
 * Desplaza usando enmascaramiento bit a bit. */
void enviar74HC595(uint8_t *datos, int numColumnas){
    for(int i=numColumnas-1; i>=0; i--){
        uint8_t auxDatos=datos[i];
        for(int j=7; j>=0; j--){
            gpio_set_level(pinDatos, (auxDatos>>j)&1);
            esp_rom_delay_us(1); //
            gpio_set_level(pinClock, 1);
            esp_rom_delay_us(1);
            gpio_set_level(pinClock, 0);
            esp_rom_delay_us(1);
        }
    }
    gpio_set_level(pinLatch, 1); // Dispara la salida física paralela de los integrados
    esp_rom_delay_us(1);
    gpio_set_level(pinLatch, 0);
}

adc_continuous_handle_t adc_handle=NULL;

/* iniciarADC: Configura el ADC en modo DMA (Acceso Directo a Memoria).
 Permite que el hardware lea el sonido de forma automática sin trabar la CPU.*/
void iniciarADC(){
    adc_continuous_handle_cfg_t cfg={
        .max_store_buf_size=numMuestras*SOC_ADC_DIGI_RESULT_BYTES*4,
        .conv_frame_size=numMuestras*SOC_ADC_DIGI_RESULT_BYTES,
    };
    adc_continuous_new_handle(&cfg, &adc_handle);
    adc_digi_pattern_config_t patron={
        .atten=ADC_ATTEN_DB_12,
        .channel=ADC_CHANNEL_0,
        .unit=ADC_UNIT_1,
        .bit_width=ADC_BITWIDTH_12,
    };
    adc_continuous_config_t dig_cfg={
        .sample_freq_hz=frecuenciaMuestreo,
        .conv_mode=ADC_CONV_SINGLE_UNIT_1,
        .format=ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num=1,
        .adc_pattern=&patron,
    };
    adc_continuous_config(adc_handle, &dig_cfg);
    adc_continuous_start(adc_handle);
}

//================================================================
// FUNCIÓN PRINCIPAL
//================================================================
void app_main(void){
    // 1. Inicialización NVS y Red
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg_wifi = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg_wifi);
    wifi_config_t wifi_config = {
        .ap={.ssid="Honor is dead", .password="But I'll see what I can do", .channel=1, .max_connection=4, .authmode=WIFI_AUTH_WPA2_PSK}
    };
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    // 2. Inicialización Servidor Web HTTP y WebSocket
    httpd_config_t config_server=HTTPD_DEFAULT_CONFIG();
    config_server.lru_purge_enable=true;
    if (httpd_start(&global_server, &config_server)==ESP_OK) {
        httpd_uri_t uri_get={.uri="/", .method=HTTP_GET, .handler=index_handler};
        httpd_register_uri_handler(global_server, &uri_get);
        httpd_uri_t ws_uri={.uri="/ws", .method=HTTP_GET, .handler=ws_handler, .is_websocket=true};
        httpd_register_uri_handler(global_server, &ws_uri);
    }

    // 3. Inicialización Hardware Físico
    iniciarADC();
    iniciarPinesLED();

    // Drenaje inicial: Limpia la basura residual del buffer DMA provocada durante el arranque del Wi-Fi
    uint8_t bufferBasura[numMuestras*SOC_ADC_DIGI_RESULT_BYTES];
    uint32_t bytesBasura=0;
    adc_continuous_read(adc_handle, bufferBasura, sizeof(bufferBasura), &bytesBasura, 0);

    //================================================================
    // LOOP PRINCIPAL EN TIEMPO REAL
    // No usa vTaskDelay; el procesador descansa en la llamada DMA
    //================================================================
    while(1){
        static uint8_t bufferRAW[numMuestras*SOC_ADC_DIGI_RESULT_BYTES];
        uint32_t bytesLeidos=0;
        
        // Bloqueante (portMAX_DELAY): Dicta el ritmo del sistema a ~16ms exactos
        esp_err_t ret=adc_continuous_read(adc_handle, bufferRAW, sizeof(bufferRAW), &bytesLeidos, portMAX_DELAY);
        
        if(ret==ESP_OK){
            adc_digi_output_data_t *p=(adc_digi_output_data_t*)bufferRAW;
            int totalMuestras=bytesLeidos/SOC_ADC_DIGI_RESULT_BYTES;
            
            // Protección de integridad (Zero-Padding) contra "envenenamiento de buffer"
            for(int i=0; i<numMuestras; i++){
                if(i<totalMuestras){
                    varReal[i]=(float)(p[i].type2.data);
                }else{
                    varReal[i]=0.0f; // Evita escalar ruido infinito si falló una lectura
                }
                varImaginaria[i]=0.0f;
            }
            
            // Procesamiento de Señales en cascada
            removerCC(varReal, numMuestras);
            calcularFFT(varReal, varImaginaria, numMuestras);
            calcularMagnitud(varReal, varImaginaria, numMuestras);

            // Arreglos de salida y limites lógicos 
            int nivelesLED[7];
            uint8_t bufferColumnas[7]={0, 0, 0, 0, 0, 0, 0};

            // Sensibilidad ajustada para evitar falsos positivos
            float energiaTope=400000.0f;
            float energiaPiso=50000.0f;
            
            // Agrupamiento geométrico de bandas emulando el espectro de Octavas (Norma ISO)
            // Rangos (Hz aprox): 63, 160, 400, 1k, 2.5k, 6.25k, 16k
            int limiteBandas[8]={1, 3, 6, 13, 29, 61, 126, 256};
            
            for(int i=0; i<7; i++){
                float energiaSuma=0;
                int inicio=limiteBandas[i], fin=limiteBandas[i+1];
                int cantidadBins=fin-inicio;
                
                // Promedio de energía dentro de la octava (Evita que agudos solapen a graves)
                for(int j=inicio; j<fin; j++){
                    energiaSuma+=varReal[j];
                }
                float energiaNota=energiaSuma/cantidadBins;
                int prenderLED=0;

                // Mapeo Psicoacústico (Escala Decibelios Log10)
                if(energiaNota>energiaPiso){
                    float dBNota=10.0f*log10f(energiaNota);
                    float dbPiso=10.0f*log10f(energiaPiso);
                    float dBTope=10.0f*log10f(energiaTope);

                    // Regla de 3 Logarítmica para visualización lineal del vúmetro
                    float porcentaje=(dBNota-dbPiso)/(dBTope-dbPiso);
                    prenderLED=(int)(porcentaje*8.0f);
                }

                // Barreras físicas de hardware
                if(prenderLED>8) prenderLED=8;
                if(prenderLED<0) prenderLED=0;

                nivelesLED[i]=prenderLED;

                // Preparación de máscara física
                uint8_t barra=0;
                for(int j=0; j<prenderLED; j++){
                    barra|=(1<<j);
                }
                bufferColumnas[i]=barra;
            }

            // Refresco de Hardware local
            enviar74HC595(bufferColumnas, 7);
            
            // Refresco de Red (Throttling) para evitar que HTTP/Wi-Fi asfixie la CPU (Watchdog)
            static int contadorWeb=0;
            contadorWeb++;
            
            if(contadorWeb>=4){ // Envía 1 paquete por cada 4 ciclos analíticos (~15 FPS en Web)
                if (ws_client_fd!=-1 && global_server!=NULL){
                    static char datos_ws[32];
                    snprintf(datos_ws, sizeof(datos_ws), "%d,%d,%d,%d,%d,%d,%d", nivelesLED[0], nivelesLED[1], nivelesLED[2], nivelesLED[3], nivelesLED[4], nivelesLED[5], nivelesLED[6]);
                    
                    httpd_ws_frame_t ws_pkt;
                    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
                    ws_pkt.payload=(uint8_t*)datos_ws;
                    ws_pkt.len=strlen(datos_ws);
                    ws_pkt.type=HTTPD_WS_TYPE_TEXT;
                    
                    // Envío Asíncrono: Delega la red al SO sin bloquear la siguiente lectura del micrófono
                    esp_err_t err_ws=httpd_ws_send_frame_async(global_server, ws_client_fd, &ws_pkt);
                    
                    if (err_ws!=ESP_OK) { // Desconexión segura si el cliente cierra la pestaña
                        ws_client_fd=-1;
                    }
                }
                contadorWeb=0;
            } // Fin Throttling de Red
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}