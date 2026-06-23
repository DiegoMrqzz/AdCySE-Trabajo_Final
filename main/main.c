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

//CÓDIGO HTML
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

//DEFINICIONES====================================================
#define numMuestras 512
#define frecuenciaMuestreo 32000 //Frecuencia de muestreo de 32 kHz
#define pinMicrofono ADC_CHANNEL_0  //GPIO0 del ESP32
#define pinClock 2
#define pinDatos 3
#define pinLatch 10

//VARIABLES FFT===================================================
float varReal[numMuestras];
float varImaginaria[numMuestras];

static int ws_client_fd=-1;
static httpd_handle_t global_server=NULL;

//Handler que envía el HTML al entrar a la IP
esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

//Handler que acepta conexión del socket
esp_err_t ws_handler(httpd_req_t *req){
    ws_client_fd=httpd_req_to_sockfd(req);
    global_server=req->handle;
    if(req->method==HTTP_GET){
        printf("Conectado, lcdll\n");
        return ESP_OK;
    }
    return ESP_OK;
}

//Función remueve offset DC
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

//Función para iniciar pines
void iniciarPinesLED(){
    gpio_reset_pin(pinClock);
    gpio_reset_pin(pinDatos);
    gpio_reset_pin(pinLatch);
    gpio_set_direction(pinClock, GPIO_MODE_OUTPUT);
    gpio_set_direction(pinDatos, GPIO_MODE_OUTPUT);
    gpio_set_direction(pinLatch, GPIO_MODE_OUTPUT);
}

//Función para enviar datos al desplazador de registros
void enviar74HC595(uint8_t *datos, int numColumnas){
    for(int i=numColumnas-1; i>=0; i--){
        uint8_t auxDatos=datos[i];
        for(int j=7; j>=0; j--){
            gpio_set_level(pinDatos, (auxDatos>>j)&1);
            esp_rom_delay_us(1);
            gpio_set_level(pinClock, 1);
            esp_rom_delay_us(1);
            gpio_set_level(pinClock, 0);
            esp_rom_delay_us(1);
        }
    }
    gpio_set_level(pinLatch, 1);
    esp_rom_delay_us(1);
    gpio_set_level(pinLatch, 0);
}

//Fast Fourier Transform (FFT)
void calcularFFT(float *varReal, float *varImaginaria, uint16_t muestras){
    uint16_t j=0;
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
        while(k>0&&k<=j){
            j-=k;
            k>>=1;
        }
        j+=k;
    }

    //Calculo FFT (Cooley-Tukey)
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

//Calculo magnitud de cada nota
void calcularMagnitud(float *varReal, float *varImaginaria, uint16_t muestras){
    for(uint16_t i=0; i<muestras/2; i++){
        varReal[i]=sqrtf((varReal[i]*varReal[i])+(varImaginaria[i]*varImaginaria[i]));
    }
}

adc_continuous_handle_t adc_handle=NULL;

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

//FUNCIÓN MAIN====================================================
void app_main(void){
    //Inicio memoria y red
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

    //Inicio servidor web
    httpd_config_t config_server=HTTPD_DEFAULT_CONFIG();
    config_server.lru_purge_enable=true;
    if (httpd_start(&global_server, &config_server)==ESP_OK) {
        httpd_uri_t uri_get={.uri="/", .method=HTTP_GET, .handler=index_handler};
        httpd_register_uri_handler(global_server, &uri_get);
        httpd_uri_t ws_uri={.uri="/ws", .method=HTTP_GET, .handler=ws_handler, .is_websocket=true};
        httpd_register_uri_handler(global_server, &ws_uri);
    }


    //Inicio LEDs, apagados
    iniciarADC();
    iniciarPinesLED();

    uint8_t bufferBasura[numMuestras*SOC_ADC_DIGI_RESULT_BYTES];
    uint32_t bytesBasura = 0;
    adc_continuous_read(adc_handle, bufferBasura, sizeof(bufferBasura), &bytesBasura, 0);

    //Loop principal
    while(1){
            static uint8_t bufferRAW[numMuestras*SOC_ADC_DIGI_RESULT_BYTES];
            uint32_t bytesLeidos=0;
            esp_err_t ret=adc_continuous_read(adc_handle, bufferRAW, sizeof(bufferRAW), &bytesLeidos, portMAX_DELAY);
            if(ret==ESP_OK){
                adc_digi_output_data_t *p=(adc_digi_output_data_t*)bufferRAW;
                int totalMuestras=bytesLeidos/SOC_ADC_DIGI_RESULT_BYTES;
            //Copio muestras de ADC a FFT
                for(int i=0; i<numMuestras; i++){
                    if(i<totalMuestras){
                        varReal[i]=(float)(p[i].type2.data);
                    }else{
                        varReal[i]=0.0f;
                    }
                    varImaginaria[i]=0.0f;  //Parte imaginaria empieza en 0
                }
                //Proceso señal
                removerCC(varReal, numMuestras);
                calcularFFT(varReal, varImaginaria, numMuestras);
                calcularMagnitud(varReal, varImaginaria, numMuestras);

                //Guardo estado de columnas
                int nivelesLED[7];
                uint8_t bufferColumnas[7]={0, 0, 0, 0, 0, 0, 0};

                //Ajusto tope según microfono y hago regla de 3 para determinar cantidad de LEDs prendidos (Vúmetro, 8 LEDs)
                //Bajar tope si el amplificador del microfono es malo, subirlo si es bueno; comparar silencio vs. ruido
                //Determino piso según el ruido que detecte el microfono en silencio
                float energiaTope=40000.0f;
                float energiaPiso=1000.0f;
                //Evaluo notas a 32kHz con 512 muestras (62.5Hz por bin)
                int limiteBandas[8]={1, 3, 6, 13, 29, 61, 126, 256};
                for(int i=0; i<7; i++){
                    float energiaSuma=0;
                    int inicio=limiteBandas[i], fin=limiteBandas[i+1];
                    int cantidadBins=fin-inicio;
                    for(int j=inicio; j<fin; j++){
                        energiaSuma+=varReal[j];
                    }
                    float energiaNota=energiaSuma/cantidadBins;
                    int prenderLED=0;

                    if(energiaNota>energiaPiso){
                        float dBNota=10.0f*log10f(energiaNota);
                        float dbPiso=10.0f*log10f(energiaPiso);
                        float dBTope=10.0f*log10f(energiaTope);

                        //Regla de 3
                        float porcentaje=(dBNota-dbPiso)/(dBTope-dbPiso);
                        prenderLED=(int)(porcentaje*8.0f);
                    }

                    if(prenderLED>8){
                        prenderLED=8;
                    }
                    if(prenderLED<0){
                        prenderLED=0;
                    }

                    nivelesLED[i]=prenderLED;

                    //Byte para columna
                    uint8_t barra=0;
                    for(int j=0; j<prenderLED; j++){
                        barra|=(1<<j);
                    }
                    bufferColumnas[i]=barra;
                }

                enviar74HC595(bufferColumnas, 7);
            
                //Envío a la página web
                if (ws_client_fd!=-1 && global_server!=NULL){
                    static char datos_ws[32];
                    //Arreglo [2, 8, 0...] a texto "2,8,0..."
                    snprintf(datos_ws, sizeof(datos_ws), "%d,%d,%d,%d,%d,%d,%d", nivelesLED[0], nivelesLED[1], nivelesLED[2], nivelesLED[3], nivelesLED[4], nivelesLED[5], nivelesLED[6]);
                    httpd_ws_frame_t ws_pkt;
                    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
                    ws_pkt.payload=(uint8_t*)datos_ws;
                    ws_pkt.len=strlen(datos_ws);
                    ws_pkt.type=HTTPD_WS_TYPE_TEXT;
                    esp_err_t err_ws = httpd_ws_send_frame_async(global_server, ws_client_fd, &ws_pkt);
                    if (err_ws != ESP_OK) {
                        ws_client_fd = -1;
                    }
                }
            }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}