// librerias
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "freertos/timers.h"
#include "driver/adc.h"
#include "driver/uart.h"

// definir constantes
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024
#define TASK_MEMORY 1024 *4

#define led_red 33
#define led_green 25
#define led_blue 26
#define button 27
#define STACK_SIZE 1024*2 

// estructura emumerate
typedef enum state_machine_
{
    RED = 0,
    GREEN,
    BLUE,

} state_machine_t;

state_machine_t state_machine;
// definir estructura timer
TimerHandle_t xTimers;
int interval_timer = 50;
int timerId = 1;
// variables ciclo util
int duty_red = 0;
int duty_green = 0;
int duty_blue = 0;
// definir variable que guarda los datos del ADC
int adc_val = 0;
// bandera
uint8_t falling = 0;
//predefinir funciones locales que se van a usar para que no existan definiciones implicitas al compilar
esp_err_t set_pwm(void);
esp_err_t set_pwm_duty(void);
esp_err_t set_timer(void);
esp_err_t create_task(void);
esp_err_t set_adc(void);

void vTimerCallback(TimerHandle_t pxTimer);

void isr_handler(void *args);
void vTaskbutton(void *pvParameters);
void vTaskadc(void *pvParameters);

void read_adc(void){
    adc_val = adc1_get_raw(ADC1_CHANNEL_6);
    
}


// configurar UART

static void uart_task(void *pvParameters){
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);

    while(1)
    {
        bzero(data,BUF_SIZE);
        int len = uart_read_bytes(UART_NUM,data,BUF_SIZE,pdMS_TO_TICKS(100));
        uint8_t value = data[0];
        
        switch (value)
        {
        case 'R':
            duty_red = 0;
            duty_green = 0;
            duty_blue = 0;
            falling = 0;
            state_machine = RED;
            break;

        case 'G':
            duty_red = 0;
            duty_green = 0;
            duty_blue = 0;
            falling = 0;
            state_machine = GREEN;
            break;

        case 'B':
            duty_red = 0;
            duty_green = 0;
            duty_blue = 0;
            falling = 0;
            state_machine = BLUE;
            break;

        case 'T':
            adc_val = adc1_get_raw(ADC1_CHANNEL_6);
            char* Txdata = (char*) malloc(100);
            sprintf (Txdata, "ADC VAL : %d\r\n", adc_val);
            uart_write_bytes(UART_NUM, Txdata, strlen(Txdata));

            break;

        default:
            break;
        }
        
        

        // vTaskDelay(pdMS_TO_TICKS(10));
        

    }

}

static void init_uart(void ){
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM,1,3,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM,BUF_SIZE,BUF_SIZE,0,NULL,0);

        xTaskCreate(uart_task,
                "Uart_Task", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                TASK_MEMORY,
                NULL,
                5,
                NULL);

}

// progrmar principal
void app_main(void)
{
    // setear pines
    gpio_reset_pin(button);
    gpio_set_direction(button,GPIO_MODE_DEF_INPUT); // como entrada
    gpio_set_pull_mode(button,GPIO_PULLUP_ONLY); // pull up. esta permanentemente en 3.3v
    set_pwm();
    set_timer();
    create_task();
    init_uart();
}

//  setear los parametros del PWM
esp_err_t set_pwm(void)
{
    //rojo
    ledc_channel_config_t channelConfigR = {0};
    channelConfigR.gpio_num = led_red;
    channelConfigR.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfigR.channel = LEDC_CHANNEL_0;
    channelConfigR.intr_type = LEDC_INTR_DISABLE;
    channelConfigR.timer_sel = LEDC_TIMER_0;
    channelConfigR.duty = 0;
    // verde
    ledc_channel_config_t channelConfigG = {0};
    channelConfigG.gpio_num = led_green;
    channelConfigG.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfigG.channel = LEDC_CHANNEL_1;
    channelConfigG.intr_type = LEDC_INTR_DISABLE;
    channelConfigG.timer_sel = LEDC_TIMER_0;
    channelConfigG.duty = 0;
    // azul
    ledc_channel_config_t channelConfigB = {0};
    channelConfigB.gpio_num = led_blue;
    channelConfigB.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfigB.channel = LEDC_CHANNEL_2;
    channelConfigB.intr_type = LEDC_INTR_DISABLE;
    channelConfigB.timer_sel = LEDC_TIMER_0;
    channelConfigB.duty = 0;
    // configuracion canal LEDC (PWM)
    ledc_channel_config(&channelConfigR);
    ledc_channel_config(&channelConfigG);
    ledc_channel_config(&channelConfigB);
    // configuracion de los parametros del timer del canal del PWM
    ledc_timer_config_t timerConfig = {0};
    timerConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
    timerConfig.duty_resolution = LEDC_TIMER_10_BIT;
    timerConfig.timer_num = LEDC_TIMER_0;
    timerConfig.freq_hz = 20000;
    // configuracion canal TIMER
    ledc_timer_config(&timerConfig);

    return ESP_OK;
}

// configurar el ciclo util al que van a funcionar los leds
esp_err_t set_pwm_duty(void)
{
    //setear
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty_red);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty_green);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, duty_blue);
    // actualizar
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);

    return ESP_OK;
}
// funcion cuando pasa el time out
void vTimerCallback(TimerHandle_t pxTimer)
{


    if (falling == 0)
    {   // aumentar intensidad
        switch (state_machine)
        {
        case RED:
            duty_red = duty_red + 20;
            break;

        case GREEN:
            duty_green = duty_green + 20;
            break;

        case BLUE:
            duty_blue = duty_blue + 20;
            break;

        default:
            break;
        }
    }
    if (falling == 1)
    {   // bajar intensidad
        switch (state_machine)
        {
        case RED:
            duty_red = duty_red - 20;
            break;

        case GREEN:
            duty_green = duty_green - 20;
            break;

        case BLUE:
            duty_blue = duty_blue - 20;
            break;

        default:
            break;
        }
    }
    // cambio de la bandera al llegar a los extremos
    if (((duty_red + duty_blue + duty_green) >= 1000) && (falling == 0))
    {
        falling = 1;
    }

    if (((duty_red + duty_blue + duty_green) <= 40) && (falling == 1))
    {
        falling = 0;
    }

    set_pwm_duty();
}
// configuracion timer
esp_err_t set_timer(void)
{
    printf("timer init \r\n");
    xTimers = xTimerCreate("Timer",                         // Just a text name, not used by the kernel.
                           (pdMS_TO_TICKS(interval_timer)), // The timer period in ticks.
                           pdTRUE,                          // The timers will auto-reload themselves when they expire.
                           (void *)timerId,                 // Assign each timer a unique id equal to its array index.
                           vTimerCallback                   // Each timer calls the same callback when it expires.
    );

    if (xTimers == NULL)
    {
        printf("timer fail \r\n");
    }
    else
    {

        if (xTimerStart(xTimers, 0) != pdPASS)
        {
            // The timer could not be set into the Active state.
            printf("timer fail \r\n");
        }
    }

    return ESP_OK;
}
// RTOS tarea
esp_err_t create_task(void)
{

    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    xTaskCreate(vTaskbutton,
                "Button_Task", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                STACK_SIZE,
                &ucParameterToPass,
                1,
                &xHandle);

        
        return ESP_OK;
}

void vTaskbutton(void *pvParameters)
{
    while (1)
    {
        if(gpio_get_level(button)==0){
            adc_val = adc1_get_raw(ADC1_CHANNEL_6);
            char* Txdata = (char*) malloc(100);
            sprintf (Txdata, "ADC VAL : %d\r\n", adc_val);
            uart_write_bytes(UART_NUM, Txdata, strlen(Txdata));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t set_adc(void)
{
    // setear cual pin adc se usara asi como la atenuacion
    esp_err_t adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    // CONFIGURAR DE CUANTOS BITS ES LA MEDIDA
    esp_err_t adc1_config_width(ADC_WIDTH_BIT_12);
    return ESP_OK;
}