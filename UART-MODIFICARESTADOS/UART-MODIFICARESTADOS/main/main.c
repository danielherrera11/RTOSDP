/* Includes */
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "freertos/timers.h"
#include "driver/adc.h"
#include "driver/uart.h"

/* Private Define*/

#define led_red 26
#define led_green 25
#define led_blue 33
#define led_warning 14
#define button_up 27
#define button_down 32

#define STACK_SIZE 2048

#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024


typedef enum temperature_control_
{
    COLD = 0,
    WARM,
    HOT,
    WARNING,
} temperature_control_t;

temperature_control_t temperature_control;



typedef enum state_machine_
{
    INIT = 0,
    COLOR_CHECK,
    PREAMBLE_CHECK,
    MINIM_CHECK,
    MAX_CHECK,
    CAUTIN,
    HISTERESIS,
} state_machine_t;

state_machine_t state_machine;

/* Private Global V */



uint8_t counter_prom=0;
float acumulable_temp = 0;

uint16_t blue_min = 20;
uint16_t blue_max = 30;

uint16_t green_min = 30;
uint16_t green_max = 40;

uint16_t red_min = 40;
uint16_t red_max = 50;
uint8_t led_to_change = 0;//RED = 1, BLUe = 2, GREEN = 3


uint8_t desired_temperature = 50;
uint8_t histeresis = 10;

TimerHandle_t xTimers;
TimerHandle_t xdebounceTimers;
int timerId = 1;
int debounce_timerId = 2;
uint8_t debounce_state =0;

float adc_val = 0;
float temperature = 0;

volatile uint8_t speed_value = 5;

uint8_t led_warning_state = 0;
/* Private function define*/

void peripheral_config(void);
void task_config(void);
void timer_config(void);
void isr_config(void);

void app_main(void)
{
    peripheral_config();
    task_config();
    timer_config();
    isr_config();

}

void peripheral_config(void){
    
    gpio_reset_pin(led_red);
    gpio_set_direction(led_red, GPIO_MODE_OUTPUT);

    gpio_reset_pin(led_green);
    gpio_set_direction(led_green, GPIO_MODE_OUTPUT);

    gpio_reset_pin(led_blue);
    gpio_set_direction(led_blue, GPIO_MODE_OUTPUT);

    gpio_reset_pin(led_warning);
    gpio_set_direction(led_warning, GPIO_MODE_OUTPUT);

    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);

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
}

/* Task Stack*/

static void temperature_task(void *pvParameters)
{
    /* Each sec read Temperature value*/
    while (1)
    {

        if(counter_prom < 10){
            float temporal_adc_value = 0;
            temporal_adc_value = adc1_get_raw(ADC1_CHANNEL_6);
            /*todo Config ADC to show Temperature*/
            temporal_adc_value = (temporal_adc_value*100)/4096;
            acumulable_temp = acumulable_temp + temporal_adc_value;
            counter_prom++;
            


        }else{

            temperature = acumulable_temp / 10;
            acumulable_temp = 0;
            counter_prom =0;

            if(temperature < desired_temperature){
                gpio_set_level(led_warning,1);
            }else{
                gpio_set_level(led_warning,0);
            }
            if (temperature > desired_temperature- histeresis){
                if(temperature< desired_temperature){
                    gpio_set_level(led_blue,1);
                }else{
                    gpio_set_level(led_blue,0);
                }
                if(temperature> desired_temperature){
                    gpio_set_level(led_red,1);
                }else{
                    gpio_set_level(led_red,0);
                }
                if(temperature < desired_temperature+histeresis){
                    gpio_set_level(led_green,1);
                }else{
                    gpio_set_level(led_green,0);
                }

            }
            else{
                gpio_set_level(led_green,0);
                gpio_set_level(led_red,0);
                gpio_set_level(led_blue,0);


            }


            

            char* Txdata = (char*) malloc(200);
            // sprintf (Txdata, "The temperature is : %f\r\n", temperature);
            sprintf (Txdata, "objetive : %d  histeresis: %d cautin:%f \r\n", desired_temperature, histeresis, temperature);
            uart_write_bytes(UART_NUM, Txdata, strlen(Txdata));

        }
        
        vTaskDelay(pdMS_TO_TICKS(100));

    }
}

static void uart_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    /* Each sec read Temperature value*/
    while (1)
    {
        bzero(data,BUF_SIZE);
        uint16_t temporal_min = 0;
        uint16_t temporal_max = 0;
        uint8_t index = 0;
        led_to_change = 0;
        state_machine = INIT;


        int len = uart_read_bytes(UART_NUM,data,BUF_SIZE,pdMS_TO_TICKS(100));

        while(index <= len){

            switch (state_machine)
            {
            case INIT:
                if(data[index] == '#'){
                
                    if((data[index]=='#')&&(data[index+1]=='S')&&(data[index+2]=='E')&&(data[index+3]=='T')&&(data[index+4]=='_')&&(data[index+5]=='H')&&(data[index+6]=='I')&&(data[index+7]=='S')&&(data[index+8]=='T')){
                        state_machine = HISTERESIS;
                        index = index + 8;

                    }
                    if((data[index]=='#')&&(data[index+1]=='S')&&(data[index+2]=='E')&&(data[index+3]=='T')&&(data[index+4]=='_')&&(data[index+5]=='T')&&(data[index+6]=='E')&&(data[index+7]=='M')&&(data[index+8]=='P')){
                        state_machine = CAUTIN;
                        index = index + 8;
                    }

                    
                }
                if((len-index)>5){

                    if((data[index]=='R')&&(data[index+1]=='E')&&(data[index+2]=='D')){
                        index = index + 2;
                        led_to_change = 1;
                        state_machine = PREAMBLE_CHECK;
                    }
                    if((data[index]=='B')&&(data[index+1]=='L')&&(data[index+2]=='U')&&(data[index+3]=='E')){
                        index = index + 3;
                        led_to_change = 2;
                        state_machine = PREAMBLE_CHECK;
                    }
                    if((data[index]=='G')&&(data[index+1]=='R')&&(data[index+2]=='E')&&(data[index+3]=='E')&&(data[index+4]=='N')){
                        index = index + 4;
                        led_to_change = 3;
                        state_machine = PREAMBLE_CHECK;
                    }
                }
                break;
            
            case PREAMBLE_CHECK:
                if(data[index] == '$'){
                    state_machine = MINIM_CHECK;
                }else{
                    state_machine = INIT;
                }
                break;

            case MINIM_CHECK:
                if(data[index] == '$'){
                    state_machine = MAX_CHECK;
                }else{
                    if((data[index] < 48) || (data[index] > 57)){
                        state_machine = INIT;
                    }
                    else{
                        temporal_min = temporal_min*10;
                        temporal_min = temporal_min+(data[index]-48);
                    }
                    
                }
                break;

            case MAX_CHECK:
                if(data[index] == '$'){
                    if(temporal_min <= temporal_max){
                        if(led_to_change == 1){
                            red_max = temporal_max;
                            red_min = temporal_min;
                        }
                        if(led_to_change == 2){
                            blue_max = temporal_max;
                            blue_min = temporal_min;
                        }
                        if(led_to_change == 3){
                            green_max = temporal_max;
                            green_min = temporal_min;
                        }
                    }
                }else{
                    if((data[index] < 48) || (data[index] > 57)){
                        state_machine = INIT;
                    }
                    else{
                        temporal_max = temporal_max*10;
                        temporal_max = temporal_max+(data[index]-48);
                    }
                }
                break;

            case CAUTIN:
                
                if((data[index]=='$')&&(data[index+3]=='$')){
                    if(((data[index+1] < 48) || (data[index+1] > 57))||((data[index+2] < 48) || (data[index+2] > 57))){
                        state_machine = INIT;
                    }else{
                        uint8_t acumulator = 0;
                        acumulator = data[index+1] - 48;
                        acumulator = acumulator * 10;
                        acumulator = acumulator + (data[index+2] - 48);
                        desired_temperature = acumulator;
                    }

                }
                

                break;

            case HISTERESIS:
                if((len-index)>3){
                    if((data[index]=='$')&&(data[index+3]=='$')){
                        if(((data[index+1] < 48) || (data[index+1] > 57))||((data[index+2] < 48) || (data[index+2] > 57))){
                            state_machine = INIT;
                        }else{
                            uint8_t acumulator = 0;
                            acumulator = data[index+1] - 48;
                            acumulator = acumulator * 10;
                            acumulator = acumulator + (data[index+2] - 48);
                            histeresis = acumulator;
                        }

                    }
                }

                break;

            default:
                break;
            
            }
            
            index++;
        }
        
        
        
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
}

void task_config(void){
    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    xTaskCreate(temperature_task,
                "temperature_task", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                STACK_SIZE,
                &ucParameterToPass,
                1,
                &xHandle);
    
        xTaskCreate(uart_task,
                "uart_task", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
                STACK_SIZE,
                &ucParameterToPass,
                1,
                &xHandle);
}

/* Timer stack*/

static void timer_cb(TimerHandle_t pxTimer){
    // if(adc_val > 3){
    //     gpio_set_level(led_warning,led_warning_state);
    //     if(led_warning_state == 0){
    //         led_warning_state =1;
    //     }else{
    //         led_warning_state = 0;
    //     }
    // }
    timer_config();

}

static void debounce_timer_cb(TimerHandle_t pxTimer){
    debounce_state =0;

}

void timer_config(void){
    

    xTimers = xTimerCreate("Timer",                         // Just a text name, not used by the kernel.
                           (pdMS_TO_TICKS(speed_value*100)), // The timer period in ticks.
                           pdFALSE,                          // The timers will auto-reload themselves when they expire.
                           (void *)timerId,                 // Assign each timer a unique id equal to its array index.
                           timer_cb                   // Each timer calls the same callback when it expires.
    );

    if (xTimerStart(xTimers, 0) != pdPASS)
    {
        // The timer could not be set into the Active state.
        printf("timer fail \r\n");
    }
        xdebounceTimers = xTimerCreate("Debounce_timer",                         // Just a text name, not used by the kernel.
                           (pdMS_TO_TICKS(200)), // The timer period in ticks.
                           pdTRUE,                          // The timers will auto-reload themselves when they expire.
                           (void *)debounce_timerId,                 // Assign each timer a unique id equal to its array index.
                           debounce_timer_cb                   // Each timer calls the same callback when it expires.
    );
    if (xTimerStart(xdebounceTimers, 0) != pdPASS)
    {
        // The timer could not be set into the Active state.
        printf("timer fail \r\n");
    }
 
}

/* Interruption stack*/

/* Private CB*/

void button1_handler(void *args){
    if(debounce_state){
        return;
    }
    debounce_state =1;

    if(desired_temperature > 5){
        desired_temperature = desired_temperature -5;
    }

}

void button2_handler(void *args){
    if(debounce_state){
        return;
    }
    debounce_state =1;

    if(desired_temperature < 95){
        desired_temperature = desired_temperature + 5;
    }

}


void isr_config(void){
    gpio_config_t pGPIOConfig1;
    

    pGPIOConfig1.pin_bit_mask = (1ULL << button_down);
    pGPIOConfig1.mode = GPIO_MODE_DEF_INPUT;
    pGPIOConfig1.pull_up_en = GPIO_PULLUP_ENABLE;
    pGPIOConfig1.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pGPIOConfig1.intr_type = GPIO_INTR_NEGEDGE;

    gpio_config_t pGPIOConfig2;
    pGPIOConfig2.pin_bit_mask = (1ULL << button_up);
    pGPIOConfig2.mode = GPIO_MODE_DEF_INPUT;
    pGPIOConfig2.pull_up_en = GPIO_PULLUP_ENABLE;
    pGPIOConfig2.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pGPIOConfig2.intr_type = GPIO_INTR_NEGEDGE;


    gpio_config(&pGPIOConfig1);
    gpio_config(&pGPIOConfig2);

    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(button_up , button2_handler , NULL);
    // gpio_install_isr_service(0);
    gpio_isr_handler_add(button_down , button1_handler , NULL);
    



}
