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
#include <math.h>

/* Private Define*/

#define led_red 26
#define led_green 25
#define led_blue 33
#define led_warning 14
#define button_up 27
#define button_down 32

#define STACK_SIZE 2048

// define the UART that is connected to the USB port
#define UART_NUM UART_NUM_0
// buffer size to recieve Rx data
#define BUF_SIZE 1024

// define variables of state machine
typedef enum temperature_control_
{
    COLD = 0,
    WARM,
    HOT,
    WARNING,
} temperature_control_t;

/* Private Global V */

temperature_control_t temperature_control;

// define timer functions 
TimerHandle_t xTimers;
TimerHandle_t xdebounceTimers;
int timerId = 1;
int debounce_timerId = 2;
uint8_t debounce_state =0;

// define adc valor 
float adc_val = 0;
// define standar values of the termistor
double Beta = 3950.0;  // Beta value
double To = 25;    // Temperature standar in Celsius
double Ro = 5;   // Resistance of Thermistor at 25 degree Celsius
float Rt = 0;
float T = 0;

// speed value of change for the LED
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
    // set and direct GPIO pins
    gpio_reset_pin(led_red);
    gpio_set_direction(led_red, GPIO_MODE_OUTPUT);

    gpio_reset_pin(led_green);
    gpio_set_direction(led_green, GPIO_MODE_OUTPUT);

    gpio_reset_pin(led_blue);
    gpio_set_direction(led_blue, GPIO_MODE_OUTPUT);

    gpio_reset_pin(led_warning);
    gpio_set_direction(led_warning, GPIO_MODE_OUTPUT);

    // set and width of the ADC
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    // configuration UART
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
        adc_val = adc1_get_raw(ADC1_CHANNEL_6);
        /*Config ADC to show Temperature*/
        adc_val = (adc_val*3.3)/4096;
        // Resistance of the termistor
        Rt = 15 * adc_val / (3.3 - adc_val);
        // temperature in celsius
        T = 1/(1/To + log(Rt/Ro)/Beta);

        // states of the LEDS
        if(T <= 25){
            temperature_control = COLD;
            gpio_set_level(led_red,0);
            gpio_set_level(led_green,0);
            gpio_set_level(led_blue,1);
            gpio_set_level(led_warning,0);
        }else if (T <= 26)
        {
            temperature_control = WARM;
            gpio_set_level(led_red,0);
            gpio_set_level(led_green,1);
            gpio_set_level(led_blue,0);
            gpio_set_level(led_warning,0);
        }else if (T <= 27)
        {
            temperature_control = HOT;
            gpio_set_level(led_red,1);
            gpio_set_level(led_green,0);
            gpio_set_level(led_blue,0);
            gpio_set_level(led_warning,0);
        }else{
            temperature_control = WARNING;
            gpio_set_level(led_red,1);
            gpio_set_level(led_green,1);
            gpio_set_level(led_blue,1);
    
        }
        //print on monitor
        char* Txdata = (char*) malloc(100);
        sprintf (Txdata, "The temperature is : %f\r\n", T);
        

        uart_write_bytes(UART_NUM, Txdata, strlen(Txdata));
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
}

/* Timer stack*/
// callback timer, every time the timer runs off the speed value will change depending on the new speed value after button press
static void timer_cb(TimerHandle_t pxTimer){
    if(adc_val > 3){
        gpio_set_level(led_warning,led_warning_state);
        if(led_warning_state == 0){
            led_warning_state =1;
        }else{
            led_warning_state = 0;
        }
    }
    timer_config();

}

// this function avoid debouncing problem with the buttons
static void debounce_timer_cb(TimerHandle_t pxTimer){
    debounce_state =0;

}

// timer configuration
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
// function when i press button button down
void button1_handler(void *args){
    if(debounce_state){
        return;
    }
    debounce_state =1;

    if(speed_value > 1){
        speed_value = speed_value -1;
    }

}

// function when i press button button up
void button2_handler(void *args){
    if(debounce_state){
        return;
    }
    debounce_state =1;

    if(speed_value < 20){
        speed_value = speed_value + 1;
    }

}

// configuration interruption
void isr_config(void){
    gpio_config_t pGPIOConfig1;
    
    // set timers
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
