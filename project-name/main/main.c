#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "freertos/timers.h"

#define led_red 33
#define led_green 25
#define led_blue 26
#define button 27
#define STACK_SIZE 1024


typedef enum state_machine_
{
    RED = 0,
    GREEN,
    BLUE,

} state_machine_t;

state_machine_t state_machine;

TimerHandle_t xTimers;
int interval_timer = 50;
int timerId = 1;

int duty_red = 0;
int duty_green = 0;
int duty_blue = 0;

uint8_t falling = 0;

esp_err_t set_pwm(void);
esp_err_t set_pwm_duty(void);
esp_err_t set_timer(void);
esp_err_t create_task(void);

void vTimerCallback(TimerHandle_t pxTimer);

void isr_handler(void *args);
void vTaskbutton(void *pvParameters);


void app_main(void)
{
    
    gpio_reset_pin(button);
    gpio_set_direction(button,GPIO_MODE_DEF_INPUT);
    gpio_set_pull_mode(button,GPIO_PULLUP_ONLY);
    set_pwm();
    set_timer();
    create_task();
}


esp_err_t set_pwm(void)
{

    ledc_channel_config_t channelConfigR = {0};
    channelConfigR.gpio_num = led_red;
    channelConfigR.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfigR.channel = LEDC_CHANNEL_0;
    channelConfigR.intr_type = LEDC_INTR_DISABLE;
    channelConfigR.timer_sel = LEDC_TIMER_0;
    channelConfigR.duty = 0;

    ledc_channel_config_t channelConfigG = {0};
    channelConfigG.gpio_num = led_green;
    channelConfigG.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfigG.channel = LEDC_CHANNEL_1;
    channelConfigG.intr_type = LEDC_INTR_DISABLE;
    channelConfigG.timer_sel = LEDC_TIMER_0;
    channelConfigG.duty = 0;

    ledc_channel_config_t channelConfigB = {0};
    channelConfigB.gpio_num = led_blue;
    channelConfigB.speed_mode = LEDC_HIGH_SPEED_MODE;
    channelConfigB.channel = LEDC_CHANNEL_2;
    channelConfigB.intr_type = LEDC_INTR_DISABLE;
    channelConfigB.timer_sel = LEDC_TIMER_0;
    channelConfigB.duty = 0;

    ledc_channel_config(&channelConfigR);
    ledc_channel_config(&channelConfigG);
    ledc_channel_config(&channelConfigB);

    ledc_timer_config_t timerConfig = {0};
    timerConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
    timerConfig.duty_resolution = LEDC_TIMER_10_BIT;
    timerConfig.timer_num = LEDC_TIMER_0;
    timerConfig.freq_hz = 20000;

    ledc_timer_config(&timerConfig);

    return ESP_OK;
}

esp_err_t set_pwm_duty(void)
{

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty_red);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty_green);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, duty_blue);

    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);

    return ESP_OK;
}

void vTimerCallback(TimerHandle_t pxTimer)
{

    if (falling == 0)
    {
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
    {
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
            duty_red = 0;
            duty_green = 0;
            duty_blue = 0;
            falling = 0;
            switch (state_machine)
            {
            case RED:
                state_machine = GREEN;
                break;

            case GREEN:
                state_machine = BLUE;
                break;

            case BLUE:
                state_machine = RED;
                break;

            default:
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}