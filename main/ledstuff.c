/* Ledc fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ledstuff.h"

/*
 * About this example
 * 1. init LEDC module:
 *    a. You need to set the timer of LEDC first, this decide the frequency and resolution of PWM.
 *    b. You need to set the LEDC channel you want to use, and bind with one of the timers.
 *
 * 2. You can install a default fade function, then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This example use GPIO18/19/4/5 as LEDC ouput, and it will change the duty repeatedly.
 *
 * 5. GPIO18/19 are from high speed channel group. GPIO4/5 are from low speed channel group.
 *
 */
#define LEDC_HS_TIMER LEDC_TIMER_0
#define LEDC_HS_MODE LEDC_HIGH_SPEED_MODE

#define LEDC_HS_CH0_GPIO (18)
#define LEDC_HS_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO (19)
#define LEDC_HS_CH1_CHANNEL LEDC_CHANNEL_1

#define LEDC_LS_TIMER LEDC_TIMER_1
#define LEDC_LS_MODE LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH2_GPIO (4)
#define LEDC_LS_CH2_CHANNEL LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO (5)
#define LEDC_LS_CH3_CHANNEL LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM (4)

typedef struct
{
    int channel;
    int io;
    int mode;
    int timer_idx;
} ledc_info_t;

ledc_info_t firstStrip[3];
ledc_info_t secondStrip[3];
ledc_info_t thirdStrip[3];

#define TAG "udp_perf:"

void setStrip(ledc_info_t strip[], unsigned char red, unsigned char green, unsigned char blue)
{
    //temp
    ledc_set_duty(strip[0].mode, strip[0].channel, 5000 - red * 5000.0 / 255);
    ledc_update_duty(strip[0].mode, strip[0].channel);
    ledc_set_duty(strip[1].mode, strip[1].channel, 5000 - green * 5000.0 / 255);
    ledc_update_duty(strip[1].mode, strip[1].channel);
    ledc_set_duty(strip[2].mode, strip[2].channel, 5000 - blue * 5000.0 / 255);
    ledc_update_duty(strip[2].mode, strip[2].channel);
}

void setFadeStrip(ledc_info_t strip[], unsigned char red, unsigned char green, unsigned char blue, unsigned char delay)
{
    ledc_set_fade_with_time(strip[0].mode, strip[0].channel, 5000 - red * 5000.0 / 255, delay);
    ledc_fade_start(strip[0].mode, strip[0].channel, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(strip[1].mode, strip[1].channel, 5000 - green * 5000.0 / 255, delay);
    ledc_fade_start(strip[1].mode, strip[1].channel, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(strip[2].mode, strip[2].channel, 5000 - blue * 5000.0 / 255, delay);
    ledc_fade_start(strip[2].mode, strip[2].channel, LEDC_FADE_NO_WAIT);
}

void testActions(stripAction actions[], int size)
{
    ESP_LOGI(TAG, "BEGIN\n");
    for (int i = 0; i < 600; i++)
    {
        for (int j = 0; j < size; j++)
        {
            if (actions[j].ms == i)
            {
                ESP_LOGI(TAG, "MS_MATCH:%d %d %d %d\n", actions[j].ms, i, actions[j].strip, j);
                //setColor
                if (actions[j].mode == 0)
                {
                    if (actions[j].strip == 1)
                    {
                        //setStrip(firstStrip, actions[j].red, actions[j].green, actions[j].blue);
                        ESP_LOGI(TAG, "SET FIRST STRIP:%d\n", actions[j].ms);
                        setStrip(firstStrip, actions[j].red, actions[j].green, actions[j].blue);
                    }
                    else if (actions[j].strip == 2)
                    {
                        //setStrip(secondStrip, actions[j].red, actions[j].green, actions[j].blue);
                        ESP_LOGI(TAG, "SET SECOND STRIP:%d\n", actions[j].ms);
                        setStrip(secondStrip, actions[j].red, actions[j].green, actions[j].blue);
                    }
                    else if (actions[j].strip == 3)
                    {
                        //setStrip(thirdStrip, actions[j].red, actions[j].green, actions[j].blue);
                        ESP_LOGI(TAG, "SET THIRD STRIP:%d\n", actions[j].ms);
                        setStrip(thirdStrip, actions[j].red, actions[j].green, actions[j].blue);
                    }
                }
                else
                {
                    if (actions[j].strip == 1)
                    {
                        //setStrip(firstStrip, actions[j].red, actions[j].green, actions[j].blue);
                        ESP_LOGI(TAG, "SET FIRST STRIP DELAY:%d\n", actions[j].ms);
                        setFadeStrip(firstStrip, actions[j].red, actions[j].green, actions[j].blue, actions[j].delay);
                    }
                    else if (actions[j].strip == 2)
                    {
                        //setStrip(secondStrip, actions[j].red, actions[j].green, actions[j].blue);
                        ESP_LOGI(TAG, "SET SECOND STRIP DELAY:%d\n", actions[j].ms);
                        setFadeStrip(secondStrip, actions[j].red, actions[j].green, actions[j].blue, actions[j].delay);
                    }
                    else if (actions[j].strip == 3)
                    {
                        //setStrip(thirdStrip, actions[j].red, actions[j].green, actions[j].blue);
                        ESP_LOGI(TAG, "SET THIRD STRIP DELAY:%d\n", actions[j].ms);
                        setFadeStrip(thirdStrip, actions[j].red, actions[j].green, actions[j].blue, actions[j].delay);
                    }
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    setStrip(firstStrip, 0, 0, 0);
    setStrip(secondStrip, 0, 0, 0);
    setStrip(thirdStrip, 0, 0, 0);
    ESP_LOGI(TAG, "END\n");
}

void setupPWM()
{
    ESP_LOGI(TAG, "SETUP_PWM_BEGIN\n");
    int ch;

    ledc_info_t firstStripTmp[3] = {{.channel = LEDC_CHANNEL_0, .io = (18), .mode = LEDC_HS_MODE, .timer_idx = LEDC_HS_TIMER}, {.channel = LEDC_CHANNEL_1, .io = (19), .mode = LEDC_HS_MODE, .timer_idx = LEDC_HS_TIMER}, {.channel = LEDC_CHANNEL_2, .io = (5), .mode = LEDC_HS_MODE, .timer_idx = LEDC_HS_TIMER}};

    ledc_info_t secondStripTmp[3] = {
        {.channel = LEDC_CHANNEL_3,
         .io = (21),
         .mode = LEDC_HS_MODE,
         .timer_idx = LEDC_HS_TIMER},
        {.channel = LEDC_CHANNEL_4,
         .io = (23),
         .mode = LEDC_HS_MODE,
         .timer_idx = LEDC_HS_TIMER},
        {.channel = LEDC_CHANNEL_5,
         .io = (22),
         .mode = LEDC_HS_MODE,
         .timer_idx = LEDC_HS_TIMER}};

    ledc_info_t thirdStripTmp[3] = {
        {.channel = LEDC_CHANNEL_6,
         .io = (4),
         .mode = LEDC_HS_MODE,
         .timer_idx = LEDC_HS_TIMER},
        {.channel = LEDC_CHANNEL_0,
         .io = (15),
         .mode = LEDC_LS_MODE,
         .timer_idx = LEDC_LS_TIMER},
        {.channel = LEDC_CHANNEL_1,
         .io = (2),
         .mode = LEDC_LS_MODE,
         .timer_idx = LEDC_LS_TIMER}};

    for (int i = 0; i < 3; i++)
    {
        firstStrip[i] = firstStripTmp[i];
        secondStrip[i] = secondStripTmp[i];
        thirdStrip[i] = thirdStripTmp[i];
    }

    ledc_timer_config_t ledc_timer = {
        .bit_num = LEDC_TIMER_13_BIT, //set timer counter bit number
        .freq_hz = 5000,              //set frequency of pwm
        .speed_mode = LEDC_HS_MODE,   //timer mode,
        .timer_num = LEDC_HS_TIMER    //timer index
    };
    //configure timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    //configure timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_LS_MODE;
    ledc_timer.timer_num = LEDC_LS_TIMER;
    ledc_timer_config(&ledc_timer);

    for (ch = 0; ch < 3; ch++)
    {
        ledc_channel_config_t ledc_channel = {
            //set LEDC channel 0
            .channel = firstStrip[ch].channel,
            //set the duty for initialization.(duty range is 0 ~ ((2**bit_num)-1)
            .duty = 0,
            //GPIO number
            .gpio_num = firstStrip[ch].io,
            //GPIO INTR TYPE, as an example, we enable fade_end interrupt here.
            .intr_type = LEDC_INTR_FADE_END,
            //set LEDC mode, from ledc_mode_t
            .speed_mode = firstStrip[ch].mode,
            //set LEDC timer source, if different channel use one timer,
            //the frequency and bit_num of these channels should be the same
            .timer_sel = firstStrip[ch].timer_idx,
        };
        //set the configuration
        ledc_channel_config(&ledc_channel);
        ledc_channel_config_t ledc_channel2 = {
            //set LEDC channel 0
            .channel = secondStrip[ch].channel,
            //set the duty for initialization.(duty range is 0 ~ ((2**bit_num)-1)
            .duty = 0,
            //GPIO number
            .gpio_num = secondStrip[ch].io,
            //GPIO INTR TYPE, as an example, we enable fade_end interrupt here.
            .intr_type = LEDC_INTR_FADE_END,
            //set LEDC mode, from ledc_mode_t
            .speed_mode = secondStrip[ch].mode,
            //set LEDC timer source, if different channel use one timer,
            //the frequency and bit_num of these channels should be the same
            .timer_sel = secondStrip[ch].timer_idx,
        };
        //set the configuration
        ledc_channel_config(&ledc_channel2);
        ledc_channel_config_t ledc_channel3 = {
            //set LEDC channel 0
            .channel = thirdStrip[ch].channel,
            //set the duty for initialization.(duty range is 0 ~ ((2**bit_num)-1)
            .duty = 0,
            //GPIO number
            .gpio_num = thirdStrip[ch].io,
            //GPIO INTR TYPE, as an example, we enable fade_end interrupt here.
            .intr_type = LEDC_INTR_FADE_END,
            //set LEDC mode, from ledc_mode_t
            .speed_mode = thirdStrip[ch].mode,
            //set LEDC timer source, if different channel use one timer,
            //the frequency and bit_num of these channels should be the same
            .timer_sel = thirdStrip[ch].timer_idx,
        };
        //set the configuration
        ledc_channel_config(&ledc_channel3);
    }

    //initialize fade service.
    ledc_fade_func_install(0);
    setStrip(firstStrip, 0, 0, 0);
    setStrip(secondStrip, 0, 0, 0);
    setStrip(thirdStrip, 0, 0, 0);
    ESP_LOGI(TAG, "SETUP_PWM_END\n");
}