/* Ledc fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <Math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "ledstuff.h"

#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#define TIMER_INTR_SEL TIMER_INTR_LEVEL                                   /*!< Timer level interrupt */
#define TIMER_GROUP TIMER_GROUP_0                                         /*!< Test on timer group 0 */
#define TIMER_DIVIDER 16                                                  /*!< Hardware timer clock divider */
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)                      /*!< used to calculate counter value */
#define TIMER_FINE_ADJ (1.4 * (TIMER_BASE_CLK / TIMER_DIVIDER) / 1000000) /*!< used to compensate alarm value */
//#define TIMER_INTERVAL0_SEC (3.4179)                                      /*!< test interval for timer 0 */
#define TIMER_INTERVAL1_SEC (0.1) /*!< test interval for timer 1 */
#define TEST_WITHOUT_RELOAD 0     /*!< example of auto-reload mode */
#define TEST_WITH_RELOAD 1        /*!< example without auto-reload mode */

typedef struct
{
    int type;             /*!< event type */
    int group;            /*!< timer group */
    int idx;              /*!< timer number */
    uint64_t counter_val; /*!< timer counter value */
} timer_event_t;

xQueueHandle timer_queue;

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

#define TAG "ledstuff"

//static int count = 0;
static char secondCount = 0;

/*
 * @brief timer group0 ISR handler
 */
void IRAM_ATTR timer_group0_isr(void *para)
{
    secondCount++;
    int timer_idx = (int)para;
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    //timer_event_t evt;    
    if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1)
    {
        /*Timer1 is an example that will reload counter value*/
        TIMERG0.hw_timer[timer_idx].update = 1;
        /*We don't call a API here because they are not declared with IRAM_ATTR*/
        TIMERG0.int_clr_timers.t1 = 1;
        uint64_t timer_val = ((uint64_t)TIMERG0.hw_timer[timer_idx].cnt_high) << 32 | TIMERG0.hw_timer[timer_idx].cnt_low;
        /*Post an event to out example task*/
        //evt.type = TEST_WITH_RELOAD;
        //evt.group = 0;
        //evt.idx = timer_idx;
        //evt.counter_val = timer_val;
        xQueueSendFromISR(timer_queue, &secondCount, NULL);
        /*For a auto-reload timer, we still need to set alarm_en bit if we want to enable alarm again.*/
        TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;
    }
}

static void example_tg0_timer1_init()
{
    int timer_group = TIMER_GROUP_0;
    int timer_idx = TIMER_1;
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, TIMER_INTERVAL1_SEC * TIMER_SCALE);
    /*Enable timer interrupt*/
    timer_enable_intr(timer_group, timer_idx);
    /*Set ISR handler*/
    timer_isr_register(timer_group, timer_idx, timer_group0_isr, (void *)timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    /*Start timer counter*/
    //timer_start(timer_group, timer_idx);
}

static void pauseTimer()
{
    int timer_group = TIMER_GROUP_0;
    int timer_idx = TIMER_1;
    timer_pause(timer_group, timer_idx);
}

static void startTimer()
{
    int timer_group = TIMER_GROUP_0;
    int timer_idx = TIMER_1;
    timer_start(timer_group, timer_idx);
}

void setStrip(ledc_info_t strip[], unsigned char red, unsigned char green, unsigned char blue)
{
    //temp
    ledc_set_duty(strip[0].mode, strip[0].channel, floor((red / 255.0) * 8191));
    ledc_update_duty(strip[0].mode, strip[0].channel);
    ledc_set_duty(strip[1].mode, strip[1].channel, floor((green / 255.0) * 8191));
    ledc_update_duty(strip[1].mode, strip[1].channel);
    ledc_set_duty(strip[2].mode, strip[2].channel, floor((blue / 255.0) * 8191));
    ledc_update_duty(strip[2].mode, strip[2].channel);
}

void setFadeStrip(ledc_info_t strip[], unsigned char red, unsigned char green, unsigned char blue, int delay)
{
    ledc_set_fade_with_time(strip[0].mode, strip[0].channel, floor((red / 255.0) * 8191), delay);
    ledc_fade_start(strip[0].mode, strip[0].channel, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(strip[1].mode, strip[1].channel, floor((green / 255.0) * 8191), delay);
    ledc_fade_start(strip[1].mode, strip[1].channel, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(strip[2].mode, strip[2].channel, floor((blue / 255.0) * 8191), delay);
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
                //ESP_LOGI(TAG, "MS_MATCH:%d %d %d %d\n", actions[j].ms, i, actions[j].strip, j);
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

void testActions2(stripAction actions[], int size)
{
    ESP_LOGI(TAG, "BEGIN\n");
    int i = 0;
    secondCount = 0;
    if (timer_queue == NULL)
    {
        timer_queue = xQueueCreate(10, sizeof(char));        
        example_tg0_timer1_init();
    }
    xQueueReset(timer_queue);    
    startTimer();
    char evt = 0;
    while (evt < 60)
    {        
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
        //i++;
        ESP_LOGI(TAG, "QUEUE:%d %d", evt, uxQueueMessagesWaiting(timer_queue));        
        for (int j = 0; j < size; j++)
        {
            if (actions[j].ms == evt)
            {
                ESP_LOGI(TAG, "MS_MATCH:%d %d %d %d\n", actions[j].ms, evt, actions[j].strip, j);
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
    }
    pauseTimer();
    xQueueReset(timer_queue);    
    //timer_queue = NULL;
    setStrip(firstStrip, 0, 0, 0);
    setStrip(secondStrip, 0, 0, 0);
    setStrip(thirdStrip, 0, 0, 0);
    ESP_LOGI(TAG, "END:%d", uxQueueMessagesWaiting(timer_queue));
}

void setupPWM()
{
    ESP_LOGI(TAG, "SETUP_PWM_BEGIN\n");
    int ch;

    ledc_info_t firstStripTmp[3] = {
        {.channel = LEDC_CHANNEL_0, .io = (2), .mode = LEDC_HS_MODE, .timer_idx = LEDC_HS_TIMER},
        {.channel = LEDC_CHANNEL_1, .io = (4), .mode = LEDC_HS_MODE, .timer_idx = LEDC_HS_TIMER},
        {.channel = LEDC_CHANNEL_2, .io = (15), .mode = LEDC_HS_MODE, .timer_idx = LEDC_HS_TIMER}};

    ledc_info_t secondStripTmp[3] = {
        {.channel = LEDC_CHANNEL_3, .io = (18), .mode = LEDC_HS_MODE, .timer_idx = LEDC_HS_TIMER},
        {.channel = LEDC_CHANNEL_4, .io = (19), .mode = LEDC_HS_MODE, .timer_idx = LEDC_HS_TIMER},
        {.channel = LEDC_CHANNEL_5, .io = (5), .mode = LEDC_HS_MODE, .timer_idx = LEDC_HS_TIMER}};

    ledc_info_t thirdStripTmp[3] = {
        {.channel = LEDC_CHANNEL_6, .io = (22), .mode = LEDC_HS_MODE, .timer_idx = LEDC_HS_TIMER},
        {.channel = LEDC_CHANNEL_0, .io = (23), .mode = LEDC_LS_MODE, .timer_idx = LEDC_LS_TIMER},
        {.channel = LEDC_CHANNEL_1, .io = (21), .mode = LEDC_LS_MODE, .timer_idx = LEDC_LS_TIMER}};

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