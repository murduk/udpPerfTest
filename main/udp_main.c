/* udp_perf Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
udp_perf example

Using this example to test udp throughput performance.
esp<->esp or esp<->ap

step1:
    init wifi as AP/STA using config SSID/PASSWORD.

step2:
    create a udp server/client socket using config PORT/(IP).
    if server: wating for the first message of client.
    if client: sending a packet to server first.

step3:
    send/receive data to/from each other.
    you can see the info in serial output.
*/

#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_spiffs.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include "stdio.h"

#include "udp_perf.h"
#include "ledstuff.h"

#include "spiram_fifo.h"
#include "audio_renderer.h"
#include "playerconfig.h"
#include "audio_player.h"

//this task establish a UDP connection and receive data from UDP
static void udp_conn(void *pvParameters)
{
    ESP_LOGI(TAG, "ttask udp_conn start.");
    /*wating for connecting to AP*/
    xEventGroupWaitBits(udp_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "sta has connected to ap.");

    /*create udp socket*/
    int socket_ret;

#if EXAMPLE_ESP_UDP_MODE_SERVER
    ESP_LOGI(TAG, "create udp server after 3s...");
    vTaskDelay(3000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "create_udp_server.");
    socket_ret = create_udp_server();
#else /*EXAMPLE_ESP_UDP_MODE_SERVER*/
    ESP_LOGI(TAG, "create udp client after 20s...");
    vTaskDelay(20000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "create_udp_client.");
    socket_ret = create_udp_client();
#endif
    if (socket_ret == ESP_FAIL)
    {
        ESP_LOGI(TAG, "create udp socket error,stop.");
        vTaskDelete(NULL);
    }

    /*create a task to tx/rx data*/
    TaskHandle_t tx_rx_task;
    xTaskCreate(&send_recv_data, "send_recv_data", 6144, NULL, 4, &tx_rx_task);

    /*waiting udp connected success*/
    xEventGroupWaitBits(udp_event_group, UDP_CONNCETED_SUCCESS, false, true, portMAX_DELAY);
    int bps;
    while (1)
    {
        total_data = 0;
        vTaskDelay(3000 / portTICK_RATE_MS); //every 3s
        bps = total_data / 3;

        if (total_data <= 0)
        {
            int err_ret = check_connected_socket();
            if (err_ret == -1)
            { //-1 reason: low level netif error
                ESP_LOGW(TAG, "udp send & recv stop.\n");
                break;
            }
        }

#if EXAMPLE_ESP_UDP_PERF_TX
        ESP_LOGI(TAG, "udp send %d byte per sec! total pack: %d \n", bps, success_pack);
#else
        //ESP_LOGI(TAG, "udp recv %d byte per sec! total pack: %d \n", bps, success_pack);
#endif /*EXAMPLE_ESP_UDP_PERF_TX*/
    }
    close_socket();
    vTaskDelete(tx_rx_task);
    vTaskDelete(NULL);
}

static renderer_config_t *create_renderer_config()
{
    renderer_config_t *renderer_config = calloc(1, sizeof(renderer_config_t));

    renderer_config->bit_depth = I2S_BITS_PER_SAMPLE_16BIT;
    renderer_config->i2s_num = I2S_NUM_0;
    renderer_config->sample_rate = 44100;
    renderer_config->sample_rate_modifier = 1.0;
    renderer_config->output_mode = AUDIO_OUTPUT_MODE;

    if(renderer_config->output_mode == I2S_MERUS) {
        renderer_config->bit_depth = I2S_BITS_PER_SAMPLE_32BIT;
    }

    if(renderer_config->output_mode == DAC_BUILT_IN) {
        renderer_config->bit_depth = I2S_BITS_PER_SAMPLE_16BIT;
    }

    return renderer_config;
}

player_t *player_config = NULL;

static void setupAudioPlayer()
{
    // init web radio
    player_config = calloc(1, sizeof(player_t));

    // init player config
    
    player_config->command = CMD_NONE;
    player_config->decoder_status = UNINITIALIZED;
    player_config->decoder_command = CMD_NONE;
    player_config->buffer_pref = BUF_PREF_SAFE;
    player_config->media_stream = calloc(1, sizeof(media_stream_t));
    player_config->media_stream->content_type = AUDIO_MPEG;
    player_config->media_stream->eof = false;

    // init renderer
    renderer_init(create_renderer_config());
    audio_player_init(player_config);
    // start radio
    //web_radio_init(radio_config);
    //web_radio_start(radio_config);
}

void initFile()
{
    ESP_LOGI(TAG, "Initializing SPIFFS");
    
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };
    
    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%d)", ret);
        }
        return;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information");
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }    
}

void app_main(void)
{
    setupPWM();
    if (!spiRamFifoInit()) {
        ESP_LOGE(TAG,"SPI RAM chip fail!");
        while(1);
    }
#if EXAMPLE_ESP_WIFI_MODE_AP
    ESP_LOGI(TAG, "EXAMPLE_ESP_WIFI_MODE_AP");
    wifi_init_softap();
#else /*EXAMPLE_ESP_WIFI_MODE_AP*/
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    initFile();    
    setupAudioPlayer();
    wifi_init_sta();
#endif
    xTaskCreate(&udp_conn, "udp_conn", 4096, NULL, 5, NULL);
}
