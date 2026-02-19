#include <stdio.h>  //za printf
#include <string.h>
#include "driver/ledc.h"    //za generisanje PWM signala
#include "freertos/FreeRTOS.h"  //za vTaskDelay
#include "freertos/task.h"  //za pravljenje taskova
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "nvs_flash.h"
#include "mpu.h"

//PODESAVANJA MOTORI
#define ESC_GPIO_S1 1  //GPIO1 za motor 1
#define ESC_GPIO_S2 2  //GPIO2 za motor 2
#define ESC_GPIO_S3 3  //GPIO3 za motor 3
#define ESC_GPIO_S4 4  //GPIO4 za motor 4
#define ESC_FREQ 50  //50Hz je univerzalna frekvencija koju vecina ESC prepoznaje
#define ESC_RESOLUTION LEDC_TIMER_13_BIT    //rezolucija u 13 bita (korak promene brzine 0,012%)

//PODESAVANJA WIFI-ja
#define ESP_WIFI_SSID "DRON-magenta"
#define ESP_WIFI_PASS "12345678"
#define PORT 3333

volatile int throttle = 1000;
volatile long last_packet_time = 0;
int i;

static const char *TAG = "DRONE_WIFI";

//MOTORI
void init_motors()
{
    //TIMER za sve motore
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = ESC_RESOLUTION,
        .freq_hz = ESC_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    int pins[4] = {ESC_GPIO_S1, ESC_GPIO_S2, ESC_GPIO_S3, ESC_GPIO_S4};
    for(i = 0; i < 4; i++)
    {
        ledc_channel_config_t channel = {
        .channel = i,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = calculate_duty(1000),       //pocetno stanje 1000us(0%)
        .gpio_num = pins[i],
        .intr_type = LEDC_INTR_DISABLE,
        .hpoint = 0
        };
        ledc_channel_config(&channel);
    }
    printf("Motori inicijalizovani.\n");
}

//WIFI
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if(event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Laptop povezan! MAC: "MACSTR", AID = %d", MAC2STR(event->mac), event->aid);
    }else if(event_id == WIFI_EVENT_AP_STADISCONNECTED){
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Laptop odspojen! MAC: "MACSTR", AID=%d", MAC2STR(event->mac), event->aid);
        if(throttle > 1000) //Hitno gasenje ako se prekine wifi konekcija (FAIL-SAFE)
        {
            throttle -= 20;  //Postepeno postavi throttle na minimum
            if(throttle < 1000)
                throttle = 1000;
        }
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = 1,
            .password = ESP_WIFI_PASS,
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WIFI Access Point spreman. SSID: %s | password: %s", ESP_WIFI_SSID, ESP_WIFI_PASS);
}

//UDP TASK
static void udp_server_task(void *pvParameter)
{
    char command_buffer[128];
    struct sockaddr_in recv_addr;

    recv_addr.sin_addr.s_addr = INADDR_ANY;
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0)
    {
        ESP_LOGE(TAG, "Nije moguce kreirati socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    if(bind(sock, (struct sockaddr *)&recv_addr, sizeof(recv_addr)) < 0)
    {
        ESP_LOGE(TAG, "Socket bind failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP server slusa na portu %d...", PORT);

    while(1)
    {
        struct sockaddr_storage source_addr;
        socklen_t socklen = sizeof(source_addr);

        memset(command_buffer, 0, sizeof(command_buffer));
        int len = recvfrom(sock, command_buffer, sizeof(command_buffer)-1, 0, (struct sockaddr *)&source_addr, &socklen);

        if(len > 0)
        {
            command_buffer[len] = 0;
            int val = atoi(command_buffer);

            if(val >= 1000 && val <= 2000)
            {
                throttle = val;
                last_packet_time = xTaskGetTickCount();
                ESP_LOGI(TAG, "Primljeno: %d", val);
            }
        }
    }
}

void app_main(void)
{
    //Inicijalizacija NVS
    esp_err_t ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //Inicijalizacija motora
    init_motors();

    //Inicijalizacija WIFI-ja
    wifi_init_softap();

    //Inicijalizacija I2C protokola
    i2c_init();

    //Inicijalizacija MPU6050
    mpu6050_init();

    // Task 1: Slusa UDP pakete i azurira throttle promenljivu
    xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);
    //xTaskCreate(motor_control_task, "motor_control", 2048, NULL, 5, NULL);

    /* Task 2: Cita senzor, racuna PID, kontrolise motore sa stabilizacijom
       Ovaj task radi na 250Hz (svaka 4ms) i obavlja sve:
        - Citanje MPU6050 podataka
        - Racunanje roll i pitch uglova
        - PID kontrola za stabilizaciju
        - Distribuiranje snage na motore
        - Fail-safe provera
    */
    xTaskCreate(flight_control_task, "flight_control", 4096, NULL, 5, NULL);
}
