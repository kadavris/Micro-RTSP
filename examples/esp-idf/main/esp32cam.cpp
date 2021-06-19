#include "CRtspSession.h"
#include "OV2640Streamer.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

//Run `idf.py menuconfig` and set various other options ing "ESP32CAM Configuration"

//common camera settings, see below for other camera options
#define CAM_FRAMESIZE FRAMESIZE_VGA //enough resolution for a simple security cam
#define CAM_QUALITY 12 //0 to 63, with higher being lower-quality (and less bandwidth), 12 seems reasonable

//MQTT settings
#define MQTT_SERVER "192.168.1.163"   //DNS resolving on EPS32 might be broken "homeassistant.localdomain"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt"
#define MQTT_PASS "mqtt"
#define MQTT_CLIENT_NAME CONFIG_LWIP_LOCAL_HOSTNAME //menuconfig->Component config->LWIP->Local netif hostname
#define MQTT_LED_TOPIC "devices/" MQTT_CLIENT_NAME "/set/led"
#define MQTT_WILL_TOPIC "devices/" MQTT_CLIENT_NAME "/connected"
#define MQTT_WILL_MSG "false"
#define MQTT_LED_STATE_TOPIC "devices/" MQTT_CLIENT_NAME "/state/led"
#define MQTT_OTA_TOPIC "devices/" MQTT_CLIENT_NAME "/ota/url"
#define MQTT_OTA_STATUS_TOPIC "devices/" MQTT_CLIENT_NAME "/ota/status"

OV2640 cam;
esp_mqtt_client_handle_t mqtt_client;

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

//lifted from Arduino framework
unsigned long millis()
{
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

void client_worker(void * client)
{
    OV2640Streamer * streamer = new OV2640Streamer((SOCKET)client, cam);
    CRtspSession * session = new CRtspSession((SOCKET)client, streamer);

    unsigned long lastFrameTime = 0;
    const unsigned long msecPerFrame = (1000 / CONFIG_CAM_FRAMERATE);

    while (session->m_stopped == false)
    {
        session->handleRequests(0);

        unsigned long now = millis();
        if ((now > (lastFrameTime + msecPerFrame)) || (now < lastFrameTime))
        {
            session->broadcastCurrentFrame(now);
            lastFrameTime = now;
        }
        else
        {
            //let the system do something else for a bit
            vTaskDelay(1);
        }
    }

    //shut ourselves down
    delete streamer;
    delete session;
    vTaskDelete(NULL);
}

extern "C" void rtsp_server(void);
void rtsp_server(void)
{
    SOCKET server_socket;
    SOCKET client_socket;
    sockaddr_in server_addr;
    sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    TaskHandle_t xHandle = NULL;

    camera_config_t config = esp32cam_aithinker_config;
    config.frame_size = CAM_FRAMESIZE;
    config.jpeg_quality = CAM_QUALITY; 
    config.xclk_freq_hz = 16500000; //seems to increase stability compared to the full 20000000
    cam.init(config);

    //setup other camera options
    sensor_t * s = esp_camera_sensor_get();
    //s->set_brightness(s, 0);     // -2 to 2
    //s->set_contrast(s, 0);       // -2 to 2
    //s->set_saturation(s, 0);     // -2 to 2
    //s->set_special_effect(s, 2); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 0);       // 0 = disable , 1 = enable //auto white-balance seems to be more trouble than its worth
    //s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    //s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    //s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    //s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    //s->set_ae_level(s, 0);       // -2 to 2
    //s->set_aec_value(s, 300);    // 0 to 1200
    //s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    //s->set_agc_gain(s, 0);       // 0 to 30
    //s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    //s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    //s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    //s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    //s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, CONFIG_CAM_HORIZONTAL_MIRROR); // 0 = disable , 1 = enable
    s->set_vflip(s, CONFIG_CAM_VERTICAL_FLIP);       // 0 = disable , 1 = enable
    //s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    //s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

    server_addr.sin_family      = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port        = htons(8554); // listen on RTSP port 8554
    server_socket               = socket(AF_INET, SOCK_STREAM, 0);

    int enable = 1;
    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
    {
        printf("setsockopt(SO_REUSEADDR) failed! errno=%d\n", errno);
    }

    // bind our server socket to the RTSP port and listen for a client connection
    if (bind(server_socket, (sockaddr*)&server_addr, sizeof(server_addr)) != 0)
    {
        printf("bind failed! errno=%d\n", errno);
    }

    if (listen(server_socket, 5) != 0)
    {
        printf("listen failed! errno=%d\n", errno);
    }

    printf("\n\nrtsp://%s.localdomain:8554/mjpeg/1\n\n", CONFIG_LWIP_LOCAL_HOSTNAME);

    // loop forever to accept client connections
    while (true)
    {
        printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
        client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_addr_len);

        printf("Client connected from: %s\n", inet_ntoa(client_addr.sin_addr));

        xTaskCreate(client_worker, "client_worker", 3584, (void*)client_socket, tskIDLE_PRIORITY, &xHandle);
    }

    close(server_socket);
}

//quick and dirty hack to convert a raw buffer into a C-style null-terminated string
void null_terminate_string(char * data, int data_len, char * normal_string, int normal_string_len)
{
    memset(normal_string, 0, normal_string_len);

    if (data_len > (normal_string_len - 1))
    {
        data_len = normal_string_len - 1;
    }

    memcpy(normal_string, data, data_len);
}

void simple_ota_update(char * url)
{
    ESP_LOGI("OTA", "Starting OTA update...");
    esp_http_client_config_t config;
    memset(&config, 0, sizeof(esp_http_client_config_t));
    config.url = url;
    config.cert_pem = (char *)server_cert_pem_start; //TODO https not working?
    config.keep_alive_enable = true;
    config.skip_cert_common_name_check = true;

    esp_err_t ret = esp_https_ota(&config);
    if (ret != ESP_OK)
    {
        ESP_LOGE("OTA", "Firmware upgrade failed!");
    }

    char status[32];
    printf("OTA complete with return code: %d\n", ret);
    snprintf(status, sizeof(status), "%d", ret);
    esp_mqtt_client_publish(mqtt_client, MQTT_OTA_STATUS_TOPIC, status, 0, 0, 0);
    delay(5000);
    esp_restart();
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    char normal_string[128];
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI("MQTT", "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_publish(client, MQTT_WILL_TOPIC, "true", 0, 1, true);
        esp_mqtt_client_subscribe(client, MQTT_LED_TOPIC, 0);
        esp_mqtt_client_subscribe(client, MQTT_OTA_TOPIC, 0);
        esp_mqtt_client_publish(client, MQTT_LED_STATE_TOPIC, "0", 0, 0, 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI("MQTT", "MQTT_EVENT_DISCONNECTED");
        esp_mqtt_client_reconnect(client);
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI("MQTT", "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI("MQTT", "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI("MQTT", "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI("MQTT", "MQTT_EVENT_DATA");
        //event topic and data are just raw data, not null-terminiated strings
        if (strncmp(MQTT_LED_TOPIC, event->topic, event->topic_len) == 0)
        {
            null_terminate_string(event->data, event->data_len, normal_string, sizeof(normal_string));
            int duty_cycle = atoi(normal_string);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_cycle);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            esp_mqtt_client_publish(client, MQTT_LED_STATE_TOPIC, normal_string, 0, 0, 0);
        }
        else if (strncmp(MQTT_OTA_TOPIC, event->topic, event->topic_len) == 0)
        {
            null_terminate_string(event->data, event->data_len, normal_string, sizeof(normal_string));
            simple_ota_update(normal_string);
        }
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI("MQTT", "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGI("MQTT", "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI("MQTT", "Other event id:%d", event->event_id);
        break;
    }
}

extern "C" void mqtt_app_start(void);
void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg;
    memset(&mqtt_cfg, 0, sizeof(esp_mqtt_client_config_t));
    mqtt_cfg.host = MQTT_SERVER;
    mqtt_cfg.port = MQTT_PORT;
    mqtt_cfg.username = MQTT_USER;
    mqtt_cfg.password = MQTT_PASS;
    mqtt_cfg.client_id = MQTT_CLIENT_NAME;
    mqtt_cfg.lwt_topic = MQTT_WILL_TOPIC;
    mqtt_cfg.lwt_msg = MQTT_WILL_MSG;
    mqtt_cfg.lwt_qos = 1;
    mqtt_cfg.lwt_retain = true;

    //setup camera LED
    ledc_timer_config_t ledc_timer =
    {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel =
    {
        .gpio_num   = 4, //specific to the ESP32-CAM
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 0,
        .hpoint     = 0,
        .flags      = { .output_invert = 0 }
    };
    ledc_channel_config(&ledc_channel);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}
