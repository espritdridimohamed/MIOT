#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_softap.h"
#include "nvs_flash.h"
#include "qrcode.h"
#include "freertos/task.h"
#include "json.hpp"
#include "driver/rmt_rx.h"
#include "driver/rmt_common.h"
#include "hal/gpio_types.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "mqtt_client.h"
#include <cstdint>

#define MAX_RETRY_NUM 5
#define IR_GPIO GPIO_NUM_15
// MQTT broker (PC) URI
#define BROKER_URI "mqtt://192.168.1.17:1883"
static const char* TAG = "main";
static bool ir_started = false;
static void start_ir_task(void);
static rmt_channel_handle_t ir_rx_channel = nullptr;
static volatile bool ir_rx_active = false;
static volatile bool ir_rx_report = false;
static volatile size_t ir_last_symbols = 0;
static int64_t ir_last_log_us = 0;
static uint32_t ir_last_code = 0;
// Sensor config: adjust pins/channels to your wiring
#define DHT_GPIO GPIO_NUM_4
#define LDR_ADC_UNIT ADC_UNIT_1
#define LDR_ADC_CHANNEL ADC_CHANNEL_6 /* GPIO34 */

static void start_sensor_task(void);
static void start_mqtt_client(void);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static inline void mqtt_publish(const char *topic, const char *payload);
static esp_mqtt_client_handle_t mqtt = nullptr;
static volatile bool mqtt_ready = false;

/*
 * within
 * Purpose: Helper for timing tolerance checks.
 */
static inline bool within(int v, int target, int tol) {
    return v >= target - tol && v <= target + tol;
}

/*
 * nec_decode
 * Purpose: Tolerant NEC frame decoder from RMT symbols.
 * Returns: true if a code was decoded (or repeat recognized).
 */
static bool nec_decode(const rmt_symbol_word_t* s, size_t n, uint32_t* out)
{
    if (!s || n < 34) return false;
    // Leader code: ~9ms + ~4.5ms (polarity may be inverted)
    int d0 = s[0].duration0;
    int d1 = s[0].duration1;
    bool has_leader = (within(d0, 9000, 1200) && within(d1, 4500, 800)) ||
                      (within(d0, 4500, 800) && within(d1, 9000, 1200));
    if (!has_leader) {
        // Possible repeat: ~9ms + ~2.25ms
        bool is_repeat = (within(d0, 9000, 1200) && within(d1, 2250, 600)) ||
                         (within(d0, 2250, 600) && within(d1, 9000, 1200));
        if (is_repeat && ir_last_code != 0) {
            if (out) *out = ir_last_code;
            return true; // treat repeat as same code
        }
        return false;
    }
    uint32_t code = 0;
    // Bits start at index 1
    size_t idx = 1;
    for (int i = 0; i < 32 && idx < n; ++i, ++idx) {
        int a = s[idx].duration0;
        int b = s[idx].duration1;
        int longer = (a > b) ? a : b;
        // NEC: 0 -> 560us + 560us, 1 -> 560us + ~1690us
        bool bit1 = within(longer, 1690, 400);
        bool bit0 = within(longer, 560, 250);
        if (!bit1 && !bit0) {
            // If timing is ambiguous, decide by threshold
            bit1 = longer > 1000;
        }
        code = (code << 1) | (bit1 ? 1u : 0u);
    }
    if (out) *out = code;
    ir_last_code = code;
    return true;
}

/*
 * dht22_read
 * Purpose: Read temperature/humidity from DHT22 on a single-wire pin.
 * Returns: true on success; outputs Celsius and RH percent.
 * Note: DHT22 updates about every 2 seconds; avoid polling faster.
 */
static bool dht22_read(gpio_num_t pin, float* out_temp_c, float* out_rh)
{
    if (!out_temp_c || !out_rh) return false;
    // Prepare line (assumes external pull-up)
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(1000);

    // Start signal: pull low >= 1 ms
    gpio_set_level(pin, 0);
    esp_rom_delay_us(1100);
    gpio_set_level(pin, 1);
    // Switch to input to let sensor respond
    gpio_set_direction(pin, GPIO_MODE_INPUT);

    auto wait_level = [&](int level, uint32_t timeout_us) -> bool {
        int64_t t0 = esp_timer_get_time();
        while (gpio_get_level(pin) != level) {
            if ((uint32_t)(esp_timer_get_time() - t0) > timeout_us) return false;
        }
        return true;
    };

    // Sensor response: ~80us low, ~80us high
    if (!wait_level(0, 200)) return false;
    if (!wait_level(1, 200)) return false;
    if (!wait_level(0, 200)) return false; // start of first bit low

    uint8_t data[5] = {0};
    for (int i = 0; i < 40; ++i) {
        // Each bit: ~50us low
        if (!wait_level(1, 120)) return false;
        // Measure high pulse length
        int64_t th = esp_timer_get_time();
        while (gpio_get_level(pin) == 1) {
            if (esp_timer_get_time() - th > 150) break; // cap ~150us
        }
        int high_us = (int)(esp_timer_get_time() - th);
        // DHT22: ~26-28us => 0, ~70us => 1
        uint8_t bit = (high_us > 50) ? 1 : 0;
        data[i/8] = (uint8_t)((data[i/8] << 1) | bit);
        // Wait for next low to begin
        if (!wait_level(0, 120)) return false;
    }

    // Verify checksum
    uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (sum != data[4]) return false;

    // DHT22 format: Humidity (16-bit, 10x), Temperature (16-bit, 10x, sign)
    uint16_t rh10 = ((uint16_t)data[0] << 8) | data[1];
    uint16_t t10  = ((uint16_t)data[2] << 8) | data[3];
    bool neg = (t10 & 0x8000) != 0;
    if (neg) t10 &= 0x7FFF;
    float temp_c = ((int)t10) / 10.0f;
    if (neg) temp_c = -temp_c;
    float rh = ((int)rh10) / 10.0f;

    *out_temp_c = temp_c;
    *out_rh = rh;
    return true;
}
/*
 * print_qr
 * Purpose: Display provisioning QR for SoftAP transport.
 */
void print_qr(void)
{
    nlohmann::json qr_string = 
    {
        {"ver","v1"},
        {"name","PROV_ESP"},
        {"pop","abcd1234"},
        {"transport","softap"}
    };
    esp_qrcode_config_t qr = ESP_QRCODE_CONFIG_DEFAULT();
    esp_qrcode_generate(&qr,qr_string.dump().c_str());
}
/*
 * wifi_prov_handler
 * Purpose: Handle Wi‑Fi provisioning lifecycle events.
 */
static void wifi_prov_handler(void *user_data, wifi_prov_cb_event_t event, void *event_data)
{
    switch(event)
    {
        case WIFI_PROV_START :
        ESP_LOGI(TAG,"[WIFI_PROV_START]");
        break;
        case WIFI_PROV_CRED_RECV :
        ESP_LOGI(TAG ,"cred : ssid : %s pass : %s",((wifi_sta_config_t*)event_data)->ssid,((wifi_sta_config_t*)event_data)->password);
        break;
        case WIFI_PROV_CRED_SUCCESS :
        ESP_LOGI(TAG,"prov success");
        wifi_prov_mgr_stop_provisioning();
        break;
        case WIFI_PROV_CRED_FAIL :
        ESP_LOGE(TAG,"credentials worng");
        wifi_prov_mgr_reset_sm_state_on_failure();
        print_qr();
        break;
        case WIFI_PROV_END: 
        ESP_LOGI(TAG,"prov ended");
        wifi_prov_mgr_deinit();
        break;
        default : break;
    }
}
/*
 * wifi_event_handler
 * Purpose: Manage Wi‑Fi/IP events; start IR task on IP.
 */
static void wifi_event_handler(void* event_handler_arg,
                                        esp_event_base_t event_base,
                                        int32_t event_id,
                                        void* event_data)
{
    static int retry_cnt =0 ;
    if(event_base == WIFI_EVENT)
    {
        switch(event_id)
        {
            case WIFI_EVENT_STA_START :
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED :
                ESP_LOGE(TAG,"ESP32 Disconnected, retrying");
                retry_cnt ++;
                if(retry_cnt < MAX_RETRY_NUM)
                {
                    esp_wifi_connect();
                }
                else ESP_LOGE(TAG,"Connection error");
                break;
            default : break;
        }
    }
    else if(event_base == IP_EVENT)
    {
        if(event_id == IP_EVENT_STA_GOT_IP)
        {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
            ESP_LOGI(TAG, "station ip : " IPSTR, IP2STR(&event->ip_info.ip));
            start_mqtt_client();
            start_ir_task();
            start_sensor_task();
        }
    }
}
/*
 * wifi_hw_init
 * Purpose: Initialize NVS, netif, event loop, handlers, and Wi‑Fi driver.
 */
void wifi_hw_init(void)
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&wifi_event_handler,NULL,&instance_any_id);
    esp_event_handler_instance_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&wifi_event_handler,NULL,&instance_got_ip);
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_start();
 

}
/*
 * on_ir_rx_done
 * Purpose: RMT RX ISR callback – signal task to report.
 */
static bool on_ir_rx_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    // ISR context: don't log; signal task to report
    ir_rx_active = false;
    ir_rx_report = true;
    ir_last_symbols = edata ? edata->num_symbols : 0;
    return true;
}

/*
 * ir_task
 * Purpose: Configure RMT RX channel and continuously capture IR symbols.
 */
static void ir_task(void *arg)
{
    rmt_rx_channel_config_t rx_cfg = {};
    rx_cfg.gpio_num = IR_GPIO;
    rx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_cfg.resolution_hz = 1000000;
    rx_cfg.mem_block_symbols = 64;
    rmt_new_rx_channel(&rx_cfg, &ir_rx_channel);

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = on_ir_rx_done;
    rmt_rx_register_event_callbacks(ir_rx_channel, &cbs, nullptr);

    rmt_receive_config_t recv_cfg = {};
    recv_cfg.signal_range_min_ns = 1000;
    recv_cfg.signal_range_max_ns = 12000000;

    rmt_symbol_word_t symbols[64];
    ir_rx_active = false;
    ESP_ERROR_CHECK(rmt_enable(ir_rx_channel));

    while (true) {
        if (!ir_rx_active) {
            ir_rx_active = true;
            esp_err_t err = rmt_receive(ir_rx_channel, symbols, sizeof(symbols), &recv_cfg);
            if (err != ESP_OK) {
                ir_rx_active = false;
            }
        }
        if (ir_rx_report) {
            ir_rx_report = false;
            int64_t now = esp_timer_get_time();
            // Debounce: log only if enough time passed or enough symbols captured
            const int64_t min_gap_us = 1500000; // 1500 ms gap for repeat suppression
            const size_t min_symbols_threshold = 20; // ignore very short glitches
            if ((ir_last_log_us == 0) || (now - ir_last_log_us >= min_gap_us) || (ir_last_symbols >= min_symbols_threshold)) {
                ir_last_log_us = now;
                uint32_t code = 0;
                if (nec_decode(symbols, ir_last_symbols, &code)) {
                    ESP_LOGI(TAG, "IR code: 0x%08lX", code);
                    char msg[16];
                    snprintf(msg, sizeof(msg), "0x%08lX", code);
                    mqtt_publish("miot/ir/code", msg);
                } else {
                    ESP_LOGI(TAG, "IR symbols captured");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*
 * start_ir_task
 * Purpose: Launch IR task once after IP acquisition.
 */
static void start_ir_task(void)
{
    if (!ir_started) {
        xTaskCreate(ir_task, "ir_task", 4096, NULL, 5, NULL);
        ir_started = true;
        ESP_LOGI(TAG, "IR task started");
    }
}
/*
 * sensor_task
 * Purpose: Initialize ADC one-shot (for LDR) and read DHT22.
 * Prints values every second. Respects DHT22's ~2s update by caching.
 */
static void sensor_task(void *arg)
{
    // ADC one-shot setup (LDR on ADC1 CH6 / GPIO34)
    adc_oneshot_unit_handle_t adc = nullptr;
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = LDR_ADC_UNIT;
    adc_oneshot_new_unit(&unit_cfg, &adc);
    adc_oneshot_chan_cfg_t ch_cfg = {};
    ch_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    ch_cfg.atten = ADC_ATTEN_DB_12;
    adc_oneshot_config_channel(adc, LDR_ADC_CHANNEL, &ch_cfg);

    // DHT22 setup: ensure line released high by external pull-up
    gpio_reset_pin(DHT_GPIO);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(DHT_GPIO, 1);

    int last_raw = 0;
    float last_temp = 0.0f, last_rh = 0.0f;
    int64_t last_dht_read = 0;

    while (true) {
        // LDR read
        int raw = 0;
        if (adc_oneshot_read(adc, LDR_ADC_CHANNEL, &raw) == ESP_OK) {
            last_raw = raw;
        }

        // DHT22 read (not more often than ~2s)
        int64_t now = esp_timer_get_time();
        if (now - last_dht_read >= 2000000) {
            float t = 0, h = 0;
            if (dht22_read(DHT_GPIO, &t, &h)) {
                last_temp = t;
                last_rh = h;
                last_dht_read = now;
            }
        }

        ESP_LOGI(TAG, "LDR raw=%d | DHT22 T=%.1fC H=%.1f%%", last_raw, last_temp, last_rh);
        // Publish to MQTT topics if connected
        if (mqtt_ready) {
            char buf[32];
            snprintf(buf, sizeof(buf), "%d", last_raw);
            mqtt_publish("miot/ldr/raw", buf);
            snprintf(buf, sizeof(buf), "%.1f", last_temp);
            mqtt_publish("miot/dht22/temperature", buf);
            snprintf(buf, sizeof(buf), "%.1f", last_rh);
            mqtt_publish("miot/dht22/humidity", buf);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*
 * start_sensor_task
 * Purpose: Launch periodic sensor sampling task once.
 */
static void start_sensor_task(void)
{
    static bool started = false;
    if (!started) {
        xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
        started = true;
        ESP_LOGI(TAG, "Sensor task started");
    }
}

/*
 * start_mqtt_client
 * Purpose: Initialize and start MQTT client to the broker URI.
 */
static void start_mqtt_client(void)
{
    static bool started = false;
    if (started) return;
    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri = BROKER_URI;
    mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(mqtt, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt);
    started = true;
}

/*
 * mqtt_event_handler
 * Purpose: Track MQTT connection status.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_ready = true;
            ESP_LOGI(TAG, "MQTT connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_ready = false;
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
        default:
            break;
    }
}

/*
 * mqtt_publish
 * Purpose: Publish a payload to a topic if connected.
 */
static inline void mqtt_publish(const char *topic, const char *payload)
{
    if (mqtt && mqtt_ready && topic && payload) {
        esp_mqtt_client_publish(mqtt, topic, payload, 0, 1, 0);
    }
}
/*
 * prov_start
 * Purpose: Start provisioning or STA mode depending on NVS state.
 */
static void prov_start(void)
{
    wifi_prov_mgr_config_t cfg = 
    {
        .scheme =wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
        .app_event_handler = wifi_prov_handler,
        
      
    };
    wifi_prov_mgr_init(cfg);
    bool is_provisioned = 0;
    wifi_prov_mgr_is_provisioned(&is_provisioned);
    wifi_prov_mgr_disable_auto_stop(100);
    if(is_provisioned)
    {
        ESP_LOGI(TAG,"Already provisioned");
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
    }
    else
    {
        wifi_prov_mgr_start_provisioning(WIFI_PROV_SECURITY_1,"abcd1234","PROV_ESP",NULL);
        print_qr();

    }

}
/*
 * app_main
 * Purpose: Entry point – bring up Wi‑Fi and provisioning.
 */
extern "C" void app_main(void)
{
    wifi_hw_init();
    prov_start();
}