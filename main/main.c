/*
 * qqWater - Controle de válvulas e bombas do poço via ESP32
 *
 * - ESP32 cria uma rede Wi-Fi própria (Access Point)
 * - Servidor web com 8 botões (6 válvulas + 2 bombas)
 * - Módulo de relé optoacoplado, ativo em LOW
 *
 * Pinos usados (nesta ordem = canal 0 a 7):
 *   GPIO13, GPIO12, GPIO14, GPIO27, GPIO26, GPIO25, GPIO33, GPIO32
 *
 * ATENÇÃO DE HARDWARE:
 *   GPIO12 é um "strapping pin" do ESP32 (define a tensão da flash no boot).
 *   Se o módulo de relé mantiver esse pino em HIGH durante o power-on/reset,
 *   o ESP32 pode falhar o boot. Isso é um problema elétrico, não de firmware.
 *   Se notar instabilidade, troque essa ligação para outro GPIO livre.
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "driver/gpio.h"

static const char *TAG = "qqwater";

// ---------------------------------------------------------------------
// Configuração de rede (Access Point)
// ---------------------------------------------------------------------
#define AP_SSID       "QQWATER"
#define AP_PASS       "pocoagua123"   // mínimo 8 caracteres
#define AP_CHANNEL    1
#define AP_MAX_CONN   4

// ---------------------------------------------------------------------
// Configuração dos relés
// ---------------------------------------------------------------------
#define NUM_RELAYS 8
#define RELAY_ACTIVE_LEVEL 1   // ativo em HIGH
#define RELAY_INACTIVE_LEVEL 0

static const gpio_num_t relay_pins[NUM_RELAYS] = {
    GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27,
    GPIO_NUM_5, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_4
};

// Ajuste os rótulos conforme a função real de cada canal no seu painel.
// Assumindo os 6 primeiros como válvulas e os 2 últimos como bombas.
static const char *relay_labels[NUM_RELAYS] = {
    "Valvula 1", "Valvula 2", "Valvula 3", "Valvula 4",
    "Valvula 5", "Valvula 6", "Bomba 1",   "Bomba 2"
};

// Estado lógico (true = ligado), independente do nível elétrico real
static bool relay_state[NUM_RELAYS] = {false};

// ---------------------------------------------------------------------
// Controle dos relés
// ---------------------------------------------------------------------
static void relay_write(int ch, bool on)
{
    if (ch < 0 || ch >= NUM_RELAYS) return;
    gpio_set_level(relay_pins[ch], on ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
    relay_state[ch] = on;
    ESP_LOGI(TAG, "Rele %d (%s) -> %s", ch, relay_labels[ch], on ? "LIGADO" : "DESLIGADO");
}

static void relays_init(void)
{
    for (int i = 0; i < NUM_RELAYS; i++) {
        gpio_reset_pin(relay_pins[i]);
        gpio_set_direction(relay_pins[i], GPIO_MODE_OUTPUT);
        // Garante que todo relé começa DESLIGADO antes de qualquer outra coisa
        gpio_set_level(relay_pins[i], RELAY_INACTIVE_LEVEL);
        relay_state[i] = false;
    }
    ESP_LOGI(TAG, "Todos os reles inicializados em estado DESLIGADO");
}

// ---------------------------------------------------------------------
// Página HTML (embutida no firmware)
// ---------------------------------------------------------------------
static const char index_html[] =
"<!DOCTYPE html><html lang='pt-br'><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<title>qqWater - Controle do Poco</title>"
"<style>"
"body{font-family:Arial,sans-serif;background:#0f1720;color:#e8eef5;margin:0;padding:16px;}"
"h1{font-size:20px;text-align:center;margin-bottom:20px;}"
".grid{display:grid;grid-template-columns:repeat(2,1fr);gap:14px;max-width:420px;margin:0 auto;}"
"button{padding:22px 8px;font-size:16px;border:none;border-radius:12px;background:#26313f;color:#e8eef5;"
"cursor:pointer;transition:background .15s;}"
"button.on{background:#1f9d55;color:#fff;}"
"button:active{opacity:.8;}"
".status{text-align:center;margin-top:18px;font-size:12px;color:#8ea0b3;}"
"</style></head><body>"
"<h1>Controle do Poco - qqWater</h1>"
"<div class='grid' id='grid'></div>"
"<div class='status' id='status'>conectando...</div>"
"<script>"
"const N=8;"
"const grid=document.getElementById('grid');"
"const st=document.getElementById('status');"
"let buttons=[];"
"for(let i=0;i<N;i++){"
"  const b=document.createElement('button');"
"  b.textContent='Canal '+(i+1);"
"  b.onclick=()=>toggle(i);"
"  grid.appendChild(b);"
"  buttons.push(b);"
"}"
"async function toggle(ch){"
"  const cur=buttons[ch].classList.contains('on');"
"  await fetch('/api/relay?ch='+ch+'&state='+(cur?0:1));"
"  refresh();"
"}"
"async function refresh(){"
"  try{"
"    const r=await fetch('/api/status');"
"    const d=await r.json();"
"    for(let i=0;i<N;i++){"
"      buttons[i].textContent=d.labels[i]+(d.state[i]?' - ON':' - OFF');"
"      buttons[i].classList.toggle('on',!!d.state[i]);"
"    }"
"    st.textContent='conectado';"
"  }catch(e){ st.textContent='sem conexao com o ESP32'; }"
"}"
"refresh();"
"setInterval(refresh,2000);"
"</script></body></html>";

// ---------------------------------------------------------------------
// Handlers HTTP
// ---------------------------------------------------------------------
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    char buf[512];
    int len = snprintf(buf, sizeof(buf), "{\"labels\":[");
    for (int i = 0; i < NUM_RELAYS; i++) {
        len += snprintf(buf + len, sizeof(buf) - len, "\"%s\"%s",
                         relay_labels[i], (i < NUM_RELAYS - 1) ? "," : "");
    }
    len += snprintf(buf + len, sizeof(buf) - len, "],\"state\":[");
    for (int i = 0; i < NUM_RELAYS; i++) {
        len += snprintf(buf + len, sizeof(buf) - len, "%d%s",
                         relay_state[i] ? 1 : 0, (i < NUM_RELAYS - 1) ? "," : "");
    }
    len += snprintf(buf + len, sizeof(buf) - len, "]}");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, len);
}

static esp_err_t relay_get_handler(httpd_req_t *req)
{
    char query[64];
    int ch = -1, state = -1;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[8];
        if (httpd_query_key_value(query, "ch", val, sizeof(val)) == ESP_OK) {
            ch = atoi(val);
        }
        if (httpd_query_key_value(query, "state", val, sizeof(val)) == ESP_OK) {
            state = atoi(val);
        }
    }

    if (ch < 0 || ch >= NUM_RELAYS || (state != 0 && state != 1)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "parametros invalidos", HTTPD_RESP_USE_STRLEN);
    }

    relay_write(ch, state == 1);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t status_uri = { .uri = "/api/status", .method = HTTP_GET, .handler = status_get_handler };
        httpd_uri_t relay_uri = { .uri = "/api/relay", .method = HTTP_GET, .handler = relay_get_handler };

        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &relay_uri);
    } else {
        ESP_LOGE(TAG, "Falha ao iniciar o servidor web");
    }
    return server;
}

// ---------------------------------------------------------------------
// Wi-Fi Access Point
// ---------------------------------------------------------------------
static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = AP_CHANNEL,
            .password = AP_PASS,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    if (strlen(AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Access Point iniciado. SSID:%s senha:%s canal:%d",
             AP_SSID, AP_PASS, AP_CHANNEL);
}

// ---------------------------------------------------------------------
// main
// ---------------------------------------------------------------------
void app_main(void)
{
    // NVS é exigido pelo driver Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Primeiro os relés (todos desligados), depois a rede
    relays_init();
    wifi_init_softap();
    start_webserver();

    ESP_LOGI(TAG, "qqWater pronto. Conecte na rede '%s' e acesse http://192.168.4.1", AP_SSID);
}