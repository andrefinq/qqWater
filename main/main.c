/*
 * qqWater - Controle de valvulas e bombas do poco via ESP32
 *
 * - Conecta no roteador cadastrado pela pagina; se nao conseguir, abre a
 *   rede propria (Access Point "QQWATER") e tenta o roteador a cada 10 min
 * - Captive portal no AP: DNS + redirecionamentos abrem a pagina sozinhos
 *   ao conectar (igual wifi de aeroporto)
 * - Login (usuario/senha) antes de acessar a pagina e a API
 * - Dois sensores de fluxo (poco e agua tratada): vazao e totalizador
 * - Servidor web com 8 boteos manuais (bloqueados durante automacao)
 * - Toggle "Modo Automatico" + botao "Parar Sistema"
 * - Modulo de rele optoacoplado, ativo em HIGH
 * - Leitura direta das 4 boias (nao usa mais o mux CD74HC4067)
 *
 * Reles (nesta ordem = canal 0 a 7):
 *   GPIO13(V1) GPIO12(V2) GPIO14(V3) GPIO27(V4)
 *   GPIO15(V5) GPIO2(V6)  GPIO4(B1)  GPIO16(B2)
 *
 * Boias (T1 alto, T1 baixo, T2 alto, T2 baixo, comum):
 *   GPIO22, GPIO23, GPIO18, GPIO19, GPIO21
 *
 * Sensores de fluxo (sinal de pulso, MAX 3.3 V no pino - ver divisor):
 *   GPIO32 (poco, YF-DN50)   GPIO33 (agua tratada, 1")
 *
 * Pinos ainda livres: GPIO25, GPIO26 (entrada/saida) e GPIO34, GPIO35,
 *   GPIO36, GPIO39 (so entrada, sem pull-up interno).
 *
 * ATENCAO DE HARDWARE:
 *   GPIO12, GPIO2, GPIO15 e GPIO5 sao "strapping pins" do ESP32. O que
 *   mais importa na pratica e o GPIO12 (tensao da flash no boot). Se
 *   notar boot instavel, ligar um pull-down de 10k entre GPIO12 e GND.
 *
 * LOGICA DE AUTOMACAO (maquina de estados):
 *   Start (tudo vazio): ativa watchdog de 110 min -> abre V1, liga B1,
 *     enche T1 -> desliga B1/V1 -> aguarda 50 min -> abre V2, liga B1,
 *     enche T2 -> desliga B1/V2 -> aguarda watchdog -> Ciclo A
 *     (se um tanque ja estiver cheio ao entrar no Start - religamento apos
 *     queda de energia -, aquele enchimento e pulado e nao soma volume)
 *   Ciclo A: purga T1 (5min) -> esvazia T1 (V5) -> enche T1 -> aguarda watchdog -> Ciclo B
 *   Ciclo B: purga T2 (5min) -> esvazia T2 (V5) -> enche T2 -> aguarda watchdog -> Ciclo A
 *   Stop: purga T1+T2 (10min) -> esvazia os dois (V5) -> desliga tudo -> Modo Automatico OFF
 *
 * BOMBA 2 (B2): V3/V4 sao valvulas esfera atuadas por motor eletrico,
 *   que abrem devagar. B2 so liga 30s depois de abrir a purga/dreno,
 *   pra evitar cavitacao enquanto a valvula ainda esta abrindo. Esse
 *   atraso e agendado (nao-bloqueante) uma unica vez, na entrada da
 *   purga - nao se repete na troca pra dreno, ja que B2 ja esta girando.
 *
 * PERSISTENCIA NA NVS:
 *   - "Modo Automatico" (liga/desliga): apos queda de energia, o boot
 *     sempre comeca em AUTO_OFF; se estava ligado, o loop de automacao
 *     dispara um Start normal sozinho (nao retoma o estado interno).
 *   - Volume de cada tanque e volume de purga (ajustaveis pelo app).
 *   - Numero de ciclos/dia e tempo de purga do ciclo (ajustaveis pelo
 *     app). Sao lidos de volta no boot com validacao de faixa, para que
 *     uma queda de energia nao devolva a automacao aos valores default.
 *   - Estado de espera + tempo restante (ver RETOMADA APOS QUEDA abaixo).
 *
 * RETOMADA APOS QUEDA DE ENERGIA:
 *   Os estados de espera (as decantacoes de 50 min e as esperas de janela
 *   de ciclo) sao os que mais custam tempo, e sao justamente os estados em
 *   que NENHUM rele esta acionado. Por isso eles gravam na NVS, a cada
 *   minuto, o proprio estado e quanto tempo ainda falta. No boot, se o
 *   Modo Automatico estava ligado e as boias confirmam os tanques cheios,
 *   a automacao volta exatamente para aquele estado com o tempo que
 *   faltava - sem repetir a decantacao inteira.
 *
 *   O ESP32 nao tem RTC com bateria, entao o tempo de apagao em si nao e
 *   contabilizado: o relogio da decantacao apenas congela e volta de onde
 *   parou. Isso e conservador (decanta-se de mais, nunca de menos), ja que
 *   a decantacao fisica continua acontecendo com a energia cortada.
 *
 *   Os estados ATIVOS (enchendo, purgando, esvaziando) NAO sao retomados.
 *   Voltar sozinho a energizar bomba e valvula depois de um reinicio
 *   inesperado e uma decisao que exige operador; nesses casos o boot faz
 *   um Start normal, como antes.
 *
 * ESTIMATIVAS DE AGUA:
 *   - "agua consumida/dia" = ciclos/dia * media dos volumes dos tanques.
 *   - "agua produzida/dia" = consumida - (ciclos/dia * volume de purga).
 *
 * ESTABILIDADE DO WI-FI / SERVIDOR WEB:
 *   Sintoma observado em campo: depois de muito tempo ligado, o celular
 *   conecta no AP mas a pagina responde "empty response"/"socket error";
 *   a automacao segue normal e so um reinicio resolve. Medidas:
 *   - pilha da task httpd 4 KB -> 8 KB (o handler de status sozinho usa
 *     2 KB de buffer + printf de float, e e chamado a cada 2 s);
 *   - TCP keep-alive nas sessoes HTTP, para derrubar conexoes de
 *     celulares que sairam do alcance/dormiram sem fechar o socket;
 *   - CONFIG_LWIP_MAX_SOCKETS 10 -> 16 no sdkconfig (antes sobrava zero:
 *     7 sessoes + escuta + controle do httpd + DNS = 10);
 *   - DNS do captive portal responde consultas nao-A (ex.: AAAA) sem
 *     registro, em vez de devolver um registro A para elas;
 *   - task de diagnostico que loga heap, sobra de pilha do httpd e
 *     numero de clientes a cada minuto, mais eventos de conexao/DHCP.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "nvs.h"
#include "driver/pulse_cnt.h"
#include "freertos/semphr.h"

static const char *TAG = "qqwater";

// ---------------------------------------------------------------------
// Configuracao de rede (Access Point)
// ---------------------------------------------------------------------
#define AP_SSID       "QQWATER"
#define AP_PASS       "pocoagua123"   // minimo 8 caracteres
#define AP_CHANNEL    1
#define AP_MAX_CONN   4

// ---------------------------------------------------------------------
// Configuracao dos reles
// ---------------------------------------------------------------------
#define NUM_RELAYS 8
#define RELAY_ACTIVE_LEVEL 1   // ativo em HIGH
#define RELAY_INACTIVE_LEVEL 0

static const gpio_num_t relay_pins[NUM_RELAYS] = {
    GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_27,
    GPIO_NUM_15, GPIO_NUM_2,  GPIO_NUM_4,  GPIO_NUM_16
};

static const char *relay_labels[NUM_RELAYS] = {
    "V1 - Entrada T1", "V2 - Entrada T2", "V3 - Saida T1",  "V4 - Saida T2",
    "V5 - Agua Tratada", "V6 - Purga",    "Bomba 1 (Poco)", "Bomba 2 (Saida)"
};

// Indices para deixar a logica de automacao legivel
#define IDX_V1 0
#define IDX_V2 1
#define IDX_V3 2
#define IDX_V4 3
#define IDX_V5 4
#define IDX_V6 5
#define IDX_B1 6
#define IDX_B2 7

static bool relay_state[NUM_RELAYS] = {false};

// ---------------------------------------------------------------------
// Controle dos reles
// ---------------------------------------------------------------------
static void relay_write(int ch, bool on)
{
    if (ch < 0 || ch >= NUM_RELAYS) return;
    gpio_set_level(relay_pins[ch], on ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL);
    relay_state[ch] = on;
    ESP_LOGI(TAG, "Rele %d (%s) -> %s", ch, relay_labels[ch], on ? "LIGADO" : "DESLIGADO");
}

static void all_relays_off(void)
{
    for (int i = 0; i < NUM_RELAYS; i++) {
        relay_write(i, false);
    }
}

static void relays_init(void)
{
    for (int i = 0; i < NUM_RELAYS; i++) {
        gpio_reset_pin(relay_pins[i]);
        gpio_set_direction(relay_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(relay_pins[i], RELAY_INACTIVE_LEVEL);
        relay_state[i] = false;
    }
    ESP_LOGI(TAG, "Todos os reles inicializados em estado DESLIGADO");
}

// ---------------------------------------------------------------------
// Leitura direta das boias nos pinos configurados
// ---------------------------------------------------------------------
#define BOIA_T1_ALTO_PIN  GPIO_NUM_22
#define BOIA_T1_BAIXO_PIN GPIO_NUM_23
#define BOIA_T2_ALTO_PIN  GPIO_NUM_18
#define BOIA_T2_BAIXO_PIN GPIO_NUM_19
#define BOIA_COMMON_PIN   GPIO_NUM_21

#define NUM_SENSOR_CHANNELS 4

static const gpio_num_t boia_pins[NUM_SENSOR_CHANNELS] = {
    BOIA_T1_ALTO_PIN,
    BOIA_T1_BAIXO_PIN,
    BOIA_T2_ALTO_PIN,
    BOIA_T2_BAIXO_PIN,
};

static const char *boia_labels[NUM_SENSOR_CHANNELS] = {
    "Tanque 1 - Alto",
    "Tanque 1 - Baixo",
    "Tanque 2 - Alto",
    "Tanque 2 - Baixo",
};

// Indices para deixar a logica de automacao legivel
#define MIDX_T1_ALTO 0
#define MIDX_T1_BAIXO 1
#define MIDX_T2_ALTO 2
#define MIDX_T2_BAIXO 3

// ---------------------------------------------------------------------
// POLARIDADE DAS BOIAS - LEIA ANTES DE MEXER
// ---------------------------------------------------------------------
// Ligacao eletrica usada por este firmware:
//   - BOIA_COMMON_PIN e SAIDA em nivel ALTO (3.3 V)
//   - cada pino de boia e ENTRADA com PULL-DOWN interno
//   - a boia (chave de nivel) liga o comum ao pino correspondente
//
// Logo, o nivel lido no pino depende SO do contato da chave:
//   contato FECHADO -> pino recebe os 3.3 V do comum  -> le 1 (HIGH)
//   contato ABERTO  -> pull-down interno domina       -> le 0 (LOW)
//
// (Atencao: nao existe pull-up nesta montagem. Comentarios de versoes
//  antigas do firmware descreviam a ligacao invertida, de quando o comum
//  ia ao GND e os pinos usavam pull-up. Se voce mudar a fiacao de volta
//  para aquele esquema, e obrigatorio inverter BOIA_LEVEL_WHEN_UP.)
//
// Comportamento das boias efetivamente instaladas (CONFERIDO no hardware):
//   boia LEVANTADA (com agua) -> contato ABERTO  -> pull-down -> le 0
//   boia CAIDA     (vazio)    -> contato FECHADO -> comum     -> le 1
// Ou seja, sao chaves que ABREM ao subir (tipo NF, fechadas com a boia
// caida). Dai BOIA_LEVEL_WHEN_UP = 0.
//
// Se um dia trocar o modelo da boia (por uma que FECHA ao subir) ou voltar
// a fiacao para comum no GND + pull-up, basta mudar este define para 1 -
// e o unico ponto do codigo que depende da polaridade.
//
// Teste rapido no banco depois de qualquer troca de boia ou de fiacao:
// erguer a boia na mao com o modo debug DESLIGADO; o log deve mostrar
// "Boia N -> ACIONADA" com ela erguida e "normal" ao soltar. Invertido,
// A_DRAIN/B_DRAIN leem tanque vazio com o tanque cheio.
#define BOIA_LEVEL_WHEN_UP 0

// boia_state[i] = true  -> boia LEVANTADA (ha agua naquela altura) = "ACIONADA"
// boia_state[i] = false -> boia CAIDA (sem agua naquela altura)    = "normal"
static bool boia_state[NUM_SENSOR_CHANNELS] = {false};
static int  boia_debounce_count[NUM_SENSOR_CHANNELS] = {0};
static bool boia_last_raw[NUM_SENSOR_CHANNELS] = {false};
// Reservado para deteccao de boia desconectada/em falha. Hoje nada escreve
// false aqui - as guardas !sensor_available() na automacao existem para
// quando essa deteccao for implementada.
static bool boia_available[NUM_SENSOR_CHANNELS] = {true, true, true, true};
#define BOIA_DEBOUNCE_THRESHOLD 3
#define BOIA_POLL_PERIOD_MS 50

static void boias_init(void)
{
    gpio_reset_pin(BOIA_COMMON_PIN);
    gpio_set_direction(BOIA_COMMON_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BOIA_COMMON_PIN, 1);

    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++) {
        gpio_reset_pin(boia_pins[i]);
        gpio_set_direction(boia_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(boia_pins[i], GPIO_PULLDOWN_ONLY);
        boia_available[i] = true;
    }

    ESP_LOGI(TAG, "Boias inicializadas nos pinos T1 alto=%d T1 baixo=%d T2 alto=%d T2 baixo=%d comum=%d",
             BOIA_T1_ALTO_PIN, BOIA_T1_BAIXO_PIN, BOIA_T2_ALTO_PIN, BOIA_T2_BAIXO_PIN, BOIA_COMMON_PIN);
}

static void boias_poll_once(void)
{
    for (int ch = 0; ch < NUM_SENSOR_CHANNELS; ch++) {
        int raw_level = gpio_get_level(boia_pins[ch]);
        bool raw_up = (raw_level == BOIA_LEVEL_WHEN_UP);

        if (raw_up == boia_last_raw[ch]) {
            if (boia_debounce_count[ch] < BOIA_DEBOUNCE_THRESHOLD) {
                boia_debounce_count[ch]++;
            }
        } else {
            boia_last_raw[ch] = raw_up;
            boia_debounce_count[ch] = 0;
        }

        if (boia_debounce_count[ch] >= BOIA_DEBOUNCE_THRESHOLD && boia_state[ch] != raw_up) {
            boia_state[ch] = raw_up;
            ESP_LOGI(TAG, "Boia %d (%s) -> %s", ch, boia_labels[ch],
                     raw_up ? "ACIONADA" : "normal");
        }
    }
}

static void boias_task(void *arg)
{
    while (1) {
        boias_poll_once();
        vTaskDelay(pdMS_TO_TICKS(BOIA_POLL_PERIOD_MS));
    }
}

// ---------------------------------------------------------------------
// Sensores de fluxo (vazao instantanea + totalizador)
// ---------------------------------------------------------------------
// Sensores de efeito Hall: a cada litro saem K pulsos. A contagem e feita
// em hardware pelo PCNT do ESP32 (nao perde pulso mesmo com a CPU ocupada).
//
//   Poco (saida do poco) ...... YF-DN50 2", 10-200 L/min, F[Hz] = 0.2*Q[L/min]
//                               -> K = 0.2*60 = 12 pulsos/L (datasheet, +-3%)
//   Tratada (saida do trat.) .. GDZHONGOIN 1", 1-50 L/min, sem datasheet.
//                               K inicial = 288 pulsos/L (F = 4.8*Q, tipico
//                               de sensores Hall de 1") - CALIBRAR.
//
// Calibracao do K: passe um volume conhecido (balde, ou compare com o
// hidrometro depois de alguns m3) e ajuste:
//     K_novo = K_atual * (volume_indicado / volume_real)
//
// LIGACAO ELETRICA - IMPORTANTE:
//   O YF-DN50 alimentado em 5 V entrega pulso de ~4.7 V (tem pull-up interno
//   para o VCC). O GPIO do ESP32 aceita no maximo 3.6 V: use um divisor
//   (ex.: 10k em serie + 20k para o GND -> ~3.2 V) no fio de sinal. Antes de
//   ligar o sensor de 1", meca o fio de sinal com o sensor parado: se der
//   ~VCC, use o mesmo divisor; se der ~0 V/flutuante (coletor aberto), basta
//   o pull-up interno do ESP32 que ja e habilitado aqui. Alimente os sensores
//   em 5 V (nao em 12/24 V) para manter o divisor valido.
#define FLOW_POCO     0
#define FLOW_TRATADA  1
#define NUM_FLOW      2

#define FLOW_PIN_POCO     GPIO_NUM_32
#define FLOW_PIN_TRATADA  GPIO_NUM_33

#define FLOW_K_MIN        0.1     // pulsos/L
#define FLOW_K_MAX        10000.0
#define FLOW_TOTAL_MAX_M3 9999999.0
#define FLOW_WINDOW_S     5       // janela da vazao instantanea (media movel)
#define FLOW_SAVE_PERIOD_US (60LL * 1000 * 1000)  // grava total a cada 1 min com fluxo
#define FLOW_PCNT_LIMIT   30000
#define FLOW_PCNT_REBASE  1000000000  // zera o contador antes de chegar no limite do int

typedef struct {
    const char *label;
    gpio_num_t  pin;
    const char *nvs_total_key;   // u64: total em mL ja "consolidado"
    const char *nvs_k_key;       // u32: K * 1000
    double      k_default;

    pcnt_unit_handle_t unit;
    bool        ok;              // PCNT inicializado
    int         last_count;

    double      k_ppl;           // pulsos por litro
    uint64_t    base_ml;         // total consolidado (mL)
    uint64_t    pulses;          // pulsos desde a ultima consolidacao
    uint32_t    window[FLOW_WINDOW_S];
    int         window_pos;
    double      lpm;             // vazao instantanea (L/min)
    bool        dirty;           // ha pulsos ainda nao gravados na NVS
    int64_t     last_save_us;
} flow_sensor_t;

static flow_sensor_t flow[NUM_FLOW] = {
    { .label = "Poco",    .pin = FLOW_PIN_POCO,    .nvs_total_key = "fl0_ml", .nvs_k_key = "fl0_k1000", .k_default = 12.0  },
    { .label = "Tratada", .pin = FLOW_PIN_TRATADA, .nvs_total_key = "fl1_ml", .nvs_k_key = "fl1_k1000", .k_default = 288.0 },
};
static SemaphoreHandle_t flow_mutex = NULL;

// Chamar com o mutex tomado. Total em mL = consolidado + pulsos/K.
static uint64_t flow_total_ml_locked(const flow_sensor_t *f)
{
    return f->base_ml + (uint64_t)((double)f->pulses * 1000.0 / f->k_ppl);
}

// Chamar com o mutex tomado. Incorpora os pulsos pendentes ao total com o
// K atual - necessario antes de trocar o K, para nao reinterpretar
// pulsos antigos com o fator novo.
static void flow_fold_locked(flow_sensor_t *f)
{
    f->base_ml = flow_total_ml_locked(f);
    f->pulses = 0;
}

static void flow_save_to_nvs(int i)
{
    uint64_t total_ml;
    uint32_t k1000;
    xSemaphoreTake(flow_mutex, portMAX_DELAY);
    flow_fold_locked(&flow[i]);
    total_ml = flow[i].base_ml;
    k1000 = (uint32_t)(flow[i].k_ppl * 1000.0 + 0.5);
    flow[i].dirty = false;
    flow[i].last_save_us = esp_timer_get_time();
    xSemaphoreGive(flow_mutex);

    nvs_handle_t handle = 0;
    if (nvs_open("qqwater", NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao abrir NVS para gravar fluxo %s", flow[i].label);
        return;
    }
    nvs_set_u64(handle, flow[i].nvs_total_key, total_ml);
    nvs_set_u32(handle, flow[i].nvs_k_key, k1000);
    nvs_commit(handle);
    nvs_close(handle);
}

static void flow_load_from_nvs(void)
{
    nvs_handle_t handle = 0;
    bool opened = (nvs_open("qqwater", NVS_READONLY, &handle) == ESP_OK);
    for (int i = 0; i < NUM_FLOW; i++) {
        flow[i].k_ppl = flow[i].k_default;
        flow[i].base_ml = 0;
        if (opened) {
            uint64_t ml = 0;
            uint32_t k1000 = 0;
            if (nvs_get_u64(handle, flow[i].nvs_total_key, &ml) == ESP_OK) {
                flow[i].base_ml = ml;
            }
            if (nvs_get_u32(handle, flow[i].nvs_k_key, &k1000) == ESP_OK) {
                double k = k1000 / 1000.0;
                if (k >= FLOW_K_MIN && k <= FLOW_K_MAX) flow[i].k_ppl = k;
            }
        }
        ESP_LOGI(TAG, "Fluxo %s: total=%.3f m3, K=%.3f pulsos/L", flow[i].label,
                 flow[i].base_ml / 1e6, flow[i].k_ppl);
    }
    if (opened) nvs_close(handle);
}

static void flow_init(void)
{
    flow_mutex = xSemaphoreCreateMutex();
    flow_load_from_nvs();

    for (int i = 0; i < NUM_FLOW; i++) {
        flow_sensor_t *f = &flow[i];
        pcnt_unit_config_t ucfg = {
            .low_limit = -FLOW_PCNT_LIMIT,
            .high_limit = FLOW_PCNT_LIMIT,
            .flags.accum_count = 1,   // acumula alem do limite de 16 bits do hardware
        };
        pcnt_chan_config_t ccfg = {
            .edge_gpio_num = f->pin,
            .level_gpio_num = -1,
        };
        pcnt_glitch_filter_config_t gcfg = {
            .max_glitch_ns = 10000,   // ignora ruido < 10 us (pulsos reais sao de ms)
        };
        pcnt_channel_handle_t chan = NULL;

        esp_err_t err = pcnt_new_unit(&ucfg, &f->unit);
        if (err == ESP_OK) err = pcnt_unit_set_glitch_filter(f->unit, &gcfg);
        if (err == ESP_OK) err = pcnt_new_channel(f->unit, &ccfg, &chan);
        if (err == ESP_OK) err = pcnt_channel_set_edge_action(chan,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // conta borda de subida
                                     PCNT_CHANNEL_EDGE_ACTION_HOLD);
        // A acumulacao alem do limite so funciona com watch points nos limites.
        if (err == ESP_OK) err = pcnt_unit_add_watch_point(f->unit, FLOW_PCNT_LIMIT);
        if (err == ESP_OK) err = pcnt_unit_add_watch_point(f->unit, -FLOW_PCNT_LIMIT);
        if (err == ESP_OK) err = pcnt_unit_enable(f->unit);
        if (err == ESP_OK) err = pcnt_unit_clear_count(f->unit);
        if (err == ESP_OK) err = pcnt_unit_start(f->unit);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Fluxo %s: falha ao iniciar PCNT no GPIO%d: %s",
                     f->label, f->pin, esp_err_to_name(err));
            continue;
        }
        // O driver do PCNT nao mexe nos resistores: liga o pull-up interno
        // (necessario se o sensor for coletor aberto, inofensivo se nao for).
        gpio_set_pull_mode(f->pin, GPIO_PULLUP_ONLY);
        f->ok = true;
        f->last_count = 0;
        ESP_LOGI(TAG, "Fluxo %s: PCNT ativo no GPIO%d", f->label, f->pin);
    }
}

// Roda a cada 1 s: le os contadores, atualiza vazao e total, grava na NVS.
static void flow_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000));
        int64_t now = esp_timer_get_time();

        for (int i = 0; i < NUM_FLOW; i++) {
            flow_sensor_t *f = &flow[i];
            if (!f->ok) continue;

            int count = 0;
            if (pcnt_unit_get_count(f->unit, &count) != ESP_OK) continue;
            int delta = count - f->last_count;
            if (delta < 0) delta = 0;   // nao deveria ocorrer (so conta subida)
            f->last_count = count;
            if (count > FLOW_PCNT_REBASE) {
                // Evita estourar o int depois de anos de contagem acumulada.
                pcnt_unit_clear_count(f->unit);
                f->last_count = 0;
            }

            bool save_now = false;
            xSemaphoreTake(flow_mutex, portMAX_DELAY);
            bool was_flowing = f->lpm > 0.0;
            f->pulses += (uint32_t)delta;
            f->window[f->window_pos] = (uint32_t)delta;
            f->window_pos = (f->window_pos + 1) % FLOW_WINDOW_S;
            uint32_t sum = 0;
            for (int w = 0; w < FLOW_WINDOW_S; w++) sum += f->window[w];
            f->lpm = ((double)sum / FLOW_WINDOW_S) * 60.0 / f->k_ppl;
            if (delta > 0) f->dirty = true;
            // Grava 1x por minuto enquanto ha fluxo, e logo que o fluxo para,
            // para uma queda de energia perder no maximo ~1 min de contagem.
            if (f->dirty && ((now - f->last_save_us) >= FLOW_SAVE_PERIOD_US ||
                             (was_flowing && f->lpm == 0.0))) {
                save_now = true;
            }
            xSemaphoreGive(flow_mutex);

            if (save_now) flow_save_to_nvs(i);
        }
    }
}

static double flow_lpm(int i)
{
    if (i < 0 || i >= NUM_FLOW || !flow_mutex) return 0.0;
    xSemaphoreTake(flow_mutex, portMAX_DELAY);
    double v = flow[i].lpm;
    xSemaphoreGive(flow_mutex);
    return v;
}

static double flow_total_m3(int i)
{
    if (i < 0 || i >= NUM_FLOW || !flow_mutex) return 0.0;
    xSemaphoreTake(flow_mutex, portMAX_DELAY);
    double v = flow_total_ml_locked(&flow[i]) / 1e6;
    xSemaphoreGive(flow_mutex);
    return v;
}

static double flow_k(int i)
{
    if (i < 0 || i >= NUM_FLOW || !flow_mutex) return 0.0;
    xSemaphoreTake(flow_mutex, portMAX_DELAY);
    double v = flow[i].k_ppl;
    xSemaphoreGive(flow_mutex);
    return v;
}

// Ajusta o totalizador para bater com o hidrometro analogico.
static bool flow_set_total_m3(int i, double m3)
{
    if (i < 0 || i >= NUM_FLOW || m3 < 0.0 || m3 > FLOW_TOTAL_MAX_M3) return false;
    xSemaphoreTake(flow_mutex, portMAX_DELAY);
    flow[i].base_ml = (uint64_t)(m3 * 1e6 + 0.5);
    flow[i].pulses = 0;
    xSemaphoreGive(flow_mutex);
    flow_save_to_nvs(i);
    ESP_LOGI(TAG, "Fluxo %s: totalizador ajustado para %.3f m3", flow[i].label, m3);
    return true;
}

static bool flow_set_k(int i, double k)
{
    if (i < 0 || i >= NUM_FLOW || k < FLOW_K_MIN || k > FLOW_K_MAX) return false;
    xSemaphoreTake(flow_mutex, portMAX_DELAY);
    flow_fold_locked(&flow[i]);   // pulsos antigos ficam com o K antigo
    flow[i].k_ppl = k;
    xSemaphoreGive(flow_mutex);
    flow_save_to_nvs(i);
    ESP_LOGI(TAG, "Fluxo %s: K ajustado para %.3f pulsos/L", flow[i].label, k);
    return true;
}

// ---------------------------------------------------------------------
// Modo debug: ignora a leitura real das boias e usa valores definidos
// manualmente pelo app, para testar a automacao sem sensores fisicos.
// ---------------------------------------------------------------------
static volatile bool debug_mode = false;
static volatile bool debug_sensor_state[NUM_SENSOR_CHANNELS] = {false};

static inline bool sensor_available(int ch)
{
    return (ch >= 0 && ch < NUM_SENSOR_CHANNELS && boia_available[ch]);
}

// true = boia levantada (ha agua naquela altura)
static inline bool effective_sensor_state(int ch)
{
    if (ch < 0 || ch >= NUM_SENSOR_CHANNELS) return false;
    if (!sensor_available(ch)) return false;
    return debug_mode ? debug_sensor_state[ch] : boia_state[ch];
}

// Tanque cheio = boia de cima levantada.
static inline bool tank_high(int tank)
{
    return tank == 1 ? effective_sensor_state(MIDX_T1_ALTO) : effective_sensor_state(MIDX_T2_ALTO);
}

// Tanque vazio = boia de baixo CAIDA (sem agua nem no fundo). Por isso a
// negacao: effective_sensor_state() significa "levantada", e a boia de
// fundo fica levantada em qualquer nivel acima dela (enchendo ou cheio).
// Resumo dos tres estados possiveis:
//   alto caida,     baixo caida     -> vazio                (tank_low = true)
//   alto caida,     baixo levantada -> enchendo/esvaziando  (tank_low = false)
//   alto levantada, baixo levantada -> cheio                (tank_low = false)
static inline bool tank_low(int tank)
{
    int idx = (tank == 1) ? MIDX_T1_BAIXO : MIDX_T2_BAIXO;
    if (!sensor_available(idx)) {
        return false;
    }
    return !effective_sensor_state(idx);
}

// ---------------------------------------------------------------------
// Automacao (maquina de estados)
// ---------------------------------------------------------------------
// Flags de sensores ainda nao instalados. Mude para true quando ligar
// fisicamente a chave correspondente - a logica ja esta pronta.
//
// FLOW_ENTRADA_INSTALLED: protecao contra bomba 1 a seco usando o sensor de
// fluxo do poco. Com true, se a vazao do poco ficar abaixo de
// DRYRUN_MIN_LPM por DRYRUN_GRACE_MS durante um enchimento, o sistema entra
// em falha e desliga tudo. So ligue depois de conferir em campo que o
// sensor do poco esta lendo a vazao corretamente.
#define FLOW_ENTRADA_INSTALLED   false
#define DRYRUN_MIN_LPM           5.0
#define OVERPRESSURE_INSTALLED   false

#define DECANT_MS        (50LL * 60 * 1000)   // tempo minimo de decantacao, fixo
#define MIN_PURGE_MIN 1
#define MAX_PURGE_MIN 60
#define STOP_WATCHDOG_MS (30LL * 60 * 1000)   // watchdog fixo da sequencia de parada
#define DRYRUN_GRACE_MS  (20LL * 1000)
#define B2_START_DELAY_MS (30LL * 1000)
#define DAY_MS           (24LL * 60 * 60 * 1000)

#define DAILY_LIMIT_M3    18.0
#define MIN_CYCLES_PER_DAY 1
#define MAX_CYCLES_PER_DAY 28   // acima disso o watchdog (1440/n - 50) fica <= 0

// Volume de cada tanque (ajustavel pelo app - a boia raramente para em
// exatos 2 m3, entao esse campo permite corrigir a estimativa sem
// precisar recompilar/reflashear o firmware).
#define TANK_VOLUME_MIN_M3 0.1
#define TANK_VOLUME_MAX_M3 10.0
static volatile double tank1_volume_m3 = 1.6;
static volatile double tank2_volume_m3 = 1.6;

// Volume perdido/descartado a cada purga (sedimento + agua). Usado so
// para estimar a "agua produzida" (consumida - purgada); nao afeta a
// automacao em si.
#define PURGE_VOLUME_MIN_M3 0.0
#define PURGE_VOLUME_MAX_M3 5.0
static volatile double purge_volume_m3 = 0.1;

typedef enum {
    AUTO_OFF = 0,
    ST_START_FILL,
    ST_START_DECANT,
    ST_START_FILL2,
    ST_START_WAIT,
    A_DECANT,
    A_PURGE,
    A_DRAIN,
    A_FILL,
    A_WAIT,
    B_DECANT,
    B_PURGE,
    B_DRAIN,
    B_FILL,
    B_WAIT,
    STOP_PURGE,
    STOP_DRAIN,
    STOP_DONE,
} auto_state_t;

// auto_enabled e persistido na NVS (so o liga/desliga - nao o estado
// interno da maquina). Depois de uma queda de energia o boot sempre
// comeca em AUTO_OFF; se auto_enabled volta true da NVS, o loop de
// automacao entende isso como "religar" e dispara um Start normal
// (ST_START_FILL), igual a apertar o toggle manualmente.
static volatile bool auto_enabled = false;
static volatile bool stop_requested = false;
static volatile bool skip_requested = false;
static volatile int32_t num_cycles_per_day = 10; // configuravel pelo app, persistido na NVS
static volatile int32_t purge_cycle_minutes = 1;  // purga do Ciclo A/B, persistido na NVS
static volatile uint32_t total_cycles = 0;        // ciclos A/B completos desde o boot
static volatile double total_water_in_m3 = 0.0;   // agua estimada que entrou (poco -> tanque)
static auto_state_t auto_state = AUTO_OFF;
static int64_t state_enter_time_us = 0;
static int64_t no_flow_since_us = -1;
// true quando o Start encontrou o tanque ja cheio (ex.: religamento apos
// queda de energia): o "enchimento" daquele estado nao aconteceu e nao
// entra no volume estimado.
static bool start_fill_already_full = false;
static char fault_msg[64] = "";

// Agendamento nao-bloqueante do atraso de 30s antes de ligar a Bomba 2
// (V3/V4 sao valvulas esfera motorizadas, abrem devagar - ligar B2 antes
// de abrirem o suficiente causa cavitacao). Agendado 1x na entrada da
// purga; o proprio loop da automacao verifica o prazo e liga a B2.
static bool b2_start_pending = false;
static int64_t b2_start_deadline_us = 0;

// Janela de seguranca da etapa ativa do ciclo (purga+esvazia+enche). Se as
// boias nao confirmarem a operacao dentro desse prazo, e uma falha real
// (vazao do poco ou boia com problema), nao um fim de ciclo normal.
static int64_t watchdog_deadline_us = 0;
static int64_t watchdog_total_ms = 0;

// Gravacao periodica do progresso dos estados de espera na NVS, para
// permitir retomar de onde parou apos uma queda de energia. Um minuto de
// resolucao e suficiente (erro maximo de 1 min numa espera de 50 a 95 min)
// e mantem o desgaste da flash desprezivel: como a NVS ignora escritas com
// valor identico, so o campo "faltam X segundos" e realmente regravado,
// uma vez por minuto e apenas durante as esperas.
#define STATE_SAVE_PERIOD_US (60LL * 1000 * 1000)
static int64_t last_state_save_us = 0;

static inline int64_t elapsed_ms(void)
{
    return (esp_timer_get_time() - state_enter_time_us) / 1000;
}

static inline int64_t purge_cycle_ms(void) { return (int64_t)purge_cycle_minutes * 60 * 1000; }
static inline int64_t purge_stop_ms(void)  { return purge_cycle_ms() * 2; }

// Watchdog do ciclo normal (Start/A/B): tempo do ciclo (1440/n) menos a
// decantacao fixa de 50 min. Recalculado a cada ativacao, usando o
// numero de ciclos configurado no momento.
static int64_t get_cycle_watchdog_ms(void)
{
    int64_t cycle_period_ms = DAY_MS / num_cycles_per_day;
    int64_t wd = cycle_period_ms - DECANT_MS;
    if (wd < 0) wd = 0;
    return wd;
}

static void activate_watchdog(int64_t total_ms)
{
    watchdog_total_ms = total_ms;
    watchdog_deadline_us = esp_timer_get_time() + total_ms * 1000;
}

static inline bool watchdog_expired(void)
{
    return esp_timer_get_time() >= watchdog_deadline_us;
}

// So chamar dentro dos estados ativos (purga/esvazia/enche) protegidos
// por um watchdog - nao chamar durante os estados de espera (*_WAIT),
// onde o watchdog expirar e o comportamento normal, nao uma falha.
static void trigger_fault(const char *msg); // definida mais abaixo
static void set_auto_enabled(bool en);      // definida mais abaixo (grava na NVS)
static void persist_auto_state(void);       // grava estado + tempo restante na NVS
static void clear_persisted_auto_state(void); // marca "nada a retomar" na NVS
static void check_watchdog_fault(void)
{
    if (watchdog_expired()) {
        trigger_fault("Problema nas boias ou na vazao do poco");
    }
}

// Agenda o atraso nao-bloqueante de ligar a B2 (chamar 1x, na entrada da purga)
static void schedule_b2_start(void)
{
    relay_write(IDX_B2, false);
    b2_start_pending = true;
    b2_start_deadline_us = esp_timer_get_time() + B2_START_DELAY_MS * 1000;
    ESP_LOGI(TAG, "Atraso de %d s agendado para ligar B2", (int)(B2_START_DELAY_MS / 1000));
}

// Estados que podem ser retomados depois de um reinicio. Todos eles tem
// duas propriedades essenciais: nenhum rele acionado (entao o boot com
// tudo desligado ja e o estado correto do hardware) e o unico "progresso"
// e a passagem do tempo, que da para gravar. Os estados ativos ficam de
// fora de proposito - ver a nota RETOMADA APOS QUEDA no topo do arquivo.
static bool state_is_resumable(auto_state_t s)
{
    switch (s) {
        case ST_START_DECANT:
        case A_DECANT:
        case B_DECANT:
        case ST_START_WAIT:
        case A_WAIT:
        case B_WAIT:
            return true;
        default:
            return false;
    }
}

static void enter_state(auto_state_t new_state)
{
    auto_state = new_state;
    state_enter_time_us = esp_timer_get_time();

    switch (new_state) {
        case AUTO_OFF:
            b2_start_pending = false;
            all_relays_off();
            break;
        case ST_START_FILL:
            fault_msg[0] = '\0';
            activate_watchdog(get_cycle_watchdog_ms());
            no_flow_since_us = -1;
            // Tanque ja cheio na entrada do Start (tipico de religamento apos
            // queda de energia): nao houve enchimento, entao nao aciona bomba
            // e valvula e nao soma volume - o loop segue direto para a
            // decantacao no proximo ciclo.
            start_fill_already_full = tank_high(1);
            if (start_fill_already_full) {
                ESP_LOGI(TAG, "Start: tanque 1 ja estava cheio - enchimento nao contabilizado");
            } else {
                relay_write(IDX_V1, true);
                relay_write(IDX_B1, true);
            }
            break;
        case ST_START_DECANT:
            relay_write(IDX_B1, false);
            relay_write(IDX_V1, false);
            break;
        case ST_START_FILL2:
            no_flow_since_us = -1;
            start_fill_already_full = tank_high(2);
            if (start_fill_already_full) {
                ESP_LOGI(TAG, "Start: tanque 2 ja estava cheio - enchimento nao contabilizado");
            } else {
                relay_write(IDX_V2, true);
                relay_write(IDX_B1, true);
            }
            break;
        case ST_START_WAIT:
            relay_write(IDX_B1, false);
            relay_write(IDX_V2, false);
            break;
        case A_DECANT:
            // so espera, reles ja desligados pelo estado anterior (*_WAIT)
            break;
        case A_PURGE:
            activate_watchdog(get_cycle_watchdog_ms());
            relay_write(IDX_V3, true);
            relay_write(IDX_V6, true);
            schedule_b2_start();
            break;
        case A_DRAIN:
            relay_write(IDX_V5, true);
            relay_write(IDX_V6, false);
            break;
        case A_FILL:
            no_flow_since_us = -1;
            relay_write(IDX_V3, false);
            relay_write(IDX_V5, false);
            relay_write(IDX_B2, false);
            relay_write(IDX_V1, true);
            relay_write(IDX_B1, true);
            break;
        case A_WAIT:
            relay_write(IDX_B1, false);
            relay_write(IDX_V1, false);
            break;
        case B_DECANT:
            break;
        case B_PURGE:
            activate_watchdog(get_cycle_watchdog_ms());
            relay_write(IDX_V4, true);
            relay_write(IDX_V6, true);
            schedule_b2_start();
            break;
        case B_DRAIN:
            relay_write(IDX_V5, true);
            relay_write(IDX_V6, false);
            break;
        case B_FILL:
            no_flow_since_us = -1;
            relay_write(IDX_V4, false);
            relay_write(IDX_V5, false);
            relay_write(IDX_B2, false);
            relay_write(IDX_V2, true);
            relay_write(IDX_B1, true);
            break;
        case B_WAIT:
            relay_write(IDX_B1, false);
            relay_write(IDX_V2, false);
            break;
        case STOP_PURGE:
            activate_watchdog(STOP_WATCHDOG_MS);
            relay_write(IDX_V3, true);
            relay_write(IDX_V4, true);
            relay_write(IDX_V6, true);
            schedule_b2_start();
            break;
        case STOP_DRAIN:
            relay_write(IDX_V5, true);
            relay_write(IDX_V6, false);
            break;
        case STOP_DONE:
            relay_write(IDX_V3, false);
            relay_write(IDX_V4, false);
            relay_write(IDX_V5, false);
            relay_write(IDX_B2, false);
            set_auto_enabled(false);
            stop_requested = false;
            enter_state(AUTO_OFF);
            return;
    }

    // Marca na NVS se este estado pode ou nao ser retomado apos um
    // reinicio. Limpar nos estados ativos e o que impede o boot de
    // ressuscitar uma decantacao velha depois de uma queda durante,
    // por exemplo, a purga.
    if (state_is_resumable(new_state)) {
        persist_auto_state();
    } else {
        clear_persisted_auto_state();
    }

    ESP_LOGI(TAG, "Automacao -> estado %d", (int)new_state);
}

static const char *auto_state_text(void)
{
    if (fault_msg[0]) return fault_msg;
    switch (auto_state) {
        case AUTO_OFF:        return "Manual (automatico desligado)";
        case ST_START_FILL:   return "Start - enchendo tanque 1";
        case ST_START_DECANT: return "Start - decantando (aguardando 50min)";
        case ST_START_FILL2:  return "Start - enchendo tanque 2";
        case ST_START_WAIT:   return "Start - aguardando watchdog para ciclo A";
        case A_DECANT:        return "Ciclo A - decantando (aguardando 50min)";
        case A_PURGE:         return "Ciclo A - purgando tanque 1";
        case A_DRAIN:         return "Ciclo A - esvaziando tanque 1";
        case A_FILL:          return "Ciclo A - enchendo tanque 1";
        case A_WAIT:          return "Aguardando janela do ciclo - ciclo A concluido";
        case B_DECANT:        return "Ciclo B - decantando (aguardando 50min)";
        case B_PURGE:         return "Ciclo B - purgando tanque 2";
        case B_DRAIN:         return "Ciclo B - esvaziando tanque 2";
        case B_FILL:          return "Ciclo B - enchendo tanque 2";
        case B_WAIT:          return "Aguardando janela do ciclo - ciclo B concluido";
        case STOP_PURGE:      return "Parando sistema - purgando";
        case STOP_DRAIN:      return "Parando sistema - esvaziando tanques";
        case STOP_DONE:       return "Sistema parado";
        default:              return "-";
    }
}

static void trigger_fault(const char *msg)
{
    strncpy(fault_msg, msg, sizeof(fault_msg) - 1);
    fault_msg[sizeof(fault_msg) - 1] = '\0';
    ESP_LOGE(TAG, "FALHA: %s", msg);
    b2_start_pending = false;
    all_relays_off();
    set_auto_enabled(false);
    stop_requested = false;
    auto_state = AUTO_OFF;
    state_enter_time_us = esp_timer_get_time();
    // Falha nao pode ser retomada sozinha no proximo boot.
    clear_persisted_auto_state();
}

// So chamar durante estados em que a Bomba 1 esta enchendo um tanque
static void check_dryrun(void)
{
    if (!FLOW_ENTRADA_INSTALLED) return;

    bool has_flow = flow_lpm(FLOW_POCO) >= DRYRUN_MIN_LPM;
    if (!has_flow) {
        if (no_flow_since_us < 0) {
            no_flow_since_us = esp_timer_get_time();
        } else if ((esp_timer_get_time() - no_flow_since_us) / 1000 >= DRYRUN_GRACE_MS) {
            trigger_fault("Sem fluxo na entrada - bomba 1 desligada");
        }
    } else {
        no_flow_since_us = -1;
    }
}

static void check_overpressure(void)
{
    if (!OVERPRESSURE_INSTALLED) return;
    if (false) {
        trigger_fault("Sobrepressao detectada - sistema desligado");
    }
}

static bool is_auto_locked(void)
{
    return auto_state != AUTO_OFF;
}

// Duracao total (ms) do estado atual, para exibir o timer no app.
// Retorna 0 se o estado depende de nivel de boia (sem timer fixo).
static int64_t state_total_ms(void)
{
    switch (auto_state) {
        case ST_START_WAIT:
        case A_WAIT:
        case B_WAIT:
            return watchdog_total_ms;
        case ST_START_DECANT:
        case A_DECANT:
        case B_DECANT:
            return DECANT_MS;
        case A_PURGE:
        case B_PURGE:
            return purge_cycle_ms();
        case STOP_PURGE:
            return purge_stop_ms();
        default:
            return 0;
    }
}

static int64_t state_remaining_ms(void)
{
    int64_t rem;
    switch (auto_state) {
        case ST_START_WAIT:
        case A_WAIT:
        case B_WAIT:
            rem = (watchdog_deadline_us - esp_timer_get_time()) / 1000;
            break;
        default:
            rem = state_total_ms() - elapsed_ms();
            break;
    }
    return rem > 0 ? rem : 0;
}

// Numero de ciclos/dia configurado pelo app. Valida faixa e recalcula o
// watchdog na proxima ativacao (nao mexe no ciclo que ja esta rodando).
static bool set_num_cycles_per_day(int32_t n)
{
    if (n < MIN_CYCLES_PER_DAY || n > MAX_CYCLES_PER_DAY) return false;
    num_cycles_per_day = n;
    ESP_LOGI(TAG, "Ciclos/dia configurado para %d", (int)n);
    return true;
}

// Estimativa de agua consumida do poco por dia. Os ciclos alternam entre
// tanque 1 e 2, entao usa a media dos dois volumes configurados.
static double estimated_consumed_daily_m3(void)
{
    return num_cycles_per_day * ((tank1_volume_m3 + tank2_volume_m3) / 2.0);
}

// Estimativa de agua "produzida" (aproveitavel) por dia: consumida menos
// o que e descartado na purga a cada ciclo (1 purga por ciclo A ou B).
static double estimated_produced_daily_m3(void)
{
    double produced = estimated_consumed_daily_m3() - num_cycles_per_day * purge_volume_m3;
    return produced > 0 ? produced : 0.0;
}

static bool set_purge_cycle_minutes(int32_t min)
{
    if (min < MIN_PURGE_MIN || min > MAX_PURGE_MIN) return false;
    purge_cycle_minutes = min;
    ESP_LOGI(TAG, "Tempo de purga (ciclo) configurado para %d min", (int)min);
    return true;
}

static bool set_tank_volume(int tank, double m3)
{
    if (m3 < TANK_VOLUME_MIN_M3 || m3 > TANK_VOLUME_MAX_M3) return false;
    if (tank == 1) {
        tank1_volume_m3 = m3;
    } else if (tank == 2) {
        tank2_volume_m3 = m3;
    } else {
        return false;
    }
    ESP_LOGI(TAG, "Volume do tanque %d configurado para %.2f m3", tank, m3);
    return true;
}

static bool set_purge_volume(double m3)
{
    if (m3 < PURGE_VOLUME_MIN_M3 || m3 > PURGE_VOLUME_MAX_M3) return false;
    purge_volume_m3 = m3;
    ESP_LOGI(TAG, "Volume de purga configurado para %.2f m3", m3);
    return true;
}

// ---------------------------------------------------------------------
// NVS
// ---------------------------------------------------------------------
// Chaves usadas (limite de 15 caracteres por chave):
//   tot_cycles      u32  - ciclos A/B completos
//   tot_water_x10   u32  - m3 acumulados * 10
//   tank1_vol_x100  u32  - volume do tanque 1 * 100
//   tank2_vol_x100  u32  - volume do tanque 2 * 100
//   purge_vol_x100  u32  - volume de purga * 100
//   num_cycles      u32  - ciclos por dia
//   purge_cyc_min   u32  - minutos de purga do ciclo
//   auto_en         u8   - modo automatico ligado/desligado
// ---------------------------------------------------------------------
static void load_totals_from_nvs(void)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open("qqwater", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "NVS de totalizadores nao encontrado, usando valores iniciais");
        return;
    }

    uint32_t cycles = 0;
    uint32_t water_x10 = 0;
    uint32_t tank1_x100 = 0;
    uint32_t tank2_x100 = 0;
    uint32_t purge_x100 = 0;
    uint32_t cycles_day = 0;
    uint32_t purge_min = 0;
    uint8_t  auto_en = 0;

    err = nvs_get_u32(handle, "tot_cycles", &cycles);
    if (err == ESP_OK) {
        total_cycles = cycles;
    }
    err = nvs_get_u32(handle, "tot_water_x10", &water_x10);
    if (err == ESP_OK) {
        total_water_in_m3 = (double)water_x10 / 10.0;
    }
    // Os setters abaixo validam a faixa: se a NVS estiver corrompida ou
    // vier de uma versao antiga com outros limites, o valor e descartado
    // e o default de compilacao permanece.
    err = nvs_get_u32(handle, "tank1_vol_x100", &tank1_x100);
    if (err == ESP_OK && !set_tank_volume(1, (double)tank1_x100 / 100.0)) {
        ESP_LOGW(TAG, "Volume do tanque 1 na NVS fora da faixa, mantendo %.2f m3", tank1_volume_m3);
    }
    err = nvs_get_u32(handle, "tank2_vol_x100", &tank2_x100);
    if (err == ESP_OK && !set_tank_volume(2, (double)tank2_x100 / 100.0)) {
        ESP_LOGW(TAG, "Volume do tanque 2 na NVS fora da faixa, mantendo %.2f m3", tank2_volume_m3);
    }
    err = nvs_get_u32(handle, "purge_vol_x100", &purge_x100);
    if (err == ESP_OK && !set_purge_volume((double)purge_x100 / 100.0)) {
        ESP_LOGW(TAG, "Volume de purga na NVS fora da faixa, mantendo %.2f m3", purge_volume_m3);
    }
    err = nvs_get_u32(handle, "num_cycles", &cycles_day);
    if (err == ESP_OK && !set_num_cycles_per_day((int32_t)cycles_day)) {
        ESP_LOGW(TAG, "Ciclos/dia na NVS fora da faixa, mantendo %d", (int)num_cycles_per_day);
    }
    err = nvs_get_u32(handle, "purge_cyc_min", &purge_min);
    if (err == ESP_OK && !set_purge_cycle_minutes((int32_t)purge_min)) {
        ESP_LOGW(TAG, "Tempo de purga na NVS fora da faixa, mantendo %d min", (int)purge_cycle_minutes);
    }
    err = nvs_get_u8(handle, "auto_en", &auto_en);
    if (err == ESP_OK) {
        // Nao entra em ST_START_FILL aqui diretamente - so restaura a flag.
        // O loop de automacao (automation_task), ao ver auto_enabled=true
        // com auto_state=AUTO_OFF, dispara um Start normal sozinho.
        auto_enabled = (auto_en != 0);
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "NVS carregada: ciclos=%lu, agua=%.1f m3, tanque1=%.2f m3, tanque2=%.2f m3, "
             "purga=%.2f m3, ciclos/dia=%d, purga ciclo=%d min, automatico=%s",
             (unsigned long)total_cycles, total_water_in_m3, tank1_volume_m3, tank2_volume_m3,
             purge_volume_m3, (int)num_cycles_per_day, (int)purge_cycle_minutes,
             auto_enabled ? "estava LIGADO" : "estava desligado");
}

static void save_totals_to_nvs(void)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open("qqwater", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao abrir NVS para gravar totalizadores: %s", esp_err_to_name(err));
        return;
    }

    uint32_t water_x10 = (uint32_t)(total_water_in_m3 * 10.0 + 0.5);
    err = nvs_set_u32(handle, "tot_cycles", total_cycles);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao salvar total_cycles na NVS: %s", esp_err_to_name(err));
    }
    err = nvs_set_u32(handle, "tot_water_x10", water_x10);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao salvar total_water_x10 na NVS: %s", esp_err_to_name(err));
    }
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao confirmar gravacao da NVS: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
}

// Grava so a flag de liga/desliga do automatico (chamada com mais
// frequencia que save_totals_to_nvs, entao fica separada).
static void save_auto_enabled_to_nvs(bool en)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open("qqwater", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao abrir NVS para gravar auto_enabled: %s", esp_err_to_name(err));
        return;
    }
    nvs_set_u8(handle, "auto_en", en ? 1 : 0);
    nvs_commit(handle);
    nvs_close(handle);
}

// Grava todos os parametros ajustaveis pelo app: volumes dos tanques,
// volume de purga, ciclos/dia e minutos de purga do ciclo. So e chamada
// quando o usuario muda algo na pagina, entao nao ha desgaste de flash.
static void save_settings_to_nvs(void)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open("qqwater", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao abrir NVS para gravar configuracoes: %s", esp_err_to_name(err));
        return;
    }
    nvs_set_u32(handle, "tank1_vol_x100", (uint32_t)(tank1_volume_m3 * 100.0 + 0.5));
    nvs_set_u32(handle, "tank2_vol_x100", (uint32_t)(tank2_volume_m3 * 100.0 + 0.5));
    nvs_set_u32(handle, "purge_vol_x100", (uint32_t)(purge_volume_m3 * 100.0 + 0.5));
    nvs_set_u32(handle, "num_cycles",     (uint32_t)num_cycles_per_day);
    nvs_set_u32(handle, "purge_cyc_min",  (uint32_t)purge_cycle_minutes);
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao confirmar gravacao das configuracoes: %s", esp_err_to_name(err));
    }
    nvs_close(handle);
}

// ---------------------------------------------------------------------
// Persistencia do estado de espera (retomada apos queda de energia)
// ---------------------------------------------------------------------
// Chaves adicionais:
//   st_state     u8   - estado da automacao (AUTO_STATE_NVS_NONE = nada a retomar)
//   st_remain_s  u32  - segundos que ainda faltam naquele estado
//   st_total_s   u32  - duracao total do estado (usada pelo timer do app)
#define AUTO_STATE_NVS_NONE 0xFF

static void persist_auto_state(void)
{
    last_state_save_us = esp_timer_get_time();

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open("qqwater", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao abrir NVS para gravar o estado: %s", esp_err_to_name(err));
        return;
    }
    nvs_set_u8(handle, "st_state", (uint8_t)auto_state);
    nvs_set_u32(handle, "st_remain_s", (uint32_t)(state_remaining_ms() / 1000));
    nvs_set_u32(handle, "st_total_s", (uint32_t)(state_total_ms() / 1000));
    nvs_commit(handle);
    nvs_close(handle);
}

static void clear_persisted_auto_state(void)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open("qqwater", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao abrir NVS para limpar o estado: %s", esp_err_to_name(err));
        return;
    }
    nvs_set_u8(handle, "st_state", AUTO_STATE_NVS_NONE);
    nvs_commit(handle);
    nvs_close(handle);
}

// Chamada uma unica vez, no inicio da automation_task. Retorna true se
// conseguiu retomar o estado salvo (e nesse caso NAO chama enter_state -
// os reles ja estao todos desligados pelo boot, que e exatamente o
// hardware correto para os estados de espera).
static bool restore_auto_state_from_nvs(void)
{
    if (!auto_enabled) {
        return false;   // automatico estava desligado: nada a retomar
    }

    nvs_handle_t handle = 0;
    if (nvs_open("qqwater", NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }

    uint8_t  state_id = AUTO_STATE_NVS_NONE;
    uint32_t remain_s = 0;
    uint32_t total_s  = 0;
    bool have_state = (nvs_get_u8(handle, "st_state", &state_id) == ESP_OK);
    if (have_state) {
        nvs_get_u32(handle, "st_remain_s", &remain_s);
        nvs_get_u32(handle, "st_total_s", &total_s);
    }
    nvs_close(handle);

    if (!have_state || state_id == AUTO_STATE_NVS_NONE) {
        return false;
    }

    auto_state_t s = (auto_state_t)state_id;
    if (!state_is_resumable(s)) {
        return false;   // NVS de versao antiga ou valor inesperado
    }

    // Conferencia contra as boias: o estado salvo pressupoe tanque cheio.
    // Se a energia ficou fora tempo suficiente para alguem esvaziar (ou se
    // houve manutencao), a premissa nao vale mais e e mais seguro comecar
    // do zero do que purgar/drenar um tanque que nao esta onde se pensa.
    bool tanks_ok;
    switch (s) {
        case ST_START_DECANT:
        case A_DECANT:
            tanks_ok = tank_high(1);
            break;
        case B_DECANT:
            tanks_ok = tank_high(2);
            break;
        default:    // ST_START_WAIT, A_WAIT, B_WAIT: os dois tanques cheios
            tanks_ok = tank_high(1) && tank_high(2);
            break;
    }
    if (!tanks_ok) {
        ESP_LOGW(TAG, "Estado %d salvo na NVS, mas as boias nao confirmam tanque cheio - "
                      "iniciando um Start normal", (int)s);
        return false;
    }

    int64_t now = esp_timer_get_time();
    int64_t remain_ms = (int64_t)remain_s * 1000;

    switch (s) {
        case ST_START_DECANT:
        case A_DECANT:
        case B_DECANT:
            // Decantacao tem duracao fixa: recoloca o relogio de entrada no
            // estado de modo que "falta remain_ms" continue valendo.
            if (remain_ms > DECANT_MS) remain_ms = DECANT_MS;
            state_enter_time_us = now - (DECANT_MS - remain_ms) * 1000;
            break;
        default:
            // Esperas de janela de ciclo sao regidas pelo watchdog, entao o
            // que se restaura e o prazo, nao o instante de entrada.
            watchdog_total_ms = (int64_t)total_s * 1000;
            if (watchdog_total_ms <= 0) watchdog_total_ms = get_cycle_watchdog_ms();
            if (remain_ms > watchdog_total_ms) remain_ms = watchdog_total_ms;
            watchdog_deadline_us = now + remain_ms * 1000;
            state_enter_time_us = now;
            break;
    }

    auto_state = s;
    fault_msg[0] = '\0';
    last_state_save_us = now;
    ESP_LOGW(TAG, "Retomando o estado %d apos reinicio - faltam %d min de espera",
             (int)s, (int)(remain_ms / 60000));
    return true;
}

// Usado no toggle manual e sempre que o sistema muda auto_enabled sozinho
// (fim de sequencia de parada, falha). Mantem a flag persistida em dia.
static void set_auto_enabled(bool en)
{
    auto_enabled = en;
    save_auto_enabled_to_nvs(en);
}

static void reset_totals(void)
{
    total_cycles = 0;
    total_water_in_m3 = 0.0;
    save_totals_to_nvs();
    ESP_LOGI(TAG, "Totalizadores zerados");
}

static void automation_task(void *arg)
{
    static bool prev_auto_enabled = false;

    // Espera as boias passarem pelo debounce antes de decidir se da para
    // retomar o estado salvo - a conferencia de tanque cheio depende delas.
    vTaskDelay(pdMS_TO_TICKS(BOIA_POLL_PERIOD_MS * (BOIA_DEBOUNCE_THRESHOLD + 3)));
    if (restore_auto_state_from_nvs()) {
        // Impede que o loop abaixo interprete auto_enabled=true como
        // "acabou de ligar o toggle" e dispare um Start por cima.
        prev_auto_enabled = true;
    }

    while (1) {
        check_overpressure();

        // Salva o progresso das esperas de tempos em tempos, para o proximo
        // boot saber de onde continuar.
        if (state_is_resumable(auto_state) &&
            (esp_timer_get_time() - last_state_save_us) >= STATE_SAVE_PERIOD_US) {
            persist_auto_state();
        }

        // Atraso centralizado de ligar a B2 (agendado em schedule_b2_start()).
        // Um unico ponto de checagem, nao-bloqueante, independente do estado atual.
        if (b2_start_pending && esp_timer_get_time() >= b2_start_deadline_us) {
            b2_start_pending = false;
            relay_write(IDX_B2, true);
        }

        if (skip_requested) {
            skip_requested = false;
            // "Pular" so acelera o relogio do timer atual (deixa ~2s restando).
            // Nunca pula a checagem de boias/sensores nem o watchdog de seguranca.
            int64_t total = state_total_ms();
            if (total > 0) {
                int64_t fast_forward_ms = total - 2000;
                if (fast_forward_ms < 0) fast_forward_ms = 0;
                state_enter_time_us = esp_timer_get_time() - (fast_forward_ms * 1000);
                switch (auto_state) {
                    case ST_START_WAIT:
                    case A_WAIT:
                    case B_WAIT:
                        watchdog_deadline_us = esp_timer_get_time() + 2000LL * 1000;
                        break;
                    default:
                        break;
                }
            }
        }

        if (stop_requested) {
            if (auto_state != STOP_PURGE && auto_state != STOP_DRAIN && auto_state != STOP_DONE) {
                all_relays_off();
                enter_state(STOP_PURGE);
            }
        } else {
            if (prev_auto_enabled && !auto_enabled) {
                // Toggle desligado direto: corte imediato, sem sequencia de esvaziar.
                all_relays_off();
                enter_state(AUTO_OFF);
            } else if (!prev_auto_enabled && auto_enabled && auto_state == AUTO_OFF) {
                enter_state(ST_START_FILL);
            }
        }
        prev_auto_enabled = auto_enabled;

        switch (auto_state) {
            case AUTO_OFF:
                break;
            case ST_START_FILL:
                if (start_fill_already_full) {
                    enter_state(ST_START_DECANT);
                    break;
                }
                check_dryrun();
                check_watchdog_fault();
                if (tank_high(1)) {
                    total_water_in_m3 += tank1_volume_m3;
                    save_totals_to_nvs();
                    enter_state(ST_START_DECANT);
                }
                break;
            case ST_START_DECANT:
                if (elapsed_ms() >= DECANT_MS) enter_state(ST_START_FILL2);
                break;
            case ST_START_FILL2:
                if (start_fill_already_full) {
                    enter_state(ST_START_WAIT);
                    break;
                }
                check_dryrun();
                check_watchdog_fault();
                if (tank_high(2)) {
                    total_water_in_m3 += tank2_volume_m3;
                    save_totals_to_nvs();
                    enter_state(ST_START_WAIT);
                }
                break;
            case ST_START_WAIT:
                if (watchdog_expired()) enter_state(A_DECANT);
                break;
            case A_DECANT:
                if (elapsed_ms() >= DECANT_MS) enter_state(A_PURGE);
                break;
            case A_PURGE:
                check_watchdog_fault();
                if (elapsed_ms() >= purge_cycle_ms()) enter_state(A_DRAIN);
                break;
            case A_DRAIN:
                check_watchdog_fault();
                if (tank_low(1) || !sensor_available(MIDX_T1_BAIXO)) enter_state(A_FILL);
                break;
            case A_FILL:
                check_dryrun();
                check_watchdog_fault();
                if (tank_high(1)) {
                    total_water_in_m3 += tank1_volume_m3;
                    total_cycles++;
                    save_totals_to_nvs();
                    enter_state(A_WAIT);
                }
                break;
            case A_WAIT:
                if (watchdog_expired()) enter_state(B_DECANT);
                break;
            case B_DECANT:
                if (elapsed_ms() >= DECANT_MS) enter_state(B_PURGE);
                break;
            case B_PURGE:
                check_watchdog_fault();
                if (elapsed_ms() >= purge_cycle_ms()) enter_state(B_DRAIN);
                break;
            case B_DRAIN:
                check_watchdog_fault();
                if (tank_low(2) || !sensor_available(MIDX_T2_BAIXO)) enter_state(B_FILL);
                break;
            case B_FILL:
                check_dryrun();
                check_watchdog_fault();
                if (tank_high(2)) {
                    total_water_in_m3 += tank2_volume_m3;
                    total_cycles++;
                    save_totals_to_nvs();
                    enter_state(B_WAIT);
                }
                break;
            case B_WAIT:
                if (watchdog_expired()) enter_state(A_DECANT);
                break;
            case STOP_PURGE:
                check_watchdog_fault();
                if (elapsed_ms() >= purge_stop_ms()) enter_state(STOP_DRAIN);
                break;
            case STOP_DRAIN:
                check_watchdog_fault();
                if (tank_low(1) && tank_low(2)) enter_state(STOP_DONE);
                break;
            case STOP_DONE:
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ---------------------------------------------------------------------
// Diagnostico de rede
// ---------------------------------------------------------------------
// Loga entrada/saida de celulares no AP e o IP entregue pelo DHCP. Se um
// Samsung "conecta" mas nunca aparece a linha de IP atribuido, o problema
// e DHCP; se recebe IP e mesmo assim a pagina nao abre, e no HTTP/TCP.
static void wifi_diag_event_handler(void *arg, esp_event_base_t base,
                                    int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "[diag] cliente conectou: " MACSTR " (aid=%d)", MAC2STR(e->mac), e->aid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "[diag] cliente desconectou: " MACSTR " (motivo=%d)", MAC2STR(e->mac), e->reason);
    } else if (base == IP_EVENT && id == IP_EVENT_ASSIGNED_IP_TO_CLIENT) {
        ip_event_assigned_ip_to_client_t *e = (ip_event_assigned_ip_to_client_t *)data;
        ESP_LOGI(TAG, "[diag] DHCP entregou " IPSTR " para " MACSTR, IP2STR(&e->ip), MAC2STR(e->mac));
    }
}

// ---------------------------------------------------------------------
// Rede: Wi-Fi cliente (roteador) com fallback para o Access Point
// ---------------------------------------------------------------------
// Comportamento:
//   - Sem rede cadastrada: so o AP "QQWATER" (como sempre foi).
//   - Com rede cadastrada: no boot tenta conectar no roteador por ate 30 s.
//     Conseguiu -> fica so no roteador (AP desligado).
//     Falhou    -> liga o AP (APSTA) e tenta o roteador de novo a cada
//                  10 min; quando conseguir, mantem o AP por mais 60 s (para
//                  quem estiver nele ver o IP novo) e depois desliga o AP.
//   - Se a conexao com o roteador cair depois, o mesmo ciclo recomeca.
//
// As credenciais ficam na NVS do proprio firmware (chaves wifi_ssid /
// wifi_pass); o armazenamento interno do driver de Wi-Fi fica em RAM para
// nao existir uma segunda copia fora do nosso controle.
//
// No roteador o ESP32 aparece com o nome "qqwater". Vale reservar um IP
// fixo para ele no DHCP do roteador, para o endereco nao mudar.
#define AP_IP_ADDR_STR          "192.168.4.1"
#define STA_HOSTNAME            "qqwater"
#define STA_CONNECT_TIMEOUT_US  (30LL * 1000 * 1000)
#define STA_RETRY_PERIOD_US     (10LL * 60 * 1000 * 1000)
#define STA_ATTEMPT_GAP_US      (3LL * 1000 * 1000)
#define AP_GRACE_AFTER_STA_US   (60LL * 1000 * 1000)

typedef enum {
    NET_AP_ONLY = 0,      // sem rede cadastrada
    NET_STA_CONNECTING,   // tentando o roteador
    NET_STA_CONNECTED,    // conectado no roteador
    NET_AP_FALLBACK,      // falhou: AP ligado, aguardando proxima tentativa
} net_state_t;

static volatile net_state_t net_state = NET_AP_ONLY;
static volatile bool ap_active = false;
static char sta_ssid[33] = "";
static char sta_pass[65] = "";
static bool sta_has_creds = false;
static esp_netif_t *netif_sta = NULL;
static wifi_config_t ap_wifi_config;

// Sinais do handler de eventos para a net_task (quem decide e a task).
static volatile bool sta_busy = false;        // ha uma tentativa de conexao em curso
static volatile bool ev_got_ip = false;
static volatile bool ev_sta_lost = false;
static volatile uint32_t sta_ip_addr = 0;     // ordem de rede; 0 = sem IP
static volatile int last_disc_reason = 0;

// Pedidos vindos da pagina (executados pela net_task, fora do httpd).
static SemaphoreHandle_t net_req_mutex = NULL;
static bool req_save = false;
static bool req_forget = false;
static char req_ssid[33];
static char req_pass[65];

static int64_t sta_attempt_start_us = 0;
static int64_t sta_last_try_us = 0;
static int64_t sta_next_retry_us = 0;
static int64_t ap_off_at_us = 0;

static const char *net_state_text(void)
{
    switch (net_state) {
        case NET_AP_ONLY:        return "Somente AP (nenhuma rede cadastrada)";
        case NET_STA_CONNECTING: return "Conectando ao roteador...";
        case NET_STA_CONNECTED:  return "Conectado ao roteador";
        case NET_AP_FALLBACK:    return "Sem roteador - AP ativo, nova tentativa a cada 10 min";
        default:                 return "-";
    }
}

static void wifi_creds_load(void)
{
    nvs_handle_t h;
    if (nvs_open("qqwater", NVS_READONLY, &h) != ESP_OK) return;
    size_t l1 = sizeof(sta_ssid), l2 = sizeof(sta_pass);
    if (nvs_get_str(h, "wifi_ssid", sta_ssid, &l1) == ESP_OK && sta_ssid[0]) {
        if (nvs_get_str(h, "wifi_pass", sta_pass, &l2) != ESP_OK) sta_pass[0] = '\0';
        sta_has_creds = true;
    }
    nvs_close(h);
}

static void wifi_creds_store(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    if (nvs_open("qqwater", NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao abrir NVS para gravar credenciais Wi-Fi");
        return;
    }
    nvs_set_str(h, "wifi_ssid", ssid);
    nvs_set_str(h, "wifi_pass", pass);
    nvs_commit(h);
    nvs_close(h);
}

static void wifi_creds_erase(void)
{
    nvs_handle_t h;
    if (nvs_open("qqwater", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, "wifi_ssid");
    nvs_erase_key(h, "wifi_pass");
    nvs_commit(h);
    nvs_close(h);
}

// Liga/desliga o AP preservando o STA quando ha rede cadastrada.
static void net_set_ap(bool on)
{
    wifi_mode_t mode = on ? (sta_has_creds ? WIFI_MODE_APSTA : WIFI_MODE_AP) : WIFI_MODE_STA;
    esp_err_t err = esp_wifi_set_mode(mode);
    if (err == ESP_OK && on) {
        err = esp_wifi_set_config(WIFI_IF_AP, &ap_wifi_config);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao trocar modo Wi-Fi: %s", esp_err_to_name(err));
        return;
    }
    if (on != ap_active) {
        ESP_LOGI(TAG, "Access Point %s", on ? "LIGADO" : "DESLIGADO");
    }
    ap_active = on;
}

static void net_apply_sta_config(void)
{
    wifi_config_t sta_cfg = {0};
    strlcpy((char *)sta_cfg.sta.ssid, sta_ssid, sizeof(sta_cfg.sta.ssid));
    strlcpy((char *)sta_cfg.sta.password, sta_pass, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.threshold.authmode = sta_pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    sta_cfg.sta.pmf_cfg.capable = true;
    sta_cfg.sta.pmf_cfg.required = false;
    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao aplicar config do roteador: %s", esp_err_to_name(err));
    }
}

static void net_begin_attempt(void)
{
    net_state = NET_STA_CONNECTING;
    sta_attempt_start_us = esp_timer_get_time();
    sta_last_try_us = 0;
    ap_off_at_us = 0;
    ESP_LOGI(TAG, "Tentando conectar no roteador '%s'", sta_ssid);
}

// Interrompe uma tentativa/conexao em curso e espera o driver confirmar.
static void net_sta_stop(void)
{
    esp_wifi_disconnect();
    for (int i = 0; i < 20 && sta_busy; i++) vTaskDelay(pdMS_TO_TICKS(100));
    sta_busy = false;
    sta_ip_addr = 0;
}

static void net_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        last_disc_reason = e->reason;
        bool had_ip = (sta_ip_addr != 0);
        sta_busy = false;
        sta_ip_addr = 0;
        if (had_ip) ev_sta_lost = true;
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        sta_ip_addr = e->ip_info.ip.addr;
        sta_busy = false;
        ev_got_ip = true;
        ESP_LOGI(TAG, "Conectado ao roteador, IP " IPSTR, IP2STR(&e->ip_info.ip));
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        sta_ip_addr = 0;
        ev_sta_lost = true;
    }
}

static void net_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        int64_t now = esp_timer_get_time();

        // --- Pedidos da pagina ---
        bool do_save = false, do_forget = false;
        char new_ssid[33], new_pass[65];
        xSemaphoreTake(net_req_mutex, portMAX_DELAY);
        if (req_forget) { do_forget = true; req_forget = false; }
        if (req_save) {
            do_save = true; req_save = false;
            strlcpy(new_ssid, req_ssid, sizeof(new_ssid));
            strlcpy(new_pass, req_pass, sizeof(new_pass));
            memset(req_pass, 0, sizeof(req_pass));
        }
        xSemaphoreGive(net_req_mutex);

        if (do_forget) {
            ESP_LOGI(TAG, "Rede Wi-Fi esquecida - voltando para somente AP");
            net_state = NET_AP_ONLY;
            if (sta_has_creds) net_sta_stop();
            wifi_creds_erase();
            sta_has_creds = false;
            sta_ssid[0] = sta_pass[0] = '\0';
            net_set_ap(true);
            ev_got_ip = ev_sta_lost = false;
            continue;
        }
        if (do_save) {
            ESP_LOGI(TAG, "Nova rede Wi-Fi cadastrada: '%s'", new_ssid);
            net_state = NET_STA_CONNECTING;   // impede o fallback de agir no meio da troca
            if (sta_has_creds) net_sta_stop();
            wifi_creds_store(new_ssid, new_pass);
            strlcpy(sta_ssid, new_ssid, sizeof(sta_ssid));
            strlcpy(sta_pass, new_pass, sizeof(sta_pass));
            memset(new_pass, 0, sizeof(new_pass));
            sta_has_creds = true;
            // Quem cadastrou provavelmente esta no AP: mantem o AP ligado
            // (agora em APSTA) para acompanhar o resultado pela pagina.
            net_set_ap(ap_active);
            net_apply_sta_config();
            ev_got_ip = ev_sta_lost = false;
            net_begin_attempt();
            continue;
        }

        // --- Eventos do driver ---
        if (ev_got_ip) {
            ev_got_ip = false;
            net_state = NET_STA_CONNECTED;
            if (ap_active) ap_off_at_us = now + AP_GRACE_AFTER_STA_US;
        }
        if (ev_sta_lost) {
            ev_sta_lost = false;
            if (net_state == NET_STA_CONNECTED) {
                ESP_LOGW(TAG, "Conexao com o roteador perdida (motivo=%d)", last_disc_reason);
                net_begin_attempt();
            }
        }

        // --- Maquina de estados ---
        switch (net_state) {
            case NET_AP_ONLY:
                break;
            case NET_STA_CONNECTING:
                if (now - sta_attempt_start_us >= STA_CONNECT_TIMEOUT_US) {
                    ESP_LOGW(TAG, "Roteador '%s' indisponivel (motivo=%d) - AP ligado, "
                                  "nova tentativa em 10 min", sta_ssid, last_disc_reason);
                    net_state = NET_AP_FALLBACK;
                    net_sta_stop();
                    if (!ap_active) net_set_ap(true);
                    sta_next_retry_us = now + STA_RETRY_PERIOD_US;
                } else if (!sta_busy && (now - sta_last_try_us) >= STA_ATTEMPT_GAP_US) {
                    sta_last_try_us = now;
                    // Marca antes de chamar: o evento de falha pode chegar
                    // antes do esp_wifi_connect() retornar.
                    sta_busy = true;
                    if (esp_wifi_connect() != ESP_OK) sta_busy = false;
                }
                break;
            case NET_STA_CONNECTED:
                if (ap_active && ap_off_at_us != 0 && now >= ap_off_at_us) {
                    ap_off_at_us = 0;
                    net_set_ap(false);
                }
                break;
            case NET_AP_FALLBACK:
                if (now >= sta_next_retry_us) net_begin_attempt();
                break;
        }
    }
}

// Pedidos feitos pelos handlers HTTP. Retornam na hora; a net_task executa.
static bool net_request_save(const char *ssid, const char *pass)
{
    size_t ls = strlen(ssid), lp = strlen(pass);
    if (ls == 0 || ls > 32 || lp > 64 || (lp > 0 && lp < 8)) return false;
    xSemaphoreTake(net_req_mutex, portMAX_DELAY);
    strlcpy(req_ssid, ssid, sizeof(req_ssid));
    strlcpy(req_pass, pass, sizeof(req_pass));
    req_save = true;
    xSemaphoreGive(net_req_mutex);
    return true;
}

static void net_request_forget(void)
{
    xSemaphoreTake(net_req_mutex, portMAX_DELAY);
    req_forget = true;
    xSemaphoreGive(net_req_mutex);
}

static void net_init(void)
{
    net_req_mutex = xSemaphoreCreateMutex();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    netif_sta = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(netif_sta, STA_HOSTNAME);

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED,
                                               wifi_diag_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED,
                                               wifi_diag_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ASSIGNED_IP_TO_CLIENT,
                                               wifi_diag_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                               net_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               net_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP,
                                               net_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    memset(&ap_wifi_config, 0, sizeof(ap_wifi_config));
    strlcpy((char *)ap_wifi_config.ap.ssid, AP_SSID, sizeof(ap_wifi_config.ap.ssid));
    strlcpy((char *)ap_wifi_config.ap.password, AP_PASS, sizeof(ap_wifi_config.ap.password));
    ap_wifi_config.ap.ssid_len = strlen(AP_SSID);
    ap_wifi_config.ap.channel = AP_CHANNEL;
    ap_wifi_config.ap.max_connection = AP_MAX_CONN;
    ap_wifi_config.ap.authmode = strlen(AP_PASS) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    wifi_creds_load();
    if (sta_has_creds) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        net_apply_sta_config();
        ESP_ERROR_CHECK(esp_wifi_start());
        ap_active = false;
        net_begin_attempt();
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        ap_active = true;
        net_state = NET_AP_ONLY;
        ESP_LOGI(TAG, "Access Point iniciado. SSID:%s canal:%d", AP_SSID, AP_CHANNEL);
    }

    xTaskCreate(net_task, "net_task", 4096, NULL, 4, NULL);
}

// ---------------------------------------------------------------------
// Servidor DNS (captive portal) - responde qualquer consulta com o IP
// do proprio ESP32, fazendo o celular achar que precisa "fazer login".
// ---------------------------------------------------------------------
#define DNS_PORT 53
#define DNS_MAX_LEN 512
// Tamanho do registro de resposta que anexamos: ponteiro de nome (2) +
// TYPE (2) + CLASS (2) + TTL (4) + RDLENGTH (2) + IPv4 (4) = 16 bytes.
#define DNS_ANSWER_LEN 16

static void dns_server_task(void *arg)
{
    uint8_t rx_buffer[DNS_MAX_LEN];
    // Com folga para o registro anexado: mesmo uma pergunta do tamanho
    // maximo cabe aqui depois da resposta ser montada.
    uint8_t response[DNS_MAX_LEN + DNS_ANSWER_LEN];

    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DNS_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "DNS: falha ao criar socket");
        vTaskDelete(NULL);
        return;
    }
    if (bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
        ESP_LOGE(TAG, "DNS: falha no bind da porta 53");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Servidor DNS (captive portal) rodando na porta 53");

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0,
                            (struct sockaddr *)&source_addr, &socklen);

        // --- Validacoes de entrada (pacote nao confiavel) ---
        if (len < 12 || len > DNS_MAX_LEN) continue;  // cabecalho DNS minimo / tamanho maximo
        if (rx_buffer[2] & 0x80) continue;            // ja e uma resposta, ignora
        if ((rx_buffer[2] >> 3) & 0x0F) continue;     // opcode != query padrao
        if (rx_buffer[4] != 0 || rx_buffer[5] != 1) continue;  // QDCOUNT precisa ser exatamente 1

        // Percorre o QNAME para achar o fim da secao de pergunta. Cortar o
        // pacote aqui descarta qualquer registro adicional (EDNS/OPT) que o
        // cliente tenha mandado - senao o registro de resposta ficaria
        // depois do OPT e o celular descartaria o pacote como malformado.
        int qend = 12;
        bool name_ok = false;
        while (qend < len) {
            uint8_t label_len = rx_buffer[qend];
            if (label_len == 0) {          // fim do nome
                qend += 1;
                name_ok = true;
                break;
            }
            if (label_len & 0xC0) break;   // ponteiro de compressao: invalido na pergunta
            qend += label_len + 1;
        }
        if (!name_ok || qend + 4 > len) continue;  // QNAME truncado ou sem QTYPE/QCLASS
        uint16_t qtype  = ((uint16_t)rx_buffer[qend]     << 8) | rx_buffer[qend + 1];
        uint16_t qclass = ((uint16_t)rx_buffer[qend + 2] << 8) | rx_buffer[qend + 3];
        qend += 4;                                 // inclui QTYPE (2) + QCLASS (2)

        // So respondemos com endereco consultas do tipo A (IPv4), classe IN.
        // Para qualquer outra (AAAA, HTTPS/SVCB, etc.) devolvemos NOERROR sem
        // registro ("esse nome existe, mas nao tem esse tipo"). Antes o
        // servidor mandava um registro A como resposta a uma pergunta AAAA, o
        // que e invalido e pode fazer o resolvedor do Android (Samsung em
        // particular) descartar a resposta e nao detectar o portal.
        bool answer_a = (qtype == 1 && qclass == 1);

        // --- Monta a resposta: cabecalho + pergunta (+ 1 registro A) ---
        int resp_len = qend;                       // no maximo DNS_MAX_LEN
        memcpy(response, rx_buffer, qend);

        response[2] = 0x81;  // QR=1 (resposta), opcode=0, sem truncamento, RD preservado
        response[3] = 0x80;  // recursao disponivel, RCODE=0
        response[6] = 0x00; response[7] = answer_a ? 0x01 : 0x00;  // ANCOUNT
        response[8] = 0x00; response[9] = 0x00;  // NSCOUNT = 0
        response[10] = 0x00; response[11] = 0x00; // ARCOUNT = 0

        if (!answer_a) {
            sendto(sock, response, resp_len, 0, (struct sockaddr *)&source_addr, socklen);
            continue;
        }

        response[resp_len++] = 0xC0; response[resp_len++] = 0x0C; // ponteiro pro nome perguntado
        response[resp_len++] = 0x00; response[resp_len++] = 0x01; // TYPE A
        response[resp_len++] = 0x00; response[resp_len++] = 0x01; // CLASS IN
        response[resp_len++] = 0x00; response[resp_len++] = 0x00;
        response[resp_len++] = 0x00; response[resp_len++] = 0x3C; // TTL 60s
        response[resp_len++] = 0x00; response[resp_len++] = 0x04; // RDLENGTH = 4 bytes
        response[resp_len++] = 192;  response[resp_len++] = 168;
        response[resp_len++] = 4;    response[resp_len++] = 1;    // 192.168.4.1

        sendto(sock, response, resp_len, 0, (struct sockaddr *)&source_addr, socklen);
    }
}

// ---------------------------------------------------------------------
// Pagina HTML (embutida no firmware)
// ---------------------------------------------------------------------
static const char index_html[] =
"<!DOCTYPE html><html lang='pt-br'><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<title>qqWater - Controle do Poco</title>"
"<style>"
"body{font-family:Arial,sans-serif;background:#0f1720;color:#e8eef5;margin:0;padding:16px;}"
"h1{font-size:20px;text-align:center;margin-bottom:16px;}"
".banner{max-width:420px;margin:0 auto 6px;padding:14px;border-radius:12px;background:#1a232e;"
"text-align:center;font-size:14px;font-weight:bold;}"
".banner.fault{background:#5c1f1f;color:#ffb3b3;}"
".timer{max-width:420px;margin:0 auto 16px;text-align:center;font-size:22px;font-weight:bold;"
"color:#e8eef5;letter-spacing:1px;}"
".timer.hidden{display:none;}"
".autobar{max-width:420px;margin:0 auto 10px;display:flex;flex-wrap:wrap;align-items:center;gap:10px;}"
".autobar-left{display:flex;align-items:center;gap:8px;flex:1 1 100%;}"
".switch{position:relative;width:46px;height:26px;flex-shrink:0;}"
".switch input{opacity:0;width:0;height:0;}"
".slider{position:absolute;inset:0;background:#26313f;border-radius:26px;cursor:pointer;transition:.2s;}"
".slider:before{content:'';position:absolute;width:20px;height:20px;left:3px;top:3px;background:#e8eef5;"
"border-radius:50%;transition:.2s;}"
"input:checked + .slider{background:#1f9d55;}"
"input:checked + .slider:before{transform:translateX(20px);}"
".autobar button{flex:1;padding:12px 10px;font-size:13px;border:none;border-radius:8px;cursor:pointer;color:#fff;white-space:nowrap;}"
"#stopBtn{background:#712b13;justify-self:end;}"
"#skipBtn{background:#3c3489;justify-self:end;}"
".cyclesbar, .purgebar{max-width:420px;margin:0 auto 14px;display:grid;grid-template-columns:125px 60px 1fr;align-items:center;gap:10px;font-size:13px;}"
".cyclesbar input, .purgebar input{width:100%;box-sizing:border-box;padding:6px;border-radius:6px;border:none;background:#26313f;color:#e8eef5;font-size:14px;text-align:center;}"
".limitwarn{max-width:420px;margin:0 auto 14px;padding:10px;border-radius:8px;background:#5c4a1f;"
"color:#ffe5b3;font-size:12px;text-align:center;}"
".limitwarn.hidden{display:none;}"
".totalsbar{max-width:420px;margin:0 auto 14px;display:grid;grid-template-columns:repeat(2, minmax(0, 1fr));gap:10px;}"
".totalbox{padding:12px;border-radius:10px;background:#1a232e;text-align:center;min-width:0;display:flex;flex-direction:column;justify-content:center;align-items:center;gap:4px;}"
".totalbox b{display:block;font-size:20px;color:#e8eef5;line-height:1.1;}"
".totalbox span{font-size:11px;color:#8ea0b3;line-height:1.2;display:block;word-break:break-word;}"
"#resetTotalsBtn{width:100%;margin-top:8px;padding:8px;font-size:12px;background:#26313f;color:#8ea0b3;"
"border:none;border-radius:8px;cursor:pointer;}"
".grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px;max-width:420px;margin:16px auto;}"
"button{padding:22px 8px;font-size:14px;border:none;border-radius:12px;background:#26313f;color:#e8eef5;"
"cursor:pointer;transition:background .15s;min-height:62px;display:flex;align-items:center;justify-content:center;}"
"button.on{background:#1f9d55;color:#fff;}"
"button:active{opacity:.8;}"
"button:disabled{opacity:.35;cursor:not-allowed;}"
".status{text-align:center;margin-top:18px;font-size:12px;color:#8ea0b3;}"
"h2{font-size:14px;color:#8ea0b3;margin:26px auto 10px;max-width:420px;display:flex;"
"justify-content:space-between;align-items:center;}"
"h2 .dbg{font-size:12px;display:flex;align-items:center;gap:6px;color:#8ea0b3;font-weight:normal;}"
"h2 .dbg .switch{width:34px;height:20px;}"
"h2 .dbg .slider:before{width:14px;height:14px;left:3px;top:3px;}"
"h2 .dbg input:checked + .slider:before{transform:translateX(14px);}"
".inputs{display:grid;grid-template-columns:repeat(2,1fr);gap:10px;max-width:420px;margin:0 auto;}"
".pill{padding:10px 12px;border-radius:10px;background:#1a232e;font-size:13px;display:flex;"
"justify-content:space-between;align-items:center;gap:8px;}"
".pill input{width:18px;height:18px;flex-shrink:0;}"
".topbar{max-width:420px;margin:0 auto;display:flex;justify-content:flex-end;}"
".topbar a{font-size:12px;color:#8ea0b3;text-decoration:none;padding:4px 0;}"
".flows{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px;max-width:420px;margin:0 auto;}"
".fcard{padding:12px;border-radius:10px;background:#1a232e;min-width:0;}"
".fcard .t{font-size:12px;color:#8ea0b3;}"
".fcard .v{font-size:22px;font-weight:bold;margin:6px 0 2px;}"
".fcard .v small{font-size:12px;color:#8ea0b3;font-weight:normal;}"
".fcard .tot{font-size:13px;}"
".fcard.off{opacity:.5;}"
"details{margin-top:8px;font-size:12px;color:#8ea0b3;}"
"summary{cursor:pointer;}"
".form label{display:block;margin:8px 0 4px;font-size:12px;color:#8ea0b3;}"
".form input{width:100%;box-sizing:border-box;padding:8px;border-radius:6px;border:none;background:#26313f;"
"color:#e8eef5;font-size:14px;}"
".form button{min-height:0;width:100%;margin-top:10px;padding:10px;font-size:13px;border-radius:8px;}"
".btn-ok{background:#1f9d55;color:#fff;}"
".btn-warn{background:#5c1f1f;color:#ffb3b3;}"
".box{max-width:420px;margin:0 auto;padding:12px;border-radius:10px;background:#1a232e;box-sizing:border-box;}"
".wst{font-size:13px;line-height:1.5;}"
".wst b{color:#e8eef5;}"
".msg{font-size:12px;min-height:16px;margin-top:8px;color:#8ea0b3;}"
"</style></head><body>"
"<div class='topbar'><a href='/logout'>Sair</a></div>"
"<h1>Controle do Poco - qqWater</h1>"
"<div class='banner' id='banner'>carregando...</div>"
"<div class='timer hidden' id='timer'>--:--:--</div>"
"<div class='autobar'>"
"<div class='autobar-left'>"
"<label class='switch'><input type='checkbox' id='autoToggle' onchange='toggleAuto(this.checked)'>"
"<span class='slider'></span></label>"
"<span>Modo Automatico</span>"
"</div>"
"<button id='skipBtn' onclick='skipStep()'>Pular Etapa</button>"
"<button id='stopBtn' onclick='stopSystem()'>Parar Sistema</button>"
"</div>"
"<div class='cyclesbar'>"
"<span>Ciclos por dia:</span>"
"<input type='number' id='cyclesInput' min='1' max='28' value='9' onchange='setCycles(this.value)'>"
"<span id='cyclesInfo'></span>"
"</div>"
"<div class='limitwarn hidden' id='limitWarn'>Producao estimada acima do limite diario de 18 m3!</div>"
"<div class='purgebar'>"
"<span>Purga ciclo (min):</span>"
"<input type='number' id='purgeCycleInput' min='1' max='60' value='5' onchange='setPurgeCycle(this.value)'>"
"<span></span>"
"</div>"
"<div class='purgebar'>"
"<span>Volume tanque 1 (m3):</span>"
"<input type='number' id='tank1VolInput' min='0.1' max='10' step='0.01' value='1.6' onchange='setTankVolume(1,this.value)'>"
"<span></span>"
"</div>"
"<div class='purgebar'>"
"<span>Volume tanque 2 (m3):</span>"
"<input type='number' id='tank2VolInput' min='0.1' max='10' step='0.01' value='1.6' onchange='setTankVolume(2,this.value)'>"
"<span></span>"
"</div>"
"<div class='purgebar'>"
"<span>Volume de purga (m3):</span>"
"<input type='number' id='purgeVolInput' min='0' max='5' step='0.01' value='0.1' onchange='setPurgeVolume(this.value)'>"
"<span id='producedInfo'></span>"
"</div>"
"<div class='totalsbar'>"
"<div class='totalbox'><b id='totalCycles'>0</b><span>ciclos completos</span></div>"
"<div class='totalbox'><b id='totalWater'>0.0</b><span>m3 (estimado) que entraram</span></div>"
"</div>"
"<button id='resetTotalsBtn' onclick='resetTotals()'>Zerar Totalizadores</button>"
"<h2><span>Medicao de vazao</span></h2>"
"<div class='flows' id='flows'></div>"
"<div class='grid' id='grid'></div>"
"<h2><span>Sensores</span><span class='dbg'><label class='switch' style='width:34px;height:20px'>"
"<input type='checkbox' id='debugToggle' onchange='toggleDebug(this.checked)'>"
"<span class='slider'></span></label>Modo Debug</span></h2>"
"<div class='inputs' id='inputs'></div>"
"<h2><span>Rede Wi-Fi</span></h2>"
"<div class='box'>"
"<div class='wst' id='wifiState'>carregando...</div>"
"<details id='wifiDet'><summary>Configurar rede</summary>"
"<div class='form'>"
"<label for='wSsid'>Nome da rede (SSID)</label><input id='wSsid' maxlength='32' autocomplete='off'>"
"<label for='wPass'>Senha</label><input id='wPass' type='password' maxlength='64' autocomplete='new-password'>"
"<button class='btn-ok' onclick='wifiSave()'>Salvar e conectar</button>"
"<button class='btn-warn' onclick='wifiForget()'>Esquecer rede (somente AP)</button>"
"<div class='msg' id='wifiMsg'></div>"
"</div></details>"
"</div>"
"<div class='status' id='status'>conectando...</div>"
"<script>"
"async function api(u,o){"
"  const r=await fetch(u,o);"
"  if(r.status===401){ location.href='/login'; throw new Error('login'); }"
"  return r;"
"}"
"const N=8;"
"const grid=document.getElementById('grid');"
"const inputsDiv=document.getElementById('inputs');"
"const st=document.getElementById('status');"
"const banner=document.getElementById('banner');"
"const timerEl=document.getElementById('timer');"
"const autoToggle=document.getElementById('autoToggle');"
"const debugToggle=document.getElementById('debugToggle');"
"const stopBtn=document.getElementById('stopBtn');"
"const skipBtn=document.getElementById('skipBtn');"
"const cyclesInput=document.getElementById('cyclesInput');"
"const cyclesInfo=document.getElementById('cyclesInfo');"
"const limitWarn=document.getElementById('limitWarn');"
"const purgeCycleInput=document.getElementById('purgeCycleInput');"
"const tank1VolInput=document.getElementById('tank1VolInput');"
"const tank2VolInput=document.getElementById('tank2VolInput');"
"const purgeVolInput=document.getElementById('purgeVolInput');"
"const producedInfo=document.getElementById('producedInfo');"
"const totalCyclesEl=document.getElementById('totalCycles');"
"const totalWaterEl=document.getElementById('totalWater');"
"let buttons=[];"
"let checks=[];"
"let remainingMs=0, lastFetch=0, totalMs=0;"
"for(let i=0;i<N;i++){"
"  const b=document.createElement('button');"
"  b.textContent='Canal '+(i+1);"
"  b.onclick=()=>toggle(i);"
"  grid.appendChild(b);"
"  buttons.push(b);"
"}"
"async function toggle(ch){"
"  const cur=buttons[ch].classList.contains('on');"
"  await api('/api/relay?ch='+ch+'&state='+(cur?0:1));"
"  refresh();"
"}"
"async function toggleAuto(checked){"
"  await api('/api/auto?state='+(checked?1:0));"
"  refresh();"
"}"
"async function toggleDebug(checked){"
"  await api('/api/debug/mode?state='+(checked?1:0));"
"  refresh();"
"}"
"async function stopSystem(){"
"  await api('/api/auto/stop');"
"  refresh();"
"}"
"async function skipStep(){"
"  await api('/api/auto/skip');"
"  refresh();"
"}"
"async function setCycles(n){"
"  await api('/api/config/cycles?n='+n);"
"  refresh();"
"}"
"async function setPurgeCycle(min){"
"  await api('/api/config/purge_cycle?min='+min);"
"  refresh();"
"}"
"async function setTankVolume(tank,m3){"
"  await api('/api/config/tank_volume?tank='+tank+'&m3='+m3);"
"  refresh();"
"}"
"async function setPurgeVolume(m3){"
"  await api('/api/config/purge_volume?m3='+m3);"
"  refresh();"
"}"
"async function resetTotals(){"
"  await api('/api/totals/reset');"
"  refresh();"
"}"
"async function toggleSensor(ch,checked){"
"  await api('/api/debug/set?ch='+ch+'&state='+(checked?1:0));"
"}"
"function fmtTime(ms){"
"  if(ms<0) ms=0;"
"  const s=Math.floor(ms/1000);"
"  const hh=String(Math.floor(s/3600)).padStart(2,'0');"
"  const mm=String(Math.floor((s%3600)/60)).padStart(2,'0');"
"  const ss=String(s%60).padStart(2,'0');"
"  return hh+':'+mm+':'+ss;"
"}"
"function tickTimer(){"
"  if(totalMs<=0){ timerEl.classList.add('hidden'); return; }"
"  timerEl.classList.remove('hidden');"
"  const elapsedSinceFetch=Date.now()-lastFetch;"
"  timerEl.textContent=fmtTime(remainingMs-elapsedSinceFetch);"
"}"
"async function refresh(){"
"  try{"
"    const r=await api('/api/status');"
"    const d=await r.json();"
"    for(let i=0;i<N;i++){"
"      buttons[i].textContent=d.labels[i]+(d.state[i]?' - ON':' - OFF');"
"      buttons[i].classList.toggle('on',!!d.state[i]);"
"      buttons[i].disabled=!!d.auto_locked;"
"    }"
"    if(checks.length===0 && d.in_labels){"
"      for(let i=0;i<d.in_labels.length;i++){"
"        const p=document.createElement('label');"
"        p.className='pill';"
"        const span=document.createElement('span');"
"        span.textContent=d.in_labels[i];"
"        const cb=document.createElement('input');"
"        cb.type='checkbox';"
"        cb.onchange=()=>toggleSensor(i,cb.checked);"
"        p.appendChild(span);"
"        p.appendChild(cb);"
"        inputsDiv.appendChild(p);"
"        checks.push(cb);"
"      }"
"    }"
"    if(d.in_state){"
"      for(let i=0;i<d.in_state.length;i++){"
"        checks[i].checked=!!d.in_state[i];"
"        checks[i].disabled=!d.debug_mode;"
"      }"
"    }"
"    autoToggle.checked=!!d.auto_enabled;"
"    debugToggle.checked=!!d.debug_mode;"
"    banner.textContent=d.auto_state_text;"
"    banner.classList.toggle('fault',!!d.fault);"
"    stopBtn.disabled=!d.auto_locked;"
"    skipBtn.disabled=!d.auto_locked;"
"    if(document.activeElement!==cyclesInput){ cyclesInput.value=d.num_cycles; }"
"    cyclesInfo.textContent='('+d.estimated_m3_day.toFixed(1)+' m3/dia consumido)';"
"    limitWarn.classList.toggle('hidden', !d.over_limit);"
"    if(document.activeElement!==purgeCycleInput){ purgeCycleInput.value=d.purge_cycle_min; }"
"    if(document.activeElement!==tank1VolInput){ tank1VolInput.value=d.tank1_volume_m3; }"
"    if(document.activeElement!==tank2VolInput){ tank2VolInput.value=d.tank2_volume_m3; }"
"    if(document.activeElement!==purgeVolInput){ purgeVolInput.value=d.purge_volume_m3; }"
"    producedInfo.textContent='('+d.estimated_produced_m3_day.toFixed(1)+' m3/dia produzido)';"
"    totalCyclesEl.textContent=d.total_cycles;"
"    totalWaterEl.textContent=d.total_water_in_m3.toFixed(1);"
"    updateFlows(d.flow);"
"    totalMs=d.state_total_ms||0;"
"    remainingMs=d.state_remaining_ms||0;"
"    lastFetch=Date.now();"
"    tickTimer();"
"    st.textContent='conectado';"
"  }catch(e){ st.textContent='sem conexao com o ESP32'; }"
"}"
"const flowsDiv=document.getElementById('flows');"
"let fcards=[];"
"function fnum(v,d){ return (typeof v==='number')?v.toFixed(d):'-'; }"
"function buildFlows(list){"
"  list.forEach((f,i)=>{"
"    const c=document.createElement('div'); c.className='fcard';"
"    c.innerHTML=\"<div class='t'></div><div class='v'><span></span> <small>L/min</small></div>\""
"      +\"<div class='tot'><span></span> m3</div>\""
"      +\"<details><summary>Ajustar</summary><div class='form'>\""
"      +\"<label>Totalizador (m3) - valor do hidrometro</label><input type='number' step='0.001' min='0' class='inT'>\""
"      +\"<label>K (pulsos por litro)</label><input type='number' step='0.001' min='0.1' class='inK'>\""
"      +\"<button class='btn-ok'>Salvar</button><div class='msg'></div></div></details>\";"
"    c.querySelector('.t').textContent=f.label;"
"    const det=c.querySelector('details');"
"    det.ontoggle=()=>{ if(det.open){ c.querySelector('.inT').value=fnum(fcards[i].d.total_m3,3); c.querySelector('.inK').value=fcards[i].d.k; } };"
"    c.querySelector('button').onclick=async()=>{"
"      const t=c.querySelector('.inT').value, k=c.querySelector('.inK').value, m=c.querySelector('.msg');"
"      const r=await api('/api/flow/config?id='+i+'&total_m3='+encodeURIComponent(t)+'&k='+encodeURIComponent(k));"
"      m.textContent=r.ok?'Salvo':await r.text();"
"      refresh();"
"    };"
"    flowsDiv.appendChild(c);"
"    fcards.push({el:c,d:f});"
"  });"
"}"
"function updateFlows(list){"
"  if(!list) return;"
"  if(fcards.length===0) buildFlows(list);"
"  list.forEach((f,i)=>{"
"    const c=fcards[i]; c.d=f;"
"    c.el.classList.toggle('off',!f.ok);"
"    c.el.querySelector('.v span').textContent=f.ok?fnum(f.lpm,1):'erro';"
"    c.el.querySelector('.tot span').textContent=fnum(f.total_m3,3);"
"  });"
"}"
"const wifiState=document.getElementById('wifiState');"
"const wifiMsg=document.getElementById('wifiMsg');"
"async function wifiRefresh(){"
"  try{"
"    const d=await (await api('/api/wifi/status')).json();"
"    wifiState.textContent='';"
"    const add=(k,v)=>{ const l=document.createElement('div'); l.textContent=k; if(v!==undefined){ const b=document.createElement('b'); b.textContent=v; l.appendChild(b);} wifiState.appendChild(l); };"
"    add(d.state);"
"    if(d.saved) add('Rede cadastrada: ',d.ssid);"
"    if(d.connected) add('IP no roteador: ',d.ip+'  ('+d.rssi+' dBm)');"
"    add('Access Point: ',d.ap?'ligado':'desligado');"
"  }catch(e){}"
"}"
"async function wifiSave(){"
"  const ssid=document.getElementById('wSsid').value.trim();"
"  const pass=document.getElementById('wPass').value;"
"  if(!ssid){ wifiMsg.textContent='Informe o nome da rede'; return; }"
"  const r=await api('/api/wifi/save',{method:'POST',body:new URLSearchParams({ssid,pass})});"
"  if(r.ok){ document.getElementById('wPass').value=''; wifiMsg.textContent='Salvo. Conectando... se der certo, o IP aparece acima e o AP desliga em 1 min.'; }"
"  else wifiMsg.textContent=await r.text();"
"  setTimeout(wifiRefresh,1500);"
"}"
"async function wifiForget(){"
"  if(!confirm('Esquecer a rede cadastrada e voltar a usar somente o AP?')) return;"
"  await api('/api/wifi/forget',{method:'POST'});"
"  wifiMsg.textContent='Rede esquecida.';"
"  setTimeout(wifiRefresh,1500);"
"}"
"refresh();"
"wifiRefresh();"
"setInterval(refresh,2000);"
"setInterval(wifiRefresh,5000);"
"setInterval(tickTimer,1000);"
"</script></body></html>";

// ---------------------------------------------------------------------
// Handlers HTTP
// ---------------------------------------------------------------------
// ---- Autenticacao (login simples por sessao/cookie) -------------------
// Usuario unico por enquanto. O banco de usuarios vira numa versao futura;
// ate la, trocar aqui. LOGIN_PREFILL=1 deixa os campos ja preenchidos na
// tela de login (pedido para esta fase) - com isso o login funciona mais
// como "porta" do que como seguranca: desligue quando houver senha real.
//
// Observacao: a pagina e servida em HTTP simples, entao usuario/senha
// trafegam sem criptografia na rede local.
#define AUTH_USER          "admin"
#define AUTH_PASS          "0000"
#define LOGIN_PREFILL      1
#define SESSION_MAX        6
#define SESSION_IDLE_US    (12LL * 60 * 60 * 1000 * 1000)   // expira apos 12 h sem uso
#define SESSION_COOKIE     "qqsid"

typedef struct {
    char    token[33];   // 16 bytes aleatorios em hex
    int64_t last_us;
} session_t;

// Os handlers rodam todos na task unica do httpd, entao nao ha concorrencia.
static session_t sessions[SESSION_MAX];

static session_t *session_find(const char *token)
{
    if (!token || strlen(token) != 32) return NULL;
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < SESSION_MAX; i++) {
        if (sessions[i].token[0] && strcmp(sessions[i].token, token) == 0) {
            if (now - sessions[i].last_us > SESSION_IDLE_US) {
                sessions[i].token[0] = '\0';
                return NULL;
            }
            return &sessions[i];
        }
    }
    return NULL;
}

static session_t *session_create(void)
{
    int slot = 0;
    for (int i = 0; i < SESSION_MAX; i++) {
        if (!sessions[i].token[0]) { slot = i; break; }
        if (sessions[i].last_us < sessions[slot].last_us) slot = i;   // reaproveita a mais antiga
    }
    uint8_t rnd[16];
    esp_fill_random(rnd, sizeof(rnd));
    for (int i = 0; i < 16; i++) sprintf(&sessions[slot].token[i * 2], "%02x", rnd[i]);
    sessions[slot].last_us = esp_timer_get_time();
    return &sessions[slot];
}

static session_t *session_from_req(httpd_req_t *req)
{
    char tok[40];
    size_t len = sizeof(tok);
    if (httpd_req_get_cookie_val(req, SESSION_COOKIE, tok, &len) != ESP_OK) return NULL;
    return session_find(tok);
}

// true = autorizado. false = ja respondeu 401 (o handler so precisa sair).
static bool require_auth(httpd_req_t *req)
{
    session_t *s = session_from_req(req);
    if (s) {
        s->last_us = esp_timer_get_time();
        return true;
    }
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"error\":\"login necessario\"}", HTTPD_RESP_USE_STRLEN);
    return false;
}

// Le o corpo (application/x-www-form-urlencoded) de um POST pequeno.
static bool read_form_body(httpd_req_t *req, char *buf, size_t size)
{
    if (req->content_len == 0 || req->content_len >= size) return false;
    size_t got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, buf + got, req->content_len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) return false;
        got += r;
    }
    buf[got] = '\0';
    return true;
}

// Decodifica %XX e '+' in-place (formato de formulario).
static void url_decode(char *s)
{
    char *o = s;
    for (char *p = s; *p; p++) {
        if (*p == '+') {
            *o++ = ' ';
        } else if (*p == '%' && isxdigit((unsigned char)p[1]) && isxdigit((unsigned char)p[2])) {
            char hex[3] = { p[1], p[2], 0 };
            *o++ = (char)strtol(hex, NULL, 16);
            p += 2;
        } else {
            *o++ = *p;
        }
    }
    *o = '\0';
}

static bool form_value(const char *body, const char *key, char *out, size_t size)
{
    if (httpd_query_key_value(body, key, out, size) != ESP_OK) return false;
    url_decode(out);
    return true;
}

// Escreve str como string JSON (com aspas e escapes). Retorna bytes escritos.
static int json_str(char *dst, size_t size, const char *str)
{
    size_t n = 0;
    if (size < 3) return 0;
    dst[n++] = '"';
    for (const unsigned char *p = (const unsigned char *)str; *p && n + 7 < size; p++) {
        if (*p == '"' || *p == '\\') { dst[n++] = '\\'; dst[n++] = *p; }
        else if (*p < 0x20) { n += snprintf(dst + n, size - n, "\\u%04x", *p); }
        else dst[n++] = *p;
    }
    dst[n++] = '"';
    dst[n] = '\0';
    return (int)n;
}

static const char login_html[] =
"<!DOCTYPE html><html lang='pt-br'><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1'>"
"<title>qqWater - Login</title>"
"<style>"
"body{font-family:Arial,sans-serif;background:#0f1720;color:#e8eef5;margin:0;padding:16px;"
"min-height:100vh;display:flex;align-items:center;justify-content:center;box-sizing:border-box;}"
"form{width:100%;max-width:320px;background:#1a232e;border-radius:14px;padding:24px;box-sizing:border-box;}"
"h1{font-size:20px;margin:0 0 4px;text-align:center;}"
"p{font-size:12px;color:#8ea0b3;text-align:center;margin:0 0 20px;}"
"label{display:block;font-size:13px;color:#8ea0b3;margin:12px 0 6px;}"
"input{width:100%;box-sizing:border-box;padding:12px;border-radius:8px;border:none;background:#26313f;"
"color:#e8eef5;font-size:16px;}"
"button{width:100%;margin-top:20px;padding:14px;border:none;border-radius:10px;background:#1f9d55;"
"color:#fff;font-size:15px;cursor:pointer;}"
"#err{color:#ffb3b3;font-size:13px;text-align:center;min-height:18px;margin-top:12px;}"
"</style></head><body>"
"<form id='f'>"
"<h1>qqWater</h1><p>Controle do Poco</p>"
"<label for='u'>Usuario</label><input id='u' name='user' autocomplete='username'"
#if LOGIN_PREFILL
" value='" AUTH_USER "'"
#endif
">"
"<label for='p'>Senha</label><input id='p' name='pass' type='password' autocomplete='current-password'"
#if LOGIN_PREFILL
" value='" AUTH_PASS "'"
#endif
">"
"<button type='submit'>Entrar</button>"
"<div id='err'></div>"
"</form>"
"<script>"
"document.getElementById('f').onsubmit=async(e)=>{"
"  e.preventDefault();"
"  const body=new URLSearchParams(new FormData(e.target));"
"  try{"
"    const r=await fetch('/api/login',{method:'POST',body});"
"    if(r.ok){ location.href='/'; return; }"
"    document.getElementById('err').textContent='Usuario ou senha invalidos';"
"  }catch(x){ document.getElementById('err').textContent='Sem conexao com o ESP32'; }"
"};"
"</script></body></html>";

static esp_err_t login_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, login_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t login_post_handler(httpd_req_t *req)
{
    char body[200], user[40], pass[40];
    if (!read_form_body(req, body, sizeof(body)) ||
        !form_value(body, "user", user, sizeof(user)) ||
        !form_value(body, "pass", pass, sizeof(pass)) ||
        strcmp(user, AUTH_USER) != 0 || strcmp(pass, AUTH_PASS) != 0) {
        vTaskDelay(pdMS_TO_TICKS(500));   // freia tentativa e erro
        httpd_resp_set_status(req, "401 Unauthorized");
        return httpd_resp_send(req, "credenciais invalidas", HTTPD_RESP_USE_STRLEN);
    }
    session_t *s = session_create();
    char cookie[96];
    snprintf(cookie, sizeof(cookie), SESSION_COOKIE "=%s; Path=/; HttpOnly; SameSite=Strict", s->token);
    httpd_resp_set_hdr(req, "Set-Cookie", cookie);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t logout_handler(httpd_req_t *req)
{
    session_t *s = session_from_req(req);
    if (s) s->token[0] = '\0';
    httpd_resp_set_hdr(req, "Set-Cookie", SESSION_COOKIE "=; Path=/; Max-Age=0");
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/login");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    session_t *s = session_from_req(req);
    if (!s) {
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "/login");
        return httpd_resp_send(req, NULL, 0);
    }
    s->last_us = esp_timer_get_time();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

// ---- Wi-Fi --------------------------------------------------------------
static esp_err_t wifi_status_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    char buf[384];
    int n = snprintf(buf, sizeof(buf), "{\"state\":\"%s\",\"ap\":%s,\"saved\":%s,\"ssid\":",
                     net_state_text(), ap_active ? "true" : "false", sta_has_creds ? "true" : "false");
    n += json_str(buf + n, sizeof(buf) - n, sta_ssid);
    uint32_t ip = sta_ip_addr;
    int rssi = 0;
    wifi_ap_record_t info;
    if (ip && esp_wifi_sta_get_ap_info(&info) == ESP_OK) rssi = info.rssi;
    if (ip) {
        esp_ip4_addr_t a = { .addr = ip };
        n += snprintf(buf + n, sizeof(buf) - n, ",\"connected\":true,\"ip\":\"" IPSTR "\",\"rssi\":%d}",
                      IP2STR(&a), rssi);
    } else {
        n += snprintf(buf + n, sizeof(buf) - n, ",\"connected\":false,\"ip\":\"\",\"rssi\":0}");
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t wifi_save_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    char body[320], ssid[40] = "", pass[72] = "";
    if (!read_form_body(req, body, sizeof(body)) || !form_value(body, "ssid", ssid, sizeof(ssid))) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "SSID obrigatorio", HTTPD_RESP_USE_STRLEN);
    }
    form_value(body, "pass", pass, sizeof(pass));
    memset(body, 0, sizeof(body));
    bool ok = net_request_save(ssid, pass);
    memset(pass, 0, sizeof(pass));
    if (!ok) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "SSID ate 32 caracteres; senha vazia (rede aberta) ou de 8 a 64",
                               HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t wifi_forget_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    net_request_forget();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

// ---- Sensores de fluxo --------------------------------------------------
// /api/flow/config?id=0|1&total_m3=X  e/ou  &k=Y
static esp_err_t flow_config_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    char query[96], val[24];
    int id = -1;
    bool ok = true, any = false;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "id", val, sizeof(val)) == ESP_OK) id = atoi(val);
        if (id >= 0 && id < NUM_FLOW) {
            if (httpd_query_key_value(query, "k", val, sizeof(val)) == ESP_OK && val[0]) {
                any = true;
                ok = ok && flow_set_k(id, atof(val));
            }
            if (httpd_query_key_value(query, "total_m3", val, sizeof(val)) == ESP_OK && val[0]) {
                any = true;
                ok = ok && flow_set_total_m3(id, atof(val));
            }
        }
    }
    if (id < 0 || id >= NUM_FLOW || !any || !ok) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "parametros invalidos (id=0|1, k entre 0.1 e 10000, total_m3 >= 0)",
                               HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

// Menor sobra de pilha (bytes) ja vista na task do httpd. Medida dentro do
// handler de status por ser o mais pesado e o mais chamado (a cada 2 s).
// Logada pela task de diagnostico; perto de zero = risco de estouro.
static volatile uint32_t httpd_stack_min_free = UINT32_MAX;

static esp_err_t status_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    char buf[3072];
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
    len += snprintf(buf + len, sizeof(buf) - len, "],\"in_labels\":[");
    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++) {
        len += snprintf(buf + len, sizeof(buf) - len, "\"%s\"%s",
                         boia_labels[i], (i < NUM_SENSOR_CHANNELS - 1) ? "," : "");
    }
    len += snprintf(buf + len, sizeof(buf) - len, "],\"in_state\":[");
    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++) {
        len += snprintf(buf + len, sizeof(buf) - len, "%d%s",
                         effective_sensor_state(i) ? 1 : 0, (i < NUM_SENSOR_CHANNELS - 1) ? "," : "");
    }
    len += snprintf(buf + len, sizeof(buf) - len, "],\"debug_state\":[");
    for (int i = 0; i < NUM_SENSOR_CHANNELS; i++) {
        len += snprintf(buf + len, sizeof(buf) - len, "%d%s",
                         debug_sensor_state[i] ? 1 : 0, (i < NUM_SENSOR_CHANNELS - 1) ? "," : "");
    }

    int64_t total_ms = state_total_ms();
    int64_t remaining_ms = state_remaining_ms();

    len += snprintf(buf + len, sizeof(buf) - len,
                     "],\"auto_enabled\":%s,\"auto_locked\":%s,\"fault\":%s,\"auto_state_text\":\"%s\","
                     "\"debug_mode\":%s,\"state_total_ms\":%lld,\"state_remaining_ms\":%lld,"
                     "\"num_cycles\":%d,\"estimated_m3_day\":%.1f,\"estimated_produced_m3_day\":%.1f,"
                     "\"over_limit\":%s,"
                     "\"purge_cycle_min\":%d,\"tank1_volume_m3\":%.2f,\"tank2_volume_m3\":%.2f,"
                     "\"purge_volume_m3\":%.2f,"
                     "\"total_cycles\":%lu,\"total_water_in_m3\":%.1f,\"flow\":[",
                     auto_enabled ? "true" : "false",
                     is_auto_locked() ? "true" : "false",
                     fault_msg[0] ? "true" : "false",
                     auto_state_text(),
                     debug_mode ? "true" : "false",
                     (long long)total_ms,
                     (long long)remaining_ms,
                     (int)num_cycles_per_day,
                     estimated_consumed_daily_m3(),
                     estimated_produced_daily_m3(),
                     (estimated_consumed_daily_m3() > DAILY_LIMIT_M3) ? "true" : "false",
                     (int)purge_cycle_minutes,
                     tank1_volume_m3,
                     tank2_volume_m3,
                     purge_volume_m3,
                     (unsigned long)total_cycles,
                     total_water_in_m3);

    for (int i = 0; i < NUM_FLOW && len < (int)sizeof(buf); i++) {
        len += snprintf(buf + len, sizeof(buf) - len,
                        "%s{\"label\":\"%s\",\"ok\":%s,\"lpm\":%.1f,\"total_m3\":%.3f,\"k\":%.3f}",
                        i ? "," : "", flow[i].label, flow[i].ok ? "true" : "false",
                        flow_lpm(i), flow_total_m3(i), flow_k(i));
    }
    if (len < (int)sizeof(buf)) {
        len += snprintf(buf + len, sizeof(buf) - len, "]}");
    }
    // snprintf acumulado: se em algum ponto o buffer encheu, len passa do
    // tamanho e o JSON estaria truncado - melhor responder erro do que lixo.
    if (len >= (int)sizeof(buf)) {
        ESP_LOGE(TAG, "JSON de status excedeu %u bytes", (unsigned)sizeof(buf));
        httpd_resp_send_500(req);
        return ESP_OK;
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, buf, len);

    uint32_t free_stack = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    if (free_stack < httpd_stack_min_free) {
        httpd_stack_min_free = free_stack;
    }
    return ret;
}

static esp_err_t relay_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    if (is_auto_locked()) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_send(req, "automacao ativa - desligue o Modo Automatico para controlar manualmente",
                                HTTPD_RESP_USE_STRLEN);
    }

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

static esp_err_t auto_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    char query[32];
    int state = -1;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[8];
        if (httpd_query_key_value(query, "state", val, sizeof(val)) == ESP_OK) {
            state = atoi(val);
        }
    }

    if (state != 0 && state != 1) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "parametro invalido", HTTPD_RESP_USE_STRLEN);
    }

    set_auto_enabled(state == 1);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t auto_stop_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    stop_requested = true;
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t auto_skip_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    skip_requested = true;
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t config_cycles_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    char query[32];
    int n = -1;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[8];
        if (httpd_query_key_value(query, "n", val, sizeof(val)) == ESP_OK) {
            n = atoi(val);
        }
    }

    if (!set_num_cycles_per_day(n)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "numero de ciclos invalido (faixa permitida: 1 a 28)",
                                HTTPD_RESP_USE_STRLEN);
    }
    save_settings_to_nvs();

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t config_purge_cycle_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    char query[32];
    int min = -1;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[8];
        if (httpd_query_key_value(query, "min", val, sizeof(val)) == ESP_OK) {
            min = atoi(val);
        }
    }

    if (!set_purge_cycle_minutes(min)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "tempo de purga invalido (faixa permitida: 1 a 60 min)",
                                HTTPD_RESP_USE_STRLEN);
    }
    save_settings_to_nvs();

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t config_tank_volume_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    char query[64];
    int tank = -1;
    double m3 = -1.0;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "tank", val, sizeof(val)) == ESP_OK) {
            tank = atoi(val);
        }
        if (httpd_query_key_value(query, "m3", val, sizeof(val)) == ESP_OK) {
            m3 = atof(val);
        }
    }

    if (!set_tank_volume(tank, m3)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "parametros invalidos (tank=1 ou 2, m3 entre 0.1 e 10.0)",
                                HTTPD_RESP_USE_STRLEN);
    }
    save_settings_to_nvs();

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t config_purge_volume_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    char query[32];
    double m3 = -1.0;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "m3", val, sizeof(val)) == ESP_OK) {
            m3 = atof(val);
        }
    }

    if (!set_purge_volume(m3)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "volume de purga invalido (entre 0.0 e 5.0 m3)",
                                HTTPD_RESP_USE_STRLEN);
    }
    save_settings_to_nvs();

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t totals_reset_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    reset_totals();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t debug_mode_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
    char query[32];
    int state = -1;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[8];
        if (httpd_query_key_value(query, "state", val, sizeof(val)) == ESP_OK) {
            state = atoi(val);
        }
    }

    if (state != 0 && state != 1) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "parametro invalido", HTTPD_RESP_USE_STRLEN);
    }

    debug_mode = (state == 1);
    ESP_LOGI(TAG, "Modo debug -> %s", debug_mode ? "LIGADO" : "DESLIGADO");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t debug_set_get_handler(httpd_req_t *req)
{
    if (!require_auth(req)) return ESP_OK;
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

    if (ch < 0 || ch >= NUM_SENSOR_CHANNELS || (state != 0 && state != 1)) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "parametros invalidos", HTTPD_RESP_USE_STRLEN);
    }

    debug_sensor_state[ch] = (state == 1);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

// Android, iOS/macOS e Windows batem em URLs conhecidas pra testar se a
// rede tem internet de verdade. Redirecionando essas URLs pra pagina de
// controle, o sistema operacional entende que existe um "portal" e
// mostra a notificacao de login sozinho.
static esp_err_t captive_redirect_handler(httpd_req_t *req)
{
    // Quem chegou pelo AP vai para o IP fixo do AP (e o que faz o celular
    // abrir a tela de "login de rede"). Quem chegou pelo roteador vai para
    // "/" no mesmo endereco que ja usou - o 192.168.4.1 nao existe la.
    bool via_ap = false;
    struct sockaddr_in local;
    socklen_t slen = sizeof(local);
    int fd = httpd_req_to_sockfd(req);
    if (fd >= 0 && getsockname(fd, (struct sockaddr *)&local, &slen) == 0) {
        via_ap = (local.sin_addr.s_addr == inet_addr(AP_IP_ADDR_STR));
    }
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", via_ap ? "http://" AP_IP_ADDR_STR "/" : "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 40;
    config.uri_match_fn = httpd_uri_match_wildcard;
    // Celular que sai do alcance (ou trava a tela) deixa sockets abertos.
    // Sem isso, o httpd esgota os slots e passa a recusar conexoes novas:
    // a pagina para de responder ("sem conexao com o ESP32") enquanto a
    // automacao continua rodando normalmente. Com lru_purge_enable o
    // servidor derruba a conexao mais antiga para atender a nova.
    config.lru_purge_enable = true;

    // O padrao (4 KB) fica no limite: so o status_get_handler usa 2 KB de
    // buffer local, mais o snprintf com varios %f. Um estouro que nao pega
    // o canario corrompe memoria em silencio e deixa o servidor "zumbi"
    // enquanto o resto do firmware segue normal.
    config.stack_size = 8192;

    // Explicito para deixar clara a conta de sockets: 7 sessoes + 3
    // internos do httpd, de CONFIG_LWIP_MAX_SOCKETS=16 (sobram 6, um deles
    // usado pelo DNS).
    config.max_open_sockets = 7;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;

    // TCP keep-alive: celular que dorme ou sai do alcance sem fechar a
    // conexao e detectado em ~30 + 3*5 = 45 s e a sessao e liberada, em
    // vez de ocupar o slot ate o LRU purge precisar dele.
    config.keep_alive_enable = true;
    config.keep_alive_idle = 30;
    config.keep_alive_interval = 5;
    config.keep_alive_count = 3;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri       = { .uri = "/",              .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t status_uri     = { .uri = "/api/status",     .method = HTTP_GET, .handler = status_get_handler };
        httpd_uri_t relay_uri      = { .uri = "/api/relay",      .method = HTTP_GET, .handler = relay_get_handler };
        httpd_uri_t auto_uri       = { .uri = "/api/auto",       .method = HTTP_GET, .handler = auto_get_handler };
        httpd_uri_t auto_stop_uri  = { .uri = "/api/auto/stop",  .method = HTTP_GET, .handler = auto_stop_get_handler };
        httpd_uri_t auto_skip_uri  = { .uri = "/api/auto/skip",  .method = HTTP_GET, .handler = auto_skip_get_handler };
        httpd_uri_t debug_mode_uri = { .uri = "/api/debug/mode", .method = HTTP_GET, .handler = debug_mode_get_handler };
        httpd_uri_t debug_set_uri  = { .uri = "/api/debug/set",  .method = HTTP_GET, .handler = debug_set_get_handler };
        httpd_uri_t cycles_uri     = { .uri = "/api/config/cycles", .method = HTTP_GET, .handler = config_cycles_get_handler };
        httpd_uri_t purge_cyc_uri  = { .uri = "/api/config/purge_cycle", .method = HTTP_GET, .handler = config_purge_cycle_get_handler };
        httpd_uri_t tank_vol_uri  = { .uri = "/api/config/tank_volume", .method = HTTP_GET, .handler = config_tank_volume_get_handler };
        httpd_uri_t purge_vol_uri = { .uri = "/api/config/purge_volume", .method = HTTP_GET, .handler = config_purge_volume_get_handler };
        httpd_uri_t totals_reset_uri = { .uri = "/api/totals/reset", .method = HTTP_GET, .handler = totals_reset_get_handler };

        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &relay_uri);
        httpd_register_uri_handler(server, &auto_uri);
        httpd_register_uri_handler(server, &auto_stop_uri);
        httpd_register_uri_handler(server, &auto_skip_uri);
        httpd_register_uri_handler(server, &debug_mode_uri);
        httpd_register_uri_handler(server, &debug_set_uri);
        httpd_register_uri_handler(server, &cycles_uri);
        httpd_register_uri_handler(server, &purge_cyc_uri);
        httpd_register_uri_handler(server, &tank_vol_uri);
        httpd_register_uri_handler(server, &purge_vol_uri);
        httpd_register_uri_handler(server, &totals_reset_uri);

        httpd_uri_t login_page_uri  = { .uri = "/login",            .method = HTTP_GET,  .handler = login_page_handler };
        httpd_uri_t login_post_uri  = { .uri = "/api/login",        .method = HTTP_POST, .handler = login_post_handler };
        httpd_uri_t logout_uri      = { .uri = "/logout",           .method = HTTP_GET,  .handler = logout_handler };
        httpd_uri_t wifi_status_uri = { .uri = "/api/wifi/status",  .method = HTTP_GET,  .handler = wifi_status_handler };
        httpd_uri_t wifi_save_uri   = { .uri = "/api/wifi/save",    .method = HTTP_POST, .handler = wifi_save_handler };
        httpd_uri_t wifi_forget_uri = { .uri = "/api/wifi/forget",  .method = HTTP_POST, .handler = wifi_forget_handler };
        httpd_uri_t flow_cfg_uri    = { .uri = "/api/flow/config",  .method = HTTP_GET,  .handler = flow_config_handler };
        httpd_register_uri_handler(server, &login_page_uri);
        httpd_register_uri_handler(server, &login_post_uri);
        httpd_register_uri_handler(server, &logout_uri);
        httpd_register_uri_handler(server, &wifi_status_uri);
        httpd_register_uri_handler(server, &wifi_save_uri);
        httpd_register_uri_handler(server, &wifi_forget_uri);
        httpd_register_uri_handler(server, &flow_cfg_uri);

        // URLs conhecidas de deteccao de captive portal (Android/iOS/Windows)
        static const char *captive_paths[] = {
            "/generate_204", "/gen_204",
            "/hotspot-detect.html", "/library/test/success.html",
            "/connecttest.txt", "/ncsi.txt", "/success.txt",
        };
        static httpd_uri_t captive_uris[7];
        for (int i = 0; i < 7; i++) {
            captive_uris[i].uri = captive_paths[i];
            captive_uris[i].method = HTTP_GET;
            captive_uris[i].handler = captive_redirect_handler;
            httpd_register_uri_handler(server, &captive_uris[i]);
        }

        // Coringa: qualquer outra URL nao reconhecida tambem redireciona.
        // Precisa ser o ULTIMO registrado (menor prioridade de match).
        static httpd_uri_t wildcard_uri = { .uri = "/*", .method = HTTP_GET, .handler = captive_redirect_handler };
        httpd_register_uri_handler(server, &wildcard_uri);
    } else {
        ESP_LOGE(TAG, "Falha ao iniciar o servidor web");
    }
    return server;
}

#define DIAG_PERIOD_MS (60 * 1000)

// Uma linha por minuto no monitor serial. Leitura rapida quando a pagina
// parar de abrir:
//   heap_min caindo sem parar ......... vazamento de memoria
//   httpd_pilha_min perto de 0 ........ estouro de pilha do servidor web
//   clientes = 0 com celular "conectado" ... problema no lado do Wi-Fi
static void diag_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(DIAG_PERIOD_MS));

        wifi_sta_list_t sta_list = {0};
        int n_sta = (ap_active && esp_wifi_ap_get_sta_list(&sta_list) == ESP_OK) ? sta_list.num : -1;

        uint32_t stack_min = httpd_stack_min_free;
        ESP_LOGI(TAG, "[diag] uptime=%lld min heap=%lu heap_min=%lu httpd_pilha_min=%ld "
                      "clientes_ap=%d ap=%s rede=%d",
                 (long long)(esp_timer_get_time() / 60000000LL),
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size(),
                 (stack_min == UINT32_MAX) ? -1L : (long)stack_min,
                 ap_active ? n_sta : 0, ap_active ? "on" : "off", (int)net_state);
    }
}

// ---------------------------------------------------------------------
// main
// ---------------------------------------------------------------------
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    load_totals_from_nvs();

    relays_init();
    boias_init();
    flow_init();
    xTaskCreate(flow_task, "flow_task", 3072, NULL, 5, NULL);
    xTaskCreate(boias_task, "boias_task", 3072, NULL, 5, NULL);
    xTaskCreate(automation_task, "automation_task", 4096, NULL, 5, NULL);
    net_init();
    xTaskCreate(dns_server_task, "dns_server_task", 4096, NULL, 5, NULL);
    start_webserver();
    xTaskCreate(diag_task, "diag_task", 3072, NULL, 3, NULL);

    ESP_LOGI(TAG, "qqWater pronto. AP '%s' -> http://" AP_IP_ADDR_STR " | roteador -> http://<ip>/ (nome '%s')",
             AP_SSID, STA_HOSTNAME);
}
