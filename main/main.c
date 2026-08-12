/*
 * qqWater - Controle de valvulas e bombas do poco via ESP32
 *
 * - ESP32 cria uma rede Wi-Fi propria (Access Point)
 * - Captive portal: DNS + redirecionamentos abrem a pagina sozinhos
 *   ao conectar (igual wifi de aeroporto)
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
 * ATENCAO DE HARDWARE:
 *   GPIO12, GPIO2, GPIO15 e GPIO5 sao "strapping pins" do ESP32. O que
 *   mais importa na pratica e o GPIO12 (tensao da flash no boot). Se
 *   notar boot instavel, ligar um pull-down de 10k entre GPIO12 e GND.
 *
 * LOGICA DE AUTOMACAO (maquina de estados):
 *   Start (tudo vazio): ativa watchdog de 110 min -> abre V1, liga B1,
 *     enche T1 -> desliga B1/V1 -> aguarda 50 min -> abre V2, liga B1,
 *     enche T2 -> desliga B1/V2 -> aguarda watchdog -> Ciclo A
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
 *
 * ESTIMATIVAS DE AGUA:
 *   - "agua consumida/dia" = ciclos/dia * media dos volumes dos tanques.
 *   - "agua produzida/dia" = consumida - (ciclos/dia * volume de purga).
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "nvs.h"

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

// Comportamento real da boia:
// - quando a boia boia, o circuito fica aberto e o pino fica flutuando (sem contato)
// - quando a boia afunda, o contato fecha e o pino e puxado para baixo pelo pull-down
// Para simplificar, consideramos "ativo/acionado" como o estado de flutuacao (sem contato),
// e "normal" como o estado de contato fechado puxado para LOW.
#define BOIA_FLOATING_LEVEL 0

static bool boia_state[NUM_SENSOR_CHANNELS] = {false};
static int  boia_debounce_count[NUM_SENSOR_CHANNELS] = {0};
static bool boia_last_raw[NUM_SENSOR_CHANNELS] = {false};
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
        bool raw_triggered = (raw_level == BOIA_FLOATING_LEVEL);

        if (raw_triggered == boia_last_raw[ch]) {
            if (boia_debounce_count[ch] < BOIA_DEBOUNCE_THRESHOLD) {
                boia_debounce_count[ch]++;
            }
        } else {
            boia_last_raw[ch] = raw_triggered;
            boia_debounce_count[ch] = 0;
        }

        if (boia_debounce_count[ch] >= BOIA_DEBOUNCE_THRESHOLD && boia_state[ch] != raw_triggered) {
            boia_state[ch] = raw_triggered;
            ESP_LOGI(TAG, "Boia %d (%s) -> %s", ch, boia_labels[ch],
                     raw_triggered ? "ACIONADA" : "normal");
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
// Modo debug: ignora a leitura real das boias e usa valores definidos
// manualmente pelo app, para testar a automacao sem sensores fisicos.
// ---------------------------------------------------------------------
static volatile bool debug_mode = false;
static volatile bool debug_sensor_state[NUM_SENSOR_CHANNELS] = {false};

static inline bool sensor_available(int ch)
{
    return (ch >= 0 && ch < NUM_SENSOR_CHANNELS && boia_available[ch]);
}

static inline bool effective_sensor_state(int ch)
{
    if (ch < 0 || ch >= NUM_SENSOR_CHANNELS) return false;
    if (!sensor_available(ch)) return false;
    return debug_mode ? debug_sensor_state[ch] : boia_state[ch];
}

static inline bool tank_high(int tank) { return tank == 1 ? effective_sensor_state(MIDX_T1_ALTO) : effective_sensor_state(MIDX_T2_ALTO); }

// A boia de fundo (baixo) e do tipo NF/NC: fica fechada (GND) sempre que
// ha agua acima dela (enchendo, cheio) e SO abre (libera o pull-up, HIGH)
// quando o tanque esvazia completamente abaixo dela. Ou seja, e o
// contrario da boia de alto - por isso o sinal e invertido aqui.
//   alto=HIGH, baixo=HIGH -> vazio        (tank_low = true)
//   alto=HIGH, baixo=GND  -> enchendo/esvaziando (tank_low = false)
//   alto=GND,  baixo=GND  -> cheio        (tank_low = false)
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
#define FLOW_ENTRADA_INSTALLED   false
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
static volatile int32_t num_cycles_per_day = 10; // configuravel pelo app
static volatile int32_t purge_cycle_minutes = 1;  // purga do Ciclo A/B, configuravel
static volatile uint32_t total_cycles = 0;        // ciclos A/B completos desde o boot
static volatile double total_water_in_m3 = 0.0;   // agua estimada que entrou (poco -> tanque)
static auto_state_t auto_state = AUTO_OFF;
static int64_t state_enter_time_us = 0;
static int64_t no_flow_since_us = -1;
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
            relay_write(IDX_V1, true);
            relay_write(IDX_B1, true);
            break;
        case ST_START_DECANT:
            relay_write(IDX_B1, false);
            relay_write(IDX_V1, false);
            break;
        case ST_START_FILL2:
            relay_write(IDX_V2, true);
            relay_write(IDX_B1, true);
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
}

// So chamar durante estados em que a Bomba 1 esta enchendo um tanque
static void check_dryrun(void)
{
    if (!FLOW_ENTRADA_INSTALLED) return;

    bool has_flow = false;
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
    uint8_t  auto_en = 0;

    err = nvs_get_u32(handle, "tot_cycles", &cycles);
    if (err == ESP_OK) {
        total_cycles = cycles;
    }
    err = nvs_get_u32(handle, "tot_water_x10", &water_x10);
    if (err == ESP_OK) {
        total_water_in_m3 = (double)water_x10 / 10.0;
    }
    err = nvs_get_u32(handle, "tank1_vol_x100", &tank1_x100);
    if (err == ESP_OK) {
        tank1_volume_m3 = (double)tank1_x100 / 100.0;
    }
    err = nvs_get_u32(handle, "tank2_vol_x100", &tank2_x100);
    if (err == ESP_OK) {
        tank2_volume_m3 = (double)tank2_x100 / 100.0;
    }
    err = nvs_get_u32(handle, "purge_vol_x100", &purge_x100);
    if (err == ESP_OK) {
        purge_volume_m3 = (double)purge_x100 / 100.0;
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
             "purga=%.2f m3, automatico=%s",
             (unsigned long)total_cycles, total_water_in_m3, tank1_volume_m3, tank2_volume_m3,
             purge_volume_m3, auto_enabled ? "estava LIGADO" : "estava desligado");
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

// Grava os campos de volume configuraveis (tanques + purga) na NVS.
static void save_volumes_to_nvs(void)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open("qqwater", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao abrir NVS para gravar volumes: %s", esp_err_to_name(err));
        return;
    }
    nvs_set_u32(handle, "tank1_vol_x100", (uint32_t)(tank1_volume_m3 * 100.0 + 0.5));
    nvs_set_u32(handle, "tank2_vol_x100", (uint32_t)(tank2_volume_m3 * 100.0 + 0.5));
    nvs_set_u32(handle, "purge_vol_x100", (uint32_t)(purge_volume_m3 * 100.0 + 0.5));
    nvs_commit(handle);
    nvs_close(handle);
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

    while (1) {
        check_overpressure();

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
// Servidor DNS (captive portal) - responde qualquer consulta com o IP
// do proprio ESP32, fazendo o celular achar que precisa "fazer login".
// ---------------------------------------------------------------------
#define DNS_PORT 53
#define DNS_MAX_LEN 512

static void dns_server_task(void *arg)
{
    char rx_buffer[DNS_MAX_LEN];
    uint8_t response[DNS_MAX_LEN];

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
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                            (struct sockaddr *)&source_addr, &socklen);
        if (len < 12) continue; // menor que um cabecalho DNS valido

        int resp_len = len;
        memcpy(response, rx_buffer, len);

        response[2] = 0x81; // resposta, sem truncamento
        response[3] = 0x80; // recursao disponivel
        response[6] = 0x00; // ANCOUNT high byte
        response[7] = 0x01; // ANCOUNT = 1 resposta

        response[resp_len++] = 0xC0; response[resp_len++] = 0x0C; // ponteiro pro nome perguntado
        response[resp_len++] = 0x00; response[resp_len++] = 0x01; // TYPE A
        response[resp_len++] = 0x00; response[resp_len++] = 0x01; // CLASS IN
        response[resp_len++] = 0x00; response[resp_len++] = 0x00; response[resp_len++] = 0x00; response[resp_len++] = 0x3C; // TTL 60s
        response[resp_len++] = 0x00; response[resp_len++] = 0x04; // RDLENGTH = 4 bytes
        response[resp_len++] = 192;  response[resp_len++] = 168;  response[resp_len++] = 4; response[resp_len++] = 1; // 192.168.4.1

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
"</style></head><body>"
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
"<div class='grid' id='grid'></div>"
"<h2><span>Sensores</span><span class='dbg'><label class='switch' style='width:34px;height:20px'>"
"<input type='checkbox' id='debugToggle' onchange='toggleDebug(this.checked)'>"
"<span class='slider'></span></label>Modo Debug</span></h2>"
"<div class='inputs' id='inputs'></div>"
"<div class='status' id='status'>conectando...</div>"
"<script>"
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
"  await fetch('/api/relay?ch='+ch+'&state='+(cur?0:1));"
"  refresh();"
"}"
"async function toggleAuto(checked){"
"  await fetch('/api/auto?state='+(checked?1:0));"
"  refresh();"
"}"
"async function toggleDebug(checked){"
"  await fetch('/api/debug/mode?state='+(checked?1:0));"
"  refresh();"
"}"
"async function stopSystem(){"
"  await fetch('/api/auto/stop');"
"  refresh();"
"}"
"async function skipStep(){"
"  await fetch('/api/auto/skip');"
"  refresh();"
"}"
"async function setCycles(n){"
"  await fetch('/api/config/cycles?n='+n);"
"  refresh();"
"}"
"async function setPurgeCycle(min){"
"  await fetch('/api/config/purge_cycle?min='+min);"
"  refresh();"
"}"
"async function setTankVolume(tank,m3){"
"  await fetch('/api/config/tank_volume?tank='+tank+'&m3='+m3);"
"  refresh();"
"}"
"async function setPurgeVolume(m3){"
"  await fetch('/api/config/purge_volume?m3='+m3);"
"  refresh();"
"}"
"async function resetTotals(){"
"  await fetch('/api/totals/reset');"
"  refresh();"
"}"
"async function toggleSensor(ch,checked){"
"  await fetch('/api/debug/set?ch='+ch+'&state='+(checked?1:0));"
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
"    const r=await fetch('/api/status');"
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
"    totalMs=d.state_total_ms||0;"
"    remainingMs=d.state_remaining_ms||0;"
"    lastFetch=Date.now();"
"    tickTimer();"
"    st.textContent='conectado';"
"  }catch(e){ st.textContent='sem conexao com o ESP32'; }"
"}"
"refresh();"
"setInterval(refresh,2000);"
"setInterval(tickTimer,1000);"
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
    char buf[2048];
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
                     "\"total_cycles\":%lu,\"total_water_in_m3\":%.1f}",
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

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, len);
}

static esp_err_t relay_get_handler(httpd_req_t *req)
{
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
    stop_requested = true;
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t auto_skip_get_handler(httpd_req_t *req)
{
    skip_requested = true;
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t config_cycles_get_handler(httpd_req_t *req)
{
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

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t config_purge_cycle_get_handler(httpd_req_t *req)
{
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

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t config_tank_volume_get_handler(httpd_req_t *req)
{
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
    save_volumes_to_nvs();

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t config_purge_volume_get_handler(httpd_req_t *req)
{
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
    save_volumes_to_nvs();

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t totals_reset_get_handler(httpd_req_t *req)
{
    reset_totals();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t debug_mode_get_handler(httpd_req_t *req)
{
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
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 28;
    config.uri_match_fn = httpd_uri_match_wildcard;

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
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    load_totals_from_nvs();

    relays_init();
    boias_init();
    xTaskCreate(boias_task, "boias_task", 3072, NULL, 5, NULL);
    xTaskCreate(automation_task, "automation_task", 4096, NULL, 5, NULL);
    wifi_init_softap();
    xTaskCreate(dns_server_task, "dns_server_task", 4096, NULL, 5, NULL);
    start_webserver();

    ESP_LOGI(TAG, "qqWater pronto. Conecte na rede '%s' e acesse http://192.168.4.1", AP_SSID);
}