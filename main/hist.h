/*
 * Historico de vazao de 7 dias em slots de 10 minutos.
 *
 * Logica pura (sem ESP-IDF), para poder ser testada no PC. A gravacao na
 * NVS, o relogio e o mutex ficam no main.c.
 *
 * Cada slot guarda, para cada sensor, o volume que passou naquele intervalo
 * de 10 min (em decilitros) e flags de eventos. A vazao media do slot e
 * volume/10 min. Os slots ficam num anel de HIST_N posicoes, indexado pelo
 * numero absoluto do slot (epoch/600): posicao = slot % HIST_N.
 *
 * Relogio: quando ha hora real (NTP ou celular), slot = epoch/600. Sem hora
 * real, o firmware usa um relogio "estimado" que continua do ultimo slot
 * gravado (o tempo que ficou sem energia nao e contado). Quando a hora real
 * aparece no meio de uma execucao, hist_tick() reposiciona os slots desta
 * execucao para o horario certo e deixa a queda como intervalo sem dado.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#define HIST_N        1008   // 7 dias x 24 h x 6 slots/h
#define HIST_CHUNK    144    // 1 dia por bloco gravado na NVS
#define HIST_NCHUNK   (HIST_N / HIST_CHUNK)
#define HIST_SLOT_S   600
#define HIST_NSENS    2

#define HF_NODATA  0x01   // sem dado (antes do historico, ou ESP desligado)
#define HF_POWER   0x02   // ESP religou neste slot apos queda de energia
#define HF_RESET   0x04   // ESP reiniciou neste slot por outro motivo (OTA, botao, erro)
#define HF_REAL    0x08   // horario do slot e real (NTP/celular), nao estimado
#define HF_CYCLE   0x10   // um ciclo A/B foi concluido neste slot

typedef struct {
    // --- persistido ---
    uint16_t dl[HIST_NSENS][HIST_N];   // decilitros por slot
    uint8_t  fl[HIST_N];               // flags HF_*
    int32_t  last;                     // ultimo slot gravado (-1 = nenhum)

    // --- estado da execucao atual ---
    int32_t  cur;                      // slot em acumulacao (-1 = ainda nao aberto)
    bool     cur_real;                 // relogio era real quando cur foi aberto
    double   acc_l[HIST_NSENS];        // litros acumulados no slot corrente
    uint8_t  cur_fl;                   // flags do slot corrente
    int32_t  run_first;                // primeiro slot gravado nesta execucao (-1)
    uint8_t  dirty;                    // bitmask de blocos (HIST_NCHUNK) a gravar
    bool     hdr_dirty;                // cabecalho (last) a gravar
} hist_t;

static inline int hist_pos(int32_t slot)
{
    int p = (int)(slot % HIST_N);
    return p < 0 ? p + HIST_N : p;
}

// Historico vazio (tudo sem dado).
void hist_reset(hist_t *h);

// Inicio de execucao (apos carregar da NVS). boot_flags: HF_POWER/HF_RESET.
void hist_start(hist_t *h, uint8_t boot_flags);

// Slot onde o relogio estimado deve comecar nesta execucao.
int32_t hist_pseudo_base_slot(const hist_t *h);

void hist_add(hist_t *h, int sens, double liters);
void hist_flag(hist_t *h, uint8_t flags);

// Chamar periodicamente (1 s) com o slot atual do relogio.
void hist_tick(hist_t *h, int32_t slot, bool real);

// Grava o slot corrente como esta (antes de um reinicio programado).
void hist_flush_partial(hist_t *h);

// Le o slot s (inclusive o corrente, parcial). Fora da janela: HF_NODATA.
void hist_get(const hist_t *h, int32_t s, uint16_t dl[HIST_NSENS], uint8_t *fl);

// Slot mais recente (o corrente, ou o ultimo gravado se nada foi aberto).
int32_t hist_end(const hist_t *h);
