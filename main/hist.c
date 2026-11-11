#include "hist.h"

#include <string.h>

static void mark_dirty_pos(hist_t *h, int pos)
{
    h->dirty |= (uint8_t)(1u << (pos / HIST_CHUNK));
}

static void clear_slot(hist_t *h, int32_t s)
{
    int p = hist_pos(s);
    for (int i = 0; i < HIST_NSENS; i++) h->dl[i][p] = 0;
    h->fl[p] = HF_NODATA;
    mark_dirty_pos(h, p);
}

// Marca [a, b] como sem dado. Intervalos maiores que o anel limpam tudo.
static void clear_range(hist_t *h, int32_t a, int32_t b)
{
    if (b < a) return;
    if ((int64_t)b - a + 1 >= HIST_N) {
        for (int p = 0; p < HIST_N; p++) {
            for (int i = 0; i < HIST_NSENS; i++) h->dl[i][p] = 0;
            h->fl[p] = HF_NODATA;
        }
        h->dirty = (uint8_t)((1u << HIST_NCHUNK) - 1);
        return;
    }
    for (int32_t s = a; s <= b; s++) clear_slot(h, s);
}

static bool in_ring(const hist_t *h, int32_t s)
{
    return h->last >= 0 && s <= h->last && s > h->last - HIST_N;
}

static uint16_t to_dl(double liters)
{
    double v = liters * 10.0 + 0.5;
    if (v < 0) v = 0;
    if (v > 65535.0) v = 65535.0;
    return (uint16_t)v;
}

void hist_reset(hist_t *h)
{
    memset(h, 0, sizeof(*h));
    for (int p = 0; p < HIST_N; p++) h->fl[p] = HF_NODATA;
    h->last = -1;
    h->cur = -1;
    h->run_first = -1;
}

void hist_start(hist_t *h, uint8_t boot_flags)
{
    h->cur = -1;
    h->cur_real = false;
    for (int i = 0; i < HIST_NSENS; i++) h->acc_l[i] = 0.0;
    h->cur_fl = boot_flags;
    h->run_first = -1;
    h->dirty = 0;
    h->hdr_dirty = false;
}

int32_t hist_pseudo_base_slot(const hist_t *h)
{
    return h->last >= 0 ? h->last + 1 : 0;
}

void hist_add(hist_t *h, int sens, double liters)
{
    if (sens >= 0 && sens < HIST_NSENS && liters > 0) h->acc_l[sens] += liters;
}

void hist_flag(hist_t *h, uint8_t flags)
{
    h->cur_fl |= flags;
}

// Se o slot ja tem dado gravado (religou dentro do mesmo slot), continua
// somando em cima dele em vez de sobrescrever.
static void preload_from_ring(hist_t *h, int32_t s)
{
    if (!in_ring(h, s)) return;
    int p = hist_pos(s);
    if (h->fl[p] & HF_NODATA) return;
    for (int i = 0; i < HIST_NSENS; i++) h->acc_l[i] += h->dl[i][p] / 10.0;
    h->cur_fl |= (uint8_t)(h->fl[p] & (HF_POWER | HF_RESET | HF_CYCLE));
}

static void open_slot(hist_t *h, int32_t slot, bool real)
{
    // Nunca escreve antes do ultimo slot gravado (relogio atrasado).
    if (h->last >= 0 && slot < h->last) slot = h->last;
    h->cur = slot;
    h->cur_real = real;
    if (h->last >= 0 && slot == h->last) preload_from_ring(h, slot);
}

static void finalize(hist_t *h)
{
    int32_t s = h->cur;
    if (s < 0) return;
    if (h->last >= 0 && s > h->last + 1) clear_range(h, h->last + 1, s - 1);

    int p = hist_pos(s);
    for (int i = 0; i < HIST_NSENS; i++) {
        h->dl[i][p] = to_dl(h->acc_l[i]);
        h->acc_l[i] = 0.0;
    }
    h->fl[p] = (uint8_t)((h->cur_fl & ~HF_NODATA & ~HF_REAL) | (h->cur_real ? HF_REAL : 0));
    mark_dirty_pos(h, p);
    h->cur_fl = 0;

    if (h->run_first < 0) h->run_first = s;
    if (s > h->last) {
        h->last = s;
        h->hdr_dirty = true;
    }
}

// O relogio passou de estimado para real nesta execucao: os slots gravados
// desde o boot estavam num horario estimado (logo apos a queda). Move-os
// para terminar no horario real e deixa o periodo sem energia sem dado.
static void transition_to_real(hist_t *h, int32_t R)
{
    int32_t n = (h->run_first >= 0 && h->last >= h->run_first) ? h->last - h->run_first + 1 : 0;
    if (n > HIST_N - 1) n = HIST_N - 1;
    int32_t prev_last = (h->run_first >= 0) ? h->run_first - 1 : h->last;

    static uint16_t tmp_dl[HIST_NSENS][HIST_N];
    static uint8_t tmp_fl[HIST_N];
    for (int32_t k = 0; k < n; k++) {
        int p = hist_pos(h->last - n + 1 + k);
        for (int i = 0; i < HIST_NSENS; i++) tmp_dl[i][k] = h->dl[i][p];
        tmp_fl[k] = h->fl[p];
    }

    // Novo inicio. Se o tempo real decorrido for menor que o estimado (nao
    // deveria acontecer), encaixa logo depois da execucao anterior.
    int32_t new_first = R - n;
    if (prev_last >= 0 && new_first <= prev_last) {
        new_first = prev_last + 1;
        R = new_first + n;
    }

    // Apaga a faixa estimada e o intervalo ate o horario real.
    // (Sem execucao anterior, comeca do primeiro slot desta execucao.)
    int32_t clear_from = (prev_last >= 0) ? prev_last + 1 : h->run_first;
    if (h->last >= 0) clear_range(h, clear_from, R - 1);

    for (int32_t k = 0; k < n; k++) {
        int p = hist_pos(new_first + k);
        for (int i = 0; i < HIST_NSENS; i++) h->dl[i][p] = tmp_dl[i][k];
        h->fl[p] = (uint8_t)((tmp_fl[k] & ~HF_NODATA) | HF_REAL);
        mark_dirty_pos(h, p);
    }

    if (n > 0) {
        h->last = R - 1;
        h->run_first = new_first;
    } else {
        h->last = prev_last;
        h->run_first = -1;
    }
    h->hdr_dirty = true;

    // O slot parcial (com o que acumulou ate agora) segue no horario real.
    if (h->last >= 0 && R <= h->last) {
        R = h->last;
        preload_from_ring(h, R);
    }
    h->cur = R;
    h->cur_real = true;
}

void hist_tick(hist_t *h, int32_t slot, bool real)
{
    if (h->cur < 0) {
        open_slot(h, slot, real);
        return;
    }
    if (real && !h->cur_real) {
        transition_to_real(h, slot);
        return;
    }
    if (slot <= h->cur) return;   // mesmo slot (ou relogio voltou): continua somando
    finalize(h);
    open_slot(h, slot, real);
}

void hist_flush_partial(hist_t *h)
{
    if (h->cur < 0) return;
    bool real = h->cur_real;
    int32_t s = h->cur;
    finalize(h);
    // Continua no mesmo slot ate o reinicio (sem perder o que vier depois).
    h->cur = s;
    h->cur_real = real;
    preload_from_ring(h, s);
}

void hist_get(const hist_t *h, int32_t s, uint16_t dl[HIST_NSENS], uint8_t *fl)
{
    if (h->cur >= 0 && s == h->cur) {
        for (int i = 0; i < HIST_NSENS; i++) dl[i] = to_dl(h->acc_l[i]);
        *fl = (uint8_t)((h->cur_fl & ~HF_NODATA & ~HF_REAL) | (h->cur_real ? HF_REAL : 0));
        return;
    }
    if (!in_ring(h, s)) {
        for (int i = 0; i < HIST_NSENS; i++) dl[i] = 0;
        *fl = HF_NODATA;
        return;
    }
    int p = hist_pos(s);
    for (int i = 0; i < HIST_NSENS; i++) dl[i] = h->dl[i][p];
    *fl = h->fl[p];
}

int32_t hist_end(const hist_t *h)
{
    return h->cur >= 0 ? h->cur : h->last;
}
