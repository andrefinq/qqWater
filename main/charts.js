// qqWater - graficos de vazao (7 dias) e indicadores.
// Embutido no firmware (EMBED_TXTFILES) e servido em /charts.js.
// Dados: GET /api/history -> 1008 slots de 10 min, do mais antigo ao atual.
(function () {
  'use strict';

  // Faixas dos medidores (fracao). Abaixo de red = vermelho, abaixo de
  // yellow = amarelo, acima = verde. A escala vai ate max.
  var GAUGE_POCO = { red: 0.70, yellow: 0.90, max: 1.2 };
  var GAUGE_REND = { red: 0.75, yellow: 0.90, max: 1.2 };

  var F_NODATA = 1, F_POWER = 2, F_RESET = 4, F_REAL = 8, F_CYCLE = 16;
  var C_POCO = '#4aa3ff', C_TRAT = '#2dd4bf', C_POWER = '#ef5350', C_RESET = '#8ea0b3';
  var C_GRID = '#26313f', C_TEXT = '#8ea0b3';
  var Z_RED = '#e5534b', Z_YEL = '#e2b93b', Z_GRN = '#3fb950';

  var H = null;           // ultimo /api/history
  var hover = -1;         // indice sob o dedo/mouse no grafico de linha
  var timeSent = false;
  var DIAS = ['dom', 'seg', 'ter', 'qua', 'qui', 'sex', 'sab'];

  function $(id) { return document.getElementById(id); }

  function api(u, o) {
    return fetch(u, o).then(function (r) {
      if (r.status === 401) { location.href = '/login'; throw new Error('login'); }
      return r;
    });
  }

  // ---------- dados ----------
  function slotMs(i) { return (H.end - (H.p.length - 1) + i) * H.slot_s * 1000; }
  function valid(i) { return !(H.f[i] & F_NODATA); }
  function lpm(arr, i) { return arr[i] / 100; }            // dL por 10 min -> L/min
  function pad(n) { return (n < 10 ? '0' : '') + n; }
  function fmtTime(ms) {
    var d = new Date(ms);
    return DIAS[d.getDay()] + ' ' + pad(d.getDate()) + '/' + pad(d.getMonth() + 1) + ' ' +
      pad(d.getHours()) + ':' + pad(d.getMinutes());
  }
  function fmtAgo(i) {
    var min = (H.p.length - 1 - i) * H.slot_s / 60;
    if (min < 60) return 'ha ' + Math.round(min) + ' min';
    var h = Math.floor(min / 60), d = Math.floor(h / 24);
    return 'ha ' + (d ? d + 'd ' : '') + (h % 24) + 'h';
  }
  function slotLabel(i) { return (H.f[i] & F_REAL) ? fmtTime(slotMs(i)) : fmtAgo(i) + ' (estimado)'; }
  function fix(v, d) { return (Math.round(v * Math.pow(10, d)) / Math.pow(10, d)).toFixed(d); }

  function stats() {
    var s = { valid: 0, poco: 0, trat: 0, cycles: 0, power: 0, resets: 0, estimated: false, maxLpm: 0 };
    for (var i = 0; i < H.p.length; i++) {
      var f = H.f[i];
      if (f & F_POWER) s.power++;
      if (f & F_RESET) s.resets++;
      if (f & F_NODATA) continue;
      s.valid++;
      s.poco += H.p[i] / 10;          // litros
      s.trat += H.t[i] / 10;
      if (f & F_CYCLE) s.cycles++;
      if (!(f & F_REAL)) s.estimated = true;
      s.maxLpm = Math.max(s.maxLpm, lpm(H.p, i), lpm(H.t, i));
    }
    return s;
  }

  // ---------- canvas ----------
  function setupCanvas(cv, h) {
    var w = cv.parentNode.clientWidth;
    var dpr = window.devicePixelRatio || 1;
    cv.style.width = w + 'px';
    cv.style.height = h + 'px';
    cv.width = Math.round(w * dpr);
    cv.height = Math.round(h * dpr);
    var ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    ctx.font = '10px Arial, sans-serif';
    return { ctx: ctx, w: w, h: h };
  }

  function niceMax(v) {
    if (v <= 0) return 10;
    var e = Math.pow(10, Math.floor(Math.log(v) / Math.LN10));
    var steps = [1, 2, 2.5, 5, 10];
    for (var k = 0; k < steps.length; k++) if (steps[k] * e >= v) return steps[k] * e;
    return 10 * e;
  }

  function yAxis(c, m, top, ymax, unit) {
    var ctx = c.ctx;
    ctx.fillStyle = C_TEXT; ctx.strokeStyle = C_GRID; ctx.lineWidth = 1;
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (var g = 0; g <= 4; g++) {
      var v = ymax * g / 4, y = top + (c.h - m.b - top) * (1 - g / 4);
      ctx.beginPath(); ctx.moveTo(m.l, Math.round(y) + 0.5); ctx.lineTo(c.w - m.r, Math.round(y) + 0.5); ctx.stroke();
      ctx.fillText(v === 0 ? '0' : v >= 10 ? Math.round(v) : fix(v, 1), m.l - 4, y);
    }
    ctx.textAlign = 'left'; ctx.textBaseline = 'top';
    ctx.fillText(unit, 2, 0);
  }

  // ---------- grafico de linha (7 dias, 10 min) ----------
  function drawLine() {
    var cv = $('chLine');
    var c = setupCanvas(cv, 210), ctx = c.ctx;
    var m = { l: 34, r: 8, b: 20 }, top = 16;
    var n = H.p.length, s = stats();
    var ymax = niceMax(Math.max(s.maxLpm, 1) * 1.05);
    var pw = c.w - m.l - m.r, ph = c.h - m.b - top;
    function X(i) { return m.l + pw * i / (n - 1); }
    function Y(v) { return top + ph * (1 - v / ymax); }

    yAxis(c, m, top, ymax, 'L/min');

    // eixo x: meia-noite local (horario real) ou blocos de 24 h a partir de agora
    ctx.fillStyle = C_TEXT; ctx.strokeStyle = C_GRID; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    var ticks = [];
    if (H.real) {
      for (var i = 1; i < n; i++) {
        var d0 = new Date(slotMs(i - 1)), d1 = new Date(slotMs(i));
        if (d0.getDate() !== d1.getDate()) ticks.push([i, DIAS[d1.getDay()] + ' ' + pad(d1.getDate())]);
      }
    } else {
      for (var k = 1; k <= 6; k++) ticks.push([n - 1 - k * 144, '-' + k + 'd']);
    }
    ticks.forEach(function (t) {
      var x = Math.round(X(t[0])) + 0.5;
      ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, c.h - m.b); ctx.stroke();
      ctx.fillText(t[1], x, c.h - m.b + 4);
    });

    // marcadores de reinicio (atras das linhas)
    for (var j = 0; j < n; j++) {
      var f = H.f[j];
      if (!(f & (F_POWER | F_RESET))) continue;
      var xm = Math.round(X(j)) + 0.5, col = (f & F_POWER) ? C_POWER : C_RESET;
      ctx.strokeStyle = col; ctx.globalAlpha = 0.55; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.moveTo(xm, top); ctx.lineTo(xm, c.h - m.b); ctx.stroke();
      ctx.setLineDash([]); ctx.globalAlpha = 1; ctx.fillStyle = col;
      ctx.beginPath(); ctx.moveTo(xm - 4, top - 8); ctx.lineTo(xm + 4, top - 8); ctx.lineTo(xm, top - 1); ctx.fill();
    }

    // series (quebradas onde nao ha dado)
    [[H.p, C_POCO], [H.t, C_TRAT]].forEach(function (sr) {
      ctx.strokeStyle = sr[1]; ctx.lineWidth = 1.5; ctx.lineJoin = 'round';
      ctx.beginPath();
      var pen = false;
      for (var i = 0; i < n; i++) {
        if (!valid(i)) { pen = false; continue; }
        var x = X(i), y = Y(lpm(sr[0], i));
        if (!pen) { ctx.moveTo(x, y); pen = true; } else ctx.lineTo(x, y);
      }
      ctx.stroke();
    });

    // cursor
    var tip = $('chTip');
    if (hover >= 0 && hover < n) {
      var xh = Math.round(X(hover)) + 0.5;
      ctx.strokeStyle = '#e8eef5'; ctx.globalAlpha = 0.5; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(xh, top); ctx.lineTo(xh, c.h - m.b); ctx.stroke(); ctx.globalAlpha = 1;
      var lines = [slotLabel(hover)];
      if (valid(hover)) {
        [[H.p, C_POCO], [H.t, C_TRAT]].forEach(function (sr) {
          ctx.fillStyle = sr[1];
          ctx.beginPath(); ctx.arc(xh, Y(lpm(sr[0], hover)), 3, 0, 2 * Math.PI); ctx.fill();
        });
        lines.push('<span style="color:' + C_POCO + '">Poco ' + fix(lpm(H.p, hover), 1) + ' L/min</span>');
        lines.push('<span style="color:' + C_TRAT + '">Tratada ' + fix(lpm(H.t, hover), 1) + ' L/min</span>');
      } else {
        lines.push('sem dado (ESP desligado)');
      }
      if (H.f[hover] & F_POWER) lines.push('<span style="color:' + C_POWER + '">religou apos queda de energia</span>');
      if (H.f[hover] & F_RESET) lines.push('reiniciou (OTA/botao/erro)');
      if (H.f[hover] & F_CYCLE) lines.push('ciclo concluido');
      tip.innerHTML = lines.join('<br>');
      tip.style.display = 'block';
      var left = xh + 10;
      if (left + tip.offsetWidth > c.w) left = xh - 10 - tip.offsetWidth;
      tip.style.left = Math.max(0, left) + 'px';
    } else {
      tip.style.display = 'none';
    }
  }

  function bindHover() {
    var cv = $('chLine');
    function at(ev) {
      var r = cv.getBoundingClientRect();
      var cx = (ev.touches ? ev.touches[0].clientX : ev.clientX) - r.left;
      var i = Math.round((cx - 34) / (r.width - 42) * (H.p.length - 1));
      hover = Math.max(0, Math.min(H.p.length - 1, i));
      drawLine();
    }
    cv.addEventListener('mousemove', at);
    cv.addEventListener('touchstart', at, { passive: true });
    cv.addEventListener('touchmove', at, { passive: true });
    cv.addEventListener('mouseleave', function () { hover = -1; drawLine(); });
  }

  // ---------- barras: media diaria ----------
  function dailyGroups() {
    var n = H.p.length, groups = [];
    if (H.real) {
      // ultimos 7 dias de calendario (horario local do celular), hoje incluso
      var byKey = {}, order = [];
      for (var i = 0; i < n; i++) {
        var d = new Date(slotMs(i));
        var key = d.getFullYear() + '-' + d.getMonth() + '-' + d.getDate();
        if (!byKey[key]) { byKey[key] = { label: DIAS[d.getDay()] + ' ' + pad(d.getDate()), idx: [] }; order.push(key); }
        byKey[key].idx.push(i);
      }
      order.slice(-7).forEach(function (k) { groups.push(byKey[k]); });
      if (groups.length) groups[groups.length - 1].label = 'hoje';
    } else {
      for (var g = 6; g >= 0; g--) {
        var idx = [];
        for (var j = n - 1 - (g + 1) * 144 + 1; j <= n - 1 - g * 144; j++) if (j >= 0) idx.push(j);
        groups.push({ label: g === 0 ? 'ult. 24h' : '-' + g + 'd', idx: idx });
      }
    }
    groups.forEach(function (gr) {
      var vs = 0, lp = 0, lt = 0;
      gr.idx.forEach(function (i) { if (valid(i)) { vs++; lp += H.p[i] / 10; lt += H.t[i] / 10; } });
      gr.valid = vs;
      gr.poco = vs ? lp / (vs * 10) : null;   // L/min medio no tempo com dado
      gr.trat = vs ? lt / (vs * 10) : null;
      gr.m3p = lp / 1000; gr.m3t = lt / 1000;
    });
    return groups;
  }

  function drawBars() {
    var cv = $('chBars');
    var c = setupCanvas(cv, 170), ctx = c.ctx;
    var m = { l: 34, r: 8, b: 20 }, top = 18;
    var gs = dailyGroups(), mx = 0;
    gs.forEach(function (g) { mx = Math.max(mx, g.poco || 0, g.trat || 0); });
    var ymax = niceMax(Math.max(mx, 1) * 1.15);
    yAxis(c, m, top, ymax, 'L/min');
    var pw = c.w - m.l - m.r, ph = c.h - m.b - top, gw = pw / gs.length, bw = Math.min(22, gw * 0.34);
    ctx.textAlign = 'center';
    gs.forEach(function (g, k) {
      var cx = m.l + gw * (k + 0.5);
      [[g.poco, C_POCO, -1], [g.trat, C_TRAT, 1]].forEach(function (b) {
        var x = cx + (b[2] < 0 ? -bw - 1 : 1);
        if (b[0] === null) return;
        var hgt = ph * b[0] / ymax;
        ctx.fillStyle = b[1];
        ctx.fillRect(x, top + ph - hgt, bw, hgt);
        ctx.fillStyle = '#e8eef5'; ctx.textBaseline = 'bottom';
        ctx.fillText(b[0] >= 10 ? Math.round(b[0]) : fix(b[0], 1), x + bw / 2, top + ph - hgt - 2);
      });
      ctx.fillStyle = C_TEXT; ctx.textBaseline = 'top';
      ctx.fillText(g.valid ? g.label : g.label + ' -', cx, c.h - m.b + 4);
    });
  }

  // ---------- medidores ----------
  function gaugeSvg(ratio, z) {
    var cx = 60, cy = 60, r = 46, max = z.max;
    function pt(v) {
      var a = Math.PI * (1 - Math.min(Math.max(v, 0), max) / max);
      return [cx + r * Math.cos(a), cy - r * Math.sin(a)];
    }
    function arc(a, b, col) {
      var p1 = pt(a), p2 = pt(b);
      return '<path d="M' + p1[0].toFixed(1) + ' ' + p1[1].toFixed(1) + ' A' + r + ' ' + r + ' 0 0 1 ' +
        p2[0].toFixed(1) + ' ' + p2[1].toFixed(1) + '" stroke="' + col + '" stroke-width="9" fill="none"/>';
    }
    var svg = '<svg viewBox="0 0 120 70" width="100%" role="img">' +
      arc(0, z.red, Z_RED) + arc(z.red, z.yellow, Z_YEL) + arc(z.yellow, max, Z_GRN);
    if (ratio !== null && isFinite(ratio)) {
      var tip = pt(ratio), ang = Math.PI * (1 - Math.min(Math.max(ratio, 0), max) / max);
      var nx = cx + (r - 14) * Math.cos(ang), ny = cy - (r - 14) * Math.sin(ang);
      svg += '<line x1="' + cx + '" y1="' + cy + '" x2="' + nx.toFixed(1) + '" y2="' + ny.toFixed(1) +
        '" stroke="#e8eef5" stroke-width="3" stroke-linecap="round"/>' +
        '<circle cx="' + tip[0].toFixed(1) + '" cy="' + tip[1].toFixed(1) + '" r="3" fill="#e8eef5"/>';
    }
    return svg + '<circle cx="' + cx + '" cy="' + cy + '" r="4" fill="#e8eef5"/></svg>';
  }

  function zoneColor(ratio, z) {
    if (ratio === null || !isFinite(ratio)) return C_TEXT;
    return ratio < z.red ? Z_RED : ratio < z.yellow ? Z_YEL : Z_GRN;
  }

  function setGauge(id, ratio, z, sub) {
    var el = $(id);
    el.querySelector('.g').innerHTML = gaugeSvg(ratio, z);
    var v = el.querySelector('.gv');
    v.textContent = (ratio === null || !isFinite(ratio)) ? '-' : Math.round(ratio * 100) + '%';
    v.style.color = zoneColor(ratio, z);
    el.querySelector('.gs').textContent = sub;
  }

  function drawCards() {
    var s = stats();
    var dias = s.valid / 144;
    var esperado = H.exp_m3_day * dias;
    var poco = s.poco / 1000, trat = s.trat / 1000, desc = s.cycles * H.purge_m3;
    setGauge('gPoco', esperado > 0 ? poco / esperado : null, GAUGE_POCO,
      fix(poco, 1) + ' de ' + fix(esperado, 1) + ' m3 esperados (' + fix(dias, 1) + ' d com dado)');
    var base = poco - desc;
    setGauge('gRend', base > 0 ? trat / base : null, GAUGE_REND,
      fix(trat, 1) + ' tratada / (' + fix(poco, 1) + ' poco - ' + fix(desc, 1) + ' descarte) m3');
    $('kPower').textContent = s.power;
    var sub = 'quedas de luz em 7 dias';
    if (s.resets) sub += ' - mais ' + s.resets + ' reinicio' + (s.resets > 1 ? 's' : '') + ' (OTA/botao/erro)';
    $('kPowerSub').textContent = sub;
    var note = { ntp: 'Horario real (internet).', celular: 'Horario real (acertado pelo celular).',
      estimado: 'Horario estimado: sem internet, o tempo em que o ESP ficou desligado nao aparece no grafico.' }[H.clock] || '';
    if (H.clock !== 'estimado' && s.estimated) note += ' Trechos antigos com horario estimado.';
    $('chNote').textContent = note;
  }

  function render() {
    if (!H) return;
    drawLine(); drawBars(); drawCards();
  }

  function load() {
    api('/api/history').then(function (r) { return r.json(); }).then(function (d) {
      H = d;
      $('charts').style.display = 'block';
      render();
      // Sem internet: usa a hora deste celular para acertar o relogio do ESP.
      if (!d.real && !timeSent) {
        timeSent = true;
        api('/api/time', { method: 'POST', body: new URLSearchParams({ epoch: Math.floor(Date.now() / 1000) }) })
          .then(function () { setTimeout(load, 1500); }).catch(function () {});
      }
    }).catch(function () {});
  }

  window.addEventListener('resize', render);
  bindHover();
  load();
  setInterval(load, 60000);
})();
