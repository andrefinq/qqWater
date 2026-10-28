# qqWater

Projeto para QQ de controle do sistema de águas (ESP32, ESP-IDF v6.0.1).

## Gravação

### Primeira vez com a tabela de partições OTA (pelo cabo USB)

A versão 1.5.0 troca a tabela de partições para permitir atualização pela
rede. Essa primeira gravação precisa ser pelo USB:

```bash
idf.py -p COM6 flash
```

Use `flash`, **nunca** `erase-flash`: a área de dados (NVS) continua no mesmo
lugar, então totalizadores, rede Wi-Fi cadastrada e configurações são
preservados. Um `erase-flash` apagaria tudo.

### Atualizações seguintes (pela página)

1. Atualize o número em `version.txt` (ex.: `1.5.0` → `1.5.1`).
2. `idf.py build`
3. Na página, seção **Firmware → Atualizar firmware**, envie o arquivo
   `build/qqWater.bin`.

O ESP grava a versão nova na outra partição e reinicia sozinho assim que a
automação estiver desligada ou em espera (decantação ou janela de ciclo).
A versão nova só é confirmada depois de 1 minuto rodando normal; se travar
ou reiniciar antes disso, o ESP volta automaticamente para a versão anterior.

Mudanças em `partitions.csv` ou no bootloader **não** podem ser feitas pela
página — exigem gravação pelo USB.
