# Diseño: Calibración de antenna delay por unidad, persistida en NVS

**Fecha:** 2026-06-12
**Estado:** Aprobado (pendiente de plan de implementación)
**Branch objetivo:** feat/tag-workflow

## 1. Problema y motivación

El antenna delay del DW3000 es **por unidad**, no global. Absorbe todo el camino
de RF de cada placa concreta: tolerancias del matching network, longitud e
impedancia real de la traza, trim del cristal de referencia y dispersión del
propio DW3000. Por eso un valor hardcodeado (`TX_ANT_DLY = RX_ANT_DLY = 16371`
en `src/phy_config.h`, calibrado contra una PCB) no da la misma precisión en
otra PCB aunque comparta antena y layout. Qorvo recomienda calibrar cada placa
individualmente (APS014).

Solución: calibrar cada tag **una sola vez** en banco vía BLE NUS contra un
anchor a distancia conocida, resolver automáticamente el antenna delay y
persistirlo en NVS.

## 2. Contexto y decisiones tomadas

- **Contexto de ejecución:** banco de desarrollo, vía BLE NUS. Sin gestos de
  botón ni anchor portátil.
- **Algoritmo:** auto-solve por corrección lineal iterativa (una sola orden,
  sin recompilar).
- **NVS obligatorio:** sin valor válido en NVS el ranging **no arranca**; el tag
  avisa por BLE. El comando `cal` queda siempre disponible aunque el ranging
  esté detenido.
- **Split TX = RX** (práctica estándar Qorvo).
- **Umbral de aceptación:** 15 mm.
- **Muestreo:** N = 100 rangings por iteración.

## 3. Arquitectura / componentes

### Componentes nuevos

- **`src/cal.c` / `src/cal.h`** — lógica de calibración:
  - Máquina de estados: `IDLE → MEASURING → SOLVING → VERIFY → DONE/FAIL`.
  - `bool cal_init(void)` — monta NVS y carga el delay. Devuelve `false` si no
    hay valor válido.
  - `void cal_get_ant_dly(uint16_t *tx, uint16_t *rx)` — entrega el valor activo.
  - `int cal_run(uint32_t ref_mm)` — dispara el auto-solve (lo invoca el handler
    de comandos).
  - `int cal_clear(void)` — borra el registro NVS.

- **NUS RX en `src/ble_log.c`** — registrar `bt_nus_cb.received` (hoy el módulo
  es solo-TX) y enrutar la recepción a un **parser de comandos** ligero.
  Comandos: `cal <dist_mm>`, `cal clear`, `cal status`.

### Componentes modificados

- **`src/uwb_ss_initiator.c`** — leer el delay vía `cal_get_ant_dly()` en lugar
  de los `#define`. Si `cal_init()` falla, el hilo de ranging no entra al bucle:
  emite `CAL REQUIRED` por NUS y espera a un `cal` exitoso, tras el cual arranca
  el ranging sin necesidad de reset.

- **`src/phy_config.h`** — `TX_ANT_DLY` / `RX_ANT_DLY` pasan a ser únicamente el
  valor de referencia de fábrica documentado; ya no son el valor activo en
  runtime (se retiran del path de runtime).

- **`CMakeLists.txt`** — añadir `src/cal.c` con `target_sources(app PRIVATE ...)`.

- **`prj.conf`** — habilitar `CONFIG_NVS=y` y configuración de la partición
  `storage`.

## 4. Algoritmo de auto-solve (corrección lineal iterativa)

Relación lineal y monótona: subir el delay combinado baja la distancia reportada,
con pendiente teórica inicial ≈ **2.34 mm por unidad combinada**
(c · ½ · 15.65 ps, donde 15.65 ps = `DWT_TIME_UNITS`).

```
recibido ref_mm
repeat (máx 4 iteraciones):
    medir N = 100 rangings con el delay actual
    descartar timeouts/errores y outliers (rechazo por mediana ± k·MAD)
    err_mm = media(medido) - ref_mm
    si |err_mm| <= UMBRAL (15 mm): break  -> éxito
    delta_units = round(err_mm / 2.34)        # subir delay si medimos de más
    ant_dly_total += delta_units
    repartir equitativamente: TX = RX = ant_dly_total / 2
verify final:
    si converge -> escribir NVS
    si no       -> FAIL, NVS intacto
```

La iteración hace el método **robusto a un slope mal estimado**: la observación
empírica previa daba ~5 mm/unidad (≈2× la teoría); si el primer paso se queda
corto, el residuo se corrige en la siguiente vuelta (paso tipo Newton con
pendiente fija).

Reporte de progreso por NUS (mensajes ≤ 20 bytes por el límite de payload NUS):
`CAL it1 err=+82mm`, `CAL OK dly=16384 res=6mm`, `CAL FAIL res=120mm`.

## 5. Formato en NVS

Registro versionado con guardas de validez (evita usar un valor de otro PHY o de
flash corrupta):

```c
struct cal_record {
    uint32_t magic;       // constante fija, p.ej. 0xCA11B000
    uint8_t  version;     // versión del layout de la estructura
    uint8_t  phy_option;  // CONFIG_OPTION_07; invalida si cambia el PHY
    uint16_t tx_ant_dly;
    uint16_t rx_ant_dly;
    uint32_t ref_mm;      // distancia de referencia usada (trazabilidad)
    uint16_t residual_mm; // calidad lograda
    uint32_t crc32;       // integridad del registro
};
```

`cal_init()` valida `magic` + `version` + `phy_option` + `crc32`. Cualquier
fallo ⇒ "sin valor válido" ⇒ ranging bloqueado.

Requiere `CONFIG_NVS=y` y la partición `storage` (región Storage de 24 KB en
0x7a000, ya presente en el flash layout).

**Decisión abierta resuelta:** el valor queda atado a `CONFIG_OPTION_07`; se
guarda `phy_option` y se invalida si se cambia de PHY. Un valor por PHY queda
fuera de alcance en esta fase.

## 6. Manejo de errores

| Situación | Comportamiento |
|---|---|
| Anchor no responde durante el muestreo | Timeout global → `CAL FAIL no-resp`, NVS intacto |
| No converge en 4 iteraciones | `CAL FAIL res=...`, NVS intacto |
| NVS lleno / error de escritura | `CAL FAIL nvs`, se reporta |
| Comando malformado | `CAL ERR usage: cal <mm>` |
| Arranque sin valor válido en NVS | `CAL REQUIRED`, ranging bloqueado hasta `cal` exitoso |

## 7. Pruebas / verificación

- **Unitario en host:** extraer la matemática del solver a funciones puras
  testeables sin hardware (TDD):
  - `err_mm` → `delta_units`.
  - validación del `cal_record` (magic/version/phy/crc).
  - cálculo y verificación de CRC32.
  - rechazo de outliers (mediana ± k·MAD).
- **En banco:**
  - Calibrar a distancia conocida; confirmar `CAL OK` con residuo < umbral.
  - Power-cycle; confirmar que carga de NVS y la distancia reportada cae dentro
    del umbral.
  - `cal clear` + power-cycle; confirmar que vuelve a `CAL REQUIRED`.

## 8. Fuera de alcance

- Calibración multi-PHY (un valor por cada `CONFIG_OPTION`).
- Calibración en campo por gesto de botón o anchor portátil.
- Compensación por temperatura.
- Flujo de producción automatizado por script de PC.
