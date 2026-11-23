/*
 * max30102.c
 *
 *  Libreria MAX30102 (pulso + SpO2) para STM32 HAL. Solo I2C, sin pin INT,
 *  sin timers: el sensor muestrea con su reloj interno y nosotros vaciamos
 *  su FIFO desde el while(1) llamando a MAX30102_Task().
 *
 *  Procesamiento (por muestra, a 100 Hz):
 *   1. Quitar DC (media movil exponencial)  -> queda solo la parte pulsatil (AC)
 *   2. Filtro pasa-bajos Butterworth 2do orden, fc = 4 Hz (quita ruido)
 *   3. Deteccion de picos en la senal IR invertida con umbral adaptativo
 *      y periodo refractario (max 200 BPM)  -> latido, BPM
 *   4. En cada latido: R = (ACred/DCred) / (ACir/DCir)
 *      SpO2 = -45.060*R^2 + 30.354*R + 94.845  (curva empirica de Maxim)
 *
 *  OJO: valores orientativos, NO es un equipo medico.
 */

#include "max30102.h"
#include <string.h>

/* ------------------------- Parametros del algoritmo ---------------------- */
#define DC_ALPHA          0.02f                    /* constante de la media DC   */
#define SETTLE_SAMPLES    (MAX30102_FS * 3 / 2)    /* 1.5 s para estabilizar     */
#define MIN_BEAT_SAMPLES  (MAX30102_FS * 60 / 200) /* 200 BPM maximo             */
#define MAX_BEAT_SAMPLES  (MAX30102_FS * 60 / 40)  /*  40 BPM minimo             */
#define NO_BEAT_TIMEOUT   (MAX30102_FS * 3)        /* 3 s sin latidos -> reset   */
#define AVG_SIZE          4                        /* latidos a promediar        */
#define MIN_BEATS_VALID   3                        /* latidos antes de reportar  */

/* Biquad pasa-bajos Butterworth, fs = 100 Hz, fc = 4 Hz */
#define LP_B0  0.013359f
#define LP_B1  0.026718f
#define LP_B2  0.013359f
#define LP_A1 -1.647460f
#define LP_A2  0.700897f

typedef struct {
    float x1, x2, y1, y2;
} biquad_t;

/* ------------------------------ Estado interno --------------------------- */
static I2C_HandleTypeDef *max_i2c;

static uint8_t  fifo_buf[32 * 6];

static uint32_t raw_ir, raw_red;
static uint8_t  finger;
static uint32_t n_sample;          /* contador de muestras desde que hay dedo */

static float    dc_ir, dc_red;
static biquad_t lp_ir, lp_red;

static float    s1, s2;            /* muestras anteriores de la senal de picos */
static float    peak_avg;          /* amplitud media de los picos              */
static uint32_t last_beat_n;
static uint8_t  have_last_beat;
static uint8_t  beat_flag;

/* acumuladores entre latidos (para SpO2) */
static float    ir_min, ir_max, red_min, red_max;
static float    dc_ir_sum, dc_red_sum;
static uint32_t acc_n;

/* promedios */
static float    bpm_buf[AVG_SIZE];
static uint8_t  bpm_idx, bpm_cnt, bpm_reject;
static float    spo2_buf[AVG_SIZE];
static uint8_t  spo2_idx, spo2_cnt;

/* ------------------------------ I2C helpers ------------------------------ */
static HAL_StatusTypeDef max_write(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(max_i2c, MAX30102_ADDR << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, MAX30102_I2C_TIMEOUT);
}

static HAL_StatusTypeDef max_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(max_i2c, MAX30102_ADDR << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, MAX30102_I2C_TIMEOUT);
}

/* ------------------------------ Procesamiento ---------------------------- */
static float biquad(biquad_t *f, float x)
{
    float y = LP_B0 * x + LP_B1 * f->x1 + LP_B2 * f->x2
            - LP_A1 * f->y1 - LP_A2 * f->y2;
    f->x2 = f->x1; f->x1 = x;
    f->y2 = f->y1; f->y1 = y;
    return y;
}

static void reset_beat_acc(void)
{
    ir_min = red_min =  1e9f;
    ir_max = red_max = -1e9f;
    dc_ir_sum = dc_red_sum = 0.0f;
    acc_n = 0;
}

static void reset_results(void)
{
    peak_avg       = 0.0f;
    have_last_beat = 0;
    bpm_idx = bpm_cnt = bpm_reject = 0;
    spo2_idx = spo2_cnt = 0;
    reset_beat_acc();
}

static void reset_all(void)
{
    n_sample = 0;
    memset(&lp_ir,  0, sizeof(lp_ir));
    memset(&lp_red, 0, sizeof(lp_red));
    s1 = s2 = 0.0f;
    reset_results();
}

static float avg(const float *b, uint8_t cnt)
{
    float s = 0.0f;
    for (uint8_t i = 0; i < cnt; i++) s += b[i];
    return cnt ? s / cnt : 0.0f;
}

static void on_beat(uint32_t beat_n, float amplitude)
{
    beat_flag = 1;

    /* umbral adaptativo */
    peak_avg = (peak_avg == 0.0f) ? amplitude : 0.75f * peak_avg + 0.25f * amplitude;

    if (have_last_beat) {
        uint32_t interval = beat_n - last_beat_n;

        if (interval >= MIN_BEAT_SAMPLES && interval <= MAX_BEAT_SAMPLES) {
            float bpm = 60.0f * MAX30102_FS / (float)interval;
            uint8_t ok = 1;

            /* rechazo de latidos raros (>30% distinto al promedio) */
            if (bpm_cnt >= MIN_BEATS_VALID) {
                float a = avg(bpm_buf, bpm_cnt);
                float d = bpm - a;
                if (d < 0) d = -d;
                if (d > 0.30f * a) {
                    ok = 0;
                    if (++bpm_reject >= 3) {      /* el pulso cambio de verdad */
                        bpm_idx = bpm_cnt = bpm_reject = 0;
                        ok = 1;
                    }
                }
            }

            if (ok) {
                bpm_reject = 0;
                bpm_buf[bpm_idx] = bpm;
                bpm_idx = (bpm_idx + 1) % AVG_SIZE;
                if (bpm_cnt < AVG_SIZE) bpm_cnt++;

                /* ---- SpO2 con el ciclo completo entre dos latidos ---- */
                float pp_ir  = ir_max  - ir_min;
                float pp_red = red_max - red_min;
                float dci    = dc_ir_sum  / (float)acc_n;
                float dcr    = dc_red_sum / (float)acc_n;

                if (acc_n > 0 && pp_ir > 0.0f && pp_red > 0.0f && dci > 0.0f && dcr > 0.0f) {
                    float R    = (pp_red / dcr) / (pp_ir / dci);
                    float spo2 = -45.060f * R * R + 30.354f * R + 94.845f;
                    if (spo2 > 100.0f) spo2 = 100.0f;
                    if (spo2 >= 70.0f) {
                        spo2_buf[spo2_idx] = spo2;
                        spo2_idx = (spo2_idx + 1) % AVG_SIZE;
                        if (spo2_cnt < AVG_SIZE) spo2_cnt++;
                    }
                }
            }
        }
    }

    last_beat_n    = beat_n;
    have_last_beat = 1;
    reset_beat_acc();
}

static void process_sample(uint32_t red, uint32_t ir)
{
    raw_red = red;
    raw_ir  = ir;

    /* ---- deteccion de dedo ---- */
    if (ir < MAX30102_FINGER_THRESHOLD) {
        finger = 0;
        return;
    }
    if (!finger) {                 /* dedo recien puesto: arrancar de cero */
        finger = 1;
        reset_all();
        dc_ir  = (float)ir;
        dc_red = (float)red;
    }
    n_sample++;

    /* ---- 1. quitar DC ---- */
    dc_ir  += DC_ALPHA * ((float)ir  - dc_ir);
    dc_red += DC_ALPHA * ((float)red - dc_red);

    /* ---- 2. pasa-bajos ---- */
    float ac_ir  = biquad(&lp_ir,  (float)ir  - dc_ir);
    float ac_red = biquad(&lp_red, (float)red - dc_red);

    if (n_sample < SETTLE_SAMPLES) {   /* esperar que los filtros se asienten */
        s2 = s1; s1 = -ac_ir;
        last_beat_n = n_sample;
        return;
    }

    /* acumuladores para SpO2 */
    if (ac_ir  < ir_min)  ir_min  = ac_ir;
    if (ac_ir  > ir_max)  ir_max  = ac_ir;
    if (ac_red < red_min) red_min = ac_red;
    if (ac_red > red_max) red_max = ac_red;
    dc_ir_sum  += dc_ir;
    dc_red_sum += dc_red;
    acc_n++;

    /* ---- 3. deteccion de picos (el latido baja la luz IR -> invertimos) ---- */
    float s = -ac_ir;
    float floor_thr = dc_ir * 0.0002f;           /* umbral minimo absoluto */
    float thr = 0.5f * peak_avg;
    if (thr < floor_thr) thr = floor_thr;

    if (s1 > s2 && s1 >= s && s1 > thr) {        /* maximo local en n-1 */
        uint32_t peak_n = n_sample - 1;
        if (!have_last_beat || (peak_n - last_beat_n) >= MIN_BEAT_SAMPLES) {
            on_beat(peak_n, s1);
        }
    }
    s2 = s1;
    s1 = s;

    /* ---- sin latidos durante mucho tiempo: reiniciar resultados ---- */
    if ((n_sample - last_beat_n) > NO_BEAT_TIMEOUT) {
        reset_results();
        last_beat_n = n_sample;
    }
}

/* ---------------------------- Recuperacion del bus ----------------------- */
static void bus_delay(void)            /* ~5-10 us -> SCL de ~50-100 kHz */
{
    for (volatile uint32_t i = 0; i < 100; i++);
}

/* Pines posibles de cada I2C en el STM32F401/F411 (puerto, pin, funcion AF).
 * Se busca cual de ellos dejo configurado CubeMX en HAL_I2C_MspInit().     */
typedef struct {
    I2C_TypeDef  *i2c;
    uint8_t       is_sda;      /* 0 = SCL, 1 = SDA */
    GPIO_TypeDef *port;
    uint8_t       pin;         /* numero de pin 0..15 */
    uint8_t       af;
} i2c_pin_cand_t;

static const i2c_pin_cand_t i2c_cands[] = {
    { I2C1, 0, GPIOB,  6, 4 }, { I2C1, 0, GPIOB,  8, 4 },
    { I2C1, 1, GPIOB,  7, 4 }, { I2C1, 1, GPIOB,  9, 4 },
#ifdef I2C2
    { I2C2, 0, GPIOB, 10, 4 },
    { I2C2, 1, GPIOB,  3, 9 }, { I2C2, 1, GPIOB, 11, 4 },
#endif
#ifdef I2C3
    { I2C3, 0, GPIOA,  8, 4 },
    { I2C3, 1, GPIOC,  9, 4 }, { I2C3, 1, GPIOB,  4, 9 }, { I2C3, 1, GPIOB, 8, 9 },
#endif
};

static GPIO_TypeDef *scl_port, *sda_port;
static uint16_t      scl_pin,   sda_pin;

/* 1 si el pin esta en modo alternativo con esa funcion AF */
static uint8_t pin_is_af(GPIO_TypeDef *port, uint8_t pin, uint8_t af)
{
    uint32_t mode = (port->MODER >> (pin * 2U)) & 0x3U;
    uint32_t afr  = (port->AFR[pin >> 3] >> ((pin & 7U) * 4U)) & 0xFU;
    return (mode == 0x2U) && (afr == af);
}

/* Busca SCL y SDA del I2C elegido. Devuelve HAL_OK si encontro ambos. */
static HAL_StatusTypeDef find_i2c_pins(I2C_TypeDef *inst)
{
    scl_port = sda_port = NULL;
    for (uint32_t i = 0; i < sizeof(i2c_cands) / sizeof(i2c_cands[0]); i++) {
        const i2c_pin_cand_t *c = &i2c_cands[i];
        if (c->i2c != inst || !pin_is_af(c->port, c->pin, c->af)) continue;
        if (c->is_sda) { sda_port = c->port; sda_pin = (uint16_t)(1U << c->pin); }
        else           { scl_port = c->port; scl_pin = (uint16_t)(1U << c->pin); }
    }
    return (scl_port && sda_port) ? HAL_OK : HAL_ERROR;
}

#define SCL_H()  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET)
#define SCL_L()  HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET)
#define SDA_H()  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_SET)
#define SDA_L()  HAL_GPIO_WritePin(sda_port, sda_pin, GPIO_PIN_RESET)
#define SDA_RD() HAL_GPIO_ReadPin(sda_port, sda_pin)
#define SCL_RD() HAL_GPIO_ReadPin(scl_port, scl_pin)

HAL_StatusTypeDef MAX30102_BusRecovery(void)
{
    I2C_HandleTypeDef *hi2c = &MAX30102_I2C;
    GPIO_InitTypeDef g = {0};
    HAL_StatusTypeDef res;

    /* 0. averiguar los pines (antes del DeInit, que borra su configuracion) */
    if (find_i2c_pins(hi2c->Instance) != HAL_OK) return HAL_ERROR;

    /* 1. soltar el periferico I2C (HAL_I2C_MspDeInit libera los pines) */
    HAL_I2C_DeInit(hi2c);

    /* 2. SCL y SDA como GPIO open-drain (los pull-ups del modulo los suben).
     *    Se pone el latch en 1 ANTES de pasarlos a salida para no generar
     *    un pulso a 0 (que el sensor veria como un START falso).          */
    SDA_H();
    SCL_H();
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = scl_pin;
    HAL_GPIO_Init(scl_port, &g);
    g.Pin   = sda_pin;
    HAL_GPIO_Init(sda_port, &g);
    bus_delay();

    res = HAL_ERROR;
    for (uint8_t intento = 0; intento < 3 && res != HAL_OK; intento++) {

        /* 3. SIEMPRE 9 pulsos de reloj con SDA suelto (en 1):
         *    el sensor termina el byte que estaba enviando y en el
         *    9no pulso (ACK) ve un NACK -> deja de transmitir.
         *    (No se corta al ver SDA=1: el siguiente bit podria ser 0) */
        for (uint8_t i = 0; i < 9; i++) {
            SCL_L(); bus_delay();
            SCL_H(); bus_delay();
        }

        /* 4. condicion STOP: SDA sube mientras SCL esta en 1 */
        SCL_L(); bus_delay();
        SDA_L(); bus_delay();
        SCL_H(); bus_delay();
        SDA_H(); bus_delay();

        if (SDA_RD() == GPIO_PIN_SET && SCL_RD() == GPIO_PIN_SET) res = HAL_OK;
    }

    /* 5. devolver los pines al I2C (MspInit) y resetear el periferico */
    if (HAL_I2C_Init(hi2c) != HAL_OK) res = HAL_ERROR;

    return res;
}

/* ---------------------------------- API ---------------------------------- */
HAL_StatusTypeDef MAX30102_Init(void)
{
    uint8_t  v;
    uint32_t t0;

    max_i2c = &MAX30102_I2C;

    if (HAL_I2C_IsDeviceReady(max_i2c, MAX30102_ADDR << 1, 3, MAX30102_I2C_TIMEOUT) != HAL_OK)
        return HAL_ERROR;

    if (max_read(MAX30102_REG_PART_ID, &v, 1) != HAL_OK || v != MAX30102_PART_ID_VALUE)
        return HAL_ERROR;

    /* reset por software y esperar a que termine */
    if (max_write(MAX30102_REG_MODE_CONFIG, 0x40) != HAL_OK) return HAL_ERROR;
    t0 = HAL_GetTick();
    do {
        if (max_read(MAX30102_REG_MODE_CONFIG, &v, 1) != HAL_OK) return HAL_ERROR;
        if ((HAL_GetTick() - t0) > 100) return HAL_TIMEOUT;
    } while (v & 0x40);

    /* limpiar flags de interrupcion (no usamos el pin INT) */
    max_read(MAX30102_REG_INT_STATUS1, &v, 1);
    max_read(MAX30102_REG_INT_STATUS2, &v, 1);
    max_write(MAX30102_REG_INT_ENABLE1, 0x00);
    max_write(MAX30102_REG_INT_ENABLE2, 0x00);

    /* vaciar FIFO */
    max_write(MAX30102_REG_FIFO_WR_PTR, 0x00);
    max_write(MAX30102_REG_OVF_COUNTER, 0x00);
    max_write(MAX30102_REG_FIFO_RD_PTR, 0x00);

    /* FIFO: promedio de 4 muestras, rollover ON  -> 0b010 1 1111 */
    max_write(MAX30102_REG_FIFO_CONFIG, 0x5F);

    /* SpO2: ADC 4096 nA (01), 400 sps (011), 411 us / 18 bits (11)
     * -> 400 / 4 (promedio) = 100 muestras/s efectivas             */
    max_write(MAX30102_REG_SPO2_CONFIG, 0x2F);

    max_write(MAX30102_REG_LED1_PA, MAX30102_LED_RED_PA);
    max_write(MAX30102_REG_LED2_PA, MAX30102_LED_IR_PA);

    /* modo SpO2 (Red + IR) */
    if (max_write(MAX30102_REG_MODE_CONFIG, 0x03) != HAL_OK) return HAL_ERROR;

    finger = 0;
    beat_flag = 0;
    reset_all();
    return HAL_OK;
}

uint8_t MAX30102_Task(void)
{
    uint8_t ptr[3];   /* WR_PTR, OVF_COUNTER, RD_PTR (registros 0x04..0x06) */
    uint8_t n;

    if (max_i2c == NULL) return 0;
    if (max_read(MAX30102_REG_FIFO_WR_PTR, ptr, 3) != HAL_OK) return 0;

    n = (uint8_t)((ptr[0] - ptr[2]) & 0x1F);
    if (n == 0 && ptr[1] != 0) n = 32;          /* FIFO llena con overflow */
    if (n == 0) return 0;

    if (max_read(MAX30102_REG_FIFO_DATA, fifo_buf, (uint16_t)n * 6) != HAL_OK) return 0;

    for (uint8_t i = 0; i < n; i++) {
        uint8_t *p = &fifo_buf[i * 6];
        uint32_t led1 = (((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2]) & 0x3FFFF;
        uint32_t led2 = (((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 8) | p[5]) & 0x3FFFF;
#if MAX30102_SWAP_RED_IR
        process_sample(led2, led1);
#else
        process_sample(led1, led2);   /* LED1 = Red, LED2 = IR */
#endif
    }
    return n;
}

uint8_t MAX30102_FingerPresent(void)
{
    return finger;
}

uint16_t MAX30102_GetBPM(void)
{
    if (!finger || bpm_cnt < MIN_BEATS_VALID) return 0;
    return (uint16_t)(avg(bpm_buf, bpm_cnt) + 0.5f);
}

uint8_t MAX30102_GetSpO2(void)
{
    if (!finger || spo2_cnt < MIN_BEATS_VALID) return 0;
    return (uint8_t)(avg(spo2_buf, spo2_cnt) + 0.5f);
}

uint8_t MAX30102_BeatDetected(void)
{
    uint8_t b = beat_flag;
    beat_flag = 0;
    return b;
}

uint32_t MAX30102_GetRawIR(void)  { return raw_ir;  }
uint32_t MAX30102_GetRawRed(void) { return raw_red; }
