/*
 * max30102.h
 *
 *  Libreria MAX30102 (pulso + SpO2) para STM32 HAL.
 *  Conexion: solo I2C (SDA, SCL, VCC, GND). No usa el pin INT ni timers.
 *
 *  Funcionamiento:
 *   - El MAX30102 muestrea solo, con su reloj interno (MAX30102_FS = 100 Hz),
 *     y guarda las muestras en su FIFO interna de 32 posiciones.
 *   - MAX30102_Task() se llama en el while(1): lee la FIFO por I2C y procesa.
 *     Debe llamarse al menos cada ~300 ms (32 muestras / 100 Hz = 320 ms)
 *     para no perder datos. HAL_GetTick() solo se usa para timeouts.
 *
 *  OJO: valores orientativos, NO es un equipo medico.
 *
 * ============================== COMO SE USA ================================
 *
 *  1) CubeMX:
 *     - Activar un I2C (I2C1, I2C2 o I2C3) en modo "I2C", Fast Mode 400 kHz
 *       (si hay fallos con cables largos usar Standard Mode 100 kHz).
 *     - Activar un UART para enviar los datos (ej. USART1 a 115200).
 *     - No hace falta timer, ni interrupciones, ni el pin INT del sensor.
 *
 *  2) En este .h configurar:
 *     - MAX30102_I2C        -> el handle del I2C usado: hi2c1, hi2c2 o hi2c3
 *                              (los pines SCL/SDA se detectan solos)
 *     - MAX30102_LED_RED_PA / MAX30102_LED_IR_PA -> corriente de los LEDs
 *     - MAX30102_FINGER_THRESHOLD -> umbral IR para detectar el dedo
 *     - MAX30102_SWAP_RED_IR -> poner 1 si la SpO2 sale absurda
 *
 *  3) En main.c:
 *
 *     USER CODE BEGIN Includes
 *         #include "max30102.h"
 *         #include "uart.h"
 *         #include "stdio.h"
 *
 *     USER CODE BEGIN PV
 *         char txt[80];
 *         uint32_t t_print = 0;
 *
 *     USER CODE BEGIN 2   (despues de MX_I2Cx_Init y MX_USARTx_UART_Init)
 *         MAX30102_BusRecovery();              // libera el bus si quedo colgado
 *         if (MAX30102_Init() != HAL_OK)
 *         {
 *             uartx_write_text(&huart1, "MAX30102 NO ENCONTRADO\r\n");
 *             while (1);
 *         }
 *
 *     
     
 
     (dentro del while(1), SIN HAL_Delay largos)
 *         MAX30102_Task();                     // leer FIFO y procesar
 *
 *         if (MAX30102_BeatDetected())         // 1 vez por latido
 *             uartx_write_text(&huart1, "<3\r\n");
 *
 *         if (HAL_GetTick() - t_print >= 1000) // cada 1 s
 *         {
 *             t_print = HAL_GetTick();
 *             if (!MAX30102_FingerPresent())
 *                 sprintf(txt, "Coloque el dedo\r\n");
 *             else if (MAX30102_GetBPM() == 0)
 *                 sprintf(txt, "Midiendo...\r\n");
 *             else
 *                 sprintf(txt, "BPM=%u SpO2=%u%%\r\n",
 *                         MAX30102_GetBPM(), MAX30102_GetSpO2());
 *             uartx_write_text(&huart1, txt);
 *         }
 *
 *  Resultados: MAX30102_GetBPM() y MAX30102_GetSpO2() devuelven 0 mientras
 *  no hay dedo o todavia no hay suficientes latidos (~5 s al poner el dedo).
 * ===========================================================================
 */

#ifndef LIBRERIAS_MAX30102_H_
#define LIBRERIAS_MAX30102_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Configuracion ----------------------------- */
#define MAX30102_ADDR            0x57      /* direccion 7 bits                 */
#define MAX30102_I2C_TIMEOUT     50        /* ms                               */

/* I2C usado: el handle generado por CubeMX (hi2c1, hi2c2, hi2c3).
 * Los pines SCL/SDA se detectan solos desde la configuracion de CubeMX.     */
#define MAX30102_I2C              hi2c1    /* I2C usado (I2C_HandleTypeDef)    */





/* Corriente de los LEDs: 0x00..0xFF, ~0.2 mA por paso (0x24 = 7.2 mA).
 * Si la lectura IR satura (~262143) bajala; si es muy baja con el dedo, subila */
#define MAX30102_LED_RED_PA      0x24
#define MAX30102_LED_IR_PA       0x24

/* Umbral IR (crudo, 18 bits) para decidir que hay un dedo encima */
#define MAX30102_FINGER_THRESHOLD 50000UL

/* Algunos modulos chinos traen los LED Red/IR invertidos.
 * Si la SpO2 sale absurda (muy baja / 100 siempre) prOBAR poner 1 */
#define MAX30102_SWAP_RED_IR     0


/* Frecuencia de muestreo efectiva (400 Hz internos / promedio de 4).
 * Los filtros estan calculados para 100 Hz: no cambiar sin recalcularlos. */
#define MAX30102_FS              100

/* ------------------------------- Registros ------------------------------- */
#define MAX30102_REG_INT_STATUS1  0x00
#define MAX30102_REG_INT_STATUS2  0x01
#define MAX30102_REG_INT_ENABLE1  0x02
#define MAX30102_REG_INT_ENABLE2  0x03
#define MAX30102_REG_FIFO_WR_PTR  0x04
#define MAX30102_REG_OVF_COUNTER  0x05
#define MAX30102_REG_FIFO_RD_PTR  0x06
#define MAX30102_REG_FIFO_DATA    0x07
#define MAX30102_REG_FIFO_CONFIG  0x08
#define MAX30102_REG_MODE_CONFIG  0x09
#define MAX30102_REG_SPO2_CONFIG  0x0A
#define MAX30102_REG_LED1_PA      0x0C   /* Red */
#define MAX30102_REG_LED2_PA      0x0D   /* IR  */
#define MAX30102_REG_REV_ID       0xFE
#define MAX30102_REG_PART_ID      0xFF
#define MAX30102_PART_ID_VALUE    0x15
extern I2C_HandleTypeDef MAX30102_I2C ;
/* --------------------------------- API ----------------------------------- */

/* Libera el bus I2C si un esclavo quedo colgado con SDA en 0 (por ej. tras
 * resetear el STM32 a mitad de una lectura). Genera 9 pulsos en SCL + STOP
 * y vuelve a inicializar el I2C. Llamar despues de MX_I2Cx_Init().
 * Devuelve HAL_OK si al final SDA y SCL quedaron libres (en 1),
 * HAL_ERROR si siguen en 0 o si no encontro los pines del I2C.             */
HAL_StatusTypeDef MAX30102_BusRecovery(void);

/* Resetea y configura el sensor. Devuelve HAL_OK si encontro el MAX30102. */
HAL_StatusTypeDef MAX30102_Init(void);

/* Llamar continuamente en el while(1). Devuelve cuantas muestras proceso. */
uint8_t  MAX30102_Task(void);

/* 1 si hay dedo sobre el sensor */
uint8_t  MAX30102_FingerPresent(void);

/* Pulso en BPM (0 = todavia no valido) */
uint16_t MAX30102_GetBPM(void);

/* SpO2 en % (0 = todavia no valido) */
uint8_t  MAX30102_GetSpO2(void);

/* Devuelve 1 una sola vez por cada latido detectado (para parpadear un LED, etc.) */
uint8_t  MAX30102_BeatDetected(void);

/* Ultimas lecturas crudas (utiles para depurar / graficar) */
uint32_t MAX30102_GetRawIR(void);
uint32_t MAX30102_GetRawRed(void);

#ifdef __cplusplus
}
#endif

#endif /* LIBRERIAS_MAX30102_H_ */
