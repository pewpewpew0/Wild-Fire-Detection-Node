/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   L476_Node — Wildfire Detection Node
  *          Target: STM32L476RGTx
  *	 Show last update date and time here, last update: Mon. May 11 2026 2:32 PST
  *  Adapted from blinky (STM32L412RBTxP). Key changes:
  *    - huart1 (USART1 PB6/PB7) = LoRa        (was huart2 on L412 PCB)
  *    - huart2 (USART2 PA2/PA3) = CO2 S88      (was huart1 on L412 PCB)
  *    - huart3 (USART3 PC4/PC5) = GPS          (new)
  *    - S88 DIR pin: PB13                       (was PB8, now I2C1_SCL)
  *    - Removed MX_ADC1_Init()                 (no ADC on WFD_Node PCB)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "i2c.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ======== configuration ======== */
#define SEND_PERIOD_MS       10000
#define LOCATION_PERIOD_MS   50000      /* GPS location packet interval in ms (20 s demo; 600000 for production) */

/* SenseAir S88 (USART2, huart2) Modbus RTU */
#define S88_BAUD_RATE        9600
#define S88_MODBUS_ADDR      0xFE   // was 0xFE
#define S88_READ_INPUT_REG   0x04
#define S88_CO2_REG          0x0003
#define S88_DIR_GPIO_Port    GPIOB
#define S88_DIR_Pin          GPIO_PIN_13   /* PB13 — was PB8 on L412 PCB */

/* LoRa (USART1, huart1) */
#define LORA_CMD_TIMEOUT_MS  500
#define LORA_ACK_TIMEOUT_MS  10000
#define LORA_MAX_RETRIES     5
#define LORA_BAND_MHZ        915
#define LORA_NETWORK_ID      3
#define LORA_ADDRESS         3
#define LORA_DEST_ADDRESS    0
#define MY_PARENT_ADDRESS    0          /* Step 4: next-hop toward sink. Edit per node.
                                         * Node A (leaf):      LORA_ADDRESS=1, MY_PARENT_ADDRESS=2
                                         * Node B (forwarder): LORA_ADDRESS=2, MY_PARENT_ADDRESS=0
                                         * Node C (leaf):      LORA_ADDRESS=3, MY_PARENT_ADDRESS=2 */
#define LORA_RF_POWER        22

/* BME280 (I2C1, hi2c1, PB8/PB9) */
#define BME280_I2C_ADDR      (0x76 << 1)
#define BME280_REG_ID         0xD0
#define BME280_REG_RESET      0xE0
#define BME280_REG_CTRL_HUM   0xF2
#define BME280_REG_CTRL_MEAS  0xF4
#define BME280_REG_CONFIG     0xF5
#define BME280_REG_PRESS_MSB  0xF7

/* DFRobot SEN0466 Gravity Gas Sensor (I2C2, hi2c2, PB10/PB11) */


/* ======== protocol ======== */
#define PKT_TYPE_DATA        0x01       /* packet category: node -> hub */
#define PKT_TYPE_REPLY       0x02       /* packet category: hub -> node (for reference) */
#define ACK_MSG              1
#define NACK_MSG             0

/* Step 3: expanded-header packet types and TTL.
 * PKT_TYPE_ACK aliases PKT_TYPE_REPLY (same value 0x02) — both names compile,
 * dead lora_send_with_ack keeps using ACK_MSG/NACK_MSG/PKT_TYPE_REPLY. */
#define PKT_TYPE_ACK         0x02       /* hub -> originator (success)   */
#define PKT_TYPE_NACK        0x03       /* hub -> originator (CRC fail)  */
#define PKT_TYPE_BEACON      0x04       /* Step 5 placeholder            */
#define LORA_INITIAL_TTL     4

/* ======== BME280 calibration ======== */
typedef struct {
    uint16_t dig_T1; int16_t  dig_T2; int16_t  dig_T3;
    uint16_t dig_P1; int16_t  dig_P2; int16_t  dig_P3;
    int16_t  dig_P4; int16_t  dig_P5; int16_t  dig_P6;
    int16_t  dig_P7; int16_t  dig_P8; int16_t  dig_P9;
    uint8_t  dig_H1; int16_t  dig_H2; uint8_t  dig_H3;
    int16_t  dig_H4; int16_t  dig_H5; int8_t   dig_H6;
} BME280_Calib;

static BME280_Calib g_bme_calib;
static int32_t      g_bme_t_fine = 0;

/* ======== global sensor state ======== */
static float    g_temperature_c = 0.0f;
static float    g_humidity_rh   = 0.0f;
static float    g_pressure_pa   = 0.0f;
static float    g_co_ppm        = 0.0f;
static uint16_t g_co2_ppm       = 0;
static uint8_t  g_bme_ready     = 0;
static uint8_t  g_gas_ready     = 0;
static uint8_t  g_s88_ready     = 0;
static uint8_t  g_co2_err       = 0;

static float   g_gps_lon   = 0.0f;
static float   g_gps_lat   = 0.0f;
static uint8_t g_gps_valid = 0;

static uint8_t g_seq_bit = 0;
static uint16_t g_pkt_id = 0;   /* Step 3: per-originator pkt_id, increments on ACK success */

/* ======== LoRa IRQ-driven RX ring buffer (Step 1) ========
 * Single-producer (USART1 ISR via HAL_UART_RxCpltCallback) /
 * single-consumer (main thread via lora_rx_pop).
 * head/tail are volatile uint16_t — single-word access is atomic on
 * Cortex-M4 so no critical section is needed for SPSC.
 */
#define LORA_RX_RING_SIZE    256u
#define LORA_RX_RING_MASK    (LORA_RX_RING_SIZE - 1u)

static volatile uint8_t  lora_rx_ring[LORA_RX_RING_SIZE];
static volatile uint16_t lora_rx_head = 0;   /* written by ISR  */
static volatile uint16_t lora_rx_tail = 0;   /* written by main */
static uint8_t           lora_rx_byte = 0;   /* HAL single-byte landing pad */

/* ======== SenseAir S88 — now on huart2 (USART2, PA2/PA3) ======== */

/* ======== SenseAir S88 — now on huart2 (USART2, PA2/PA3) ======== */
static uint16_t s88_crc16(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else               crc >>= 1;
        }
    }
    return crc;
}

static void s88_set_tx(void) { HAL_GPIO_WritePin(S88_DIR_GPIO_Port, S88_DIR_Pin, GPIO_PIN_SET);   }
static void s88_set_rx(void) { HAL_GPIO_WritePin(S88_DIR_GPIO_Port, S88_DIR_Pin, GPIO_PIN_RESET); }

static void s88_uart_clear_errors(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);
}

static uint8_t s88_read_co2(uint16_t *co2_ppm)
{
    uint8_t cmd[8], rx[7];
    cmd[0] = S88_MODBUS_ADDR; cmd[1] = S88_READ_INPUT_REG;
    cmd[2] = (uint8_t)(S88_CO2_REG >> 8); cmd[3] = (uint8_t)(S88_CO2_REG & 0xFF);
    cmd[4] = 0x00; cmd[5] = 0x01;
    uint16_t crc = s88_crc16(cmd, 6);
    cmd[6] = (uint8_t)(crc & 0xFF); cmd[7] = (uint8_t)((crc >> 8) & 0xFF);

    s88_set_tx();
    if (HAL_UART_Transmit(&huart2, cmd, 8, 200) != HAL_OK) { s88_set_rx(); g_co2_err = 1; return 0; }
    s88_set_rx();
    s88_uart_clear_errors();
    if (HAL_UART_Receive(&huart2, rx, 7, 2000) != HAL_OK) { g_co2_err = 2; return 0; }
    if (rx[1] != S88_READ_INPUT_REG || rx[2] != 0x02)     { g_co2_err = 3; return 0; }

    uint16_t crc_calc = s88_crc16(rx, 5);
    uint16_t crc_recv = (uint16_t)rx[5] | ((uint16_t)rx[6] << 8);
    if (crc_calc != crc_recv)                              { g_co2_err = 4; return 0; }
    g_co2_err = 0;

    *co2_ppm = (uint16_t)((rx[3] << 8) | rx[4]);
    return 1;
}

static uint8_t s88_init(void)
{
    uint16_t tmp = 0;
    HAL_Delay(10000);
    s88_set_rx();
    g_s88_ready = s88_read_co2(&tmp) ? 1 : 0;
    return g_s88_ready;
}

/* ======== DFRobot CO gas sensor — huart3 (USART3, PB10/PB11) @ 9600 ======== */
static uint8_t co_checksum(const uint8_t *data, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 1; i < (uint8_t)(len - 1); i++) sum += data[i];
    return (uint8_t)((~sum) + 1);
}

static uint8_t gas_init_co_only(void)
{
    /* switch to Q&A mode: FF 01 78 04 00 00 00 00 83 */
    uint8_t cmd[9] = {0xFF, 0x01, 0x78, 0x04, 0x00, 0x00, 0x00, 0x00, 0x83};
    uint8_t ack[9];
    HAL_Delay(500);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    if (HAL_UART_Transmit(&huart3, cmd, 9, 200) != HAL_OK) return 0;
    if (HAL_UART_Receive(&huart3, ack, 9, 1500) != HAL_OK) return 0;
    if (ack[0] != 0xFF || ack[1] != 0x78 || ack[2] != 0x01) return 0;
    return 1;
}

static uint8_t gas_read_co_ppm(float *co_ppm)
{
    /* request: FF 01 86 00 00 00 00 00 79 */
    uint8_t cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    uint8_t rx[9];
    if (HAL_UART_Transmit(&huart3, cmd, 9, 200) != HAL_OK) return 0;
    if (HAL_UART_Receive(&huart3, rx, 9, 1000) != HAL_OK) return 0;
    if (rx[0] != 0xFF || rx[1] != 0x86) return 0;
    if (co_checksum(rx, 9) != rx[8]) return 0;
    *co_ppm = (float)((rx[2] << 8) | rx[3]);
    return 1;
}

/* ======== BME280 — hi2c1 (PB8/PB9) ======== */
static int bme280_read_regs(uint8_t reg, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(&hi2c1, BME280_I2C_ADDR, &reg, 1, 100) != HAL_OK) return -1;
    if (HAL_I2C_Master_Receive (&hi2c1, BME280_I2C_ADDR, data, len, 100) != HAL_OK) return -1;
    return 0;
}

static int bme280_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return (HAL_I2C_Master_Transmit(&hi2c1, BME280_I2C_ADDR, buf, 2, 100) == HAL_OK) ? 0 : -1;
}

static uint8_t bme280_read_calibration(void)
{
    uint8_t buf1[26], buf2[7];
    if (bme280_read_regs(0x88, buf1, 26) != 0) return 0;
    if (bme280_read_regs(0xE1, buf2,  7) != 0) return 0;

    g_bme_calib.dig_T1 = (uint16_t)((buf1[1] << 8) | buf1[0]);
    g_bme_calib.dig_T2 = (int16_t) ((buf1[3] << 8) | buf1[2]);
    g_bme_calib.dig_T3 = (int16_t) ((buf1[5] << 8) | buf1[4]);
    g_bme_calib.dig_P1 = (uint16_t)((buf1[7] << 8) | buf1[6]);
    g_bme_calib.dig_P2 = (int16_t) ((buf1[9] << 8) | buf1[8]);
    g_bme_calib.dig_P3 = (int16_t) ((buf1[11]<< 8) | buf1[10]);
    g_bme_calib.dig_P4 = (int16_t) ((buf1[13]<< 8) | buf1[12]);
    g_bme_calib.dig_P5 = (int16_t) ((buf1[15]<< 8) | buf1[14]);
    g_bme_calib.dig_P6 = (int16_t) ((buf1[17]<< 8) | buf1[16]);
    g_bme_calib.dig_P7 = (int16_t) ((buf1[19]<< 8) | buf1[18]);
    g_bme_calib.dig_P8 = (int16_t) ((buf1[21]<< 8) | buf1[20]);
    g_bme_calib.dig_P9 = (int16_t) ((buf1[23]<< 8) | buf1[22]);

    uint8_t h1 = 0;
    if (bme280_read_regs(0xA1, &h1, 1) != 0) return 0;
    g_bme_calib.dig_H1 = h1;
    g_bme_calib.dig_H2 = (int16_t)((buf2[1] << 8) | buf2[0]);
    g_bme_calib.dig_H3 = buf2[2];
    g_bme_calib.dig_H4 = (int16_t)((buf2[3] << 4) |  (buf2[4] & 0x0F));
    g_bme_calib.dig_H5 = (int16_t)((buf2[5] << 4) |  (buf2[4] >> 4));
    g_bme_calib.dig_H6 = (int8_t)  buf2[6];
    return 1;
}

static uint8_t bme280_init(void)
{
    uint8_t id = 0;
    if (bme280_read_regs(BME280_REG_ID, &id, 1) != 0) return 0;
    if (id != 0x60) return 0;
    if (bme280_write_reg(BME280_REG_RESET, 0xB6) != 0) return 0;
    HAL_Delay(5);
    if (!bme280_read_calibration()) return 0;
    if (bme280_write_reg(BME280_REG_CTRL_HUM,  0x01) != 0) return 0;
    if (bme280_write_reg(BME280_REG_CONFIG,    0xA0) != 0) return 0;
    if (bme280_write_reg(BME280_REG_CTRL_MEAS, 0x27) != 0) return 0;
    return 1;
}

static uint8_t bme280_read(float *t_c, float *h_rh, float *p_pa)
{
    uint8_t  data[8];
    int32_t  adc_T, adc_P, adc_H;
    int32_t  var1, var2, T;
    int64_t  var1_p, var2_p, p;
    int32_t  v_x1_u32r;

    if (bme280_read_regs(BME280_REG_PRESS_MSB, data, 8) != 0) return 0;

    adc_P = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4));
    adc_T = (int32_t)(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4));
    adc_H = (int32_t)(((uint32_t)data[6] <<  8) |  (uint32_t)data[7]);

    if (adc_T == 0x800000 || adc_P == 0x800000 || adc_H == 0x8000) return 0;

    var1 = ((((adc_T >> 3) - ((int32_t)g_bme_calib.dig_T1 << 1)))
             * ((int32_t)g_bme_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)g_bme_calib.dig_T1)
              * ((adc_T >> 4) - (int32_t)g_bme_calib.dig_T1)) >> 12)
             * (int32_t)g_bme_calib.dig_T3) >> 14;
    g_bme_t_fine = var1 + var2;
    T = (g_bme_t_fine * 5 + 128) >> 8;

    var1_p = ((int64_t)g_bme_t_fine) - 128000;
    var2_p = var1_p * var1_p * (int64_t)g_bme_calib.dig_P6;
    var2_p = var2_p + ((var1_p * (int64_t)g_bme_calib.dig_P5) << 17);
    var2_p = var2_p + (((int64_t)g_bme_calib.dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)g_bme_calib.dig_P3) >> 8)
           + ((var1_p * (int64_t)g_bme_calib.dig_P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p) * (int64_t)g_bme_calib.dig_P1) >> 33;
    if (var1_p == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2_p) * 3125) / var1_p;
    var1_p = ((int64_t)g_bme_calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2_p = ((int64_t)g_bme_calib.dig_P8 * p) >> 19;
    p = ((p + var1_p + var2_p) >> 8) + (((int64_t)g_bme_calib.dig_P7) << 4);

    v_x1_u32r = g_bme_t_fine - (int32_t)76800;
    v_x1_u32r = (((((adc_H << 14)
                    - (((int32_t)g_bme_calib.dig_H4) << 20)
                    - (((int32_t)g_bme_calib.dig_H5) * v_x1_u32r))
                   + (int32_t)16384) >> 15)
                 * (((((((v_x1_u32r * (int32_t)g_bme_calib.dig_H6) >> 10)
                        * (((v_x1_u32r * (int32_t)g_bme_calib.dig_H3) >> 11)
                           + (int32_t)32768)) >> 10)
                      + (int32_t)2097152)
                     * (int32_t)g_bme_calib.dig_H2 + 8192) >> 14));
    v_x1_u32r = v_x1_u32r
              - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
                  * (int32_t)g_bme_calib.dig_H1) >> 4);
    if (v_x1_u32r < 0)         v_x1_u32r = 0;
    if (v_x1_u32r > 419430400) v_x1_u32r = 419430400;

    if (t_c)  *t_c  = (float)T / 100.0f;
    if (p_pa) *p_pa = (float)p / 256.0f;
    if (h_rh) *h_rh = ((float)(v_x1_u32r >> 12)) / 1024.0f;
    return 1;
}

/* ======== binary protocol helpers ======== */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else               crc = (uint16_t)(crc  << 1);
        }
    }
    return crc;
}

static void bytes_to_hex_str(const uint8_t *data, uint16_t len, char *out)
{
    static const char hx[] = "0123456789abcdef";
    for (uint16_t i = 0; i < len; i++) {
        out[i * 2]     = hx[data[i] >> 4];
        out[i * 2 + 1] = hx[data[i] & 0x0F];
    }
    out[len * 2] = '\0';
}

static uint8_t hex_char_to_nibble(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    return 0;
}

static uint8_t hex_str_to_bytes(const char *hex, uint8_t *out, uint8_t max_bytes)
{
    uint8_t len = (uint8_t)(strlen(hex) / 2);
    if (len > max_bytes) len = max_bytes;
    for (uint8_t i = 0; i < len; i++)
        out[i] = (uint8_t)((hex_char_to_nibble(hex[i * 2]) << 4)
                          |  hex_char_to_nibble(hex[i * 2 + 1]));
    return len;
}

/* ======== GPS — huart3 (USART3, PC4/PC5) @ 9600 ======== */
static void gps_try_read(void)
{
    char    line[128];
    uint8_t idx = 0;
    uint8_t b;

    /* read one NMEA sentence; 5 ms per-byte timeout drops out quickly if no data */
    while (idx < sizeof(line) - 1) {
        if (HAL_UART_Receive(&huart3, &b, 1, 5) != HAL_OK) break;
        if (b == '\n') break;
        if (b == '\r') continue;
        line[idx++] = (char)b;
    }
    line[idx] = '\0';
    if (idx < 15 || strncmp(line, "$GPGGA", 6) != 0) return;

    /* tokenise on comma */
    char   *fields[12];
    uint8_t nf  = 0;
    char   *tok = strtok(line, ",");
    while (tok && nf < 12) { fields[nf++] = tok; tok = strtok(NULL, ","); }

    /* field[6] = fix quality; 0 or missing = no fix */
    if (nf < 7 || fields[6][0] == '0' || fields[6][0] == '\0') return;

    /* latitude: DDMM.MMMMM -> decimal degrees */
    float lat_raw = strtof(fields[2], NULL);
    int   lat_deg = (int)(lat_raw / 100);
    float lat_dec = lat_deg + (lat_raw - (float)(lat_deg * 100)) / 60.0f;
    if (fields[3][0] == 'S') lat_dec = -lat_dec;

    /* longitude: DDDMM.MMMMM -> decimal degrees */
    float lon_raw = strtof(fields[4], NULL);
    int   lon_deg = (int)(lon_raw / 100);
    float lon_dec = lon_deg + (lon_raw - (float)(lon_deg * 100)) / 60.0f;
    if (fields[5][0] == 'W') lon_dec = -lon_dec;

    g_gps_lat   = lat_dec;
    g_gps_lon   = lon_dec;
    g_gps_valid = 1;
}

/* ======== LoRa IRQ-driven RX helpers (Step 1) ======== */

/* Non-blocking ring buffer pop. Returns 1 if a byte was available, else 0. */
static uint8_t lora_rx_pop(uint8_t *out)
{
    if (lora_rx_head == lora_rx_tail) return 0;
    *out = lora_rx_ring[lora_rx_tail];
    lora_rx_tail = (uint16_t)((lora_rx_tail + 1u) & LORA_RX_RING_MASK);
    return 1;
}

/* HAL RX-complete callback: byte arrived on USART1, push to ring and re-arm.
 * Scoped to USART1 only; does not affect huart2 (S88) or huart3 (GPS/CO) polled paths.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    uint16_t next = (uint16_t)((lora_rx_head + 1u) & LORA_RX_RING_MASK);
    if (next != lora_rx_tail) {                  /* drop byte if ring full */
        lora_rx_ring[lora_rx_head] = lora_rx_byte;
        lora_rx_head = next;
    }
    (void)HAL_UART_Receive_IT(huart, &lora_rx_byte, 1);
}

/* HAL RX-error callback: overrun is common at 115200; clear flags and re-arm.
 * Only handles USART1; s88_uart_clear_errors() handles huart2 inline as before.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    (void)HAL_UART_Receive_IT(huart, &lora_rx_byte, 1);
}

/* ======== LoRa — now on huart1 (USART1, PB6/PB7) ======== */
static void lora_cmd(const char *cmd, uint32_t timeout_ms)
{
    char line[128];
    snprintf(line, sizeof(line), "%s\r\n", cmd);
    HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)strlen(line), timeout_ms);
    uint8_t rx[128]; size_t i = 0;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms && i < sizeof(rx) - 1) {
        uint8_t c;
        if (lora_rx_pop(&c)) rx[i++] = c;
    }
}

static uint16_t lora_readline(char *buf, uint16_t buf_size, uint32_t timeout_ms)
{
    uint16_t idx = 0;
    uint32_t t0  = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms && idx < buf_size - 1) {
        uint8_t b;
        if (!lora_rx_pop(&b)) continue;
        if (b == '\n') {
            if (idx > 0 && buf[idx - 1] == '\r') idx--;
            break;
        }
        buf[idx++] = (char)b;
    }
    buf[idx] = '\0';
    return idx;
}

/* Send a binary hex payload and wait for a binary ACK/NACK from the hub.
   hex_payload      : hex-encoded packet string (e.g. "0a1b2c...")
   payload_byte_len : number of raw bytes the hex string represents */
static uint8_t lora_send_with_ack(const char *hex_payload, uint16_t payload_byte_len)
{
	(void)payload_byte_len;
    char    cmd[256];
    char    rx_line[128];
    uint8_t reply[16];

    for (uint8_t attempt = 0; attempt < LORA_MAX_RETRIES; attempt++) {

        snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%d,%s\r\n",
                 LORA_DEST_ADDRESS, (int)strlen(hex_payload), hex_payload);
        HAL_UART_Transmit(&huart1, (uint8_t *)cmd, (uint16_t)strlen(cmd), LORA_CMD_TIMEOUT_MS);

        uint32_t t0 = HAL_GetTick();

        while ((HAL_GetTick() - t0) < (uint32_t)LORA_ACK_TIMEOUT_MS) {

            uint32_t remaining = LORA_ACK_TIMEOUT_MS - (HAL_GetTick() - t0);
            if (remaining == 0) break;

            memset(rx_line, 0, sizeof(rx_line));
            lora_readline(rx_line, sizeof(rx_line), remaining);

            if (strncmp(rx_line, "+RCV=", 5) != 0) continue;

            /* split +RCV=src,len,payload[,rssi,snr] — hex payload has no commas */
            char tmp[128];
            strncpy(tmp, rx_line + 5, sizeof(tmp) - 1);
            tmp[sizeof(tmp) - 1] = '\0';

            char *tok = strtok(tmp, ",");   /* src_addr */
            if (!tok) continue;
            tok = strtok(NULL, ",");         /* declared len */
            if (!tok) continue;
            tok = strtok(NULL, ",");         /* hex payload */
            if (!tok) continue;

            /* decode hex -> bytes; reply = 7 header bytes + 2 CRC bytes */
            uint8_t nbytes = hex_str_to_bytes(tok, reply, sizeof(reply));
            if (nbytes < 9) continue;

            /* verify CRC over all bytes except the trailing 2 */
            uint16_t crc_calc = crc16_ccitt(reply, (uint16_t)(nbytes - 2));
            uint16_t crc_recv = (uint16_t)(reply[nbytes - 2] | ((uint16_t)reply[nbytes - 1] << 8));
            if (crc_calc != crc_recv) continue;

            /* reply layout: [0]=hub_id [1]=dest_id [2]=type_pkt_num [3..6]=message(int32 LE) [7..8]=crc */
            uint8_t  rcv_seq = reply[2] & 0x0F;
            int32_t  message;
            memcpy(&message, &reply[3], 4);   /* little-endian int32 */

            if (rcv_seq != g_seq_bit) continue;

            if (message == ACK_MSG) {
                g_seq_bit ^= 1u;
                return 1;
            }
            if (message == NACK_MSG) break;   /* stop waiting, retry send */
        }

        HAL_Delay(200);
    }

    return 0;
}

/* ======== LoRa async TX state machine (Step 2) ========
 * Replaces the blocking 5x10s wait inside lora_send_with_ack with a
 * non-blocking state machine driven by lora_tick() from the main loop.
 * On-the-wire behavior is identical: same +SEND format, same retry count,
 * same ACK timeout, same 200ms inter-attempt backoff, same
 * g_seq_bit-toggles-on-ACK rule. The difference is that the main loop is
 * free to run gps_try_read() (and, later, beacon / forwarding logic)
 * while we wait. Callers must gate new sends on lora_is_busy() == 0.
 */

typedef enum {
    LORA_TX_IDLE     = 0,
    LORA_TX_WAIT_ACK = 1,
    LORA_TX_BACKOFF  = 2
} lora_tx_state_t;

static lora_tx_state_t g_tx_state   = LORA_TX_IDLE;
static char            g_tx_hex[64];      /* 22-byte BASE packet = 44 hex chars + nul */
static uint8_t         g_tx_attempt = 0;
static uint32_t        g_tx_t0      = 0;  /* attempt-start (WAIT_ACK) or backoff-start (BACKOFF) */
static uint8_t         g_tx_last_ok = 0;  /* result of most recent completed send */

/* Step 4: per-pending metadata. Set by lora_send_begin, used by lora_tick to
 * match incoming ACK/NACK and to decide whether to increment g_pkt_id (own
 * send) or relay the ACK downstream (forwarded send). */
static uint8_t  g_tx_originator = 0;
static uint16_t g_tx_pkt_id     = 0;
static uint8_t  g_tx_is_own     = 0;     /* 1 = our own data, 0 = forwarding for someone */
static uint8_t  g_tx_next_hop   = 0;     /* AT+SEND destination (overrides LORA_DEST_ADDRESS) */

/* Step 4 forwarding-state cache: (originator, pkt_id) -> prev_hop, with ~30s
 * TTL. Used both for relaying ACKs back along the reverse path and for
 * deduplicating DATA retransmits from a child. */
#define FWD_CACHE_SIZE       8u
#define FWD_CACHE_TTL_MS     30000u

/* Step 4 cache states: PENDING = forward in flight, CONFIRMED = upstream
 * has acked, downstream ACK is safe to re-send from cache on a dup DATA.
 * OBSERVED = we've heard the originator broadcasting this pkt_id but
 * have not relayed yet — the originator may be reaching the hub
 * directly, so we don't add extra traffic until enough retries confirm
 * the direct path is failing. */
#define FWD_STATE_PENDING    0u
#define FWD_STATE_CONFIRMED  1u
#define FWD_STATE_OBSERVED   2u

/* Cooperative-relay threshold: step in as a forwarder only after hearing
 * this many direct broadcasts of the same (originator, pkt_id) — that's
 * the signal the originator can't reach the hub on its own. */
#define RELAY_HEARD_THRESHOLD 3u

typedef struct {
    uint8_t  originator;
    uint16_t pkt_id;
    uint8_t  prev_hop;
    uint8_t  state;       /* FWD_STATE_OBSERVED, PENDING, or CONFIRMED */
    uint8_t  heard_count; /* incremented each time we hear this (orig, pkt_id) */
    uint32_t ts_ms;
    uint8_t  valid;
} fwd_entry_t;

static fwd_entry_t fwd_cache[FWD_CACHE_SIZE];

/* Independent line accumulator for the async path. Safe because lora_cmd()
 * (the only other ring consumer) runs only during init, before the main
 * loop begins ticking. */
static char     lora_line_buf[128];
static uint16_t lora_line_idx = 0;

static uint16_t lora_readline_nb(char **out)
{
    uint8_t b;
    while (lora_rx_pop(&b)) {
        if (b == '\n') {
            if (lora_line_idx > 0 && lora_line_buf[lora_line_idx - 1] == '\r')
                lora_line_idx--;
            lora_line_buf[lora_line_idx] = '\0';
            *out = lora_line_buf;
            uint16_t len = lora_line_idx;
            lora_line_idx = 0;
            return len;
        }
        if (lora_line_idx < sizeof(lora_line_buf) - 1) {
            lora_line_buf[lora_line_idx++] = (char)b;
        } else {
            lora_line_idx = 0;   /* overflow — discard partial line */
        }
    }
    return 0;
}

/* Step 4 forwarding cache helpers. Linear scan; size is 8 so cost is trivial. */
/* Step 4 forwarding cache helpers. Linear scan; size is 8 so cost is trivial.
 * fwd_lookup returns a pointer so the caller can read prev_hop and state
 * together, and mutate state in place. Auto-refreshes timestamp on hit so an
 * active conversation keeps the entry alive. */
static fwd_entry_t *fwd_lookup(uint8_t originator, uint16_t pkt_id)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < FWD_CACHE_SIZE; i++) {
        if (!fwd_cache[i].valid) continue;
        if ((now - fwd_cache[i].ts_ms) >= FWD_CACHE_TTL_MS) {
            fwd_cache[i].valid = 0;            /* lazy expiry */
            continue;
        }
        if (fwd_cache[i].originator == originator && fwd_cache[i].pkt_id == pkt_id) {
            fwd_cache[i].ts_ms = now;          /* refresh on access */
            return &fwd_cache[i];
        }
    }
    return NULL;
}

static void fwd_insert(uint8_t originator, uint16_t pkt_id, uint8_t prev_hop, uint8_t state)
{
    uint32_t now = HAL_GetTick();

    /* Refresh existing entry if any (e.g. PENDING insert when one already exists). */
    for (uint8_t i = 0; i < FWD_CACHE_SIZE; i++) {
        if (fwd_cache[i].valid
            && fwd_cache[i].originator == originator
            && fwd_cache[i].pkt_id == pkt_id) {
            fwd_cache[i].prev_hop = prev_hop;
            fwd_cache[i].state    = state;
            fwd_cache[i].ts_ms    = now;
            return;
        }
    }

    /* Find an empty or lazily-expired slot. */
    for (uint8_t i = 0; i < FWD_CACHE_SIZE; i++) {
        if (!fwd_cache[i].valid
            || (now - fwd_cache[i].ts_ms) >= FWD_CACHE_TTL_MS) {
            fwd_cache[i].originator = originator;
            fwd_cache[i].pkt_id     = pkt_id;
            fwd_cache[i].prev_hop   = prev_hop;
            fwd_cache[i].state      = state;
            fwd_cache[i].ts_ms      = now;
            fwd_cache[i].valid      = 1;
            return;
        }
    }

    /* Cache full and nothing expired — evict the oldest. */
    uint8_t  oldest_idx = 0;
    uint32_t oldest_age = 0;
    for (uint8_t i = 0; i < FWD_CACHE_SIZE; i++) {
        uint32_t age = now - fwd_cache[i].ts_ms;
        if (age > oldest_age) { oldest_age = age; oldest_idx = i; }
    }
    fwd_cache[oldest_idx].originator = originator;
    fwd_cache[oldest_idx].pkt_id     = pkt_id;
    fwd_cache[oldest_idx].prev_hop   = prev_hop;
    fwd_cache[oldest_idx].state      = state;
    fwd_cache[oldest_idx].ts_ms      = now;
    fwd_cache[oldest_idx].valid      = 1;
}

static void fwd_remove(uint8_t originator, uint16_t pkt_id)
{
    for (uint8_t i = 0; i < FWD_CACHE_SIZE; i++) {
        if (fwd_cache[i].valid
            && fwd_cache[i].originator == originator
            && fwd_cache[i].pkt_id == pkt_id) {
            fwd_cache[i].valid = 0;
            return;
        }
    }
}

/* Record a direct-from-originator hearing of (originator, pkt_id). On
 * the first observation, insert a new OBSERVED entry with heard_count=1.
 * On subsequent observations, increment heard_count (capped at 0xFF).
 * Returns the entry so the caller can read heard_count and decide
 * whether to promote to PENDING and relay. */
static fwd_entry_t *fwd_observe(uint8_t originator, uint16_t pkt_id, uint8_t prev_hop)
{
    uint32_t now = HAL_GetTick();

    for (uint8_t i = 0; i < FWD_CACHE_SIZE; i++) {
        if (fwd_cache[i].valid
            && fwd_cache[i].originator == originator
            && fwd_cache[i].pkt_id == pkt_id) {
            if (fwd_cache[i].heard_count < 0xFF) fwd_cache[i].heard_count++;
            fwd_cache[i].ts_ms = now;
            return &fwd_cache[i];
        }
    }

    /* New entry: find empty/expired slot, else evict the oldest. */
    uint8_t  target = 0;
    uint32_t oldest_age = 0;
    for (uint8_t i = 0; i < FWD_CACHE_SIZE; i++) {
        if (!fwd_cache[i].valid
            || (now - fwd_cache[i].ts_ms) >= FWD_CACHE_TTL_MS) {
            target = i;
            oldest_age = 0xFFFFFFFFu;
            break;
        }
        uint32_t age = now - fwd_cache[i].ts_ms;
        if (age > oldest_age) { oldest_age = age; target = i; }
    }

    fwd_cache[target].originator  = originator;
    fwd_cache[target].pkt_id      = pkt_id;
    fwd_cache[target].prev_hop    = prev_hop;
    fwd_cache[target].state       = FWD_STATE_OBSERVED;
    fwd_cache[target].heard_count = 1;
    fwd_cache[target].ts_ms       = now;
    fwd_cache[target].valid       = 1;
    return &fwd_cache[target];
}

static void lora_tx_transmit_current(void)
{
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%d,%s\r\n",
             g_tx_next_hop, (int)strlen(g_tx_hex), g_tx_hex);
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, (uint16_t)strlen(cmd), LORA_CMD_TIMEOUT_MS);
    g_tx_t0    = HAL_GetTick();
    g_tx_state = LORA_TX_WAIT_ACK;
}

static uint8_t lora_is_busy(void)
{
    return (uint8_t)(g_tx_state != LORA_TX_IDLE);
}

static uint8_t lora_send_begin(const char *hex_payload, uint16_t payload_byte_len,
                               uint8_t next_hop, uint8_t originator,
                               uint16_t pkt_id, uint8_t is_own)
{
    (void)payload_byte_len;
    if (g_tx_state != LORA_TX_IDLE) return 0;
    size_t n = strlen(hex_payload);
    if (n + 1 > sizeof(g_tx_hex)) return 0;
    memcpy(g_tx_hex, hex_payload, n + 1);
    g_tx_attempt    = 0;
    g_tx_last_ok    = 0;
    g_tx_next_hop   = next_hop;
    g_tx_originator = originator;
    g_tx_pkt_id     = pkt_id;
    g_tx_is_own     = is_own;
    lora_tx_transmit_current();
    return 1;
}

/* Step 4 fire-and-forget transmit, used only for relaying ACKs to a child.
 * No state machine, no retry — if the ACK is lost on the last hop, the
 * originator's DATA retransmit re-triggers the whole sequence. */
static void lora_send_fire_and_forget(const char *hex_payload, uint8_t next_hop)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%d,%s\r\n",
             next_hop, (int)strlen(hex_payload), hex_payload);
    HAL_UART_Transmit(&huart1, (uint8_t *)cmd, (uint16_t)strlen(cmd), LORA_CMD_TIMEOUT_MS);
}

/* Step 4: send a downstream ACK on behalf of a confirmed flow. Generated
 * locally from cached information — no relay across the mesh. Called when
 * an upstream ACK confirms our pending forward, and again on any duplicate
 * DATA that arrives while the entry is CONFIRMED. */
static void send_link_ack(uint8_t originator, uint16_t pkt_id, uint8_t to)
{
    uint8_t ack[9];
    ack[0] = originator;                         /* originator (data flow)  */
    ack[1] = originator;                         /* final_dest = originator */
    ack[2] = (uint8_t)LORA_ADDRESS;              /* prev_hop = me           */
    ack[3] = to;                                  /* next_hop = downstream   */
    ack[4] = (uint8_t)((PKT_TYPE_ACK << 4) | (LORA_INITIAL_TTL & 0x0F));
    memcpy(&ack[5], &pkt_id, 2);
    uint16_t crc = crc16_ccitt(ack, 7);
    ack[7] = (uint8_t)(crc & 0xFF);
    ack[8] = (uint8_t)(crc >> 8);

    char hex[19];
    bytes_to_hex_str(ack, 9, hex);
    lora_send_fire_and_forget(hex, to);
}

static void lora_tick(void)
{
    /* drain all complete lines from the ring this tick */
    char    *line;
    uint16_t llen;
    while ((llen = lora_readline_nb(&line)) > 0) {
        if (strncmp(line, "+RCV=", 5) != 0)  continue;

        char tmp[128];
        strncpy(tmp, line + 5, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = '\0';

        char *tok = strtok(tmp, ",");        /* src_addr     */
        if (!tok) continue;
        tok       = strtok(NULL, ",");        /* declared len */
        if (!tok) continue;
        tok       = strtok(NULL, ",");        /* hex payload  */
        if (!tok) continue;

        uint8_t reply[32];
        uint8_t nbytes = hex_str_to_bytes(tok, reply, sizeof(reply));
        if (nbytes < 9) continue;

        uint16_t crc_calc = crc16_ccitt(reply, (uint16_t)(nbytes - 2));
        uint16_t crc_recv = (uint16_t)(reply[nbytes - 2] | ((uint16_t)reply[nbytes - 1] << 8));
        if (crc_calc != crc_recv) continue;

        uint8_t  rx_originator = reply[0];
        uint8_t  rx_final_dest = reply[1];
        uint8_t  rx_prev_hop   = reply[2];
        uint8_t  rx_type       = (reply[4] >> 4) & 0x0F;
        uint8_t  rx_ttl        =  reply[4] & 0x0F;
        uint16_t rx_pkt_id;
        memcpy(&rx_pkt_id, &reply[5], 2);

        if (rx_type == PKT_TYPE_DATA) {
            /* DATA addressed to us at the LoRa layer. final_dest == us means
             * we'd be the destination — only the hub is in that role; nodes
             * just drop. */
            if (rx_final_dest == LORA_ADDRESS) continue;

            /* Don't relay our own packets back to ourselves if another node
             * forwarded a copy that loops back. */
            if (rx_originator == LORA_ADDRESS) continue;

            fwd_entry_t *entry = fwd_lookup(rx_originator, rx_pkt_id);

            /* Already confirmed end-to-end — the downstream ACK must have been
             * lost. Regenerate it locally and ship it back. */
            if (entry && entry->state == FWD_STATE_CONFIRMED) {
                send_link_ack(rx_originator, rx_pkt_id, entry->prev_hop);
                continue;
            }

            /* In-flight forward for this same packet — drop the dup, our
             * current forward will resolve and ack the downstream then. */
            if (entry && entry->state == FWD_STATE_PENDING) continue;

            if (rx_ttl == 0) continue;

            /* Cooperative relay: count direct broadcasts from the originator
             * (rx_prev_hop == rx_originator). If the originator can reach
             * the hub directly, hub ACKs on first transmission and we never
             * see a retry — so we sit idle in OBSERVED and add no traffic.
             * Only after RELAY_HEARD_THRESHOLD hearings do we conclude the
             * originator is stuck (out of range, RF too weak) and step in
             * to forward on its behalf. Forwarded copies (prev_hop !=
             * originator) don't drive the count — we don't want to pile on
             * after another node has already relayed. */
            if (rx_prev_hop == rx_originator) {
                entry = fwd_observe(rx_originator, rx_pkt_id, rx_prev_hop);
            }
            if (!entry || entry->heard_count < RELAY_HEARD_THRESHOLD) continue;

            if (g_tx_state != LORA_TX_IDLE) continue;  /* slot busy, drop */

            /* Threshold reached — promote OBSERVED -> PENDING and relay. */
            entry->state = FWD_STATE_PENDING;

            /* Rewrite header in place: prev_hop = me, next_hop = parent, TTL--. */
            reply[2] = (uint8_t)LORA_ADDRESS;
            reply[3] = (uint8_t)MY_PARENT_ADDRESS;
            reply[4] = (uint8_t)((PKT_TYPE_DATA << 4) | ((rx_ttl - 1u) & 0x0F));

            uint16_t new_crc = crc16_ccitt(reply, (uint16_t)(nbytes - 2));
            reply[nbytes - 2] = (uint8_t)(new_crc & 0xFF);
            reply[nbytes - 1] = (uint8_t)(new_crc >> 8);

            char fwd_hex[64];
            bytes_to_hex_str(reply, nbytes, fwd_hex);
            lora_send_begin(fwd_hex, nbytes, MY_PARENT_ADDRESS,
                            rx_originator, rx_pkt_id, 0u);
            continue;
        }

        if (rx_type == PKT_TYPE_ACK || rx_type == PKT_TYPE_NACK) {
            uint8_t matches_pending = (g_tx_state == LORA_TX_WAIT_ACK)
                                   && (rx_originator == g_tx_originator)
                                   && (rx_pkt_id     == g_tx_pkt_id);

            if (!matches_pending) continue;   /* not for our pending send — drop */

            if (rx_type == PKT_TYPE_ACK) {
                if (g_tx_is_own) {
                    g_pkt_id++;                /* advance our own counter */
                }
                g_tx_last_ok = 1;
                g_tx_state   = LORA_TX_IDLE;

                /* If we were forwarding for someone, mark the entry confirmed
                 * and send the downstream ACK locally. Subsequent dup DATA
                 * for this (originator, pkt_id) will trigger a re-send. */
                if (!g_tx_is_own) {
                    fwd_entry_t *entry = fwd_lookup(rx_originator, rx_pkt_id);
                    if (entry) {
                        entry->state = FWD_STATE_CONFIRMED;
                        send_link_ack(rx_originator, rx_pkt_id, entry->prev_hop);
                    }
                }
                return;
            }

            /* NACK matching pending. */
            g_tx_attempt++;
            if (g_tx_attempt >= LORA_MAX_RETRIES) {
                g_tx_last_ok = 0;
                g_tx_state   = LORA_TX_IDLE;
                /* Own send: advance pkt_id so a lost downstream ACK doesn't
                 * strand us on the same pkt_id forever. The hub may have
                 * already received and inserted this packet — we just lost
                 * the ACK chain. Accept the bookkeeping mismatch, move on. */
                if (g_tx_is_own) {
                    g_pkt_id++;
                }
                /* Forward gave up — drop cache entry so a future retransmit
                 * can try fresh instead of being stonewalled forever. */
                if (!g_tx_is_own) {
                    fwd_remove(g_tx_originator, g_tx_pkt_id);
                }
            } else {
                g_tx_t0    = HAL_GetTick();
                g_tx_state = LORA_TX_BACKOFF;
            }
            return;
        }

        /* Other packet types (BEACON in Step 5) — ignore. */
    }

    /* timing-driven transitions */
    if (g_tx_state == LORA_TX_BACKOFF) {
        if ((HAL_GetTick() - g_tx_t0) >= 200u) {
            lora_tx_transmit_current();
        }
    } else if (g_tx_state == LORA_TX_WAIT_ACK) {
        if ((HAL_GetTick() - g_tx_t0) >= (uint32_t)LORA_ACK_TIMEOUT_MS) {
            g_tx_attempt++;
            if (g_tx_attempt >= LORA_MAX_RETRIES) {
                g_tx_last_ok = 0;
                g_tx_state   = LORA_TX_IDLE;
                /* Own send: advance pkt_id on timeout-driven give-up too. */
                if (g_tx_is_own) {
                    g_pkt_id++;
                }
                /* Same cleanup on timeout-driven give-up. */
                if (!g_tx_is_own) {
                    fwd_remove(g_tx_originator, g_tx_pkt_id);
                }
            } else {
                g_tx_t0    = HAL_GetTick();
                g_tx_state = LORA_TX_BACKOFF;
            }
        }
    }
}

/* Pack and transmit a LOCATION packet (13 bytes).
   Sent only when a valid GPS fix is available. */
static void send_location_packet(void)
{
    uint8_t  pkt[17];
    char     hex[35];
    int32_t  lon_i = (int32_t)(g_gps_lon * 100000.0f);
    int32_t  lat_i = (int32_t)(g_gps_lat * 100000.0f);
    uint16_t crc;

    pkt[0] = (uint8_t)LORA_ADDRESS;        /* originator */
    pkt[1] = (uint8_t)LORA_DEST_ADDRESS;   /* final_dest */
    pkt[2] = (uint8_t)LORA_ADDRESS;        /* prev_hop   */
    pkt[3] = (uint8_t)MY_PARENT_ADDRESS;   /* next_hop   */
    pkt[4] = (uint8_t)((PKT_TYPE_DATA << 4) | (LORA_INITIAL_TTL & 0x0F));
    memcpy(&pkt[5],  &g_pkt_id, 2);
    memcpy(&pkt[7],  &lon_i,    4);
    memcpy(&pkt[11], &lat_i,    4);
    crc     = crc16_ccitt(pkt, 15);
    pkt[15] = (uint8_t)(crc & 0xFF);
    pkt[16] = (uint8_t)(crc >> 8);

    bytes_to_hex_str(pkt, 17, hex);
    lora_send_begin(hex, 17, MY_PARENT_ADDRESS, LORA_ADDRESS, g_pkt_id, 1u);
}

/* Pack and transmit a BASE sensor packet (22 bytes). */
static void send_base_packet(float temp_c, float hum_rh, float press_hpa,
                              float co_ppm, uint16_t co2_ppm)
{
    uint8_t  pkt[26];
    char     hex[53];
    int16_t  temp_i  = (int16_t)(temp_c    * 100.0f);
    int16_t  hum_i   = (int16_t)(hum_rh    * 100.0f);
    int32_t  pres_i  = (int32_t)(press_hpa * 100.0f);
    int32_t  co_i    = (int32_t)(co_ppm    * 100.0f);
    int32_t  co2_i   = (int32_t)co2_ppm * 100;
    uint16_t crc;

    pkt[0]  = (uint8_t)LORA_ADDRESS;        /* originator */
    pkt[1]  = (uint8_t)LORA_DEST_ADDRESS;   /* final_dest */
    pkt[2]  = (uint8_t)LORA_ADDRESS;        /* prev_hop   */
    pkt[3]  = (uint8_t)MY_PARENT_ADDRESS;   /* next_hop   */
    pkt[4]  = (uint8_t)((PKT_TYPE_DATA << 4) | (LORA_INITIAL_TTL & 0x0F));
    memcpy(&pkt[5],  &g_pkt_id, 2);
    memcpy(&pkt[7],  &temp_i,   2);
    memcpy(&pkt[9],  &hum_i,    2);
    memcpy(&pkt[11], &pres_i,   4);
    memcpy(&pkt[15], &co_i,     4);
    memcpy(&pkt[19], &co2_i,    4);
    pkt[23] = 0x00;                 /* fire_u8 = 0 (predicted by hub) */
    crc     = crc16_ccitt(pkt, 24);
    pkt[24] = (uint8_t)(crc & 0xFF);
    pkt[25] = (uint8_t)(crc >> 8);

    bytes_to_hex_str(pkt, 26, hex);
    lora_send_begin(hex, 26, MY_PARENT_ADDRESS, LORA_ADDRESS, g_pkt_id, 1u);
}

/* ======== clock ======== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    o.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    o.HSIState            = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState        = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&o);

    c.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    c.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV1;
    c.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&c, FLASH_LATENCY_0);
}

/* ======== main ======== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();   /* LoRa   — PB6/PB7 */
    MX_USART2_UART_Init();   /* CO2    — PA2/PA3 */
    MX_USART3_UART_Init();   /* CO gas  — PB10/PB11 */
    //MX_USART3_UART_Init();   /* GPS    — PC4/PC5 */
    MX_I2C1_Init();          /* BME280 — PB8/PB9 */
    //MX_I2C2_Init();          /* Gas    — PB10/PB11 */

    /* ---- USART1: LoRa module (PB6/PB7) @ 115200 ---- */
    HAL_UART_DeInit(&huart1);
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling        = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart1);

    /* ---- USART2: SenseAir S88 Modbus RTU (PA2/PA3) @ 9600 ---- */
    HAL_UART_DeInit(&huart2);
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = S88_BAUD_RATE;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling        = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart2);

    /* ---- USART3: GPS NEO-6M (PC4/PC5) @ 9600 ---- */
    HAL_UART_DeInit(&huart3);
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 9600;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling        = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart3);

    /* Step 1: arm IRQ-driven single-byte RX for LoRa.
     * MspInit (called from HAL_UART_Init above) has already enabled the USART1 NVIC line. */
    HAL_UART_Receive_IT(&huart1, &lora_rx_byte, 1);

    HAL_Delay(1000);

    /* ---- LoRa configuration ---- */
    char tmp[64];
    lora_cmd("AT", 500);
    lora_cmd("AT+MODE=0", 500);
    snprintf(tmp, sizeof(tmp), "AT+BAND=%d000000", LORA_BAND_MHZ);   lora_cmd(tmp, 500);
    snprintf(tmp, sizeof(tmp), "AT+NETWORKID=%d",  LORA_NETWORK_ID); lora_cmd(tmp, 500);
    snprintf(tmp, sizeof(tmp), "AT+ADDRESS=%d",    LORA_ADDRESS);    lora_cmd(tmp, 500);
    lora_cmd("AT+PARAMETER=9,7,1,12", 500);
    snprintf(tmp, sizeof(tmp), "AT+CRFOP=%d",      LORA_RF_POWER);   lora_cmd(tmp, 500);

    /* ---- sensor initialisation ---- */
    g_bme_ready = bme280_init();
    g_gas_ready = gas_init_co_only();
    g_s88_ready = s88_init();

    /* Per-node boot stagger: spread send timings across the 10 s window so
     * leaves' transmissions don't collide with the forwarder's own sends.
     * Address-based and deterministic — no RNG needed.
     * Node 1: 0 ms, Node 2: 3000 ms, Node 3: 6000 ms. */
    HAL_Delay((LORA_ADDRESS - 1u) * 3000u);

    /* ---- main loop ---- */
    uint32_t last_send     = HAL_GetTick();
    uint32_t last_location = HAL_GetTick();

    while (1) {
        /* opportunistically grab a GPS fix from USART3 */
        gps_try_read();
        lora_tick();

        uint32_t now = HAL_GetTick();

        /* send GPS location packet every LOCATION_PERIOD_MS (only when fix valid) */
        if (!lora_is_busy() && (now - last_location) >= LOCATION_PERIOD_MS) {
            last_location += LOCATION_PERIOD_MS;
            //if (g_gps_valid) //ML: i commented this out
            send_location_packet();
        }

        /* send sensor BASE packet every SEND_PERIOD_MS */
        if (!lora_is_busy() && (now - last_send) >= SEND_PERIOD_MS) {
            last_send += SEND_PERIOD_MS;

            uint8_t ok_bme = 0, ok_co = 0, ok_co2 = 0;
            if (g_bme_ready) ok_bme = bme280_read(&g_temperature_c, &g_humidity_rh, &g_pressure_pa);
            if (g_gas_ready) ok_co  = gas_read_co_ppm(&g_co_ppm);
            ok_co2 = s88_read_co2(&g_co2_ppm);

            float    out_temp_c    = ok_bme ? g_temperature_c          : 0.0f;
            float    out_hum_rh    = ok_bme ? g_humidity_rh            : 0.0f;
            float    out_press_hpa = ok_bme ? (g_pressure_pa / 100.0f) : 0.0f;
            float    out_co_ppm    = ok_co  ? g_co_ppm                 : 0.0f; //set to 99, because a 0ppm indicates a good reading
            uint16_t out_co2_ppm   = ok_co2 ? g_co2_ppm : (0u + g_co2_err);

            send_base_packet(out_temp_c, out_hum_rh, out_press_hpa,
                             out_co_ppm, out_co2_ppm);
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif
//the end :)
