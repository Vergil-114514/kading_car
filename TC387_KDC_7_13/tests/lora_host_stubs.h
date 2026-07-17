#ifndef LORA_HOST_STUBS_H
#define LORA_HOST_STUBS_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define _zf_common_headfile_h_
#define CODE_ZF_DEVICE_LORA3A22_H_

typedef uint8_t uint8;
typedef int16_t int16;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef volatile uint8 vuint8;
typedef volatile uint16 vuint16;
typedef volatile uint32 vuint32;

#define UART_2                          (2)
#define UART2_TX_P10_5                  (105)
#define UART2_RX_P10_6                  (106)
#define WIRELESS_UART                   (1)
#define LORA3A22_UART_INDEX             (UART_2)
#define LORA3A22_UART_RX_PIN            (UART2_TX_P10_5)
#define LORA3A22_UART_TX_PIN            (UART2_RX_P10_6)
#define LORA3A22_UART_BAUDRATE           (115200)
#define LORA3A22_DATA_LEN                (18)
#define LORA3A22_FRAME_STAR              (0xA3)
#define LORA3A22_LINK_TIMEOUT_MS         (300U)

typedef struct
{
    uint8 head;
    uint8 sum_check;
    int16 joystick[4];
    uint8 key[4];
    uint8 switch_key[4];
} lora3a22_uart_transfer_dat_struct;

uint8 uart_read_byte(int index);
void uart_init(int index, int baud, int tx_pin, int rx_pin);
void uart_rx_interrupt(int index, int enable);
void set_wireless_type(int type, void (*callback)(void));
uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32 state);

void lora3a22_init(void);
void lora3a22_uart_callback(void);
void lora3a22_5ms_callback(void);
uint8 lora3a22_get_snapshot(lora3a22_uart_transfer_dat_struct *snapshot,
                            uint32 *sequence);

extern vuint8 lora3a22_state_flag;
extern vuint16 lora3a22_response_time;
extern vuint32 lora3a22_frame_sequence;

#endif
