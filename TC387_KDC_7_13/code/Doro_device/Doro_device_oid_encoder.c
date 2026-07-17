#include <Doro_device/Doro_device_oid_encoder.h>
#include "zf_common_debug.h"
#include "zf_driver_delay.h"
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     欧艾迪 12-bit 编码器初始化 (硬件 SPI + 宏定义版)
// 参数说明     无 (参数已全部在头文件中通过宏定义配置)
// 返回参数     void
// 使用示例     doro_oid_encoder_init();
//-------------------------------------------------------------------------------------------------------------------
void oid_encoder_init (void)
{
    // 1. 初始化硬件 SPI
    // 根据欧艾迪手册时序图：时钟空闲为高电平(CPOL=1)，在第二个跳变沿(上升沿)采样(CPHA=1)，对应 SPI_MODE_3
    // 注意：片选参数填 SPI_CS_NULL，意在使用软件手动控制 CS
    spi_init(OID_ENCODER_SPI_N, SPI_MODE3, OID_ENCODER_BAUDRATE, OID_ENCODER_SPI_SCK, OID_ENCODER_SPI_MOSI, OID_ENCODER_SPI_MISO, SPI_CS_NULL);

    // 2. 将 CS 初始化为普通推挽输出，并默认拉高（空闲状态）
    gpio_init(OID_ENCODER_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取欧艾迪编码器角度数据 (12-bit)
// 参数说明     无
// 返回参数     uint16          返回 0~4095 (对应 0~360度)
// 备注信息     利用硬件 SPI 连续读取 24 位，通过位移提取前 12 位角度
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取欧艾迪编码器角度数据 (12-bit)
//-------------------------------------------------------------------------------------------------------------------
uint16 oid_encoder_get_angle (void)
{
    uint8 rx_buf[3] = {0, 0, 0};
    uint16 angle;

    spi_init(OID_ENCODER_SPI_N, SPI_MODE3, OID_ENCODER_BAUDRATE, OID_ENCODER_SPI_SCK, OID_ENCODER_SPI_MOSI, OID_ENCODER_SPI_MISO, SPI_CS_NULL);

    // 1. 手动拉低片选，启动 SPI 帧
    gpio_low(OID_ENCODER_CS_PIN);

    // 【关键修复 1】：补充 CS 拉低到时钟输出的建立时间 (t_CLK_FE > 500ns)
    // TC377 执行太快，必须让编码器有时间准备第一位数据
    system_delay_us(1); // 延时 1us，确保绝对大于 500ns

    // 2. 利用硬件 QSPI 快速连续读取 3 个字节 (24 个时钟周期)
    spi_read_8bit_array(OID_ENCODER_SPI_N, rx_buf, 3);

    // 3. 通信结束，拉高片选
    gpio_high(OID_ENCODER_CS_PIN);

    // 【关键修复 2】：补充 CS 释放后的恢复时间 (t_CSn > 500ns)
    // 防止外部主循环过快轮询，导致下一次通信出错
    system_delay_us(1);

    // 4. 数据解析 (MSB First 模式)
    // 提取前 12 位有效角度数据
    angle = (uint16)((rx_buf[0] << 4) | (rx_buf[1] >> 4));

    return angle;
}



