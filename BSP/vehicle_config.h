#ifndef __VEHICLE_CONFIG_H
#define __VEHICLE_CONFIG_H

/*
 * Vehicle drive hardware profile.
 *
 * 2WD: FL + FR motors/encoders are enabled. RL + RR resources are not
 *      initialized, and PA5/PA6/PA7 are available for SPI1.
 * 4WD: FL + FR + RL + RR motors/encoders are enabled. Encoder CH3 occupies
 *      PA6/PA7, so SPI1 cannot be enabled on its default pins.
 *
 * Change only VEHICLE_DRIVE_MODE when switching chassis hardware.
 */
#define VEHICLE_DRIVE_MODE_2WD             2U
#define VEHICLE_DRIVE_MODE_4WD             4U

#ifndef VEHICLE_DRIVE_MODE
#define VEHICLE_DRIVE_MODE                 VEHICLE_DRIVE_MODE_4WD  
#endif

/*
 * USART1 外部链路选择。
 *
 * 普通直连：PA9/PA10 直接连接 USB-TTL 转换器或其他 UART 设备。
 * E220：PA9/PA10 连接 E220 的 RXD/TXD，PE6 读取 E220 AUX。
 *
 * E220 模块已配置为 115200、8N1。使用透明传输模式时，M0 和 M1
 * 必须由硬件可靠拉低；本固件不控制这两个引脚。
 */
#define VEHICLE_UART1_LINK_DIRECT           0U
#define VEHICLE_UART1_LINK_E220             1U

#ifndef VEHICLE_UART1_LINK_MODE
#define VEHICLE_UART1_LINK_MODE             VEHICLE_UART1_LINK_E220
#endif

#if (VEHICLE_UART1_LINK_MODE == VEHICLE_UART1_LINK_DIRECT)
#define VEHICLE_UART1_E220_ENABLE           0U
#define VEHICLE_UART1_LINK_NAME             "DIRECT"
#elif (VEHICLE_UART1_LINK_MODE == VEHICLE_UART1_LINK_E220)
#define VEHICLE_UART1_E220_ENABLE           1U
#define VEHICLE_UART1_LINK_NAME             "E220"
#else
#error "VEHICLE_UART1_LINK_MODE must be VEHICLE_UART1_LINK_DIRECT or VEHICLE_UART1_LINK_E220"
#endif

#if (VEHICLE_DRIVE_MODE == VEHICLE_DRIVE_MODE_2WD)
#define VEHICLE_REAR_DRIVE_ENABLE          0U
#define VEHICLE_SPI1_PINS_AVAILABLE        1U
#define VEHICLE_DRIVE_MODE_NAME            "2WD"
#elif (VEHICLE_DRIVE_MODE == VEHICLE_DRIVE_MODE_4WD)
#define VEHICLE_REAR_DRIVE_ENABLE          1U
#define VEHICLE_SPI1_PINS_AVAILABLE        0U
#define VEHICLE_DRIVE_MODE_NAME            "4WD"
#else
#error "VEHICLE_DRIVE_MODE must be VEHICLE_DRIVE_MODE_2WD or VEHICLE_DRIVE_MODE_4WD"
#endif

#endif /* __VEHICLE_CONFIG_H */
