/*
 * drv_fast_qspi.h
 * @brief only support SPI0, max 30MHz
 * Created on: Sep 26, 2022
 * Author: Yanye
 */

#ifndef BSP_DRIVER_DRV_FAST_QSPI_H_
#define BSP_DRIVER_DRV_FAST_QSPI_H_

#include "rt_spi.h"
#include "rtthread.h"

// 启用仅8位模式，rt_spi_message的is_16bit标记将无效
// GD32F303: QPI只支持8位模式，SPI支持8/16位模式
// 启用后，省去了位宽切换代码，速度约提升8.9KB/s
#define SPI0_ONLY_8BIT_WIDTH    1

rt_err_t fast_qspi_init(const struct rt_spi_configuration *configuration);

rt_uint32_t fast_qspi_xfer(struct rt_spi_message *message);

#endif /* BSP_DRIVER_DRV_FAST_QSPI_H_ */
