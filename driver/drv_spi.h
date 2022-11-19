/*
 * spi.h
 * @brief
 * Created on: Nov 4, 2021
 * Author: Yanye
 */

#ifndef _SRC_BSP_DRIVER_SPI0_H_
#define _SRC_BSP_DRIVER_SPI0_H_

// SPI0(at APB2 120Mhz max) fSCK 30MHz max
// current APB2 = 60MHz

// SPI1/2(at APB1 60MHz max) fSCK 30MHz max
// current APB1 = 60MHz

#define SPI_CLK_30M       0
#define SPI_CLK_15M       1
#define SPI_CLK_7P5M      2
#define SPI_CLK_3P7M      3
#define SPI_CLK_1P8M      4
#define SPI_CLK_0P9M      5

#endif /* _SRC_BSP_DRIVER_SPI0_H_ */
