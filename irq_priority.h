/*
 * irq_priority.h
 * @brief 
 * Created on: Apr 13, 2022
 * Author: 
 */

#ifndef BSP_IRQ_PRIORITY_H_
#define BSP_IRQ_PRIORITY_H_

// 0 bits for pre-emption priority, 4 bits for subpriority
// 0: highest, 15: lowest

#define SYSTICK_IRQ_PRIORITY      15



#define SPI1_TX_DMA_PRIORITY      11
#define SPI1_RX_DMA_PRIORITY      11

#define SPI0_TX_DMA_PRIORITY      10
#define SPI0_RX_DMA_PRIORITY      10


#endif /* BSP_IRQ_PRIORITY_H_ */
