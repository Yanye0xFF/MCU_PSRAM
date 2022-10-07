/*
 * drv_fast_qspi.c
 * @brief only support SPI0, max 30MHz
 * Created on: Sep 26, 2022
 * Author: Yanye
 */
#include "drv_fast_qspi.h"
#include "rtconfig.h"
#include "core_cm4.h"

#if (GD_USE_FAST_QSPI)

#include "gd32f30x.h"
#include "gd32f30x_spi.h"
#include "gd32f30x_rcu.h"
#include "gd32f30x_gpio.h"
#include "irq_priority.h"

// private:
#define SPI0_GPIO_PORT   GPIOA
#define SPI0_CS_PIN      GPIO_PIN_1
#define SPI0_IO2_PIN     GPIO_PIN_2
#define SPI0_IO3_PIN     GPIO_PIN_3
#define SPI0_SCLK_PIN    GPIO_PIN_5
// D1
#define SPI0_MISO_PIN    GPIO_PIN_6
// D0
#define SPI0_MOSI_PIN    GPIO_PIN_7

static const uint32_t RX_DUMMY_BYTE_ADDR = 0x0803FC00;

static volatile uint8_t dma_rx_done;
static volatile uint8_t dma_tx_done;

// SPI0_RX
void DMA0_Channel1_IRQHandler(void) {
    // receive done
    dma_interrupt_flag_clear(DMA0, DMA_CH1, DMA_INT_FLAG_FTF);
    dma_rx_done = 1;
}

// SPI0_TX
void DMA0_Channel2_IRQHandler(void) {
    // transmit done
    dma_interrupt_flag_clear(DMA0, DMA_CH2, DMA_INT_FLAG_FTF);
    dma_tx_done = 1;
}

rt_err_t fast_qspi_init(const struct rt_spi_configuration *configuration) {
    spi_parameter_struct spi_param;
    dma_parameter_struct dma_param;

    rt_uint16_t spi_mode = (configuration->mode & 0x3); // BIT0 & BIT1
    rt_uint16_t spi_msb = (configuration->mode >> 2) & 0x1; // BIT2
    rt_uint16_t spi_slave = (configuration->mode >> 3) & 0x1; // BIT3

    if(configuration->data_width == 8) {
        spi_param.frame_size = SPI_FRAMESIZE_8BIT;
    }else if(configuration->data_width == 16) {
#if (SPI0_ONLY_8BIT_WIDTH)
        return -RT_EINVAL;
#else
        spi_param.frame_size = SPI_FRAMESIZE_16BIT;
#endif
    }else {
        return -RT_EINVAL;
    }

    rcu_periph_clock_enable(RCU_SPI0);
    // only init cs & sclk
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_10MHZ, SPI0_CS_PIN);
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, SPI0_SCLK_PIN);

    // DMA clock enable @see pre_init.c
    // SPI0 RX
    dma_deinit(DMA0, DMA_CH1);
    dma_param.periph_addr  = (uint32_t)&SPI_DATA(SPI0);
    dma_param.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    // edit 'DMA_CHxMADDR' register directly when DMA transmit
    dma_param.memory_addr  = 0;
    dma_param.memory_width = DMA_MEMORY_WIDTH_8BIT;
    // edit 'DMA_CHCNT' register directly when DMA transmit
    dma_param.number = 0;
    dma_param.priority     = DMA_PRIORITY_HIGH;
    dma_param.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_param.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_param.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init(DMA0, DMA_CH1, &dma_param);
    dma_circulation_disable(DMA0, DMA_CH1);
    dma_memory_to_memory_disable(DMA0, DMA_CH1);

    dma_interrupt_flag_clear(DMA0, DMA_CH1, DMA_INT_FLAG_FTF);
    dma_interrupt_enable(DMA0, DMA_CH1, DMA_INT_FTF);
    nvic_irq_enable(DMA0_Channel1_IRQn, 0, SPI0_RX_DMA_PRIORITY);

    // SPI0 TX
    dma_deinit(DMA0, DMA_CH2);
    dma_param.periph_addr  = (uint32_t)&SPI_DATA(SPI0);
    dma_param.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    // edit 'DMA_CHxMADDR' register directly when DMA transmit
    dma_param.memory_addr  = 0;
    dma_param.memory_width = DMA_MEMORY_WIDTH_8BIT;
    // edit 'DMA_CHCNT' register directly when DMA receive
    dma_param.number = 0;
    dma_param.priority     = DMA_PRIORITY_HIGH;
    dma_param.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_param.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_param.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init(DMA0, DMA_CH2, &dma_param);
    dma_circulation_disable(DMA0, DMA_CH2);
    dma_memory_to_memory_disable(DMA0, DMA_CH2);

    dma_interrupt_flag_clear(DMA0, DMA_CH2, DMA_INT_FLAG_FTF);
    dma_interrupt_enable(DMA0, DMA_CH2, DMA_INT_FTF);
    nvic_irq_enable(DMA0_Channel2_IRQn, 0, SPI0_TX_DMA_PRIORITY);

    spi_disable(SPI0);

    spi_param.device_mode = (spi_slave ? SPI_SLAVE : SPI_MASTER);
    spi_param.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_param.nss = SPI_NSS_SOFT;
    spi_param.endian = (spi_msb ? SPI_ENDIAN_MSB : SPI_ENDIAN_LSB);
    spi_param.prescale = CTL0_PSC(configuration->max_hz);
    spi_param.clock_polarity_phase = spi_mode;

    spi_init(SPI0, &spi_param);
    spi_enable(SPI0);

    return RT_EOK;
}

#if (SPI0_ONLY_8BIT_WIDTH == 0)
// default: spi 8bit, dma 8bit
static uint8_t spi_bit_width_flag = 0x00;

#define SPI_8BIT_MODE     0x0
#define SPI_16BIT_MODE    0x1
#endif
// default: none mode
static uint8_t qspi_io_flag = 0xFF;

#define IO_FLAG_SPI_INOUT_MODE    0xFE
#define IO_FLAG_QPI_OUT_MODE     0xFD
#define IO_FLAG_QPI_IN_MODE      0xFB

#define SPI_INOUT_MASK    0x1
#define QPI_OUT_MASK      0x2
#define QPI_IN_MASK       0x4

rt_uint32_t fast_qspi_xfer(struct rt_spi_message *message) {
    uint32_t ctl;
    /* take CS */
    if (message->cs_take) {
        GPIO_BC(GPIOA) = SPI0_CS_PIN;
    }

    // only support: DMA mode, half-duplex, 8bit mode, QSPI or SPI

    // disable DMA channel to update 'counter register', see below.
    DMA_CHCTL(DMA0, DMA_CH2) &= ~DMA_CHXCTL_CHEN;
    DMA_CHCTL(DMA0, DMA_CH1) &= ~DMA_CHXCTL_CHEN;

#if (SPI0_ONLY_8BIT_WIDTH)
    DMA_CHCNT(DMA0, DMA_CH2) = message->length;
    DMA_CHCNT(DMA0, DMA_CH1) = message->length;
#else
    if(message->is_16bit && (!message->qspi_mode)) {
        if(spi_bit_width_flag == SPI_8BIT_MODE) {
            // disable spi
            ctl = SPI_CTL0(SPI0);
            ctl &= ~SPI_CTL0_SPIEN;
            SPI_CTL0(SPI0) = ctl;
            // change to 16bit
            ctl |= SPI_CTL0_FF16;
            SPI_CTL0(SPI0) = ctl;
            // enable spi
            ctl |= SPI_CTL0_SPIEN;
            SPI_CTL0(SPI0) = ctl;

            // dma mem & periph width
            ctl = DMA_CHCTL(DMA0, DMA_CH2);
            ctl &= ~(DMA_CHXCTL_MWIDTH | DMA_CHXCTL_PWIDTH);
            ctl |= (DMA_MEMORY_WIDTH_16BIT | DMA_PERIPHERAL_WIDTH_16BIT);
            DMA_CHCTL(DMA0, DMA_CH2) = ctl;

            // dma mem & periph width
            ctl = DMA_CHCTL(DMA0, DMA_CH1);
            ctl &= ~(DMA_CHXCTL_MWIDTH | DMA_CHXCTL_PWIDTH);
            ctl |= (DMA_MEMORY_WIDTH_16BIT | DMA_PERIPHERAL_WIDTH_16BIT);
            DMA_CHCTL(DMA0, DMA_CH1) = ctl;

            spi_bit_width_flag = SPI_16BIT_MODE;
        }
        // spi-mode 16bit width
        DMA_CHCNT(DMA0, DMA_CH2) = (message->length >> 1);
        DMA_CHCNT(DMA0, DMA_CH1) = (message->length >> 1);

    }else {
        if(spi_bit_width_flag == SPI_16BIT_MODE) {
            // disable spi
            ctl = SPI_CTL0(SPI0);
            ctl &= ~SPI_CTL0_SPIEN;
            SPI_CTL0(SPI0) = ctl;
            // change to 8bit
            ctl &= ~SPI_CTL0_FF16;
            SPI_CTL0(SPI0) = ctl;
            // enable spi
            ctl |= SPI_CTL0_SPIEN;
            SPI_CTL0(SPI0) = ctl;

            // dma mem & periph width
            ctl = DMA_CHCTL(DMA0, DMA_CH2);
            ctl &= ~(DMA_CHXCTL_MWIDTH | DMA_CHXCTL_PWIDTH);
            ctl |= (DMA_MEMORY_WIDTH_8BIT | DMA_PERIPHERAL_WIDTH_8BIT);
            DMA_CHCTL(DMA0, DMA_CH2) = ctl;

            // dma mem & periph width
            ctl = DMA_CHCTL(DMA0, DMA_CH1);
            ctl &= ~(DMA_CHXCTL_MWIDTH | DMA_CHXCTL_PWIDTH);
            ctl |= (DMA_MEMORY_WIDTH_8BIT | DMA_PERIPHERAL_WIDTH_8BIT);
            DMA_CHCTL(DMA0, DMA_CH1) = ctl;

            spi_bit_width_flag = SPI_8BIT_MODE;
        }
        // update DMA transfer count register
        DMA_CHCNT(DMA0, DMA_CH2) = message->length;
        DMA_CHCNT(DMA0, DMA_CH1) = message->length;
    }
#endif
    if(message->recv_buf != RT_NULL) {
        if(message->qspi_mode) {
            if(qspi_io_flag & QPI_IN_MASK) {
                // PA7(D0) PA6(D1) PA3(D3) PA2(D2) IN_FLOATING 50M
                GPIO_CTL0(GPIOA) &= 0x00FF00FF;
                GPIO_CTL0(GPIOA) |= 0x44004400;
                // SPI is in quad wire read mode
                SPI_QCTL(SPI0) = 0x03;
                qspi_io_flag = IO_FLAG_QPI_IN_MODE;
            }
        }else {
            if(qspi_io_flag & SPI_INOUT_MASK) {
                // SPI0_MOSI_PIN PA7 AF_PP 50M
                // SPI0_MISO_PIN PA6 IN_FLOATING 50M
                GPIO_CTL0(GPIOA) &= 0x00FFFFFF;
                GPIO_CTL0(GPIOA) |= 0xB4000000;
                // SPI is in single wire mode
                SPI_QCTL(SPI0) = 0x0;
                qspi_io_flag = IO_FLAG_SPI_INOUT_MODE;
            }
        }
        // dummy read to clear 'RBNE' flag
        *((uint32_t *)message->recv_buf) = SPI_DATA(SPI0);

        // 接收时需要由主机端发送Dummy Bytes提供时钟
        // TX DMA
        DMA_CHMADDR(DMA0, DMA_CH2) = RX_DUMMY_BYTE_ADDR;
        DMA_CHCTL(DMA0, DMA_CH2) |= DMA_CHXCTL_CHEN;

        // RX DMA
        // attention: These bits can not be written when CHEN in the DMA_CHxCTL register is ‘1’.
        DMA_CHMADDR(DMA0, DMA_CH1) = (uint32_t)message->recv_buf;
        // enable DMA channel, 'counter register' will be read-only after this operation.
        DMA_CHCTL(DMA0, DMA_CH1) |= DMA_CHXCTL_CHEN;
        // When the TBE or RBNE bit in SPI_STAT is set, it will
        // generate a DMA request at corresponding DMA channel.
        SPI_CTL1(SPI0) |= (SPI_CTL1_DMATEN | SPI_CTL1_DMAREN);

        dma_rx_done = 0;
        dma_tx_done = 0;

        while(!dma_tx_done) {};

        while(SPI_STAT(SPI0) & SPI_FLAG_TRANS){};

        while(!dma_rx_done) {};

        DMA_CHCTL(DMA0, DMA_CH1) &= ~DMA_CHXCTL_CHEN;
        DMA_CHCTL(DMA0, DMA_CH2) &= ~DMA_CHXCTL_CHEN;

    }else if(message->send_buf != RT_NULL) {
        if(message->qspi_mode) {
            if(qspi_io_flag & QPI_OUT_MASK) {
                GPIO_CTL0(GPIOA) &= 0x00FF00FF;
                GPIO_CTL0(GPIOA) |= 0xBB00BB00;
                // SPI is in quad wire write mode
                SPI_QCTL(SPI0) = 0x01;
                qspi_io_flag = IO_FLAG_QPI_OUT_MODE;
            }
        }else {
            if(qspi_io_flag & SPI_INOUT_MASK) {
                GPIO_CTL0(GPIOA) &= 0x00FFFFFF;
                GPIO_CTL0(GPIOA) |= 0xB4000000;
                SPI_QCTL(SPI0) = 0x0;
                qspi_io_flag = IO_FLAG_SPI_INOUT_MODE;
            }
        }

        DMA_CHMADDR(DMA0, DMA_CH2) = (uint32_t)message->send_buf;
        DMA_CHCTL(DMA0, DMA_CH2) |= DMA_CHXCTL_CHEN;
        SPI_CTL1(SPI0) |= SPI_CTL1_DMATEN;

        dma_tx_done = 0;

        while(!dma_tx_done) {};

        // 在SPI外设时钟速度较慢的情况下，DMA传送完成，但SPI外设还未发送完成，直接向下执行会造成CS线提前拉高
        // 此处等待SPI外设传输完成'Transmitting on-going bit'，最差情况：在时钟为SPI_CLK_0P9M情况下会等待6个SPI时钟周期
        while(SPI_STAT(SPI0) & SPI_FLAG_TRANS){};

        DMA_CHCTL(DMA0, DMA_CH2) &= ~DMA_CHXCTL_CHEN;
    }

    /* release CS */
    if (message->cs_release) {
        GPIO_BOP(GPIOA) = SPI0_CS_PIN;
    }

    return message->length;
}

#endif
