/*
 * drv_psram.c
 * @brief
 * Created on: Sep 24, 2022
 * Author: Yanye
 */
#include <stdint.h>
#include "rtthread.h"
#include "drv_psram.h"
#include "rtconfig.h"
#include "drv_spi.h"

#if (GD_USE_FAST_QSPI)
#include "drv_fast_qspi.h"
#else
#include "rt_spi.h"
#endif

// 使用PSRAM 缓存第0页空间作为初始化数据发送空间
static uint8_t *init_xfer_buffer = (uint8_t *)0x2000A000u;

static uint8_t MFID;

// KGD[7:0]     Known Good Die
// ‘b0101_0101   FAIL
// ‘b0101_1101   PASS
// Default is FAIL die, and only mark PASS after all tests passed.
static uint8_t KGD;

// 48bits
static uint8_t EID[6];

#if (GD_USE_FAST_QSPI == 0)
static struct rt_spi_device *psram_itf_dev;
#endif

// SPI mode or QSPI mode
#define LY68L64_RSTEN    0x66u
#define LY68L64_RST      0x99u

// only in SPI mode
#define LY68L64_QUAD_MODE_ENTER  0x35u
// only available in QPI mode
#define LY68L64_QUAD_MODE_EXIT   0xF5u

// SPI Read ID‘h9F (available only in SPI mode)
#define LY68L64_READ_ID    0x9Fu
// factory value of MFID & EID
#define FACTORY_MFID       0x0du
#define FACTORY_KGD        0x5du

#define LY68L64_FAST_READ_QUAD    0xEBu
#define LY68L64_WRITE_QUAD        0x38u

#if (SPI0_ONLY_8BIT_WIDTH == 0)
uint32_t endian_swap(void *buffer, uint32_t size) {
    uint16_t msb, lsb;
    uint16_t *ptr = (uint16_t *)buffer;

    for(uint32_t i = 0, j = (size >> 1); i < j; i++) {
        msb = (ptr[i] << 8) & 0xFF00;
        lsb = (ptr[i] >> 8) & 0x00FF;
        ptr[i] = (msb | lsb);
    }

    return size;
}
#endif
// call by 'rt_device_init' or 'rt_device_open' once only
rt_err_t psram_init(rt_device_t dev) {
    // The Reset operation is used as a system (software) reset that puts the device in SPI standby mode
    // which is also the default mode after power-up.
    const struct rt_spi_configuration config = {
        .mode = (RT_SPI_MODE_0 | RT_SPI_MSB | RT_SPI_MASTER),
        .data_width = 8,
        .max_hz = SPI_CLK_30M
    };

#if (GD_USE_FAST_QSPI)
    fast_qspi_init(&config);
#else
    psram_itf_dev = (struct rt_spi_device *)rt_device_find("psram-itf");
    if(psram_itf_dev == RT_NULL) {
        return -RT_ENOSYS;
    }
    rt_device_open((rt_device_t)psram_itf_dev, RT_DEVICE_FLAG_RDWR);
    rt_spi_configure(psram_itf_dev, &config);
#endif

    struct rt_spi_message message;
    message.send_buf = init_xfer_buffer;
    message.recv_buf = RT_NULL;
    message.length = 1;
    message.next = RT_NULL;
    message.cs_take = 1;
    message.cs_release = 1;
    message.qspi_mode = 0;
    message.is_16bit = 0;

    // Reset Enable CMD(‘h66)
    init_xfer_buffer[0] = LY68L64_RSTEN;
#if (GD_USE_FAST_QSPI)
    fast_qspi_xfer(&message);
#else
    rt_spi_transfer_message(psram_itf_dev, &message);
#endif
    // release CS

    // Reset CMD(‘h99)
    init_xfer_buffer[0] = LY68L64_RST;
#if (GD_USE_FAST_QSPI)
    fast_qspi_xfer(&message);
#else
    rt_spi_transfer_message(psram_itf_dev, &message);
#endif

    // SPI Read ID Operation
    message.length = 4;
    message.cs_release = 0;
#if (SPI0_ONLY_8BIT_WIDTH == 0)
    message.is_16bit = 1;
#endif
    // Read ID(‘h9F) + 24bit Dummy Address
    init_xfer_buffer[0] = LY68L64_READ_ID;
    init_xfer_buffer[1] = 0x00;
    init_xfer_buffer[2] = 0x00;
    init_xfer_buffer[3] = 0x00;

#if (SPI0_ONLY_8BIT_WIDTH == 0)
    endian_swap(init_xfer_buffer, message.length);
#endif

#if (GD_USE_FAST_QSPI)
    fast_qspi_xfer(&message);
#else
    rt_spi_transfer_message(psram_itf_dev, &message);
#endif

    // MF ID(‘h0D) KGD(‘h5D) EID[47:0]
    message.send_buf = RT_NULL;
    message.recv_buf = init_xfer_buffer;
    message.length = 8;
    message.cs_take = 0;
    message.cs_release = 1;
#if (GD_USE_FAST_QSPI)
    fast_qspi_xfer(&message);
#else
    rt_spi_transfer_message(psram_itf_dev, &message);
#endif

#if (SPI0_ONLY_8BIT_WIDTH == 0)
    endian_swap(init_xfer_buffer, message.length);
#endif

    MFID = init_xfer_buffer[0];
    KGD = init_xfer_buffer[1];
    rt_memcpy(EID, (init_xfer_buffer + 2), sizeof(EID));

    for(int i = 0; i < 8; i++) {
        rt_kprintf("0x%02x ", init_xfer_buffer[i]);
    }
    rt_kprintf("\n");

    if((FACTORY_MFID != MFID) || (FACTORY_KGD != KGD)) {
        return -RT_EIO;
    }

    // SPI Quad Mode Enable Operation
    message.send_buf = init_xfer_buffer;
    message.recv_buf = RT_NULL;
    message.length = 1;
    message.next = RT_NULL;
    message.cs_take = 1;
    message.cs_release = 1;
    message.is_16bit = 0;

    init_xfer_buffer[0] = LY68L64_QUAD_MODE_ENTER;
#if (GD_USE_FAST_QSPI)
    fast_qspi_xfer(&message);
#else
    rt_spi_transfer_message(psram_itf_dev, &message);
#endif

    return RT_EOK;
}

// call by 'rt_device_open' every time
rt_err_t psram_open(rt_device_t dev, rt_uint16_t oflag) {
    // nothing to do ...
    return RT_EOK;
}

// call by 'rt_device_close' when ref_count == 0
rt_err_t psram_close(rt_device_t dev) {
    // nothing to do ...
    return RT_EOK;
}

rt_size_t psram_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size) {
    uint8_t header[8];
    header[0] = LY68L64_FAST_READ_QUAD;
    header[1] = (uint8_t)((pos >> 16) & 0xFF);
    header[2] = (uint8_t)((pos >> 8) & 0xFF);
    header[3] = (uint8_t)(pos & 0xFF);
    // dummy bytes
    header[4] = 0x00;
    header[5] = 0x00;
    header[6] = 0x00;

    struct rt_spi_message message;
    message.send_buf = header;
    message.recv_buf = RT_NULL;
    message.length = 7;
    message.next = RT_NULL;
    message.cs_take = 1;
    message.cs_release = 0;
    message.qspi_mode = 1;
    message.is_16bit = 0;
    // send 8bit cmd & 24bit address + 6 clk
#if (GD_USE_FAST_QSPI)
    fast_qspi_xfer(&message);
#else
    rt_spi_transfer_message(psram_itf_dev, &message);
#endif

    message.send_buf = RT_NULL;
    message.recv_buf = buffer;
    message.length = size;
    message.cs_take = 0;
    message.cs_release = 1;
    // write data
#if (GD_USE_FAST_QSPI)
    fast_qspi_xfer(&message);
#else
    rt_spi_transfer_message(psram_itf_dev, &message);
#endif

    return RT_EOK;
}
rt_size_t psram_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size) {
    uint8_t header[4];
    header[0] = LY68L64_WRITE_QUAD;
    header[1] = (uint8_t)((pos >> 16) & 0xFF);
    header[2] = (uint8_t)((pos >> 8) & 0xFF);
    header[3] = (uint8_t)(pos & 0xFF);

    struct rt_spi_message message;
    message.send_buf = header;
    message.recv_buf = RT_NULL;
    message.length = 4;
    message.next = RT_NULL;
    message.cs_take = 1;
    message.cs_release = 0;
    message.qspi_mode = 1;
    message.is_16bit = 0;

    // endian_swap(header, message.length);

    // send 8bit cmd & 24bit address
#if (GD_USE_FAST_QSPI)
    fast_qspi_xfer(&message);
#else
    rt_spi_transfer_message(psram_itf_dev, &message);
#endif

    message.send_buf = buffer;
    message.length = size;
    message.cs_take = 0;
    message.cs_release = 1;
    // write data
#if (GD_USE_FAST_QSPI)
    fast_qspi_xfer(&message);
#else
    rt_spi_transfer_message(psram_itf_dev, &message);
#endif

    return RT_EOK;
}

rt_err_t psram_control(rt_device_t dev, int cmd, void *args) {
    uint8_t header[4];
    struct rt_spi_message message;
    uint8_t *ptr;

    if(cmd == PSRAM_CMD_SWITCH_TO_SPI_MODE) {
        message.send_buf = header;
        message.recv_buf = RT_NULL;
        message.length = 1;
        message.next = RT_NULL;
        message.cs_take = 1;
        message.cs_release = 1;
        message.qspi_mode = 1;
        message.is_16bit = 0;

        header[0] = LY68L64_QUAD_MODE_EXIT;
        #if (GD_USE_FAST_QSPI)
            fast_qspi_xfer(&message);
        #else
            rt_spi_transfer_message(psram_itf_dev, &message);
        #endif

    }else if(PSRAM_CMD_GET_MFID == cmd) {
        ptr = (uint8_t *)args;
        *ptr = MFID;

    }else if(PSRAM_CMD_GET_KGD == cmd) {
        ptr = (uint8_t *)args;
        *ptr = KGD;

    }else if(PSRAM_CMD_GET_EID == cmd) {
        ptr = (uint8_t *)args;
        rt_memcpy(ptr, EID, sizeof(EID));

    }

    return RT_EOK;
}


static struct rt_device psram_device = {
    .type = RT_Device_Class_Block,
    .flag = 0,
    .open_flag = 0,
    .ref_count = 0,
    .device_id = 0,

    .rx_indicate = RT_NULL,
    .tx_complete = RT_NULL,

    // POSIX API
    .init = psram_init,
    .open = psram_open,
    .close = psram_close,
    .read = psram_read,
    .write = psram_write,
    .control = psram_control,

    .user_data = RT_NULL,
};

int rt_hw_psram_init(void) {
    rt_device_register(&psram_device, "psram", RT_DEVICE_FLAG_RDWR);
    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_psram_init);

