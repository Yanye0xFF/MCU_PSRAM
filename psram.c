//
// Created by Yanye on 8/29/2022.
//
#include "psram.h"
#include "rtthread.h"
#include "rtdbg.h"
#include "drv_psram.h"

// 作为cache的SRAM物理基地址
// 20000000 + A000
#define CACHE_BASE    (0x2000A000u)

// 作为cache的SRAM大小(KB)
#define CACHE_SIZE    8

// PSRAM大小(KB)
#define PSRAM_SIZE    8192

// cache页小于等于PSRAM物理页大小，LY68L6400 Page size is 1K
// unit: byte
#define PAGE_SIZE         1024

#define PAGE_ADDR_BITS    10

// 作为cache的SRAM分页数量
#define SRAM_PAGE_NUM     8

// PSRAM的分页数量
#define PSRAM_PAGE_NUM    8192

// SRAM页面缓存，顺序编号0~7
struct page_frame sram_page_items[SRAM_PAGE_NUM]; // 96Bytes

// psram_*read/psram_*write读写快速地址转换缓存
struct quick_conv qcv[3]; // 36Bytes

// 使用LRU算法决定SRAM页面满时被置换出的页面，栈深度 == SRAM缓存页面数量
static uint8_t sram_page_lru_stack[SRAM_PAGE_NUM]; // 8Bytes
static int8_t lru_stack_top;

static rt_device_t psram_dev;

int page_init(void) {
    psram_dev = rt_device_find("psram");
    if(psram_dev == RT_NULL) {
        LOG_E("probe psram device fail.\n");
        return -1;
    }

    rt_err_t err = rt_device_open(psram_dev, RT_DEVICE_FLAG_RDWR);
    if(-RT_EIO == err) {
        // switch to spi mode
        rt_device_control(psram_dev, PSRAM_CMD_SWITCH_TO_SPI_MODE, RT_NULL);
        LOG_I("switch to spi mode\n");
    }
    err = rt_device_open(psram_dev, RT_DEVICE_FLAG_RDWR);
    if(RT_EOK != err) {
        LOG_E("psram device open fail: %d\n", err);
        return -1;
    }

    rt_memset(sram_page_items, 0x00, sizeof(struct page_frame) * SRAM_PAGE_NUM);
    for(uint32_t i = 0; i < SRAM_PAGE_NUM; i++) {
        sram_page_items[i].caddr = CACHE_BASE + (i << 10); // 1K/缓存页
    }
    // 失效地址转换缓存
    for(uint32_t i = 0; i < 3; i++) {
        qcv[i].valid = 0;
    }
    // 清空LRU栈
    rt_memset(sram_page_lru_stack, 0x00, SRAM_PAGE_NUM);
    lru_stack_top = -1;

    return 0;
}

/**
 * @brief 更新LRU页面访问记录栈
 * @param cpage 当前访问的SRAM缓存页面号: 0~(SRAM_PAGE_NUM - 1)
 * @return 被置换出的SRAM缓存页面号，-1表示无需置换
 * */
int8_t update_lru_stack(uint8_t cpage) {
    int8_t swap_page = -1;
    // sram_page_lru_stack: 0->栈底 7->栈顶
    for(int8_t i = lru_stack_top; i >= 0; i--) {
        // 自栈顶向栈底查找相同的cpage编号并将其移除
        if(sram_page_lru_stack[i] == cpage) {
            swap_page = (int8_t)(cpage & 0x7F);
            for(int8_t j = i; j < lru_stack_top; j++) {
                sram_page_lru_stack[j] = sram_page_lru_stack[j + 1];
            }
            lru_stack_top--;
            break;
        }
    }
    // 新页面号插入栈顶
    if(lru_stack_top >= (SRAM_PAGE_NUM - 1)) {
        // 没有找到重复的页面，移除栈底最近最久未使用页面
        swap_page = (int8_t)(sram_page_lru_stack[0] & 0x7F);
        for(uint32_t i = 0; i < (SRAM_PAGE_NUM - 1); i++) {
            sram_page_lru_stack[i] = sram_page_lru_stack[i + 1];
        }
        sram_page_lru_stack[lru_stack_top] = cpage;

    }else {
        lru_stack_top++;
        sram_page_lru_stack[lru_stack_top] = cpage;
    }
    /*
    printf("lru stack: ");
    for(uint32_t i = 0; i < SRAM_PAGE_NUM; i++) {
        printf("%d ", sram_page_lru_stack[i]);
    }
    printf("\nswap page:%d, top:%d\n", swap_page, lru_stack_top);
    */
    return swap_page;
}

uint8_t get_lru_stack_bottom(void) {
    return sram_page_lru_stack[0];
}

/**
 * @brief 标记page缓存页为脏页
 * @param page 被标记的SRAM缓存页
 * */
void page_mark_dirty(struct page_frame *page) {
    page->dirty = 1;
}

/**
 * @brief 标记vaddr所在的缓存页为脏页，vaddr对应的页面必须在SRAM缓存中
 * @param vaddr 被标记的页内虚拟地址
 * @return true: 标记成功，false: 当前虚拟地址所对应的页面不在缓存中
 * */
bool page_mark_dirty_ex(uint32_t vaddr) {
    uint32_t page_num = (vaddr >> 10);
    for(uint32_t i = 0; i < SRAM_PAGE_NUM; i++) {
        if(sram_page_items[i].inuse && (page_num == sram_page_items[i].page)) {
            sram_page_items[i].dirty = 1;
            //log_line("mark dirty page: %d", i);
            return true;
        }
    }
    return false;
}

/**
 * @brief 从PSRAM加载指定物理页的数据到SRAM缓存页
 * @param cpage SRAM缓存页面号: 0~(SRAM_PAGE_NUM - 1)
 * @param ppage PSRAM物理页面号: 0~(PSRAM_PAGE_NUM - 1)
 * */
void page_load_from_psram(uint8_t cpage, uint16_t ppage) {
    void *buffer = (void *)sram_page_items[cpage].caddr;
    psram_dev->read(psram_dev, (ppage * PAGE_SIZE), buffer, PAGE_SIZE);
}

/**
 * @brief 将SRAM缓存页写入PSRAM指定物理页
 * @param cpage SRAM缓存页面号: 0~(SRAM_PAGE_NUM - 1)
 * @param ppage PSRAM物理页面号: 0~(PSRAM_PAGE_NUM - 1)
 * */
void page_save_into_psram(uint8_t cpage, uint16_t ppage) {
    void *buffer = (void *)sram_page_items[cpage].caddr;
    psram_dev->write(psram_dev, (ppage * PAGE_SIZE), buffer, PAGE_SIZE);
}

/**
 * @brief 虚拟地址和物理地址转换
 * @note 返回的物理地址在当前页范围内可以直接随机访问，跨页访问需要手动换页或使用psram_*read/psram_*write访问
 *       如果使用物理地址直接随机访问且修改了内存数据，需要调用page_mark_dirty标记脏页，以便在交换时回写到PSRAM
 * @param vaddr 虚拟地址 0x0~0x7FFFFF
 * @param page vaddr虚拟地址对应的SRAM缓存页信息
 * @return vaddr对应SRAM cache页的物理地址，可以字节，半字，字为单位随机访问
 * */
uint32_t page_va2pa(uint32_t vaddr, struct page_frame **page) {
    uint32_t page_num = (vaddr >> 10);
    uint32_t paddr;
    // 查找SRAM缓存有效页面，如果缓存项有效且页面号相等则直接返回
    for(uint8_t i = 0; i < SRAM_PAGE_NUM; i++) {
        if(sram_page_items[i].inuse && (page_num == sram_page_items[i].page)) {
            update_lru_stack(i);
            paddr = sram_page_items[i].caddr + (vaddr & 0x3FFu);
            *page = (sram_page_items + i);
            //log_line("find cache page: %d, %08x", i, paddr);
            return paddr;
        }
    }
    // 查找SRAM缓存空闲页面，将数据从PSRAM载入
    for(uint8_t i = 0; i < SRAM_PAGE_NUM; i++) {
        if(!sram_page_items[i].inuse) {
            sram_page_items[i].page = page_num;
            sram_page_items[i].inuse = 1;
            sram_page_items[i].dirty = 0;
            update_lru_stack(i);
            page_load_from_psram(i, page_num);
            paddr = sram_page_items[i].caddr + (vaddr & 0x3FFu);
            *page = (sram_page_items + i);
            //log_line("find free page: %d, %08x", i, paddr);
            return paddr;
        }
    }
    // SRAM页面缓存满，获取LRU栈底页面号，进行页面置换操作
    uint8_t spage = get_lru_stack_bottom();
    // 标记为dirty的页需要回写，未标记的可直接丢弃
    if(sram_page_items[spage].dirty) {
        page_save_into_psram(spage, sram_page_items[spage].page);
    }
    page_load_from_psram(spage, page_num);
    sram_page_items[spage].page = page_num;
    sram_page_items[spage].inuse = 1;
    sram_page_items[spage].dirty = 0;
    // 更新LRU栈，将前面置换的SRAM缓存页面移入栈顶
    update_lru_stack(spage);

    // page_va2pa手动换页时，psram_*read/psram_*write可能使用已被交换出去的页
    // 因此需要查找快速地址转换缓存，失效spage缓存项
    for(uint32_t i = 0; i < 3; i++) {
        // b/s/i read/write使用独立的缓存，需要全部检查一遍
        if(qcv[i].valid && (qcv[i].cpage == spage)) {
            qcv[i].valid = 0;
        }
    }

    paddr = sram_page_items[spage].caddr + (vaddr & 0x3FFu);
    *page = (sram_page_items + spage);

    //log_line("find swap page: %d, %08x", spage, paddr);

    return paddr;
}

/**
 * @brief 计算虚拟地址vaddr的上下边界
 * @note for example: vaddr = 0xC0 then vstart = 0x0, vend = 0x3FF
 * @param vaddr 被计算的虚拟地址
 * @param vstart 虚拟地址的下边界
 * @param vend 虚拟地址的上边界
 * @return none
 * */
void page_vaddr_bounds(uint32_t vaddr, uint32_t *vstart, uint32_t *vend) {
    *vstart = (vaddr & 0xFFFFFC00u);
    *vend = (vaddr & 0xFFFFFC00u) + ((uint32_t)1 << PAGE_ADDR_BITS) - 1u;
}

/**
 * @brief 计算物理地址paddr的上下边界
 * @note SRAM缓存页地址paddr和虚拟地址页大小一样，计算方法一样
 * */
void page_paddr_bounds(uint32_t paddr, uint32_t *pstart, uint32_t *pend) {
    *pstart = (paddr & 0xFFFFFC00u);
    *pend = (paddr & 0xFFFFFC00u) + ((uint32_t)1 << PAGE_ADDR_BITS) - 1u;
}

#define RW_UINT32    2
#define RW_UINT16    1
#define RW_UINT8     0

#define MEM_READ     1
#define MEM_WRITE    0

/**
 * @brief 读操作不会标记脏页，写操作会标记脏页，都会使用或更新快速地址转换缓存qcv[cache_idx]
 * @param type 读写的数据类型RW_UINT32, RW_UINT16 or RW_UINT8
 * @param rw 1: 读操作，0: 写操作
 * @param vaddr 读写的虚拟地址，需要按类型对齐
 * @param data 写操作输入值，读操作时填0
 * @return 读操作返回值，可强制转换成所需类型，写操作返回值忽略
 * */
static uint32_t page_mem_rw(uint32_t type, uint32_t rw, uint32_t vaddr, uint32_t data) {
    const uint32_t mask_lut[] = {0xFF, 0xFFFF, 0xFFFFFFFF};

    volatile uint32_t *ptr;
    struct page_frame *page;

    uint32_t paddr, vaddr_aligned;
    uint32_t value, offset;
    uint32_t idx = type;

    // 统一使用4字节地址读写，再根据type指定的数据类型丢弃某些位
    vaddr_aligned = (vaddr & 0xFFFFFFFC);
    offset = (vaddr - vaddr_aligned) << 3;

    if(qcv[idx].valid
        && (vaddr >= qcv[idx].vaddr_start)
        && (vaddr <= qcv[idx].vaddr_end)) {

        page = (sram_page_items + qcv[idx].cpage);
        paddr = page->caddr + (vaddr_aligned & 0x3FF);

    }else {
        paddr = page_va2pa(vaddr_aligned, &page);
        qcv[idx].valid = 1;
        qcv[idx].cpage = ((uintptr_t)page - (uintptr_t)sram_page_items) / sizeof (struct page_frame);
        page_vaddr_bounds(vaddr, &qcv[idx].vaddr_start, &qcv[idx].vaddr_end);
    }
    // read
    ptr = (volatile uint32_t *)(uintptr_t)paddr;
    value = *ptr;

    if(rw) {
        // modify
        value = (value >> offset) & mask_lut[type];
    }else {
        page->dirty = 1;
        // modify
        value &= ~(mask_lut[type] << offset);
        value |= ((data & mask_lut[type]) << offset);
        // write
        *ptr = value;
    }
    // return
    return value;
}

/**
 * @param vaddr 被读取的虚拟地址
 * */
uint8_t psram_bread(uint32_t vaddr) {
    uint8_t res;
    res = (uint8_t)page_mem_rw(RW_UINT8, MEM_READ, vaddr, 0);
    return res;
}

/**
 * @param vaddr 被写入的虚拟地址
 * @param value 写入的8bit数值
 * */
void psram_bwrite(uint32_t vaddr, uint8_t value) {
    page_mem_rw(RW_UINT8, MEM_WRITE, vaddr, value);
}

/**
 * @param vaddr 被读取的虚拟地址，必须在2字节边界
 * */
uint16_t psram_sread(uint32_t vaddr) {
    uint16_t res;
    res = (uint16_t)page_mem_rw(RW_UINT16, MEM_READ, vaddr, 0);
    return res;
}

/**
 * @param vaddr 被写入的虚拟地址，必须在2字节边界
 * @param value 写入的16bit数值
 * */
void psram_swrite(uint32_t vaddr, uint16_t value) {
    page_mem_rw(RW_UINT16, MEM_WRITE, vaddr, value);
}

/**
 * @param vaddr 被读取的虚拟地址，必须在4字节边界
 * */
uint32_t psram_iread(uint32_t vaddr) {
    uint32_t res;
    res = (uint32_t)page_mem_rw(RW_UINT32, MEM_READ, vaddr, 0);
    return res;
}

/**
 * @param vaddr 被写入的虚拟地址，必须在4字节边界
 * @param value 写入的32bit数值
 * */
void psram_iwrite(uint32_t vaddr, uint32_t value) {
    page_mem_rw(RW_UINT32, MEM_WRITE, vaddr, value);
}
