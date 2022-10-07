//
// Created by Yanye on 8/29/2022.
//

#ifndef _PSRAM_H
#define _PSRAM_H

#include "stdbool.h"
#include "stdint.h"

// 使用虚拟地址访问PSRAM, 范围：0x0 ~ 0x7FFFFF
// vaddr组成
// MSB <--- 32bit地址 --->   LSB
// | 22bit页索引 | 10bit页内偏移 |

// SRAM分页描述符
struct page_frame {
    // SRAM分页首地址，以CACHE_BASE为基地址
    uint32_t caddr;
    // 虚拟/物理页面号：0~8191
    uint16_t page;
    // SRAM页是否正在使用
    uint16_t inuse : 1;
    // SRAM页是否被更改，标记为dirty的页交换时需要回写PSRAM
    uint16_t dirty : 1;
    // 保留字段
    uint16_t : 14;
};

// 快速地址转换缓存
struct quick_conv {
    uint16_t valid;
    // SRAM页框号
    uint16_t cpage;
    // 映射的虚拟起始地址
    uint32_t vaddr_start;
    // 映射的虚拟结束地址
    uint32_t vaddr_end;
};

// page control interface.
int page_init(void);
int8_t update_lru_stack(uint8_t cpage);
uint32_t page_va2pa(uint32_t vaddr, struct page_frame **page);
void page_vaddr_bounds(uint32_t vaddr, uint32_t *vstart, uint32_t *vend);
void page_paddr_bounds(uint32_t paddr, uint32_t *pstart, uint32_t *pend);
void page_mark_dirty(struct page_frame *page);
bool page_mark_dirty_ex(uint32_t vaddr);

// PSRAM read/write interface.
uint8_t psram_bread(uint32_t vaddr);
void psram_bwrite(uint32_t vaddr, uint8_t value);

uint16_t psram_sread(uint32_t vaddr);
void psram_swrite(uint32_t vaddr, uint16_t value);

uint32_t psram_iread(uint32_t vaddr);
void psram_iwrite(uint32_t vaddr, uint32_t value);

#endif //_PSRAM_H
