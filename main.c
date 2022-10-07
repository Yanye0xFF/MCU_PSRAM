/*
 * main.c
 * Created on: May 1, 2022
 * Author: Yanye
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stdint.h"

#include "rtthread.h"
#include <string.h>

#include "psram.h"

int main(void) {

    int res = page_init();
    rt_kprintf("page init:%d\n", res);

    struct page_frame *page;
    uint32_t paddr, vaddr;
    
    // 使用虚拟地址转换的物理地址访问
    paddr = page_va2pa(0x0, &page);
    void *ptr = (void *)paddr;

    rt_memset(ptr, 0xFF, 1024);
    
    uint8_t *ptr2 = (uint8_t *)paddr;
    ptr2 = 0x11;

    page_mark_dirty(page);


    // 使用PSRAM读写接口
    uint32_t val = psram_iread(0x100000);

    psram_iwrite(0x200000, 0x11223344);
    
    return 0;
}
