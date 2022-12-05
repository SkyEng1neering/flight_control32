#include <stdio.h>
#include "flash.h"

#define FLASH_PRINTF                    printf
#define PAGE_INITIAL_VALUE              0xFFFFFFFF

bool flash_unlock() {
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }
    return true;
}

bool flash_lock() {
    if (HAL_FLASH_Lock() != HAL_OK) {
        return false;
    }
    return true;
}

bool flash_erase_config_page() {
    FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.Banks        = FLASH_BANK_1;
    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress  = FLASH_CONFIG_PAGE_ADDRESS;
    EraseInitStruct.NbPages      = 0x01;

    uint32_t PageError = 0x00;
    if ((HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) ||
            (PageError != PAGE_INITIAL_VALUE)) {
        return false;
    }
    return true;
}

bool flash_write(uint32_t flashAddress, uint32_t data) {
    uint8_t stat = HAL_ERROR;
    uint32_t startTick = HAL_GetTick();
    while ((stat != HAL_OK) && (HAL_GetTick() - startTick < FLASH_WRITE_TIMEOUT_MS)) {
        stat = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress, data);
    }
    if (stat != HAL_OK) {
        return false;
    }
    return true;
}

bool flash_write_config_page(void* newDataBuf) {//it means that newDataBuf == 1024 b, like flash page size
    if (flash_unlock() != true) {
        FLASH_PRINTF("FLASH: Unlock error\n");
        return false;
    }
    if (flash_erase_config_page() != true) {
        FLASH_PRINTF("FLASH: Erase error\n");
        return false;
    }
    uint32_t *dataPtr = (uint32_t*)newDataBuf;
    uint32_t counter = 0;
    for (uint32_t i = FLASH_CONFIG_PAGE_ADDRESS; i < FLASH_CONFIG_PAGE_ADDRESS + FLASH_CONFIG_PAGE_SIZE; i += sizeof(uint32_t)) {
        FLASH_PRINTF("FLASH: Try to write: %u bytes, address: 0x%08lX, total written: %lu bytes\n", sizeof(uint32_t), i, counter*sizeof(uint32_t));
        if (flash_write(i, dataPtr[counter]) != true) {
            FLASH_PRINTF("FLASH: flash_write() error, ind: %lu\n", i);
            return false;
        }
        counter++;
    }
    if (flash_lock() != true) {
        FLASH_PRINTF("FLASH: Lock error\n");
        return false;
    }
    return true;
}

void flash_dump_config_page(void* data_ptr) {
    uint32_t *ptr = (uint32_t *)data_ptr;
    uint32_t counter = 0;
    for (uint32_t i = FLASH_CONFIG_PAGE_ADDRESS; i < FLASH_CONFIG_PAGE_ADDRESS + FLASH_CONFIG_PAGE_SIZE; i += sizeof(uint32_t)) {
        ptr[counter] = flash_read(i);
        counter++;
    }
}

uint32_t flash_read(uint32_t address) {
    return (*(__IO uint32_t*) address);
}

uint64_t flash_read_64(uint32_t address) {
    return (*(__IO uint64_t*) address);
}

uint8_t flash_read_8(uint32_t address) {
    return (*(__IO uint8_t*) address);
}
