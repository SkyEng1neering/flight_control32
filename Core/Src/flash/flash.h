#ifndef FLASH_FLASH_H_
#define FLASH_FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "string.h"
#include "stm32f1xx.h"

#define FLASH_WRITE_TIMEOUT_MS              15000
#define FLASH_START_ADDRESS                 0x08000000
#define FLASH_CONFIG_PAGE_ADDRESS           0x0801FC00
#define FLASH_CONFIG_PAGE_SIZE              1024

bool flash_unlock();
bool flash_lock();
bool flash_erase_config_page();
bool flash_write(uint32_t dataAddress, uint32_t flashAddress);
bool flash_write_config_page(void* newDataBuf);
void flash_dump_config_page(void* data_ptr);
uint32_t flash_read(uint32_t address);
uint64_t flash_read_64(uint32_t address);
uint8_t flash_read_8(uint32_t address);

#ifdef __cplusplus
}
#endif
#endif /* FLASH_FLASH_H_ */
