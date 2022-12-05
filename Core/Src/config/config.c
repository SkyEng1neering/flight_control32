#include "config.h"
#include "flash.h"

static uint8_t conf_serialize_arr[FLASH_CONFIG_PAGE_SIZE];

void get_config(struct FlightConfig* conf_ptr) {
    flash_dump_config_page(conf_serialize_arr);
    memcpy(conf_ptr, conf_serialize_arr, sizeof(struct FlightConfig));
}

bool save_config(struct FlightConfig* conf_ptr) {
    memcpy(conf_serialize_arr, conf_ptr, sizeof(struct FlightConfig));
    return flash_write_config_page(conf_serialize_arr);
}
