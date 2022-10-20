#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

typedef struct
{
  uint8_t board;
  uint8_t version;
} trill_identity;

trill_identity trill_cmd_identify();

typedef struct
{
  uint8_t mode;
} trill_mode;

trill_mode trill_cmd_mode(uint8_t mode);

typedef struct
{
  uint8_t speed;
  uint8_t num_bits;
} trill_scan_setting;

trill_scan_setting trill_cmd_scan_setting(uint8_t speed, uint8_t num_bits);

typedef struct
{
} trill_baseline;

trill_baseline trill_cmd_baseline_update();

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t size;
} trill_touch;

typedef struct
{
  uint8_t data[20];
  uint8_t num_touches;
  trill_touch touches[5];
} trill_touches;

trill_touches trill_read_touch();