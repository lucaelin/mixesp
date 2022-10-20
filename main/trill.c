#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "i2c.h"
#include "trill.h"

#define SENSOR_ADDR 0x20 /*!< Slave address of the MPU9250 sensor */

#define COMMAND_REG 0x00 /*!< Register addresses of the "who am I" register */
#define DATA_REG 0x04

#define COMMAND_NONE 0x00
#define COMMAND_MODE 0x01
#define COMMAND_SCAN_SETTING 0x02
#define COMMAND_BASELINE_UPDATE 0x06
/*
    kCommandPrescaler = 3
    kCommandNoiseThreshold = 4
    kCommandIdac = 5
    kCommandBaselineUpdate = 6
    kCommandMinimumSize = 7
    kCommandAutoScanInterval = 16
*/
#define COMMAND_IDENTIFY 0xFF

static esp_err_t i2c_register_write_byte(uint8_t reg_addr, uint8_t data)
{
  int ret;
  uint8_t write_buf[2] = {reg_addr, data};

  ret = i2c_master_write_to_device(I2C_MASTER_NUM, SENSOR_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

  return ret;
}

static esp_err_t trill_cmd_no_arg_read(uint8_t cmd, uint8_t *read_buf, size_t read_len)
{
  uint8_t data[2] = {COMMAND_REG, cmd};
  if (read_len)
  {
    return i2c_master_write_read_device(I2C_MASTER_NUM, SENSOR_ADDR, &data, sizeof(data), read_buf, read_len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  }
  else
  {
    return i2c_master_write_to_device(I2C_MASTER_NUM, SENSOR_ADDR, &data, sizeof(data), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  }
}

static esp_err_t trill_cmd_one_arg_read(uint8_t cmd, uint8_t arg, uint8_t *read_buf, size_t read_len)
{
  uint8_t data[3] = {COMMAND_REG, cmd, arg};
  if (read_len)
  {
    return i2c_master_write_read_device(I2C_MASTER_NUM, SENSOR_ADDR, &data, sizeof(data), read_buf, read_len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  }
  else
  {
    return i2c_master_write_to_device(I2C_MASTER_NUM, SENSOR_ADDR, &data, sizeof(data), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  }
}

static esp_err_t trill_cmd_two_arg_read(uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t *read_buf, size_t read_len)
{
  uint8_t data[4] = {COMMAND_REG, cmd, arg1, arg2};
  if (read_len)
  {
    return i2c_master_write_read_device(I2C_MASTER_NUM, SENSOR_ADDR, &data, sizeof(data), read_buf, read_len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  }
  else
  {
    return i2c_master_write_to_device(I2C_MASTER_NUM, SENSOR_ADDR, &data, sizeof(data), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  }
}

static esp_err_t trill_data_read(uint8_t *read_buf, size_t read_len)
{
  uint8_t data[1] = {DATA_REG};
  if (read_len)
  {
    return i2c_master_write_read_device(I2C_MASTER_NUM, SENSOR_ADDR, &data, sizeof(data), read_buf, read_len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  }
  else
  {
    return i2c_master_write_to_device(I2C_MASTER_NUM, SENSOR_ADDR, &data, sizeof(data), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
  }
}

trill_identity trill_cmd_identify()
{
  uint8_t data[3];
  ESP_ERROR_CHECK(trill_cmd_no_arg_read(COMMAND_IDENTIFY, data, sizeof(data)));
  trill_identity ret = {data[1], data[2]};
  return ret;
}

trill_mode trill_cmd_mode(uint8_t mode)
{
  uint8_t data[0];
  ESP_ERROR_CHECK(trill_cmd_one_arg_read(COMMAND_MODE, mode, data, sizeof(data)));
  trill_mode ret = {mode};
  return ret;
}

trill_scan_setting trill_cmd_scan_setting(uint8_t speed, uint8_t num_bits)
{
  uint8_t data[0];
  ESP_ERROR_CHECK(trill_cmd_two_arg_read(COMMAND_SCAN_SETTING, speed, num_bits, data, sizeof(data)));
  trill_scan_setting ret = {speed, num_bits};
  return ret;
}

trill_baseline trill_cmd_baseline_update()
{
  uint8_t data[0];
  ESP_ERROR_CHECK(trill_cmd_no_arg_read(COMMAND_BASELINE_UPDATE, data, sizeof(data)));
  trill_baseline ret = {};
  return ret;
}

trill_touches trill_read_touch()
{
  trill_touches ret;
  ret.num_touches = 0;
  ESP_ERROR_CHECK(trill_data_read(ret.data, 20));

  for (uint8_t i = 0; i < 5; i++)
  {
    ret.touches[i].x = 0;
    ret.touches[i].y = ret.data[i * 2] * 0xa00 + ret.data[i * 2 + 1];
    ret.touches[i].size = ret.data[10 + i * 2] * 0xa00 + ret.data[10 + i * 2 + 1];

    if (ret.touches[i].size != 0)
    {
      ret.num_touches++;
    }
  }
  return ret;
}
