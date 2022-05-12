/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT pimoroni_pim447

#include <device.h>
#include <drivers/i2c.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "pim447.h"

LOG_MODULE_REGISTER(PIM447, CONFIG_SENSOR_LOG_LEVEL);

static int pim447_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct pim447_data *data = dev->data;
	const struct device *i2c = pim447_i2c_device(dev);
	uint8_t address = pim447_i2c_address(dev);
	uint8_t tx_buf[] = {
		PIM447_CMD_READ_LEFT
	};
	uint8_t rx_buf[5];

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	if (i2c_write_read(i2c, address, tx_buf, sizeof(tx_buf),
			   rx_buf, sizeof(rx_buf)) < 0) {
		LOG_DBG("Failed to read sample!");
		return -EIO;
	}

	data->dx = (int16_t)rx_buf[1] - rx_buf[0];
	data->dy = (int16_t)rx_buf[2] - rx_buf[3];
	data->last_swtch = data->swtch;
	data->swtch = rx_buf[4];
	LOG_DBG("Sample fetched from PIM447 left %x, right %x, up %x, down %x, press_count %x, press_state %x, was_pressed %x",
		rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], (rx_buf[4] & ~PIM477_SWITCH_STATE_FLAG), (rx_buf[4] & PIM477_SWITCH_STATE_FLAG) >> 7, (data->last_swtch & PIM477_SWITCH_STATE_FLAG) >> 7
	);

	return 0;
}

static int pim447_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct pim447_data *data = dev->data;

	if (chan == SENSOR_CHAN_POS_DX) {
		val->val1 = data->dx;
	} else if (chan == SENSOR_CHAN_POS_DY) {
		val->val1 = data->dy;
	} else if (chan == SENSOR_CHAN_POS_DZ) {
		val->val1 = (((data->last_swtch & PIM477_SWITCH_STATE_FLAG) >> 7) << 1) | ((data->swtch & PIM477_SWITCH_STATE_FLAG) >> 7); // pressed status
		val->val2 = data->swtch & ~PIM477_SWITCH_STATE_FLAG; // number of clicks
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int pim447_led_set(const struct device *dev,
		   uint8_t led_register,
		   uint8_t offset,
		   const struct sensor_value *val)
{
	const struct device *i2c = pim447_i2c_device(dev);
	uint8_t address = pim447_i2c_address(dev);
	uint8_t tx_buf[] = {
		led_register,
		(val->val1 >> offset) & 0xFF,
	};

	if (i2c_write(i2c, tx_buf, sizeof(tx_buf), address) < 0) {
		LOG_DBG("Failed to set trackball LED");
		return -EIO;
	}

	return 0;
}

int pim447_attr_set(const struct device *dev,
		    enum sensor_channel chan,
		    enum sensor_attribute attr,
		    const struct sensor_value *val)
{
	const struct device *i2c = pim447_i2c_device(dev);
	uint8_t address = pim447_i2c_address(dev);
	enum pim447_sensor_attribute pim447_attr = (enum pim447_sensor_attribute)attr;
	if (pim447_attr == PIM447_SENSOR_ATTR_LED) {
		uint8_t tx_buf[] = {
			PIM447_CMD_LED_RED,
			(val->val1 >> 24) & 0xFF,
			(val->val1 >> 16) & 0xFF,
			(val->val1 >>  8) & 0xFF,
			(val->val1 >>  0) & 0xFF,
		};

		if (i2c_write(i2c, tx_buf, sizeof(tx_buf), address) < 0) {
			LOG_DBG("Failed to set the trackball LED attributes");
			return -EIO;
		}
	} else if (pim447_attr == PIM447_SENSOR_ATTR_LED_R) {
		return pim447_led_set(dev, PIM447_CMD_LED_RED, 24, val);
	} else if (pim447_attr == PIM447_SENSOR_ATTR_LED_G) {
		return pim447_led_set(dev, PIM447_CMD_LED_GREEN, 16, val);
	} else if (pim447_attr == PIM447_SENSOR_ATTR_LED_B) {
		return pim447_led_set(dev, PIM447_CMD_LED_BLUE, 8, val);
	} else if (pim447_attr == PIM447_SENSOR_ATTR_LED_W) {
		return pim447_led_set(dev, PIM447_CMD_LED_WHITE, 0, val);
	} else {
		LOG_DBG("Unsupported trackball LED attribute %d", pim447_attr);
		return -ENOTSUP;
	}
	LOG_INF("Successfully written PIM447 led with %x", val->val1);

	return 0;
}

static const struct sensor_driver_api pim447_driver_api = {
#ifdef CONFIG_PIM447_TRIGGER
	.trigger_set = pim447_trigger_set,
#endif
	.attr_set = pim447_attr_set,
	.sample_fetch = pim447_sample_fetch,
	.channel_get = pim447_channel_get,
};

static uint16_t pim447_version(const struct device *dev)
{
	const struct device *i2c = pim447_i2c_device(dev);
	uint8_t address = pim447_i2c_address(dev);
	uint8_t tx_buf[] = {
		PIM447_CMD_READ_CHIP_ID_LOW
	};
	uint8_t rx_buf[2];

	if (i2c_write_read(i2c, address, tx_buf, sizeof(tx_buf),
			   rx_buf, sizeof(rx_buf)) < 0) {
		LOG_DBG("Failed to read chip version sample!");
		return -EIO;
	}

	return rx_buf[0] | (rx_buf[1] << 8);
}


static int pim447_init(const struct device *dev)
{
	struct pim447_data *data = dev->data;
	const struct pim447_config *cfg = dev->config;
	const struct device *i2c = device_get_binding(cfg->bus_name);

	if (i2c == NULL) {
		LOG_DBG("Failed to get pointer to %s device!",
			cfg->bus_name);
		return -EINVAL;
	}
	data->bus = i2c;

	if (!cfg->base_address) {
		LOG_DBG("No I2C address");
		return -EINVAL;
	}
	data->dev = dev;

	if (pim447_version(dev) != PIM447_CHIP_ID) {
		LOG_ERR("Invalid chip ID %x for PIM447 device at I2C address %x", pim447_version(dev), pim447_i2c_address(dev));
		return -EINVAL;
	}

#ifdef CONFIG_PIM447_TRIGGER
	if (pim447_init_interrupt(dev) < 0) {
		LOG_DBG("Failed to initialize interrupt");
		return -EIO;
	}
#endif

    // Test set LED to red
	struct sensor_value val;
	val.val1 = 0xf000000;
    pim447_attr_set(dev, 0, PIM447_SENSOR_ATTR_LED, &val);
	// Initialize data
	data->swtch = 0;
	data->last_swtch = 0;
	LOG_INF("Successfully initialized PIM447");

	return 0;
}

#ifdef CONFIG_PIM447_TRIGGER
#define PIM447_INST(n)\
    struct pim447_data pim447_data_##n;\
    const struct pim447_config pim447_cfg_##n = { \
        .bus_name = DT_INST_BUS_LABEL(n),\
        .alert_gpio_name = DT_INST_GPIO_LABEL(n, alert_gpios),\
        .base_address = DT_INST_REG_ADDR(n), \
        .alert_pin = DT_INST_GPIO_PIN(n, alert_gpios), \
        .alert_flags = DT_INST_GPIO_FLAGS(n, alert_gpios),\
    }; \
    DEVICE_DT_INST_DEFINE(n, pim447_init, device_pm_control_nop, &pim447_data_##n, &pim447_cfg_##n,      \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &pim447_driver_api);
#else
#define PIM447_INST(n)\
    struct pim447_data pim447_data_##n;\
    const struct pim447_config pim447_cfg_##n = { \
        .bus_name = DT_INST_BUS_LABEL(n),\
        .base_address = DT_INST_REG_ADDR(n), \
    };\
    DEVICE_DT_INST_DEFINE(n, pim447_init, device_pm_control_nop, &pim447_data_##n, &pim447_cfg_##n,      \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &pim447_driver_api);
#endif

DT_INST_FOREACH_STATUS_OKAY(PIM447_INST)

