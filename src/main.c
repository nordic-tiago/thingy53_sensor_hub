/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/settings/settings.h>

#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/sensor.h>
#include <drivers/adc.h>
#include <hal/nrf_saadc.h>
#include "services/sensor_hub_service.h"

//adc settings
#define ADC_RESOLUTION 14
#define ADC_MAX 1024
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_GAIN_INT 6
#define ADC_REFERENCE ADC_REF_INTERNAL
//#define ADC_REF_INTERNAL_MV 600UL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_1ST_CHANNEL_ID 0
//#define BUFFER_SIZE 1

/*define BATTERY_VOLTAGE(sample) (sample * ADC_GAIN_INT	\
				 * ADC_REF_INTERNAL_MV / ADC_MAX)*/

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED          DK_LED2 //green LED
#define UPDATE_INTERVAL  		500

static const struct bt_data ad[] = 
{
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = 
{

};

const struct device *adc_dev;
static const struct adc_channel_cfg m_vdd_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	//.channel_id = ADC_1ST_CHANNEL_ID,
	.input_positive = NRF_SAADC_INPUT_AIN2,
};

struct bt_conn *m_connection_handle = NULL;
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	m_connection_handle = conn;

	printk("Connected\n");	
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);

	m_connection_handle = NULL;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected        = connected,
	.disconnected     = disconnected,
};

static int adc_sample(int16_t *value)
{
	int err;

	const struct adc_sequence sequence = {
		.channels = BIT(ADC_1ST_CHANNEL_ID),
		.buffer = value,
		.buffer_size = sizeof(value),
		.oversampling = 4,
		.calibrate = true,
		.resolution = ADC_RESOLUTION,
	};

	if (!adc_dev) 
	{
		return -1;
	}

	err = adc_read(adc_dev, &sequence);
	if (err) 
	{
        printk("adc_read() failed with code %d\n", err);
		return err;
	}
	
	/*#define BATTERY_VOLTAGE(sample) (sample * ADC_GAIN_INT	\
				 * ADC_REF_INTERNAL_MV / ADC_MAX)*/

	adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN_1_6, ADC_RESOLUTION, (int32_t*)value);
	*value = *value * 1680000 / 180000;
    /*for (int i = 0; i < BUFFER_SIZE; i++) 
	{
		value[i] = BATTERY_VOLTAGE(m_sample_buffer[i]);
	}*/

	return err;
}

static int sample_and_update_all_sensor_values(const struct device *bme688Dev, const struct device *bh1749Dev)
{
	int err;
	struct sensor_value temperature_value;
	struct sensor_value pressure_value;
	struct sensor_value humidity_value;
	struct sensor_value red_value;
	struct sensor_value green_value;
	struct sensor_value blue_value;
	
	//trigger sampling of bme688 sensors
	err = sensor_sample_fetch(bme688Dev);
	if(err)
	{
		printk("Failed to collect samples from bme688\n");
		return err;
	}

	//collect temperature sample and update characteristic
	err = sensor_channel_get(bme688Dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature_value);
	if(err)
	{
		printk("Failed to fetch temperature sample");
		return err;
	}
	sensor_hub_update_temperature(m_connection_handle, (uint8_t*)(&temperature_value.val1), sizeof(temperature_value.val1));

	//collect pressure sample and update characteristic
	err = sensor_channel_get(bme688Dev, SENSOR_CHAN_PRESS, &pressure_value);
	if(err)
	{
		printk("Failed to fetch pressure sample");
		return err;
	}
	sensor_hub_update_pressure(m_connection_handle, (uint8_t*)(&pressure_value.val1), sizeof(pressure_value.val1));

	//collect humidity sample and update characteristic
	err = sensor_channel_get(bme688Dev, SENSOR_CHAN_HUMIDITY, &humidity_value);
	if(err)
	{
		printk("Failed to fetch humidity sample");
		return err;
	}
	sensor_hub_update_humidity(m_connection_handle, (uint8_t*)(&humidity_value.val1), sizeof(humidity_value.val1));

	//trigger sampling of bh1749
	err = sensor_sample_fetch_chan(bh1749Dev, SENSOR_CHAN_ALL);
	/* The sensor does only support fetching SENSOR_CHAN_ALL */
	if (err) 
	{
		printk("sensor_sample_fetch failed err %d\n", err);
		return err;
	}
	
	//collect red light sample and update characteristic
	err = sensor_channel_get(bh1749Dev, SENSOR_CHAN_RED, &red_value);
	if (err) 
	{
		printk("sensor_channel_get failed err %d\n", err);
		return err;
	}
	sensor_hub_update_red_color(m_connection_handle, (uint8_t*)(&red_value.val1), sizeof(red_value.val1));

	//collect green light sample and update characteristic
	err = sensor_channel_get(bh1749Dev, SENSOR_CHAN_GREEN, &green_value);
	if (err) 
	{
		printk("sensor_channel_get failed err %d\n", err);
		return err;
	}
	sensor_hub_update_green_color(m_connection_handle, (uint8_t*)(&green_value.val1), sizeof(green_value.val1));

	//collect red light sample and update characteristic
	err = sensor_channel_get(bh1749Dev, SENSOR_CHAN_BLUE, &blue_value);
	if (err) 
	{
		printk("sensor_channel_get failed err %d\n", err);
		return err;
	}
	sensor_hub_update_blue_color(m_connection_handle, (uint8_t*)(&blue_value.val1), sizeof(blue_value.val1));

	//collect ADC VDD sample
	int16_t vdd_mv_sample = 0;
	adc_sample(&vdd_mv_sample);
	sensor_hub_update_adc_meas(m_connection_handle, (uint8_t*)(&vdd_mv_sample), sizeof(vdd_mv_sample));

	printk("All sensors sampled and characteristics updated!\n");

	return 0;
}

void main(void)
{
	uint8_t blink_status = 0;
	int err;

	printk("Starting Sensor Hub application\n");

	//setting up BH1749 light and color sensor */
	const struct device *bh1749rgbDev = DEVICE_DT_GET_ONE(rohm_bh1749);

	if (!device_is_ready(bh1749rgbDev)) 
	{
		printk("Sensor device not ready\n");
		return;
	}

	//setting up BME688 environmental sensor */
	const struct device *bme688SensorDev = DEVICE_DT_GET_ONE(bosch_bme680);

	if (!device_is_ready(bme688SensorDev)) 
	{
		printk("Sensor device not ready\n");
		return;
	}

	//setting up ADC
	adc_dev = DEVICE_DT_GET_ONE(nordic_nrf_saadc);

	if (!device_is_ready(adc_dev)) 
	{
		printk("ADC is not ready\n");
		return;
	}

	//NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
	k_msleep(10);

    err = adc_channel_setup(adc_dev, &m_vdd_channel_cfg);
    if (err) 
	{
	    printk("Error in ADC setup: %d\n", err);
		return;
	}

	//setting up pin to enable voltage divider, which is defined in the device tree
	struct gpio_dt_spec volt_div = GPIO_DT_SPEC_GET(DT_PATH(vbatt), power_gpios);

	if (!device_is_ready(volt_div.port)) {
		return;
	}

	gpio_pin_configure_dt(&volt_div, GPIO_OUTPUT);
	gpio_pin_set_dt(&volt_div, 1);

	err = dk_leds_init();
	if (err) 
	{
		printk("LEDs init failed (err %d)\n", err);
		return;
	}

	err = bt_enable(NULL);
	if (err) 
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) 
	{
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) 
	{
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");

	for (;;) 
	{
		
		if(!(blink_status % 2) && m_connection_handle)
		{
			//status LEDs are OFF, and there is a BLE central connected, time to take samples
			sample_and_update_all_sensor_values(bme688SensorDev, bh1749rgbDev);
		}

		//change LED status
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		
		//put thread to sleep for UPDATE_INTERVAL
		k_sleep(K_MSEC(UPDATE_INTERVAL));
	}
}
