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
#include <zephyr/drivers/adc.h>
#include <hal/nrf_saadc.h>
#include "services/sensor_hub_service.h"

// ---- ADC settings START ---- //
#define ADC_RESOLUTION 14

static const struct adc_channel_cfg adc_channel_cfg = {
	.gain = ADC_GAIN_1_6,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
	.input_positive = NRF_SAADC_INPUT_AIN2,
};

static struct adc_sequence adc_seq = {
	.channels = BIT(0),	
	.oversampling = 4,
	.calibrate = true,
	.resolution = ADC_RESOLUTION,
};
// ---- ADC settings END ---- //


// ---- Voltage divider settings START ---- //
/* The vbatt node in the board device tree contains the HW definition for the voltage divider, which can be applied
to the battery voltage via an enable pin. That information will be retrieved to populate an instance of the
structure defined below.
The Voltage divider can be found on page 4 of the Thingy:53 schematics.
Board device tree can be found in <sdk_location>\<sdk_version>\zephyr\boards\arm\thingy53_nrf5340\thingy53_nrf5340_common.dts -> vbatt */
struct voltage_divider_config {
	struct gpio_dt_spec enable_pin;
	uint32_t output_ohm;
	uint32_t full_ohm;
};


static const struct voltage_divider_config volt_div_conf = {
	.enable_pin = GPIO_DT_SPEC_GET(DT_PATH(vbatt), power_gpios),
	.output_ohm = DT_PROP(DT_PATH(vbatt), output_ohms),
	.full_ohm = DT_PROP(DT_PATH(vbatt), full_ohms),
};
// ---- Voltage divider settings END ---- //


// ---- APP settings START ---- //
#define RUN_STATUS_LED          DK_LED2 //Green LED on Thingy:53
#define UPDATE_INTERVAL  		500
// ---- APP settings END ---- //


// ---- Bluetooth settings START ---- //
#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

//BT advertisement packet data
static const struct bt_data ad[] = 
{
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

//BT scan response data
static const struct bt_data sd[] = 
{

};
// ---- Bluetooth settings END ---- //

//BT globals and callbacks
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

//Local function prototypes
static int sample_and_update_all_sensor_values(const struct device *bme688Dev, const struct device *bh1749Dev, const struct device *adc_dev);

// Main loop
void main(void)
{
	uint8_t blink_status = 0;
	int err;

	printk("Starting Sensor Hub application\n");

	//Setting up BH1749 light and color sensor
	const struct device *bh1749rgbDev = DEVICE_DT_GET_ONE(rohm_bh1749);

	if (!device_is_ready(bh1749rgbDev)) 
	{
		printk("Sensor device not ready\n");
		return;
	}

	//Setting up BME688 environmental sensor */
	const struct device *bme688SensorDev = DEVICE_DT_GET_ONE(bosch_bme680);

	if (!device_is_ready(bme688SensorDev)) 
	{
		printk("Sensor device not ready\n");
		return;
	}

	//Setting up ADC
	const struct device *adc_dev = DEVICE_DT_GET_ONE(nordic_nrf_saadc);

	if (!device_is_ready(adc_dev)) 
	{
		printk("ADC is not ready\n");
		return;
	}

    err = adc_channel_setup(adc_dev, &adc_channel_cfg);
    if (err) 
	{
	    printk("Error in ADC setup: %d\n", err);
		return;
	}

	//Setting up voltage divider
	if (!device_is_ready(volt_div_conf.enable_pin.port)) {		
		return;
	}

	gpio_pin_configure_dt(&volt_div_conf.enable_pin, GPIO_OUTPUT);
	gpio_pin_set_dt(&volt_div_conf.enable_pin, 1);

	//Setting up LEDs on Thingy:53
	err = dk_leds_init();
	if (err) 
	{
		printk("LEDs init failed (err %d)\n", err);
		return;
	}

	//Setting up Bluetooth
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
			/*When blink is even number it means the LED has been OFF for 500ms, so we can sample
			the sensors if there is a BLE central connected */
			sample_and_update_all_sensor_values(bme688SensorDev, bh1749rgbDev, adc_dev);
		}

		//Change LED status
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		
		//Put thread to sleep for UPDATE_INTERVAL
		k_sleep(K_MSEC(UPDATE_INTERVAL));
	}
}

//This function samples all the needed data from the sensors and sends it over to the service.c/.h module that handles the GATT data transfer
static int sample_and_update_all_sensor_values(const struct device *bme688Dev, const struct device *bh1749Dev, const struct device *adc_dev)
{
	int err;
	struct sensor_value temperature_value;
	struct sensor_value pressure_value;
	struct sensor_value humidity_value;
	struct sensor_value red_value;
	struct sensor_value green_value;
	struct sensor_value blue_value;
	int16_t batt_volt;
	
	//Trigger sampling of BME688 sensor
	err = sensor_sample_fetch(bme688Dev);
	if(err)
	{
		printk("Failed to collect samples from bme688\n");
		return err;
	}

	//Collect temperature sample and update characteristic
	err = sensor_channel_get(bme688Dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature_value);
	
	if(err)
	{
		printk("Failed to fetch temperature sample");
		return err;
	}
	sensor_hub_update_temperature(m_connection_handle, (uint8_t*)(&temperature_value.val1), sizeof(temperature_value.val1));
	printk("Temperature %d \n", temperature_value.val1);
	//Collect pressure sample and update characteristic
	err = sensor_channel_get(bme688Dev, SENSOR_CHAN_PRESS, &pressure_value);

	if(err)
	{
		printk("Failed to fetch pressure sample");
		return err;
	}
	sensor_hub_update_pressure(m_connection_handle, (uint8_t*)(&pressure_value.val1), sizeof(pressure_value.val1));

	//Collect humidity sample and update characteristic
	err = sensor_channel_get(bme688Dev, SENSOR_CHAN_HUMIDITY, &humidity_value);

	if(err)
	{
		printk("Failed to fetch humidity sample");
		return err;
	}
	sensor_hub_update_humidity(m_connection_handle, (uint8_t*)(&humidity_value.val1), sizeof(humidity_value.val1));

	//Trigger sampling of BH1749 - The sensor does only support fetching SENSOR_CHAN_ALL
	err = sensor_sample_fetch_chan(bh1749Dev, SENSOR_CHAN_ALL);

	if (err) 
	{
		printk("sensor_sample_fetch failed err %d\n", err);
		return err;
	}
	
	//Collect red light sample and update characteristic
	err = sensor_channel_get(bh1749Dev, SENSOR_CHAN_RED, &red_value);
	if (err) 
	{
		printk("sensor_channel_get failed err %d\n", err);
		return err;
	}
	sensor_hub_update_red_color(m_connection_handle, (uint8_t*)(&red_value.val1), sizeof(red_value.val1));

	//Collect green light sample and update characteristic
	err = sensor_channel_get(bh1749Dev, SENSOR_CHAN_GREEN, &green_value);
	if (err) 
	{
		printk("sensor_channel_get failed err %d\n", err);
		return err;
	}
	sensor_hub_update_green_color(m_connection_handle, (uint8_t*)(&green_value.val1), sizeof(green_value.val1));

	//Collect red light sample and update characteristic
	err = sensor_channel_get(bh1749Dev, SENSOR_CHAN_BLUE, &blue_value);
	if (err) 
	{
		printk("sensor_channel_get failed err %d\n", err);
		return err;
	}
	sensor_hub_update_blue_color(m_connection_handle, (uint8_t*)(&blue_value.val1), sizeof(blue_value.val1));
	printk("blue value %d \n", blue_value.val1);

	//Collect ADC battery measurement	
	adc_seq.buffer = &batt_volt;
	adc_seq.buffer_size = sizeof(batt_volt);
	
	err = adc_read(adc_dev, &adc_seq);
	if (err) 
	{
        printk("adc_read() failed with code %d\n", err);
		return err;
	}	

	//Convert raw ADC measurements to mV with Zephyr ADC API
	adc_raw_to_millivolts(adc_ref_internal(adc_dev), ADC_GAIN_1_6, ADC_RESOLUTION, (int32_t*)(&batt_volt));	
    
	//Calculate actual battery voltage using voltage divider
	batt_volt = batt_volt * volt_div_conf.full_ohm / volt_div_conf.output_ohm;	
	printk("Battery Voltage %d \n", batt_volt);
	
	sensor_hub_update_batt_volt(m_connection_handle, (uint8_t*)(&batt_volt), sizeof(batt_volt));

	printk("All sensors sampled and characteristics updated!\n");

	return 0;
}
