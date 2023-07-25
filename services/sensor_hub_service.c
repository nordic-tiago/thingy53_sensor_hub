// sensor_hub_service.c

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/gatt.h>

#include "sensor_hub_service.h"

#define BT_UUID_SENSOR_HUB              BT_UUID_DECLARE_128(SENSOR_HUB_SERVICE_UUID)
#define BT_UUID_SENSOR_HUB_TEMP         BT_UUID_DECLARE_128(TEMP_CHARACTERISTIC_UUID)
#define BT_UUID_SENSOR_HUB_PRESSURE     BT_UUID_DECLARE_128(PRESSURE_CHARACTERISTIC_UUID)
#define BT_UUID_SENSOR_HUB_HUMIDITY     BT_UUID_DECLARE_128(HUMIDITY_CHARACTERISTIC_UUID)
#define BT_UUID_SENSOR_HUB_RED_COLOR    BT_UUID_DECLARE_128(RED_COLOR_CHARACTERISTIC_UUID)
#define BT_UUID_SENSOR_HUB_GREEN_COLOR  BT_UUID_DECLARE_128(GREEN_COLOR_CHARACTERISTIC_UUID)
#define BT_UUID_SENSOR_HUB_BLUE_COLOR   BT_UUID_DECLARE_128(BLUE_COLOR_CHARACTERISTIC_UUID)
#define BT_UUID_SENSOR_HUB_BATT_VOLT    BT_UUID_DECLARE_128(BATT_VOLT_CHARACTERISTIC_UUID)

/*This function is called whenever the Client Characteristic Control Descriptor (CCCD) has been 
changed by the GATT client, for each of the characteristics*/
static void on_cccd_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    switch(value)
    {
        case BT_GATT_CCC_NOTIFY: 
           
            // Start sending stuff!
            break;
        
        case 0: 
            // Stop sending stuff
            break;

        default: 
            printk("Error, CCCD has been set to an invalid value");     
    }
}

//Sensor hub Service Declaration and Registration
BT_GATT_SERVICE_DEFINE(sensor_hub,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_SENSOR_HUB),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_HUB_TEMP,
                    BT_GATT_CHRC_NOTIFY,
                    BT_GATT_PERM_READ,
                    NULL, NULL, NULL),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_HUB_PRESSURE,
                    BT_GATT_CHRC_NOTIFY,
                    BT_GATT_PERM_READ,
                    NULL, NULL, NULL),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_HUB_HUMIDITY,
                    BT_GATT_CHRC_NOTIFY,
                    BT_GATT_PERM_READ,
                    NULL, NULL, NULL),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_HUB_RED_COLOR,
                    BT_GATT_CHRC_NOTIFY,
                    BT_GATT_PERM_READ,
                    NULL, NULL, NULL),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_HUB_GREEN_COLOR,
                    BT_GATT_CHRC_NOTIFY,
                    BT_GATT_PERM_READ,
                    NULL, NULL, NULL),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_HUB_BLUE_COLOR,
                    BT_GATT_CHRC_NOTIFY,
                    BT_GATT_PERM_READ,
                    NULL, NULL, NULL),
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_HUB_BATT_VOLT,
                    BT_GATT_CHRC_NOTIFY,
                    BT_GATT_PERM_READ,
                    NULL, NULL, NULL), 
    BT_GATT_CCC(on_cccd_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* The below functions send a notification to a GATT client with the provided data,
given that the CCCD has been set to Notify (0x1) */
void sensor_hub_update_temperature(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    const struct bt_gatt_attr *attr = &sensor_hub.attrs[2]; 

    struct bt_gatt_notify_params params = 
    {
        .uuid   = BT_UUID_SENSOR_HUB_TEMP,
        .attr   = attr,
        .data   = data,
        .len    = len,
        .func   = NULL
    };

    if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) 
    {
        // Send the notification
	    if(bt_gatt_notify_cb(conn, &params))
        {
            printk("Error, unable to send notification\n");
        }
    }
    else
    {
        printk("Warning, notification not enabled for temperature characteristic\n");
    }
}

void sensor_hub_update_pressure(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    const struct bt_gatt_attr *attr = &sensor_hub.attrs[5]; 

    struct bt_gatt_notify_params params = 
    {
        .uuid   = BT_UUID_SENSOR_HUB_PRESSURE,
        .attr   = attr,
        .data   = data,
        .len    = len,
        .func   = NULL
    };

    if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) 
    {
        // Send the notification
	    if(bt_gatt_notify_cb(conn, &params))
        {
            printk("Error, unable to send notification\n");
        }
    }
    else
    {
        printk("Warning, notification not enabled for pressure characteristic\n");
    }
}

void sensor_hub_update_humidity(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    const struct bt_gatt_attr *attr = &sensor_hub.attrs[8]; 

    struct bt_gatt_notify_params params = 
    {
        .uuid   = BT_UUID_SENSOR_HUB_HUMIDITY,
        .attr   = attr,
        .data   = data,
        .len    = len,
        .func   = NULL
    };

    if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) 
    {
        // Send the notification
	    if(bt_gatt_notify_cb(conn, &params))
        {
            printk("Error, unable to send notification\n");
        }
    }
    else
    {
        printk("Warning, notification not enabled for humidity characteristic\n");
    }
}

void sensor_hub_update_red_color(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    const struct bt_gatt_attr *attr = &sensor_hub.attrs[11]; 

    struct bt_gatt_notify_params params = 
    {
        .uuid   = BT_UUID_SENSOR_HUB_RED_COLOR,
        .attr   = attr,
        .data   = data,
        .len    = len,
        .func   = NULL
    };

    if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) 
    {
        // Send the notification
	    if(bt_gatt_notify_cb(conn, &params))
        {
            printk("Error, unable to send notification\n");
        }
    }
    else
    {
        printk("Warning, notification not enabled for red color characteristic\n");
    }
}

void sensor_hub_update_green_color(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    const struct bt_gatt_attr *attr = &sensor_hub.attrs[14]; 

    struct bt_gatt_notify_params params = 
    {
        .uuid   = BT_UUID_SENSOR_HUB_GREEN_COLOR,
        .attr   = attr,
        .data   = data,
        .len    = len,
        .func   = NULL
    };

    if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) 
    {
        // Send the notification
	    if(bt_gatt_notify_cb(conn, &params))
        {
            printk("Error, unable to send notification\n");
        }
    }
    else
    {
        printk("Warning, notification not enabled for green color characteristic\n");
    }
}

void sensor_hub_update_blue_color(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    const struct bt_gatt_attr *attr = &sensor_hub.attrs[17]; 

    struct bt_gatt_notify_params params = 
    {
        .uuid   = BT_UUID_SENSOR_HUB_BLUE_COLOR,
        .attr   = attr,
        .data   = data,
        .len    = len,
        .func   = NULL
    };

    if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) 
    {
        // Send the notification
	    if(bt_gatt_notify_cb(conn, &params))
        {
            printk("Error, unable to send notification\n");
        }
    }
    else
    {
        printk("Warning, notification not enabled for blue color characteristic\n");
    }
}

void sensor_hub_update_batt_volt(struct bt_conn *conn, const uint8_t *data, uint16_t len)
{
    const struct bt_gatt_attr *attr = &sensor_hub.attrs[20]; 

    struct bt_gatt_notify_params params = 
    {
        //.uuid   = BT_UUID_SENSOR_HUB_ADC_MEAS,
        .attr   = attr,
        .data   = data,
        .len    = len,
        .func   = NULL
    };

    if(bt_gatt_is_subscribed(conn, attr, BT_GATT_CCC_NOTIFY)) 
    {
        // Send the notification
	    if(bt_gatt_notify_cb(conn, &params))
        {
            printk("Error, unable to send notification\n");
        }
    }
    else
    {
        printk("Warning, notification not enabled for battery voltage characteristic\n");
    }
}