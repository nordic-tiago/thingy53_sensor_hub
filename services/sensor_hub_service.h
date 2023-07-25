// sensor_hub_service.h

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

//Declaration of custom GATT service and characteristics UUIDs
#define SENSOR_HUB_SERVICE_UUID \
    BT_UUID_128_ENCODE(0xa5b46352, 0x9d13, 0x479f, 0x9fcb, 0x3dcdf0a13f4d)

#define TEMP_CHARACTERISTIC_UUID \
    BT_UUID_128_ENCODE(0x506a55c4, 0xb5e7, 0x46fa, 0x8326, 0x8acaeb1189eb)

#define PRESSURE_CHARACTERISTIC_UUID \
    BT_UUID_128_ENCODE(0x51838aff, 0x2d9a, 0xb32a, 0xb32a, 0x8187e41664ba)

#define HUMIDITY_CHARACTERISTIC_UUID \
    BT_UUID_128_ENCODE(0x753e3050, 0xdf06, 0x4b53, 0xb090, 0x5e1d810c4383)

#define RED_COLOR_CHARACTERISTIC_UUID \
    BT_UUID_128_ENCODE(0x82754bbb, 0x6ed3, 0x4d69, 0xa0e1, 0xf19f6b654ec2)

#define GREEN_COLOR_CHARACTERISTIC_UUID \
    BT_UUID_128_ENCODE(0xdb7f9f36, 0x92ce, 0x4509, 0xa2ef, 0xaf72ba38fb48)

#define BLUE_COLOR_CHARACTERISTIC_UUID \
    BT_UUID_128_ENCODE(0xf5d2eab5, 0x41e8, 0x4f7c, 0xaef7, 0xc9fff4c544c0)

#define BATT_VOLT_CHARACTERISTIC_UUID \
    BT_UUID_128_ENCODE(0xfa3cf070, 0xd0c7, 0x4668, 0x96c4, 0x86125c8ac5df)

void sensor_hub_update_temperature(struct bt_conn *conn, const uint8_t *data, uint16_t len);
void sensor_hub_update_humidity(struct bt_conn *conn, const uint8_t *data, uint16_t len);
void sensor_hub_update_pressure(struct bt_conn *conn, const uint8_t *data, uint16_t len);
void sensor_hub_update_red_color(struct bt_conn *conn, const uint8_t *data, uint16_t len);
void sensor_hub_update_green_color(struct bt_conn *conn, const uint8_t *data, uint16_t len);
void sensor_hub_update_blue_color(struct bt_conn *conn, const uint8_t *data, uint16_t len);
void sensor_hub_update_batt_volt(struct bt_conn *conn, const uint8_t *data, uint16_t len);