/*
 * Copyright (c) 2022 Mr. Green's Workshop https://www.MrGreensWorkshop.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/init.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>
#include <hardware/gpio.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), pwr_gpios)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
    ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

#define DT_SPEC_AND_COMMA_PWR(node_id, prop, idx) \
    GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx),

static const struct gpio_dt_spec pwr_status[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), pwr_gpios,
                         DT_SPEC_AND_COMMA_PWR)};

static char *pwr_status_names[] = {"STAT2", "STAT1", "#PG", "A9G", "INA219"};

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
                         DT_SPEC_AND_COMMA)};

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
// #define LED1_NODE DT_ALIAS(led1)
#define GSM_NODE DT_ALIAS(gsmpwr)
#define INA_NODE DT_ALIAS(inapwr)
#define SD_NODE DT_ALIAS(sdpwr)

// #define INA_NODE DT_ALIAS(ina)
// #define SD_NODE DT_ALIAS(sd)
// #define LED2_NODE DT_ALIAS(led2)

// #define NPG_NODE DT_ALIAS(npginput)
// #define STAT1_NODE DT_ALIAS(stat1input)
// #define STAT2_NODE DT_ALIAS(stat2input)

// Devicetree
#define CONSOLE_DEVICE DEVICE_DT_GET(DT_CHOSEN(zephyr_console))
#define USB_CDC_ACM_CHECK DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart)

PINCTRL_DT_DEV_CONFIG_DECLARE(DT_NODELABEL(i2c1));

PINCTRL_DT_STATE_PINS_DEFINE(DT_PATH(zephyr_user), ina219_def);
PINCTRL_DT_STATE_PINS_DEFINE(DT_PATH(zephyr_user), pcf_def);

#define I2C_DEV_NODE DT_ALIAS(i2c1)
struct device *i2c1_dev = DEVICE_DT_GET(I2C_DEV_NODE);
struct device *nxp_pcf8523 = DEVICE_DT_GET_ONE(nxp_pcf8523);
struct device *ti_ina219 = DEVICE_DT_GET_ONE(ti_ina219);

static const struct pinctrl_state ina219_state[] = {
    PINCTRL_DT_STATE_INIT(ina219_def, PINCTRL_STATE_DEFAULT),
};

static const struct pinctrl_state pcf_state[] = {
    PINCTRL_DT_STATE_INIT(pcf_def, PINCTRL_STATE_DEFAULT),
};

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
// static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec gsm_pwr = GPIO_DT_SPEC_GET(GSM_NODE, gpios);
static const struct gpio_dt_spec ina_pwr = GPIO_DT_SPEC_GET(INA_NODE, gpios);
static const struct gpio_dt_spec sd_pwr = GPIO_DT_SPEC_GET(SD_NODE, gpios);
// static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
// static const struct gpio_dt_spec npg = GPIO_DT_SPEC_GET(NPG_NODE, gpios);
// static const struct gpio_dt_spec stat1 = GPIO_DT_SPEC_GET(STAT1_NODE, gpios);
// static const struct gpio_dt_spec stat2 = GPIO_DT_SPEC_GET(STAT2_NODE, gpios);

/* delay between greetings (in ms) */
#define SLEEPTIME 500

struct patch_info
{
    const uint8_t *const name;

    const struct device *rx_dev;
    struct ring_buf *rx_ring_buf;
    bool rx_error;
    bool rx_overflow;

    const struct device *tx_dev;
};

#define DEV_CONSOLE DEVICE_DT_GET(DT_CHOSEN(zephyr_console))
#define DEV_OTHER DEVICE_DT_GET(DT_CHOSEN(uart_passthrough))

#define RING_BUF_SIZE 128

RING_BUF_DECLARE(rb_console, RING_BUF_SIZE);
struct patch_info patch_c2o = {
    .name = "c2o",

    .rx_dev = DEV_CONSOLE,
    .rx_ring_buf = &rb_console,
    .rx_error = true,
    .rx_overflow = true,

    .tx_dev = DEV_OTHER,
};

RING_BUF_DECLARE(rb_other, RING_BUF_SIZE);
struct patch_info patch_o2c = {
    .name = "o2c",

    .rx_dev = DEV_OTHER,
    .rx_ring_buf = &rb_other,
    .rx_error = true,
    .rx_overflow = true,

    .tx_dev = DEV_CONSOLE,
};

static void uart_cb(const struct device *dev, void *ctx)
{
    struct patch_info *patch = (struct patch_info *)ctx;
    int ret;
    uint8_t *buf;
    uint32_t len;

    while (uart_irq_update(patch->rx_dev) > 0)
    {
        ret = uart_irq_rx_ready(patch->rx_dev);
        if (ret < 0)
        {
            patch->rx_error = true;
        }
        if (ret <= 0)
        {
            break;
        }

        len = ring_buf_put_claim(patch->rx_ring_buf, &buf, RING_BUF_SIZE);
        if (len == 0)
        {
            /* no space for Rx, disable the IRQ */
            uart_irq_rx_disable(patch->rx_dev);
            patch->rx_overflow = true;
            break;
        }

        ret = uart_fifo_read(patch->rx_dev, buf, len);
        if (ret < 0)
        {
            patch->rx_error = true;
        }
        if (ret <= 0)
        {
            break;
        }
        len = ret;

        ret = ring_buf_put_finish(patch->rx_ring_buf, len);
        if (ret != 0)
        {
            patch->rx_error = true;
            break;
        }
    }
}

static void passthrough(struct patch_info *patch)
{
    int ret;
    uint8_t *buf;
    uint32_t len;

    if (patch->rx_error)
    {
        printk("<<%s: Rx Error!>>\n", patch->name);
        patch->rx_error = false;
    }

    if (patch->rx_overflow)
    {
        printk("<<%s: Rx Overflow!>>\n", patch->name);
        patch->rx_overflow = false;
    }

    len = ring_buf_get_claim(patch->rx_ring_buf, &buf, RING_BUF_SIZE);
    if (len == 0)
    {
        goto done;
    }

    ret = uart_fifo_fill(patch->tx_dev, buf, len);
    if (ret < 0)
    {
        goto error;
    }
    len = ret;

    ret = ring_buf_get_finish(patch->rx_ring_buf, len);
    if (ret < 0)
    {
        goto error;
    }

done:
    uart_irq_rx_enable(patch->rx_dev);
    return;

error:
    printk("<<%s: Tx Error!>>\n", patch->name);
}

int compare(const void *a, const void *b)
{
    uint16_t int_a = *((uint16_t *)a);
    uint16_t int_b = *((uint16_t *)b);

    if (int_a == int_b)
        return 0;
    else if (int_a < int_b)
        return -1;
    else
        return 1;
}

#define STACKSIZE 1024
#define PRIORITY 7

// extern struct k_thread thread_a_data;
// extern void thread_a_entry_point(void *dummy1, void *dummy2, void *dummy3);
// extern K_THREAD_STACK_DEFINE(thread_a_stack_area, STACKSIZE);
// extern const k_tid_t thread_b;

void pinctrl_free_pins(const struct pinctrl_dev_config *config, uint8_t id)
{
    struct pinctrl_state *state;
    int err = pinctrl_lookup_state(config, id, &state);
    if (err < 0)
    {
        LOG_ERR("Could not pinctrl_lookup_state");
    }

    LOG_INF("Free pin %u", state->pins->pin_num);
    gpio_set_input_enabled(state->pins->pin_num, false);
}

int main(void)
{
    // #if (IS_ENABLED(USB_CDC_ACM_CHECK) && IS_ENABLED(CONFIG_LOG))
    //     const struct device *const console_dev = CONSOLE_DEVICE;
    //     uint32_t dtr_line = 0;

    //     while (!dtr_line)
    //     {
    //         uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr_line);
    //         k_sleep(K_MSEC(100));
    //     }
    // #endif

    int ret;
    bool led_state = true;
    k_sleep(K_MSEC(1000));

    if (!gpio_is_ready_dt(&gsm_pwr))
    {
        return 0;
    }

    ret = gpio_pin_configure_dt(&gsm_pwr, GPIO_OUTPUT_INACTIVE);
    if (ret < 0)
    {
        return 0;
    }

    if (!gpio_is_ready_dt(&ina_pwr))
    {
        return 0;
    }

    ret = gpio_pin_configure_dt(&ina_pwr, GPIO_OUTPUT_INACTIVE);
    if (ret < 0)
    {
        return 0;
    }

    gpio_pin_set_dt(&gsm_pwr, 0);
    gpio_pin_set_dt(&ina_pwr, 1);

    if (!gpio_is_ready_dt(&led))
    {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0)
    {
        return 0;
    }

    if (!gpio_is_ready_dt(&led))
    {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0)
    {
        return 0;
    }

    LOG_INF("PicoDRO application.");

    // k_thread_create(&thread_a_data, thread_a_stack_area,
    //                 K_THREAD_STACK_SIZEOF(thread_a_stack_area), thread_a_entry_point, NULL,
    //                 NULL, NULL, PRIORITY, 0, K_FOREVER);
    // k_thread_name_set(&thread_a_data, "thread_a");

    // k_thread_start(&thread_a_data);

    int err;
    uint32_t count = 0;
    uint16_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        /* buffer size in bytes, not number of samples */
        .buffer_size = sizeof(buf),
        .resolution = 12,
        .calibrate = true,
        .oversampling = 256,
    };

    /* Configure channels individually prior to sampling. */
    for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++)
    {
        if (!adc_is_ready_dt(&adc_channels[i]))
        {
            LOG_ERR("ADC controller device %s not ready", adc_channels[i].dev->name);
            return 0;
        }

        err = adc_channel_setup_dt(&adc_channels[i]);
        if (err < 0)
        {
            LOG_ERR("Could not setup channel #%d (%d)", i, err);
            return 0;
        }
    }

    /* Configure channels individually prior to sampling. */
    for (size_t i = 0U; i < ARRAY_SIZE(pwr_status); i++)
    {
        if (gpio_pin_configure_dt(&pwr_status[i], GPIO_INPUT) < 0)
        {
            LOG_ERR("Could not setup GPIO #%s", pwr_status[i].port->name);
            return 0;
        }
    }

    uint16_t adc[100];
    uint32_t last_v = 0;
    uint32_t error_value = 0;

    // printk("Console Device: %p\n", patch_c2o.rx_dev);
    // printk("Other Device:   %p\n", patch_o2c.rx_dev);

    // uart_irq_callback_user_data_set(patch_c2o.rx_dev, uart_cb, (void *)&patch_c2o);
    // uart_irq_callback_user_data_set(patch_o2c.rx_dev, uart_cb, (void *)&patch_o2c);

    struct pinctrl_dev_config *i2c1_config = PINCTRL_DT_DEV_CONFIG_GET(DT_NODELABEL(i2c1));

    pinctrl_free_pins(i2c1_config, PINCTRL_STATE_DEFAULT);
    k_sleep(K_MSEC(100));

    err = pinctrl_update_states(i2c1_config, pcf_state, ARRAY_SIZE(pcf_state));
    if (err < 0)
    {
        LOG_ERR("Could not pinctrl_update_states I2C1 pcf_state");
    }

    err = pinctrl_apply_state(i2c1_config, PINCTRL_STATE_DEFAULT);
    if (err < 0)
    {
        LOG_ERR("Could not pinctrl_apply_state I2C1");
    }

    k_sleep(K_MSEC(100));

    err = device_init(nxp_pcf8523);
    if (err < 0)
    {
        LOG_ERR("Could not device_init nxp_pcf8523");
    }
    k_sleep(K_MSEC(100));

    if (!device_is_ready(nxp_pcf8523))
    {
        LOG_ERR("RTC not ready");
    }
    else
    {
        LOG_INF("RTC ready");
    }

    k_sleep(K_MSEC(100));

    pinctrl_free_pins(i2c1_config, PINCTRL_STATE_DEFAULT);
    k_sleep(K_MSEC(100));

    err = pinctrl_update_states(i2c1_config, ina219_state, ARRAY_SIZE(ina219_state));
    if (err < 0)
    {
        LOG_ERR("Could not pinctrl_update_states I2C1 ina219_state");
    }

    err = pinctrl_apply_state(i2c1_config, PINCTRL_STATE_DEFAULT);
    if (err < 0)
    {
        LOG_ERR("Could not pinctrl_apply_state I2C1");
    }

    k_sleep(K_MSEC(100));

    err = device_init(ti_ina219);
    if (err < 0)
    {
        LOG_ERR("Could not device_init ti_ina219");
    }
    k_sleep(K_MSEC(100));

    if (!device_is_ready(ti_ina219))
    {
        LOG_ERR("INA219 not ready");
    }
    else
    {
        LOG_INF("INA219 ready");
    }

    struct sensor_value v_bus, power, current;

    while (1)
    {
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0)
        {
            return 0;
        }

        led_state = !led_state;
        printf("LED state: %s\n", led_state ? "ON" : "OFF");
        ret = sensor_sample_fetch(ti_ina219);
        if (ret)
        {
            printf("Could not fetch sensor data.\n");
            // return 0;
        }

        sensor_channel_get(ti_ina219, SENSOR_CHAN_VOLTAGE, &v_bus);
        sensor_channel_get(ti_ina219, SENSOR_CHAN_POWER, &power);
        sensor_channel_get(ti_ina219, SENSOR_CHAN_CURRENT, &current);

        printf("Bus: %f [V] -- "
               "Power: %f [W] -- "
               "Current: %f [A]\n",
               sensor_value_to_double(&v_bus),
               sensor_value_to_double(&power),
               sensor_value_to_double(&current));

        /*
        // printk("ADC reading[%u]: ", count++);
        for (size_t n = 0U; n < 100; n++)
        {
            int32_t val_mv;

            (void)adc_sequence_init_dt(&adc_channels[0], &sequence);

            err = adc_read_dt(&adc_channels[0], &sequence);
            if (err < 0)
            {
                printk("Could not read (%d)\n", err);
                continue;
            }

            adc[n] = buf;
        }

        qsort(adc, 100, sizeof(uint16_t), compare);
        uint32_t v = 0;
        for (size_t n = 90; n < 100; n++)
        {
            v = v + adc[n];
        }

        uint32_t value = v * adc_ref_internal(adc_channels[0].dev) / 4096;
        value *= 6900;
        value /= 47;

        if (value >= last_v)
        {
            error_value = value - last_v;
            error_value /= (value / 100);
        }
        else
        {
            error_value = last_v - value;
            error_value /= (value / 100);
        }
        last_v = value;
        // printk("ADC reading: %u mV, error %u %%\n", value / 1000, error_value);

        // printf("GPIO state:\n");

        uint8_t pwr_pins[5];

        for (size_t i = 0U; i < ARRAY_SIZE(pwr_status); i++)
        {
            pwr_pins[i] = gpio_pin_get(pwr_status[i].port, pwr_status[i].pin);
            // printk("\t %s: %u, value: %u\n", pwr_status_names[i], pwr_status[i].pin, pwr_pins[i]);
        }

        uint32_t pwr_status = pwr_pins[2] << 16 + pwr_pins[1] << 8 + pwr_pins[0];

        // if (!pwr_pins[2])
        //     printf("PWR status: power good\n");

        // if (pwr_pins[0] && !pwr_pins[1] && !pwr_pins[2])
        //     printf("PWR status: charge\n");

        // if (!pwr_pins[0] && pwr_pins[1] && !pwr_pins[2])
        //     printf("PWR status: complete\n");

        // if (pwr_pins[0] && pwr_pins[1] && !pwr_pins[2])
        //     printf("PWR status: thermal limit\n");

        // if (pwr_pins[0] && pwr_pins[1] && pwr_pins[2])
        //     printf("PWR status: shutdown\n");

        // if (!pwr_pins[0] && !pwr_pins[1] && !pwr_pins[2])
        //     printf("PWR status: test\n");

        gpio_pin_set_dt(&led, pwr_pins[0] && !pwr_pins[1]);
*/
        // passthrough(&patch_c2o);
        // passthrough(&patch_o2c);

        k_sleep(K_MSEC(2000));
    }

    return 0;
}

// static int init_my_devices(void)
// {

//     if (!gpio_is_ready_dt(&ina_pwr))
//     {
//         return 0;
//     }

//     // if (!gpio_pin_configure_dt(&ina_pwr, GPIO_OUTPUT_INACTIVE))
//     // {
//     //     return 0;
//     // }

//     gpio_pin_set_dt(&ina_pwr, 1);

//     return device_init(ti_ina219);
// }

// SYS_INIT(init_my_devices, POST_KERNEL, 99);