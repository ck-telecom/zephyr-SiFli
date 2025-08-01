/* h4.c - H:4 UART based Bluetooth driver */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/bluetooth.h>

#define LOG_LEVEL CONFIG_BT_HCI_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_driver);

#include "common/bt_str.h"

static int uart_fifo_read(const struct device *dev, uint8_t *rx_data,    const int size);
static int uart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size);
#define uart_irq_rx_disable(uart)
#define uart_irq_tx_disable(uart)

#define DT_DRV_COMPAT sifli_bt_mbox

#include "ipc_queue.h"

struct h4_data
{
    struct
    {
        struct net_buf *buf;
        struct k_fifo   fifo;

        uint16_t        remaining;
        uint16_t        discard;

        bool            have_hdr;
        bool            discardable;

        uint8_t         hdr_len;

        uint8_t         type;
        union
        {
            struct bt_hci_evt_hdr evt;
            struct bt_hci_acl_hdr acl;
            struct bt_hci_iso_hdr iso;
            struct bt_hci_sco_hdr sco;
            uint8_t hdr[4];
        };
    } rx;

    struct
    {
        uint8_t         type;
        struct net_buf *buf;
        struct k_fifo   fifo;
    } tx;

    struct k_sem sem;
    bt_hci_recv_t recv;
};

struct h4_config
{
    const struct device *uart;
    k_thread_stack_t *rx_thread_stack;
    size_t rx_thread_stack_size;
    struct k_thread *rx_thread;
};

int zbt_config_mailbox(void);


#define GLOBAL_INT_DISABLE();                                               \
do {                                                                        \
    uint32_t __old;                                                         \
    __old = irq_lock();                                  \

/** @brief Restore interrupts from the previous global disable.
 * @sa GLOBAL_INT_DISABLE
 */
#define GLOBAL_INT_RESTORE();                                               \
    irq_unlock(__old);                                      \
} while(0)

struct uart_txrxchannel
{
    /// call back function pointer
    void (*callback)(void *, uint8_t);
    /// Dummy data pointer returned to callback when operation is over.
    void *dummy;
    uint8_t *buf;
    uint32_t data_size;
    uint32_t offset;
};

struct mbox_env_tag
{
    ipc_queue_handle_t ipc_port;
    // Adapt for uart
    struct uart_txrxchannel tx;
    struct uart_txrxchannel rx;
    uint8_t is_init;
};
struct mbox_env_tag mbox_env;
static inline void process_tx(const struct device *dev);
static inline void process_rx(const struct device *dev);
static void log_hci_to_console(struct net_buf *buf);

int ble_stack_filter(struct net_buf *buf)
{
    return 1;
}


uint8_t bt_buf_type2h4(enum bt_buf_type type)
{
    uint8_t r;
    switch (type)
    {
    case BT_BUF_CMD:
        r = BT_HCI_H4_CMD;
        break;
    case BT_BUF_EVT:
        r = BT_HCI_H4_EVT;
        break;
    case BT_BUF_ACL_IN:
    case BT_BUF_ACL_OUT:
        r = BT_HCI_H4_ACL;
        break;
    case BT_BUF_ISO_IN:
    case BT_BUF_ISO_OUT:
        r = BT_HCI_H4_ISO;
        break;
    default:
        r = BT_HCI_H4_NONE;
    }
    return r;
}

static inline void h4_get_type(const struct device *dev)
{
    const struct h4_config *cfg = dev->config;
    struct h4_data *h4 = dev->data;

    /* Get packet type */
    if (uart_fifo_read(cfg->uart, &h4->rx.type, 1) != 1)
    {
        LOG_WRN("Unable to read H:4 packet type");
        h4->rx.type = BT_HCI_H4_NONE;
        return;
    }

    switch (h4->rx.type)
    {
    case BT_HCI_H4_EVT:
        h4->rx.remaining = sizeof(h4->rx.evt);
        h4->rx.hdr_len = h4->rx.remaining;
        break;
    case BT_HCI_H4_ACL:
        h4->rx.remaining = sizeof(h4->rx.acl);
        h4->rx.hdr_len = h4->rx.remaining;
        break;
    case BT_HCI_H4_ISO:
        if (IS_ENABLED(CONFIG_BT_ISO))
        {
            h4->rx.remaining = sizeof(h4->rx.iso);
            h4->rx.hdr_len = h4->rx.remaining;
            break;
        }
        __fallthrough;
    default:
        LOG_ERR("Unknown H:4 type 0x%02x", h4->rx.type);
        h4->rx.type = BT_HCI_H4_NONE;
    }
}

static void h4_read_hdr(const struct device *dev)
{
    const struct h4_config *cfg = dev->config;
    struct h4_data *h4 = dev->data;
    int bytes_read = h4->rx.hdr_len - h4->rx.remaining;
    int ret;

    ret = uart_fifo_read(cfg->uart, h4->rx.hdr + bytes_read, h4->rx.remaining);
    if (unlikely(ret < 0))
    {
        LOG_ERR("Unable to read from UART (ret %d)", ret);
    }
    else
    {
        h4->rx.remaining -= ret;
    }
}

static inline void get_acl_hdr(const struct device *dev)
{
    struct h4_data *h4 = dev->data;

    h4_read_hdr(dev);

    if (!h4->rx.remaining)
    {
        struct bt_hci_acl_hdr *hdr = &h4->rx.acl;

        h4->rx.remaining = sys_le16_to_cpu(hdr->len);
        LOG_DBG("Got ACL header. Payload %u bytes", h4->rx.remaining);
        h4->rx.have_hdr = true;
    }
}

static inline void get_sco_hdr(const struct device *dev)
{
    struct h4_data *h4 = dev->data;

    h4_read_hdr(dev);

    if (!h4->rx.remaining)
    {
        struct bt_hci_sco_hdr *hdr = &h4->rx.sco;

        h4->rx.remaining = hdr->len;
        LOG_DBG("Got SCO header. Payload %u bytes", h4->rx.remaining);
        h4->rx.have_hdr = true;
    }
}


static inline void get_iso_hdr(const struct device *dev)
{
    struct h4_data *h4 = dev->data;

    h4_read_hdr(dev);

    if (!h4->rx.remaining)
    {
        struct bt_hci_iso_hdr *hdr = &h4->rx.iso;

        h4->rx.remaining = bt_iso_hdr_len(sys_le16_to_cpu(hdr->len));
        LOG_DBG("Got ISO header. Payload %u bytes", h4->rx.remaining);

        h4->rx.have_hdr = true;
    }
}

static inline void get_evt_hdr(const struct device *dev)
{
    struct h4_data *h4 = dev->data;

    struct bt_hci_evt_hdr *hdr = &h4->rx.evt;

    h4_read_hdr(dev);

    if (h4->rx.hdr_len == sizeof(*hdr) && h4->rx.remaining < sizeof(*hdr))
    {
        switch (h4->rx.evt.evt)
        {
        case BT_HCI_EVT_LE_META_EVENT:
            h4->rx.remaining++;
            h4->rx.hdr_len++;
            break;
#if defined(CONFIG_BT_CLASSIC)
        case BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI:
        case BT_HCI_EVT_EXTENDED_INQUIRY_RESULT:
            h4->rx.discardable = true;
            break;
#endif
        }
    }

    if (!h4->rx.remaining)
    {
        if (h4->rx.evt.evt == BT_HCI_EVT_LE_META_EVENT &&
                (h4->rx.hdr[sizeof(*hdr)] == BT_HCI_EVT_LE_ADVERTISING_REPORT))
        {
            LOG_DBG("Marking adv report as discardable");
            h4->rx.discardable = true;
        }

        h4->rx.remaining = hdr->len - (h4->rx.hdr_len - sizeof(*hdr));
        LOG_DBG("Got event header. Payload %u bytes", hdr->len);
        h4->rx.have_hdr = true;
    }
}


static inline void copy_hdr(struct h4_data *h4)
{
    net_buf_add_mem(h4->rx.buf, h4->rx.hdr, h4->rx.hdr_len);
}

static void reset_rx(struct h4_data *h4)
{
    h4->rx.type = BT_HCI_H4_NONE;
    h4->rx.remaining = 0U;
    h4->rx.have_hdr = false;
    h4->rx.hdr_len = 0U;
    h4->rx.discardable = false;
}

static struct net_buf *get_rx(struct h4_data *h4, k_timeout_t timeout)
{
    LOG_DBG("type 0x%02x, evt 0x%02x", h4->rx.type, h4->rx.evt.evt);

    switch (h4->rx.type)
    {
    case BT_HCI_H4_EVT:
        return bt_buf_get_evt(h4->rx.evt.evt, h4->rx.discardable, timeout);
    case BT_HCI_H4_ACL:
        return bt_buf_get_rx(BT_BUF_ACL_IN, timeout);
#ifdef BT_MAX_SCO_CONN        
    case BT_HCI_H4_SCO:
        return bt_buf_get_rx(BT_BUF_SCO_IN, timeout);
#endif        
    case BT_HCI_H4_ISO:
        if (IS_ENABLED(CONFIG_BT_ISO))
        {
            return bt_buf_get_rx(BT_BUF_ISO_IN, timeout);
        }
    }

    return NULL;
}

static void rx_thread(void *p1, void *p2, void *p3)
{
    const struct device *dev = p1;
    struct h4_data *data = dev->data;

    while (1)
    {
        int len;
        k_sem_take(&(data->sem), K_FOREVER);
        len=ipc_queue_get_rx_size(mbox_env.ipc_port);
        while (len)
        {
            LOG_DBG("rx_thread len %d", len);
            struct h4_data *h4 = dev->data;
            struct net_buf *buf;
            process_rx(dev);
            buf = k_fifo_get(&h4->rx.fifo, K_NO_WAIT);
            while (buf)
            {
                LOG_DBG("Calling bt_recv(%p),len=%d,data=%p", (void*)buf, buf->len, (void*)buf->data);

                if (buf->len && buf->data)
                    log_hci_to_console(buf);
                else
                    break;

                if (ble_stack_filter(buf))
                {
                    if (h4->recv)
                        h4->recv(dev, buf);
                    else
                        net_buf_unref(buf);
                }
                buf = k_fifo_get(&h4->rx.fifo, K_NO_WAIT);
            };
            len=ipc_queue_get_rx_size(mbox_env.ipc_port);
        }
        process_tx(dev);
    }
}

static size_t h4_discard(const struct device *uart, size_t len)
{
    uint8_t buf[33];
    int err;

    err = uart_fifo_read(uart, buf, MIN(len, sizeof(buf)));
    if (unlikely(err < 0))
    {
        LOG_ERR("Unable to read from UART (err %d)", err);
        return 0;
    }

    return err;
}

static inline void read_payload(const struct device *dev)
{
    const struct h4_config *cfg = dev->config;
    struct h4_data *h4 = dev->data;
    struct net_buf *buf;
    int read;

    if (!h4->rx.buf)
    {
        size_t buf_tailroom;

        h4->rx.buf = get_rx(h4, K_NO_WAIT);
        if (!h4->rx.buf)
        {
            if (h4->rx.discardable)
            {
                LOG_WRN("Discarding event 0x%02x", h4->rx.evt.evt);
                h4->rx.discard = h4->rx.remaining;
                reset_rx(h4);
                return;
            }

            LOG_WRN("Failed to allocate, deferring to rx_thread");
            uart_irq_rx_disable(cfg->uart);
            return;
        }

        LOG_DBG("Allocated rx.buf %p", (void*)h4->rx.buf);

        buf_tailroom = net_buf_tailroom(h4->rx.buf);
        if (buf_tailroom < h4->rx.remaining)
        {
            LOG_ERR("Not enough space in buffer %u/%zu", h4->rx.remaining,
                    buf_tailroom);
            h4->rx.discard = h4->rx.remaining;
            reset_rx(h4);
            return;
        }

        copy_hdr(h4);
    }

    read = uart_fifo_read(cfg->uart, net_buf_tail(h4->rx.buf), h4->rx.remaining);
    if (unlikely(read < 0))
    {
        LOG_ERR("Failed to read UART (err %d)", read);
        return;
    }

    net_buf_add(h4->rx.buf, read);
    h4->rx.remaining -= read;

    LOG_DBG("got %d bytes, remaining %u", read, h4->rx.remaining);
    LOG_DBG("Payload (len %u): %s", h4->rx.buf->len,
            bt_hex(h4->rx.buf->data, h4->rx.buf->len));

    if (h4->rx.remaining)
    {
        return;
    }

    buf = h4->rx.buf;
    h4->rx.buf = NULL;

    if (h4->rx.type == BT_HCI_H4_EVT)
    {
        bt_buf_set_type(buf, BT_BUF_EVT);
    }
    else if (h4->rx.type == BT_HCI_H4_ISO)
    {
        bt_buf_set_type(buf, BT_BUF_ISO_IN);
    }
#ifdef BT_MAX_SCO_CONN    
    else if (h4->rx.type == BT_HCI_H4_SCO)
    {
        bt_buf_set_type(buf, BT_BUF_SCO_IN);
    }
#endif    
    else
    {
        bt_buf_set_type(buf, BT_BUF_ACL_IN);
    }

    reset_rx(h4);

    LOG_DBG("Putting buf %p to rx fifo", (void*)buf);
    k_fifo_put(&h4->rx.fifo, buf);
}

static inline void read_header(const struct device *dev)
{
    struct h4_data *h4 = dev->data;

    switch (h4->rx.type)
    {
    case BT_HCI_H4_NONE:
        h4_get_type(dev);
        return;
    case BT_HCI_H4_EVT:
        get_evt_hdr(dev);
        break;
    case BT_HCI_H4_ACL:
        get_acl_hdr(dev);
        break;
    case BT_HCI_H4_SCO:
        get_sco_hdr(dev);
        break;
    case BT_HCI_H4_ISO:
        if (IS_ENABLED(CONFIG_BT_ISO))
        {
            get_iso_hdr(dev);
            break;
        }
        else
        {
            LOG_ERR("ISO got unexpected\n");
        }
        __fallthrough;
    default:
        CODE_UNREACHABLE;
        return;
    }

    if (h4->rx.have_hdr && h4->rx.buf)
    {
        if (h4->rx.remaining > net_buf_tailroom(h4->rx.buf))
        {
            LOG_ERR("Not enough space in buffer");
            h4->rx.discard = h4->rx.remaining;
            reset_rx(h4);
        }
        else
        {
            copy_hdr(h4);
        }
    }
}


#define H4TL_PACKET_HOST    0x61
#define H4TL_PACKET_CTRL    0x62

static void log_hci_to_console(struct net_buf *buf)
{
    static uint8_t temp[1024], *trace ;
    static uint16_t seq;
    trace = &temp[0];
    *trace++ = 0x06;
    *trace++ = 0x01;
    *trace++ = (buf->len + 8) & 0xff;
    *trace++ = (buf->len + 8) >> 8;
    *trace++ = seq & 0xff;
    *trace++ = seq >> 8;
    *trace++ = 0;
    *trace++ = 0;
    *trace++ = 0;
    *trace++ = 0;
    switch (bt_buf_get_type(buf))
    {
    case BT_BUF_CMD:
    case BT_BUF_ACL_OUT:
    case BT_BUF_ISO_OUT:
        *trace++ = H4TL_PACKET_HOST;
        break;
    case BT_BUF_ISO_IN:
    case BT_BUF_EVT:
    case BT_BUF_ACL_IN:
    default:
        *trace++ = H4TL_PACKET_CTRL;
    }
    *trace++ = bt_buf_type2h4(bt_buf_get_type(buf));
    seq++;
    memcpy(trace, buf->data, buf->len);
    //LOG_BIN_MIX(temp, buf->len + 12);
}


static inline void process_tx(const struct device *dev)
{
    const struct h4_config *cfg = dev->config;
    struct h4_data *h4 = dev->data;
    int bytes;

    if (!h4->tx.buf)
    {
        h4->tx.buf = k_fifo_get(&h4->tx.fifo, K_NO_WAIT);
        if (!h4->tx.buf)
        {
            //LOG_ERR("TX interrupt but no pending buffer!");
            uart_irq_tx_disable(cfg->uart);
            return;
        }
    }
    else
    {
        LOG_WRN("Other tx is running");
        return;
    }

    while (h4->tx.buf)
    {
        LOG_DBG("process_tx %p", (void*)h4->tx.buf);
        if (!h4->tx.type)
        {
            switch (bt_buf_get_type(h4->tx.buf))
            {
            case BT_BUF_ACL_OUT:
                h4->tx.type = BT_HCI_H4_ACL;
                break;
            case BT_BUF_CMD:
                h4->tx.type = BT_HCI_H4_CMD;
                break;
            case BT_BUF_ISO_OUT:
                if (IS_ENABLED(CONFIG_BT_ISO))
                {
                    h4->tx.type = BT_HCI_H4_ISO;
                    break;
                }
                __fallthrough;
            default:
                LOG_ERR("Unknown buffer type");
                goto done;
            }

            LOG_DBG("process_tx type %x", h4->tx.type);
            bytes = uart_fifo_fill(cfg->uart, &h4->tx.type, 1);
            if (bytes != 1)
            {
                LOG_WRN("Unable to send H:4 type");
                h4->tx.type = BT_HCI_H4_NONE;
                return;
            }
        }

        LOG_DBG("process_tx data %p, len %d", (void*)h4->tx.buf->data, h4->tx.buf->len);
        log_hci_to_console(h4->tx.buf);
        // Zephyr BLE stack command complete event will add data to original tx command tx buffer, might break the following step.
        // Add critical to avoid schedule to rx processing.
        uint32_t level=irq_lock();
        bytes = uart_fifo_fill(cfg->uart, h4->tx.buf->data, h4->tx.buf->len);
        if (unlikely(bytes < 0))
        {
            LOG_ERR("Unable to write to UART (err %d)", bytes);
        }
        else
        {
            LOG_DBG("process_tx bytes %d", bytes);
            net_buf_pull(h4->tx.buf, bytes);
        }
        if (h4->tx.buf->len)
        {
            irq_unlock(level);
            return;
        }
        irq_unlock(level);
done:
        h4->tx.type = BT_HCI_H4_NONE;
        LOG_DBG("process_tx net_buf_unref");
        net_buf_unref(h4->tx.buf);
        h4->tx.buf = k_fifo_get(&h4->tx.fifo, K_NO_WAIT);
        if (!h4->tx.buf)
        {
            uart_irq_tx_disable(cfg->uart);
        }
    }
}

static inline void process_rx(const struct device *dev)
{
    const struct h4_config *cfg = dev->config;
    struct h4_data *h4 = dev->data;

    LOG_DBG("remaining %u discard %u have_hdr %u rx.buf %p len %u",
            h4->rx.remaining, h4->rx.discard, h4->rx.have_hdr, (void*)h4->rx.buf,
            h4->rx.buf ? h4->rx.buf->len : 0);

    if (h4->rx.discard)
    {
        h4->rx.discard -= h4_discard(cfg->uart, h4->rx.discard);
        return;
    }

    if (h4->rx.have_hdr)
    {
        read_payload(dev);
    }
    else
    {
        read_header(dev);
    }
}

static int h4_send(const struct device *dev, struct net_buf *buf)
{
    struct h4_data *h4 = dev->data;

    LOG_DBG("buf %p type %u len %u", (void*)buf, bt_buf_get_type(buf), buf->len);

    k_fifo_put(&h4->tx.fifo, buf);
    k_sem_give(&(h4->sem));

    return 0;
}

/** Setup the HCI transport, which usually means to reset the Bluetooth IC
  *
  * @param dev The device structure for the bus connecting to the IC
  *
  * @return 0 on success, negative error value on failure
  */
int __weak bt_hci_transport_setup(const struct device *uart)
{
    h4_discard(uart, 32);
    return 0;
}

extern uint8_t lcpu_power_on(void);
static int h4_open(const struct device *dev, bt_hci_recv_t recv)
{
    struct h4_data *h4 = dev->data;
    const struct h4_config *cfg = dev->config;
    k_tid_t tid;

    zbt_config_mailbox();
    LOG_DBG("h4 open %p", (void*)recv);
    tid = k_thread_create(cfg->rx_thread, cfg->rx_thread_stack,
                          cfg->rx_thread_stack_size,
                          rx_thread, (void *)dev, NULL, NULL,
                          K_PRIO_COOP(CONFIG_BT_RX_PRIO),
                          0, K_NO_WAIT);
    k_thread_name_set(tid, "hci_rx_th");
    k_thread_start(tid);
    lcpu_power_on();
    k_sleep(K_MSEC(50));
    h4->recv = recv;
    return 0;
}

#if defined(CONFIG_BT_HCI_SETUP)
static int h4_setup(const struct device *dev, const struct bt_hci_setup_params *params)
{
    const struct h4_config *cfg = dev->config;

    ARG_UNUSED(params);

    /* Extern bt_h4_vnd_setup function.
     * This function executes vendor-specific commands sequence to
     * initialize BT Controller before BT Host executes Reset sequence.
     * bt_h4_vnd_setup function must be implemented in vendor-specific HCI
     * extansion module if CONFIG_BT_HCI_SETUP is enabled.
     */
    extern int bt_h4_vnd_setup(const struct device * dev);

    return bt_h4_vnd_setup(cfg->uart);
}
#endif

static const DEVICE_API(bt_hci, h4_driver_api) =
{
    .open = h4_open,
    .send = h4_send,
#if defined(CONFIG_BT_HCI_SETUP)
    .setup = h4_setup,
#endif
};

#define CONFIG_SF_BT_DRV_RX_STACK_SIZE 1024

#define BT_MBOX_DEVICE_INIT(inst) \
    static K_KERNEL_STACK_DEFINE(rx_thread_stack_##inst, CONFIG_SF_BT_DRV_RX_STACK_SIZE); \
    static struct k_thread rx_thread_##inst; \
    static const struct h4_config h4_config_##inst = { \
        .rx_thread_stack = rx_thread_stack_##inst, \
        .rx_thread_stack_size = K_KERNEL_STACK_SIZEOF(rx_thread_stack_##inst), \
        .rx_thread = &rx_thread_##inst, \
    }; \
    static struct h4_data h4_data_##inst= { \
		.rx = { \
			.fifo = Z_FIFO_INITIALIZER(h4_data_##inst.rx.fifo), \
		}, \
		.tx = { \
			.fifo = Z_FIFO_INITIALIZER(h4_data_##inst.tx.fifo), \
		}, \
    }; \
    DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &h4_data_##inst, &h4_config_##inst, \
              POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &h4_driver_api)

BT_MBOX_DEVICE_INIT(0)

#define IO_MB_CH      (0)
#define TX_BUF_SIZE   HCPU2LCPU_MB_CH1_BUF_SIZE
#define TX_BUF_ADDR   HCPU2LCPU_MB_CH1_BUF_START_ADDR
#define TX_BUF_ADDR_ALIAS HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR);
#define RX_BUF_ADDR   LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR);
#define RX_BUF_REV_B_ADDR   LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_REV_B_START_ADDR);


/**
 * @brief Read data from FIFO.
 *
 * @details This function is expected to be called from UART
 * interrupt handler (ISR), if uart_irq_rx_ready() returns true.
 * Result of calling this function not from an ISR is undefined
 * (hardware-dependent). It's unspecified whether "RX ready"
 * condition as returned by uart_irq_rx_ready() is level- or
 * edge- triggered. That means that once uart_irq_rx_ready() is
 * detected, uart_fifo_read() must be called until it reads all
 * available data in the FIFO (i.e. until it returns less data
 * than was requested).
 *
 * @param dev UART device instance.
 * @param rx_data Data container.
 * @param size Container size.
 *
 * @return Number of bytes read.
 * @retval -ENOSYS If this function is not implemented.
 * @retval -ENOTSUP If API is not enabled.
 */
static int uart_fifo_read(const struct device *dev, uint8_t *rx_data,
                          const int size)

{
    int len;
    struct mbox_env_tag *env = &mbox_env;

    // Sanity check
    __ASSERT(rx_data!=NULL, "Invalid rx data");
    __ASSERT(size!= 0, "Invalid size");

    GLOBAL_INT_DISABLE();
    len = ipc_queue_read(env->ipc_port, rx_data, size);
    GLOBAL_INT_RESTORE();
    return len;
}


/**
 * @brief Fill FIFO with data.
 *
 * @details This function is expected to be called from UART
 * interrupt handler (ISR), if uart_irq_tx_ready() returns true.
 * Result of calling this function not from an ISR is undefined
 * (hardware-dependent). Likewise, *not* calling this function
 * from an ISR if uart_irq_tx_ready() returns true may lead to
 * undefined behavior, e.g. infinite interrupt loops. It's
 * mandatory to test return value of this function, as different
 * hardware has different FIFO depth (oftentimes just 1).
 *
 * @param dev UART device instance.
 * @param tx_data Data to transmit.
 * @param size Number of bytes to send.
 *
 * @return Number of bytes sent.
 * @retval -ENOSYS  if this function is not supported
 * @retval -ENOTSUP If API is not enabled.
 */
static int uart_fifo_fill(const struct device *dev,
                          const uint8_t *tx_data,
                          int size)
{
    struct mbox_env_tag *env = &mbox_env;

    __ASSERT(rx_data!=NULL, "Invalid tx data");
    __ASSERT(size!= 0, "Invalid size");
    int written;

    written = ipc_queue_write(env->ipc_port, tx_data, size, 10);
    LOG_DBG("ipc_queue_write %d,%d\n", size,written);

    // TODO: Check data written.
    return written;

}

static int32_t mbox_rx_ind(ipc_queue_handle_t handle, size_t size)
{
    struct h4_data *h4 = &h4_data_0;

    LOG_DBG("mbox_rx_ind");
    k_sem_give(&(h4->sem));
    return 0;
}

static void mbox_sf32lb_isr(const struct device *dev)
{
    extern void LCPU2HCPU_IRQHandler(void);
    LOG_DBG("mbox_sf32lb_isr %p", (void *)dev);
    LCPU2HCPU_IRQHandler();    
}

int zbt_config_mailbox(void)
{
    ipc_queue_cfg_t q_cfg;

    //__asm("B .");
    struct h4_data *h4 = &h4_data_0;
    k_sem_init(&(h4->sem), 0, 1);

    q_cfg.qid = IO_MB_CH;
    q_cfg.tx_buf_size = TX_BUF_SIZE;
    q_cfg.tx_buf_addr = TX_BUF_ADDR;
    q_cfg.tx_buf_addr_alias = TX_BUF_ADDR_ALIAS;
#ifndef SF32LB52X
    /* Config IPC queue. */
    q_cfg.rx_buf_addr = RX_BUF_ADDR;
#else // SF32LB52X
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id < HAL_CHIP_REV_ID_A4)
    {
        q_cfg.rx_buf_addr = RX_BUF_ADDR;
    }
    else
    {
        q_cfg.rx_buf_addr = RX_BUF_REV_B_ADDR;
    }
#endif // !SF32LB52X

    q_cfg.rx_ind = NULL;
    q_cfg.user_data = 0;

    if (q_cfg.rx_ind == NULL) {
        q_cfg.rx_ind = mbox_rx_ind;
    }
    
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), mbox_sf32lb_isr, DEVICE_DT_INST_GET(0),0);
    mbox_env.ipc_port = ipc_queue_init(&q_cfg);
    __ASSERT(IPC_QUEUE_INVALID_HANDLE != mbox_env.ipc_port, "Invalid Handle");
    if (ipc_queue_open(mbox_env.ipc_port))
        __ASSERT(0,"Could not open IPC");
    irq_enable(DT_INST_IRQN(0));
    
    
    mbox_env.is_init = 1;
    return 0;
}


