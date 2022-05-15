#define LOG_TAG         "Plat"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_firmware.h"

extern struct chipone_ts_data *cts_data;

#ifdef CFG_CTS_FW_LOG_REDIRECT
size_t cts_plat_get_max_fw_log_size(struct cts_platform_data *pdata)
{
    return CTS_FW_LOG_BUF_LEN;
}

u8 *cts_plat_get_fw_log_buf(struct cts_platform_data *pdata,
    size_t size)
{
    return pdata->fw_log_buf;
}
#endif

size_t cts_plat_get_max_i2c_xfer_size(struct cts_platform_data *pdata)
{
    return CFG_CTS_MAX_I2C_XFER_SIZE;
}

#ifdef CONFIG_CTS_I2C_HOST
#else
//mtk spi debug add start
#ifdef CONFIG_SPI_MT65XX
struct mtk_chip_config chipone_spi_ctrdata = {
    .rx_mlsb = 1,
    .tx_mlsb = 1,    
    .cs_pol  = 0,
    .sample_sel = 0,
    .cs_setuptime = 15,
    .cs_holdtime = 15,
    .cs_idletime = 20,
};
#elif defined CONFIG_MTK_SPI
struct mt_chip_conf chipone_spi_ctrdata = {
    .setuptime = 15,
    .holdtime = 15,
    .high_time = 21, //for mt6582, 104000khz/(4+4) = 130000khz
    .low_time = 21,
    .cs_idletime = 20,
    .ulthgh_thrsh = 0,

    .cpol = 0,
    .cpha = 0,

    .rx_mlsb = 1,
    .tx_mlsb = 1,

    .tx_endian = 0,
    .rx_endian = 0,
    .pause = 1,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};
#elif defined CONFIG_ARCH_BENGAL

const struct spi_geni_qcom_ctrl_data chipone_spi_ctrdata = {
    .spi_cs_clk_delay = 3,
};

#endif
//mtk spi debug add end

int cts_spi_send_recv(struct cts_platform_data *pdata, size_t len , u8 *tx_buffer, u8 *rx_buffer)
{
    struct chipone_ts_data *cts_data;
    struct spi_message msg;
    struct spi_transfer cmd = {
        .cs_change = 0,
        .delay_usecs = 0,
        .speed_hz = pdata->spi_speed,
        //.speed_hz = cts_spi_speed * 1000u,
        .tx_buf = tx_buffer,
        .rx_buf = rx_buffer,
        .len    = len,
        //.tx_dma = 0,
        //.rx_dma = 0,
        .bits_per_word = 8,
    };
    int ret = 0;
    cts_data = container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);
#ifdef CFG_CTS_MANUAL_CS
    cts_plat_set_cs(pdata, 0);
#endif
    spi_message_init(&msg);
    spi_message_add_tail(&cmd,  &msg);
    ret = spi_sync(cts_data->spi_client, &msg);
    if (ret) {
        cts_err("spi sync failed %d", ret);
    }
#ifdef CFG_CTS_MANUAL_CS
    cts_plat_set_cs(pdata, 1);
#endif
    return ret;
}

size_t cts_plat_get_max_spi_xfer_size(struct cts_platform_data *pdata)
{
    return CFG_CTS_MAX_SPI_XFER_SIZE;
}

u8 *cts_plat_get_spi_xfer_buf(struct cts_platform_data *pdata,
    size_t xfer_size)
{
    return pdata->spi_cache_buf;
}

int cts_plat_spi_write(struct cts_platform_data *pdata, u8 dev_addr,
        const void *src, size_t len, int retry, int delay)
{
    int ret = 0, retries = 0;
    u16 crc;
    size_t data_len;

    if (len > CFG_CTS_MAX_SPI_XFER_SIZE) {
        cts_err("write too much data:wlen=%zu\n", len);
        return -EIO;
    }

    if (pdata->cts_dev->rtdata.program_mode) {
        pdata->spi_tx_buf[0] = dev_addr;
        memcpy(&pdata->spi_tx_buf[1], src, len);

        do {
            ret = cts_spi_send_recv(pdata, len + 1, pdata->spi_tx_buf, pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI write failed %d", ret);
                if (delay) {
                    mdelay(delay);
                }
            } else {
                return 0;
            }
        } while (++retries < retry);
    }
    else {
        data_len = len - 2;
        pdata->spi_tx_buf[0] = dev_addr;
        pdata->spi_tx_buf[1] = *((u8 *)src + 1);
        pdata->spi_tx_buf[2] = *((u8 *)src);
        put_unaligned_le16(data_len, &pdata->spi_tx_buf[3]);
        crc = (u16)cts_crc32(pdata->spi_tx_buf, 5);
        put_unaligned_le16(crc, &pdata->spi_tx_buf[5]);
        memcpy(&pdata->spi_tx_buf[7], (char *)src + 2, data_len);
        crc = (u16)cts_crc32((char *)src + 2, data_len);
        put_unaligned_le16(crc, &pdata->spi_tx_buf[7+data_len]);
        do {
            ret = cts_spi_send_recv(pdata, len + 7, pdata->spi_tx_buf, pdata->spi_rx_buf);
            udelay(10 * data_len);
            if (ret) {
                cts_err("SPI write failed %d", ret);
                if (delay) {
                    mdelay(delay);
                }
            } else {
                return 0;
            }
        } while (++retries < retry);
    }
    return ret;
}

/*
spi debug
*/
#if 1
int cts_plat_spi_read(struct cts_platform_data *pdata, u8 dev_addr,
        const u8 *wbuf, size_t wlen, void *rbuf, size_t rlen,
        int retry, int delay)
{
    int ret = 0, retries = 0;
    u16 crc;

    if (wlen > CFG_CTS_MAX_SPI_XFER_SIZE || rlen > CFG_CTS_MAX_SPI_XFER_SIZE) {
        cts_err("write/read too much data:wlen=%zd, rlen=%zd", wlen, rlen);
        return -EIO;
    }

    if (pdata->cts_dev->rtdata.program_mode)
    {
        pdata->spi_tx_buf[0] = dev_addr | 0x01;
        memcpy(&pdata->spi_tx_buf[1], wbuf, wlen);
        do {
            ret = cts_spi_send_recv(pdata, rlen + 5, pdata->spi_tx_buf, pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI read failed %d", ret);
                if (delay) {
                    mdelay(delay);
                }
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf+5, rlen);
            return 0;
        } while(++retries < retry);
    } else {
        do {
            if (wlen != 0) {
                pdata->spi_tx_buf[0] = dev_addr | 0x01;
                pdata->spi_tx_buf[1] = wbuf[1];
                pdata->spi_tx_buf[2] = wbuf[0];
                put_unaligned_le16(rlen, &pdata->spi_tx_buf[3]);
                crc = (u16)cts_crc32(pdata->spi_tx_buf, 5);
                put_unaligned_le16(crc, &pdata->spi_tx_buf[5]);
                ret = cts_spi_send_recv(pdata, 7, pdata->spi_tx_buf, pdata->spi_rx_buf);
                if (ret) {
                    cts_err("SPI read failed %d", ret);
                    if (delay) {
                        mdelay(delay);
                    }
                    continue;
                }
            }
            memset(pdata->spi_tx_buf, 0, 7);
            pdata->spi_tx_buf[0] = dev_addr | 0x01;
            udelay(100);
            ret =
                cts_spi_send_recv(pdata, rlen + 2,
                          pdata->spi_tx_buf,
                          pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI read failed %d", ret);
                if (delay) {
                    mdelay(delay);
                }
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf, rlen);
            crc = (u16) cts_crc32(pdata->spi_rx_buf, rlen);
            if (get_unaligned_le16(&pdata->spi_rx_buf[rlen]) != crc) {
                cts_err("SPI RX CRC error: rx_crc %04x != %04x",
                    get_unaligned_le16(&pdata->spi_rx_buf[rlen]), crc);
                continue;
            }
            return 0;
        } while (++retries < retry);
    }
    if (retries >= retry) {
        cts_err("SPI read too much retry");
    }

    return -EIO;
}
#else
int cts_plat_spi_read(struct cts_platform_data *pdata, u8 dev_addr,
        const u8 *wbuf, size_t wlen, void *rbuf, size_t rlen,
        int retry, int delay)
{
    int ret = 0, retries = 0;
    u16 crc;

    if (wlen > CFG_CTS_MAX_SPI_XFER_SIZE || rlen > CFG_CTS_MAX_SPI_XFER_SIZE) {
        cts_err("write/read too much data:wlen=%zd, rlen=%zd", wlen, rlen);
        return -EIO;
    }

    if (pdata->cts_dev->rtdata.program_mode)
    {
        pdata->spi_tx_buf[0] = dev_addr | 0x01;
        memcpy(&pdata->spi_tx_buf[1], wbuf, wlen);
        do {
            ret = cts_spi_send_recv(pdata, rlen + 5, pdata->spi_tx_buf, pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI read failed %d", ret);
                if (delay) {
                    mdelay(delay);
                }
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf+5, rlen);
            return 0;
        } while(++retries < retry);
    } else {
        do {
            extern u16 cts_spi_speed;
            u8 temp_wbuf[7];
            struct spi_message msg;

            struct chipone_ts_data *cts_data;
            struct spi_transfer cmd[2] = {
                {
                    .cs_change = 1,
                    .delay_usecs = 30,
                    //.speed_hz = cts_spi_speed * 1000u,
                    .speed_hz = pdata->spi_speed,
                    .tx_buf = temp_wbuf,
                    .rx_buf =  pdata->spi_rx_buf,
                    .len    = 7,
                    .bits_per_word = 8,
                },
                {
                    .cs_change = 1,
                    .delay_usecs = 10,
                    //.speed_hz = cts_spi_speed * 1000u,
                    .speed_hz = pdata->spi_speed,
                    .tx_buf = pdata->spi_tx_buf,
                    .rx_buf = pdata->spi_rx_buf,
                    .len    = rlen + 2,
                    .bits_per_word = 8,
                },
            };

            cts_data = container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);
            if (wlen != 0) {
                temp_wbuf[0] = dev_addr | 0x01;
                temp_wbuf[1] = wbuf[1];
                temp_wbuf[2] = wbuf[0];
                put_unaligned_le16(rlen, &temp_wbuf[3]);
                crc = (u16)cts_crc32(temp_wbuf, 5);
                put_unaligned_le16(crc, &temp_wbuf[5]);
            }
            memset(pdata->spi_tx_buf, 0, 7);
            pdata->spi_tx_buf[0] = dev_addr | 0x01;
#ifdef CFG_CTS_MANUAL_CS
            cts_plat_set_cs(pdata, 0);
#endif
            spi_message_init(&msg);
            spi_message_add_tail(&cmd[0],  &msg);
            spi_message_add_tail(&cmd[1],  &msg);
            ret = spi_sync(cts_data->spi_client, &msg);
#ifdef CFG_CTS_MANUAL_CS
            cts_plat_set_cs(pdata, 1);
#endif            
            if (ret) {
                cts_err("SPI sync failed %d", ret);
                if (delay) {
                    mdelay(delay);
                }
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf, rlen);
            crc = (u16) cts_crc32(pdata->spi_rx_buf, rlen);
            if (get_unaligned_le16(&pdata->spi_rx_buf[rlen]) != crc) {
                cts_err("SPI RX CRC error: rx_crc %04x != %04x",
                    get_unaligned_le16(&pdata->spi_rx_buf[rlen]), crc);
                continue;
            }
            return 0;
        } while (++retries < retry);
    }
    if (retries >= retry) {
        cts_err("SPI read too much retry");
    }

    return -EIO;
}

#endif


int cts_plat_spi_read_delay_idle(struct cts_platform_data *pdata, u8 dev_addr,
        const u8 *wbuf, size_t wlen, void *rbuf,
        size_t rlen, int retry, int delay, int idle)
{
    int ret = 0, retries = 0;
    u16 crc;

    if (wlen > CFG_CTS_MAX_SPI_XFER_SIZE ||
        rlen > CFG_CTS_MAX_SPI_XFER_SIZE) {
        cts_err("write/read too much data:wlen=%zu, rlen=%zu", wlen,
            rlen);
        return -E2BIG;
    }

    if (pdata->cts_dev->rtdata.program_mode) {
        pdata->spi_tx_buf[0] = dev_addr | 0x01;
        memcpy(&pdata->spi_tx_buf[1], wbuf, wlen);
        do {
            ret =
                cts_spi_send_recv(pdata, rlen + 5,
                    pdata->spi_tx_buf,
                    pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI read failed %d", ret);
                if (delay) {
                    mdelay(delay);
                }
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf + 5, rlen);
            return 0;
        } while (++retries < retry);
    } else {
        do {
            if (wlen != 0) {
                pdata->spi_tx_buf[0] = dev_addr | 0x01;
                pdata->spi_tx_buf[1] = wbuf[1];
                pdata->spi_tx_buf[2] = wbuf[0];
                put_unaligned_le16(rlen, &pdata->spi_tx_buf[3]);
                crc = (u16) cts_crc32(pdata->spi_tx_buf, 5);
                put_unaligned_le16(crc, &pdata->spi_tx_buf[5]);
                ret =
                    cts_spi_send_recv(pdata, 7,
                        pdata->spi_tx_buf,
                        pdata->spi_rx_buf);
                if (ret) {
                    cts_err("SPI read failed %d", ret);
                    if (delay) {
                        mdelay(delay);
                    }
                    continue;
                }
            }
            memset(pdata->spi_tx_buf, 0, 7);
            pdata->spi_tx_buf[0] = dev_addr | 0x01;
            udelay(idle);
            ret =
                cts_spi_send_recv(pdata, rlen + 2,
                    pdata->spi_tx_buf,
                    pdata->spi_rx_buf);
            if (ret) {
                if (delay) {
                    mdelay(delay);
                }
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf, rlen);
            crc = (u16)cts_crc32(pdata->spi_rx_buf, rlen);
            if (get_unaligned_le16(&pdata->spi_rx_buf[rlen]) != crc) {
                continue;
            }
            return 0;
        } while (++retries < retry);
    }
    if (retries >= retry) {
        cts_err("cts_plat_spi_read error");
    }

    return -EIO;
}

int cts_plat_is_normal_mode(struct cts_platform_data *pdata)
{
    struct chipone_ts_data *cts_data;
    u8 tx_buf[4] = {0};
    u16 fwid;
    u32 addr;
    int ret;

    cts_set_normal_addr(pdata->cts_dev);
    cts_data = container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);
    addr = CTS_DEVICE_FW_REG_CHIP_TYPE;
    put_unaligned_be16(addr, tx_buf);
    ret = cts_plat_spi_read(pdata, CTS_DEV_NORMAL_MODE_SPIADDR, tx_buf, 2, &fwid, 2, 3, 10);
    fwid = be16_to_cpu(fwid);
    if (ret || !cts_is_fwid_valid(fwid)) {
        return false;
    }

    return true;
}
#endif


#if defined(CFG_CTS_HANDLE_IRQ_USE_WORKQUEUE)

#endif /* CFG_CTS_HANDLE_IRQ_USE_WORKQUEUE */

#ifdef CFG_CTS_HANDLE_IRQ_USE_KTHREAD

#endif // CFG_CTS_HANDLE_IRQ_USE_KTHREAD 

#ifdef CONFIG_CTS_OF
static int cts_plat_parse_dt(struct cts_platform_data *pdata,
        struct device_node *dev_node)
{
    int ret = 0;

    cts_info("Parse device tree");

    pdata->int_gpio = of_get_named_gpio(dev_node, CFG_CTS_OF_INT_GPIO_NAME, 0);
    if (!gpio_is_valid(pdata->int_gpio)) {
        cts_err("Parse INT GPIO from dt failed %d", pdata->int_gpio);
        pdata->int_gpio = -1;
    }
    cts_info("  %-12s: %d", "int gpio", pdata->int_gpio);

    pdata->irq = gpio_to_irq(pdata->int_gpio);
    if (pdata->irq < 0) {
        cts_err("Parse irq failed %d", ret);
        return pdata->irq;
    }
    cts_info("  %-12s: %d", "irq num", pdata->irq);

#ifdef CFG_CTS_HAS_RESET_PIN
    pdata->rst_gpio = of_get_named_gpio(dev_node, CFG_CTS_OF_RST_GPIO_NAME, 0);
    if (!gpio_is_valid(pdata->rst_gpio)) {
        cts_err("Parse RST GPIO from dt failed %d", pdata->rst_gpio);
        pdata->rst_gpio = -1;
    }
    cts_info("  %-12s: %d", "rst gpio", pdata->rst_gpio);
#endif /* CFG_CTS_HAS_RESET_PIN */

    ret = of_property_read_u32(dev_node, CFG_CTS_OF_X_RESOLUTION_NAME,
            &pdata->res_x);
    if (ret) {
        cts_warn("Parse X resolution from dt failed %d", ret);
        //return ret;
    }
    cts_info("  %-12s: %d", "X resolution", pdata->res_x);

    ret = of_property_read_u32(dev_node, CFG_CTS_OF_Y_RESOLUTION_NAME,
            &pdata->res_y);
    if (ret) {
        cts_warn("Parse Y resolution from dt failed %d", ret);
        //return ret;
    }
    cts_info("  %-12s: %d", "Y resolution", pdata->res_y);

    ret = of_property_read_u32(dev_node, "spi-max-frequency", &pdata->spi_speed);
	if (ret || pdata->spi_speed <= 0 || pdata->spi_speed > 9600000) {
		pdata->spi_speed = 6000000;
		cts_err("error reading spi frequency, use default frequency");
		ret = 0;
	} 
	cts_info("spi frequency:%d\n", pdata->spi_speed);


    return 0;
}
#endif /* CONFIG_CTS_OF */


int cts_init_platform_data(struct cts_platform_data *pdata, struct spi_device *spi, struct device_node *dev_node)

{
    int ret = 0;  
#ifdef CFG_CTS_GESTURE
    u8 gesture_keymap[CFG_CTS_NUM_GESTURE][2] = CFG_CTS_GESTURE_KEYMAP;
#endif // CFG_CTS_GESTURE 


#ifdef CONFIG_CTS_OF
    {
        struct device *dev;
        dev = &spi->dev;
        ret = cts_plat_parse_dt(pdata, dev_node);
        if (ret) {
            cts_err("Parse dt failed %d", ret);
            return ret;
        }
    }
#endif /* CONFIG_CTS_OF */

    cts_info("Init");

    pdata->spi_client = spi;
    pdata->spi_client->irq = pdata->irq;

    mutex_init(&pdata->dev_lock);
    spin_lock_init(&pdata->irq_lock);


#ifdef CFG_CTS_GESTURE
    {
        //u8 gesture_keymap[CFG_CTS_NUM_GESTURE][2] = CFG_CTS_GESTURE_KEYMAP;
        memcpy(pdata->gesture_keymap, gesture_keymap, sizeof(gesture_keymap));
        pdata->gesture_num = CFG_CTS_NUM_GESTURE;
    }
#endif // CFG_CTS_GESTURE 

    
    pdata->spi_client->mode = SPI_MODE_0;
    pdata->spi_client->bits_per_word = 8;
    //pdata->spi_speed = CFG_CTS_SPI_SPEED_KHZ;
    pdata->spi_speed = pdata->spi_speed;
    //spi_setup(pdata->spi_client);
#if defined CONFIG_SPI_MT65XX
    memcpy(&pdata->spi_ctrdata, &chipone_spi_ctrdata, sizeof(struct mtk_chip_config));
    pdata->spi_client->controller_data = (void *)&pdata->spi_ctrdata;
#elif defined CONFIG_MTK_SPI
    memcpy(&pdata->spi_ctrdata, &chipone_spi_ctrdata, sizeof(struct mt_chip_conf));
    pdata->spi_client->controller_data = (void *)&pdata->spi_ctrdata;
#endif

#ifdef CONFIG_ARCH_BENGAL
    memcpy(&pdata->spi_ctrdata, &chipone_spi_ctrdata, sizeof(struct spi_geni_qcom_ctrl_data));
    pdata->spi_client->controller_data = (void *)&pdata->spi_ctrdata;
    
#endif    
    pdata->spi_client->max_speed_hz = pdata->spi_speed;
    spi_setup(pdata->spi_client);
    return 0;
}

int cts_plat_request_resource(struct cts_platform_data *pdata)
{
    int ret;

    cts_info("Request resource");

    ret = gpio_request_one(pdata->int_gpio, GPIOF_IN,
        CFG_CTS_DEVICE_NAME "-int");
    if (ret) {
        cts_err("Request INT gpio (%d) failed %d", pdata->int_gpio, ret);
        goto err_out;
    }

#ifdef CFG_CTS_HAS_RESET_PIN
    ret = gpio_request_one(pdata->rst_gpio, GPIOF_OUT_INIT_HIGH,
        CFG_CTS_DEVICE_NAME "-rst");
    if (ret) {
        cts_err("Request RST gpio (%d) failed %d", pdata->rst_gpio, ret);
        goto err_free_int;
    }
#endif /* CFG_CTS_HAS_RESET_PIN */


    return 0;

#ifdef CONFIG_CTS_REGULATOR
err_free_rst:
#endif /* CONFIG_CTS_REGULATOR */
#ifdef CFG_CTS_HAS_RESET_PIN
    gpio_free(pdata->rst_gpio);
err_free_int:
#endif /* CFG_CTS_HAS_RESET_PIN */
    gpio_free(pdata->int_gpio);
err_out:
    return ret;
}

void cts_plat_free_resource(struct cts_platform_data *pdata)
{
    cts_info("Free resource");

    gpio_set_value(pdata->rst_gpio, 0);
    mdelay(10);

    if (gpio_is_valid(pdata->int_gpio)) {
        gpio_free(pdata->int_gpio);
    }
#ifdef CFG_CTS_HAS_RESET_PIN
    if (gpio_is_valid(pdata->rst_gpio)) {
        gpio_free(pdata->rst_gpio);
    }
#endif /* CFG_CTS_HAS_RESET_PIN */
}


void cts_plat_free_irq(struct cts_platform_data *pdata)
{
    free_irq(pdata->irq, pdata);
}

int cts_plat_enable_irq(struct cts_platform_data *pdata)
{
    unsigned long irqflags;

    cts_dbg("Enable IRQ");

    if (pdata->irq > 0) {
        spin_lock_irqsave(&pdata->irq_lock, irqflags);
        if (pdata->irq_is_disable)/* && !cts_is_device_suspended(pdata->chip)) */{
            enable_irq(pdata->irq);
            pdata->irq_is_disable = false;
        }
        spin_unlock_irqrestore(&pdata->irq_lock, irqflags);

        return 0;
    }

    return -ENODEV;
}

int cts_plat_disable_irq(struct cts_platform_data *pdata)
{
    unsigned long irqflags;

    cts_dbg("Disable IRQ");

    if (pdata->irq > 0) {
        spin_lock_irqsave(&pdata->irq_lock, irqflags);
        if (!pdata->irq_is_disable) {
            disable_irq_nosync(pdata->irq);
            pdata->irq_is_disable = true;
        }
        spin_unlock_irqrestore(&pdata->irq_lock, irqflags);

        return 0;
    }

    return -ENODEV;
}

#ifdef CFG_CTS_HAS_RESET_PIN
int cts_plat_reset_device(struct cts_platform_data *pdata)
{
    /* !!!can not be modified*/
    /* !!!can not be modified*/
    /* !!!can not be modified*/
    cts_info("Reset device");

    gpio_set_value(pdata->rst_gpio, 1);
    mdelay(1);
    gpio_set_value(pdata->rst_gpio, 0);
    //mdelay(10);
    mdelay(4);
    gpio_set_value(pdata->rst_gpio, 1);
    //mdelay(40);
    mdelay(6);

    return 0;
}

int cts_plat_set_reset(struct cts_platform_data *pdata, int val)
{
    cts_info("Set reset-pin to %s", val ? "HIGH" : "LOW");
    if (val) {
        gpio_set_value(pdata->rst_gpio, 1);
    } else {
        gpio_set_value(pdata->rst_gpio, 0);
    }
    return 0;
}
#endif /* CFG_CTS_HAS_RESET_PIN */

int cts_plat_get_int_pin(struct cts_platform_data *pdata)
{
    return gpio_get_value(pdata->int_gpio);
}

int cts_plat_power_up_device(struct cts_platform_data *pdata)
{
    cts_info("Power up device");

    return 0;
}

int cts_plat_power_down_device(struct cts_platform_data *pdata)
{
    cts_info("Power down device");

    return 0;
}



int cts_plat_release_all_touch(struct cts_platform_data *pdata)
{
    //struct input_dev *input_dev = pdata->ts_input_dev;
    struct input_dev *input_dev = cts_data->vtsdev->idev;
#if defined(CONFIG_CTS_SLOTPROTOCOL)
    int id;
#endif /* CONFIG_CTS_SLOTPROTOCOL */

    cts_info("Release all touch");

#ifdef CONFIG_CTS_SLOTPROTOCOL
    for (id = 0; id < CFG_CTS_MAX_TOUCH_NUM; id++) {
        input_mt_slot(input_dev, id);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
    }
    input_report_key(input_dev, BTN_TOUCH, 0);
#else
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_mt_sync(input_dev);
#endif /* CONFIG_CTS_SLOTPROTOCOL */
    input_sync(input_dev);

    return 0;
}



#ifdef CFG_CTS_GESTURE
int cts_plat_enable_irq_wake(struct cts_platform_data *pdata)
{
    cts_info("Enable IRQ wake");

    if (pdata->irq > 0) {
        if (!pdata->irq_wake_enabled) {
            pdata->irq_wake_enabled = true;
            return enable_irq_wake(pdata->irq);
        }

        cts_warn("Enable irq wake while already disabled");
        return -EINVAL;
    }

    cts_warn("Enable irq wake while irq invalid %d", pdata->irq);
    return -ENODEV;
}

int cts_plat_disable_irq_wake(struct cts_platform_data *pdata)
{
    cts_info("Disable IRQ wake");

    if (pdata->irq > 0) {
        if (pdata->irq_wake_enabled) {
            pdata->irq_wake_enabled = false;
            return disable_irq_wake(pdata->irq);
        }

        cts_warn("Disable irq wake while already disabled");
        return -EINVAL;
    }

    cts_warn("Disable irq wake while irq invalid %d", pdata->irq);
    return -ENODEV;
}

#endif /* CFG_CTS_GESTURE */

