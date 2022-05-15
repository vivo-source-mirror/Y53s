

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/completion.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "fts.h"
#include "fts_lib/ftsCompensation.h"
#include "fts_lib/ftsIO.h"
#include "fts_lib/ftsError.h"
#include "fts_lib/ftsFrame.h"
#include "fts_lib/ftsTest.h"
#include "fts_lib/ftsTime.h"
#include "fts_lib/ftsTool.h"
#include "../vts_core.h"

#ifdef SCRIPTLESS

/*I2C CMd functions: functions to interface with GUI without script */

static ssize_t fts_i2c_wr_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	int i;
	char buff[16];
	memset(info->Out_buff, 0x00, ARRAY_SIZE(info->Out_buff));
	if (info->byte_count_read == 0) {
		snprintf(info->Out_buff, sizeof(info->Out_buff), "{FAILED}");
		return snprintf(buf, TSP_BUF_SIZE, "%s\n", info->Out_buff);
	}
#ifdef SCRIPTLESS_DEBUG
	 printk("%s:DATA READ {", __func__);
	for (i = 0; i < info->byte_count_read; i++) {
		printk(" %02X", (unsigned int)info->cmd_wr_result[i]);
		if (i < (info->byte_count_read - 1)) {
			printk(" ");
		}
	}
	printk("}\n");
#endif
	snprintf(buff, sizeof(buff), "{");
	strlcat(info->Out_buff, buff, ARRAY_SIZE(info->Out_buff));
	for (i = 0; i < (info->byte_count_read + 2); i++) {
		if (i == 0) {
			char temp_byte_count_read = (info->byte_count_read >> 8) & 0xFF;
			snprintf(buff, sizeof(buff), "%02X", temp_byte_count_read);
		} else if (i == 1) {
			char temp_byte_count_read = (info->byte_count_read) & 0xFF;
			snprintf(buff, sizeof(buff), "%02X", temp_byte_count_read);

		} else {
			snprintf(buff, sizeof(buff), "%02X", info->cmd_wr_result[i - 2]);
		}
		/*snprintf(buff, sizeof(buff), "%02X", info->cmd_wr_result[i]); */
		strlcat(info->Out_buff, buff, ARRAY_SIZE(info->Out_buff));
		if (i < (info->byte_count_read + 1)) {
			snprintf(buff, sizeof(buff), " ");
			strlcat(info->Out_buff, buff, ARRAY_SIZE(info->Out_buff));
		}
	}
	snprintf(buff, sizeof(buff), "}");
	strlcat(info->Out_buff, buff, ARRAY_SIZE(info->Out_buff));
	return snprintf(buf, TSP_BUF_SIZE, "%s\n", info->Out_buff);
}

static ssize_t fts_i2c_wr_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char pAddress[9] = {0};
	unsigned int byte_count = 0;
	int i;

	unsigned int data[9] = {0};

	memset(data, 0x00, ARRAY_SIZE(data));
	memset(info->cmd_wr_result, 0x00, ARRAY_SIZE(info->cmd_wr_result));
	sscanf(buf, "%x %x %x %x %x %x %x %x %x ", (data+8), (data),(data+1),(data+2),(data+3),(data+4),(data+5),(data+6),(data+7));

	byte_count = data[8];

	/*if (sizeof(buf) != byte_count) {
		printk("%s : Byte count is wrong\n", __func__);
		return count;
	} */
#ifdef SCRIPTLESS_DEBUG
	printk(" \n");
	printk("%s: Input Data 1:", __func__);

	for(i =0 ; i <byte_count; i++) {
		 printk(" %02X", data[i]);
		pAddress[i] = (unsigned char)data[i];
	}
	printk("\n");
#else
	for(i =0 ; i <byte_count; i++) {
		pAddress[i] = (unsigned char)data[i];
	}
#endif
	info->byte_count_read = (((unsigned int) data[byte_count-2])<<8) | data[byte_count-1];
	ret = ftm4_writeCmd(info, pAddress, 3);
	msleep(20);
	ret = ftm4_readCmd(info, &pAddress[3], (byte_count - 5), info->cmd_wr_result, info->byte_count_read);
#ifdef SCRIPTLESS_DEBUG
	 printk("%s:DATA READ \n{", __func__);
	for (i = 0; i < (2 + info->byte_count_read); i++) {
		if ((i == 0)) {
			char temp_byte_count_read = (info->byte_count_read >> 8) & 0xFF;
			printk("%02X", (unsigned int)temp_byte_count_read);
		} else if (i == 1) {
			char temp_byte_count_read = (info->byte_count_read) & 0xFF;
			printk("%02X", (unsigned int)temp_byte_count_read);

		} else {
			printk("%02X", (unsigned int)info->cmd_read_result[i - 2]);
		}
		if (i < (info->byte_count_read + 1)) {
			printk(" ");
		}

	}
	printk("}\n");
#endif
	if (ret)
		dev_err(dev, "Unable to read register \n");
	return count;
}

static ssize_t fts_i2c_read_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	int i;
	char buff[16];

	memset(info->Out_buff, 0x00, ARRAY_SIZE(info->Out_buff));
	if (info->byte_count_read == 0) {
		snprintf(info->Out_buff, sizeof(info->Out_buff), "{FAILED}");
		return snprintf(buf, TSP_BUF_SIZE, "%s\n", info->Out_buff);
	}
#ifdef SCRIPTLESS_DEBUG
	printk("%s:DATA READ {", __func__);
	for (i = 0; i < info->byte_count_read; i++) {
		printk("%02X", (unsigned int)info->cmd_read_result[i]);
		if (i < (info->byte_count_read - 1)) {
			printk(" ");
		}
	}
	printk("}\n");
#endif
	snprintf(buff, sizeof(buff), "{");
	strlcat(info->Out_buff, buff, ARRAY_SIZE(info->Out_buff));
	for (i = 0; i < (info->byte_count_read + 2); i++) {
		if (i == 0) {
			char temp_byte_count_read = (info->byte_count_read >> 8) & 0xFF;
			snprintf(buff, sizeof(buff), "%02X", temp_byte_count_read);
		} else if (i == 1) {
			char temp_byte_count_read = (info->byte_count_read) & 0xFF;
			snprintf(buff, sizeof(buff), "%02X", temp_byte_count_read);

		} else {
			snprintf(buff, sizeof(buff), "%02X", info->cmd_read_result[i - 2]);
		}
		strlcat(info->Out_buff, buff, ARRAY_SIZE(info->Out_buff));
		if (i < (info->byte_count_read + 1)) {
			snprintf(buff, sizeof(buff), " ");
			strlcat(info->Out_buff, buff, ARRAY_SIZE(info->Out_buff));
		}
	}
	snprintf(buff, sizeof(buff), "}");
	strlcat(info->Out_buff, buff, ARRAY_SIZE(info->Out_buff));

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", info->Out_buff);
}

static ssize_t fts_i2c_read_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char pAddress[9] = {0};
	unsigned int byte_count = 0;
	int i;
	unsigned int data[9] = {0};

	info->byte_count_read = 0;
	memset(data, 0x00, ARRAY_SIZE(data));
	memset(info->cmd_read_result, 0x00, ARRAY_SIZE(info->cmd_read_result));
	sscanf(buf, "%x %x %x %x %x %x %x %x %x ", (data+8), (data),(data+1),(data+2),(data+3),(data+4),(data+5),(data+6),(data+7));
	byte_count = data[8];


	if(byte_count >8 ) {
#ifdef SCRIPTLESS_DEBUG
		printk("%s : Byte count is more than 8\n", __func__);
#endif
		return count;
	}
	/*if (sizeof(buf) != byte_count) {
		printk("%s : Byte count is wrong\n", __func__);
		return count;
	} */
#ifdef SCRIPTLESS_DEBUG
	printk(" \n");
	printk("%s: Input Data 1:", __func__);
	for (i = 0; i < byte_count; i++) {
		 printk(" %02X", data[i]);
		pAddress[i] = (unsigned char)data[i];
	}
	printk(" \n");
#else
	for (i = 0; i < byte_count; i++) {
		pAddress[i] = (unsigned char)data[i];
	}
#endif
	info->byte_count_read = (((unsigned int) data[byte_count-2])<<8)|data[byte_count-1];
	ret = ftm4_readCmd(info, pAddress, (byte_count - 2), info->cmd_read_result, info->byte_count_read);
#ifdef SCRIPTLESS_DEBUG
	printk("%s:DATA READ \n{", __func__);
	for (i = 0; i < (info->byte_count_read + 2); i++) {
		if ((i == 0)) {
			char temp_byte_count_read = (info->byte_count_read >> 8) & 0xFF;
			printk("%02X", (unsigned int)temp_byte_count_read);
		} else if (i == 1) {
			char temp_byte_count_read = (info->byte_count_read) & 0xFF;
			printk("%02X", (unsigned int)temp_byte_count_read);

		} else {
			printk("%02X", (unsigned int)info->cmd_read_result[i - 2]);
		}
		if (i < (info->byte_count_read + 1)) {
			printk(" ");
		}
	}
	printk("}\n");
#endif
	if (ret)
		dev_err(dev, "Unable to read register \n");
	return count;
}


 static ssize_t fts_i2c_write_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	return snprintf(buf, TSP_BUF_SIZE, "%s", info->cmd_write_result);

}

static ssize_t fts_i2c_write_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned int byte_count = 0;
	int i;
	unsigned int *data = NULL;
	unsigned char *pAddress_i2c = NULL;

	data = (unsigned int *)kmalloc(2048 * sizeof(int), GFP_KERNEL);
	if (data == NULL) {
		fts_info(info, "kmalloc data mem fail");
		return count;
	}
	
	pAddress_i2c = (unsigned char *)kmalloc(2048, GFP_KERNEL);
	if (data == NULL) {
		fts_info(info, "kmalloc pAddress_i2c mem fail");
		kfree(data);
		return count;
	}
	
	memset(data, 0x00, 2048 * sizeof(int));
	memset(pAddress_i2c, 0x00, 2048);
	memset(info->cmd_write_result, 0x00, ARRAY_SIZE(info->cmd_write_result));
	sscanf(buf, "%x %x", data, (data + 1));
	byte_count = data[0] << 8 | data[1];

	if (byte_count <= 2048) {
		for (i = 0; i < (byte_count); i++) {
			sscanf(&buf[3 * (i + 2)], "%x ", (data + i));
		}
	} else {
		printk("%s : message size is more than allowed limit of 512 bytes\n", __func__);
		snprintf(info->cmd_write_result, sizeof(info->cmd_write_result), "{Write NOT OK}\n");
	}
	fts_info(info, "Byte_count=  %02d| Count = %02d | size of buf:%02d", byte_count, (int)count, (int)sizeof(buf));
	fts_info(info, "Input Data is:");
	for (i = 0; i < byte_count; i++) {
		printk("%02X", data[i]);
		pAddress_i2c[i] = (unsigned char)data[i];
	}

	if ((pAddress_i2c[0] == 0xb3) && (pAddress_i2c[3] == 0xb1)) {
		ret = ftm4_writeCmd(info, pAddress_i2c, 3);
		msleep(20);
		ret = ftm4_writeCmd(info, &pAddress_i2c[3], byte_count - 3);
	} else {
		ret = ftm4_writeCmd(info, pAddress_i2c, byte_count);
	}

	if (ret < 0) {
		fts_info(info, "Write NOT OK");
		snprintf(info->cmd_write_result, sizeof(info->cmd_write_result), "{Write NOT OK}\n");
	} else {
		snprintf(info->cmd_write_result, sizeof(info->cmd_write_result), "{Write OK}\n");
		fts_info(info, "Write OK");
	}

	kfree(data);
	kfree(pAddress_i2c);
	return count;
}


static DEVICE_ATTR(iread, 0644, NULL, fts_i2c_read_store);		/*(S_IWUGO) */
static DEVICE_ATTR(iread_result, (0644), fts_i2c_read_show, NULL);/*(S_IRUGO) */
static DEVICE_ATTR(iwr, (0644), NULL, fts_i2c_wr_store);	/*(S_IWUGO) */
static DEVICE_ATTR(iwr_result, (0644), fts_i2c_wr_show, NULL);/*(S_IRUGO) */
static DEVICE_ATTR(iwrite, (0644), NULL, fts_i2c_write_store);	/*(S_IWUGO) */
static DEVICE_ATTR(iwrite_result, (0644), fts_i2c_write_show, NULL);/*(S_IRUGO) */


static struct attribute *i2c_cmd_attributes[] = {
	&dev_attr_iread.attr,
	&dev_attr_iread_result.attr,
	&dev_attr_iwr.attr,
	&dev_attr_iwr_result.attr,
	&dev_attr_iwrite.attr,
	&dev_attr_iwrite_result.attr,
	NULL,
};

static struct attribute_group i2c_cmd_attr_group = {
	.attrs = i2c_cmd_attributes,
};

int ftm4_gui_init(struct fts_ts_info *info)
{
	 int ret;
	 char *class_name = info->vtsdev->type == VTS_TYPE_MAIN ? "fts" : "fts-second";
	 char *devname = info->vtsdev->type == VTS_TYPE_MAIN ? "fts_i2c" : "fts_i2c_second";

	 info->fts_cmd_class = class_create(THIS_MODULE, class_name);
	 if (IS_ERR(info->fts_cmd_class)) {
		 VTE("create class fts failed!\n");
		 return PTR_ERR(info->fts_cmd_class);
	 }

	 info->i2c_cmd_dev = device_create(info->fts_cmd_class, NULL, DCHIP_ID_0, info, devname);
	 if (IS_ERR(info->i2c_cmd_dev)) {
		 VTE("create device fts_i2c failed!");
		 class_destroy(info->fts_cmd_class);
		 return PTR_ERR(info->i2c_cmd_dev);
	 }

	 ret = sysfs_create_group(&info->i2c_cmd_dev->kobj, &i2c_cmd_attr_group);
	 if (ret) {
		 VTE("failed to create sysfs group");
		 device_destroy(info->fts_cmd_class, DCHIP_ID_0);
		 class_destroy(info->fts_cmd_class);
		 return ret;
	 }

	 return 0;
 }


void ftm4_gui_deinit(struct fts_ts_info *info)
{
	sysfs_remove_group(&info->i2c_cmd_dev->kobj, &i2c_cmd_attr_group);
	device_destroy(info->fts_cmd_class, DCHIP_ID_0);
	class_destroy(info->fts_cmd_class);
}

 #endif
