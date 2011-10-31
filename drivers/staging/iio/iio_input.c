/*
 * The industrial I/O input client driver
 *
 * Copyright (c) 2011 Jonathan Cameron
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include "buffer.h"
#include "consumer.h"
#include "iio_input.h"

struct iio_input_state {
	struct iio_cb_buffer *buff;
	struct input_dev *idev;
};

static int iio_channel_value(u8 *data,
			     const struct iio_chan_spec *chan,
			     int *val)
{
	int value;

	if (chan->scan_type.sign == 's') {
		switch (chan->scan_type.storagebits) {
		case 8:
			value = *(s8 *)(data);
			break;
		case 16:
			value = *(s16 *)(data);
			break;
		case 32:
			value = *(s32 *)(data);
			break;
		default:
			return -EINVAL;
		}
		value >>= chan->scan_type.shift;
		value &= (1 << chan->scan_type.realbits) - 1;
		value = (value << (sizeof(value)*8 - chan->scan_type.realbits))
			>> (sizeof(value)*8 - chan->scan_type.realbits);
	} else {
		switch (chan->scan_type.storagebits) {
		case 8:
			value = *(u8 *)(data);
			break;
		case 16:
			value = *(u16 *)(data);
			break;
		case 32:
			value = *(u32 *)(data);
			break;
		default:
			return -EINVAL;
		}
		value >>= chan->scan_type.shift;
		value &= (1 << chan->scan_type.realbits) - 1;
	}
	*val = value;

	return 0;
}

static int iio_input_store_to(u8 *data, void *private)
{
	struct iio_input_state *st = private;
	struct iio_channel *channel;
	struct iio_input_channel_data *input_data;
	int offset = 0;
	int value, ret;

	channel = iio_st_channel_cb_get_channels(st->buff);
	while (channel->indio_dev) {
		input_data = channel->data;
		offset = ALIGN(offset,
			       channel->channel->scan_type.storagebits/8);
		offset += channel->channel->scan_type.storagebits/8;
		ret = iio_channel_value(&data[offset],
					channel->channel,
					&value);
		if (ret < 0)
			return ret;
		input_report_abs(st->idev, input_data->code, value);
	}
	input_sync(st->idev);

	return 0;
}

static int __devinit iio_input_probe(struct platform_device *pdev)
{
	struct iio_input_state *st;
	int ret;
	struct iio_channel *channel;
	struct iio_input_channel_data *input_data;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;
	platform_set_drvdata(pdev, st);
	st->buff = iio_st_channel_get_all_cb(dev_name(&pdev->dev),
					     &iio_input_store_to,
					     st);
	if (IS_ERR(st->buff)) {
		ret = PTR_ERR(st->buff);
		goto error_free_state;
	}

	st->idev = input_allocate_device();
	if (!st->idev) {
		ret = -ENOMEM;
		goto error_channels_release_all;
	}

	__set_bit(EV_ABS, st->idev->evbit);
	channel = iio_st_channel_cb_get_channels(st->buff);
	while (channel->indio_dev) {
		input_data = channel->data;
		input_set_abs_params(st->idev, input_data->code,
				     input_data->min,
				     input_data->max,
				     input_data->fuzz,
				     input_data->flat);
	}

	ret = input_register_device(st->idev);
	if (ret < 0)
		goto error_free_idev;

	/* NORMALLY IN THE OPEN */
	iio_st_channel_start_all_cb(st->buff);

	return 0;
error_free_idev:
	input_free_device(st->idev);
error_channels_release_all:
	iio_st_channel_release_all_cb(st->buff);
error_free_state:
	kfree(st);
	return ret;
}

static int __devexit iio_input_remove(struct platform_device *pdev)
{
	struct iio_input_state *st = platform_get_drvdata(pdev);
	/* NORMALLY IN THE CLOSE */
	iio_st_channel_stop_all_cb(st->buff);
	input_unregister_device(st->idev);
	iio_st_channel_release_all_cb(st->buff);

	kfree(st);
	return 0;
}

static struct platform_driver iio_input_driver = {
	.driver = {
		.name = "iio_snoop",
		.owner = THIS_MODULE,
	},
	.probe = iio_input_probe,
	.remove = __devexit_p(iio_input_remove),
};

module_platform_driver(iio_input_driver);

MODULE_AUTHOR("Jonathan Cameron <jic23@cam.ac.uk>");
MODULE_DESCRIPTION("IIO input buffer driver");
MODULE_LICENSE("GPL v2");
