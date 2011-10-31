/*
 * The industrial I/O input client driver
 *
 * Copyright (c) 2011 Jonathan Cameron
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/**
 * iio_input_channel_data - description of the channel for input subsystem
 * @code:	Absolute axis.
 * @min:	Minimum value.
 * @max:	Maximum value.
 * @fuzz:	Used to filter noise from the event stream.
 * @flat:	Values within this value will be discarded by joydev
 *		and reported as 0 instead.
 */
struct iio_input_channel_data {
	unsigned int code;
	int min, max, fuzz, flat;
};
