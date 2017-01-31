/*
 * Qualcomm PM8xxx PMIC XOADC driver
 *
 * These ADCs are known as HK/XO (house keeping / chrystal oscillator)
 * "XO" in "XOADC" means Chrystal Oscillator. It's a bunch of
 * specific-purpose and general purpose ADC converters and channels.
 *
 * Copyright (C) 2016 Linaro Ltd.
 * Author: Linus Walleij <linus.walleij@linaro.org>
 */

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>

#include "qcom-vadc-common.h"

/*
 * Definitions for the "user processor" registers lifted from the v3.4
 * Qualcomm tree. Their kernel has two out-of-tree drivers for the ADC:
 * drivers/misc/pmic8058-xoadc.c
 * drivers/hwmon/pm8xxx-adc.c
 * None of them contain any complete register specification, so this is
 * a best effort of combining the information.
 */

/* These appear to be "battery monitor" registers */
#define ADC_ARB_BTM_CNTRL1			0x17e
#define ADC_ARB_BTM_CNTRL1_EN_BTM		BIT(0)
#define ADC_ARB_BTM_CNTRL1_SEL_OP_MODE		BIT(1)
#define ADC_ARB_BTM_CNTRL1_MEAS_INTERVAL1	BIT(2)
#define ADC_ARB_BTM_CNTRL1_MEAS_INTERVAL2	BIT(3)
#define ADC_ARB_BTM_CNTRL1_MEAS_INTERVAL3	BIT(4)
#define ADC_ARB_BTM_CNTRL1_MEAS_INTERVAL4	BIT(5)
#define ADC_ARB_BTM_CNTRL1_EOC			BIT(6)
#define ADC_ARB_BTM_CNTRL1_REQ			BIT(7)

#define ADC_ARB_BTM_AMUX_CNTRL			0x17f
#define ADC_ARB_BTM_ANA_PARAM			0x180
#define ADC_ARB_BTM_DIG_PARAM			0x181
#define ADC_ARB_BTM_RSV				0x182
#define ADC_ARB_BTM_DATA1			0x183
#define ADC_ARB_BTM_DATA0			0x184
#define ADC_ARB_BTM_BAT_COOL_THR1		0x185
#define ADC_ARB_BTM_BAT_COOL_THR0		0x186
#define ADC_ARB_BTM_BAT_WARM_THR1		0x187
#define ADC_ARB_BTM_BAT_WARM_THR0		0x188
#define ADC_ARB_BTM_CNTRL2			0x18c

/* Proper ADC registers */

#define ADC_ARB_USRP_CNTRL			0x197
#define ADC_ARB_USRP_CNTRL_EN_ARB		BIT(0)
#define ADC_ARB_USRP_CNTRL_RSV1			BIT(1)
#define ADC_ARB_USRP_CNTRL_RSV2			BIT(2)
#define ADC_ARB_USRP_CNTRL_RSV3			BIT(3)
#define ADC_ARB_USRP_CNTRL_RSV4			BIT(4)
#define ADC_ARB_USRP_CNTRL_RSV5			BIT(5)
#define ADC_ARB_USRP_CNTRL_EOC			BIT(6)
#define ADC_ARB_USRP_CNTRL_REQ			BIT(7)

#define ADC_ARB_USRP_AMUX_CNTRL			0x198
#define ADC_ARB_USRP_AMUX_CNTRL_RSV0		BIT(0)
#define ADC_ARB_USRP_AMUX_CNTRL_RSV1		BIT(1)
#define ADC_ARB_USRP_AMUX_CNTRL_PREMUX0		BIT(2)
#define ADC_ARB_USRP_AMUX_CNTRL_PREMUX1		BIT(3)
#define ADC_ARB_USRP_AMUX_CNTRL_SEL0		BIT(4)
#define ADC_ARB_USRP_AMUX_CNTRL_SEL1		BIT(5)
#define ADC_ARB_USRP_AMUX_CNTRL_SEL2		BIT(6)
#define ADC_ARB_USRP_AMUX_CNTRL_SEL3		BIT(7)
#define ADC_AMUX_PREMUX_SHIFT			2
#define ADC_AMUX_SEL_SHIFT			4

/* We know very little about the bits in this register */
#define ADC_ARB_USRP_ANA_PARAM			0x199
#define ADC_ARB_USRP_ANA_PARAM_DIS		0xFE
#define ADC_ARB_USRP_ANA_PARAM_EN		0xFF

#define ADC_ARB_USRP_DIG_PARAM			0x19A
#define ADC_ARB_USRP_DIG_PARAM_SEL_SHIFT0	BIT(0)
#define ADC_ARB_USRP_DIG_PARAM_SEL_SHIFT1	BIT(1)
#define ADC_ARB_USRP_DIG_PARAM_CLK_RATE0	BIT(2)
#define ADC_ARB_USRP_DIG_PARAM_CLK_RATE1	BIT(3)
#define ADC_ARB_USRP_DIG_PARAM_EOC		BIT(4)
/*
 * On a later ADC the decimation factors are defined as
 * 00 = 512, 01 = 1024, 10 = 2048, 11 = 4096 so assume this
 * holds also for this older XOADC.
 */
#define ADC_ARB_USRP_DIG_PARAM_DEC_RATE0	BIT(5)
#define ADC_ARB_USRP_DIG_PARAM_DEC_RATE1	BIT(6)
#define ADC_ARB_USRP_DIG_PARAM_EN		BIT(7)
#define ADC_DIG_PARAM_DEC_SHIFT			5

#define ADC_ARB_USRP_RSV			0x19B
#define ADC_ARB_USRP_RSV_RST			BIT(0)
#define ADC_ARB_USRP_RSV_DTEST0			BIT(1)
#define ADC_ARB_USRP_RSV_DTEST1			BIT(2)
#define ADC_ARB_USRP_RSV_OP			BIT(3)
#define ADC_ARB_USRP_RSV_IP_SEL0		BIT(4)
#define ADC_ARB_USRP_RSV_IP_SEL1		BIT(5)
#define ADC_ARB_USRP_RSV_IP_SEL2		BIT(6)
#define ADC_ARB_USRP_RSV_TRM			BIT(7)
#define ADC_RSV_IP_SEL_SHIFT			4

#define ADC_ARB_USRP_DATA0			0x19D
#define ADC_ARB_USRP_DATA1			0x19C

/**
 * Physical channels, the vendor tree call these "channel path type"
 * for some reason, they have pretty much hardwired characteristics.
 * MPP = Multi-Purpose Pin. Channel info:
 *
 * @PM8XXX_CHANNEL_VCOIN: Coincell backup power source
 * @PM8XXX_CHANNEL_VBAT: Battery voltage
 * @PM8XXX_CHANNEL_DCIN: Charger voltage
 * @PM8XXX_CHANNEL_ICHG: Charger current monitor
 * @PM8XXX_CHANNEL_VPH_PWR: Main system power VPH
 * @PM8XXX_CHANNEL_MPP5: general purpose, used for headset detection on some
 * set-ups
 * @PM8XXX_CHANNEL_MPP6: general purpose, used for battery temperature on some
 * set-ups
 * @PM8XXX_CHANNEL_MPP7: general purpose, used for system temperature on some
 * set-ups
 * @PM8XXX_CHANNEL_MPP8: general purpose, used for battery ID detection on
 * some set-ups
 * @PM8XXX_CHANNEL_MPP9: general purpose, used for external charger current on
 * some set-ups
 * @PM8XXX_CHANNEL_USB_VBUS: USB bus voltage, used for USB charging
 * @PM8XXX_CHANNEL_DIE_TEMP: PMIC die temperature
 * @PM8XXX_CHANNEL_INTERNAL: 625mV reference channel
 * @PM8XXX_CHANNEL_125V: 1250mV reference channel
 * @PM8XXX_CHANNEL_INTERNAL_2: 325mV reference channel
 * @PM8XXX_CHANNEL_MUXOFF: channel to reduce input load on mux, apparently also
 * measures XO temperature
 */
#define PM8XXX_CHANNEL_VCOIN		0x0
#define PM8XXX_CHANNEL_VBAT		0x1
#define PM8XXX_CHANNEL_DCIN		0x2
#define PM8XXX_CHANNEL_ICHG		0x3
#define PM8XXX_CHANNEL_VPH_PWR		0x4
#define PM8XXX_CHANNEL_MPP5		0x5
#define PM8XXX_CHANNEL_MPP6		0x6
#define PM8XXX_CHANNEL_MPP7		0x7
#define PM8XXX_CHANNEL_MPP8		0x8
#define PM8XXX_CHANNEL_MPP9		0x9
#define PM8XXX_CHANNEL_USB_VBUS		0xa
#define PM8XXX_CHANNEL_DIE_TEMP		0xb
#define PM8XXX_CHANNEL_INTERNAL		0xc
#define PM8XXX_CHANNEL_125V		0xd
#define PM8XXX_CHANNEL_INTERNAL_2	0xe
#define PM8XXX_CHANNEL_MUXOFF		0xf

#define XOADC_CHANNELS		16 /* 4 bits */

/* MPP = Multi-Purpose Pins, premux scaling */
#define MPP_SCALE_0 0x0 /* No scaling on the signal */
#define MPP_SCALE_1 0x1 /* Unity scaling selected by the user */
#define MPP_SCALE_1_DIV3 0x2 /* 1/3 prescaler on the input from MPP */

/* Defines reference voltage for the XOADC */
#define AMUX_RSV0 0x0 /* XO_IN/XOADC_GND, special selection to read XO temp */
#define AMUX_RSV1 0x1 /* PMIC_IN/XOADC_GND */
#define AMUX_RSV2 0x2 /* PMIC_IN/BMS_CSP */
#define AMUX_RSV3 0x3 /* not used */
#define AMUX_RSV4 0x4 /* XOADC_GND/XOADC_GND */
#define AMUX_RSV5 0x5 /* XOADC_VREF/XOADC_GND */
#define XOADC_RSV_MAX 5 /* 3 bits 0..7, 3 and 6,7 are invalid */

/*
 * The different hardware channel names, hardware characteristics
 * and sensible default settings.
 * @datasheet_name: the hardwarename of this channel
 * @prescale: the channels have hard-coded prescale ratios defined
 * by the hardware, this tells us what it is
 * @type: corresponding IIO channel type, usually IIO_VOLTAGE or
 * IIO_TEMP
 * @scale_fn_type: the liner interpolation etc to convert the
 * ADC code to the value that IIO expects, in uV or millicelsius
 * etc. This scale function can be pretty elaborate if different
 * thermistors are connected or other hardware characteristics are
 * deployed.
 * @amux_ip_rsv: ratiometric scale value used by the analig muxer: this
 * selects the reference voltage for ratiometric scaling
 */
struct xoadc_channel {
	const char *datasheet_name;
	const struct vadc_prescale_ratio prescale;
	enum iio_chan_type type;
	enum vadc_scale_fn_type scale_fn_type;
	u8 amux_ip_rsv:3;
};

#define XOADC_CHAN(_dname, _type, _prenum, _preden, _scale, _amip) 	\
	[PM8XXX_CHANNEL_##_dname] = {					\
		.datasheet_name = __stringify(_dname),			\
		.prescale = { .num = _prenum, .den = _preden },		\
		.type = _type,						\
		.scale_fn_type = _scale,				\
		.amux_ip_rsv = _amip,					\
	}								\

/*
 * Taken from arch/arm/mach-msm/board-9615.c in the vendor tree:
 * TODO: incomplete, needs testing.
 */
static const struct xoadc_channel pm8018_xoadc_channels[] = {
	XOADC_CHAN(VCOIN, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(VBAT, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(VPH_PWR, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(DIE_TEMP, IIO_TEMP, 1, 1, SCALE_PMIC_THERM, AMUX_RSV1),
	/* Used for battery ID or battery temperature */
	XOADC_CHAN(MPP8, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV2),
	XOADC_CHAN(INTERNAL, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(125V, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MUXOFF, IIO_TEMP, 1, 1, SCALE_XOTHERM, AMUX_RSV0),
};

/*
 * Taken from arch/arm/mach-msm/board-8930-pmic.c in the vendor tree:
 * TODO: needs testing.
 */
static const struct xoadc_channel pm8038_xoadc_channels[] = {
	XOADC_CHAN(VCOIN, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(VBAT, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(DCIN, IIO_VOLTAGE, 1, 6, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(ICHG, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(VPH_PWR, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MPP5, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MPP6, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MPP7, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	/* MPP8 used for battery temperature in most cases */
	XOADC_CHAN(MPP8, IIO_TEMP, 1, 1, SCALE_THERM_100K_PULLUP, AMUX_RSV2),
	XOADC_CHAN(MPP9, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(USB_VBUS, IIO_VOLTAGE, 1, 4, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(DIE_TEMP, IIO_TEMP, 1, 1, SCALE_PMIC_THERM, AMUX_RSV1),
	XOADC_CHAN(INTERNAL, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(125V, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(INTERNAL_2, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MUXOFF, IIO_TEMP, 1, 1, SCALE_XOTHERM, AMUX_RSV0),
};

/*
 * This was created by cross-referencing the vendor tree
 * arch/arm/mach-msm/board-msm8x60.c msm_adc_channels_data[]
 * with the "channel types" (first field) to find the right
 * configuration for these channels on an MSM8x60 i.e. PM8058
 * setup.
 */
static const struct xoadc_channel pm8058_xoadc_channels[] = {
	XOADC_CHAN(VCOIN, IIO_VOLTAGE, 1, 2, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(VBAT, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(DCIN, IIO_VOLTAGE, 1, 10, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(ICHG, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(VPH_PWR, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MPP5, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MPP6, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MPP7, IIO_VOLTAGE, 1, 2, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MPP8, IIO_VOLTAGE, 1, 2, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MPP9, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(USB_VBUS, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(DIE_TEMP, IIO_TEMP, 1, 1, SCALE_PMIC_THERM, AMUX_RSV1),
	XOADC_CHAN(INTERNAL, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(125V, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(INTERNAL_2, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MUXOFF, IIO_TEMP, 1, 1, SCALE_XOTHERM, AMUX_RSV0),
};

/*
 * The PM8921 is similar, this comes from the vendor tree
 * board-flo-pmic.c (Nexus 7) and board-8064-pmic.c
 * Notably the prescalers are differing.
 */
static const struct xoadc_channel pm8921_xoadc_channels[] = {
	XOADC_CHAN(VCOIN, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(VBAT, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(DCIN, IIO_VOLTAGE, 1, 6, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(ICHG, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(VPH_PWR, IIO_VOLTAGE, 1, 3, SCALE_DEFAULT, AMUX_RSV1),
	/* MPP scaling is unknown, these scalings are a guess */
	XOADC_CHAN(MPP5, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MPP6, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MPP7, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	/* MPP8 is usually used for battery temperature */
	XOADC_CHAN(MPP8, IIO_TEMP, 1, 1, SCALE_THERM_100K_PULLUP, AMUX_RSV1),
	XOADC_CHAN(MPP9, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(USB_VBUS, IIO_VOLTAGE, 1, 4, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(DIE_TEMP, IIO_TEMP, 1, 1, SCALE_PMIC_THERM, AMUX_RSV1),
	XOADC_CHAN(INTERNAL, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(125V, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(INTERNAL_2, IIO_VOLTAGE, 1, 1, SCALE_DEFAULT, AMUX_RSV1),
	XOADC_CHAN(MUXOFF, IIO_TEMP, 1, 1, SCALE_XOTHERM, AMUX_RSV0),
};

/**
 * struct pm8xxx_chan_info - ADC channel information
 * @name: name of this channel
 * @calibration: whether to use absolute or ratiometric calibration
 * @scale_fn_type: scaling function type
 * @amux_channel: channel 0..15
 * @decimation: 0,1,2,3
 * @amux_mpp_channel: MPP channel 0..3, selects prescaling
 * @amux_ip_rsv: ratiometric scale value if using ratiometric
 * calibration: 0, 1, 2, 4, 5.
 */
struct pm8xxx_chan_info {
	const char *name;
	enum vadc_calibration calibration;
	const struct vadc_prescale_ratio *prescale;
	enum vadc_scale_fn_type scale_fn_type;
	u8 amux_channel:4;
	u8 decimation:2;
	u8 amux_mpp_channel:2;
	u8 amux_ip_rsv:3;
};

/**
 * struct pm8xxx_xoadc - state container for the XOADC
 * @dev: pointer to device
 * @map: regmap to access registers
 * @vref: reference voltage regulator
 * @hw_channels: the hardware channel defaults, information about the hardcoded
 * characteristics of the channels, and sensible default settings
 * @nchans: number of channels, configured by the device tree
 * @chans: the channel information per-channel, configured by the device tree
 * @iio_chans: IIO channel specifiers
 * @graph: linear calibration parameters for absolute and
 * ratiometric measurements
 * @complete: completion to indicate end of conversion
 * @lock: lock to restrict access to the hardware to one client at the time
 * @is_pm8058: the PM8058 has some bugs that need handling
 */
struct pm8xxx_xoadc {
	struct device *dev;
	struct regmap *map;
	struct regulator *vref;
	const struct xoadc_channel *hw_channels;
	unsigned int nchans;
	struct pm8xxx_chan_info *chans;
	struct iio_chan_spec *iio_chans;
	struct vadc_linear_graph graph[2];
	struct completion complete;
	struct mutex lock;
	bool is_pm8058;
};

static irqreturn_t pm8xxx_eoc_irq(int irq, void *d)
{
	struct iio_dev *indio_dev = d;
	struct pm8xxx_xoadc *adc = iio_priv(indio_dev);

	complete(&adc->complete);

	return IRQ_HANDLED;
}

static struct pm8xxx_chan_info *
pm8xxx_get_channel(struct pm8xxx_xoadc *adc, u8 chan)
{
	struct pm8xxx_chan_info *ch;
	int i;

	for (i = 0; i < adc->nchans; i++) {
		ch = &adc->chans[i];
		if (ch->amux_channel == chan)
			break;
	}
	if (i == adc->nchans)
		return NULL;

	return ch;
}

static int pm8xxx_read_channel_rsv(struct pm8xxx_xoadc *adc,
				   const struct pm8xxx_chan_info *ch,
				   u8 rsv, u16 *adc_code,
				   bool force_ratiometric)
{
	int ret;
	unsigned int val;
	u8 rsvmask, rsvval;
	u8 lsb, msb;

	dev_dbg(adc->dev, "read channel \"%s\", amux %d, mpp %d, rsv %d\n",
		ch->name, ch->amux_channel, ch->amux_mpp_channel, rsv);

	mutex_lock(&adc->lock);

	/* Mux in this channel */
	ret = regmap_write(adc->map, ADC_ARB_USRP_AMUX_CNTRL,
			   ch->amux_channel << ADC_AMUX_SEL_SHIFT |
			   ch->amux_mpp_channel << ADC_AMUX_PREMUX_SHIFT);
	if (ret)
		goto unlock;

	/* Set up ratiometric scale value, mask off all bits except these */
	rsvmask = (ADC_ARB_USRP_RSV_RST | ADC_ARB_USRP_RSV_DTEST0 |
		   ADC_ARB_USRP_RSV_DTEST1 | ADC_ARB_USRP_RSV_OP);
	if (adc->is_pm8058 && !force_ratiometric) {
		/*
		 * Apparently the PM8058 has some kind of bug which is
		 * reflected in the vendor tree drivers/misc/pmix8058-xoadc.c
		 * which just hardcodes the RSV selector to SEL1 (0x20) for
		 * most cases and SEL0 (0x10) for the MUXOFF channel only.
		 * If we force ratiometric (currently only done when attempting
		 * to do ratiometric calibration) this doesn't seem to work
		 * very well and I suspect ratiometric conversion is simply
		 * broken or not supported on the PM8058.
		 *
		 * Maybe IO_SEL2 doesn't exist on PM8058 and bits 4 & 5 select
		 * the mode alone.
		 *
		 * Some PM8058 register documentation would be nice to get
		 * this right.
		 */
		if (ch->amux_channel == PM8XXX_CHANNEL_MUXOFF)
			rsvval = ADC_ARB_USRP_RSV_IP_SEL0;
		else
			rsvval = ADC_ARB_USRP_RSV_IP_SEL1;
	} else {
		if (rsv == 0xff)
			rsvval = (ch->amux_ip_rsv << ADC_RSV_IP_SEL_SHIFT) |
				ADC_ARB_USRP_RSV_TRM;
		else
			rsvval = (rsv << ADC_RSV_IP_SEL_SHIFT) |
				ADC_ARB_USRP_RSV_TRM;
	}

	ret = regmap_update_bits(adc->map,
				 ADC_ARB_USRP_RSV,
				 ~rsvmask,
				 rsvval);
	if (ret)
		goto unlock;

	ret = regmap_write(adc->map, ADC_ARB_USRP_ANA_PARAM,
			   ADC_ARB_USRP_ANA_PARAM_DIS);
	if (ret)
		goto unlock;

	/* Decimation factor */
	ret = regmap_write(adc->map, ADC_ARB_USRP_DIG_PARAM,
			   ADC_ARB_USRP_DIG_PARAM_SEL_SHIFT0 |
			   ADC_ARB_USRP_DIG_PARAM_SEL_SHIFT1 |
			   ch->decimation << ADC_DIG_PARAM_DEC_SHIFT);
	if (ret)
		goto unlock;

	ret = regmap_write(adc->map, ADC_ARB_USRP_ANA_PARAM,
			   ADC_ARB_USRP_ANA_PARAM_EN);
	if (ret)
		goto unlock;

	/* Enable the arbiter, the Qualcomm code does it twice like this */
	ret = regmap_write(adc->map, ADC_ARB_USRP_CNTRL,
			   ADC_ARB_USRP_CNTRL_EN_ARB);
	if (ret)
		goto unlock;
	ret = regmap_write(adc->map, ADC_ARB_USRP_CNTRL,
			   ADC_ARB_USRP_CNTRL_EN_ARB);
	if (ret)
		goto unlock;


	/* Fire a request! */
	reinit_completion(&adc->complete);
	ret = regmap_write(adc->map, ADC_ARB_USRP_CNTRL,
			   ADC_ARB_USRP_CNTRL_EN_ARB |
			   ADC_ARB_USRP_CNTRL_REQ);
	if (ret)
		goto unlock;

	/* Next the interrupt occurs */
	ret = wait_for_completion_timeout(&adc->complete,
					  VADC_CONV_TIME_MAX_US);
	if (!ret) {
		dev_err(adc->dev, "conversion timed out\n");
		ret = -ETIMEDOUT;
		goto unlock;
	}

	ret = regmap_read(adc->map, ADC_ARB_USRP_DATA0, &val);
	if (ret)
		goto unlock;
	lsb = val;
	ret = regmap_read(adc->map, ADC_ARB_USRP_DATA1, &val);
	if (ret)
		goto unlock;
	msb = val;
	*adc_code = (msb << 8) | lsb;

	/* Turn off the ADC by setting the arbiter to 0 twice */
	ret = regmap_write(adc->map, ADC_ARB_USRP_CNTRL, 0);
	if (ret)
		goto unlock;
	ret = regmap_write(adc->map, ADC_ARB_USRP_CNTRL, 0);
	if (ret)
		goto unlock;

unlock:
	mutex_unlock(&adc->lock);
	return ret;
}

static int pm8xxx_read_channel(struct pm8xxx_xoadc *adc,
			       const struct pm8xxx_chan_info *ch,
			       u16 *adc_code)
{
	/*
	 * Normally we just use the ratiometric scale value (RSV) predefined
	 * for the channel, but during calibration we need to modify this
	 * so this wrapper is a helper hiding the more complex version.
	 */
	return pm8xxx_read_channel_rsv(adc, ch, 0xff, adc_code, false);
}

static int pm8xxx_calibrate_device(struct pm8xxx_xoadc *adc)
{
	const struct pm8xxx_chan_info *ch;
	u16 read_1250v;
	u16 read_0625v;
	u16 read_nomux_rsv5;
	u16 read_nomux_rsv4;
	int ret;

	adc->graph[VADC_CALIB_ABSOLUTE].dx = VADC_ABSOLUTE_RANGE_UV;
	adc->graph[VADC_CALIB_RATIOMETRIC].dx = VADC_RATIOMETRIC_RANGE;

	/* Common reference channel calibration */
	ch = pm8xxx_get_channel(adc, PM8XXX_CHANNEL_125V);
	if (!ch)
		return -ENODEV;
	ret = pm8xxx_read_channel(adc, ch, &read_1250v);
	if (ret) {
		dev_err(adc->dev, "could not read 1.25V reference channel\n");
		return -ENODEV;
	}
	ch = pm8xxx_get_channel(adc, PM8XXX_CHANNEL_INTERNAL);
	if (!ch)
		return -ENODEV;
	ret = pm8xxx_read_channel(adc, ch, &read_0625v);
	if (ret) {
		dev_err(adc->dev, "could not read 0.625V reference channel\n");
		return -ENODEV;
	}
	if (read_1250v == read_0625v) {
		dev_err(adc->dev, "read same ADC code for 1.25V and 0.625V\n");
		return -ENODEV;
	}

	adc->graph[VADC_CALIB_ABSOLUTE].dy = read_1250v - read_0625v;
	adc->graph[VADC_CALIB_ABSOLUTE].gnd = read_0625v;

	dev_info(adc->dev, "absolute calibration dx = %d uV, dy = %d units\n",
		 VADC_ABSOLUTE_RANGE_UV, adc->graph[VADC_CALIB_ABSOLUTE].dy);

	/* Ratiometric calibration */
	ch = pm8xxx_get_channel(adc, PM8XXX_CHANNEL_MUXOFF);
	if (!ch)
		return -ENODEV;
	ret = pm8xxx_read_channel_rsv(adc, ch, AMUX_RSV5,
				      &read_nomux_rsv5, true);
	if (ret) {
		dev_err(adc->dev, "could not read MUXOFF reference channel\n");
		return -ENODEV;
	}
	ret = pm8xxx_read_channel_rsv(adc, ch, AMUX_RSV4,
				      &read_nomux_rsv4, true);
	if (ret) {
		dev_err(adc->dev, "could not read MUXOFF reference channel\n");
		return -ENODEV;
	}
	adc->graph[VADC_CALIB_RATIOMETRIC].dy =
		read_nomux_rsv5 - read_nomux_rsv4;
	adc->graph[VADC_CALIB_RATIOMETRIC].gnd = read_nomux_rsv4;

	dev_info(adc->dev, "ratiometric calibration dx = %d, dy = %d units\n",
		 VADC_RATIOMETRIC_RANGE,
		 adc->graph[VADC_CALIB_RATIOMETRIC].dy);

	return 0;
}

static int pm8xxx_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct pm8xxx_xoadc *adc = iio_priv(indio_dev);
	const struct pm8xxx_chan_info *ch;
	u16 adc_code;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		ch = pm8xxx_get_channel(adc, chan->address);
		if (!ch) {
			dev_err(adc->dev, "no such channel %lu\n",
				chan->address);
			return -EINVAL;
		}
		ret = pm8xxx_read_channel(adc, ch, &adc_code);
		if (ret)
			return ret;

		ret = qcom_vadc_scale(ch->scale_fn_type,
				      &adc->graph[ch->calibration],
				      ch->prescale,
				      (ch->calibration == VADC_CALIB_ABSOLUTE),
				      adc_code, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		ch = pm8xxx_get_channel(adc, chan->address);
		if (!ch) {
			dev_err(adc->dev, "no such channel %lu\n",
				chan->address);
			return -EINVAL;
		}
		ret = pm8xxx_read_channel(adc, ch, &adc_code);
		if (ret)
			return ret;

		*val = (int)adc_code;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int pm8xxx_of_xlate(struct iio_dev *indio_dev,
			   const struct of_phandle_args *iiospec)
{
	struct pm8xxx_xoadc *adc = iio_priv(indio_dev);
	unsigned int i;

	for (i = 0; i < adc->nchans; i++)
		if (adc->iio_chans[i].channel == iiospec->args[0])
			return i;

	return -EINVAL;
}

static const struct iio_info pm8xxx_xoadc_info = {
	.driver_module = THIS_MODULE,
	.of_xlate = pm8xxx_of_xlate,
	.read_raw = pm8xxx_read_raw,
};

static int pm8xxx_xoadc_parse_channel(struct device *dev,
				      struct device_node *np,
				      const struct xoadc_channel *hw_channels,
				      struct iio_chan_spec *iio_chan,
				      struct pm8xxx_chan_info *ch)
{
	const char *name = np->name;
	u32 chan, rsv, dec;
	int ret;

	ret = of_property_read_u32(np, "reg", &chan);
	if (ret) {
		dev_err(dev, "invalid channel number %s\n", name);
		return ret;
	}
	if (chan >= XOADC_CHANNELS) {
		dev_err(dev, "%s too big channel number %d\n", name, chan);
		return -EINVAL;
	}

	/* Look up default channel settings */
	ch->amux_channel = chan;
	ch->name = name;
	ch->prescale = &hw_channels[chan].prescale;
	ch->scale_fn_type = hw_channels[chan].scale_fn_type;
	ch->amux_ip_rsv = hw_channels[chan].amux_ip_rsv;
	/* Everyone seems to use absolute calibration except in special cases */
	ch->calibration = VADC_CALIB_ABSOLUTE;
	/* Everyone seems to use default ("type 2") decimation */
	ch->decimation = VADC_DEF_DECIMATION;
	/* Everyone seems to use this 1-to-1 premux scaling channel */
	ch->amux_mpp_channel = MPP_SCALE_0;

	if (!of_property_read_u32(np, "qcom,ratiometric", &rsv)) {
		ch->calibration = VADC_CALIB_RATIOMETRIC;
		if (rsv > XOADC_RSV_MAX) {
			dev_err(dev, "%s too large RSV value %d\n", name, rsv);
			return -EINVAL;
		}
		if (rsv == AMUX_RSV3) {
			dev_err(dev, "%s invalid RSV value %d\n", name, rsv);
			return -EINVAL;
		}
	}

	/* Optional decimation, if omitted we use the default */
	ret = of_property_read_u32(np, "qcom,decimation", &dec);
	if (!ret) {
		ret = qcom_vadc_decimation_from_dt(dec);
		if (ret < 0) {
			dev_err(dev, "%s invalid decimation %d\n",
				name, dec);
			return ret;
		}
		ch->decimation = ret;
	}

	iio_chan->channel = ch->amux_channel;
	iio_chan->address = ch->amux_channel;
	iio_chan->datasheet_name = ch->name;
	iio_chan->type = hw_channels[chan].type;
	/* All channels are raw or processed */
	iio_chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_PROCESSED);
	iio_chan->indexed = 1;

	dev_dbg(dev, "channel %d \"%s\" ref voltage: %d, decimation %d "
		"prescale %d/%d, scale function %d\n",
		ch->amux_channel, ch->name, ch->amux_ip_rsv,
		ch->decimation,	ch->prescale->num, ch->prescale->den,
		ch->scale_fn_type);

	return 0;
}

static int pm8xxx_xoadc_parse_channels(struct pm8xxx_xoadc *adc,
				       struct device_node *np)
{
	struct device_node *child;
	struct pm8xxx_chan_info *ch;
	int ret;
	int i;

	adc->nchans = of_get_available_child_count(np);
	if (!adc->nchans) {
		dev_err(adc->dev, "no channel children\n");
		return -ENODEV;
	}
	dev_dbg(adc->dev, "found %d ADC channels\n", adc->nchans);

	adc->iio_chans = devm_kcalloc(adc->dev, adc->nchans,
				      sizeof(*adc->iio_chans), GFP_KERNEL);
	if (!adc->iio_chans)
		return -ENOMEM;

	adc->chans = devm_kcalloc(adc->dev, adc->nchans,
				  sizeof(*adc->chans), GFP_KERNEL);
	if (!adc->chans)
		return -ENOMEM;

	i = 0;
	for_each_available_child_of_node(np, child) {
		ch = &adc->chans[i];
		ret = pm8xxx_xoadc_parse_channel(adc->dev, child,
						 adc->hw_channels,
						 &adc->iio_chans[i],
						 ch);
		if (ret) {
			of_node_put(child);
			return ret;
		}
		i++;
	}

	/* Check for required channels */
	ch = pm8xxx_get_channel(adc, PM8XXX_CHANNEL_125V);
	if (!ch) {
		dev_err(adc->dev, "missing 1.25V reference channel\n");
		return -ENODEV;
	}
	ch = pm8xxx_get_channel(adc, PM8XXX_CHANNEL_INTERNAL);
	if (!ch) {
		dev_err(adc->dev, "missing 0.625V reference channel\n");
		return -ENODEV;
	}
	ch = pm8xxx_get_channel(adc, PM8XXX_CHANNEL_MUXOFF);
	if (!ch) {
		dev_err(adc->dev, "missing MUXOFF reference channel\n");
		return -ENODEV;
	}

	return 0;
}

static int pm8xxx_xoadc_probe(struct platform_device *pdev)
{
	struct pm8xxx_xoadc *adc;
	struct iio_dev *indio_dev;
	struct device_node *np = pdev->dev.of_node;
	struct regmap *map;
	struct device *dev = &pdev->dev;
	const char *compat;
	int ret;

	/* We need this compatible string */
	if (of_property_read_string(np, "compatible", &compat))
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;
	platform_set_drvdata(pdev, indio_dev);

	adc = iio_priv(indio_dev);
	adc->dev = dev;
	init_completion(&adc->complete);
	mutex_init(&adc->lock);

	/* Look up variant data */
	adc->hw_channels = of_device_get_match_data(dev);
	if (!adc->hw_channels) {
		dev_err(dev, "missing variant data\n");
		return -ENODEV;
	}

	/* This variant has special bugs that we need to deal with */
	if (of_device_is_compatible(np, "qcom,pm8058-adc"))
		adc->is_pm8058 = true;

	ret = pm8xxx_xoadc_parse_channels(adc, np);
	if (ret)
		return ret;

	map = dev_get_regmap(dev->parent, NULL);
	if (!map) {
		dev_err(dev, "parent regmap unavailable.\n");
		return -ENXIO;
	}
	adc->map = map;

	/* Bring up regulator */
	adc->vref = devm_regulator_get(dev, "xoadc-ref");
	if (IS_ERR(adc->vref)) {
		dev_err(dev, "failed to get XOADC VREF regulator\n");
		return PTR_ERR(adc->vref);
	}
	ret = regulator_enable(adc->vref);
	if (ret) {
		dev_err(dev, "failed to enable XOADC VREF regulator\n");
		return ret;
	}

	ret = devm_request_threaded_irq(dev, platform_get_irq(pdev, 0),
			pm8xxx_eoc_irq, NULL, 0, "pm8xxx-adc", indio_dev);
	if (ret) {
		dev_err(dev, "unable to request IRQ\n");
		goto out_disable_vref;
	}

	indio_dev->dev.parent = dev;
	indio_dev->dev.of_node = np;
	indio_dev->name = compat;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &pm8xxx_xoadc_info;
	indio_dev->channels = adc->iio_chans;
	indio_dev->num_channels = adc->nchans;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto out_disable_vref;

	ret = pm8xxx_calibrate_device(adc);
	if (ret)
		goto out_unreg_device;

	dev_info(dev, "PM8xxx XOADC driver enabled\n");

	return 0;

out_unreg_device:
	iio_device_unregister(indio_dev);
out_disable_vref:
	regulator_disable(adc->vref);

	return ret;
}

static int pm8xxx_xoadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct pm8xxx_xoadc *adc = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	regulator_disable(adc->vref);

	return 0;
}

static const struct of_device_id pm8xxx_xoadc_id_table[] = {
	{
		.compatible = "qcom,pm8018-adc",
		.data = &pm8018_xoadc_channels,
	},
	{
		.compatible = "qcom,pm8038-adc",
		.data = &pm8038_xoadc_channels,
	},
	{
		.compatible = "qcom,pm8058-adc",
		.data = &pm8058_xoadc_channels,
	},
	{
		.compatible = "qcom,pm8921-adc",
		.data = &pm8921_xoadc_channels,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, pm8xxx_xoadc_id_table);

static struct platform_driver pm8xxx_xoadc_driver = {
	.driver		= {
		.name	= "pm8xxx-adc",
		.of_match_table = pm8xxx_xoadc_id_table,
	},
	.probe		= pm8xxx_xoadc_probe,
	.remove		= pm8xxx_xoadc_remove,
};
module_platform_driver(pm8xxx_xoadc_driver);

MODULE_DESCRIPTION("PM8xxx XOADC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pm8xxx-xoadc");
