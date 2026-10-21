// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/kernel.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_mux.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/usb/typec.h>
#include <linux/soc/qcom/fsa4480-i2c.h>
#include <linux/iio/consumer.h>
#include <linux/qti-regmap-debugfs.h>
#include <linux/usb/lenovo_ucsi.h>

#define FSA4480_I2C_NAME	"fsa4480-driver"

#define FSA4480_SWITCH_SETTINGS 0x04
#define FSA4480_SWITCH_CONTROL  0x05
#define FSA4480_SWITCH_STATUS1  0x07
#define FSA4480_SLOW_L          0x08
#define FSA4480_SLOW_R          0x09
#define FSA4480_SLOW_MIC        0x0A
#define FSA4480_SLOW_SENSE      0x0B
#define FSA4480_SLOW_GND        0x0C
#define FSA4480_DELAY_L_R       0x0D
#define FSA4480_DELAY_L_MIC     0x0E
#define FSA4480_DELAY_L_SENSE   0x0F
#define FSA4480_DELAY_L_AGND    0x10
#define FSA4480_RESET           0x1E


DEFINE_MUTEX(fsa4480_state_lock);
EXPORT_SYMBOL_GPL(fsa4480_state_lock);
DECLARE_RWSEM(fsa4480_api_lock);
EXPORT_SYMBOL_GPL(fsa4480_api_lock);

bool fsa4480_switch_C1_enable;
EXPORT_SYMBOL(fsa4480_switch_C1_enable);
unsigned int audio_4480_headset_ref;
EXPORT_SYMBOL(audio_4480_headset_ref);
struct fsa4480_priv *audio_4480_priv1;
EXPORT_SYMBOL(audio_4480_priv1);
struct fsa4480_sub_priv *audio_4480_priv2;
EXPORT_SYMBOL(audio_4480_priv2);

struct fsa4480_reg_val {
	u16 reg;
	u8 val;
};

static const struct regmap_config fsa4480_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FSA4480_RESET,
};

static const struct fsa4480_reg_val fsa_reg_i2c_defaults[] = {
	{FSA4480_SLOW_L, 0x00},
	{FSA4480_SLOW_R, 0x00},
	{FSA4480_SLOW_MIC, 0x00},
	{FSA4480_SLOW_SENSE, 0x00},
	{FSA4480_SLOW_GND, 0x00},
	{FSA4480_DELAY_L_R, 0x00},
	{FSA4480_DELAY_L_MIC, 0x00},
	{FSA4480_DELAY_L_SENSE, 0x00},
	{FSA4480_DELAY_L_AGND, 0x09},
	{FSA4480_SWITCH_SETTINGS, 0x98},
};

static void fsa4480_usbc_update_settings(struct fsa4480_priv *fsa_priv,
		u32 switch_control, u32 switch_enable)
{
	u32 prev_control, prev_enable;

	if (!fsa_priv->regmap) {
		dev_err(fsa_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, &prev_control);
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, &prev_enable);
	if (prev_control == switch_control && prev_enable == switch_enable) {
		dev_dbg(fsa_priv->dev, "%s: settings unchanged\n", __func__);
		return;
	}

	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x80);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, switch_control);
	/* FSA4480 chip hardware requirement */
	usleep_range(50, 55);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, switch_enable);
}

static int fsa4480_usbc_event_changed_psupply(struct fsa4480_priv *fsa_priv,
				      unsigned long evt, void *ptr)
{
	struct device *dev = NULL;

	if (!fsa_priv)
		return -EINVAL;

	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;
	dev_dbg(dev, "%s: queueing usbc_analog_work\n",
		__func__);
	pm_stay_awake(fsa_priv->dev);
	queue_work(system_freezable_wq, &fsa_priv->usbc_analog_work);

	return 0;
}

static int fsa4480_usbc_event_changed(struct notifier_block *nb_ptr,
				      unsigned long evt, void *ptr)
{
	struct fsa4480_priv *fsa_priv =
			container_of(nb_ptr, struct fsa4480_priv, nb);
	struct device *dev;

	if (!fsa_priv)
		return -EINVAL;

	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;
	return fsa4480_usbc_event_changed_psupply(fsa_priv, evt, ptr);
}



static int fsa4480_usbc_mux_set(struct typec_mux_dev *mux,
				 struct typec_mux_state *state)
{
	struct fsa4480_priv *fsa_priv = typec_mux_get_drvdata(mux);
	enum typec_accessory acc;

	if (!fsa_priv)
		return -EINVAL;

	if (!fsa_priv->dev)
		return -EINVAL;

	mutex_lock(&fsa4480_state_lock);
	if (fsa_priv->removing)
		goto out;
	if (state->mode == TYPEC_MODE_AUDIO)
		acc = TYPEC_ACCESSORY_AUDIO;
	else if (state->mode == TYPEC_MODE_DEBUG)
		acc = TYPEC_ACCESSORY_DEBUG;
	else
		acc = TYPEC_ACCESSORY_NONE;

	dev_dbg(fsa_priv->dev, "%s: USB change event received, supply mode %d, usbc mode %d, expected %d\n",
			__func__, acc, fsa_priv->usbc_mode.counter,
			TYPEC_ACCESSORY_AUDIO);

	if (cc_port_num != 1)
		goto out;

	if (acc == TYPEC_ACCESSORY_DEBUG)
		goto out;

	if (atomic_read(&(fsa_priv->usbc_mode)) != acc) {
		if (acc == TYPEC_ACCESSORY_AUDIO)
			audio_4480_headset_ref++;
		else if (audio_4480_headset_ref > 0)
			audio_4480_headset_ref--;
		atomic_set(&(fsa_priv->usbc_mode), acc);
		dev_dbg(fsa_priv->dev, "%s: queueing usbc_analog_work\n", __func__);

		pm_stay_awake(fsa_priv->dev);
		queue_work(system_freezable_wq, &fsa_priv->usbc_analog_work);
	}

out:
	mutex_unlock(&fsa4480_state_lock);
	return 0;
}

static int fsa4480_usbc_analog_setup_switches_ucsi(
						struct fsa4480_priv *fsa_priv)
{
	struct fsa4480_sub_priv *peer;
	int rc = 0;
	int mode;
	struct device *dev;
	u32 reg6;
	u32 reg17;
	u32 value;
	int i = 0;
	u32 reg0;

	if (!fsa_priv)
		return -EINVAL;
	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	mutex_lock(&fsa4480_state_lock);
	if (fsa_priv->removing) {
		mutex_unlock(&fsa4480_state_lock);
		return -ENODEV;
	}
	peer = audio_4480_priv2;
	/* get latest mode again within locked context */
	mode = atomic_read(&(fsa_priv->usbc_mode));

	dev_dbg(dev, "%s: setting GPIOs active = %d\n",
		__func__, mode != TYPEC_ACCESSORY_NONE);

	switch (mode) {
	/* add all modes FSA should notify for in here */
	case TYPEC_ACCESSORY_AUDIO:
        /* activate switches */
        regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);
        regmap_write(fsa_priv->regmap, FSA4480_SLOW_L, 0x4f);
        regmap_read(fsa_priv->regmap, FSA4480_SLOW_L, &value);
        dev_dbg(fsa_priv->dev, "%s: reg[0x08]=0x%x\n",__func__, value);
        regmap_write(fsa_priv->regmap, FSA4480_SLOW_R, 0x4f);
        regmap_read(fsa_priv->regmap, FSA4480_SLOW_R, &value);
        dev_dbg(fsa_priv->dev, "%s: reg[0x09]=0x%x\n",__func__, value);
        regmap_write(fsa_priv->regmap, FSA4480_SLOW_MIC, 0x4f);
        regmap_read(fsa_priv->regmap, FSA4480_SLOW_MIC, &value);
        dev_dbg(fsa_priv->dev, "%s: reg[0x0a]=0x%x\n",__func__, value);
        regmap_write(fsa_priv->regmap, FSA4480_SLOW_SENSE, 0x4f);
        regmap_read(fsa_priv->regmap, FSA4480_SLOW_SENSE, &value);
        dev_dbg(fsa_priv->dev, "%s: reg[0x0b]=0x%x\n",__func__, value);
        regmap_write(fsa_priv->regmap, FSA4480_SLOW_GND, 0x4f);
        regmap_read(fsa_priv->regmap, FSA4480_SLOW_GND, &value);
        dev_dbg(fsa_priv->dev, "%s: reg[0x0c]=0x%x\n",__func__, value);

        regmap_write(fsa_priv->regmap, 0x04, 0x9f);
        dev_dbg(fsa_priv->dev, "%s: set reg[0x04] done.\n",__func__);
        regmap_read(fsa_priv->regmap, 0x04, &value);
        dev_dbg(fsa_priv->dev, "%s: reg[0x04]=0x%x\n",__func__, value);

        regmap_write(fsa_priv->regmap, 0x05, 0x00);
        dev_dbg(fsa_priv->dev, "%s: set reg[0x05] done.\n",__func__);
        regmap_read(fsa_priv->regmap, 0x05, &value);
        dev_dbg(fsa_priv->dev, "%s: reg[0x05]=0x%x\n",__func__, value);

        regmap_write(fsa_priv->regmap, 0x12, 0x0d);
        dev_dbg(fsa_priv->dev, "%s: set reg[0x12] done.\n",__func__);
        regmap_read(fsa_priv->regmap, 0x12, &value);
        dev_dbg(fsa_priv->dev, "%s: reg[0x12]=0x%x\n",__func__, value);

        for (i = 0; i < 10; i++) {
            msleep(5);
            if (value == 0x0c)
                break;
            regmap_read(fsa_priv->regmap, 0x12, &value);
            dev_dbg(fsa_priv->dev, "%s: reg[0x12]=0x%x\n",__func__, value);
        }

        msleep(1000);

        regmap_read(fsa_priv->regmap, 0x17, &reg17);
        dev_dbg(fsa_priv->dev, "%s: reg[0x17]=0x%x\n",__func__, reg17);
        regmap_read(fsa_priv->regmap, 0x06, &reg6);
        regmap_read(fsa_priv->regmap, 0x00, &reg0);
        if(reg17 == 0x01 ||reg6 == 0x05){
            regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);
            fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);
            msleep(5);
            if(reg0 == 0xf1){
                regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);
                regmap_write(fsa_priv->regmap, 0x1c, 0x0A);
                regmap_read(fsa_priv->regmap, 0x1c, &value);
                dev_dbg(fsa_priv->dev, "%s: reg[0x1c]=0x%x\n",__func__, value);
                regmap_write(fsa_priv->regmap, 0x12, 0x09);
                msleep(1000);
            }
        }
        regmap_read(fsa_priv->regmap, FSA4480_SLOW_MIC, &value);
        if (reg17 == 0x01 && reg6 == 0x00 && value == 0x01) {
            regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);
            fsa4480_usbc_update_settings(fsa_priv, 0x07, 0x9f);
            msleep(5);
        }
        if(reg17 == 0x02){
            regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x87);
            msleep(1);
            regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, 0x07);
            msleep(5);
            regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9f);
            msleep(5);
        }

		fsa4480_switch_C1_enable = true;

		/* notify call chain on event */
		mutex_unlock(&fsa4480_state_lock);
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
					     mode, NULL);
		mutex_lock(&fsa4480_state_lock);
		break;
	case TYPEC_ACCESSORY_NONE:
		fsa4480_switch_C1_enable = false;

		/* notify call chain on event */
		mutex_unlock(&fsa4480_state_lock);
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
				TYPEC_ACCESSORY_NONE, NULL);
		mutex_lock(&fsa4480_state_lock);

		/* deactivate switches */
	    regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
        dev_dbg(fsa_priv->dev, "%s: TYPEC_ACCESSORY_NONE",__func__);
		if (audio_4480_headset_ref > 0 && peer) {
			regmap_write(peer->regmap, FSA4480_RESET, 0x01);
		    regmap_write(peer->regmap, FSA4480_SLOW_L, 0x4f);
		    regmap_write(peer->regmap, FSA4480_SLOW_R, 0x4f);
		    regmap_write(peer->regmap, FSA4480_SLOW_MIC, 0x4f);
		    regmap_write(peer->regmap, FSA4480_SLOW_SENSE, 0x4f);
		    regmap_write(peer->regmap, FSA4480_SLOW_GND, 0x4f);
			regmap_write(peer->regmap, 0x04, 0x9f);
			regmap_write(peer->regmap, 0x05, 0x00);
			regmap_write(peer->regmap, 0x12, 0x0d);
		    dev_dbg(peer->dev, "%s: set reg[0x12] done.\n",__func__);
			regmap_read(peer->regmap, 0x12, &value);
			dev_dbg(peer->dev, "%s: reg[0x12]=0x%x\n",__func__, value);

			for (i = 0; i < 10; i++) {
                msleep(5);
                if (value == 0x0c)
                    break;
                regmap_read(peer->regmap, 0x12, &value);
                dev_dbg(peer->dev, "%s: reg[0x12]=0x%x\n",__func__, value);
            }
			msleep(1000);
			regmap_read(peer->regmap, 0x17, &reg17);
            dev_dbg(peer->dev, "%s: reg[0x17]=0x%x\n",__func__, reg17);
            regmap_read(peer->regmap, 0x06, &reg6);
            regmap_read(peer->regmap, 0x00, &reg0);
            if(reg17 == 0x01 ||reg6 == 0x05){
                regmap_write(peer->regmap, FSA4480_RESET, 0x01);
                regmap_write(peer->regmap, 0x04, 0x80);
			    regmap_write(peer->regmap, 0x05, 0x00);
			    usleep_range(50, 55);
			    regmap_write(peer->regmap, 0x04, 0x9F);
                msleep(5);
                if(reg0 == 0xf1){
                    regmap_write(peer->regmap, FSA4480_RESET, 0x01);
                    regmap_write(peer->regmap, 0x1c, 0x0A);
                    regmap_read(peer->regmap, 0x1c, &value);
                    dev_dbg(peer->dev, "%s: reg[0x1c]=0x%x\n",__func__, value);
                    regmap_write(peer->regmap, 0x12, 0x09);
                    msleep(1000);
                }
            }
			if(reg17 == 0x02){
		    regmap_write(peer->regmap, FSA4480_SWITCH_SETTINGS, 0x87);
			msleep(1);
			regmap_write(peer->regmap, FSA4480_SWITCH_CONTROL, 0x07);
			msleep(5);
			regmap_write(peer->regmap, FSA4480_SWITCH_SETTINGS, 0x9f);
			msleep(5);
            }

			mutex_unlock(&fsa4480_state_lock);

			blocking_notifier_call_chain(&peer->fsa4480_sub_notifier,
				TYPEC_ACCESSORY_AUDIO, NULL);

			mutex_lock(&fsa4480_state_lock);
		}
		break;
	default:
		/* ignore other usb connection modes */
		break;
	}

	mutex_unlock(&fsa4480_state_lock);
	return rc;
}

static int fsa4480_usbc_analog_setup_switches(struct fsa4480_priv *fsa_priv)
{
	int rc = 0;
	int mode;
	union power_supply_propval iio_val;
	struct device *dev;

	if (!fsa_priv)
		return -EINVAL;
	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	if (!fsa_priv->use_powersupply)
		return fsa4480_usbc_analog_setup_switches_ucsi(fsa_priv);

	if (fsa_priv->iio_ch) {
		rc = iio_read_channel_processed(fsa_priv->iio_ch, &iio_val.intval);
		if (rc < 0) {
			dev_err(dev, "%s: Unable to read USB TYPEC_MODE: %d\n",
			__func__, rc);
			return rc;
		}
		atomic_set(&(fsa_priv->usbc_mode), iio_val.intval);
	}

	mutex_lock(&fsa4480_state_lock);
	if (fsa_priv->removing) {
		mutex_unlock(&fsa4480_state_lock);
		return -ENODEV;
	}
	/* get latest mode again within locked context */
	mode = atomic_read(&(fsa_priv->usbc_mode));

	dev_dbg(dev, "%s: setting GPIOs active = %d\n",
		__func__, mode != TYPEC_ACCESSORY_NONE);

	switch (mode) {
	/* add all modes FSA should notify for in here */
	case TYPEC_ACCESSORY_AUDIO:
		/* activate switches */
		fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);

		/* notify call chain on event */
		mutex_unlock(&fsa4480_state_lock);
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
					     mode, NULL);
		mutex_lock(&fsa4480_state_lock);
		break;
	case TYPEC_ACCESSORY_NONE:
		/* notify call chain on event */
		mutex_unlock(&fsa4480_state_lock);
		blocking_notifier_call_chain(&fsa_priv->fsa4480_notifier,
				TYPEC_ACCESSORY_NONE, NULL);
		mutex_lock(&fsa4480_state_lock);

		/* deactivate switches */
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		break;
	default:
		/* ignore other usb connection modes */
		break;
	}
	mutex_unlock(&fsa4480_state_lock);
	return rc;
}

/*
 * fsa4480_reg_notifier - register notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on success, or error code
 */
int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EPROBE_DEFER;
	down_read(&fsa4480_api_lock);
	mutex_lock(&fsa4480_state_lock);

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv || fsa_priv->removing) {
		rc = -EPROBE_DEFER;
		goto out;
	}

	mutex_unlock(&fsa4480_state_lock);
	rc = blocking_notifier_chain_register
				(&fsa_priv->fsa4480_notifier, nb);

	dev_dbg(fsa_priv->dev, "%s: registered notifier for %s\n",
		__func__, node->name);
	if (rc) {
		mutex_lock(&fsa4480_state_lock);
		goto out;
	}

	/*
	 * as part of the init sequence check if there is a connected
	 * USB C analog adapter
	 */
	if (fsa_priv->use_powersupply) {
	       /*
		* reset the cached usbc mode when power supply is used,
		* to bypass the event filtering logic.
		*/
		atomic_set(&(fsa_priv->usbc_mode), TYPEC_ACCESSORY_NONE);
		fsa4480_usbc_analog_setup_switches(fsa_priv);
	} else if (atomic_read(&(fsa_priv->usbc_mode)) == TYPEC_ACCESSORY_AUDIO) {
		dev_dbg(fsa_priv->dev, "%s: analog adapter already inserted\n",
			__func__);
		rc = fsa4480_usbc_analog_setup_switches(fsa_priv);
	}

	if (rc)
		blocking_notifier_chain_unregister(&fsa_priv->fsa4480_notifier, nb);
	mutex_lock(&fsa4480_state_lock);
out:
	mutex_unlock(&fsa4480_state_lock);
	up_read(&fsa4480_api_lock);
	put_device(&client->dev);
	return rc;
}
EXPORT_SYMBOL_GPL(fsa4480_reg_notifier);

/*
 * fsa4480_unreg_notifier - unregister notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on pass, or error code
 */
int fsa4480_unreg_notifier(struct notifier_block *nb,
			     struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EPROBE_DEFER;
	down_read(&fsa4480_api_lock);
	mutex_lock(&fsa4480_state_lock);

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv) {
		rc = -EPROBE_DEFER;
		goto out;
	}

	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	mutex_unlock(&fsa4480_state_lock);
	rc = blocking_notifier_chain_unregister
					(&fsa_priv->fsa4480_notifier, nb);
	mutex_lock(&fsa4480_state_lock);
out:
	mutex_unlock(&fsa4480_state_lock);
	up_read(&fsa4480_api_lock);
	put_device(&client->dev);
	return rc;
}
EXPORT_SYMBOL_GPL(fsa4480_unreg_notifier);

static int fsa4480_validate_display_port_settings(struct fsa4480_priv *fsa_priv)
{
	u32 switch_status = 0;

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status);

	if ((switch_status != 0x23) && (switch_status != 0x1C)) {
		pr_err("AUX SBU1/2 switch status is invalid = %u\n",
				switch_status);
		return -EIO;
	}

	return 0;
}
/*
 * fsa4480_switch_event - configure FSA switch position based on event
 *
 * @node - phandle node to fsa4480 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int fsa4480_switch_event(struct device_node *node,
			 enum fsa_function event)
{
	int rc = 0;
	int switch_control = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EPROBE_DEFER;
	mutex_lock(&fsa4480_state_lock);

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv || fsa_priv->removing) {
		rc = -EPROBE_DEFER;
		goto out;
	}
	if (!fsa_priv->regmap) {
		rc = -EINVAL;
		goto out;
	}

	switch (event) {
	case FSA_MIC_GND_SWAP:
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL,
				&switch_control);
		if ((switch_control & 0x07) == 0x07)
			switch_control = 0x0;
		else
			switch_control = 0x7;
		fsa4480_usbc_update_settings(fsa_priv, switch_control, 0x9F);
		break;
	case FSA_USBC_ORIENTATION_CC1:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0xF8);
		rc = fsa4480_validate_display_port_settings(fsa_priv);
		break;
	case FSA_USBC_ORIENTATION_CC2:
		fsa4480_usbc_update_settings(fsa_priv, 0x78, 0xF8);
		rc = fsa4480_validate_display_port_settings(fsa_priv);
		break;
	case FSA_USBC_DISPLAYPORT_DISCONNECTED:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		break;
	default:
		break;
	}

out:
	mutex_unlock(&fsa4480_state_lock);
	put_device(&client->dev);
	return rc;
}
EXPORT_SYMBOL_GPL(fsa4480_switch_event);

static void fsa4480_usbc_analog_work_fn(struct work_struct *work)
{
	struct fsa4480_priv *fsa_priv =
		container_of(work, struct fsa4480_priv, usbc_analog_work);

	if (!fsa_priv) {
		pr_err("%s: fsa container invalid\n", __func__);
		return;
	}
	down_read(&fsa4480_api_lock);
	fsa4480_usbc_analog_setup_switches(fsa_priv);
	up_read(&fsa4480_api_lock);
	pm_relax(fsa_priv->dev);
}

static void fsa4480_update_reg_defaults(struct regmap *regmap)
{
	u8 i;

	for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_defaults); i++)
		regmap_write(regmap, fsa_reg_i2c_defaults[i].reg,
				   fsa_reg_i2c_defaults[i].val);
}

static int fsa4480_probe(struct i2c_client *i2c)
{
	struct typec_mux_desc mux_desc = { };
	struct fsa4480_priv *fsa_priv;
	u32 use_powersupply = 0;
	int rc = 0;

	fsa_priv = devm_kzalloc(&i2c->dev, sizeof(*fsa_priv),
				GFP_KERNEL);
	if (!fsa_priv)
		return -ENOMEM;

	memset(fsa_priv, 0, sizeof(struct fsa4480_priv));
	fsa_priv->dev = &i2c->dev;
	mutex_init(&fsa_priv->notification_lock);
	INIT_WORK(&fsa_priv->usbc_analog_work, fsa4480_usbc_analog_work_fn);
	BLOCKING_INIT_NOTIFIER_HEAD(&fsa_priv->fsa4480_notifier);


	fsa_priv->regmap = devm_regmap_init_i2c(i2c, &fsa4480_regmap_config);
	if (IS_ERR_OR_NULL(fsa_priv->regmap)) {
		dev_err(fsa_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!fsa_priv->regmap) {
			rc = -EINVAL;
			goto err_data;
		}
		rc = PTR_ERR(fsa_priv->regmap);
		goto err_data;
	}

	fsa4480_update_reg_defaults(fsa_priv->regmap);
	devm_regmap_qti_debugfs_register(fsa_priv->dev, fsa_priv->regmap);

	// On legacy targets that use PMIC charger path,check use_powersupply prop
	rc = of_property_read_u32(fsa_priv->dev->of_node,
			"qcom,use-power-supply", &use_powersupply);
	if (rc || use_powersupply == 0) {
		fsa_priv->use_powersupply = 0;
		mux_desc.drvdata = fsa_priv;
		mux_desc.fwnode = dev_fwnode(fsa_priv->dev);
		mux_desc.set = fsa4480_usbc_mux_set;

		fsa_priv->mux = typec_mux_register(fsa_priv->dev, &mux_desc);
		if (IS_ERR(fsa_priv->mux)) {
			rc = dev_err_probe(fsa_priv->dev, PTR_ERR(fsa_priv->mux),
				"failed to register typec mux\n");
			goto err_data;
		}
	} else {
		fsa_priv->use_powersupply = 1;
		fsa_priv->nb.notifier_call = fsa4480_usbc_event_changed;
		fsa_priv->nb.priority = 0;
		fsa_priv->usb_psy = power_supply_get_by_name("usb");
		if (!fsa_priv->usb_psy) {
			rc = -EPROBE_DEFER;
			dev_err(fsa_priv->dev,
				"%s: could not get USB psy info: %d\n",
				__func__, rc);
			goto err_data;
		}

		fsa_priv->iio_ch = iio_channel_get(fsa_priv->dev, "typec_mode");
		if (IS_ERR(fsa_priv->iio_ch)) {
			rc = PTR_ERR(fsa_priv->iio_ch);
			dev_err(fsa_priv->dev,
				"%s: iio_channel_get failed for typec_mode\n",
				__func__);
			goto err_supply;
		}
		rc = power_supply_reg_notifier(&fsa_priv->nb);
		if (rc) {
			dev_err(fsa_priv->dev,
				"%s: power supply reg failed: %d\n",
			__func__, rc);
			goto err_supply;
		}
	}

	mutex_lock(&fsa4480_state_lock);
	i2c_set_clientdata(i2c, fsa_priv);
	mutex_unlock(&fsa4480_state_lock);
	mutex_lock(&fsa4480_state_lock);
	audio_4480_priv1 = fsa_priv;
	mutex_unlock(&fsa4480_state_lock);



	return 0;

err_supply:
	power_supply_put(fsa_priv->usb_psy);
err_data:
	return rc;
}

static void fsa4480_remove(struct i2c_client *i2c)
{
	struct fsa4480_priv *fsa_priv = i2c_get_clientdata(i2c);

	if (!fsa_priv)
		return;
	mutex_lock(&fsa4480_state_lock);
	fsa_priv->removing = true;
	audio_4480_priv1 = NULL;
	if (atomic_read(&fsa_priv->usbc_mode) == TYPEC_ACCESSORY_AUDIO &&
	    audio_4480_headset_ref)
		audio_4480_headset_ref--;
	mutex_unlock(&fsa4480_state_lock);
	if (fsa_priv->use_powersupply)
		power_supply_unreg_notifier(&fsa_priv->nb);
	else
		typec_mux_unregister(fsa_priv->mux);
	cancel_work_sync(&fsa_priv->usbc_analog_work);
	pm_relax(fsa_priv->dev);
	down_write(&fsa4480_api_lock);
	mutex_lock(&fsa4480_state_lock);
	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	i2c_set_clientdata(i2c, NULL);
	mutex_unlock(&fsa4480_state_lock);
	up_write(&fsa4480_api_lock);
	if (fsa_priv->use_powersupply) {
		iio_channel_release(fsa_priv->iio_ch);
		power_supply_put(fsa_priv->usb_psy);
	}
	mutex_destroy(&fsa_priv->notification_lock);
}

static const struct of_device_id fsa4480_i2c_dt_match[] = {
	{
		.compatible = "qcom,fsa4480-i2c",
	},
	{}
};

static struct i2c_driver fsa4480_i2c_driver = {
	.driver = {
		.name = FSA4480_I2C_NAME,
		.of_match_table = fsa4480_i2c_dt_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = fsa4480_probe,
	.remove = fsa4480_remove,
};

static int __init fsa4480_init(void)
{
	int rc;

	rc = i2c_add_driver(&fsa4480_i2c_driver);
	if (rc)
		pr_err("fsa4480: Failed to register I2C driver: %d\n", rc);

	return rc;
}
module_init(fsa4480_init);

static void __exit fsa4480_exit(void)
{
	i2c_del_driver(&fsa4480_i2c_driver);
}
module_exit(fsa4480_exit);

MODULE_DESCRIPTION("FSA4480 I2C driver");
MODULE_LICENSE("GPL");
