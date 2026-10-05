/* SPDX-License-Identifier: GPL-2.0-only */
/* TB323 OTG/fan controls. OEM whitelist forced detach is intentionally absent. */
static bool otg_state;
static char __chg_cmdline[1000];
#define otg_info(fn, fmt, ...) pr_info("[LENOVO_OTG][%s]" fmt, fn, ##__VA_ARGS__)
static char *chg_get_cmd(void)
{
	struct device_node *chosen;
	const char *bootargs;

	if (!__chg_cmdline[0]) {
		chosen = of_find_node_by_path("/chosen");
		if (chosen) {
			bootargs = of_get_property(chosen, "bootargs", NULL);
			if (bootargs)
				strscpy(__chg_cmdline, bootargs, sizeof(__chg_cmdline));
			of_node_put(chosen);
		}
	}
	return __chg_cmdline;
}

static void mt_get_otg_enable(void)
{
	char value[64] = { 0 };
	char *start = strstr(chg_get_cmd(), "usb_otg_dis="), *end;
	int len;

	if (!start)
		return;
	end = strchr(start, ' ');
	if (!end)
		end = start + strlen(start);
	start += 12;
	len = end - start;
	if (len <= 0 || len >= sizeof(value))
		return;
	strncpy(value, start, len);
	if (value[0] == '1')
		otg_state = true;
}

static ssize_t otg_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (otg_state)
		memcpy(buf, "n\n", 3);
	else
		memcpy(buf, "y\n", 3);
	return 2;
}

static ssize_t otg_enable_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	bool enabled = false;
	int ret = kstrtobool(buf, &enabled);

	if (ret)
		return ret;
	otg_state = !enabled;
	return count;
}
static DEVICE_ATTR_RW(otg_enable);

static void lenovo_monitor_device(struct usb_device *udev, unsigned long event)
{
	if (udev->parent && le16_to_cpu(udev->descriptor.idVendor) == 0x17ef &&
	    le16_to_cpu(udev->descriptor.idProduct) == 0x62e9)
		ext_fan_connected = event == USB_DEVICE_ADD;
}
