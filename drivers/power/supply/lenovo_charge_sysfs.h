/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/platform_device.h>

#define MAX_UEVENT_PROP_NUM	20

#define BATT_SYSFS_FIELD_RW(_name, _prop, _report)							\
{												\
	.attr = __ATTR(_name, 0664, lenovo_battery_sysfs_show, lenovo_battery_sysfs_store),	\
	.prop = _prop,										\
	.report_uevent = _report,								\
}

#define BATT_SYSFS_FIELD_RO(_name, _prop, _report)				\
{									\
	.attr = __ATTR(_name, 0664, lenovo_battery_sysfs_show, NULL),	\
	.prop = _prop,							\
	.report_uevent = _report,					\
}

/* These selectors index the local attribute table, not the ADSP property bank. */
enum sysfs_property {
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLE = 0,
	POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING = 1,
	POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING_EU = 2,
	POWER_SUPPLY_PROP_BATTERY_RECHARGE_SETTING = 3,
	POWER_SUPPLY_PROP_BATTERY_MAINTENANCE = 4,
	POWER_SUPPLY_PROP_BATTERY_MAINTENANCE20 = 5,
	POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE = 6,
	POWER_SUPPLY_PROP_BATTERY_PRODUCE_DATE = 7,
	POWER_SUPPLY_PROP_BATTERY_ACTIVATE_DATE = 8,
	POWER_SUPPLY_PROP_BATTERY_SOH = 9,
	POWER_SUPPLY_PROP_BATTERY_TIME_TO_FULL_NOW = 10,
	POWER_SUPPLY_PROP_BATTERY_TIME_TO_EMPTY_NOW = 11,
	POWER_SUPPLY_PROP_BATTERY_CV = 12,
	POWER_SUPPLY_PROP_BATTERY_CHG_FV = 13,
	POWER_SUPPLY_PROP_BATTERY_FAKE_CYCLE_COUNT = 14,
	POWER_SUPPLY_PROP_BATTERY_FAKE_SOC = 15,
	POWER_SUPPLY_PROP_BATTERY_ABNORMAL_STATE = 16,
	POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE_NVM = 17,
};

struct sysfs_info {
	struct device_attribute attr;
	enum sysfs_property prop;
	bool report_uevent;
};
