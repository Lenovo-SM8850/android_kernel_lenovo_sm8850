// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#define pr_fmt(fmt)	"BATTERY_CHG: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/rpmsg.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/thermal.h>
#include <linux/soc/qcom/qti_pmic_glink.h>
#include <linux/soc/qcom/battery_charger.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#include <linux/gpio/consumer.h>
#include "lenovo_charge_sysfs.h"

#define MSG_OWNER_BC			32778
#define MSG_TYPE_REQ_RESP		1
#define MSG_TYPE_NOTIFY			2

/* opcode for battery charger */
#define BC_SET_NOTIFY_REQ		0x04
#define BC_DISABLE_NOTIFY_REQ		0x05
#define BC_NOTIFY_IND			0x07
#define BC_BATTERY_STATUS_GET		0x30
#define BC_BATTERY_STATUS_SET		0x31
#define BC_USB_STATUS_GET		0x32
#define BC_USB_STATUS_SET		0x33
#define BC_USB2_STATUS_GET		0x10032
#define BC_USB2_STATUS_SET		0x10033
#define DUAL_USB_USB_NUM_PORTS		16
#define DUAL_USB_USB_OTG_RESET		69
#define BC_WLS_STATUS_GET		0x34
#define BC_WLS_STATUS_SET		0x35
#define BC_SHIP_MODE_REQ_SET		0x36
#define BC_WLS_FW_CHECK_UPDATE		0x40
#define BC_WLS_FW_PUSH_BUF_REQ		0x41
#define BC_WLS_FW_UPDATE_STATUS_RESP	0x42
#define BC_WLS_FW_PUSH_BUF_RESP		0x43
#define BC_WLS_FW_GET_VERSION		0x44
#define BC_SHUTDOWN_NOTIFY		0x47
#define BC_CHG_CTRL_LIMIT_EN		0x48
#define BC_HBOOST_VMAX_CLAMP_NOTIFY	0x79
#define BC_GENERIC_NOTIFY		0x80

#define HQ_POWER_SUPPLY_GET_REQ 0x60
#define HQ_POWER_SUPPLY_SET_REQ 0x61
#define LENOVO_QI_UPDATE_NOTIFY 0x65
#define BC_BATT_ABNORMAL_NOTIFY 0x90

/* Generic definitions */
#define MAX_STR_LEN			128
#define BC_WAIT_TIME_MS			1000
#define BC_PROBE_MIN_MS			2000
#define BC_PROBE_MAX_MS			60000
#define BC_FW_VER_WAIT_TIME_MS		1500
#define WLS_FW_PREPARE_TIME_MS		1000
#define WLS_FW_WAIT_TIME_MS		500
#define WLS_FW_UPDATE_TIME_MS		1000
#define WLS_FW_BUF_SIZE			128
#define CPS_WLS_FW_BUF_SIZE		256
#define DEFAULT_RESTRICT_FCC_UA		1000000

enum psy_type {
	PSY_TYPE_BATTERY,
	PSY_TYPE_USB,
	PSY_TYPE_WLS,
	PSY_TYPE_HQ,
	PSY_TYPE_USB2,
	PSY_TYPE_MAX,
};

enum ship_mode_type {
	SHIP_MODE_PMIC,
	SHIP_MODE_PACK_SIDE,
};

/* property ids */
enum battery_property_id {
	BATT_STATUS,
	BATT_HEALTH,
	BATT_PRESENT,
	BATT_CHG_TYPE,
	BATT_CAPACITY,
	BATT_SOH,
	BATT_VOLT_OCV,
	BATT_VOLT_NOW,
	BATT_VOLT_MAX,
	BATT_CURR_NOW,
	BATT_CHG_CTRL_LIM,
	BATT_CHG_CTRL_LIM_MAX,
	BATT_TEMP,
	BATT_TECHNOLOGY,
	BATT_CHG_COUNTER,
	BATT_CYCLE_COUNT,
	BATT_CHG_FULL_DESIGN,
	BATT_CHG_FULL,
	BATT_MODEL_NAME,
	BATT_TTF_AVG,
	BATT_TTE_AVG,
	BATT_RESISTANCE,
	BATT_POWER_NOW,
	BATT_POWER_AVG,
	BATT_CHG_CTRL_EN,
	BATT_CHG_CTRL_START_THR,
	BATT_CHG_CTRL_END_THR,
	BATT_CURR_AVG,
	BATT_PARALLEL_CELL_COUNT,
	BATT_PROP_MAX,
};

enum usb_property_id {
	USB_ONLINE,
	USB_VOLT_NOW,
	USB_VOLT_MAX,
	USB_CURR_NOW,
	USB_CURR_MAX,
	USB_INPUT_CURR_LIMIT,
	USB_TYPE,
	USB_ADAP_TYPE,
	USB_MOISTURE_DET_EN,
	USB_MOISTURE_DET_STS,
	USB_TEMP,
	USB_REAL_TYPE,
	USB_TYPEC_COMPLIANT,
	USB_PROP_MAX,
};

enum wireless_property_id {
	WLS_ONLINE,
	WLS_VOLT_NOW,
	WLS_VOLT_MAX,
	WLS_CURR_NOW,
	WLS_CURR_MAX,
	WLS_TYPE,
	WLS_BOOST_EN,
	WLS_HBOOST_VMAX,
	WLS_INPUT_CURR_LIMIT,
	WLS_ADAP_TYPE,
	WLS_CONN_TEMP,
	WLS_PROP_MAX,
};

enum huaqin_property_id {
	HQ_POWER_SUPPLY_PROP_CC_ORIENTATION = 0,            /*0 :  1: */
	HQ_POWER_SUPPLY_PROP_CC_ORIENTATION2 = 1,
	HQ_POWER_SUPPLY_PROP_IC_MAIN_CHARGE = 2,
	HQ_POWER_SUPPLY_PROP_IC_FG_1 = 3,
	HQ_POWER_SUPPLY_PROP_IC_FG_2 = 4,
	HQ_POWER_SUPPLY_PROP_IC_CP_1 = 5,
	HQ_POWER_SUPPLY_PROP_IC_CP_2 = 6,
	HQ_POWER_SUPPLY_PROP_IC_QI = 7,
	HQ_POWER_SUPPLY_PROP_IC_PDPHY = 8,
	HQ_POWER_SUPPLY_PROP_IC_QC = 9,
	HQ_POWER_SUPPLY_PROP_BATTERY_MANUFACTURER_1 = 10,
	HQ_POWER_SUPPLY_PROP_BATTERY_MANUFACTURER_2 = 11,
	HQ_POWER_SUPPLY_PROP_FG_1_TEMP = 12,
	HQ_POWER_SUPPLY_PROP_FG_2_TEMP = 13,
	HQ_POWER_SUPPLY_PROP_FG_1_CURRENT = 14,
	HQ_POWER_SUPPLY_PROP_FG_2_CURRENT = 15,
	HQ_POWER_SUPPLY_PROP_FG_1_VOLTAGE = 16,
	HQ_POWER_SUPPLY_PROP_FG_2_VOLTAGE = 17,
	HQ_POWER_SUPPLY_PROP_FG_1_PRODUCE_DATE = 18,
	HQ_POWER_SUPPLY_PROP_FG_2_PRODUCE_DATE = 19,
	HQ_POWER_SUPPLY_PROP_FG_1_ACTIVATE_DATE = 20,
	HQ_POWER_SUPPLY_PROP_FG_2_ACTIVATE_DATE = 21,
	HQ_POWER_SUPPLY_PROP_FG_1_SOH = 22,
	HQ_POWER_SUPPLY_PROP_FG_2_SOH = 23,
	HQ_POWER_SUPPLY_PROP_FG_1_CYCLE_COUNT = 24,
	HQ_POWER_SUPPLY_PROP_FG_2_CYCLE_COUNT = 25,
	HQ_POWER_SUPPLY_PROP_FG_MONITOR_CMD = 26,
	HQ_POWER_SUPPLY_PROP_INPUT_SUSPEND = 27,
	HQ_POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLE = 28,
	HQ_POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING = 29,
	HQ_POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING_EU = 30,
	HQ_POWER_SUPPLY_PROP_BATTERY_RECHARGE_SETTING = 31,
	HQ_POWER_SUPPLY_PROP_BATTERY_MAINTENANCE = 32,
	HQ_POWER_SUPPLY_PROP_BATTERY_MAINTENANCE20 = 33,
	HQ_POWER_SUPPLY_PROP_BATTERY_PRODUCE_DATE = 34,
	HQ_POWER_SUPPLY_PROP_BATTERY_ACTIVATE_DATE = 35,
	HQ_POWER_SUPPLY_PROP_BATTERY_SOH = 36,
	HQ_POWER_SUPPLY_PROP_BATTERY_CV = 37,
	HQ_POWER_SUPPLY_PROP_BATTERY_CHG_FV = 38,
	HQ_POWER_SUPPLY_PROP_BATTERY_FAKE_CYCLE_COUNT = 39,
	HQ_POWER_SUPPLY_PROP_BATTERY_FAKE_SOC = 40,
	HQ_POWER_SUPPLY_PROP_BATTERY_ABNORMAL_STATE = 41,
	HQ_POWER_SUPPLY_PROP_IC_SC7603A = 42,
	HQ_POWER_SUPPLY_PROP_IC_SC7603B = 43,
	HQ_POWER_SUPPLY_PROP_FG_1_SOC = 44,
	HQ_POWER_SUPPLY_PROP_FG_2_SOC = 45,
	HQ_POWER_SUPPLY_PROP_CP_MASTER_IBUS = 46,
	HQ_POWER_SUPPLY_PROP_CP_SLAVE_IBUS = 47,
	HQ_POWER_SUPPLY_PROP_IBUS_TOTAL = 48,
	HQ_POWER_SUPPLY_PROP_DET_GPIO_PORT1 = 49,
	HQ_POWER_SUPPLY_PROP_DET_GPIO_PORT2 = 50,
	HQ_POWER_SUPPLY_PROP_CP_DUMP_REGISTER = 51,
	HQ_POWER_SUPPLY_PROP_CP_DEVICE_ID = 52,
	HQ_POWER_SUPPLY_PROP_PPS_STATUS = 53,
	HQ_POWER_SUPPLY_PROP_FG_1_USED = 54,
	HQ_POWER_SUPPLY_PROP_FG_2_USED = 55,
	HQ_POWER_SUPPLY_PROP_RESERVED_56 = 56,
	HQ_POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE_NVM = 57,
	HQ_POWER_SUPPLY_PROPERTIES_MAX = 58
};

enum {
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP = 0x80,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5,
};

struct battery_charger_set_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			power_state;
	u32			low_capacity;
	u32			high_capacity;
};

struct battery_charger_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			notification;
};

struct battery_charger_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			property_id;
	u32			value;
};

struct battery_charger_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			value;
	u32			ret_code;
};

struct battery_model_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	char			model[MAX_STR_LEN];
};


struct wireless_fw_check_req {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
	u32			fw_size;
	u32			fw_crc;
};

struct wireless_fw_check_resp {
	struct pmic_glink_hdr	hdr;
	u32			ret_code;
};

struct wireless_fw_push_buf_req {
	struct pmic_glink_hdr	hdr;
	u8			buf[CPS_WLS_FW_BUF_SIZE];
	u32			fw_chunk_id;
};

struct wireless_fw_push_buf_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_status;
};

struct wireless_fw_update_status {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_done;
};

struct wireless_fw_get_version_req {
	struct pmic_glink_hdr	hdr;
};

struct wireless_fw_get_version_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
};

struct battery_charger_ship_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			ship_mode_type;
};

struct battery_charger_chg_ctrl_msg {
	struct pmic_glink_hdr	hdr;
	u32			enable;
	u32			target_soc;
	u32			delta_soc;
};

static_assert(sizeof(struct battery_charger_req_msg) == 24);
static_assert(offsetof(struct battery_charger_req_msg, battery_id) == 12);
static_assert(offsetof(struct battery_charger_req_msg, property_id) == 16);
static_assert(offsetof(struct battery_charger_req_msg, value) == 20);
static_assert(sizeof(struct battery_charger_resp_msg) == 24);
static_assert(offsetof(struct battery_charger_resp_msg, property_id) == 12);
static_assert(offsetof(struct battery_charger_resp_msg, value) == 16);
static_assert(offsetof(struct battery_charger_resp_msg, ret_code) == 20);
static_assert(sizeof(struct battery_charger_notify_msg) == 16);

struct psy_state {
	struct power_supply	*psy;
	char			*model;
	const int		*map;
	u32			*prop;
	u32			prop_count;
	u32			opcode_get;
	u32			opcode_set;
	bool			isolate;
	bool			timed_out;
	u32			req_opcode;
	u32			req_property;
	bool			probe_pending;
	u32			probe_property;
	unsigned long		probe_jiffies;
};


struct battery_chg_dev;

/* Select each Lenovo GLINK protocol by compatible. */
struct battery_chg_protocol {
	struct psy_state banks[PSY_TYPE_MAX];
	const struct attribute_group **class_groups;
	int (*create_battery_group)(struct battery_chg_dev *bcdev);
	int (*create_usb_group)(struct battery_chg_dev *bcdev);
	bool dual_port;
};

struct battery_chg_dev {
	struct device			*dev;
	const struct battery_chg_protocol *protocol;
	struct class			battery_class;
	struct pmic_glink_client	*client;
	struct mutex			rw_lock;
	struct rw_semaphore		state_sem;
	struct completion		ack;
	struct completion		fw_buf_ack;
	struct completion		fw_update_ack;
	struct psy_state		psy_list[PSY_TYPE_MAX];
	struct dentry			*debugfs_dir;
	void				*notifier_cookie;
	u32				*thermal_levels;
	const char			*wls_fw_name;
	int				curr_thermal_level;
	int				num_thermal_levels;
	int				shutdown_volt_mv;
	atomic_t			state;
	struct work_struct		subsys_up_work;
	struct work_struct		usb_type_work;
	struct work_struct		battery_check_work;
	struct wakeup_source		*charge_wakelock;
	int				fake_soc;
	bool				block_tx;
	bool				ship_mode_en;
	bool				ship_mode_immediate;
	bool				debug_battery_detected;
	bool				wls_fw_update_reqd;
	u32				wls_fw_version;
	u16				wls_fw_crc;
	u32				wls_fw_update_time_ms;
	struct notifier_block		reboot_notifier;
	u32				thermal_fcc_ua;
	u32				restrict_fcc_ua;
	u32				last_fcc_ua;
	u32				usb_icl_ua;
	u32 usb2_icl_ua;
	u32 usb_num_ports;
	struct mutex chg_status_lock;
	unsigned long chg_status_updated;
	bool chg_status_valid;
	bool usb2_enabled;
	struct delayed_work bank_probe_work;
	u32 probe_backoff_ms;
	unsigned long last_probe_jiffies;
	u32				thermal_fcc_step;
	bool				restrict_chg_en;
	u8				chg_ctrl_start_thr;
	u8				chg_ctrl_end_thr;
	bool				chg_ctrl_en;
	/* To track the driver initialization status */
	bool				initialized;
	bool				notify_en;
	bool				error_prop;
	struct delayed_work devices_get_informs_work;
	bool battery_abnormal;
	int fake_abnormal_level;
	struct delayed_work lenovo_chg_monitor_work;
	struct gpio_desc *usb1_burn_en;
	struct gpio_desc *usb2_burn_en;
	struct thermal_zone_device *usb1_therm;
	struct thermal_zone_device *usb2_therm;
	bool lenovo_chg_monitor_started;
	bool class_registered;
};

static_assert(BATT_PROP_MAX == 29);
static_assert(USB_PROP_MAX == 13);
static_assert(WLS_PROP_MAX == 11);
static_assert(HQ_POWER_SUPPLY_PROPERTIES_MAX == 58);
static_assert(sizeof(struct battery_model_resp_msg) == 144);

static struct battery_chg_dev *gbcdev;

static const int battery_prop_map[BATT_PROP_MAX] = {
	[BATT_STATUS]		= POWER_SUPPLY_PROP_STATUS,
	[BATT_HEALTH]		= POWER_SUPPLY_PROP_HEALTH,
	[BATT_PRESENT]		= POWER_SUPPLY_PROP_PRESENT,
	[BATT_CHG_TYPE]		= POWER_SUPPLY_PROP_CHARGE_TYPE,
	[BATT_CAPACITY]		= POWER_SUPPLY_PROP_CAPACITY,
	[BATT_VOLT_OCV]		= POWER_SUPPLY_PROP_VOLTAGE_OCV,
	[BATT_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[BATT_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[BATT_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[BATT_CHG_CTRL_LIM]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	[BATT_CHG_CTRL_LIM_MAX]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	[BATT_TEMP]		= POWER_SUPPLY_PROP_TEMP,
	[BATT_TECHNOLOGY]	= POWER_SUPPLY_PROP_TECHNOLOGY,
	[BATT_CHG_COUNTER]	= POWER_SUPPLY_PROP_CHARGE_COUNTER,
	[BATT_CYCLE_COUNT]	= POWER_SUPPLY_PROP_CYCLE_COUNT,
	[BATT_CHG_FULL_DESIGN]	= POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	[BATT_CHG_FULL]		= POWER_SUPPLY_PROP_CHARGE_FULL,
	[BATT_MODEL_NAME]	= POWER_SUPPLY_PROP_MODEL_NAME,
	[BATT_TTF_AVG]		= POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	[BATT_TTE_AVG]		= POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	[BATT_POWER_NOW]	= POWER_SUPPLY_PROP_POWER_NOW,
	[BATT_POWER_AVG]	= POWER_SUPPLY_PROP_POWER_AVG,
	[BATT_CHG_CTRL_START_THR] = POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	[BATT_CHG_CTRL_END_THR]   = POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	[BATT_CURR_AVG]		= POWER_SUPPLY_PROP_CURRENT_AVG,
};

static const int usb_prop_map[USB_PROP_MAX] = {
	[USB_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[USB_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[USB_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[USB_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[USB_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[USB_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[USB_ADAP_TYPE]		= POWER_SUPPLY_PROP_USB_TYPE,
	[USB_TEMP]		= POWER_SUPPLY_PROP_TEMP,
};

static const int wls_prop_map[WLS_PROP_MAX] = {
	[WLS_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[WLS_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[WLS_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[WLS_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[WLS_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[WLS_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[WLS_CONN_TEMP]		= POWER_SUPPLY_PROP_TEMP,
};

static const int hq_prop_map[HQ_POWER_SUPPLY_PROPERTIES_MAX] = { };
static const int dual_usb_usb_prop_map[17] = {
	[USB_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[USB_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[USB_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[USB_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[USB_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[USB_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[USB_ADAP_TYPE]		= POWER_SUPPLY_PROP_USB_TYPE,
	[USB_TEMP]		= POWER_SUPPLY_PROP_TEMP,
	[13 ... 16] = -1,
};
static const int dual_usb_hq_prop_map[70] = { [0 ... 69] = -1 };

/* Standard usb_type definitions similar to power_supply_sysfs.c */
static const char * const power_supply_usb_type_text[] = {
	"Unknown", "SDP", "DCP", "CDP", "ACA", "C",
	"PD", "PD_DRP", "PD_PPS", "BrickID"
};

/* Custom usb_type definitions */
static const char * const qc_power_supply_usb_type_text[] = {
	"HVDCP", "HVDCP_3", "HVDCP_3P5"
};

/* wireless_type definitions */
static const char * const qc_power_supply_wls_type_text[] = {
	"Unknown", "BPP", "EPP", "HPP"
};

static RAW_NOTIFIER_HEAD(hboost_notifier);

int register_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_register(&hboost_notifier, nb);
}
EXPORT_SYMBOL(register_hboost_event_notifier);

int unregister_hboost_event_notifier(struct notifier_block *nb)
{
	return raw_notifier_chain_unregister(&hboost_notifier, nb);
}
EXPORT_SYMBOL(unregister_hboost_event_notifier);

static int battery_chg_fw_write(struct battery_chg_dev *bcdev, void *data,
				int len)
{
	int rc;

	down_read(&bcdev->state_sem);
	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_debug("glink state is down\n");
		up_read(&bcdev->state_sem);
		return -ENOTCONN;
	}
	up_read(&bcdev->state_sem);

	reinit_completion(&bcdev->fw_buf_ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->fw_buf_ack,
					msecs_to_jiffies(WLS_FW_WAIT_TIME_MS));
		if (!rc) {
			pr_err("Error, timed out sending message\n");
			return -ETIMEDOUT;
		}

		rc = 0;
	}

	return rc;
}

static int isolated_bank_index(struct battery_chg_dev *bcdev, u32 opcode)
{
	int i;

	for (i = 0; i < PSY_TYPE_MAX; i++) {
		struct psy_state *pst = &bcdev->psy_list[i];

		if (!pst->isolate || (i == PSY_TYPE_USB2 && !pst->prop_count))
			continue;
		if (opcode == pst->opcode_get || opcode == pst->opcode_set)
			return i;
	}
	return -1;
}

static bool isolated_bank_timed_out(struct battery_chg_dev *bcdev)
{
	int i;

	for (i = 0; i < PSY_TYPE_MAX; i++) {
		struct psy_state *pst = &bcdev->psy_list[i];

		if (pst->isolate && READ_ONCE(pst->timed_out) &&
		    (i != PSY_TYPE_USB2 || bcdev->usb2_enabled))
			return true;
	}
	return false;
}

static void kick_bank_probe(struct battery_chg_dev *bcdev, u32 delay_ms)
{
	down_read(&bcdev->state_sem);
	if (bcdev->initialized && isolated_bank_timed_out(bcdev) &&
	    (delay_ms || time_after_eq(jiffies,
		READ_ONCE(bcdev->last_probe_jiffies) +
		msecs_to_jiffies(BC_PROBE_MIN_MS)))) {
		mod_delayed_work(system_wq, &bcdev->bank_probe_work,
				 msecs_to_jiffies(delay_ms));
	}
	up_read(&bcdev->state_sem);
}

static int battery_chg_write(struct battery_chg_dev *bcdev, void *data,
				int len, u32 wait_time)
{
	struct pmic_glink_hdr *hdr = data;
	struct battery_charger_req_msg *req = data;
	int bank = isolated_bank_index(bcdev, hdr->opcode);
	struct psy_state *pst = bank < 0 ? NULL : &bcdev->psy_list[bank];
	bool usb2 = bcdev->psy_list[PSY_TYPE_USB2].prop_count &&
		    (hdr->opcode == BC_USB2_STATUS_GET ||
		     hdr->opcode == BC_USB2_STATUS_SET);
	int rc;

	/* Suppressed callers must not queue behind a recovery transaction. */
	if ((usb2 && !bcdev->usb2_enabled) ||
	    (pst && READ_ONCE(pst->timed_out)))
		return -ENODATA;

	/*
	 * When the subsystem goes down, it's better to return the last
	 * known values until it comes back up. Hence, return 0 so that
	 * pmic_glink_write() is not attempted until pmic glink is up.
	 */
	down_read(&bcdev->state_sem);
	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_debug("glink state is down\n");
		up_read(&bcdev->state_sem);
		return 0;
	}
	up_read(&bcdev->state_sem);

	if (bcdev->debug_battery_detected && bcdev->block_tx) {
		return 0;
	}

	mutex_lock(&bcdev->rw_lock);
	/* Another reader may have timed out while we waited for the lock. */
	if (pst && READ_ONCE(pst->timed_out)) {
		rc = -ENODATA;
		goto out;
	}
	if (pst) {
		if (len == sizeof(*req))
			WRITE_ONCE(pst->req_property, req->property_id);
		/* Publish the property before the active opcode. */
		smp_store_release(&pst->req_opcode, hdr->opcode);
	}
	reinit_completion(&bcdev->ack);
	bcdev->error_prop = false;
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->ack,
					msecs_to_jiffies(wait_time));
		if (!rc) {
			if (pst) {
				WRITE_ONCE(pst->req_opcode, 0);
				WRITE_ONCE(pst->probe_property, len == sizeof(*req) ?
					   READ_ONCE(pst->req_property) : pst->prop_count);
				WRITE_ONCE(pst->probe_pending, false);
				WRITE_ONCE(pst->timed_out, true);
				if (pst->psy)
					pr_err_ratelimited("Bank %s isolated: opcode %#x prop_id %u\n",
						pst->psy->desc->name, hdr->opcode,
						READ_ONCE(pst->req_property));
				else
					pr_err_ratelimited("Bank %d isolated: opcode %#x prop_id %u\n",
						bank, hdr->opcode, READ_ONCE(pst->req_property));
				kick_bank_probe(bcdev, READ_ONCE(bcdev->probe_backoff_ms));
			}
			if (len == sizeof(*req))
				pr_err("Error, timed out sending message opcode %#x prop_id %u\n",
					hdr->opcode, req->property_id);
			else
				pr_err("Error, timed out sending message opcode %#x\n",
					hdr->opcode);
			rc = -ETIMEDOUT;
			goto out;
		}
		rc = 0;

		/*
		 * In case the opcode used is not supported, the remote
		 * processor might ack it immediately with a return code indicating
		 * an error. This additional check is to check if such an error has
		 * happened and return immediately with error in that case. This
		 * avoids wasting time waiting in the above timeout condition for this
		 * type of error.
		 */
		if (bcdev->error_prop) {
			bcdev->error_prop = false;
			rc = -ENODATA;
		} else if (pst) {
			WRITE_ONCE(bcdev->probe_backoff_ms, BC_PROBE_MIN_MS);
		}
	}
out:
	if (pst)
		WRITE_ONCE(pst->req_opcode, 0);
	mutex_unlock(&bcdev->rw_lock);

	return rc;
}

/* A recovery probe never waits for a reply while holding the shared lock. */
static void battery_chg_bank_probe_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(to_delayed_work(work),
				struct battery_chg_dev, bank_probe_work);
	struct battery_charger_req_msg req_msg = { { 0 } };
	bool probe_sent = false;
	u32 backoff_ms;
	int i;

	down_read(&bcdev->state_sem);
	if (!bcdev->initialized ||
	    atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		up_read(&bcdev->state_sem);
		return;
	}
	up_read(&bcdev->state_sem);
	if (bcdev->debug_battery_detected && bcdev->block_tx)
		return;
	/* A kick may have queued while the previous probe was being sent. */
	if (time_before(jiffies, READ_ONCE(bcdev->last_probe_jiffies) +
			msecs_to_jiffies(BC_PROBE_MIN_MS)))
		goto rearm;

	for (i = 0; i < PSY_TYPE_MAX; i++) {
		struct psy_state *pst = &bcdev->psy_list[i];

		if (!pst->isolate || !READ_ONCE(pst->timed_out) ||
		    (i == PSY_TYPE_USB2 && !bcdev->usb2_enabled))
			continue;
		if (!mutex_trylock(&bcdev->rw_lock))
			break;
		if (!READ_ONCE(pst->timed_out)) {
			mutex_unlock(&bcdev->rw_lock);
			continue;
		}
		req_msg.hdr.owner = MSG_OWNER_BC;
		req_msg.hdr.type = MSG_TYPE_REQ_RESP;
		req_msg.hdr.opcode = pst->opcode_get;
		req_msg.property_id = READ_ONCE(pst->probe_property);
		if (req_msg.property_id >= pst->prop_count)
			req_msg.property_id = i == PSY_TYPE_WLS ?
					      WLS_ADAP_TYPE : USB_ADAP_TYPE;
		WRITE_ONCE(pst->probe_property, req_msg.property_id);
		WRITE_ONCE(pst->probe_jiffies, jiffies);
		WRITE_ONCE(pst->probe_pending, true);
		WRITE_ONCE(bcdev->last_probe_jiffies, jiffies);
		/* Consume one backoff step per probe round. */
		if (!probe_sent) {
			backoff_ms = READ_ONCE(bcdev->probe_backoff_ms);
			WRITE_ONCE(bcdev->probe_backoff_ms,
				   min(backoff_ms * 2, (u32)BC_PROBE_MAX_MS));
		}
		probe_sent = true;
		pmic_glink_write(bcdev->client, &req_msg, sizeof(req_msg));
		mutex_unlock(&bcdev->rw_lock);
	}

rearm:
	down_read(&bcdev->state_sem);
	if (bcdev->initialized && isolated_bank_timed_out(bcdev)) {
		backoff_ms = READ_ONCE(bcdev->probe_backoff_ms);
		/* Preserve an immediate notification kick already queued. */
		queue_delayed_work(system_wq, &bcdev->bank_probe_work,
				   msecs_to_jiffies(backoff_ms));
	}
	up_read(&bcdev->state_sem);
}

static int write_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id, u32 val)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = val;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;

	if (pst->psy)
		pr_debug("psy: %s prop_id: %u val: %u\n", pst->psy->desc->name,
			req_msg.property_id, val);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg), BC_WAIT_TIME_MS);
}

static int read_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = 0;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	if (pst->psy)
		pr_debug("psy: %s prop_id: %u\n", pst->psy->desc->name,
			req_msg.property_id);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg), BC_WAIT_TIME_MS);
}



static int get_property_id(struct psy_state *pst,
			enum power_supply_property prop)
{
	u32 i;

	for (i = 0; i < pst->prop_count; i++)
		if (pst->map[i] == prop)
			return i;

	if (pst->psy)
		pr_err("No property id for property %d in psy %s\n", prop,
			pst->psy->desc->name);

	return -ENOENT;
}

static void battery_chg_notify_disable(struct battery_chg_dev *bcdev)
{
	struct battery_charger_set_notify_msg req_msg = { { 0 } };
	int rc;

	if (bcdev->notify_en) {
		/* Send request to disable notification */
		req_msg.hdr.owner = MSG_OWNER_BC;
		req_msg.hdr.type = MSG_TYPE_NOTIFY;
		req_msg.hdr.opcode = BC_DISABLE_NOTIFY_REQ;

		rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg), BC_WAIT_TIME_MS);
		if (rc < 0)
			pr_err("Failed to disable notification rc=%d\n", rc);
		else
			bcdev->notify_en = false;
	}
}

static void battery_chg_notify_enable(struct battery_chg_dev *bcdev)
{
	struct battery_charger_set_notify_msg req_msg = { { 0 } };
	int rc;

	if (!bcdev->notify_en) {
		/* Send request to enable notification */
		req_msg.hdr.owner = MSG_OWNER_BC;
		req_msg.hdr.type = MSG_TYPE_NOTIFY;
		req_msg.hdr.opcode = BC_SET_NOTIFY_REQ;

		rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg), BC_WAIT_TIME_MS);
		if (rc < 0)
			pr_err("Failed to enable notification rc=%d\n", rc);
		else
			bcdev->notify_en = true;
	}
}

static void battery_chg_state_cb(void *priv, enum pmic_glink_state state)
{
	struct battery_chg_dev *bcdev = priv;

	pr_debug("state: %d\n", state);

	down_write(&bcdev->state_sem);
	if (!bcdev->initialized) {
		pr_warn("Driver not initialized, pmic_glink state %d\n", state);
		up_write(&bcdev->state_sem);
		return;
	}
	atomic_set(&bcdev->state, state);
	up_write(&bcdev->state_sem);

	if (state == PMIC_GLINK_STATE_UP)
		schedule_work(&bcdev->subsys_up_work);
	else if (state == PMIC_GLINK_STATE_DOWN)
		bcdev->notify_en = false;
}

/**
 * qti_battery_charger_get_prop() - Gets the property being requested
 *
 * @name: Power supply name
 * @prop_id: Property id to be read
 * @val: Pointer to value that needs to be updated
 *
 * Return: 0 if success, negative on error.
 */
int qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val)
{
	struct power_supply *psy;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst;
	int rc = 0;

	if (prop_id >= BATTERY_CHARGER_PROP_MAX)
		return -EINVAL;

	if (strcmp(name, "battery") && strcmp(name, "usb") &&
	    strcmp(name, "wireless"))
		return -EINVAL;

	psy = power_supply_get_by_name(name);
	if (!psy)
		return -ENODEV;

	bcdev = power_supply_get_drvdata(psy);
	if (!bcdev)
		return -ENODEV;

	power_supply_put(psy);

	switch (prop_id) {
	case BATTERY_RESISTANCE:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
		if (!rc)
			*val = pst->prop[BATT_RESISTANCE];
		break;
	default:
		break;
	}

	return rc;
}
EXPORT_SYMBOL(qti_battery_charger_get_prop);

static bool validate_message(struct battery_chg_dev *bcdev,
			struct battery_charger_resp_msg *resp_msg, size_t len)
{
	if (len != sizeof(*resp_msg)) {
		pr_err("Incorrect response length %zu for opcode %#x\n", len,
			resp_msg->hdr.opcode);
		return false;
	}

	if (resp_msg->ret_code) {
		pr_err_ratelimited("Error in response for opcode %#x prop_id %u, rc=%d\n",
			resp_msg->hdr.opcode, resp_msg->property_id,
			(int)resp_msg->ret_code);
		bcdev->error_prop = true;
		return false;
	}

	return true;
}



#define MODEL_DEBUG_BOARD	"Debug_Board"
static void handle_message(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_resp_msg *resp_msg = data;
	struct battery_model_resp_msg *model_resp_msg = data;
	struct wireless_fw_check_resp *fw_check_msg;
	struct wireless_fw_push_buf_resp *fw_resp_msg;
	struct wireless_fw_update_status *fw_update_msg;
	struct wireless_fw_get_version_resp *fw_ver_msg;
	struct psy_state *pst;
	bool ack_set = false;
	int bank = isolated_bank_index(bcdev, resp_msg->hdr.opcode);

	if (bank == PSY_TYPE_USB2 && !bcdev->usb2_enabled)
		goto unknown_opcode;

	/* Late replies must not change the shared error or ACK another bank. */
	if (bank >= 0) {
		pst = &bcdev->psy_list[bank];
		/* Acquire the property published with the active opcode. */
		if (smp_load_acquire(&pst->req_opcode) != resp_msg->hdr.opcode ||
		    (resp_msg->hdr.opcode == pst->opcode_get &&
		     len == sizeof(*resp_msg) &&
		     READ_ONCE(pst->req_property) != resp_msg->property_id)) {
			if (len == sizeof(*resp_msg) &&
			    resp_msg->hdr.opcode == pst->opcode_get &&
			    resp_msg->property_id < pst->prop_count) {
				if (!resp_msg->ret_code)
					pst->prop[resp_msg->property_id] = resp_msg->value;
				if (READ_ONCE(pst->timed_out) &&
				    READ_ONCE(pst->probe_pending) &&
				    resp_msg->property_id == READ_ONCE(pst->probe_property) &&
				    time_before_eq(jiffies, READ_ONCE(pst->probe_jiffies) +
						msecs_to_jiffies(BC_WAIT_TIME_MS))) {
					WRITE_ONCE(pst->probe_pending, false);
					WRITE_ONCE(pst->timed_out, false);
					if (pst->psy)
						pr_info_ratelimited("Bank %s recovered\n",
							pst->psy->desc->name);
					else
						pr_info_ratelimited("Bank %d recovered\n", bank);
					schedule_work(&bcdev->usb_type_work);
				}
			}
			return;
		}
	}

	switch (resp_msg->hdr.opcode) {
	case BC_BATTERY_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

		/* Handle model response uniquely as it's a string */
		if (pst->model && len == sizeof(*model_resp_msg)) {
			memcpy(pst->model, model_resp_msg->model, MAX_STR_LEN);
			ack_set = true;
			bcdev->debug_battery_detected = !strcmp(pst->model,
					MODEL_DEBUG_BOARD);
			break;
		}

		/* Other response should be of same type as they've u32 value */
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_USB_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		goto usb_response;
	case BC_USB2_STATUS_GET:
		if (!bcdev->usb2_enabled)
			goto unknown_opcode;
		pst = &bcdev->psy_list[PSY_TYPE_USB2];
usb_response:
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];
		if (validate_message(bcdev, resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}
		break;

	case HQ_POWER_SUPPLY_GET_REQ:
		pst = &bcdev->psy_list[PSY_TYPE_HQ];
		if (validate_message(bcdev, resp_msg, len) &&
			resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}
		break;

	case BC_BATTERY_STATUS_SET:
	case BC_USB_STATUS_SET:
	case BC_WLS_STATUS_SET:
	case HQ_POWER_SUPPLY_SET_REQ:
		if (validate_message(bcdev, data, len))
			ack_set = true;

		break;
	case BC_USB2_STATUS_SET:
		if (!bcdev->usb2_enabled)
			goto unknown_opcode;
		if (bcdev->usb2_enabled && validate_message(bcdev, data, len))
			ack_set = true;
		break;
	case BC_SET_NOTIFY_REQ:
	case BC_DISABLE_NOTIFY_REQ:
	case BC_SHUTDOWN_NOTIFY:
	case BC_SHIP_MODE_REQ_SET:
	case BC_CHG_CTRL_LIMIT_EN:
		/* Always ACK response for notify or ship_mode request */
		ack_set = true;
		break;
	case BC_WLS_FW_CHECK_UPDATE:
		if (len == sizeof(*fw_check_msg)) {
			fw_check_msg = data;
			if (fw_check_msg->ret_code == 1)
				bcdev->wls_fw_update_reqd = true;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for wls_fw_check_update\n",
				len);
		}
		break;
	case BC_WLS_FW_PUSH_BUF_RESP:
		if (len == sizeof(*fw_resp_msg)) {
			fw_resp_msg = data;
			if (fw_resp_msg->fw_update_status == 1)
				complete(&bcdev->fw_buf_ack);
		} else {
			pr_err("Incorrect response length %zu for wls_fw_push_buf_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_UPDATE_STATUS_RESP:
		if (len == sizeof(*fw_update_msg)) {
			fw_update_msg = data;
			if (fw_update_msg->fw_update_done == 1)
				complete(&bcdev->fw_update_ack);
			else
				pr_err("Wireless FW update not done %d\n",
					(int)fw_update_msg->fw_update_done);
		} else {
			pr_err("Incorrect response length %zu for wls_fw_update_status_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_GET_VERSION:
		if (len == sizeof(*fw_ver_msg)) {
			fw_ver_msg = data;
			bcdev->wls_fw_version = fw_ver_msg->fw_version;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for wls_fw_get_version\n",
				len);
		}
		break;
	default:
unknown_opcode:
		pr_err("Unknown opcode: %u\n", resp_msg->hdr.opcode);
		break;
	}

	if (ack_set || bcdev->error_prop)
		complete(&bcdev->ack);
}



static struct power_supply_desc usb_psy_desc;
static struct power_supply_desc dual_usb_usb2_psy_desc;
static struct power_supply_desc wls_psy_desc;

static bool is_offmode(void)
{
	static struct device_node *np = NULL;
	static bool offmode = 0;

	if(np != NULL) {
		return offmode;
	}

	const char *bootparams = NULL;
	np = of_find_node_by_path("/chosen");
	if (!np)
		goto out;
	of_property_read_string(np, "bootargs", &bootparams);
	if (!bootparams)
		pr_err("%s: failed to get bootargs property\n", __func__);
	else if (strstr(bootparams, "offmode.charge=1")) {
		offmode = true;
	}

out:
	pr_err("offmode.charge=%d\n", offmode);
	return offmode;
}


static void battery_chg_update_usb_type_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, usb_type_work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	struct psy_state *pst_wls = &bcdev->psy_list[PSY_TYPE_WLS];
	struct psy_state *pst_battery = &bcdev->psy_list[PSY_TYPE_BATTERY];
	u32 wls_type = 0;
	bool wls_read_ok;
	int rc;

	if (bcdev->psy_list[PSY_TYPE_USB2].psy) {
		struct psy_state *usb2 = &bcdev->psy_list[PSY_TYPE_USB2];
		u32 type;

		if (!read_property_id(bcdev, usb2, USB_ADAP_TYPE)) {
			type = usb2->prop[USB_ADAP_TYPE];
			dual_usb_usb2_psy_desc.type = type == POWER_SUPPLY_USB_TYPE_CDP ?
				POWER_SUPPLY_TYPE_USB_CDP :
				(type == POWER_SUPPLY_USB_TYPE_PD ||
				 type == POWER_SUPPLY_USB_TYPE_PD_DRP ||
				 type == POWER_SUPPLY_USB_TYPE_PD_PPS) ? POWER_SUPPLY_TYPE_USB_PD :
				(type == POWER_SUPPLY_USB_TYPE_DCP || type >= 0x80) ?
				POWER_SUPPLY_TYPE_USB_DCP : POWER_SUPPLY_TYPE_USB;
			if (type != POWER_SUPPLY_USB_TYPE_SDP &&
			    type != POWER_SUPPLY_USB_TYPE_CDP && type != POWER_SUPPLY_USB_TYPE_PD)
				bcdev->usb2_icl_ua = 0;
		}
		read_property_id(bcdev, usb2, USB_REAL_TYPE);
		read_property_id(bcdev, usb2, USB_VOLT_NOW);
		power_supply_changed(usb2->psy);
	}
	rc = read_property_id(bcdev, pst_battery, BATT_STATUS);
	if (rc < 0) {
		pr_err("Failed to read BATT_STATUS, rc=%d\n", rc);
		return;
	}

	rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0) {
		pr_err("Failed to read USB_ADAP_TYPE rc=%d\n", rc);
		return;
	}

	rc = read_property_id(bcdev, pst_wls, WLS_ADAP_TYPE);
	wls_read_ok = rc >= 0;
	if (!wls_read_ok)
		pr_err_ratelimited("Failed to read WLS_ADAP_TYPE rc=%d\n", rc);
	else
		wls_type = pst_wls->prop[WLS_ADAP_TYPE];

	/* The TB323 work refreshes the cached real type and both port voltages. */
	rc = read_property_id(bcdev, pst, USB_REAL_TYPE);
	if (rc < 0)
		pr_err("Failed to read USB_REAL_TYPE rc=%d\n", rc);
	rc = read_property_id(bcdev, pst, USB_VOLT_NOW);
	if (rc < 0)
		pr_err("Failed to read USB_VOLT_NOW rc=%d\n", rc);
	if (wls_read_ok) {
		rc = read_property_id(bcdev, pst_wls, WLS_VOLT_NOW);
		if (rc < 0)
			pr_err("Failed to read WLS_VOLT_NOW rc=%d\n", rc);
	}
	if ((pst->prop[USB_ADAP_TYPE] || wls_type) &&
	    pst_battery->prop[BATT_STATUS] != POWER_SUPPLY_STATUS_FULL &&
	    (pst_battery->prop[BATT_STATUS] != POWER_SUPPLY_STATUS_NOT_CHARGING || is_offmode())) {
		if (!bcdev->charge_wakelock->active) {
			pr_info("hold wakelock!!! usb_type:[%d,%d], status:%d\n ",
				pst->prop[USB_ADAP_TYPE], wls_type, pst_battery->prop[BATT_STATUS]);
			__pm_stay_awake(bcdev->charge_wakelock);
		}
	} else if (wls_read_ok && bcdev->charge_wakelock->active) {
		pr_info("release wakelock!!! usb_type:[%d,%d], status:%d\n ",
			pst->prop[USB_ADAP_TYPE], wls_type, pst_battery->prop[BATT_STATUS]);
		__pm_relax(bcdev->charge_wakelock);
	}

	/* Stock's UNKNOWN branch preserves ICL and skips the second-port update. */
	switch (pst->prop[USB_ADAP_TYPE]) {
	case POWER_SUPPLY_USB_TYPE_UNKNOWN:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
		return;
	case POWER_SUPPLY_USB_TYPE_SDP:
	case POWER_SUPPLY_USB_TYPE_CDP:
	case POWER_SUPPLY_USB_TYPE_PD:
		break;
	default:
		bcdev->usb_icl_ua = 0;
		break;
	}

	switch (pst->prop[USB_ADAP_TYPE]) {
	case POWER_SUPPLY_USB_TYPE_SDP:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
	case POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case POWER_SUPPLY_USB_TYPE_CDP:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case POWER_SUPPLY_USB_TYPE_ACA:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case POWER_SUPPLY_USB_TYPE_C:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_TYPE_C;
		break;
	case POWER_SUPPLY_USB_TYPE_PD:
	case POWER_SUPPLY_USB_TYPE_PD_DRP:
	case POWER_SUPPLY_USB_TYPE_PD_PPS:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_PD;
		break;
	default:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
		break;
	}


	if (wls_read_ok) {
		switch (wls_type) {
		case POWER_SUPPLY_USB_TYPE_SDP:
			wls_psy_desc.type = POWER_SUPPLY_TYPE_USB;
			break;
		case POWER_SUPPLY_USB_TYPE_DCP:
		case POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID:
		case QTI_POWER_SUPPLY_USB_TYPE_HVDCP:
		case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3:
		case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5:
			wls_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		case POWER_SUPPLY_USB_TYPE_CDP:
			wls_psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
			break;
		case POWER_SUPPLY_USB_TYPE_ACA:
			wls_psy_desc.type = POWER_SUPPLY_TYPE_USB_ACA;
			break;
		case POWER_SUPPLY_USB_TYPE_C:
			wls_psy_desc.type = POWER_SUPPLY_TYPE_USB_TYPE_C;
			break;
		case POWER_SUPPLY_USB_TYPE_PD:
		case POWER_SUPPLY_USB_TYPE_PD_DRP:
		case POWER_SUPPLY_USB_TYPE_PD_PPS:
			wls_psy_desc.type = POWER_SUPPLY_TYPE_USB_PD;
			break;
		default:
			wls_psy_desc.type = POWER_SUPPLY_TYPE_USB;
			break;
		}
	}
	if (!bcdev->lenovo_chg_monitor_started)
		mod_delayed_work(system_wq, &bcdev->lenovo_chg_monitor_work, 0);

}

static void battery_chg_check_status_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev,
					battery_check_work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	int level = 0;
	struct psy_state *pst_hq = &bcdev->psy_list[PSY_TYPE_HQ];
	char *batt_envp[5][2] = {
		{"BATTERY_STATE=LENOVO_CHARGE_EVENT_0",	NULL}, 	//battery good
		{"BATTERY_STATE=LENOVO_CHARGE_EVENT_1",	NULL}, 	//battery failure or missing
		{"BATTERY_STATE=LENOVO_CHARGE_EVENT_2",	NULL},	//dual battery are not same supplier
		{"BATTERY_STATE=LENOVO_CHARGE_EVENT_3",	NULL},	//dual battery time diff one year
		{"BATTERY_STATE=LENOVO_CHARGE_EVENT_4", NULL},
	};

	if (bcdev->battery_abnormal) {
		rc = read_property_id(bcdev, pst_hq, HQ_POWER_SUPPLY_PROP_BATTERY_ABNORMAL_STATE);
		if (rc < 0) {
			pr_err("Failed to read BATTERY_ABNORMAL_STATE, rc=%d\n", rc);
			return;
		}

		if (bcdev->fake_abnormal_level >=0 && bcdev->fake_abnormal_level <= 4)
			level = bcdev->fake_abnormal_level;
		else
			level = pst_hq->prop[HQ_POWER_SUPPLY_PROP_BATTERY_ABNORMAL_STATE];

		pr_info("[LENOVO_BATT_CHECK] : BATTERY_ABNORMAL_STATE, level=%d\n", level);
		kobject_uevent_env(&bcdev->dev->kobj,KOBJ_CHANGE, batt_envp[level]);
		bcdev->battery_abnormal = false;
	}

	rc = read_property_id(bcdev, pst, BATT_STATUS);
	if (rc < 0) {
		pr_err("Failed to read BATT_STATUS, rc=%d\n", rc);
		return;
	}

	if (pst->prop[BATT_STATUS] == POWER_SUPPLY_STATUS_CHARGING) {
		pr_debug("Battery is charging\n");
		return;
	}

	rc = read_property_id(bcdev, pst, BATT_CAPACITY);
	if (rc < 0) {
		pr_err("Failed to read BATT_CAPACITY, rc=%d\n", rc);
		return;
	}

	if (DIV_ROUND_CLOSEST(pst->prop[BATT_CAPACITY], 100) > 0) {
		pr_debug("Battery SOC is > 0\n");
		return;
	}

	/*
	 * If we are here, then battery is not charging and SOC is 0.
	 * Check the battery voltage and if it's lower than shutdown voltage,
	 * then initiate an emergency shutdown.
	 */

	rc = read_property_id(bcdev, pst, BATT_VOLT_NOW);
	if (rc < 0) {
		pr_err("Failed to read BATT_VOLT_NOW, rc=%d\n", rc);
		return;
	}

	if (pst->prop[BATT_VOLT_NOW] / 1000 > bcdev->shutdown_volt_mv) {
		pr_debug("Battery voltage is > %d mV\n",
			bcdev->shutdown_volt_mv);
		return;
	}

	pr_emerg("Initiating a shutdown in 100 ms\n");
	msleep(100);
	pr_emerg("Attempting kernel_power_off: Battery voltage low\n");
	kernel_power_off();
}

static void handle_notification(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_notify_msg *notify_msg = data;
	struct psy_state *pst = NULL;
	u32 hboost_vmax_mv, notification;

	if (len != sizeof(*notify_msg)) {
		pr_err("Incorrect response length %zu\n", len);
		return;
	}

	notification = notify_msg->notification;
	pr_debug("notification: %#x\n", notification);
	if ((notification & 0xffff) == BC_HBOOST_VMAX_CLAMP_NOTIFY) {
		hboost_vmax_mv = (notification >> 16) & 0xffff;
		raw_notifier_call_chain(&hboost_notifier, VMAX_CLAMP, &hboost_vmax_mv);
		pr_debug("hBoost is clamped at %u mV\n", hboost_vmax_mv);
		return;
	}

	switch (notification) {
	case BC_BATTERY_STATUS_GET:
	case BC_GENERIC_NOTIFY:
		schedule_work(&bcdev->usb_type_work);
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		if (bcdev->shutdown_volt_mv > 0)
			schedule_work(&bcdev->battery_check_work);
		break;
	case BC_USB_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		schedule_work(&bcdev->usb_type_work);
		break;
	case BC_USB2_STATUS_GET:
		if (!bcdev->usb2_enabled)
			break;
		pst = &bcdev->psy_list[PSY_TYPE_USB2];
		if (!READ_ONCE(pst->timed_out))
			schedule_work(&bcdev->usb_type_work);
		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];
		schedule_work(&bcdev->usb_type_work);
		break;
	case HQ_POWER_SUPPLY_GET_REQ:
		pst = &bcdev->psy_list[PSY_TYPE_HQ];
		break;
	case BC_BATT_ABNORMAL_NOTIFY:
		bcdev->battery_abnormal = true;
		schedule_work(&bcdev->battery_check_work);
		break;
	default:
		break;
	}

	switch (notification) {
	case BC_USB_STATUS_GET:
	case BC_USB2_STATUS_GET:
	case BC_WLS_STATUS_GET:
		kick_bank_probe(bcdev, 0);
		break;
	default:
		break;
	}

	if (pst && pst->psy) {
		/*
		 * For charger mode, keep the device awake at least for 50 ms
		 * so that device won't enter suspend when a non-SDP charger
		 * is removed. This would allow the userspace process like
		 * "charger" to be able to read power supply uevents to take
		 * appropriate actions (e.g. shutting down when the charger is
		 * unplugged).
		 */
		power_supply_changed(pst->psy);
		pm_wakeup_dev_event(bcdev->dev, 50, true);
	}
}

static int battery_chg_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct battery_chg_dev *bcdev = priv;

	pr_debug("owner: %u type: %u opcode: %#x len: %zu\n", hdr->owner,
		hdr->type, hdr->opcode, len);

	down_read(&bcdev->state_sem);
	if (!bcdev->initialized) {
		pr_debug("Driver initialization failed: Dropping glink callback message: state %d\n",
			 atomic_read(&bcdev->state));
		up_read(&bcdev->state_sem);
		return 0;
	}
	up_read(&bcdev->state_sem);

	if (hdr->opcode == BC_NOTIFY_IND)
		handle_notification(bcdev, data, len);
	else
		handle_message(bcdev, data, len);

	return 0;
}

static int wls_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int prop_id, rc;

	pval->intval = -ENODATA;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];

	return 0;
}

static int wls_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	return 0;
}

static int wls_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	return 0;
}

static enum power_supply_property wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_TEMP,
};

static struct power_supply_desc wls_psy_desc = {
	.name			= "wireless",
	/* Stock starts both connector supplies as USB until type work runs. */
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= wls_props,
	.num_properties		= ARRAY_SIZE(wls_props),
	.get_property		= wls_psy_get_prop,
	.set_property		= wls_psy_set_prop,
	.property_is_writeable	= wls_psy_prop_is_writeable,
};

static const char *get_wls_type_name(u32 wls_type)
{
	if (wls_type >= ARRAY_SIZE(qc_power_supply_wls_type_text))
		return "Unknown";

	return qc_power_supply_wls_type_text[wls_type];
}

static const char *get_usb_type_name(u32 usb_type)
{
	u32 i;

	if (usb_type >= QTI_POWER_SUPPLY_USB_TYPE_HVDCP &&
	    usb_type <= QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5) {
		for (i = 0; i < ARRAY_SIZE(qc_power_supply_usb_type_text);
		     i++) {
			if (i == (usb_type - QTI_POWER_SUPPLY_USB_TYPE_HVDCP))
				return qc_power_supply_usb_type_text[i];
		}
		return "Unknown";
	}

	for (i = 0; i < ARRAY_SIZE(power_supply_usb_type_text); i++) {
		if (i == usb_type)
			return power_supply_usb_type_text[i];
	}

	return "Unknown";
}

static int usb_psy_set_icl(struct battery_chg_dev *bcdev, u32 prop_id, int val)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	u32 temp;
	int rc;

	rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0) {
		pr_err("Failed to read prop USB_ADAP_TYPE, rc=%d\n", rc);
		return rc;
	}

	/* Allow this only for SDP, CDP or USB_PD and not for other charger types */
	switch (pst->prop[USB_ADAP_TYPE]) {
	case POWER_SUPPLY_USB_TYPE_SDP:
	case POWER_SUPPLY_USB_TYPE_PD:
	case POWER_SUPPLY_USB_TYPE_CDP:
		break;
	default:
		bcdev->usb_icl_ua = 0;
		return -EINVAL;
	}

	/*
	 * Input current limit (ICL) can be set by different clients. E.g. USB
	 * driver can request for a current of 500/900 mA depending on the
	 * port type. Also, clients like EUD driver can pass 0 or -22 to
	 * suspend or unsuspend the input for its use case.
	 */

	temp = val;
	if (val < 0)
		temp = UINT_MAX;

	rc = write_property_id(bcdev, pst, prop_id, temp);
	if (rc < 0) {
		pr_err("Failed to set ICL (%u uA) rc=%d\n", temp, rc);
	} else {
		pr_debug("Set ICL to %u\n", temp);
		bcdev->usb_icl_ua = temp;
	}

	return rc;
}

static int dual_usb_usb2_set_icl(struct battery_chg_dev *bcdev, u32 prop_id, int val)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB2];
	u32 temp;
	int rc;

	rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0) {
		pr_err("Failed to read prop USB_ADAP_TYPE, rc=%d\n", rc);
		return rc;
	}

	/* Allow this only for SDP, CDP or USB_PD and not for other charger types */
	switch (pst->prop[USB_ADAP_TYPE]) {
	case POWER_SUPPLY_USB_TYPE_SDP:
	case POWER_SUPPLY_USB_TYPE_PD:
	case POWER_SUPPLY_USB_TYPE_CDP:
		break;
	default:
		bcdev->usb2_icl_ua = 0;
		return -EINVAL;
	}

	/*
	 * Input current limit (ICL) can be set by different clients. E.g. USB
	 * driver can request for a current of 500/900 mA depending on the
	 * port type. Also, clients like EUD driver can pass 0 or -22 to
	 * suspend or unsuspend the input for its use case.
	 */

	temp = val;
	if (val < 0)
		temp = UINT_MAX;

	rc = write_property_id(bcdev, pst, prop_id, temp);
	if (rc < 0) {
		pr_err("Failed to set ICL (%u uA) rc=%d\n", temp, rc);
	} else {
		pr_debug("Set ICL to %u\n", temp);
		bcdev->usb2_icl_ua = temp;
	}

	return rc;
}

static int usb_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int prop_id, rc;

	pval->intval = -ENODATA;

	if (psy == bcdev->psy_list[PSY_TYPE_USB2].psy)
		pst = &bcdev->psy_list[PSY_TYPE_USB2];
	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];
	if (prop == POWER_SUPPLY_PROP_TEMP)
		pval->intval = DIV_ROUND_CLOSEST((int)pval->intval, 10);

	return 0;
}

static int usb_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int prop_id, rc = 0;

	if (psy == bcdev->psy_list[PSY_TYPE_USB2].psy)
		pst = &bcdev->psy_list[PSY_TYPE_USB2];
	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		pr_info("%s online:%d\n", __func__, pval->intval);
		rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_HQ],
				HQ_POWER_SUPPLY_PROP_RESERVED_56, pval->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (psy == bcdev->psy_list[PSY_TYPE_USB2].psy)
			rc = dual_usb_usb2_set_icl(bcdev, prop_id, pval->intval);
		else
		rc = usb_psy_set_icl(bcdev, prop_id, pval->intval);
		break;
	default:
		break;
	}

	return rc;
}

static int usb_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_TEMP,
};


static struct power_supply_desc usb_psy_desc = {
		.name			= "usb",
		.type			= POWER_SUPPLY_TYPE_USB,
		.properties		= usb_props,
		.num_properties		= ARRAY_SIZE(usb_props),
		.get_property		= usb_psy_get_prop,
		.set_property		= usb_psy_set_prop,
		.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
			     BIT(POWER_SUPPLY_USB_TYPE_SDP)     |
			     BIT(POWER_SUPPLY_USB_TYPE_DCP)     |
			     BIT(POWER_SUPPLY_USB_TYPE_CDP)     |
			     BIT(POWER_SUPPLY_USB_TYPE_ACA)     |
			     BIT(POWER_SUPPLY_USB_TYPE_C)       |
			     BIT(POWER_SUPPLY_USB_TYPE_PD)      |
			     BIT(POWER_SUPPLY_USB_TYPE_PD_DRP)  |
			     BIT(POWER_SUPPLY_USB_TYPE_PD_PPS)  |
			     BIT(POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID),
		.property_is_writeable	= usb_psy_prop_is_writeable,
};

#define CHARGE_CTRL_START_THR_MIN	50
#define CHARGE_CTRL_START_THR_MAX	95
#define CHARGE_CTRL_END_THR_MIN		55
#define CHARGE_CTRL_END_THR_MAX		100
#define CHARGE_CTRL_DELTA_SOC		5

static int battery_psy_set_charge_threshold(struct battery_chg_dev *bcdev,
					u32 target_soc, u32 delta_soc)
{
	struct battery_charger_chg_ctrl_msg msg = { { 0 } };
	int rc;

	if (!bcdev->chg_ctrl_en)
		return 0;

	if (target_soc > CHARGE_CTRL_END_THR_MAX)
		target_soc = CHARGE_CTRL_END_THR_MAX;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_CHG_CTRL_LIMIT_EN;
	/* Stock leaves enable zero for threshold updates (opcode 0x48). */
	msg.enable = 0;
	msg.target_soc = target_soc;
	msg.delta_soc = delta_soc;

	rc = battery_chg_write(bcdev, &msg, sizeof(msg), BC_WAIT_TIME_MS);
	if (rc < 0)
		pr_err("Failed to set charge_control thresholds, rc=%d\n", rc);
	else
		pr_debug("target_soc: %u delta_soc: %u\n", target_soc, delta_soc);

	return rc;
}

static int battery_psy_set_charge_end_threshold(struct battery_chg_dev *bcdev,
					int val)
{
	u32 delta_soc = CHARGE_CTRL_DELTA_SOC;
	int rc;

	if (val < CHARGE_CTRL_END_THR_MIN ||
	    val > CHARGE_CTRL_END_THR_MAX) {
		pr_err("Charge control end_threshold should be within [%u %u]\n",
			CHARGE_CTRL_END_THR_MIN, CHARGE_CTRL_END_THR_MAX);
		return -EINVAL;
	}

	if (bcdev->chg_ctrl_start_thr && val > bcdev->chg_ctrl_start_thr)
		delta_soc = val - bcdev->chg_ctrl_start_thr;

	rc = battery_psy_set_charge_threshold(bcdev, val, delta_soc);
	if (rc < 0)
		pr_err("Failed to set charge control end threshold %u, rc=%d\n",
			val, rc);
	else
		bcdev->chg_ctrl_end_thr = val;

	return rc;
}

static int battery_psy_set_charge_start_threshold(struct battery_chg_dev *bcdev,
					int val)
{
	u32 target_soc, delta_soc;
	int rc;

	if (val < CHARGE_CTRL_START_THR_MIN ||
	    val > CHARGE_CTRL_START_THR_MAX) {
		pr_err("Charge control start_threshold should be within [%u %u]\n",
			CHARGE_CTRL_START_THR_MIN, CHARGE_CTRL_START_THR_MAX);
		return -EINVAL;
	}

	if (val > bcdev->chg_ctrl_end_thr) {
		target_soc = val +  CHARGE_CTRL_DELTA_SOC;
		delta_soc = CHARGE_CTRL_DELTA_SOC;
	} else {
		target_soc = bcdev->chg_ctrl_end_thr;
		delta_soc = bcdev->chg_ctrl_end_thr - val;
	}

	rc = battery_psy_set_charge_threshold(bcdev, target_soc, delta_soc);
	if (rc < 0)
		pr_err("Failed to set charge control start threshold %u, rc=%d\n",
			val, rc);
	else
		bcdev->chg_ctrl_start_thr = val;

	return rc;
}

static int get_charge_control_en(struct battery_chg_dev *bcdev)
{
	int rc;

	rc = read_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHG_CTRL_EN);
	if (rc < 0)
		pr_err("Failed to read the CHG_CTRL_EN, rc = %d\n", rc);
	else
		bcdev->chg_ctrl_en =
			bcdev->psy_list[PSY_TYPE_BATTERY].prop[BATT_CHG_CTRL_EN];

	return rc;
}

static int __battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					u32 fcc_ua)
{
	int rc;

	if (bcdev->restrict_chg_en) {
		fcc_ua = min_t(u32, fcc_ua, bcdev->restrict_fcc_ua);
		fcc_ua = min_t(u32, fcc_ua, bcdev->thermal_fcc_ua);
	}

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHG_CTRL_LIM, fcc_ua);
	if (rc < 0) {
		pr_err("Failed to set FCC %u, rc=%d\n", fcc_ua, rc);
	} else {
		pr_debug("Set FCC to %u uA\n", fcc_ua);
		bcdev->last_fcc_ua = fcc_ua;
	}

	return rc;
}

static int battery_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop, union power_supply_propval *val);
int qti_battery_charger_set_charge_current_ua(int current_ua);
int qti_battery_charger_set_charge_current_ua(int current_ua)
{
	struct power_supply *psy;
	struct battery_chg_dev *bcdev;
	int ret;

	if (current_ua < 0)
		return -EINVAL;
	psy = power_supply_get_by_name("battery");
	if (!psy)
		return -ENODEV;
	bcdev = power_supply_get_drvdata(psy);
	if (!bcdev || psy->desc->get_property != battery_psy_get_prop) {
		ret = -ENODEV;
	} else {
		/* Preserve any tighter limit already selected by the gen6 driver. */
		if (bcdev->thermal_fcc_ua)
			current_ua = min_t(u32, current_ua, bcdev->thermal_fcc_ua);
		ret = __battery_psy_set_charge_current(bcdev, current_ua);
	}
	power_supply_put(psy);
	return ret;
}
EXPORT_SYMBOL_GPL(qti_battery_charger_set_charge_current_ua);

static int battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					int val)
{
	int rc;
	u32 fcc_ua, prev_fcc_ua;

	if (!bcdev->num_thermal_levels)
		return 0;

	if (bcdev->num_thermal_levels < 0) {
		pr_err("Incorrect num_thermal_levels\n");
		return -EINVAL;
	}

	if (val < 0 || val > bcdev->num_thermal_levels)
		return -EINVAL;

	if (bcdev->thermal_fcc_step == 0)
		fcc_ua = bcdev->thermal_levels[val];
	else
		fcc_ua = bcdev->psy_list[PSY_TYPE_BATTERY].prop[BATT_CHG_CTRL_LIM_MAX]
				- (bcdev->thermal_fcc_step * val);

	prev_fcc_ua = bcdev->thermal_fcc_ua;
	bcdev->thermal_fcc_ua = fcc_ua;

	rc = __battery_psy_set_charge_current(bcdev, fcc_ua);
	if (!rc)
		bcdev->curr_thermal_level = val;
	else
		bcdev->thermal_fcc_ua = prev_fcc_ua;

	pr_info("Lenovo Thermal limit fcc_ua:%d\n", fcc_ua);
	return rc;
}

static int battery_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int prop_id, rc;


	pval->intval = -ENODATA;

	/*
	 * The prop id of TIME_TO_FULL_NOW and TIME_TO_FULL_AVG is same.
	 * So, map the prop id of TIME_TO_FULL_AVG for TIME_TO_FULL_NOW.
	 */
	if (prop == POWER_SUPPLY_PROP_TIME_TO_FULL_NOW)
		prop = POWER_SUPPLY_PROP_TIME_TO_FULL_AVG;
	if (prop == POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW)
		prop = POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		pval->strval = pst->model;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		pval->intval = DIV_ROUND_CLOSEST(pst->prop[prop_id], 100);
		if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) &&
		   (bcdev->fake_soc >= 0 && bcdev->fake_soc <= 100))
			pval->intval = bcdev->fake_soc;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		pval->intval = DIV_ROUND_CLOSEST((int)pst->prop[prop_id], 10);
		break;
	default:
		pval->intval = pst->prop[prop_id];
		break;
	}

	return rc;
}

static int battery_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		return battery_psy_set_charge_start_threshold(bcdev,
								pval->intval);
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return battery_psy_set_charge_end_threshold(bcdev,
								pval->intval);
	default:
		return -EINVAL;
	}

	return 0;
}

static int battery_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		return 1;
	default:
		break;
	}

	return 0;
}

static const enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
};

static const struct power_supply_desc batt_psy_desc = {
	.name			= "battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= battery_props,
	.num_properties		= ARRAY_SIZE(battery_props),
	.get_property		= battery_psy_get_prop,
	.set_property		= battery_psy_set_prop,
	.property_is_writeable	= battery_psy_prop_is_writeable,
};

static const struct attribute_group dual_usb_reset_group;
static const struct attribute_group *dual_usb_reset_groups[] = {
	&dual_usb_reset_group,
	NULL,
};

static int battery_chg_init_psy(struct battery_chg_dev *bcdev)
{
	struct power_supply_config psy_cfg = {};
	int rc;

	psy_cfg.drv_data = bcdev;
	psy_cfg.of_node = bcdev->dev->of_node;
	if (bcdev->psy_list[PSY_TYPE_USB2].prop_count)
		psy_cfg.attr_grp = dual_usb_reset_groups;
	bcdev->psy_list[PSY_TYPE_USB].psy =
		devm_power_supply_register(bcdev->dev, &usb_psy_desc, &psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_USB].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_USB].psy);
		bcdev->psy_list[PSY_TYPE_USB].psy = NULL;
		pr_err("Failed to register USB power supply, rc=%d\n", rc);
		return rc;
	}

	if (bcdev->psy_list[PSY_TYPE_USB2].prop_count) {
		struct psy_state *usb = &bcdev->psy_list[PSY_TYPE_USB];
		u32 ports = of_property_read_bool(bcdev->dev->of_node,
						 "qcom,multiport-usb") ? 2 : 1;

		/* Confirm the ADSP port count. */
		rc = read_property_id(bcdev, usb, DUAL_USB_USB_NUM_PORTS);
		bcdev->usb_num_ports = rc ? 1 : usb->prop[DUAL_USB_USB_NUM_PORTS];
		bcdev->usb2_enabled = !rc && ports == 2 &&
			usb->prop[DUAL_USB_USB_NUM_PORTS] == ports;
	}
	if (bcdev->usb2_enabled) {
		dual_usb_usb2_psy_desc = usb_psy_desc;
		dual_usb_usb2_psy_desc.name = "usb2";
		bcdev->psy_list[PSY_TYPE_USB2].psy = devm_power_supply_register(
			bcdev->dev, &dual_usb_usb2_psy_desc, &psy_cfg);
		if (IS_ERR(bcdev->psy_list[PSY_TYPE_USB2].psy)) {
			rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_USB2].psy);
			bcdev->psy_list[PSY_TYPE_USB2].psy = NULL;
			return rc;
		}
	}

	bcdev->psy_list[PSY_TYPE_WLS].psy =
		devm_power_supply_register(bcdev->dev, &wls_psy_desc, &psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy);
		bcdev->psy_list[PSY_TYPE_WLS].psy = NULL;
		pr_err("Failed to register wireless power supply, rc=%d\n", rc);
		return rc;
	}

	bcdev->psy_list[PSY_TYPE_BATTERY].psy =
		devm_power_supply_register(bcdev->dev, &batt_psy_desc,
						&psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy);
		bcdev->psy_list[PSY_TYPE_BATTERY].psy = NULL;
		pr_err("Failed to register battery power supply, rc=%d\n", rc);
		return rc;
	}


	return 0;
}

static void battery_chg_subsys_up_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, subsys_up_work);
	int rc;

	battery_chg_notify_enable(bcdev);

	/*
	 * Give some time after enabling notification so that USB adapter type
	 * information can be obtained properly which is essential for setting
	 * USB ICL.
	 */
	msleep(200);

	if (bcdev->last_fcc_ua) {
		rc = __battery_psy_set_charge_current(bcdev,
				bcdev->last_fcc_ua);
		if (rc < 0)
			pr_err("Failed to set FCC (%u uA), rc=%d\n",
				bcdev->last_fcc_ua, rc);
	}

	if (bcdev->usb_icl_ua) {
		rc = usb_psy_set_icl(bcdev, USB_INPUT_CURR_LIMIT,
				bcdev->usb_icl_ua);
		if (rc < 0)
			pr_err("Failed to set ICL(%u uA), rc=%d\n",
				bcdev->usb_icl_ua, rc);
	}
	if (bcdev->usb2_icl_ua)
		dual_usb_usb2_set_icl(bcdev, USB_INPUT_CURR_LIMIT, bcdev->usb2_icl_ua);

	kick_bank_probe(bcdev, 0);
}

static int wireless_fw_send_firmware(struct battery_chg_dev *bcdev,
					const struct firmware *fw)
{
	struct wireless_fw_push_buf_req msg = {};
	const u8 *ptr;
	u32 i, buf_size, num_chunks, partial_chunk_size;
	int rc;

	if (strstr(bcdev->wls_fw_name, "cps"))
		buf_size = CPS_WLS_FW_BUF_SIZE;
	else
		buf_size = WLS_FW_BUF_SIZE;

	num_chunks = fw->size / buf_size;
	partial_chunk_size = fw->size % buf_size;

	if (!num_chunks)
		return -EINVAL;

	pr_debug("Updating FW...\n");

	ptr = fw->data;
	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_WLS_FW_PUSH_BUF_REQ;

	for (i = 0; i < num_chunks; i++, ptr += buf_size) {
		msg.fw_chunk_id = i + 1;
		memcpy(msg.buf, ptr, buf_size);

		pr_debug("sending FW chunk %u\n", i + 1);
		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	if (partial_chunk_size) {
		msg.fw_chunk_id = i + 1;
		memset(msg.buf, 0, buf_size);
		memcpy(msg.buf, ptr, partial_chunk_size);

		pr_debug("sending partial FW chunk %u\n", i + 1);
		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	return 0;
}

static int wireless_fw_check_for_update(struct battery_chg_dev *bcdev,
					u32 version, size_t size)
{
	struct wireless_fw_check_req req_msg = {};

	bcdev->wls_fw_update_reqd = false;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_CHECK_UPDATE;
	req_msg.fw_version = version;
	req_msg.fw_size = size;
	req_msg.fw_crc = bcdev->wls_fw_crc;

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg), BC_FW_VER_WAIT_TIME_MS);
}

#define IDT9415_FW_MAJOR_VER_OFFSET		0x84
#define IDT9415_FW_MINOR_VER_OFFSET		0x86
#define IDT_FW_MAJOR_VER_OFFSET		0x94
#define IDT_FW_MINOR_VER_OFFSET		0x96
#define CPS_FW_MAJOR_VER_OFFSET		0xC4
#define CPS_FW_MINOR_VER_OFFSET		0xC5

static u32 wireless_fw_version_get(struct battery_chg_dev *bcdev, const u8 *data, bool force)
{
	u16 maj_ver = 0, min_ver = 0;
	u32 version = 0;

	if (force)
		return UINT_MAX;

	if (strstr(bcdev->wls_fw_name, "idt9412")) {
		maj_ver = le16_to_cpu(*(__le16 *)(data + IDT_FW_MAJOR_VER_OFFSET));
		min_ver = le16_to_cpu(*(__le16 *)(data + IDT_FW_MINOR_VER_OFFSET));
		version = (maj_ver << 16 | min_ver);
	} else if (strstr(bcdev->wls_fw_name, "idt")) {
		maj_ver = le16_to_cpu(*(__le16 *)(data + IDT9415_FW_MAJOR_VER_OFFSET));
		min_ver = le16_to_cpu(*(__le16 *)(data + IDT9415_FW_MINOR_VER_OFFSET));
		version = (maj_ver << 16 | min_ver);
	} else if (strstr(bcdev->wls_fw_name, "cps"))
		version = be16_to_cpu(*(__be16 *)(data + CPS_FW_MAJOR_VER_OFFSET));

	return version;
}

static int wireless_fw_update(struct battery_chg_dev *bcdev, bool force)
{
	const struct firmware *fw;
	struct psy_state *pst;
	u32 version;
	int rc;

	if (!bcdev->wls_fw_name) {
		pr_err("wireless FW name is not specified\n");
		return -EINVAL;
	}

	pm_stay_awake(bcdev->dev);

	/*
	 * Check for USB presence. If nothing is connected, check whether
	 * battery SOC is at least 50% before allowing FW update.
	 */
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	rc = read_property_id(bcdev, pst, USB_ONLINE);
	if (rc < 0)
		goto out;


	if (!pst->prop[USB_ONLINE]) {
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(bcdev, pst, BATT_CAPACITY);
		if (rc < 0)
			goto out;

		if ((pst->prop[BATT_CAPACITY] / 100) < 50) {
			pr_err("Battery SOC should be at least 50%% or connect charger\n");
			rc = -EINVAL;
			goto out;
		}
	}

	rc = firmware_request_nowarn(&fw, bcdev->wls_fw_name, bcdev->dev);
	if (rc) {
		pr_err("Couldn't get firmware rc=%d\n", rc);
		goto out;
	}

	if (!fw || !fw->data || !fw->size) {
		pr_err("Invalid firmware\n");
		rc = -EINVAL;
		goto release_fw;
	}

	if (fw->size < SZ_16K) {
		pr_err("Invalid firmware size %zu\n", fw->size);
		rc = -EINVAL;
		goto release_fw;
	}

	version = wireless_fw_version_get(bcdev, fw->data, force);

	pr_debug("FW size: %zu version: %#x\n", fw->size, version);

	rc = wireless_fw_check_for_update(bcdev, version, fw->size);
	if (rc < 0) {
		pr_err("Wireless FW update not needed, rc=%d\n", rc);
		goto release_fw;
	}

	if (!bcdev->wls_fw_update_reqd) {
		pr_warn("Wireless FW update not required\n");
		goto release_fw;
	}

	/* Wait for IDT to be setup by charger firmware */
	msleep(WLS_FW_PREPARE_TIME_MS);

	reinit_completion(&bcdev->fw_update_ack);
	rc = wireless_fw_send_firmware(bcdev, fw);
	if (rc < 0) {
		pr_err("Failed to send FW chunk, rc=%d\n", rc);
		goto release_fw;
	}

	pr_debug("Waiting for fw_update_ack\n");
	rc = wait_for_completion_timeout(&bcdev->fw_update_ack,
				msecs_to_jiffies(bcdev->wls_fw_update_time_ms));
	if (!rc) {
		pr_err("Error, timed out updating firmware\n");
		rc = -ETIMEDOUT;
		goto release_fw;
	} else {
		pr_debug("Waited for %d ms\n",
			bcdev->wls_fw_update_time_ms - jiffies_to_msecs(rc));
		rc = 0;
	}

	pr_info("Wireless FW update done\n");

release_fw:
	bcdev->wls_fw_crc = 0;
	release_firmware(fw);
out:
	pm_relax(bcdev->dev);

	return rc;
}

static ssize_t qti_charger_ro_show(const struct class *c,
		const struct class_attribute *attr, char *buf,
		u32 psy_type, u32 prop_id)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[psy_type];
	int ret = read_property_id(bcdev, pst, prop_id);

	return ret < 0 ? ret : sysfs_emit(buf, "%d\n", pst->prop[prop_id]);
}

#define QTI_CHARGER_RO_SHOW(name, psy_type, prop_id)			\
	static ssize_t name##_show(const struct class *c,			\
					const struct class_attribute *attr,	\
					char *buf)			\
	{								\
		return qti_charger_ro_show(c, attr, buf, psy_type, prop_id); \
	}								\
	static CLASS_ATTR_RO(name)

#define QTI_CHARGER_RW_SHOW(name, psy_type, prop_id)			\
	static ssize_t name##_show(const struct class *c,			\
					const struct class_attribute *attr,	\
					char *buf)			\
	{								\
		return qti_charger_ro_show(c, attr, buf, psy_type, prop_id); \
	}								\
	static CLASS_ATTR_RW(name)

#define QTI_CHARGER_RW_PROP_SHOW(name, field)				\
	static ssize_t name##_show(const struct class *c,			\
					const struct class_attribute *attr,	\
					char *buf)			\
	{								\
		struct battery_chg_dev *bcdev = container_of(c,		\
				struct battery_chg_dev, battery_class);	\
		return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->field);	\
	}								\
	static CLASS_ATTR_RW(name)

#define QTI_CHARGER_TYPE_RO_SHOW(name, psy, psy_type, prop_id)		\
	static ssize_t name##_show(const struct class *c,			\
					const struct class_attribute *attr,	\
					char *buf)			\
	{								\
		struct battery_chg_dev *bcdev = container_of(c,		\
				struct battery_chg_dev, battery_class);	\
		struct psy_state *pst = &bcdev->psy_list[psy_type];	\
		int rc;							\
									\
		rc = read_property_id(bcdev, pst, prop_id);		\
		if (rc < 0)						\
			return rc;					\
									\
		return scnprintf(buf, PAGE_SIZE, "%s\n",		\
				get_##psy##_type_name(pst->prop[prop_id]));	\
	}								\
	static CLASS_ATTR_RO(name)

static ssize_t wireless_fw_update_time_ms_store(const struct class *c,
				const struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	if (kstrtou32(buf, 0, &bcdev->wls_fw_update_time_ms))
		return -EINVAL;

	return count;
}

static ssize_t wireless_fw_update_time_ms_show(const struct class *c,
		const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->wls_fw_update_time_ms);
}
static CLASS_ATTR_RW(wireless_fw_update_time_ms);

static ssize_t wireless_fw_crc_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	u16 val;

	if (kstrtou16(buf, 0, &val) || !val)
		return -EINVAL;

	bcdev->wls_fw_crc = val;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_crc);

static ssize_t wireless_fw_version_show(const struct class *c,
					const struct class_attribute *attr,
					char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct wireless_fw_get_version_req req_msg = {};
	int rc;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_GET_VERSION;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg), BC_FW_VER_WAIT_TIME_MS);
	if (rc < 0) {
		pr_err("Failed to get FW version rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%#x\n", bcdev->wls_fw_version);
}
static CLASS_ATTR_RO(wireless_fw_version);

static ssize_t wireless_fw_force_update_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, true);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_force_update);

static ssize_t wireless_fw_update_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, false);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_update);

QTI_CHARGER_TYPE_RO_SHOW(wireless_type, wls, PSY_TYPE_WLS, WLS_ADAP_TYPE);

static ssize_t charge_control_en_store(const struct class *c,
				const struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	if (val == bcdev->chg_ctrl_en)
		return count;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHG_CTRL_EN, val);
	if (rc < 0) {
		pr_err("Failed to set charge_control_en, rc=%d\n", rc);
		return rc;
	}

	bcdev->chg_ctrl_en = val;

	return count;
}

static ssize_t charge_control_en_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;

	rc = get_charge_control_en(bcdev);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->chg_ctrl_en);
}
static CLASS_ATTR_RW(charge_control_en);

QTI_CHARGER_TYPE_RO_SHOW(usb_2_real_type, usb, PSY_TYPE_USB2, USB_REAL_TYPE);

static ssize_t usb_num_ports_show(const struct class *c,
		const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc = read_property_id(bcdev, pst, DUAL_USB_USB_NUM_PORTS);

	return rc ? rc : scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[DUAL_USB_USB_NUM_PORTS]);
}
static CLASS_ATTR_RO(usb_num_ports);

QTI_CHARGER_TYPE_RO_SHOW(usb_real_type, usb, PSY_TYPE_USB, USB_REAL_TYPE);


static ssize_t usb_typec_compliant_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_TYPEC_COMPLIANT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			(int)pst->prop[USB_TYPEC_COMPLIANT]);
}
static CLASS_ATTR_RO(usb_typec_compliant);

static ssize_t restrict_cur_store(const struct class *c, const struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 fcc_ua, prev_fcc_ua;

	if (kstrtou32(buf, 0, &fcc_ua) || fcc_ua > bcdev->thermal_fcc_ua)
		return -EINVAL;

	prev_fcc_ua = bcdev->restrict_fcc_ua;
	bcdev->restrict_fcc_ua = fcc_ua;
	if (bcdev->restrict_chg_en) {
		rc = __battery_psy_set_charge_current(bcdev, fcc_ua);
		if (rc < 0) {
			bcdev->restrict_fcc_ua = prev_fcc_ua;
			return rc;
		}
	}

	return count;
}

static ssize_t restrict_cur_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->restrict_fcc_ua);
}
static CLASS_ATTR_RW(restrict_cur);

static ssize_t restrict_chg_store(const struct class *c, const struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	bcdev->restrict_chg_en = val;
	rc = __battery_psy_set_charge_current(bcdev, bcdev->restrict_chg_en ?
			bcdev->restrict_fcc_ua : bcdev->thermal_fcc_ua);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t restrict_chg_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->restrict_chg_en);
}
static CLASS_ATTR_RW(restrict_chg);

static ssize_t fake_soc_store(const struct class *c, const struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	bcdev->fake_soc = val;
	pr_debug("Set fake soc to %d\n", val);

	if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) && pst->psy)
		power_supply_changed(pst->psy);

	return count;
}

static ssize_t fake_soc_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->fake_soc);
}
static CLASS_ATTR_RW(fake_soc);

static ssize_t wireless_boost_en_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_WLS],
				WLS_BOOST_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t wireless_boost_en_show(const struct class *c,
					const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int rc;

	rc = read_property_id(bcdev, pst, WLS_BOOST_EN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[WLS_BOOST_EN]);
}
static CLASS_ATTR_RW(wireless_boost_en);

static ssize_t moisture_detection_en_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_USB],
				USB_MOISTURE_DET_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t moisture_detection_en_show(const struct class *c,
					const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_EN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_EN]);
}
static CLASS_ATTR_RW(moisture_detection_en);
static ssize_t moisture_detection_usb_2_en_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_USB2],
				USB_MOISTURE_DET_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t moisture_detection_usb_2_en_show(const struct class *c,
					const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB2];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_EN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_EN]);
}
static CLASS_ATTR_RW(moisture_detection_usb_2_en);

static ssize_t moisture_detection_status_show(const struct class *c,
					const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_STS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_STS]);
}
static CLASS_ATTR_RO(moisture_detection_status);
static ssize_t moisture_detection_usb_2_status_show(const struct class *c,
					const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB2];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_STS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_STS]);
}
static CLASS_ATTR_RO(moisture_detection_usb_2_status);

static ssize_t resistance_show(const struct class *c,
					const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[BATT_RESISTANCE]);
}
static CLASS_ATTR_RO(resistance);

static ssize_t soh_show(const struct class *c, const struct class_attribute *attr,
			char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_SOH);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[BATT_SOH]);
}
static CLASS_ATTR_RO(soh);

static int battery_chg_ship_mode(struct battery_chg_dev *bcdev)
{
	struct battery_charger_ship_mode_req_msg msg = { { 0 } };
	int rc;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_SHIP_MODE_REQ_SET;
	msg.ship_mode_type = SHIP_MODE_PMIC;

	rc = battery_chg_write(bcdev, &msg, sizeof(msg), BC_WAIT_TIME_MS);
	if (rc < 0)
		pr_emerg("Failed to write ship mode: %d\n", rc);

	return rc;
}


static ssize_t ship_mode_en_store(const struct class *c,
				const struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;

	if (kstrtobool(buf, &bcdev->ship_mode_en))
		return -EINVAL;

	if (bcdev->ship_mode_en && bcdev->ship_mode_immediate) {
		rc = battery_chg_ship_mode(bcdev);
		if (rc < 0)
			return rc;
	}

	return count;
}

static ssize_t ship_mode_en_show(const struct class *c,
				const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->ship_mode_en);
}
static CLASS_ATTR_RW(ship_mode_en);

static ssize_t typec_cc_orientation_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_CC_ORIENTATION);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_CC_ORIENTATION]);
}
static CLASS_ATTR_RO(typec_cc_orientation);

static ssize_t typec_cc_orientation2_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_CC_ORIENTATION2);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_CC_ORIENTATION2]);
}
static CLASS_ATTR_RO(typec_cc_orientation2);
static ssize_t chip_main_charge_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_IC_MAIN_CHARGE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_IC_MAIN_CHARGE]);
}
static CLASS_ATTR_RO(chip_main_charge);

static ssize_t chip_fg1_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_IC_FG_1);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_IC_FG_1]);
}
static CLASS_ATTR_RO(chip_fg1);

static ssize_t chip_fg2_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_IC_FG_2);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_IC_FG_2]);
}
static CLASS_ATTR_RO(chip_fg2);

static ssize_t chip_cp1_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_IC_CP_1);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_IC_CP_1]);
}
static CLASS_ATTR_RO(chip_cp1);

static ssize_t chip_cp2_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_IC_CP_2);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_IC_CP_2]);
}
static CLASS_ATTR_RO(chip_cp2);

static ssize_t chip_qi_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_IC_QI);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_IC_QI]);
}
static CLASS_ATTR_RO(chip_qi);

static ssize_t chip_pdphy_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_IC_PDPHY);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_IC_PDPHY]);
}
static CLASS_ATTR_RO(chip_pdphy);


static ssize_t chip_qc_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_IC_QC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_IC_QC]);
}
static CLASS_ATTR_RO(chip_qc);

static ssize_t battery1_manufacturer_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_MANUFACTURER_1);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_MANUFACTURER_1]);
}
static CLASS_ATTR_RO(battery1_manufacturer);

static ssize_t battery2_manufacturer_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_MANUFACTURER_2);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_MANUFACTURER_2]);
}
static CLASS_ATTR_RO(battery2_manufacturer);

static ssize_t fg1_temp_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_HQ],
				HQ_POWER_SUPPLY_PROP_FG_1_TEMP, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fg1_temp_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_1_TEMP);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_1_TEMP]);
}
static CLASS_ATTR_RW(fg1_temp);

static ssize_t fg2_temp_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_HQ],
				HQ_POWER_SUPPLY_PROP_FG_2_TEMP, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fg2_temp_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_2_TEMP);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_2_TEMP]);
}
static CLASS_ATTR_RW(fg2_temp);

static ssize_t fg1_current_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_1_CURRENT);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_1_CURRENT]);
}
static CLASS_ATTR_RO(fg1_current);

static ssize_t fg2_current_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_2_CURRENT);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_2_CURRENT]);
}
static CLASS_ATTR_RO(fg2_current);

static ssize_t fg1_voltage_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_1_VOLTAGE);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_1_VOLTAGE]);
}
static CLASS_ATTR_RO(fg1_voltage);

static ssize_t fg2_voltage_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_2_VOLTAGE);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_2_VOLTAGE]);
}
static CLASS_ATTR_RO(fg2_voltage);

/* battery1 produce date */
static ssize_t fg1_produce_date_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_1_PRODUCE_DATE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_1_PRODUCE_DATE]);
}
static CLASS_ATTR_RO(fg1_produce_date);

/* battery1 activate date */
static ssize_t fg1_activate_date_store(const struct class *c, const struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc = 0;
	int val = 0;

	if (kstrtos32(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_HQ],
				HQ_POWER_SUPPLY_PROP_FG_1_ACTIVATE_DATE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fg1_activate_date_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_1_ACTIVATE_DATE);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_1_ACTIVATE_DATE]);
}
static CLASS_ATTR_RW(fg1_activate_date);

/* battery1 soh date */
static ssize_t fg1_soh_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_1_SOH);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_1_SOH]);
}
static CLASS_ATTR_RO(fg1_soh);

/* battery1 cycle count */
static ssize_t fg1_cycle_count_store(const struct class *c, const struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
					battery_class);
	int rc = 0;
	int val = 0;

	if (kstrtos32(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_HQ],
				HQ_POWER_SUPPLY_PROP_FG_1_CYCLE_COUNT, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fg1_cycle_count_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_1_CYCLE_COUNT);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_1_CYCLE_COUNT]);
}
static CLASS_ATTR_RW(fg1_cycle_count);

/* battery2 produce date */
static ssize_t fg2_produce_date_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_2_PRODUCE_DATE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_2_PRODUCE_DATE]);
}
static CLASS_ATTR_RO(fg2_produce_date);

/* battery2 activate date */
static ssize_t fg2_activate_date_store(const struct class *c, const struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
					battery_class);
	int rc = 0;
	int val = 0;

	if (kstrtos32(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_HQ],
				HQ_POWER_SUPPLY_PROP_FG_2_ACTIVATE_DATE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fg2_activate_date_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_2_ACTIVATE_DATE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_2_ACTIVATE_DATE]);
}
static CLASS_ATTR_RW(fg2_activate_date);

/* battery2 soh date */
static ssize_t fg2_soh_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_2_SOH);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_2_SOH]);
}
static CLASS_ATTR_RO(fg2_soh);

/* battery2 cycle count */
static ssize_t fg2_cycle_count_store(const struct class *c, const struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
					battery_class);
	int rc = 0;
	int val = 0;

	if (kstrtos32(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_HQ],
				HQ_POWER_SUPPLY_PROP_FG_2_CYCLE_COUNT, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fg2_cycle_count_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_2_CYCLE_COUNT);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_2_CYCLE_COUNT]);
}
static CLASS_ATTR_RW(fg2_cycle_count);

static ssize_t fg_monitor_cmd_store(const struct class *c, const struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
					battery_class);
	int rc = 0;
	int val = 0;

	if (kstrtos32(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_HQ], HQ_POWER_SUPPLY_PROP_FG_MONITOR_CMD, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fg_monitor_cmd_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_MONITOR_CMD);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_MONITOR_CMD]);
}
static CLASS_ATTR_RW(fg_monitor_cmd);

static ssize_t input_suspend_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc = 0;
	bool val = 0;

	if ((IS_ERR(gbcdev) || !gbcdev))
		return -EINVAL;
	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(gbcdev, &gbcdev->psy_list[PSY_TYPE_HQ],
				HQ_POWER_SUPPLY_PROP_INPUT_SUSPEND, val);
	if (rc < 0)
		return rc;

	pr_info("Lenovo Charger input_suspend:%d\n", val);
	return count;
}

static ssize_t input_suspend_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct psy_state *pst;
	if ((IS_ERR(gbcdev) || !gbcdev))
		return -EINVAL;
	pst = &gbcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(gbcdev, pst, HQ_POWER_SUPPLY_PROP_INPUT_SUSPEND);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_INPUT_SUSPEND]);
}

static struct device_attribute input_suspend_attr =
	__ATTR(input_suspend, 0644, input_suspend_show, input_suspend_store);

static ssize_t chip_sc7603a_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_IC_SC7603A);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_IC_SC7603A]);
}
static CLASS_ATTR_RO(chip_sc7603a);

static ssize_t chip_sc7603b_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_IC_SC7603B);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_IC_SC7603B]);
}
static CLASS_ATTR_RO(chip_sc7603b);

static ssize_t fg1_soc_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_1_SOC);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_1_SOC]);
}
static CLASS_ATTR_RO(fg1_soc);

static ssize_t fg2_soc_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_2_SOC);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_2_SOC]);
}
static CLASS_ATTR_RO(fg2_soc);

static ssize_t cp_master_ibus_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_CP_MASTER_IBUS);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_CP_MASTER_IBUS]);
}
static CLASS_ATTR_RO(cp_master_ibus);

static ssize_t cp_slave_ibus_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_CP_SLAVE_IBUS);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_CP_SLAVE_IBUS]);
}
static CLASS_ATTR_RO(cp_slave_ibus);

static ssize_t ibus_total_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_IBUS_TOTAL);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_IBUS_TOTAL]);
}
static CLASS_ATTR_RO(ibus_total);

static ssize_t cp_dump_register_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_CP_DUMP_REGISTER);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_CP_DUMP_REGISTER]);
}
static CLASS_ATTR_RO(cp_dump_register);

static ssize_t cp_device_id_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_CP_DEVICE_ID);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_CP_DEVICE_ID]);
}
static CLASS_ATTR_RO(cp_device_id);

static ssize_t pps_status_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_PPS_STATUS);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_PPS_STATUS]);
}
static CLASS_ATTR_RO(pps_status);

static ssize_t fg1_used_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_1_USED);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_1_USED]);
}
static ssize_t fg1_used_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_HQ],
				HQ_POWER_SUPPLY_PROP_FG_1_USED, val);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_RW(fg1_used);

static ssize_t fg2_used_show(const struct class *c, const struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_FG_2_USED);
	if (rc < 0)

		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_FG_2_USED]);
}
static ssize_t fg2_used_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_HQ],
				HQ_POWER_SUPPLY_PROP_FG_2_USED, val);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_RW(fg2_used);

static ssize_t real_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct psy_state *pst;
	const char *type = "Unknown";
	int rc;
	if ((IS_ERR(gbcdev) || !gbcdev))
		return -EINVAL;
	pst = &gbcdev->psy_list[PSY_TYPE_USB];
	rc = read_property_id(gbcdev, pst, USB_REAL_TYPE);
	if (rc < 0)
		return rc;
	type = get_usb_type_name(pst->prop[USB_REAL_TYPE]);
	return scnprintf(buf, PAGE_SIZE, "%s\n", type);
}
static struct device_attribute real_type_attr = __ATTR(real_type, 0644, real_type_show, NULL);

static ssize_t det_gpio_port1_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct psy_state *pst;
	if ((IS_ERR(gbcdev) || !gbcdev))
		return -EINVAL;
	pst = &gbcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(gbcdev, pst, HQ_POWER_SUPPLY_PROP_DET_GPIO_PORT1);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_DET_GPIO_PORT1]);
}
static struct device_attribute det_gpio_port1_attr = __ATTR(det_gpio_port1, 0444, det_gpio_port1_show, NULL);

static ssize_t det_gpio_port2_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct psy_state *pst;
	if ((IS_ERR(gbcdev) || !gbcdev))
		return -EINVAL;
	pst = &gbcdev->psy_list[PSY_TYPE_HQ];
	int rc;

	rc = read_property_id(gbcdev, pst, HQ_POWER_SUPPLY_PROP_DET_GPIO_PORT2);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[HQ_POWER_SUPPLY_PROP_DET_GPIO_PORT2]);
}
static struct device_attribute det_gpio_port2_attr = __ATTR(det_gpio_port2, 0444, det_gpio_port2_show, NULL);

#define BATT_PARALLEL_CELL_AVAIL_COUNT(val) FIELD_GET(GENMASK(15, 8), val)
#define BATT_PARALLEL_CELL_TOTAL_COUNT(val) FIELD_GET(GENMASK(7, 0), val)
static ssize_t battery_parallel_cell_count_show(const struct class *c,
			const struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev, battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_PARALLEL_CELL_COUNT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "Parallel battery cell count (available/total): %lu/%lu\n",
		BATT_PARALLEL_CELL_AVAIL_COUNT(pst->prop[BATT_PARALLEL_CELL_COUNT]),
		BATT_PARALLEL_CELL_TOTAL_COUNT(pst->prop[BATT_PARALLEL_CELL_COUNT]));
}
static CLASS_ATTR_RO(battery_parallel_cell_count);

static ssize_t battery_chg_notify_store(const struct class *c,
					const struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	if (val) {
		bcdev->notify_en = false;
		battery_chg_notify_enable(bcdev);
	} else {
		bcdev->notify_en = true;
		battery_chg_notify_disable(bcdev);
	}

	return count;
}
static CLASS_ATTR_WO(battery_chg_notify);

/* Battery classes with wireless */

static struct attribute *battery_class_attrs[] = {
	&class_attr_soh.attr,
	&class_attr_resistance.attr,
	&class_attr_moisture_detection_status.attr,
	&class_attr_moisture_detection_en.attr,
	&class_attr_wireless_boost_en.attr,
	&class_attr_fake_soc.attr,
	&class_attr_wireless_fw_update.attr,
	&class_attr_wireless_fw_force_update.attr,
	&class_attr_wireless_fw_version.attr,
	&class_attr_wireless_fw_crc.attr,
	&class_attr_wireless_fw_update_time_ms.attr,
	&class_attr_wireless_type.attr,
	&class_attr_ship_mode_en.attr,
	&class_attr_restrict_chg.attr,
	&class_attr_restrict_cur.attr,
	&class_attr_usb_real_type.attr,
	&class_attr_usb_typec_compliant.attr,
	&class_attr_charge_control_en.attr,
	&class_attr_battery_parallel_cell_count.attr,
	&class_attr_typec_cc_orientation.attr,
	&class_attr_typec_cc_orientation2.attr,
	&class_attr_chip_main_charge.attr,
	&class_attr_chip_fg1.attr,
	&class_attr_chip_fg2.attr,
	&class_attr_chip_cp1.attr,
	&class_attr_chip_cp2.attr,
	&class_attr_chip_qi.attr,
	&class_attr_chip_pdphy.attr,
	&class_attr_chip_qc.attr,
	&class_attr_chip_sc7603a.attr,
	&class_attr_chip_sc7603b.attr,
	&class_attr_battery1_manufacturer.attr,
	&class_attr_fg1_temp.attr,
	&class_attr_fg1_current.attr,
	&class_attr_fg1_voltage.attr,
	&class_attr_fg1_produce_date.attr,
	&class_attr_fg1_activate_date.attr,
	&class_attr_fg1_soh.attr,
	&class_attr_fg1_cycle_count.attr,
	&class_attr_fg1_soc.attr,
	&class_attr_fg1_used.attr,
	&class_attr_battery2_manufacturer.attr,
	&class_attr_fg2_temp.attr,
	&class_attr_fg2_current.attr,
	&class_attr_fg2_voltage.attr,
	&class_attr_fg2_produce_date.attr,
	&class_attr_fg2_activate_date.attr,
	&class_attr_fg2_soh.attr,
	&class_attr_fg2_cycle_count.attr,
	&class_attr_fg2_soc.attr,
	&class_attr_fg2_used.attr,
	&class_attr_cp_master_ibus.attr,
	&class_attr_cp_slave_ibus.attr,
	&class_attr_ibus_total.attr,
	&class_attr_cp_dump_register.attr,
	&class_attr_cp_device_id.attr,
	&class_attr_pps_status.attr,
	&class_attr_fg_monitor_cmd.attr,
	NULL,
};
ATTRIBUTE_GROUPS(battery_class);

QTI_CHARGER_RO_SHOW(fg1_fw_version, PSY_TYPE_HQ, 58);
QTI_CHARGER_RO_SHOW(fg2_fw_version, PSY_TYPE_HQ, 59);
QTI_CHARGER_RO_SHOW(fg1_charge_full, PSY_TYPE_HQ, 63);
QTI_CHARGER_RO_SHOW(fg2_charge_full, PSY_TYPE_HQ, 64);
QTI_CHARGER_RO_SHOW(fg1_charge_full_design, PSY_TYPE_HQ, 65);
QTI_CHARGER_RO_SHOW(fg2_charge_full_design, PSY_TYPE_HQ, 66);
QTI_CHARGER_RO_SHOW(fg1_charge_counter, PSY_TYPE_HQ, 67);
QTI_CHARGER_RO_SHOW(fg2_charge_counter, PSY_TYPE_HQ, 68);
QTI_CHARGER_RO_SHOW(usb_2_typec_compliant, PSY_TYPE_USB2, USB_TYPEC_COMPLIANT);

static struct attribute *dual_usb_battery_class_attrs[] = {
	&class_attr_usb_num_ports.attr,
	&class_attr_fg1_fw_version.attr,
	&class_attr_fg2_fw_version.attr,
	&class_attr_fg1_charge_full.attr,
	&class_attr_fg2_charge_full.attr,
	&class_attr_fg1_charge_full_design.attr,
	&class_attr_fg2_charge_full_design.attr,
	&class_attr_fg1_charge_counter.attr,
	&class_attr_fg2_charge_counter.attr,
	NULL,
};
static const struct attribute_group dual_usb_battery_class_group = {
	.attrs = dual_usb_battery_class_attrs,
};
static const struct attribute_group *dual_usb_class_groups[] = {
	&battery_class_group,
	&dual_usb_battery_class_group,
	NULL,
};

static struct attribute *dual_usb_two_port_class_attrs[] = {
	&class_attr_soh.attr,
	&class_attr_resistance.attr,
	&class_attr_moisture_detection_status.attr,
	&class_attr_moisture_detection_en.attr,
	&class_attr_wireless_boost_en.attr,
	&class_attr_fake_soc.attr,
	&class_attr_wireless_fw_update.attr,
	&class_attr_wireless_fw_force_update.attr,
	&class_attr_wireless_fw_version.attr,
	&class_attr_wireless_fw_crc.attr,
	&class_attr_wireless_fw_update_time_ms.attr,
	&class_attr_wireless_type.attr,
	&class_attr_ship_mode_en.attr,
	&class_attr_restrict_chg.attr,
	&class_attr_restrict_cur.attr,
	&class_attr_usb_real_type.attr,
	&class_attr_usb_typec_compliant.attr,
	&class_attr_charge_control_en.attr,
	&class_attr_battery_parallel_cell_count.attr,
	&class_attr_usb_num_ports.attr,
	&class_attr_usb_2_real_type.attr,
	&class_attr_usb_2_typec_compliant.attr,
	&class_attr_moisture_detection_usb_2_status.attr,
	&class_attr_moisture_detection_usb_2_en.attr,
	NULL,
};
ATTRIBUTE_GROUPS(dual_usb_two_port_class);


static struct attribute *battery_class_pmw6100_attrs[] = {
	&class_attr_soh.attr,
	&class_attr_resistance.attr,
	&class_attr_moisture_detection_status.attr,
	&class_attr_moisture_detection_en.attr,
	&class_attr_wireless_boost_en.attr,
	&class_attr_fake_soc.attr,
	&class_attr_wireless_fw_update.attr,
	&class_attr_wireless_fw_force_update.attr,
	&class_attr_wireless_fw_version.attr,
	&class_attr_wireless_fw_crc.attr,
	&class_attr_wireless_fw_update_time_ms.attr,
	&class_attr_wireless_type.attr,
	&class_attr_ship_mode_en.attr,
	&class_attr_restrict_chg.attr,
	&class_attr_restrict_cur.attr,
	&class_attr_usb_real_type.attr,
	&class_attr_usb_typec_compliant.attr,
	&class_attr_charge_control_en.attr,
	&class_attr_battery_parallel_cell_count.attr,
	&class_attr_battery_chg_notify.attr,
	NULL,
};
static const struct attribute_group battery_class_pmw6100_group = {
	.attrs = battery_class_pmw6100_attrs,
};
static const struct attribute_group *battery_class_pmw6100_groups[] __maybe_unused = {
	&battery_class_pmw6100_group,
	NULL,
};

#ifdef CONFIG_DEBUG_FS
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev)
{
	int rc;
	struct dentry *dir;

	dir = debugfs_create_dir("battery_charger", NULL);
	if (IS_ERR(dir)) {
		rc = PTR_ERR(dir);
		pr_err("Failed to create charger debugfs directory, rc=%d\n",
			rc);
		return;
	}

	bcdev->debugfs_dir = dir;
	debugfs_create_bool("block_tx", 0600, dir, &bcdev->block_tx);
}
#else
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev) { }
#endif

static int battery_chg_parse_dt(struct battery_chg_dev *bcdev)
{
	struct device_node *node = bcdev->dev->of_node;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int i, rc, len;
	u32 prev, val;


	of_property_read_string(node, "qcom,wireless-fw-name",
				&bcdev->wls_fw_name);

	of_property_read_u32(node, "qcom,shutdown-voltage",
				&bcdev->shutdown_volt_mv);

	bcdev->ship_mode_immediate = of_property_read_bool(node, "qcom,ship-mode-immediate");

	rc = read_property_id(bcdev, pst, BATT_CHG_CTRL_LIM_MAX);
	if (rc < 0) {
		pr_err("Failed to read prop BATT_CHG_CTRL_LIM_MAX, rc=%d\n",
			rc);
		return rc;
	}

	rc = of_property_count_elems_of_size(node, "qcom,thermal-mitigation",
						sizeof(u32));
	if (rc <= 0) {

		rc = of_property_read_u32(node, "qcom,thermal-mitigation-step",
						&val);

		if (rc < 0)
			return 0;

		if (val < 500000 || val >= pst->prop[BATT_CHG_CTRL_LIM_MAX]) {
			pr_err("thermal_fcc_step %d is invalid\n", val);
			return -EINVAL;
		}

		bcdev->thermal_fcc_step = val;
		len = pst->prop[BATT_CHG_CTRL_LIM_MAX] / bcdev->thermal_fcc_step;

		/*
		 * FCC values must be above 500mA.
		 * Since len is truncated when calculated, check and adjust len so
		 * that the above requirement is met.
		 */
		if (pst->prop[BATT_CHG_CTRL_LIM_MAX] - (bcdev->thermal_fcc_step * len) < 500000)
			len = len - 1;
	} else {
		bcdev->thermal_fcc_step = 0;
		len = rc;
		prev = pst->prop[BATT_CHG_CTRL_LIM_MAX];

		for (i = 0; i < len; i++) {
			rc = of_property_read_u32_index(node, "qcom,thermal-mitigation",
				i, &val);
			if (rc < 0)
				return rc;

			if (val > prev) {
				pr_err("Thermal levels should be in descending order\n");
				bcdev->num_thermal_levels = -EINVAL;
				return 0;
			}

			prev = val;
		}

		bcdev->thermal_levels = devm_kcalloc(bcdev->dev, len + 1,
						sizeof(*bcdev->thermal_levels),
						GFP_KERNEL);
		if (!bcdev->thermal_levels)
			return -ENOMEM;

		/*
		 * Element 0 is for normal charging current. Elements from index 1
		 * onwards is for thermal mitigation charging currents.
		 */

		bcdev->thermal_levels[0] = pst->prop[BATT_CHG_CTRL_LIM_MAX];

		rc = of_property_read_u32_array(node, "qcom,thermal-mitigation",
					&bcdev->thermal_levels[1], len);
		if (rc < 0) {
			pr_err("Error in reading qcom,thermal-mitigation, rc=%d\n", rc);
			return rc;
		}
	}

	bcdev->num_thermal_levels = len;
	bcdev->thermal_fcc_ua = pst->prop[BATT_CHG_CTRL_LIM_MAX];

	return 0;
}

static int battery_chg_reboot_notify(struct notifier_block *nb, unsigned long code,
		void *unused)
{
	struct battery_charger_notify_msg msg_notify = { { 0 } };
	struct battery_chg_dev *bcdev = container_of(nb, struct battery_chg_dev,
						     reboot_notifier);
	int rc;

	msg_notify.hdr.owner = MSG_OWNER_BC;
	msg_notify.hdr.type = MSG_TYPE_NOTIFY;
	msg_notify.hdr.opcode = BC_SHUTDOWN_NOTIFY;

	rc = battery_chg_write(bcdev, &msg_notify, sizeof(msg_notify), BC_WAIT_TIME_MS);
	if (rc < 0)
		pr_err("Failed to send shutdown notification rc=%d\n", rc);

	if (!bcdev->ship_mode_en)
		return NOTIFY_DONE;

	if (code == SYS_POWER_OFF && !bcdev->ship_mode_immediate)
		battery_chg_ship_mode(bcdev);

	return NOTIFY_DONE;
}

static void panel_event_notifier_callback(enum panel_event_notifier_tag tag,
			struct panel_event_notification *notification, void *data)
{
	struct battery_chg_dev *bcdev = data;

	if (!notification) {
		pr_debug("Invalid panel notification\n");
		return;
	}

	pr_debug("panel event received, type: %d\n", notification->notif_type);
	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_BLANK:
		battery_chg_notify_disable(bcdev);
		break;
	case DRM_PANEL_EVENT_UNBLANK:
		battery_chg_notify_enable(bcdev);
		break;
	default:
		pr_debug("Ignore panel event: %d\n", notification->notif_type);
		break;
	}
}

static int battery_chg_register_panel_notifier(struct battery_chg_dev *bcdev)
{
	struct device_node *np = bcdev->dev->of_node;
	struct device_node *pnode;
	struct drm_panel *panel, *active_panel = NULL;
	void *cookie = NULL;
	int i, count, rc;

	count = of_count_phandle_with_args(np, "qcom,display-panels", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		pnode = of_parse_phandle(np, "qcom,display-panels", i);
		if (!pnode)
			return 0;

		panel = of_drm_find_panel(pnode);
		of_node_put(pnode);
		if (!IS_ERR(panel)) {
			active_panel = panel;
			break;
		}
	}

	if (!active_panel) {
		rc = PTR_ERR(panel);
		if (rc != -EPROBE_DEFER)
			dev_err(bcdev->dev, "Failed to find active panel, rc=%d\n", rc);
		/* Stock continues charger probe even if no panel is ready. */
		return 0;
	}

	cookie = panel_event_notifier_register(
			PANEL_EVENT_NOTIFICATION_PRIMARY,
			PANEL_EVENT_NOTIFIER_CLIENT_BATTERY_CHARGER,
			active_panel,
			panel_event_notifier_callback,
			(void *)bcdev);
	if (IS_ERR(cookie)) {
		rc = PTR_ERR(cookie);
		dev_err(bcdev->dev, "Failed to register panel event notifier, rc=%d\n", rc);
		return 0;
	}

	pr_debug("register panel notifier successful\n");
	bcdev->notifier_cookie = cookie;
	return 0;
}

static int
battery_chg_get_max_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct battery_chg_dev *bcdev = tcd->devdata;

	*state = bcdev->num_thermal_levels;

	return 0;
}

static int
battery_chg_get_cur_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long *state)
{
	struct battery_chg_dev *bcdev = tcd->devdata;

	*state = bcdev->curr_thermal_level;

	return 0;
}

static int
battery_chg_set_cur_charge_cntl_limit(struct thermal_cooling_device *tcd,
					unsigned long state)
{
	struct battery_chg_dev *bcdev = tcd->devdata;

	return battery_psy_set_charge_current(bcdev, (int)state);
}

static const struct thermal_cooling_device_ops battery_tcd_ops = {
	.get_max_state = battery_chg_get_max_charge_cntl_limit,
	.get_cur_state = battery_chg_get_cur_charge_cntl_limit,
	.set_cur_state = battery_chg_set_cur_charge_cntl_limit,
};




static int lenovo_battery_sysfs_get_property(struct power_supply *psy, enum sysfs_property prop, union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = psy->drv_data;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc = 0;
	pval->intval = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE:
		pval->intval = bcdev->ship_mode_en;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLE:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLE);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLE];
		break;
	case POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING];
		break;
	case POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING_EU:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING_EU);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING_EU];
		break;
	case POWER_SUPPLY_PROP_BATTERY_RECHARGE_SETTING:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_RECHARGE_SETTING);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_RECHARGE_SETTING];
		break;
	case POWER_SUPPLY_PROP_BATTERY_MAINTENANCE:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_MAINTENANCE);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_MAINTENANCE];
		break;
	case POWER_SUPPLY_PROP_BATTERY_MAINTENANCE20:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_MAINTENANCE20);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_MAINTENANCE20];
		break;
	case POWER_SUPPLY_PROP_BATTERY_PRODUCE_DATE:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_PRODUCE_DATE);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_PRODUCE_DATE];
		break;
	case POWER_SUPPLY_PROP_BATTERY_ACTIVATE_DATE:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_ACTIVATE_DATE);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_ACTIVATE_DATE];
		break;
	case POWER_SUPPLY_PROP_BATTERY_SOH:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_SOH);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_SOH];
		break;
	case POWER_SUPPLY_PROP_BATTERY_FAKE_CYCLE_COUNT:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_FAKE_CYCLE_COUNT);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_FAKE_CYCLE_COUNT];
		break;
	case POWER_SUPPLY_PROP_BATTERY_FAKE_SOC:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_FAKE_SOC);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_FAKE_SOC];
		break;
	case POWER_SUPPLY_PROP_BATTERY_CV:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_CV);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_CV];
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHG_FV:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_CHG_FV);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_CHG_FV];
		break;
	case POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE_NVM:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE_NVM);
		if (rc < 0)
			return rc;
		pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE_NVM];
		break;
	case POWER_SUPPLY_PROP_BATTERY_ABNORMAL_STATE:
		rc = read_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_ABNORMAL_STATE);
		if (rc < 0) {
			pr_err("Failed to read BATTERY_ABNORMAL_STATE, rc=%d\n", rc);
			return rc;
		}
		if (bcdev->fake_abnormal_level >= 0 && bcdev->fake_abnormal_level <= 4)
			pval->intval = bcdev->fake_abnormal_level;
		else
			pval->intval = pst->prop[HQ_POWER_SUPPLY_PROP_BATTERY_ABNORMAL_STATE];
		break;
	default:
		dev_err(bcdev->dev, ">>>Lenovo battery get error property<<<\n");
		return -EINVAL;
	}

	return 0;
}

static int lenovo_battery_sysfs_set_property(struct power_supply *psy, enum sysfs_property prop, const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = psy->drv_data;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int rc = 0;

	pr_info("Lenovo battery set property = %d, value = %d\n", prop, pval->intval);

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE:
		bcdev->ship_mode_en = (bool)!!pval->intval;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLE:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLE, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set charging_enable, rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set protection setting, rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING_EU:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING_EU, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set protection setting eu, rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_RECHARGE_SETTING:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_RECHARGE_SETTING, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set recharge setting rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_MAINTENANCE:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_MAINTENANCE, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set battery maintenance rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_MAINTENANCE20:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_MAINTENANCE20, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set battery maintenance20 rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_ACTIVATE_DATE:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_ACTIVATE_DATE, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set charging enable, rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_CV:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_CV, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set battery cv, rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHG_FV:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_CHG_FV, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set battery chg FV, rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_FAKE_CYCLE_COUNT:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_FAKE_CYCLE_COUNT, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set fake cycle count, rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_FAKE_SOC:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_FAKE_SOC, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set fake soc, rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE_NVM:
		rc = write_property_id(bcdev, pst, HQ_POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE_NVM, pval->intval);
		if (rc < 0) {
			pr_err("Failed to set shipping mode nvm, rc=%d\n", rc);
			return rc;
		}
		break;
	case POWER_SUPPLY_PROP_BATTERY_ABNORMAL_STATE:
		bcdev->fake_abnormal_level = pval->intval;
		bcdev->battery_abnormal = true;
		schedule_work(&bcdev->battery_check_work);
		break;
	default:
		dev_err(bcdev->dev, ">>>Lenovo battery set error property<<<\n");
		return -EINVAL;
	}

	return 0;
}

static ssize_t lenovo_battery_sysfs_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct power_supply *psy = NULL;
	struct sysfs_info *sysfs_attr;
	union power_supply_propval pval = {0};
	int val;
	ssize_t ret;

	ret = kstrtos32(buf, 10, &val);
	if (ret < 0)
		return ret;

	pval.intval = val;

	psy = dev_get_drvdata(dev);
	if (!psy) {
		dev_info(&psy->dev, "invalid battery psy");
		return ret;
	}

	sysfs_attr = container_of(attr, struct sysfs_info, attr);
	lenovo_battery_sysfs_set_property(psy, sysfs_attr->prop, &pval);

	return count;
}

static ssize_t lenovo_battery_sysfs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct power_supply *psy = dev_get_drvdata(dev);
	struct sysfs_info *sysfs_attr;
	union power_supply_propval pval = {0};
	ssize_t count = 0;

	if (!psy) {
		dev_info(&psy->dev, "invalid battery psy");
		return count;
	}

	sysfs_attr = container_of(attr, struct sysfs_info, attr);
	lenovo_battery_sysfs_get_property(psy, sysfs_attr->prop, &pval);

	if (sysfs_attr->prop == POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING ||
		sysfs_attr->prop == POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING_EU) {
		count = scnprintf(buf, PAGE_SIZE, "%06d\n", pval.intval);
	} else {
		count = scnprintf(buf, PAGE_SIZE, "%d\n", pval.intval);
	}

	return count;
}

static struct sysfs_info battery_sysfs_field_tbl[] = {
	BATT_SYSFS_FIELD_RW(charging_enabled, POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLE, false),
	BATT_SYSFS_FIELD_RW(protection_setting, POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING, false),
	BATT_SYSFS_FIELD_RW(protection_setting_eu, POWER_SUPPLY_PROP_BATTERY_PROTECTION_SETTING_EU, false),
	BATT_SYSFS_FIELD_RW(recharge_setting, POWER_SUPPLY_PROP_BATTERY_RECHARGE_SETTING, false),
	BATT_SYSFS_FIELD_RW(battery_maintenance, POWER_SUPPLY_PROP_BATTERY_MAINTENANCE, false),
	BATT_SYSFS_FIELD_RW(battery_maintenance20, POWER_SUPPLY_PROP_BATTERY_MAINTENANCE20, false),
	BATT_SYSFS_FIELD_RW(shipping_mode, POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE, false),
	BATT_SYSFS_FIELD_RO(produce_date, POWER_SUPPLY_PROP_BATTERY_PRODUCE_DATE, false),
	BATT_SYSFS_FIELD_RW(activate_date, POWER_SUPPLY_PROP_BATTERY_ACTIVATE_DATE, false),
	BATT_SYSFS_FIELD_RO(soh, POWER_SUPPLY_PROP_BATTERY_SOH, false),
	//BATT_SYSFS_FIELD_RO(time_to_full_now, POWER_SUPPLY_PROP_BATTERY_TIME_TO_FULL_NOW, false),
	BATT_SYSFS_FIELD_RW(cv, POWER_SUPPLY_PROP_BATTERY_CV, false),
	BATT_SYSFS_FIELD_RW(chg_fv, POWER_SUPPLY_PROP_BATTERY_CHG_FV, false),
	BATT_SYSFS_FIELD_RW(fake_cycle_count, POWER_SUPPLY_PROP_BATTERY_FAKE_CYCLE_COUNT, false),
	BATT_SYSFS_FIELD_RW(fake_soc, POWER_SUPPLY_PROP_BATTERY_FAKE_SOC, false),
	BATT_SYSFS_FIELD_RW(abnormal_level, POWER_SUPPLY_PROP_BATTERY_ABNORMAL_STATE, false),
	BATT_SYSFS_FIELD_RW(shipping_mode_nvm, POWER_SUPPLY_PROP_BATTERY_SHIPPING_MODE_NVM, false),
};

static struct attribute
	*battery_sysfs_attrs[ARRAY_SIZE(battery_sysfs_field_tbl) + 1];

static const struct attribute_group battery_sysfs_attr_group = {
	.attrs = battery_sysfs_attrs,
};

static int lenovo_battery_sysfs_create_group(struct battery_chg_dev *bcdev)
{
	int i = 0, length = 0, rc = -1;

	if (!(bcdev->psy_list[PSY_TYPE_BATTERY].psy)) {
		dev_info(bcdev->dev, "Invalid battery psy\n");
		return rc;
	}

	length = ARRAY_SIZE(battery_sysfs_field_tbl);
	for (i = 0; i < length; i++)
		battery_sysfs_attrs[i] = &battery_sysfs_field_tbl[i].attr.attr;

	battery_sysfs_attrs[length] = NULL;
	rc = sysfs_create_group(&bcdev->psy_list[PSY_TYPE_BATTERY].psy->dev.kobj, &battery_sysfs_attr_group);
	if (rc) {
		dev_info(bcdev->dev, "Failed to create battery_sysfs\n");
	}

	return rc;
}

static ssize_t fg_fw_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(dev_get_drvdata(dev));
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	u32 value = 0;

	if (!read_property_id(bcdev, pst, 58))
		value = pst->prop[58];
	return sysfs_emit(buf, "%04X\n", value);
}
static DEVICE_ATTR(fg_fw_version, 0664, fg_fw_version_show, NULL);

static ssize_t chg_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(dev_get_drvdata(dev));
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	u32 value = 0;

	mutex_lock(&bcdev->chg_status_lock);
	if (bcdev->chg_status_valid &&
	    time_before_eq(jiffies, bcdev->chg_status_updated + 13)) {
		value = pst->prop[60];
	} else if (!read_property_id(bcdev, pst, 60)) {
		value = pst->prop[60];
		bcdev->chg_status_updated = jiffies;
		bcdev->chg_status_valid = true;
	}
	mutex_unlock(&bcdev->chg_status_lock);
	return sysfs_emit(buf, "%d\n", value);
}
static DEVICE_ATTR(chg_status, 0664, chg_status_show, NULL);

static struct attribute *dual_usb_battery_psy_attrs[] = {
	&dev_attr_fg_fw_version.attr,
	&dev_attr_chg_status.attr,
	NULL,
};
static const struct attribute_group dual_usb_battery_psy_group = {
	.attrs = dual_usb_battery_psy_attrs,
};

static int dual_usb_battery_sysfs_create_group(struct battery_chg_dev *bcdev)
{
	struct kobject *kobj = &bcdev->psy_list[PSY_TYPE_BATTERY].psy->dev.kobj;
	int ret = lenovo_battery_sysfs_create_group(bcdev);

	if (ret)
		return ret;
	ret = sysfs_create_group(kobj, &dual_usb_battery_psy_group);
	if (ret)
		sysfs_remove_group(kobj, &battery_sysfs_attr_group);
	return ret;
}

static ssize_t charge_adapter_power_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(dev_get_drvdata(dev));
	struct psy_state *hq = &bcdev->psy_list[PSY_TYPE_HQ];
	struct psy_state *usb = &bcdev->psy_list[PSY_TYPE_USB];
	u32 power, voltage;
	int ret;

	ret = read_property_id(bcdev, hq, 61);
	if (ret)
		return ret;
	power = hq->prop[61];
	ret = read_property_id(bcdev, usb, USB_VOLT_NOW);
	if (ret)
		return ret;
	voltage = usb->prop[USB_VOLT_NOW];
	ret = read_property_id(bcdev, hq, HQ_POWER_SUPPLY_PROP_IBUS_TOTAL);
	if (ret)
		return ret;
	return sysfs_emit(buf, "%d,%d,%d\n", power / 1000, voltage / 1000,
			 hq->prop[HQ_POWER_SUPPLY_PROP_IBUS_TOTAL]);
}
static DEVICE_ATTR_RO(charge_adapter_power);

static ssize_t charge_adapter_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(dev_get_drvdata(dev));
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	int ret = read_property_id(bcdev, pst, 62);

	return ret ? ret : sysfs_emit(buf, "%x\n", pst->prop[62]);
}
static DEVICE_ATTR_RO(charge_adapter_type);

static ssize_t usb_otg_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(dev_get_drvdata(dev));
	int value;
	int rc = kstrtoint(buf, 0, &value);

	if (rc)
		return -EINVAL;
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_HQ],
			DUAL_USB_USB_OTG_RESET, value);
	return rc ? rc : count;
}
static DEVICE_ATTR_WO(usb_otg_reset);

static struct attribute *usb_psy_attrs[] = {
	&input_suspend_attr.attr,
	&real_type_attr.attr,
	&det_gpio_port1_attr.attr,
	&det_gpio_port2_attr.attr,
	NULL,
};

static const struct attribute_group usb_psy_attrs_group = {
	.attrs = usb_psy_attrs,
};

static int lenovo_usb_sysfs_create_group(struct battery_chg_dev *bcdev)
{
	return sysfs_create_group(&bcdev->psy_list[PSY_TYPE_USB].psy->dev.kobj,
								&usb_psy_attrs_group);
}



/* Leave a port's cutoff latched on sensor failure; still monitor the other port. */
static bool lenovo_usb_temp(struct battery_chg_dev *bcdev,
		struct thermal_zone_device **zone, const char *name, int *temp)
{
	int rc;

	if (IS_ERR_OR_NULL(*zone))
		*zone = thermal_zone_get_zone_by_name(name);
	if (IS_ERR_OR_NULL(*zone))
		return false;
	rc = thermal_zone_get_temp(*zone, temp);
	if (rc) {
		dev_err_ratelimited(bcdev->dev, "Failed to read %s: %d\n", name, rc);
		return false;
	}
	*temp /= 100;
	return true;
}

static void lenovo_chg_monitor_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work, struct battery_chg_dev,
			lenovo_chg_monitor_work.work);
	struct psy_state *batt = &bcdev->psy_list[PSY_TYPE_BATTERY];
	struct psy_state *usb = &bcdev->psy_list[PSY_TYPE_USB];
	struct psy_state *wls = &bcdev->psy_list[PSY_TYPE_WLS];
	int t1 = 0, t2 = 0, bt;
	bool valid1, valid2;
	struct psy_state *hq = &bcdev->psy_list[PSY_TYPE_HQ];
	bool present = hq->prop[HQ_POWER_SUPPLY_PROP_IC_FG_1] || hq->prop[HQ_POWER_SUPPLY_PROP_IC_FG_2];
	const char *type = "Unknown";

	bcdev->lenovo_chg_monitor_started = true;
	bt = (int)batt->prop[BATT_TEMP];
	bt = (bt + (bt > 0 ? 5 : -5)) / 10;
	valid1 = bcdev->usb1_burn_en && lenovo_usb_temp(bcdev,
		&bcdev->usb1_therm, "usb1-conn-therm", &t1);
	valid2 = bcdev->usb2_burn_en && lenovo_usb_temp(bcdev,
		&bcdev->usb2_therm, "usb2-conn-therm", &t2);
	if (valid1) {
		if (t1 < 450 || (t1 < 500 && t1 - bt < 100))
			gpiod_set_value_cansleep(bcdev->usb1_burn_en, 0);
		else if ((t1 > 750 || (t1 > 650 && t1 - bt > 300)) && present)
			gpiod_set_value_cansleep(bcdev->usb1_burn_en, 1);
	}
	if (valid2) {
		if (t2 < 450 || (t2 < 500 && t2 - bt < 100))
			gpiod_set_value_cansleep(bcdev->usb2_burn_en, 0);
		else if ((t2 > 700 || (t2 > 600 && t2 - bt > 200)) && present)
			gpiod_set_value_cansleep(bcdev->usb2_burn_en, 1);
	}
	type = get_usb_type_name(usb->prop[USB_REAL_TYPE]);
	pr_info("TYPE = [%s,%d,%d], VBUS = [%d,%d], USB_TEMP = [%d,%d], BATT_TEMP = %d, BATT_EXSIT = %d, GPIO:[%d,%d]\n",
		type, usb_psy_desc.type, wls_psy_desc.type,
		usb->prop[USB_VOLT_NOW], wls->prop[WLS_VOLT_NOW], t1, t2, bt, present,
		bcdev->usb1_burn_en ? gpiod_get_value_cansleep(bcdev->usb1_burn_en) : -1,
		bcdev->usb2_burn_en ? gpiod_get_value_cansleep(bcdev->usb2_burn_en) : -1);
	if (READ_ONCE(bcdev->initialized))
		schedule_delayed_work(&bcdev->lenovo_chg_monitor_work, msecs_to_jiffies(5000));
}

static void devices_get_informs_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work, struct battery_chg_dev,
			devices_get_informs_work.work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_HQ];
	/* Keep stock request ordering; hardwareinfo registrations are omitted. */
	static const u32 props[] = { 2, 3, 4, 52, 5, 6, 8, 9, 42, 43 };
	int i;

	for (i = 0; i < ARRAY_SIZE(props); i++)
		if (read_property_id(bcdev, pst, props[i]) < 0)
			return;
}



static struct attribute *dual_usb_reset_attrs[] = {
	&dev_attr_usb_otg_reset.attr,
	NULL,
};
static const struct attribute_group dual_usb_reset_group = {
	.attrs = dual_usb_reset_attrs,
};

static struct attribute *dual_usb_usb_psy_attrs[] = {
	&dev_attr_charge_adapter_power.attr,
	&dev_attr_charge_adapter_type.attr,
	NULL,
};
static const struct attribute_group dual_usb_usb_psy_group = {
	.attrs = dual_usb_usb_psy_attrs,
};

static int dual_usb_usb_sysfs_create_group(struct battery_chg_dev *bcdev)
{
	struct kobject *kobj = &bcdev->psy_list[PSY_TYPE_USB].psy->dev.kobj;
	int ret = lenovo_usb_sysfs_create_group(bcdev);

	if (ret)
		return ret;
	ret = sysfs_create_group(kobj, &dual_usb_usb_psy_group);
	if (ret)
		sysfs_remove_group(kobj, &usb_psy_attrs_group);
	return ret;
}

static const struct battery_chg_protocol lenovo_dual_bank_v1 = {
	.banks = {
		[PSY_TYPE_BATTERY] = {
			.map = battery_prop_map, .prop_count = BATT_PROP_MAX,
			.opcode_get = BC_BATTERY_STATUS_GET, .opcode_set = BC_BATTERY_STATUS_SET,
		},
		[PSY_TYPE_USB] = {
			.map = usb_prop_map, .prop_count = USB_PROP_MAX,
			.opcode_get = BC_USB_STATUS_GET, .opcode_set = BC_USB_STATUS_SET,
		},
		[PSY_TYPE_WLS] = {
			.map = wls_prop_map, .prop_count = WLS_PROP_MAX,
			.opcode_get = BC_WLS_STATUS_GET, .opcode_set = BC_WLS_STATUS_SET,
			.isolate = true,
		},
		[PSY_TYPE_HQ] = {
			.map = hq_prop_map, .prop_count = HQ_POWER_SUPPLY_PROPERTIES_MAX,
			.opcode_get = HQ_POWER_SUPPLY_GET_REQ, .opcode_set = HQ_POWER_SUPPLY_SET_REQ,
		},
	},
	.class_groups = battery_class_groups,
	.create_battery_group = lenovo_battery_sysfs_create_group,
	.create_usb_group = lenovo_usb_sysfs_create_group,
	.dual_port = true,
};

/* USB2 uses distinct opcodes while battery_id remains zero. */
static const struct battery_chg_protocol lenovo_dual_usb_v1 = {
	.banks = {
		[PSY_TYPE_BATTERY] = {
			.map = battery_prop_map, .prop_count = BATT_PROP_MAX,
			.opcode_get = BC_BATTERY_STATUS_GET, .opcode_set = BC_BATTERY_STATUS_SET,
		},
		[PSY_TYPE_USB] = {
			.map = dual_usb_usb_prop_map, .prop_count = 17,
			.opcode_get = BC_USB_STATUS_GET, .opcode_set = BC_USB_STATUS_SET,
		},
		[PSY_TYPE_USB2] = {
			.map = dual_usb_usb_prop_map, .prop_count = 17,
			.opcode_get = BC_USB2_STATUS_GET, .opcode_set = BC_USB2_STATUS_SET,
			.isolate = true,
		},
		[PSY_TYPE_WLS] = {
			.map = wls_prop_map, .prop_count = WLS_PROP_MAX,
			.opcode_get = BC_WLS_STATUS_GET, .opcode_set = BC_WLS_STATUS_SET,
			.isolate = true,
		},
		[PSY_TYPE_HQ] = {
			.map = dual_usb_hq_prop_map, .prop_count = 70,
			.opcode_get = HQ_POWER_SUPPLY_GET_REQ,
			.opcode_set = HQ_POWER_SUPPLY_SET_REQ,
		},
	},
	.class_groups = dual_usb_class_groups,
	.create_battery_group = dual_usb_battery_sysfs_create_group,
	.create_usb_group = dual_usb_usb_sysfs_create_group,
	.dual_port = true,
};

static int battery_chg_probe(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev;
	struct device *dev = &pdev->dev;
	struct pmic_glink_client_data client_data = { };
	struct thermal_cooling_device *tcd;
	struct psy_state *pst;
	int rc, i;
	const struct battery_chg_protocol *protocol = device_get_match_data(dev);

	if (!protocol)
		return dev_err_probe(dev, -ENODEV, "charger protocol not specified\n");

	bcdev = devm_kzalloc(&pdev->dev, sizeof(*bcdev), GFP_KERNEL);
	if (!bcdev)
		return -ENOMEM;

	bcdev->protocol = protocol;
	memcpy(bcdev->psy_list, protocol->banks, sizeof(bcdev->psy_list));

	for (i = 0; i < PSY_TYPE_MAX; i++) {
		bcdev->psy_list[i].prop =
			devm_kcalloc(&pdev->dev, bcdev->psy_list[i].prop_count,
					sizeof(u32), GFP_KERNEL);
		if (!bcdev->psy_list[i].prop)
			return -ENOMEM;
	}

	bcdev->psy_list[PSY_TYPE_BATTERY].model =
		devm_kzalloc(&pdev->dev, MAX_STR_LEN, GFP_KERNEL);
	if (!bcdev->psy_list[PSY_TYPE_BATTERY].model)
		return -ENOMEM;

	bcdev->lenovo_chg_monitor_started = false;
	mutex_init(&bcdev->rw_lock);
	mutex_init(&bcdev->chg_status_lock);
	init_rwsem(&bcdev->state_sem);
	init_completion(&bcdev->ack);
	init_completion(&bcdev->fw_buf_ack);
	init_completion(&bcdev->fw_update_ack);
	if (protocol->dual_port) {
		bcdev->usb1_burn_en = devm_gpiod_get_optional(dev,
			"lenovo,usb1-burn-en", GPIOD_OUT_LOW);
		bcdev->usb2_burn_en = devm_gpiod_get_optional(dev,
			"lenovo,usb2-burn-en", GPIOD_OUT_LOW);
		if (IS_ERR(bcdev->usb1_burn_en) || IS_ERR(bcdev->usb2_burn_en)) {
			dev_err(dev, "USB burn GPIO request failed (%ld, %ld); cutoff disabled\n",
				IS_ERR(bcdev->usb1_burn_en) ? PTR_ERR(bcdev->usb1_burn_en) : 0L,
				IS_ERR(bcdev->usb2_burn_en) ? PTR_ERR(bcdev->usb2_burn_en) : 0L);
			bcdev->usb1_burn_en = NULL;
			bcdev->usb2_burn_en = NULL;
		}
	}

	INIT_DELAYED_WORK(&bcdev->lenovo_chg_monitor_work, lenovo_chg_monitor_work);
	INIT_WORK(&bcdev->subsys_up_work, battery_chg_subsys_up_work);
	INIT_WORK(&bcdev->usb_type_work, battery_chg_update_usb_type_work);
	INIT_DEFERRABLE_WORK(&bcdev->bank_probe_work, battery_chg_bank_probe_work);
	bcdev->probe_backoff_ms = BC_PROBE_MIN_MS;
	bcdev->last_probe_jiffies = jiffies - msecs_to_jiffies(BC_PROBE_MIN_MS);
	INIT_WORK(&bcdev->battery_check_work, battery_chg_check_status_work);
	INIT_DELAYED_WORK(&bcdev->devices_get_informs_work, devices_get_informs_work);
	bcdev->charge_wakelock = wakeup_source_register(dev, "charge_wakelock");
	if (!bcdev->charge_wakelock)
		return -ENOMEM;
	bcdev->dev = dev;

	rc = battery_chg_register_panel_notifier(bcdev);
	if (rc < 0)
		goto reg_error;

	client_data.id = MSG_OWNER_BC;
	client_data.name = "battery_charger";
	client_data.msg_cb = battery_chg_callback;
	client_data.priv = bcdev;
	client_data.state_cb = battery_chg_state_cb;

	bcdev->client = pmic_glink_register_client(dev, &client_data);
	if (IS_ERR(bcdev->client)) {
		rc = PTR_ERR(bcdev->client);
		if (rc != -EPROBE_DEFER)
			dev_err(dev, "Error in registering with pmic_glink %d\n",
				rc);
		goto reg_error;
	}

	down_write(&bcdev->state_sem);
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_UP);
	/*
	 * This should be initialized here so that battery_chg_callback
	 * can run successfully when battery_chg_parse_dt() starts
	 * reading BATT_CHG_CTRL_LIM_MAX parameter and waits for a response.
	 */
	bcdev->initialized = true;
	up_write(&bcdev->state_sem);

	bcdev->reboot_notifier.notifier_call = battery_chg_reboot_notify;
	bcdev->reboot_notifier.priority = 255;
	register_reboot_notifier(&bcdev->reboot_notifier);

	rc = battery_chg_parse_dt(bcdev);
	if (rc < 0) {
		dev_err(dev, "Failed to parse dt rc=%d\n", rc);
		goto error;
	}

	bcdev->restrict_fcc_ua = DEFAULT_RESTRICT_FCC_UA;
	platform_set_drvdata(pdev, bcdev);
	bcdev->fake_soc = -EINVAL;
	rc = battery_chg_init_psy(bcdev);
	if (rc < 0)
		goto error;

	bcdev->battery_class.name = "qcom-battery";
	bcdev->battery_class.class_groups = protocol->class_groups;
	if (bcdev->psy_list[PSY_TYPE_USB2].prop_count && bcdev->usb_num_ports == 2)
		bcdev->battery_class.class_groups = dual_usb_two_port_class_groups;
	rc = class_register(&bcdev->battery_class);
	if (rc < 0) {
		dev_err(dev, "Failed to create battery_class rc=%d\n", rc);
		goto error;
	}

	bcdev->class_registered = true;
	rc = protocol->create_battery_group(bcdev);
	if (rc < 0) {
		dev_err(dev, "Failed to create battery class rc=%d\n", rc);
		goto error;
	}

	rc = protocol->create_usb_group(bcdev);
	if (rc < 0) {
		dev_err(dev, "Failed to create usb class rc=%d\n", rc);
		goto error;
	}

	pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	tcd = devm_thermal_of_cooling_device_register(dev, dev->of_node,
			(char *)pst->psy->desc->name, bcdev, &battery_tcd_ops);
	if (IS_ERR_OR_NULL(tcd)) {
		rc = PTR_ERR_OR_ZERO(tcd);
		dev_err(dev, "Failed to register thermal cooling device rc=%d\n",
			rc);
		goto error;
	}

	bcdev->wls_fw_update_time_ms = WLS_FW_UPDATE_TIME_MS;
	battery_chg_add_debugfs(bcdev);
	bcdev->notify_en = false;
	battery_chg_notify_enable(bcdev);
	device_init_wakeup(bcdev->dev, true);
	schedule_work(&bcdev->usb_type_work);

	rc = get_charge_control_en(bcdev);
	if (rc < 0)
		pr_debug("Failed to read charge_control_en, rc = %d\n", rc);

	/* Stock: monitor at 20 seconds, hardware requests at 15 seconds. */
	if (protocol->dual_port)
		schedule_delayed_work(&bcdev->lenovo_chg_monitor_work, msecs_to_jiffies(20000));
	schedule_delayed_work(&bcdev->devices_get_informs_work, msecs_to_jiffies(15000));
	bcdev->battery_abnormal = false;
	bcdev->fake_abnormal_level = -EINVAL;
	WRITE_ONCE(gbcdev, bcdev);

	return 0;
error:
	if (bcdev->class_registered)
		class_unregister(&bcdev->battery_class);
	down_write(&bcdev->state_sem);
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_DOWN);
	bcdev->initialized = false;
	up_write(&bcdev->state_sem);

	cancel_delayed_work_sync(&bcdev->bank_probe_work);
	cancel_work_sync(&bcdev->usb_type_work);
	cancel_work_sync(&bcdev->subsys_up_work);
	cancel_work_sync(&bcdev->battery_check_work);
	cancel_delayed_work_sync(&bcdev->lenovo_chg_monitor_work);
	cancel_delayed_work_sync(&bcdev->devices_get_informs_work);
	complete(&bcdev->ack);
	unregister_reboot_notifier(&bcdev->reboot_notifier);
	pmic_glink_unregister_client(bcdev->client);
reg_error:
	if (bcdev->notifier_cookie)
		panel_event_notifier_unregister(bcdev->notifier_cookie);
	wakeup_source_unregister(bcdev->charge_wakelock);
	return rc;
}

static void battery_chg_remove(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev = platform_get_drvdata(pdev);

	WRITE_ONCE(gbcdev, NULL);
	down_write(&bcdev->state_sem);
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_DOWN);
	bcdev->initialized = false;
	up_write(&bcdev->state_sem);

	if (bcdev->notifier_cookie)
		panel_event_notifier_unregister(bcdev->notifier_cookie);

	device_init_wakeup(bcdev->dev, false);
	debugfs_remove_recursive(bcdev->debugfs_dir);
	class_unregister(&bcdev->battery_class);
	cancel_work_sync(&bcdev->subsys_up_work);
	cancel_delayed_work_sync(&bcdev->bank_probe_work);
	cancel_work_sync(&bcdev->usb_type_work);
	cancel_work_sync(&bcdev->battery_check_work);
	cancel_delayed_work_sync(&bcdev->lenovo_chg_monitor_work);
	cancel_delayed_work_sync(&bcdev->devices_get_informs_work);
	unregister_reboot_notifier(&bcdev->reboot_notifier);
	pmic_glink_unregister_client(bcdev->client);
	wakeup_source_unregister(bcdev->charge_wakelock);
}

static const struct of_device_id battery_chg_match_table[] = {
	{ .compatible = "lenovo,dual-bank-battery-charger-v1", .data = &lenovo_dual_bank_v1 },
	{ .compatible = "lenovo,dual-usb-battery-charger-v1", .data = &lenovo_dual_usb_v1 },
	{ .compatible = "qcom,battery-charger" },
	{ .compatible = "qcom,pmw6100-battery-charger" },
	{},
};

static struct platform_driver battery_chg_driver = {
	.driver = {
		.name = "qti_battery_charger",
		.of_match_table = battery_chg_match_table,
	},
	.probe = battery_chg_probe,
	.remove = battery_chg_remove,
};
module_platform_driver(battery_chg_driver);

MODULE_DESCRIPTION("QTI Glink battery charger driver");
MODULE_LICENSE("GPL");
