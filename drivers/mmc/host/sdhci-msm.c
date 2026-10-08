// SPDX-License-Identifier: GPL-2.0-only
/*
 * drivers/mmc/host/sdhci-msm.c - Qualcomm SDHCI Platform driver
 *
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sd.h>
#include <linux/pm_runtime.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/interconnect.h>
#include <linux/iopoll.h>
#include <linux/regulator/consumer.h>
#include <linux/interconnect.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_qos.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/reset.h>
#include <linux/clk/qcom.h>
#include <linux/nvmem-consumer.h>
#include <linux/ipc_logging.h>
#include <linux/pinctrl/qcom-pinctrl.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/pm_domain.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/mmc/sdio_func.h>
#include <soc/qcom/sdhci-msm.h>

#include <soc/qcom/ice.h>

#include "drivers/mmc/host/sdhci-cqhci.h"
#include "drivers/mmc/host/sdhci-pltfm.h"
#include "cqhci.h"
#include "drivers/mmc/core/core.h"
#include <linux/qtee_shmbridge.h>

#define CORE_MCI_VERSION		0x50
#define CORE_VERSION_MAJOR_SHIFT	28
#define CORE_VERSION_MAJOR_MASK		(0xf << CORE_VERSION_MAJOR_SHIFT)
#define CORE_VERSION_MINOR_MASK		0xff
#define CORE_VERSION_TARGET_MASK        0x000000FF
#define SDHCI_MSM_VER_420               0x49

#define CORE_MCI_GENERICS		0x70
#define SWITCHABLE_SIGNALING_VOLTAGE	BIT(29)

#define HC_MODE_EN		0x1
#define CORE_POWER		0x0
#define CORE_SW_RST		BIT(7)
#define FF_CLK_SW_RST_DIS	BIT(13)

#define VENDOR_SPEC_FUNC4      0x260
#define EXTERN_FB_CLK          BIT(24)

#define CORE_PWRCTL_BUS_OFF	BIT(0)
#define CORE_PWRCTL_BUS_ON	BIT(1)
#define CORE_PWRCTL_IO_LOW	BIT(2)
#define CORE_PWRCTL_IO_HIGH	BIT(3)
#define CORE_PWRCTL_BUS_SUCCESS BIT(0)
#define CORE_PWRCTL_BUS_FAIL    BIT(1)
#define CORE_PWRCTL_IO_SUCCESS	BIT(2)
#define CORE_PWRCTL_IO_FAIL     BIT(3)
#define REQ_BUS_OFF		BIT(0)
#define REQ_BUS_ON		BIT(1)
#define REQ_IO_LOW		BIT(2)
#define REQ_IO_HIGH		BIT(3)
#define INT_MASK		0xf
#define MAX_PHASES		16
#define CORE_DLL_LOCK		BIT(7)
#define CORE_DDR_DLL_LOCK	BIT(11)
#define CORE_DLL_EN		BIT(16)
#define CORE_CDR_EN		BIT(17)
#define CORE_CK_OUT_EN		BIT(18)
#define CORE_CDR_EXT_EN		BIT(19)
#define CORE_DLL_PDN		BIT(29)
#define CORE_DLL_RST		BIT(30)
#define CORE_CMD_DAT_TRACK_SEL	BIT(0)

#define CORE_DDR_CAL_EN		BIT(0)
#define CORE_FLL_CYCLE_CNT	BIT(18)
#define CORE_LOW_FREQ_MODE	BIT(19)
#define CORE_DLL_CLOCK_DISABLE	BIT(21)

#define DLL_USR_CTL_POR_VAL	0x10800
#define ENABLE_DLL_LOCK_STATUS	BIT(26)
#define FINE_TUNE_MODE_EN	BIT(27)
#define BIAS_OK_SIGNAL		BIT(29)

#define DLL_CONFIG_3_LOW_FREQ_VAL	0x08
#define DLL_CONFIG_3_HIGH_FREQ_VAL	0x10

#define CORE_VENDOR_SPEC_POR_VAL 0xa9c
#define CORE_CLK_PWRSAVE	BIT(1)
#define CORE_VNDR_SPEC_ADMA_ERR_SIZE_EN	BIT(7)
#define CORE_HC_MCLK_SEL_DFLT	(2 << 8)
#define CORE_HC_MCLK_SEL_HS400	(3 << 8)
#define CORE_HC_MCLK_SEL_MASK	(3 << 8)
#define CORE_IO_PAD_PWR_SWITCH_EN	BIT(15)
#define CORE_IO_PAD_PWR_SWITCH	BIT(16)
#define CORE_HC_SELECT_IN_EN	BIT(18)
#define CORE_HC_SELECT_IN_HS400	(6 << 19)
#define CORE_HC_SELECT_IN_MASK	(7 << 19)
#define CORE_HC_SELECT_IN_SDR50	(4 << 19)

#define CORE_8_BIT_SUPPORT	BIT(18)
#define CORE_3_0V_SUPPORT	BIT(25)
#define CORE_1_8V_SUPPORT	BIT(26)
#define CORE_VOLT_SUPPORT	(CORE_3_0V_SUPPORT | CORE_1_8V_SUPPORT)

#define CORE_SYS_BUS_SUPPORT_64_BIT     BIT(28)

#define CORE_CSR_CDC_CTLR_CFG0		0x130
#define CORE_SW_TRIG_FULL_CALIB		BIT(16)
#define CORE_HW_AUTOCAL_ENA		BIT(17)

#define CORE_CSR_CDC_CTLR_CFG1		0x134
#define CORE_CSR_CDC_CAL_TIMER_CFG0	0x138
#define CORE_TIMER_ENA			BIT(16)

#define CORE_CSR_CDC_CAL_TIMER_CFG1	0x13C
#define CORE_CSR_CDC_REFCOUNT_CFG	0x140
#define CORE_CSR_CDC_COARSE_CAL_CFG	0x144
#define CORE_CDC_OFFSET_CFG		0x14C
#define CORE_CSR_CDC_DELAY_CFG		0x150
#define CORE_CDC_SLAVE_DDA_CFG		0x160
#define CORE_CSR_CDC_STATUS0		0x164
#define CORE_CALIBRATION_DONE		BIT(0)

#define CORE_CDC_ERROR_CODE_MASK	0x7000000

#define CQ_CMD_DBG_RAM                  0x110
#define CQ_CMD_DBG_RAM_WA               0x150
#define CQ_CMD_DBG_RAM_OL               0x154

#define CORE_CSR_CDC_GEN_CFG		0x178
#define CORE_CDC_SWITCH_BYPASS_OFF	BIT(0)
#define CORE_CDC_SWITCH_RC_EN		BIT(1)

#define CORE_CDC_T4_DLY_SEL		BIT(0)
#define CORE_CMDIN_RCLK_EN		BIT(1)
#define CORE_START_CDC_TRAFFIC		BIT(6)

#define CORE_PWRSAVE_DLL	BIT(3)
#define CORE_FIFO_ALT_EN	BIT(10)
#define CORE_CMDEN_HS400_INPUT_MASK_CNT BIT(13)
#define DDR_CONFIG_POR_VAL		0x80040873
#define DLL_USR_CTL_POR_VAL		0x10800
#define ENABLE_DLL_LOCK_STATUS		BIT(26)
#define FINE_TUNE_MODE_EN		BIT(27)
#define BIAS_OK_SIGNAL			BIT(29)
#define DLL_CONFIG_3_POR_VAL		0x10

#define INVALID_TUNING_PHASE	-1
#define SDHCI_MSM_MIN_CLOCK	400000
#define CORE_FREQ_100MHZ	(100 * 1000 * 1000)
#define TCXO_FREQ		19200000

#define ROUND(x, y)		((x) / (y) + \
				((x) % (y) * 10 / (y) >= 5 ? 1 : 0))

#define CDR_SELEXT_SHIFT	20
#define CDR_SELEXT_MASK		(0xf << CDR_SELEXT_SHIFT)
#define CMUX_SHIFT_PHASE_SHIFT	24
#define CMUX_SHIFT_PHASE_MASK	(7 << CMUX_SHIFT_PHASE_SHIFT)

#define MSM_MMC_AUTOSUSPEND_DELAY_MS	10

/* Timeout value to avoid infinite waiting for pwr_irq */
#define MSM_PWR_IRQ_TIMEOUT_MS 5000

/* Max load for eMMC Vdd-io supply */
#define MMC_VQMMC_MAX_LOAD_UA	325000

/*
 * Due to level shifter insertion, HS mode frequency is reduced to 37.5MHz
 * but clk's driver supply 37MHz only and uses ceil ops. So vote for
 * 37MHz to avoid picking next ceil value.
 */
#define LEVEL_SHIFTER_HIGH_SPEED_FREQ	37500000

#define VS_CAPABILITIES_SDR_50_SUPPORT BIT(0)
#define VS_CAPABILITIES_SDR_104_SUPPORT BIT(1)
#define VS_CAPABILITIES_DDR_50_SUPPORT BIT(2)
/* Max number of log pages */
#define SDHCI_MSM_MAX_LOG_SZ	10
#define sdhci_msm_log_str(host, fmt, ...)	\
	do {	\
		if (host->sdhci_msm_ipc_log_ctx && host->dbg_en)	\
			ipc_log_string(host->sdhci_msm_ipc_log_ctx,	\
					"%s: " fmt, __func__, ##__VA_ARGS__);\
	} while (0)

#define msm_host_readl(msm_host, host, offset) \
	msm_host->var_ops->msm_readl_relaxed(host, offset)

#define msm_host_writel(msm_host, val, host, offset) \
	msm_host->var_ops->msm_writel_relaxed(val, host, offset)

/* CQHCI vendor specific registers */
#define CQHCI_VENDOR_CFG1	0xA00
#define CQHCI_VENDOR_DIS_RST_ON_CQ_EN	(0x3 << 13)
#define RCLK_TOGGLE BIT(1)

/* enum for writing to TLMM_NORTH_SPARE register as defined by pinctrl API */
#define TLMM_NORTH_SPARE	2
#define TLMM_NORTH_SPARE_CORE_IE	BIT(15)

#define SDHCI_CMD_FLAGS_MASK	0xff
#define DOMAIN_IDX_POWER	0
#define DOMAIN_IDX_PERF		1

struct sdhci_msm_offset {
	u32 core_hc_mode;
	u32 core_mci_data_cnt;
	u32 core_mci_status;
	u32 core_mci_fifo_cnt;
	u32 core_mci_version;
	u32 core_generics;
	u32 core_testbus_config;
	u32 core_testbus_sel2_bit;
	u32 core_testbus_ena;
	u32 core_testbus_sel2;
	u32 core_pwrctl_status;
	u32 core_pwrctl_mask;
	u32 core_pwrctl_clear;
	u32 core_pwrctl_ctl;
	u32 core_sdcc_debug_reg;
	u32 core_dll_config;
	u32 core_dll_status;
	u32 core_vendor_spec;
	u32 core_vendor_spec_adma_err_addr0;
	u32 core_vendor_spec_adma_err_addr1;
	u32 core_vendor_spec_func2;
	u32 core_vendor_spec_capabilities0;
	u32 core_vendor_spec_capabilities1;
	u32 core_ddr_200_cfg;
	u32 core_vendor_spec3;
	u32 core_dll_config_2;
	u32 core_dll_config_3;
	u32 core_ddr_config_old; /* Applicable to sdcc minor ver < 0x49 */
	u32 core_ddr_config;
	u32 core_dll_usr_ctl; /* Present on SDCC5.1 onwards */
};

static const struct sdhci_msm_offset sdhci_msm_v5_offset = {
	.core_mci_data_cnt = 0x35c,
	.core_mci_status = 0x324,
	.core_mci_fifo_cnt = 0x308,
	.core_mci_version = 0x318,
	.core_generics = 0x320,
	.core_testbus_config = 0x32c,
	.core_testbus_sel2_bit = 3,
	.core_testbus_ena = (1 << 31),
	.core_testbus_sel2 = (1 << 3),
	.core_pwrctl_status = 0x240,
	.core_pwrctl_mask = 0x244,
	.core_pwrctl_clear = 0x248,
	.core_pwrctl_ctl = 0x24c,
	.core_sdcc_debug_reg = 0x358,
	.core_dll_config = 0x200,
	.core_dll_status = 0x208,
	.core_vendor_spec = 0x20c,
	.core_vendor_spec_adma_err_addr0 = 0x214,
	.core_vendor_spec_adma_err_addr1 = 0x218,
	.core_vendor_spec_func2 = 0x210,
	.core_vendor_spec_capabilities0 = 0x21c,
	.core_vendor_spec_capabilities1 = 0x220,
	.core_ddr_200_cfg = 0x224,
	.core_vendor_spec3 = 0x250,
	.core_dll_config_2 = 0x254,
	.core_dll_config_3 = 0x258,
	.core_ddr_config = 0x25c,
	.core_dll_usr_ctl = 0x388,
};

static const struct sdhci_msm_offset sdhci_msm_mci_offset = {
	.core_hc_mode = 0x78,
	.core_mci_data_cnt = 0x30,
	.core_mci_status = 0x34,
	.core_mci_fifo_cnt = 0x44,
	.core_mci_version = 0x050,
	.core_generics = 0x70,
	.core_testbus_config = 0x0cc,
	.core_testbus_sel2_bit = 4,
	.core_testbus_ena = (1 << 3),
	.core_testbus_sel2 = (1 << 4),
	.core_pwrctl_status = 0xdc,
	.core_pwrctl_mask = 0xe0,
	.core_pwrctl_clear = 0xe4,
	.core_pwrctl_ctl = 0xe8,
	.core_sdcc_debug_reg = 0x124,
	.core_dll_config = 0x100,
	.core_dll_status = 0x108,
	.core_vendor_spec = 0x10c,
	.core_vendor_spec_adma_err_addr0 = 0x114,
	.core_vendor_spec_adma_err_addr1 = 0x118,
	.core_vendor_spec_func2 = 0x110,
	.core_vendor_spec_capabilities0 = 0x11c,
	.core_ddr_200_cfg = 0x184,
	.core_vendor_spec3 = 0x1b0,
	.core_dll_config_2 = 0x1b4,
	.core_dll_config_3 = 0x1b8,
	.core_ddr_config_old = 0x1b8,
	.core_ddr_config = 0x1bc,
};

struct sdhci_msm_variant_ops {
	u32 (*msm_readl_relaxed)(struct sdhci_host *host, u32 offset);
	void (*msm_writel_relaxed)(u32 val, struct sdhci_host *host,
			u32 offset);
};

/*
 * From V5, register spaces have changed. Wrap this info in a structure
 * and choose the data_structure based on version info mentioned in DT.
 */
struct sdhci_msm_variant_info {
	bool mci_removed;
	bool restore_dll_config;
	const struct sdhci_msm_variant_ops *var_ops;
	const struct sdhci_msm_offset *offset;
	/*
	 * pd_names != NULL marks this variant as firmware managed;
	 * see fw_managed detection in sdhci_msm_probe().
	 */
	const struct dev_pm_domain_attach_data pd_data;
};

struct msm_bus_vectors {
	u64 ab;
	u64 ib;
};

struct msm_bus_path {
	unsigned int num_paths;
	struct msm_bus_vectors *vec;
};

struct sdhci_msm_bus_vote_data {
	const char *name;
	unsigned int num_usecase;
	struct msm_bus_path *usecase;

	unsigned int *bw_vecs;
	unsigned int bw_vecs_size;

	struct icc_path *sdhc_ddr;
	struct icc_path *cpu_sdhc;

	u32 curr_vote;
};

/*
 * DLL registers which needs be programmed with configuration settings.
 * Add any new register only at the end and don't change the
 * sequence.
 */
struct sdhci_msm_dll_cfg {
	u32 dll_config;
	u32 dll_config_2;
	u32 dll_config_3;
	u32 dll_usr_ctl;
	u32 ddr_config;
};

struct cqe_regs_restore {
	u32 cqe_vendor_cfg1;
};

struct sdhci_msm_regs_restore {
	bool is_supported;
	bool is_valid;
	u32 vendor_pwrctl_mask;
	u32 vendor_pwrctl_ctl;
	u32 vendor_caps_0;
	u32 vendor_func;
	u32 vendor_func2;
	u32 vendor_func3;
	u32 hc_2c_2e;
	u32 hc_28_2a;
	u32 hc_34_36;
	u32 hc_38_3a;
	u32 hc_3c_3e;
	u32 hc_caps_1;
	u32 testbus_config;
	u32 dll_config;
	u32 dll_config2;
	u32 dll_config3;
	u32 dll_usr_ctl;
	u32 ext_fb_clk;
};

enum vdd_io_level {
	/* set vdd_io_data->low_vol_level */
	VDD_IO_LOW,
	/* set vdd_io_data->high_vol_level */
	VDD_IO_HIGH,
	/*
	 * set whatever there in voltage_level (third argument) of
	 * sdhci_msm_set_vdd_io_vol() function.
	 */
	VDD_IO_SET_LEVEL,
};

enum dll_init_context {
	DLL_INIT_NORMAL = 0,
	DLL_INIT_FROM_CX_COLLAPSE_EXIT,
};

/* This structure keeps information per regulator */
struct sdhci_msm_reg_data {
	struct sdhci_msm_host *msm_host;
	/* voltage regulator handle */
	struct regulator *reg;
	/* Alternative voltage enable/disable regulator handle */
	struct regulator *reg_en_dis;
	/* regulator name */
	const char *name;
	/* regulator enable/disable name */
	char en_dis_name[32];
	/* voltage level to be set */
	u32 low_vol_level;
	u32 high_vol_level;
	/* Load values for low power and high power mode */
	u32 lpm_uA;
	u32 hpm_uA;

	/* is this regulator enabled? */
	bool is_enabled;
	/* is this regulator needs to be always on? */
	bool is_always_on;
	/* is low power mode setting required for this regulator? */
	bool lpm_sup;
	bool set_voltage_sup;
	bool is_voltage_supplied;
	bool multi_card_tray_wa_needed;
};

/*
 * This structure keeps information for all the
 * regulators required for a SDCC slot.
 */
struct sdhci_msm_vreg_data {
	/* keeps VDD/VCC regulator info */
	struct sdhci_msm_reg_data *vdd_data;
	 /* keeps VDD IO regulator info */
	struct sdhci_msm_reg_data *vdd_io_data;
};

/* Per cpu cluster qos group */
struct qos_cpu_group {
	cpumask_t mask;	/* CPU mask of cluster */
	unsigned int *votes;	/* Different votes for cluster */
	struct dev_pm_qos_request *qos_req;	/* Pointer to host qos request*/
	bool voted;
	struct sdhci_msm_host *host;
	bool initialized;
	bool curr_vote;
};

/* Per host qos request structure */
struct sdhci_msm_qos_req {
	struct qos_cpu_group *qcg;	/* CPU group per host */
	unsigned int num_groups;	/* Number of groups */
	unsigned int active_mask;	/* Active affine irq mask */
};

enum constraint {
	QOS_PERF,
	QOS_POWER,
	QOS_MAX,
};

#define TUNING_PHASE_SIZE	11

struct ggc_bus_mode_cfg_t {
	u32 tx_selb_tb[TUNING_PHASE_SIZE];
	u32 all_selb_tb[TUNING_PHASE_SIZE];
	u32 tx_selb_failed_history;
	int bus_mode;
	int default_sela;
	int default_selb;
	u32 default_delaycode;
	u32 dll_voltage_unlock_cnt[4];
	u32 max_delaycode;
	u32 min_delaycode;
	u32 delaycode_narrowdown_index;
	u32 fail_phase;
};

struct t_gg_reg_strt {
	u32 ofs;
	u32 mask;
	u32 value;
};

enum tuning_stat_et {
	NO_TUNING = 0,
	OUTPUT_PASS_TYPE = 1,
	SET_PHASE_FAIL_TYPE = 2,
	TUNING_FAIL_TYPE = 3,
	READ_STATUS_FAIL_TYPE = 4,
	TUNING_CMD7_TIMEOUT = 5,
	RETUNING_CASE = 6,
};

struct rl_bit_lct {
	u8 bits;
	u8 rl_bits;
};

struct chk_type_t {
	u8 right_valid:1;
	u8 first_valid:1;
	u8 record_valid:1;
	u8 reserved:5;
};

static const char *const op_dbg_str[] = {
	"no tuning",
	"pass",
	"set_phase_fail",
	"tuning fail",
	"read status fail",
	"tuning CMD7 timeout",
	"retuning case"
};

struct ggc_platform_t {
	struct ggc_bus_mode_cfg_t sdr50;
	struct ggc_bus_mode_cfg_t sdr104;
	struct ggc_bus_mode_cfg_t *cur_bus_mode;
	struct t_gg_reg_strt pha_stas_rx_low32;
	struct t_gg_reg_strt pha_stas_rx_high32;
	struct t_gg_reg_strt pha_stas_tx_low32;
	struct t_gg_reg_strt pha_stas_tx_high32;
	struct t_gg_reg_strt dll_sela_after_mask;
	struct t_gg_reg_strt dll_selb_after_mask;

	struct t_gg_reg_strt dll_delay_100m_backup;
	struct t_gg_reg_strt dll_delay_200m_backup;

	struct t_gg_reg_strt dll_sela_100m_cfg;
	struct t_gg_reg_strt dll_sela_200m_cfg;
	struct t_gg_reg_strt dll_selb_100m_cfg;
	struct t_gg_reg_strt dll_selb_200m_cfg;
	struct t_gg_reg_strt dll_selb_100m_cfg_en;
	struct t_gg_reg_strt dll_selb_200m_cfg_en;
	struct t_gg_reg_strt internl_tuning_en_100m;
	struct t_gg_reg_strt internl_tuning_en_200m;
	struct t_gg_reg_strt cmd19_cnt_cfg;

	struct t_gg_reg_strt inject_failure_for_tuning_enable_cfg;
	struct t_gg_reg_strt inject_failure_for_200m_tuning_cfg;
	struct t_gg_reg_strt inject_failure_for_100m_tuning_cfg;

	struct mmc_card *card;

	int def_sela_100m;
	int def_sela_200m;
	int def_selb_100m;
	int def_selb_200m;

	u32 _gg_reg_cur[16];
	u8 _cur_read_buf[512];
	u32 ggc_cur_ssc_level;
	int ssc_crc_recovery_retry_cnt;
	u32 crc_retry_cnt;
	u32 cclk_ds_18v;
	u32 cdata_ds_18v;
	u32 ds_inc_cnt;
	bool ggc_400k_update_ds_flg;
	bool ggc_400k_update_ssc_arg_flg;
	bool ggc_ocb_in_check;
	bool dll_unlock_reinit_flg;
	u8 driver_strength_reinit_flg;
	bool tuning_cmd7_timeout_reinit_flg;
	u32 tuning_cmd7_timeout_reinit_cnt;
	u32 ggc_400k_setting_change_flg: 1;
	u32 ggc_cur_sela;
	u32 ggc_target_selb;
	u32 target_tuning_pass_win;
	bool selx_tuning_done_flag;
	u32 ggc_cmd_tx_selb_failed_range;
	int ggc_sw_selb_tuning_first_selb;
	enum tuning_stat_et ggc_sela_tuning_result[11];
	int dll_voltage_scan_map[4];
	int cur_dll_voltage_idx;

	int sdr50_notuning_sela_inject_flag;
	int sdr50_notuning_crc_error_flag;
	u32 sdr50_notuning_sela_rx_inject;
	u32 bh201_sdr50_sela_sw_inject;
	u32 bh201_sdr50_selb_hw_inject;
	u32 bh201_sdr104_selb_hw_inject;
	u32 bh201_drive_strength;
	bool tuning_in_progress;
	int bh201_used;

	u8			v18_disable;
	u8			legacy_mode;
	u8			card_removed_flag;
	u8			degrade;
	u8			degrade_count;
	u8			tuning_fail_count;
	u32			origin_cap;
	unsigned int  cur_sd_bus_speed;
};

static void bht_load(struct mmc_host *mmc_host, struct mmc_card *card);
static int sdhci_bht_execute_tuning(struct mmc_host *mmc, u32 opcode);
static void ggc_chip_init(struct sdhci_host *host);
static void ggc_tuning_result_reset(struct sdhci_host *host);
static void sdhci_bh201_parse(struct mmc_host *mmc_host);

#define   GGC_CFG_DATA {0x07000000, 0x07364022, 0x01015412, 0x01062400,\
	0x10400076, 0x00025432, 0x01046076, 0x62011000,\
	0x31503106, 0x64141731, 0x10057513, 0x00336200,\
	0x00020006, 0x40001400, 0x12200310, 0x4A416677}

static int sdhci_bh201_create_sysfs(struct mmc_host *mmc);
static void sdhci_bh201_remove_sysfs(struct mmc_host *mmc);

struct sdhci_msm_host {
	struct ggc_platform_t ggc;
	struct platform_device *pdev;
	void __iomem *core_mem;	/* MSM SDCC mapped address */
#if IS_ENABLED(CONFIG_QTI_HW_KEY_MANAGER)
	void __iomem *ice_hwkm_mem;
#endif
	int pwr_irq;		/* power irq */
	struct clk *bus_clk;	/* SDHC bus voter clock */
	struct clk *xo_clk;	/* TCXO clk needed for FLL feature of cm_dll*/
	/* core, iface, cal and sleep clocks */
	struct clk_bulk_data bulk_clks[4];
	struct dev_pm_domain_list *pd_list;
#ifdef CONFIG_MMC_CRYPTO
	struct qcom_ice *ice;
#endif
	unsigned long clk_rate;
	struct sdhci_msm_vreg_data *vreg_data;
	struct mmc_host *mmc;
	int opp_token;
	bool has_opp_table;
	struct cqhci_host *cq_host;
	bool use_14lpp_dll_reset;
	bool tuning_done;
	bool calibration_done;
	u8 saved_tuning_phase;
	bool use_cdclp533;
	u32 curr_pwr_state;
	u32 curr_io_level;
	wait_queue_head_t pwr_irq_wait;
	bool pwr_irq_flag;
	u32 caps_0;
	bool mci_removed;
	bool restore_dll_config;
	bool fw_managed;
	const struct sdhci_msm_variant_ops *var_ops;
	const struct sdhci_msm_offset *offset;
	bool use_cdr;
	u32 transfer_mode;
	bool updated_ddr_cfg;
	bool skip_bus_bw_voting;
	struct sdhci_msm_bus_vote_data *bus_vote_data;
	struct delayed_work bus_vote_work;
	struct workqueue_struct *workq;	/* QoS work queue */
	struct sdhci_msm_qos_req *sdhci_qos;
	struct irq_affinity_notify affinity_notify;
	struct device_attribute pm_qos;
	u32 pm_qos_delay;
	bool cqhci_offset_changed;
	bool reg_store;
	bool vbias_skip_wa;
	struct reset_control *core_reset;
	bool pltfm_init_done;
	bool fake_core_3_0v_support;
	bool use_7nm_dll;
	struct sdhci_msm_regs_restore regs_restore;
	struct cqe_regs_restore cqe_regs;
	u32 *sup_ice_clk_table;
	unsigned char sup_ice_clk_cnt;
	u32 ice_clk_max;
	u32 ice_clk_min;
	u32 ice_clk_rate;
	bool uses_tassadar_dll;
	bool uses_level_shifter;
	bool dll_lock_bist_fail_wa;
	u32 dll_config;
	u32 ddr_config;
	u16 last_cmd;
	bool vqmmc_enabled;
	void *sdhci_msm_ipc_log_ctx;
	bool dbg_en;
	bool enable_ext_fb_clk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_default;
	struct pinctrl_state *pinctrl_state_gpio;
	struct sdhci_msm_dll_cfg *dll_cfg;
};

static struct sdhci_msm_host *sdhci_slot[3];

static int sdhci_msm_update_qos_constraints(struct qos_cpu_group *qcg,
					enum constraint type);
static void sdhci_msm_bus_voting(struct sdhci_host *host, bool enable);

static int sdhci_msm_dt_get_array(struct device *dev, const char *prop_name,
				u32 **bw_vecs, int *len, u32 size);

static unsigned int sdhci_msm_get_sup_clk_rate(struct sdhci_host *host,
				u32 req_clk);
static void sdhci_msm_dump_pwr_ctrl_regs(struct sdhci_host *host);
static void sdhci_msm_vbias_bypass_wa(struct sdhci_host *host);

static const struct sdhci_msm_offset *sdhci_priv_msm_offset(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	return msm_host->offset;
}

/*
 * APIs to read/write to vendor specific registers which were there in the
 * core_mem region before MCI was removed.
 */
static u32 sdhci_msm_mci_variant_readl_relaxed(struct sdhci_host *host,
		u32 offset)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	return readl_relaxed(msm_host->core_mem + offset);
}

static u32 sdhci_msm_v5_variant_readl_relaxed(struct sdhci_host *host,
		u32 offset)
{
	return readl_relaxed(host->ioaddr + offset);
}

static void sdhci_msm_mci_variant_writel_relaxed(u32 val,
		struct sdhci_host *host, u32 offset)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	writel_relaxed(val, msm_host->core_mem + offset);
}

static void sdhci_msm_v5_variant_writel_relaxed(u32 val,
		struct sdhci_host *host, u32 offset)
{
	writel_relaxed(val, host->ioaddr + offset);
}

static unsigned int msm_get_clock_mult_for_bus_mode(struct sdhci_host *host)
{
	struct mmc_ios ios = host->mmc->ios;
	/*
	 * The SDHC requires internal clock frequency to be double the
	 * actual clock that will be set for DDR mode. The controller
	 * uses the faster clock(100/400MHz) for some of its parts and
	 * send the actual required clock (50/200MHz) to the card.
	 */
	if (ios.timing == MMC_TIMING_UHS_DDR50 ||
	    ios.timing == MMC_TIMING_MMC_DDR52 ||
	    (ios.timing == MMC_TIMING_MMC_HS400 &&
	    ios.clock == MMC_HS200_MAX_DTR) ||
	    host->flags & SDHCI_HS400_TUNING)
		return 2;
	return 1;
}

static void msm_set_clock_rate_for_bus_mode(struct sdhci_host *host,
					    unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct mmc_ios curr_ios = host->mmc->ios;
	struct clk *core_clk = msm_host->bulk_clks[0].clk;
	unsigned long achieved_rate;
	unsigned int desired_rate;
	unsigned int mult;
	int rc;

	mult = msm_get_clock_mult_for_bus_mode(host);
	desired_rate = clock * mult;

	if (curr_ios.timing == MMC_TIMING_SD_HS &&
			msm_host->uses_level_shifter &&
			!msm_host->enable_ext_fb_clk)
		desired_rate = LEVEL_SHIFTER_HIGH_SPEED_FREQ;

	rc = dev_pm_opp_set_rate(mmc_dev(host->mmc), desired_rate);
	if (rc) {
		pr_err("%s: Failed to set clock at rate %u at timing %d\n",
		       mmc_hostname(host->mmc), desired_rate,
		       curr_ios.timing);
		return;
	}
	/*
	 * Qualcomm Technologies, Inc. clock drivers by default round
	 * clock _up_ if they can't make the requested rate.  This is not
	 * good for SD.  Yell if we encounter it.
	 */
	achieved_rate = clk_get_rate(core_clk);
	if (achieved_rate > desired_rate)
		pr_debug("%s: Card appears overclocked; req %u Hz, actual %lu Hz\n",
			mmc_hostname(host->mmc), desired_rate, achieved_rate);
	host->mmc->actual_clock = achieved_rate / mult;
	msm_host->clk_rate = desired_rate;

	pr_debug("%s: Setting clock at rate %lu at timing %d\n",
		 mmc_hostname(host->mmc), achieved_rate,
		 curr_ios.timing);
	sdhci_msm_log_str(msm_host, "Setting clock at rate %lu at timing %d\n",
			clk_get_rate(core_clk), curr_ios.timing);
}

static void msm_fmr_set_clock_rate_for_bus_mode(struct sdhci_host *host,
					    unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct device *dev = mmc_dev(host->mmc);
	struct device *perf_dev;
	struct dev_pm_opp *opp;
	unsigned long clock_long = (unsigned long)clock;
	unsigned long desired_rate;
	unsigned int mult;
	int ret;

	if (!msm_host->pd_list) {
		dev_err(dev, "pd_list is NULL\n");
		return;
	}

	perf_dev = msm_host->pd_list->pd_devs[DOMAIN_IDX_PERF];
	if (!perf_dev) {
		dev_err(dev, "perf_dev is NULL\n");
		return;
	}

	mult = msm_get_clock_mult_for_bus_mode(host);
	desired_rate = clock_long * mult;

	/*
	 * Find the nearest frequency level for the requested frequency.
	 */
	opp = dev_pm_opp_find_freq_floor(perf_dev, &desired_rate);
	if (IS_ERR(opp)) {
		dev_err(dev, "Failed to find opp for freq %lu ret=%ld\n",
			clock_long, PTR_ERR(opp));
	} else {
		ret = dev_pm_opp_set_opp(perf_dev, opp);
		if (ret)
			dev_err(dev, "No opp for freq %lu ret=%ld\n", clock_long, PTR_ERR(opp));
		dev_pm_opp_put(opp);
	}

	host->mmc->actual_clock = clock;

	/* Stash the rate we requested to use in sdhci_msm_runtime_resume() */
	msm_host->clk_rate = desired_rate;
}

/* Platform specific tuning */
static inline int msm_dll_poll_ck_out_en(struct sdhci_host *host, u8 poll)
{
	u32 wait_cnt = 50;
	u8 ck_out_en;
	struct mmc_host *mmc = host->mmc;
	const struct sdhci_msm_offset *msm_offset =
					sdhci_priv_msm_offset(host);

	/* Poll for CK_OUT_EN bit.  max. poll time = 50us */
	ck_out_en = !!(readl_relaxed(host->ioaddr +
			msm_offset->core_dll_config) & CORE_CK_OUT_EN);

	while (ck_out_en != poll) {
		if (--wait_cnt == 0) {
			dev_err(mmc_dev(mmc), "%s: CK_OUT_EN bit is not %d\n",
			       mmc_hostname(mmc), poll);
			return -ETIMEDOUT;
		}
		udelay(1);

		ck_out_en = !!(readl_relaxed(host->ioaddr +
			msm_offset->core_dll_config) & CORE_CK_OUT_EN);
	}

	return 0;
}

static int msm_config_cm_dll_phase(struct sdhci_host *host, u8 phase)
{
	int rc;
	static const u8 grey_coded_phase_table[] = {
		0x0, 0x1, 0x3, 0x2, 0x6, 0x7, 0x5, 0x4,
		0xc, 0xd, 0xf, 0xe, 0xa, 0xb, 0x9, 0x8
	};
	unsigned long flags;
	u32 config;
	struct mmc_host *mmc = host->mmc;
	const struct sdhci_msm_offset *msm_offset =
					sdhci_priv_msm_offset(host);

	if (phase > 0xf)
		return -EINVAL;

	spin_lock_irqsave(&host->lock, flags);

	config = readl_relaxed(host->ioaddr + msm_offset->core_dll_config);
	config &= ~(CORE_CDR_EN | CORE_CK_OUT_EN);
	config |= (CORE_CDR_EXT_EN | CORE_DLL_EN);
	writel_relaxed(config, host->ioaddr + msm_offset->core_dll_config);

	/* Wait until CK_OUT_EN bit of DLL_CONFIG register becomes '0' */
	rc = msm_dll_poll_ck_out_en(host, 0);
	if (rc)
		goto err_out;

	/*
	 * Write the selected DLL clock output phase (0 ... 15)
	 * to CDR_SELEXT bit field of DLL_CONFIG register.
	 */
	config = readl_relaxed(host->ioaddr + msm_offset->core_dll_config);
	config &= ~CDR_SELEXT_MASK;
	config |= grey_coded_phase_table[phase] << CDR_SELEXT_SHIFT;
	writel_relaxed(config, host->ioaddr + msm_offset->core_dll_config);

	config = readl_relaxed(host->ioaddr + msm_offset->core_dll_config);
	config |= CORE_CK_OUT_EN;
	writel_relaxed(config, host->ioaddr + msm_offset->core_dll_config);

	/* Wait until CK_OUT_EN bit of DLL_CONFIG register becomes '1' */
	rc = msm_dll_poll_ck_out_en(host, 1);
	if (rc)
		goto err_out;

	config = readl_relaxed(host->ioaddr + msm_offset->core_dll_config);
	config |= CORE_CDR_EN;
	config &= ~CORE_CDR_EXT_EN;
	writel_relaxed(config, host->ioaddr + msm_offset->core_dll_config);
	goto out;

err_out:
	dev_err(mmc_dev(mmc), "%s: Failed to set DLL phase: %d\n",
	       mmc_hostname(mmc), phase);
out:
	spin_unlock_irqrestore(&host->lock, flags);
	return rc;
}

/*
 * Find out the greatest range of consecuitive selected
 * DLL clock output phases that can be used as sampling
 * setting for SD3.0 UHS-I card read operation (in SDR104
 * timing mode) or for eMMC4.5 card read operation (in
 * HS400/HS200 timing mode).
 * Select the 3/4 of the range and configure the DLL with the
 * selected DLL clock output phase.
 */

static int msm_find_most_appropriate_phase(struct sdhci_host *host,
					   u8 *phase_table, u8 total_phases)
{
	int ret;
	u8 ranges[MAX_PHASES][MAX_PHASES] = { {0}, {0} };
	u8 phases_per_row[MAX_PHASES] = { 0 };
	int row_index = 0, col_index = 0, selected_row_index = 0, curr_max = 0;
	int i, cnt, phase_0_raw_index = 0, phase_15_raw_index = 0;
	bool phase_0_found = false, phase_15_found = false;
	struct mmc_host *mmc = host->mmc;

	if (!total_phases || (total_phases > MAX_PHASES)) {
		dev_err(mmc_dev(mmc), "%s: Invalid argument: total_phases=%d\n",
		       mmc_hostname(mmc), total_phases);
		return -EINVAL;
	}

	for (cnt = 0; cnt < total_phases; cnt++) {
		ranges[row_index][col_index] = phase_table[cnt];
		phases_per_row[row_index] += 1;
		col_index++;

		if ((cnt + 1) == total_phases) {
			continue;
		/* check if next phase in phase_table is consecutive or not */
		} else if ((phase_table[cnt] + 1) != phase_table[cnt + 1]) {
			row_index++;
			col_index = 0;
		}
	}

	if (row_index >= MAX_PHASES)
		return -EINVAL;

	/* Check if phase-0 is present in first valid window? */
	if (!ranges[0][0]) {
		phase_0_found = true;
		phase_0_raw_index = 0;
		/* Check if cycle exist between 2 valid windows */
		for (cnt = 1; cnt <= row_index; cnt++) {
			if (phases_per_row[cnt]) {
				for (i = 0; i < phases_per_row[cnt]; i++) {
					if (ranges[cnt][i] == 15) {
						phase_15_found = true;
						phase_15_raw_index = cnt;
						break;
					}
				}
			}
		}
	}

	/* If 2 valid windows form cycle then merge them as single window */
	if (phase_0_found && phase_15_found) {
		/* number of phases in raw where phase 0 is present */
		u8 phases_0 = phases_per_row[phase_0_raw_index];
		/* number of phases in raw where phase 15 is present */
		u8 phases_15 = phases_per_row[phase_15_raw_index];

		if (phases_0 + phases_15 >= MAX_PHASES)
			/*
			 * If there are more than 1 phase windows then total
			 * number of phases in both the windows should not be
			 * more than or equal to MAX_PHASES.
			 */
			return -EINVAL;

		/* Merge 2 cyclic windows */
		i = phases_15;
		for (cnt = 0; cnt < phases_0; cnt++) {
			ranges[phase_15_raw_index][i] =
			    ranges[phase_0_raw_index][cnt];
			if (++i >= MAX_PHASES)
				break;
		}

		phases_per_row[phase_0_raw_index] = 0;
		phases_per_row[phase_15_raw_index] = phases_15 + phases_0;
	}

	for (cnt = 0; cnt <= row_index; cnt++) {
		if (phases_per_row[cnt] > curr_max) {
			curr_max = phases_per_row[cnt];
			selected_row_index = cnt;
		}
	}

	i = (curr_max * 3) / 4;
	if (i)
		i--;

	ret = ranges[selected_row_index][i];

	if (ret >= MAX_PHASES) {
		ret = -EINVAL;
		dev_err(mmc_dev(mmc), "%s: Invalid phase selected=%d\n",
		       mmc_hostname(mmc), ret);
	}

	return ret;
}

static inline void msm_cm_dll_set_freq(struct sdhci_host *host)
{
	u32 mclk_freq = 0, config;
	const struct sdhci_msm_offset *msm_offset =
					sdhci_priv_msm_offset(host);

	/* Program the MCLK value to MCLK_FREQ bit field */
	if (host->clock <= 112000000)
		mclk_freq = 0;
	else if (host->clock <= 125000000)
		mclk_freq = 1;
	else if (host->clock <= 137000000)
		mclk_freq = 2;
	else if (host->clock <= 150000000)
		mclk_freq = 3;
	else if (host->clock <= 162000000)
		mclk_freq = 4;
	else if (host->clock <= 175000000)
		mclk_freq = 5;
	else if (host->clock <= 187000000)
		mclk_freq = 6;
	else if (host->clock <= 208000000)
		mclk_freq = 7;

	config = readl_relaxed(host->ioaddr + msm_offset->core_dll_config);
	config &= ~CMUX_SHIFT_PHASE_MASK;
	config |= mclk_freq << CMUX_SHIFT_PHASE_SHIFT;
	writel_relaxed(config, host->ioaddr + msm_offset->core_dll_config);
}

/* Initialize the DLL (Programmable Delay Line) */
static int msm_init_cm_dll(struct sdhci_host *host,
				enum dll_init_context init_context)
{
	struct mmc_host *mmc = host->mmc;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct mmc_ios curr_ios = mmc->ios;
	int wait_cnt = 50;
	int rc = 0;
	unsigned long flags, dll_clock = 0;
	u32 ddr_cfg_offset, core_vendor_spec;
	const struct sdhci_msm_offset *msm_offset =
					msm_host->offset;

	dll_clock = sdhci_msm_get_sup_clk_rate(host, host->clock);
	spin_lock_irqsave(&host->lock, flags);

	core_vendor_spec = readl_relaxed(host->ioaddr +
			msm_offset->core_vendor_spec);

	/*
	 * Step 1 - Always disable PWRSAVE during the DLL power
	 * up regardless of its current setting.
	 */

	writel_relaxed((core_vendor_spec & ~CORE_CLK_PWRSAVE),
			host->ioaddr +
			msm_offset->core_vendor_spec);

	if (msm_host->use_14lpp_dll_reset) {
		/* Step 2 - Disable CK_OUT */
		writel_relaxed((readl_relaxed(host->ioaddr +
			msm_offset->core_dll_config)
			& ~CORE_CK_OUT_EN), host->ioaddr +
			msm_offset->core_dll_config);

		/* Step 3 - Disable the DLL clock */
		writel_relaxed((readl_relaxed(host->ioaddr +
			msm_offset->core_dll_config_2)
			| CORE_DLL_CLOCK_DISABLE), host->ioaddr +
			msm_offset->core_dll_config_2);
	}

	/*
	 * Step 4 - Write 1 to DLL_RST bit of DLL_CONFIG register
	 * and Write 1 to DLL_PDN bit of DLL_CONFIG register.
	 */
	writel_relaxed((readl_relaxed(host->ioaddr +
		msm_offset->core_dll_config) | CORE_DLL_RST),
		host->ioaddr + msm_offset->core_dll_config);

	writel_relaxed((readl_relaxed(host->ioaddr +
		msm_offset->core_dll_config) | CORE_DLL_PDN),
		host->ioaddr + msm_offset->core_dll_config);

	/*
	 * Step 5 and Step 6 - Configure Tassadar DLL and USER_CTRL
	 * (Only applicable for 7FF projects).
	 */
	if (msm_host->use_7nm_dll) {
		if (msm_host->dll_cfg) {
			writel_relaxed(msm_host->dll_cfg->dll_config_3,
					host->ioaddr +
					msm_offset->core_dll_config_3);
			writel_relaxed(msm_host->dll_cfg->dll_usr_ctl,
					host->ioaddr +
					msm_offset->core_dll_usr_ctl);
		} else {
			writel_relaxed(DLL_CONFIG_3_POR_VAL, host->ioaddr +
				msm_offset->core_dll_config_3);
			writel_relaxed(DLL_USR_CTL_POR_VAL | FINE_TUNE_MODE_EN |
					ENABLE_DLL_LOCK_STATUS | BIAS_OK_SIGNAL,
					host->ioaddr +
					msm_offset->core_dll_usr_ctl);
		}
	}

	/*
	 * Step 8 - Set DDR_CONFIG since step 7 is setting TEST_CTRL
	 * that can be skipped.
	 */
	if (msm_host->updated_ddr_cfg)
		ddr_cfg_offset = msm_offset->core_ddr_config;
	else
		ddr_cfg_offset = msm_offset->core_ddr_config_old;

	if (msm_host->dll_cfg && msm_host->dll_cfg->ddr_config)
		writel_relaxed(msm_host->dll_cfg->ddr_config, host->ioaddr +
			ddr_cfg_offset);
	else
		writel_relaxed(DDR_CONFIG_POR_VAL, host->ioaddr +
			ddr_cfg_offset);

	/* Step 9 - Set DLL_CONFIG_2 */
	if (msm_host->use_14lpp_dll_reset) {
		u32 mclk_freq = 0;
		int cycle_cnt = 0;
		/*
		 * Only configure the mclk_freq in normal DLL init
		 * context. If the DLL init is coming from
		 * CX Collapse Exit context, the host->clock may be zero.
		 * The DLL_CONFIG_2 register has already been restored to
		 * proper value prior to getting here.
		 */
		if (init_context == DLL_INIT_NORMAL) {
			cycle_cnt = readl_relaxed(host->ioaddr +
					msm_offset->core_dll_config_2)
					& CORE_FLL_CYCLE_CNT ? 8 : 4;

			mclk_freq = ROUND(dll_clock * cycle_cnt, TCXO_FREQ);
			if (dll_clock < 100000000)
				pr_err("%s: %s: Non standard clk freq =%lu\n",
				mmc_hostname(mmc), __func__, dll_clock);
			writel_relaxed(((readl_relaxed(host->ioaddr +
				msm_offset->core_dll_config_2)
				& ~(0xFF << 10)) | (mclk_freq << 10)),
				host->ioaddr + msm_offset->core_dll_config_2);
		}

		if (msm_host->dll_lock_bist_fail_wa &&
			(curr_ios.timing == MMC_TIMING_UHS_SDR104 ||
				!mmc_card_is_removable(mmc))) {
			writel_relaxed((readl_relaxed(host->ioaddr +
				msm_offset->core_dll_config_2)
				| CORE_LOW_FREQ_MODE), host->ioaddr +
				msm_offset->core_dll_config_2);
		}
		/* wait for 5us before enabling DLL clock */
		udelay(5);
	}

	/*
	 * Step 10 - Update the lower two bytes of DLL_CONFIG only with
	 * configuration values. Since these are the static settings.
	 */
	if (msm_host->dll_cfg) {
		writel_relaxed((readl_relaxed(host->ioaddr +
			msm_offset->core_dll_config) |
			(msm_host->dll_cfg->dll_config & 0xffff)),
			host->ioaddr + msm_offset->core_dll_config);
	}

	/*
	 * Configure DLL user control register to enable DLL status.
	 * This setting is applicable to SDCC v5.1 onwards only.
	 */
	if (msm_host->uses_tassadar_dll) {
		u32 config;
		config = DLL_USR_CTL_POR_VAL | FINE_TUNE_MODE_EN |
			ENABLE_DLL_LOCK_STATUS | BIAS_OK_SIGNAL;
		writel_relaxed(config, host->ioaddr +
				msm_offset->core_dll_usr_ctl);

		config = readl_relaxed(host->ioaddr +
				msm_offset->core_dll_config_3);
		config &= ~0xFF;
		if (msm_host->clk_rate < 150000000)
			config |= DLL_CONFIG_3_LOW_FREQ_VAL;
		else
			config |= DLL_CONFIG_3_HIGH_FREQ_VAL;
		writel_relaxed(config, host->ioaddr +
			msm_offset->core_dll_config_3);
	}

	/* Step 11 - Wait for 52us */
	spin_unlock_irqrestore(&host->lock, flags);
	usleep_range(55, 60);
	spin_lock_irqsave(&host->lock, flags);

	/*
	 * Step12 - Write 0 to DLL_RST bit of DLL_CONFIG register
	 * and Write 0 to DLL_PDN bit of DLL_CONFIG register.
	 */
	writel_relaxed((readl_relaxed(host->ioaddr +
		msm_offset->core_dll_config) & ~CORE_DLL_RST),
		host->ioaddr + msm_offset->core_dll_config);

	writel_relaxed((readl_relaxed(host->ioaddr +
		msm_offset->core_dll_config) & ~CORE_DLL_PDN),
		host->ioaddr + msm_offset->core_dll_config);

	/* Step 13 - Write 1 to DLL_RST bit of DLL_CONFIG register */
	writel_relaxed((readl_relaxed(host->ioaddr +
		msm_offset->core_dll_config) | CORE_DLL_RST),
		host->ioaddr + msm_offset->core_dll_config);

	/* Step 14 - Write 0 to DLL_RST bit of DLL_CONFIG register */
	writel_relaxed((readl_relaxed(host->ioaddr +
		msm_offset->core_dll_config) & ~CORE_DLL_RST),
		host->ioaddr + msm_offset->core_dll_config);

	/* Step 15 - Set CORE_DLL_CLOCK_DISABLE to 0 */
	if (msm_host->use_14lpp_dll_reset) {
		writel_relaxed((readl_relaxed(host->ioaddr +
				msm_offset->core_dll_config_2)
				& ~CORE_DLL_CLOCK_DISABLE), host->ioaddr +
				msm_offset->core_dll_config_2);
	}

	/* Step 16 - Set DLL_EN bit to 1. */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_offset->core_dll_config) | CORE_DLL_EN),
			host->ioaddr + msm_offset->core_dll_config);

	/*
	 * Step 17 - Wait for 8000 input clock. Here we calculate the
	 * delay from fixed clock freq 192MHz, which turns out 42us.
	 */
	spin_unlock_irqrestore(&host->lock, flags);
	usleep_range(45, 50);
	spin_lock_irqsave(&host->lock, flags);

	/* Step 18 - Set CK_OUT_EN bit to 1. */
	writel_relaxed((readl_relaxed(host->ioaddr +
			msm_offset->core_dll_config)
			| CORE_CK_OUT_EN), host->ioaddr +
			msm_offset->core_dll_config);

	/*
	 * Step 19 - Wait until DLL_LOCK bit of DLL_STATUS register
	 * becomes '1'.
	 */
	while (!(readl_relaxed(host->ioaddr + msm_offset->core_dll_status) &
		 CORE_DLL_LOCK)) {
		/* max. wait for 50us sec for LOCK bit to be set */
		if (--wait_cnt == 0) {
			dev_err(mmc_dev(mmc), "%s: DLL failed to LOCK\n",
			       mmc_hostname(mmc));
			rc = -ETIMEDOUT;
			goto out;
		}
		/* wait for 1us before polling again */
		udelay(1);
	}

out:
	if (core_vendor_spec & CORE_CLK_PWRSAVE) {
		/* Step 20 - Reenable PWRSAVE as needed */
		writel_relaxed((readl_relaxed(host->ioaddr +
			msm_offset->core_vendor_spec)
			| CORE_CLK_PWRSAVE), host->ioaddr +
			msm_offset->core_vendor_spec);
	}

	spin_unlock_irqrestore(&host->lock, flags);
	return rc;
}

static void msm_hc_select_default(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	u32 config;
	const struct sdhci_msm_offset *msm_offset =
					msm_host->offset;

	if (!msm_host->use_cdclp533) {
		config = readl_relaxed(host->ioaddr +
				msm_offset->core_vendor_spec3);
		config &= ~CORE_PWRSAVE_DLL;
		writel_relaxed(config, host->ioaddr +
				msm_offset->core_vendor_spec3);
	}

	config = readl_relaxed(host->ioaddr + msm_offset->core_vendor_spec);
	config &= ~CORE_HC_MCLK_SEL_MASK;
	config |= CORE_HC_MCLK_SEL_DFLT;
	writel_relaxed(config, host->ioaddr + msm_offset->core_vendor_spec);

	/*
	 * Disable HC_SELECT_IN to be able to use the UHS mode select
	 * configuration from Host Control2 register for all other
	 * modes.
	 * Write 0 to HC_SELECT_IN and HC_SELECT_IN_EN field
	 * in VENDOR_SPEC_FUNC
	 */
	config = readl_relaxed(host->ioaddr + msm_offset->core_vendor_spec);
	config &= ~CORE_HC_SELECT_IN_EN;
	config &= ~CORE_HC_SELECT_IN_MASK;
	writel_relaxed(config, host->ioaddr + msm_offset->core_vendor_spec);

	/*
	 * Make sure above writes impacting free running MCLK are completed
	 * before changing the clk_rate at GCC.
	 */
	wmb();
}

/*
 * After MCLK ugating, toggle the FIFO write clock to get
 * the FIFO pointers and flags to valid state.
 */
static void sdhci_msm_toggle_fifo_write_clk(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_host_offset = msm_host->offset;
	struct mmc_ios ios = host->mmc->ios;
	u32 config;

	if ((msm_host->tuning_done || ios.enhanced_strobe) &&
			(host->mmc->ios.timing == MMC_TIMING_MMC_HS400)) {
		/*
		 * set HC_REG_DLL_CONFIG_3[1] to select MCLK as
		 * DLL input clock
		 */
		config = readl_relaxed(host->ioaddr + msm_host_offset->core_dll_config_3);
		config |= RCLK_TOGGLE;
		writel_relaxed(config, host->ioaddr + msm_host_offset->core_dll_config_3);
		/* ensure above write as toggling same bit quickly */
		wmb();
		udelay(2);
		/*
		 * clear HC_REG_DLL_CONFIG_3[1] to select RCLK as
		 * DLL input clock
		 */
		config = readl_relaxed(host->ioaddr + msm_host_offset->core_dll_config_3);
		config &= ~RCLK_TOGGLE;
		writel_relaxed(config, host->ioaddr + msm_host_offset->core_dll_config_3);
	}
}

static void msm_hc_select_hs400(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct mmc_ios ios = host->mmc->ios;
	u32 config, dll_lock;
	int rc;
	const struct sdhci_msm_offset *msm_offset =
					msm_host->offset;

	/* Select the divided clock (free running MCLK/2) */
	config = readl_relaxed(host->ioaddr + msm_offset->core_vendor_spec);
	config &= ~CORE_HC_MCLK_SEL_MASK;
	config |= CORE_HC_MCLK_SEL_HS400;

	writel_relaxed(config, host->ioaddr + msm_offset->core_vendor_spec);
	/*
	 * Select HS400 mode using the HC_SELECT_IN from VENDOR SPEC
	 * register
	 */
	if ((msm_host->tuning_done || ios.enhanced_strobe) &&
	    !msm_host->calibration_done) {
		config = readl_relaxed(host->ioaddr +
				msm_offset->core_vendor_spec);
		config |= CORE_HC_SELECT_IN_HS400;
		config |= CORE_HC_SELECT_IN_EN;
		writel_relaxed(config, host->ioaddr +
				msm_offset->core_vendor_spec);
	}
	if (!msm_host->clk_rate && !msm_host->use_cdclp533) {
		/*
		 * Poll on DLL_LOCK or DDR_DLL_LOCK bits in
		 * core_dll_status to be set. This should get set
		 * within 15 us at 200 MHz.
		 */
		rc = readl_relaxed_poll_timeout(host->ioaddr +
						msm_offset->core_dll_status,
						dll_lock,
						(dll_lock &
						(CORE_DLL_LOCK |
						CORE_DDR_DLL_LOCK)), 10,
						1000);
		if (rc == -ETIMEDOUT)
			pr_err("%s: Unable to get DLL_LOCK/DDR_DLL_LOCK, dll_status: 0x%08x\n",
			       mmc_hostname(host->mmc), dll_lock);
	}
	/*
	 * Make sure above writes impacting free running MCLK are completed
	 * before changing the clk_rate at GCC.
	 */
	wmb();
}

/*
 * sdhci_msm_hc_select_mode :- In general all timing modes are
 * controlled via UHS mode select in Host Control2 register.
 * eMMC specific HS200/HS400 doesn't have their respective modes
 * defined here, hence we use these values.
 *
 * HS200 - SDR104 (Since they both are equivalent in functionality)
 * HS400 - This involves multiple configurations
 *		Initially SDR104 - when tuning is required as HS200
 *		Then when switching to DDR @ 400MHz (HS400) we use
 *		the vendor specific HC_SELECT_IN to control the mode.
 *
 * In addition to controlling the modes we also need to select the
 * correct input clock for DLL depending on the mode.
 *
 * HS400 - divided clock (free running MCLK/2)
 * All other modes - default (free running MCLK)
 */
static void sdhci_msm_hc_select_mode(struct sdhci_host *host)
{
	struct mmc_ios ios = host->mmc->ios;

	if (ios.timing == MMC_TIMING_MMC_HS400 ||
	    host->flags & SDHCI_HS400_TUNING)
		msm_hc_select_hs400(host);
	else
		msm_hc_select_default(host);
}

static int sdhci_msm_cdclp533_calibration(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	u32 config, calib_done;
	int ret;
	const struct sdhci_msm_offset *msm_offset =
					msm_host->offset;

	pr_debug("%s: %s: Enter\n", mmc_hostname(host->mmc), __func__);

	/*
	 * Retuning in HS400 (DDR mode) will fail, just reset the
	 * tuning block and restore the saved tuning phase.
	 */
	ret = msm_init_cm_dll(host, DLL_INIT_NORMAL);
	if (ret)
		goto out;

	/* Set the selected phase in delay line hw block */
	ret = msm_config_cm_dll_phase(host, msm_host->saved_tuning_phase);
	if (ret)
		goto out;

	config = readl_relaxed(host->ioaddr + msm_offset->core_dll_config);
	config |= CORE_CMD_DAT_TRACK_SEL;
	writel_relaxed(config, host->ioaddr + msm_offset->core_dll_config);

	config = readl_relaxed(host->ioaddr + msm_offset->core_ddr_200_cfg);
	config &= ~CORE_CDC_T4_DLY_SEL;
	writel_relaxed(config, host->ioaddr + msm_offset->core_ddr_200_cfg);

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_GEN_CFG);
	config &= ~CORE_CDC_SWITCH_BYPASS_OFF;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_GEN_CFG);

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_GEN_CFG);
	config |= CORE_CDC_SWITCH_RC_EN;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_GEN_CFG);

	config = readl_relaxed(host->ioaddr + msm_offset->core_ddr_200_cfg);
	config &= ~CORE_START_CDC_TRAFFIC;
	writel_relaxed(config, host->ioaddr + msm_offset->core_ddr_200_cfg);

	/* Perform CDC Register Initialization Sequence */

	writel_relaxed(0x11800EC, host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);
	writel_relaxed(0x3011111, host->ioaddr + CORE_CSR_CDC_CTLR_CFG1);
	writel_relaxed(0x1201000, host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG0);
	writel_relaxed(0x4, host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG1);
	writel_relaxed(0xCB732020, host->ioaddr + CORE_CSR_CDC_REFCOUNT_CFG);
	writel_relaxed(0xB19, host->ioaddr + CORE_CSR_CDC_COARSE_CAL_CFG);
	writel_relaxed(0x4E2, host->ioaddr + CORE_CSR_CDC_DELAY_CFG);
	writel_relaxed(0x0, host->ioaddr + CORE_CDC_OFFSET_CFG);
	writel_relaxed(0x16334, host->ioaddr + CORE_CDC_SLAVE_DDA_CFG);

	/* CDC HW Calibration */

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);
	config |= CORE_SW_TRIG_FULL_CALIB;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);
	config &= ~CORE_SW_TRIG_FULL_CALIB;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);
	config |= CORE_HW_AUTOCAL_ENA;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_CTLR_CFG0);

	config = readl_relaxed(host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG0);
	config |= CORE_TIMER_ENA;
	writel_relaxed(config, host->ioaddr + CORE_CSR_CDC_CAL_TIMER_CFG0);

	ret = readl_relaxed_poll_timeout(host->ioaddr + CORE_CSR_CDC_STATUS0,
					 calib_done,
					 (calib_done & CORE_CALIBRATION_DONE),
					 1, 50);

	if (ret == -ETIMEDOUT) {
		pr_err("%s: %s: CDC calibration was not completed\n",
		       mmc_hostname(host->mmc), __func__);
		goto out;
	}

	ret = readl_relaxed(host->ioaddr + CORE_CSR_CDC_STATUS0)
			& CORE_CDC_ERROR_CODE_MASK;
	if (ret) {
		pr_err("%s: %s: CDC error code %d\n",
		       mmc_hostname(host->mmc), __func__, ret);
		ret = -EINVAL;
		goto out;
	}

	config = readl_relaxed(host->ioaddr + msm_offset->core_ddr_200_cfg);
	config |= CORE_START_CDC_TRAFFIC;
	writel_relaxed(config, host->ioaddr + msm_offset->core_ddr_200_cfg);
out:
	pr_debug("%s: %s: Exit, ret %d\n", mmc_hostname(host->mmc),
		 __func__, ret);
	return ret;
}

static int sdhci_msm_cm_dll_sdc4_calibration(struct sdhci_host *host)
{
	struct mmc_host *mmc = host->mmc;
	u32 dll_status, config, ddr_cfg_offset;
	int ret;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_offset =
					sdhci_priv_msm_offset(host);

	pr_debug("%s: %s: Enter\n", mmc_hostname(host->mmc), __func__);

	/*
	 * Currently the core_ddr_config register defaults to desired
	 * configuration on reset. Currently reprogramming the power on
	 * reset (POR) value in case it might have been modified by
	 * bootloaders. In the future, if this changes, then the desired
	 * values will need to be programmed appropriately.
	 */
	if (msm_host->updated_ddr_cfg)
		ddr_cfg_offset = msm_offset->core_ddr_config;
	else
		ddr_cfg_offset = msm_offset->core_ddr_config_old;

	if (msm_host->dll_cfg && msm_host->dll_cfg->ddr_config)
		config = msm_host->dll_cfg->ddr_config;
	else
		config = DDR_CONFIG_POR_VAL;

	writel_relaxed(config, host->ioaddr + ddr_cfg_offset);

	if (mmc->ios.enhanced_strobe) {
		config = readl_relaxed(host->ioaddr +
				msm_offset->core_ddr_200_cfg);
		config |= CORE_CMDIN_RCLK_EN;
		writel_relaxed(config, host->ioaddr +
				msm_offset->core_ddr_200_cfg);
	}

	config = readl_relaxed(host->ioaddr + msm_offset->core_dll_config_2);
	config |= CORE_DDR_CAL_EN;
	writel_relaxed(config, host->ioaddr + msm_offset->core_dll_config_2);

	ret = readl_relaxed_poll_timeout(host->ioaddr +
					msm_offset->core_dll_status,
					dll_status,
					(dll_status & CORE_DDR_DLL_LOCK),
					10, 1000);

	if (ret == -ETIMEDOUT) {
		pr_err("%s: %s: CM_DLL_SDC4 calibration was not completed\n",
		       mmc_hostname(host->mmc), __func__);
		goto out;
	}

	/*
	 * Set CORE_PWRSAVE_DLL bit in CORE_VENDOR_SPEC3.
	 * When MCLK is gated OFF, it is not gated for less than 0.5us
	 * and MCLK must be switched on for at-least 1us before DATA
	 * starts coming. Controllers with 14lpp and later tech DLL cannot
	 * guarantee above requirement. So PWRSAVE_DLL should not be
	 * turned on for host controllers using this DLL.
	 */
	if (!msm_host->use_14lpp_dll_reset) {
		config = readl_relaxed(host->ioaddr +
				msm_offset->core_vendor_spec3);
		config |= CORE_PWRSAVE_DLL;
		writel_relaxed(config, host->ioaddr +
				msm_offset->core_vendor_spec3);
	}

	/*
	 * Drain writebuffer to ensure above DLL calibration
	 * and PWRSAVE DLL is enabled.
	 */
	wmb();
out:
	pr_debug("%s: %s: Exit, ret %d\n", mmc_hostname(host->mmc),
		 __func__, ret);
	return ret;
}

static int sdhci_msm_hs400_dll_calibration(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	int ret;
	struct mmc_host *mmc = host->mmc;
	u32 config;
	const struct sdhci_msm_offset *msm_offset =
					msm_host->offset;

	pr_debug("%s: %s: Enter\n", mmc_hostname(host->mmc), __func__);

	/*
	 * Retuning in HS400 (DDR mode) will fail, just reset the
	 * tuning block and restore the saved tuning phase.
	 */
	ret = msm_init_cm_dll(host, DLL_INIT_NORMAL);
	if (ret)
		goto out;

	if (!mmc->ios.enhanced_strobe) {
		/* set the selected phase in delay line hw block */
		ret = msm_config_cm_dll_phase(host,
				      msm_host->saved_tuning_phase);
		if (ret)
			goto out;
		config = readl_relaxed(host->ioaddr +
				msm_offset->core_dll_config);
		config |= CORE_CMD_DAT_TRACK_SEL;
		writel_relaxed(config, host->ioaddr +
				msm_offset->core_dll_config);
	}

	if (msm_host->use_cdclp533)
		ret = sdhci_msm_cdclp533_calibration(host);
	else
		ret = sdhci_msm_cm_dll_sdc4_calibration(host);
out:
	pr_debug("%s: %s: Exit, ret %d\n", mmc_hostname(host->mmc),
		 __func__, ret);
	return ret;
}

static bool sdhci_msm_is_tuning_needed(struct sdhci_host *host)
{
	struct mmc_ios *ios = &host->mmc->ios;

	if (ios->timing == MMC_TIMING_UHS_SDR50 &&
			host->flags & SDHCI_SDR50_NEEDS_TUNING)
		return true;

	/*
	 * Tuning is required for SDR104, HS200 and HS400 cards and
	 * if clock frequency is greater than 100MHz in these modes.
	 */
	if (host->clock <= CORE_FREQ_100MHZ ||
	    !(ios->timing == MMC_TIMING_MMC_HS400 ||
	    ios->timing == MMC_TIMING_MMC_HS200 ||
	    ios->timing == MMC_TIMING_UHS_SDR104) ||
	    ios->enhanced_strobe)
		return false;

	return true;
}

static int sdhci_msm_restore_sdr_dll_config(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	int ret;

	/*
	 * SDR DLL comes into picture only for timing modes which needs
	 * tuning.
	 */
	if (!sdhci_msm_is_tuning_needed(host))
		return 0;

	/* Reset the tuning block */
	ret = msm_init_cm_dll(host, DLL_INIT_NORMAL);
	if (ret)
		return ret;

	/* Restore the tuning block */
	ret = msm_config_cm_dll_phase(host, msm_host->saved_tuning_phase);

	return ret;
}

static void sdhci_msm_set_cdr(struct sdhci_host *host, bool enable)
{
	const struct sdhci_msm_offset *msm_offset = sdhci_priv_msm_offset(host);
	u32 config, oldconfig = readl_relaxed(host->ioaddr +
					      msm_offset->core_dll_config);

	config = oldconfig;
	if (enable) {
		config |= CORE_CDR_EN;
		config &= ~CORE_CDR_EXT_EN;
	} else {
		config &= ~CORE_CDR_EN;
		config |= CORE_CDR_EXT_EN;
	}

	if (config != oldconfig) {
		writel_relaxed(config, host->ioaddr +
			       msm_offset->core_dll_config);
	}
}

static int sdhci_msm_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct sdhci_host *host = mmc_priv(mmc);
	int tuning_seq_cnt = 10;
	u8 phase, tuned_phases[16], tuned_phase_cnt = 0;
	int rc = 0;
	u32 config;
	struct mmc_ios ios = host->mmc->ios;
	u32 core_vendor_spec;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_offset =
					sdhci_priv_msm_offset(host);

	if (!sdhci_msm_is_tuning_needed(host)) {
		msm_host->use_cdr = false;
		sdhci_msm_set_cdr(host, false);
		return 0;
	}

	/* Clock-Data-Recovery used to dynamically adjust RX sampling point */
	msm_host->use_cdr = true;

	/*
	 * Clear tuning_done flag before tuning to ensure proper
	 * HS400 settings.
	 */
	msm_host->tuning_done = 0;

	if (ios.timing == MMC_TIMING_UHS_SDR50 &&
			host->flags & SDHCI_SDR50_NEEDS_TUNING) {
		/*
		 * Bit1 SDHCI_CTRL_UHS_SDR50 of the Host Control 2 register is
		 * already set by the sdhci_set_ios -> sdhci_msm_set_uhs_signaling().
		 * It is not necessary to set it again here.
		 */

		config = readl_relaxed(host->ioaddr + msm_offset->core_vendor_spec);
		config |= CORE_HC_SELECT_IN_EN;
		config &= ~CORE_HC_SELECT_IN_MASK;
		config |= CORE_HC_SELECT_IN_SDR50;
		writel_relaxed(config, host->ioaddr + msm_offset->core_vendor_spec);
	}

	/*
	 * For HS400 tuning in HS200 timing requires:
	 * - select MCLK/2 in VENDOR_SPEC
	 * - program MCLK to 400MHz (or nearest supported) in GCC
	 */
	if (host->flags & SDHCI_HS400_TUNING) {
		sdhci_msm_hc_select_mode(host);
		msm_set_clock_rate_for_bus_mode(host, ios.clock);
		host->flags &= ~SDHCI_HS400_TUNING;
	}

	core_vendor_spec = readl_relaxed(host->ioaddr +
			msm_offset->core_vendor_spec);

	/* Make sure that PWRSAVE bit is set to '0' during tuning */
	writel_relaxed((core_vendor_spec & ~CORE_CLK_PWRSAVE),
			host->ioaddr +
			msm_offset->core_vendor_spec);

retry:
	/* First of all reset the tuning block */
	rc = msm_init_cm_dll(host, DLL_INIT_NORMAL);
	if (rc)
		goto out;

	phase = 0;
	do {
		/* Set the phase in delay line hw block */
		rc = msm_config_cm_dll_phase(host, phase);
		if (rc)
			goto out;

		rc = mmc_send_tuning(mmc, opcode, NULL);
		if (!rc) {
			/* Tuning is successful at this tuning point */
			tuned_phases[tuned_phase_cnt++] = phase;
			dev_dbg(mmc_dev(mmc), "%s: Found good phase = %d\n",
				 mmc_hostname(mmc), phase);
		}
	} while (++phase < ARRAY_SIZE(tuned_phases));

	if (tuned_phase_cnt) {
		if (tuned_phase_cnt == ARRAY_SIZE(tuned_phases)) {
			/*
			 * All phases valid is _almost_ as bad as no phases
			 * valid.  Probably all phases are not really reliable
			 * but we didn't detect where the unreliable place is.
			 * That means we'll essentially be guessing and hoping
			 * we get a good phase.  Better to try a few times.
			 */
			dev_dbg(mmc_dev(mmc), "%s: All phases valid; try again\n",
				mmc_hostname(mmc));
			if (--tuning_seq_cnt) {
				tuned_phase_cnt = 0;
				goto retry;
			}
		}

		rc = msm_find_most_appropriate_phase(host, tuned_phases,
						     tuned_phase_cnt);
		if (rc < 0)
			goto out;
		else
			phase = rc;

		/*
		 * Finally set the selected phase in delay
		 * line hw block.
		 */
		rc = msm_config_cm_dll_phase(host, phase);
		if (rc)
			goto out;
		msm_host->saved_tuning_phase = phase;
		dev_dbg(mmc_dev(mmc), "%s: Setting the tuning phase to %d\n",
			 mmc_hostname(mmc), phase);
		sdhci_msm_log_str(msm_host, "Setting the tuning phase to %d\n",
				phase);
	} else {
		if (--tuning_seq_cnt)
			goto retry;
		/* Tuning failed */
		dev_dbg(mmc_dev(mmc), "%s: No tuning point found\n",
		       mmc_hostname(mmc));
		sdhci_msm_log_str(msm_host, "No tuning point found\n");
		rc = -EIO;
	}

	if (!rc)
		msm_host->tuning_done = true;
out:

	/* Set PWRSAVE bit to '1' after completion of tuning as needed */
	if (core_vendor_spec & CORE_CLK_PWRSAVE) {
		writel_relaxed((readl_relaxed(host->ioaddr +
			msm_offset->core_vendor_spec)
			| CORE_CLK_PWRSAVE), host->ioaddr +
			msm_offset->core_vendor_spec);
	}

	return rc;
}

/*
 * sdhci_msm_hs400 - Calibrate the DLL for HS400 bus speed mode operation.
 * This needs to be done for both tuning and enhanced_strobe mode.
 * DLL operation is only needed for clock > 100MHz. For clock <= 100MHz
 * fixed feedback clock is used.
 */
static void sdhci_msm_hs400(struct sdhci_host *host, struct mmc_ios *ios)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	int ret;

	if (host->clock > CORE_FREQ_100MHZ &&
	    (msm_host->tuning_done || ios->enhanced_strobe) &&
	    !msm_host->calibration_done) {
		ret = sdhci_msm_hs400_dll_calibration(host);
		if (!ret)
			msm_host->calibration_done = true;
		else
			pr_err("%s: Failed to calibrate DLL for hs400 mode (%d)\n",
			       mmc_hostname(host->mmc), ret);
	}
}

static void sdhci_msm_set_uhs_signaling(struct sdhci_host *host,
					unsigned int uhs)
{
	struct mmc_host *mmc = host->mmc;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	u16 ctrl_2;
	u32 config;
	const struct sdhci_msm_offset *msm_offset =
					msm_host->offset;

	ctrl_2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	/* Select Bus Speed Mode for host */
	ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
	switch (uhs) {
	case MMC_TIMING_UHS_SDR12:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR12;
		break;
	case MMC_TIMING_UHS_SDR25:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR25;
		break;
	case MMC_TIMING_UHS_SDR50:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR50;
		break;
	case MMC_TIMING_MMC_HS400:
	case MMC_TIMING_MMC_HS200:
	case MMC_TIMING_UHS_SDR104:
		ctrl_2 |= SDHCI_CTRL_UHS_SDR104;
		break;
	case MMC_TIMING_UHS_DDR50:
	case MMC_TIMING_MMC_DDR52:
		ctrl_2 |= SDHCI_CTRL_UHS_DDR50;
		break;
	}

	/*
	 * When clock frequency is less than 100MHz, the feedback clock must be
	 * provided and DLL must not be used so that tuning can be skipped. To
	 * provide feedback clock, the mode selection can be any value less
	 * than 3'b011 in bits [2:0] of HOST CONTROL2 register.
	 */
	if (host->clock <= CORE_FREQ_100MHZ) {
		if (uhs == MMC_TIMING_MMC_HS400 ||
		    uhs == MMC_TIMING_MMC_HS200 ||
		    uhs == MMC_TIMING_UHS_SDR104)
			ctrl_2 &= ~SDHCI_CTRL_UHS_MASK;
		/*
		 * DLL is not required for clock <= 100MHz
		 * Thus, make sure DLL it is disabled when not required
		 */
		config = readl_relaxed(host->ioaddr +
				msm_offset->core_dll_config);
		config |= CORE_DLL_RST;
		writel_relaxed(config, host->ioaddr +
				msm_offset->core_dll_config);

		config = readl_relaxed(host->ioaddr +
				msm_offset->core_dll_config);
		config |= CORE_DLL_PDN;
		writel_relaxed(config, host->ioaddr +
				msm_offset->core_dll_config);

		/*
		 * The DLL needs to be restored and CDCLP533 recalibrated
		 * when the clock frequency is set back to 400MHz.
		 */
		msm_host->calibration_done = false;
	}

	dev_dbg(mmc_dev(mmc), "%s: clock=%u uhs=%u ctrl_2=0x%x\n",
		mmc_hostname(host->mmc), host->clock, uhs, ctrl_2);
	sdhci_writew(host, ctrl_2, SDHCI_HOST_CONTROL2);

	if (mmc->ios.timing == MMC_TIMING_MMC_HS400)
		sdhci_msm_hs400(host, &mmc->ios);
}

/*
 * Ensure larger discard size by always setting max_busy_timeout to zero.
 * This will always return max_busy_timeout as zero to the sdhci layer.
 */

static unsigned int sdhci_msm_get_max_timeout_count(struct sdhci_host *host)
{
	return 0;
}

#define MAX_PROP_SIZE 32
static int sdhci_msm_dt_parse_vreg_info(struct device *dev,
		struct sdhci_msm_reg_data **vreg_data, const char *vreg_name)
{
	int len, ret = 0;
	const __be32 *prop;
	char prop_name[MAX_PROP_SIZE];
	struct sdhci_msm_reg_data *vreg;
	struct device_node *np = dev->of_node;
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	snprintf(prop_name, MAX_PROP_SIZE, "%s-supply", vreg_name);
	if (!of_parse_phandle(np, prop_name, 0)) {
		dev_info(dev, "No vreg data found for %s\n", vreg_name);
		return ret;
	}

	vreg = devm_kzalloc(dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg) {
		ret = -ENOMEM;
		return ret;
	}

	vreg->msm_host = msm_host;
	vreg->name = vreg_name;

	/*
	 * To support chipsets where PMIC doesn't support
	 * PBS ram sequence to turn OFF regulators automatically on
	 * multicard tray removal, parse and use new regulator resources
	 * which are exposed by PMIC team for enabling/disabling
	 * conditions only.
	 */
	if (mmc_card_is_removable(host->mmc) &&
			!vreg->multi_card_tray_wa_needed) {
		snprintf(prop_name, MAX_PROP_SIZE, "%s-en-dis", vreg_name);
		strscpy(vreg->en_dis_name, prop_name, sizeof(vreg->en_dis_name));

		snprintf(prop_name, MAX_PROP_SIZE, "%s-supply", vreg->en_dis_name);
		if (of_parse_phandle(np, prop_name, 0)) {
			dev_dbg(dev, "Multi card tray WA needed\n");
			vreg->multi_card_tray_wa_needed = true;
		}
	}

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-always-on", vreg_name);
	if (of_get_property(np, prop_name, NULL))
		vreg->is_always_on = true;

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-lpm-sup", vreg_name);
	if (of_get_property(np, prop_name, NULL))
		vreg->lpm_sup = true;

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-voltage-level", vreg_name);
	prop = of_get_property(np, prop_name, &len);
	if (!prop || (len != (2 * sizeof(__be32)))) {
		vreg->is_voltage_supplied = false;
		dev_warn(dev, "%s %s property\n",
			prop ? "invalid format" : "no", prop_name);
	} else {
		vreg->is_voltage_supplied = true;
		vreg->low_vol_level = be32_to_cpup(&prop[0]);
		vreg->high_vol_level = be32_to_cpup(&prop[1]);
	}

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-current-level", vreg_name);
	prop = of_get_property(np, prop_name, &len);
	if (!prop || (len != (2 * sizeof(__be32)))) {
		dev_warn(dev, "%s %s property\n",
			prop ? "invalid format" : "no", prop_name);
	} else {
		vreg->lpm_uA = be32_to_cpup(&prop[0]);
		vreg->hpm_uA = be32_to_cpup(&prop[1]);
	}

	*vreg_data = vreg;
	dev_dbg(dev, "%s: %s %s vol=[%d %d]uV, curr=[%d %d]uA\n",
		vreg->name, vreg->is_always_on ? "always_on," : "",
		vreg->lpm_sup ? "lpm_sup," : "", vreg->low_vol_level,
		vreg->high_vol_level, vreg->lpm_uA, vreg->hpm_uA);

	return ret;
}

static int sdhci_msm_set_pincfg(struct sdhci_msm_host *msm_host, bool level)
{
	struct platform_device *pdev = msm_host->pdev;
	int ret;

	if (level)
		ret = pinctrl_pm_select_default_state(&pdev->dev);
	else
		ret = pinctrl_pm_select_sleep_state(&pdev->dev);

	return ret;
}

static int sdhci_msm_dt_parse_dll_cfg_info(struct device *dev,
		struct sdhci_msm_host *msm_host)

{
	u32 *dll_cfg_table = NULL;
	int dll_cfg_table_len, dll_cfg_reg_count;
	int ret = 0;

	/* Try to parse "qcom,dll-config-list" first */
	if (sdhci_msm_dt_get_array(dev, "qcom,dll-config-list",
			&dll_cfg_table, &dll_cfg_table_len, 0)) {
		/* Try legacy property name for backward compatibility */
		if (sdhci_msm_dt_get_array(dev, "qcom,dll-hsr-list",
				&dll_cfg_table, &dll_cfg_table_len, 0))
			goto skip_dll_cfg;
	}

	dll_cfg_reg_count = sizeof(struct sdhci_msm_dll_cfg) / sizeof(u32);
	if (dll_cfg_table_len != dll_cfg_reg_count) {
		dev_err(dev, "Number of DLL config entries are not matching\n");
		ret = -EINVAL;
	} else {
		msm_host->dll_cfg = (struct sdhci_msm_dll_cfg *)dll_cfg_table;
	}

skip_dll_cfg:
	if (!msm_host->dll_cfg)
		dev_info(dev, "Failed to get dll config settings from dt\n");
	return ret;
}

static int sdhci_msm_parse_reset_data(struct device *dev,
			struct sdhci_msm_host *msm_host)
{
	int ret = 0;

	msm_host->core_reset = devm_reset_control_get(dev,
					"core_reset");
	if (IS_ERR(msm_host->core_reset)) {
		ret = PTR_ERR(msm_host->core_reset);
		dev_err(dev, "core_reset unavailable,err = %d\n",
				ret);
		msm_host->core_reset = NULL;
	}

	return ret;
}

/* Parse platform data */
static bool sdhci_msm_populate_pdata(struct device *dev,
						struct sdhci_msm_host *msm_host)
{
	struct device_node *np = dev->of_node;
	int ice_clk_table_len;
	u32 *ice_clk_table = NULL;

	msm_host->vreg_data = devm_kzalloc(dev, sizeof(struct
						    sdhci_msm_vreg_data),
					GFP_KERNEL);
	if (!msm_host->vreg_data) {
		dev_err(dev, "failed to allocate memory for vreg data\n");
		goto out;
	}

	if (sdhci_msm_dt_parse_vreg_info(dev, &msm_host->vreg_data->vdd_data,
					 "vdd")) {
		dev_err(dev, "failed parsing vdd data\n");
		goto out;
	}
	if (sdhci_msm_dt_parse_vreg_info(dev,
					 &msm_host->vreg_data->vdd_io_data,
					 "vdd-io")) {
		dev_err(dev, "failed parsing vdd-io data\n");
		goto out;
	}

	if (of_get_property(np, "qcom,core_3_0v_support", NULL))
		msm_host->fake_core_3_0v_support = true;

	msm_host->regs_restore.is_supported =
		of_property_read_bool(np, "qcom,restore-after-cx-collapse");

	msm_host->uses_level_shifter =
		of_property_read_bool(np, "qcom,uses_level_shifter");

	if (msm_host->uses_level_shifter)
		msm_host->enable_ext_fb_clk =
			of_property_read_bool(np, "qcom,external-fb-clk");

	msm_host->dll_lock_bist_fail_wa =
		of_property_read_bool(np, "qcom,dll_lock_bist_fail_wa");

	if (sdhci_msm_dt_parse_dll_cfg_info(dev, msm_host))
		goto out;

	if (!sdhci_msm_dt_get_array(dev, "qcom,ice-clk-rates",
			&ice_clk_table, &ice_clk_table_len, 0)) {
		if (ice_clk_table && ice_clk_table_len) {
			if (ice_clk_table_len != 2) {
				dev_err(dev, "Need max and min frequencies\n");
				goto out;
			}
			msm_host->sup_ice_clk_table = ice_clk_table;
			msm_host->sup_ice_clk_cnt = ice_clk_table_len;
			msm_host->ice_clk_max = msm_host->sup_ice_clk_table[0];
			msm_host->ice_clk_min = msm_host->sup_ice_clk_table[1];
			dev_dbg(dev, "ICE clock rates (Hz): max: %u min: %u\n",
				msm_host->ice_clk_max, msm_host->ice_clk_min);
		}
	}

	msm_host->vbias_skip_wa =
		of_property_read_bool(np, "qcom,vbias-skip-wa");

	sdhci_msm_parse_reset_data(dev, msm_host);

	return false;
out:
	return true;
}

static int sdhci_msm_set_vmmc(struct mmc_host *mmc)
{
	if (IS_ERR(mmc->supply.vmmc))
		return 0;

	return mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, mmc->ios.vdd);
}

static int msm_toggle_vqmmc(struct sdhci_msm_host *msm_host,
			      struct mmc_host *mmc, bool level)
{
	int ret;
	struct mmc_ios ios;

	if (msm_host->vqmmc_enabled == level)
		return 0;

	if (level) {
		/* Set the IO voltage regulator to default voltage level */
		if (msm_host->caps_0 & CORE_3_0V_SUPPORT)
			ios.signal_voltage = MMC_SIGNAL_VOLTAGE_330;
		else if (msm_host->caps_0 & CORE_1_8V_SUPPORT)
			ios.signal_voltage = MMC_SIGNAL_VOLTAGE_180;

		if (msm_host->caps_0 & CORE_VOLT_SUPPORT) {
			ret = mmc_regulator_set_vqmmc(mmc, &ios);
			if (ret < 0) {
				dev_err(mmc_dev(mmc), "%s: vqmmc set volgate failed: %d\n",
					mmc_hostname(mmc), ret);
				goto out;
			}
		}
		ret = regulator_enable(mmc->supply.vqmmc);
	} else {
		ret = regulator_disable(mmc->supply.vqmmc);
	}

	if (ret)
		dev_err(mmc_dev(mmc), "%s: vqmm %sable failed: %d\n",
			mmc_hostname(mmc), level ? "en":"dis", ret);
	else
		msm_host->vqmmc_enabled = level;
out:
	return ret;
}

static int msm_config_vqmmc_mode(struct sdhci_msm_host *msm_host,
			      struct mmc_host *mmc, bool hpm)
{
	int load, ret;

	load = hpm ? MMC_VQMMC_MAX_LOAD_UA : 0;
	ret = regulator_set_load(mmc->supply.vqmmc, load);
	if (ret)
		dev_err(mmc_dev(mmc), "%s: vqmmc set load failed: %d\n",
			mmc_hostname(mmc), ret);
	return ret;
}

static int sdhci_msm_set_vqmmc(struct sdhci_msm_host *msm_host,
			      struct mmc_host *mmc, bool level)
{
	int ret;
	bool always_on;

	if (IS_ERR(mmc->supply.vqmmc) ||
			(mmc->ios.power_mode == MMC_POWER_UNDEFINED))
		return 0;
	/*
	 * For eMMC don't turn off Vqmmc, Instead just configure it in LPM
	 * and HPM modes by setting the corresponding load.
	 *
	 * Till eMMC is initialized (i.e. always_on == 0), just turn on/off
	 * Vqmmc. Vqmmc gets turned off only if init fails and mmc_power_off
	 * gets invoked. Once eMMC is initialized (i.e. always_on == 1),
	 * Vqmmc should remain ON, So just set the load instead of turning it
	 * off/on.
	 */
	always_on = !mmc_card_is_removable(mmc) &&
			mmc->card && mmc_card_mmc(mmc->card);

	if (always_on)
		ret = msm_config_vqmmc_mode(msm_host, mmc, level);
	else
		ret = msm_toggle_vqmmc(msm_host, mmc, level);

	return ret;
}

static inline void sdhci_msm_init_pwr_irq_wait(struct sdhci_msm_host *msm_host)
{
	init_waitqueue_head(&msm_host->pwr_irq_wait);
}

static inline void sdhci_msm_complete_pwr_irq_wait(
		struct sdhci_msm_host *msm_host)
{
	wake_up(&msm_host->pwr_irq_wait);
}

/*
 * sdhci_msm_check_power_status API should be called when registers writes
 * which can toggle sdhci IO bus ON/OFF or change IO lines HIGH/LOW happens.
 * To what state the register writes will change the IO lines should be passed
 * as the argument req_type. This API will check whether the IO line's state
 * is already the expected state and will wait for power irq only if
 * power irq is expected to be triggered based on the current IO line state
 * and expected IO line state.
 */
static void sdhci_msm_check_power_status(struct sdhci_host *host, u32 req_type)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	bool done = false;
	u32 val = SWITCHABLE_SIGNALING_VOLTAGE;
	const struct sdhci_msm_offset *msm_offset =
					msm_host->offset;
	struct mmc_host *mmc = host->mmc;

	pr_debug("%s: %s: request %d curr_pwr_state %x curr_io_level %x\n",
			mmc_hostname(host->mmc), __func__, req_type,
			msm_host->curr_pwr_state, msm_host->curr_io_level);

	sdhci_msm_log_str(msm_host, "request %d curr_pwr_state %x curr_io_level %x\n",
				req_type, msm_host->curr_pwr_state,
				msm_host->curr_io_level);
	/*
	 * The power interrupt will not be generated for signal voltage
	 * switches if SWITCHABLE_SIGNALING_VOLTAGE in MCI_GENERICS is not set.
	 * Since sdhci-msm-v5, this bit has been removed and SW must consider
	 * it as always set.
	 */
	if (!msm_host->mci_removed)
		val = msm_host_readl(msm_host, host,
				msm_offset->core_generics);
	if ((req_type & REQ_IO_HIGH || req_type & REQ_IO_LOW) &&
	    !(val & SWITCHABLE_SIGNALING_VOLTAGE)) {
		return;
	}

	/*
	 * The IRQ for request type IO High/LOW will be generated when -
	 * there is a state change in 1.8V enable bit (bit 3) of
	 * SDHCI_HOST_CONTROL2 register. The reset state of that bit is 0
	 * which indicates 3.3V IO voltage. So, when MMC core layer tries
	 * to set it to 3.3V before card detection happens, the
	 * IRQ doesn't get triggered as there is no state change in this bit.
	 * The driver already handles this case by changing the IO voltage
	 * level to high as part of controller power up sequence. Hence, check
	 * for host->pwr to handle a case where IO voltage high request is
	 * issued even before controller power up.
	 */
	if ((req_type & REQ_IO_HIGH) && !host->pwr) {
		pr_debug("%s: do not wait for power IRQ that never comes, req_type: %d\n",
				mmc_hostname(host->mmc), req_type);
		return;
	}

	if ((req_type & msm_host->curr_pwr_state) ||
			(req_type & msm_host->curr_io_level))
		done = true;
	/*
	 * This is needed here to handle cases where register writes will
	 * not change the current bus state or io level of the controller.
	 * In this case, no power irq will be triggerred and we should
	 * not wait.
	 */
	if (!done) {
		if (!wait_event_timeout(msm_host->pwr_irq_wait,
				msm_host->pwr_irq_flag,
				msecs_to_jiffies(MSM_PWR_IRQ_TIMEOUT_MS))) {
			dev_warn(&msm_host->pdev->dev,
				 "%s: pwr_irq for req: (%d) timed out\n",
				 mmc_hostname(host->mmc), req_type);
			sdhci_msm_dump_pwr_ctrl_regs(host);
		}

	}

	if (mmc->card && mmc->ops->get_cd && !mmc->ops->get_cd(mmc) &&
			(req_type & REQ_BUS_ON)) {
		host->pwr = 0;
		sdhci_writeb(host, 0, SDHCI_POWER_CONTROL);
	}

	pr_debug("%s: %s: request %d done\n", mmc_hostname(host->mmc),
			__func__, req_type);

	sdhci_msm_log_str(msm_host, "request %d done\n", req_type);
}

/*
 * sdhci_msm_enhanced_strobe_mask :-
 * Before running CMDQ transfers in HS400 Enhanced Strobe mode,
 * SW should write 3 to
 * HC_VENDOR_SPECIFIC_FUNC3.CMDEN_HS400_INPUT_MASK_CNT register.
 * The default reset value of this register is 2.
 */
static void sdhci_msm_enhanced_strobe_mask(struct mmc_host *mmc, bool set)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_offset = msm_host->offset;
	u32 config;

	if (!mmc->ios.enhanced_strobe) {
		pr_debug("%s: host/card does not support hs400 enhanced strobe\n",
					mmc_hostname(host->mmc));
		return;
	}

	config = readl_relaxed(host->ioaddr + msm_offset->core_vendor_spec3);

	if (set)
		config |= CORE_CMDEN_HS400_INPUT_MASK_CNT;
	else
		config &= ~CORE_CMDEN_HS400_INPUT_MASK_CNT;

	writel_relaxed(config, host->ioaddr + msm_offset->core_vendor_spec3);
}

static void sdhci_msm_dump_pwr_ctrl_regs(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_offset =
					msm_host->offset;

	pr_err("%s: PWRCTL_STATUS: 0x%08x | PWRCTL_MASK: 0x%08x | PWRCTL_CTL: 0x%08x\n",
		mmc_hostname(host->mmc),
		msm_host_readl(msm_host, host, msm_offset->core_pwrctl_status),
		msm_host_readl(msm_host, host, msm_offset->core_pwrctl_mask),
		msm_host_readl(msm_host, host, msm_offset->core_pwrctl_ctl));
}

static int sdhci_msm_clear_pwrctl_status(struct sdhci_host *host, u32 value)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_offset = msm_host->offset;
	int ret = 0, retry = 10;

	/*
	 * There is a rare HW scenario where the first clear pulse could be
	 * lost when actual reset and clear/read of status register is
	 * happening at a time. Hence, retry for at least 10 times to make
	 * sure status register is cleared. Otherwise, this will result in
	 * a spurious power IRQ resulting in system instability.
	 */
	do {
		if (retry == 0) {
			pr_err("%s: Timedout clearing (0x%x) pwrctl status register\n",
				mmc_hostname(host->mmc), value);
			sdhci_msm_dump_pwr_ctrl_regs(host);
			WARN_ON(1);
			ret = -EBUSY;
			break;
		}

		/*
		 * Clear the PWRCTL_STATUS interrupt bits by writing to the
		 * corresponding bits in the PWRCTL_CLEAR register.
		 */
		msm_host_writel(msm_host, value, host,
				msm_offset->core_pwrctl_clear);
		/*
		 * SDHC has core_mem and hc_mem device memory and these memory
		 * addresses do not fall within 1KB region. Hence, any update to
		 * core_mem address space would require an mb() to ensure this
		 * gets completed before its next update to registers within
		 * hc_mem.
		 */
		mb();
		retry--;
		udelay(10);
	} while (value & msm_host_readl(msm_host, host,
				msm_offset->core_pwrctl_status));

	return ret;
}

/* Regulator utility functions */
static int sdhci_msm_vreg_init_reg(struct device *dev,
					struct sdhci_msm_reg_data *vreg)
{
	int ret = 0;

	/* check if regulator is already initialized? */
	if (vreg->reg)
		goto out;

	/* Get the regulator handle */
	vreg->reg = devm_regulator_get(dev, vreg->name);
	if (IS_ERR(vreg->reg)) {
		ret = PTR_ERR(vreg->reg);
		pr_err("%s: devm_regulator_get(%s) failed. ret=%d\n",
			__func__, vreg->name, ret);
		goto out;
	}

	if (vreg->multi_card_tray_wa_needed) {
		vreg->reg_en_dis = devm_regulator_get(dev, vreg->en_dis_name);
		if (IS_ERR(vreg->reg_en_dis)) {
			ret = PTR_ERR(vreg->reg_en_dis);
			pr_err("%s: devm_regulator_get(%s) failed. ret=%d\n",
				__func__, vreg->en_dis_name, ret);
			goto out;
		}
	}

	if (regulator_count_voltages(vreg->reg) > 0) {
		vreg->set_voltage_sup = true;
		/* sanity check */
		if ((vreg->is_voltage_supplied && !vreg->high_vol_level) ||
				!vreg->hpm_uA) {
			pr_err("%s: %s invalid constraints specified\n",
			       __func__, vreg->name);
			ret = -EINVAL;
		}
	}

out:
	return ret;
}

static int sdhci_msm_vreg_set_optimum_mode(struct sdhci_msm_reg_data
						  *vreg, int uA_load)
{
	int ret = 0;

	sdhci_msm_log_str(vreg->msm_host, "reg=%s uA_load=%d\n",
			vreg->name, uA_load);
	/*
	 * regulators that do not support regulator_set_voltage also
	 * do not support regulator_set_optimum_mode
	 */
	if (vreg->set_voltage_sup) {
		ret = regulator_set_load(vreg->reg, uA_load);
		if (ret < 0)
			pr_err("%s: regulator_set_load(reg=%s,uA_load=%d) failed. ret=%d\n",
			       __func__, vreg->name, uA_load, ret);
		else
			/*
			 * regulator_set_load() can return non zero
			 * value even for success case.
			 */
			ret = 0;
	}
	return ret;
}

static int sdhci_msm_vreg_set_voltage(struct sdhci_msm_reg_data *vreg,
					int min_uV, int max_uV)
{
	int ret = 0;

	sdhci_msm_log_str(vreg->msm_host, "reg=%s min_uV=%d max_uV=%d\n",
			vreg->name, min_uV, max_uV);

	if (vreg->set_voltage_sup && vreg->is_voltage_supplied) {
		ret = regulator_set_voltage(vreg->reg, min_uV, max_uV);
		if (ret) {
			pr_err("%s: regulator_set_voltage(%s)failed. min_uV=%d,max_uV=%d,ret=%d\n",
			       __func__, vreg->name, min_uV, max_uV, ret);
		}
	}

	return ret;
}

static int sdhci_msm_vreg_enable(struct sdhci_msm_reg_data *vreg)
{
	int ret = 0;
	bool wa_needed = vreg->multi_card_tray_wa_needed;
	const char *reg_name = wa_needed ? vreg->en_dis_name : vreg->name;

	/* Put regulator in HPM (high power mode) */
	ret = sdhci_msm_vreg_set_optimum_mode(vreg, vreg->hpm_uA);
	if (ret < 0)
		return ret;

	if (!vreg->is_enabled) {
		/* Set voltage level */
		ret = sdhci_msm_vreg_set_voltage(vreg, vreg->high_vol_level,
						vreg->high_vol_level);
		if (ret)
			return ret;
	}

	if (wa_needed)
		ret = regulator_enable(vreg->reg_en_dis);
	else
		ret = regulator_enable(vreg->reg);

	if (ret) {
		pr_err("%s: regulator_enable(%s) failed. ret=%d\n",
				__func__, reg_name, ret);
		return ret;
	}
	vreg->is_enabled = true;
	return ret;
}

static int sdhci_msm_vreg_disable(struct sdhci_msm_reg_data *vreg)
{
	int ret = 0;
	bool wa_needed = vreg->multi_card_tray_wa_needed;
	const char *reg_name = wa_needed ? vreg->en_dis_name : vreg->name;

	/* Never disable regulator marked as always_on */
	if (vreg->is_enabled && !vreg->is_always_on) {

		if (wa_needed)
			ret = regulator_disable(vreg->reg_en_dis);
		else
			ret = regulator_disable(vreg->reg);
		if (ret) {
			pr_err("%s: regulator_disable(%s) failed. ret=%d\n",
				__func__, reg_name, ret);
			goto out;
		}
		vreg->is_enabled = false;

		ret = sdhci_msm_vreg_set_optimum_mode(vreg, 0);
		if (ret < 0)
			goto out;

		/* Set min. voltage level to 0 */
		ret = sdhci_msm_vreg_set_voltage(vreg, 0, vreg->high_vol_level);
		if (ret)
			goto out;
	} else if (vreg->is_enabled && vreg->is_always_on) {
		if (vreg->lpm_sup) {
			/* Put always_on regulator in LPM (low power mode) */
			ret = sdhci_msm_vreg_set_optimum_mode(vreg,
							      vreg->lpm_uA);
			if (ret < 0)
				goto out;
		}
	}
out:
	return ret;
}

static int sdhci_msm_setup_vreg(struct sdhci_msm_host *msm_host,
			bool enable, bool is_init)
{
	int ret = 0, i;
	struct sdhci_msm_vreg_data *curr_slot;
	struct sdhci_msm_reg_data *vreg_table[2];
	struct mmc_host *mmc = msm_host->mmc;
	u32 val = 0;

	sdhci_msm_log_str(msm_host, "%s regulators\n",
			enable ? "Enabling" : "Disabling");

	curr_slot = msm_host->vreg_data;
	if (!curr_slot) {
		pr_debug("%s: vreg info unavailable,assuming the slot is powered by always on domain\n",
			 __func__);
		goto out;
	}

	vreg_table[0] = curr_slot->vdd_data;
	vreg_table[1] = curr_slot->vdd_io_data;

	/* When eMMC absent disable regulator marked as always_on */
	if (!enable && vreg_table[1]->is_always_on && !mmc->card)
		vreg_table[1]->is_always_on = false;

	if (mmc->card &&
		mmc->card->ext_csd.power_off_notification == EXT_CSD_NO_POWER_NOTIFICATION
		&& mmc->caps & MMC_CAP_NONREMOVABLE)
		return ret;

	if (!enable && !(mmc->caps & MMC_CAP_NONREMOVABLE)) {

		/*
		 * Disable Receiver of the Pad to avoid crowbar currents
		 * when Pad power supplies are collapsed. Provide SW control
		 * on the core_ie of SDC2 Pads. SW write 1’b0
		 * into the bit 15 of register TLMM_NORTH_SPARE.
		 */

		val = msm_spare_read(TLMM_NORTH_SPARE);
		val &= ~TLMM_NORTH_SPARE_CORE_IE;
		msm_spare_write(TLMM_NORTH_SPARE, val);
	}

	for (i = 0; i < ARRAY_SIZE(vreg_table); i++) {
		if (vreg_table[i]) {
			if (enable)
				ret = sdhci_msm_vreg_enable(vreg_table[i]);
			else
				ret = sdhci_msm_vreg_disable(vreg_table[i]);
			if (ret)
				goto out;
		}
	}

	if (enable && !(mmc->caps & MMC_CAP_NONREMOVABLE)) {

		/*
		 * Disable Receiver of the Pad to avoid crowbar currents
		 * when Pad power supplies are collapsed. Provide SW control
		 * on the core_ie of SDC2 Pads. SW write 1’b1
		 * into the bit 15 of register TLMM_NORTH_SPARE.
		 */

		val = msm_spare_read(TLMM_NORTH_SPARE);
		val |= TLMM_NORTH_SPARE_CORE_IE;
		msm_spare_write(TLMM_NORTH_SPARE, val);
	}

out:
	return ret;
}

/* This init function should be called only once for each SDHC slot */
static int sdhci_msm_vreg_init(struct device *dev,
				struct sdhci_msm_host *msm_host,
				bool is_init)
{
	int ret = 0;
	struct sdhci_msm_vreg_data *curr_slot;
	struct sdhci_msm_reg_data *curr_vdd_reg, *curr_vdd_io_reg;

	curr_slot = msm_host->vreg_data;
	if (!curr_slot)
		goto out;

	curr_vdd_reg = curr_slot->vdd_data;
	curr_vdd_io_reg = curr_slot->vdd_io_data;

	if (!is_init)
		/* Deregister all regulators from regulator framework */
		goto out;

	/*
	 * Get the regulator handle from voltage regulator framework
	 * and then try to set the voltage level for the regulator
	 */
	if (curr_vdd_reg) {
		ret = sdhci_msm_vreg_init_reg(dev, curr_vdd_reg);
		if (ret)
			goto out;
	}
	if (curr_vdd_io_reg)
		ret = sdhci_msm_vreg_init_reg(dev, curr_vdd_io_reg);
out:
	if (ret)
		dev_err(dev, "vreg reset failed (%d)\n", ret);

	return ret;
}

static int sdhci_msm_set_vdd_io_vol(struct sdhci_msm_host *msm_host,
			enum vdd_io_level level,
			unsigned int voltage_level)
{
	int ret = 0;
	int set_level;
	struct sdhci_msm_reg_data *vdd_io_reg;

	if (!msm_host->vreg_data)
		return ret;

	vdd_io_reg = msm_host->vreg_data->vdd_io_data;
	if (vdd_io_reg && vdd_io_reg->is_enabled) {
		switch (level) {
		case VDD_IO_LOW:
			set_level = vdd_io_reg->low_vol_level;
			break;
		case VDD_IO_HIGH:
			set_level = vdd_io_reg->high_vol_level;
			break;
		case VDD_IO_SET_LEVEL:
			set_level = voltage_level;
			break;
		default:
			pr_err("%s: invalid argument level = %d\n",
					__func__, level);
			ret = -EINVAL;
			return ret;
		}
		ret = sdhci_msm_vreg_set_voltage(vdd_io_reg, set_level,
				set_level);
	}
	return ret;
}

static void sdhci_msm_vbias_bypass_wa(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_host_offset =
					msm_host->offset;
	struct mmc_host *mmc = host->mmc;
	u32 config;
	int card_detect = 0;

	if (mmc->ops->get_cd)
		card_detect = mmc->ops->get_cd(mmc);

	config = readl_relaxed(host->ioaddr +
			msm_host_offset->core_vendor_spec);

	/*
	 * Following cases are covered.
	 * 1. Card Probe
	 * 2. Card suspend
	 * 3. Card Resume
	 * 4. Card remove
	 */
	if ((mmc->card == NULL) && card_detect &&
		(mmc->ios.power_mode == MMC_POWER_UP))
		config &= ~CORE_IO_PAD_PWR_SWITCH;
	else if (mmc->card && card_detect &&
			(mmc->ios.power_mode == MMC_POWER_OFF))
		config |= CORE_IO_PAD_PWR_SWITCH;
	else if (mmc->card && card_detect &&
			(mmc->ios.power_mode == MMC_POWER_UP))
		config &= ~CORE_IO_PAD_PWR_SWITCH;
	else if (mmc->card == NULL && !card_detect &&
			(mmc->ios.power_mode == MMC_POWER_OFF))
		config |= CORE_IO_PAD_PWR_SWITCH;

	writel_relaxed(config, host->ioaddr +
				msm_host_offset->core_vendor_spec);
}

static void sdhci_msm_handle_pwr_irq(struct sdhci_host *host, int irq)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct mmc_host *mmc = host->mmc;
	u32 irq_status, irq_ack = 0;
	int retry = 10, ret = 0;
	u32 pwr_state = 0, io_level = 0;
	u32 config;
	const struct sdhci_msm_offset *msm_offset = msm_host->offset;

	irq_status = msm_host_readl(msm_host, host,
			msm_offset->core_pwrctl_status);
	irq_status &= INT_MASK;

	sdhci_msm_log_str(msm_host, "Received IRQ(%d), irq_status=0x%x\n",
			irq, irq_status);

	msm_host_writel(msm_host, irq_status, host,
			msm_offset->core_pwrctl_clear);

	/*
	 * There is a rare HW scenario where the first clear pulse could be
	 * lost when actual reset and clear/read of status register is
	 * happening at a time. Hence, retry for at least 10 times to make
	 * sure status register is cleared. Otherwise, this will result in
	 * a spurious power IRQ resulting in system instability.
	 */
	while (irq_status & msm_host_readl(msm_host, host,
				msm_offset->core_pwrctl_status)) {
		if (retry == 0) {
			pr_err("%s: Timedout clearing (0x%x) pwrctl status register\n",
					mmc_hostname(host->mmc), irq_status);
			sdhci_msm_dump_pwr_ctrl_regs(host);
			WARN_ON(1);
			break;
		}
		msm_host_writel(msm_host, irq_status, host,
			msm_offset->core_pwrctl_clear);
		retry--;
		udelay(10);
	}

	if (mmc->card && mmc->ops->get_cd && !mmc->ops->get_cd(mmc) &&
		irq_status & CORE_PWRCTL_BUS_ON) {
		irq_ack = CORE_PWRCTL_BUS_FAIL;
		msm_host_writel(msm_host, irq_ack, host,
				msm_offset->core_pwrctl_ctl);
		return;
	}
	/* Handle BUS ON/OFF*/
	if (irq_status & CORE_PWRCTL_BUS_ON) {
		ret = sdhci_msm_setup_vreg(msm_host, true, false);
		if (!ret)
			ret = sdhci_msm_set_vdd_io_vol(msm_host,
					VDD_IO_HIGH, 0);
		if (ret)
			irq_ack |= CORE_PWRCTL_BUS_FAIL;
		else
			irq_ack |= CORE_PWRCTL_BUS_SUCCESS;

		pwr_state = REQ_BUS_ON;
		io_level = REQ_IO_HIGH;
	}
	if (irq_status & CORE_PWRCTL_BUS_OFF) {
		if (!(host->mmc->caps & MMC_CAP_NONREMOVABLE) ||
		    msm_host->pltfm_init_done)
			ret = sdhci_msm_setup_vreg(msm_host,
					false, false);
		if (!ret)
			ret = sdhci_msm_set_vdd_io_vol(msm_host,
					VDD_IO_LOW, 0);
		if (ret)
			irq_ack |= CORE_PWRCTL_BUS_FAIL;
		else
			irq_ack |= CORE_PWRCTL_BUS_SUCCESS;

		pwr_state = REQ_BUS_OFF;
		io_level = REQ_IO_LOW;
	}

	if (pwr_state) {
		ret = sdhci_msm_set_vmmc(mmc);
		if (!ret)
			ret = sdhci_msm_set_vqmmc(msm_host, mmc,
					pwr_state & REQ_BUS_ON);
		if (!ret)
			ret = sdhci_msm_set_pincfg(msm_host,
					pwr_state & REQ_BUS_ON);
		if (!ret && (pwr_state & REQ_BUS_ON) && msm_host->ggc.bh201_used) {
			pr_info("bh201 delay 100ms\n");
			mdelay(100);
		}
		if (!ret)
			irq_ack |= CORE_PWRCTL_BUS_SUCCESS;
		else
			irq_ack |= CORE_PWRCTL_BUS_FAIL;

		msm_host->curr_pwr_state = pwr_state;
	}

	/* Handle IO LOW/HIGH */
	if (irq_status & CORE_PWRCTL_IO_LOW) {
		io_level = REQ_IO_LOW;
		/* Switch voltage low */
		ret = sdhci_msm_set_vdd_io_vol(msm_host, VDD_IO_LOW, 0);
		if (ret)
			irq_ack |= CORE_PWRCTL_IO_FAIL;
		else
			irq_ack |= CORE_PWRCTL_IO_SUCCESS;
	}

	if (irq_status & CORE_PWRCTL_IO_HIGH) {
		io_level = REQ_IO_HIGH;
		/* Switch voltage high */
		ret = sdhci_msm_set_vdd_io_vol(msm_host, VDD_IO_HIGH, 0);
		if (ret)
			irq_ack |= CORE_PWRCTL_IO_FAIL;
		else
			irq_ack |= CORE_PWRCTL_IO_SUCCESS;
	}

	if (io_level && !IS_ERR(mmc->supply.vqmmc) && !pwr_state) {
		ret = mmc_regulator_set_vqmmc(mmc, &mmc->ios);
		if (ret < 0) {
			dev_err(mmc_dev(mmc), "%s: IO_level setting failed(%d). signal_voltage: %d, vdd: %d irq_status: 0x%08x\n",
					mmc_hostname(mmc), ret,
					mmc->ios.signal_voltage, mmc->ios.vdd,
					irq_status);
			irq_ack |= CORE_PWRCTL_IO_FAIL;
		}
	}

	/*
	 * The driver has to acknowledge the interrupt, switch voltages and
	 * report back if it succeded or not to this register. The voltage
	 * switches are handled by the sdhci core, so just report success.
	 */
	msm_host_writel(msm_host, irq_ack, host,
			msm_offset->core_pwrctl_ctl);

	/*
	 * If we don't have info regarding the voltage levels supported by
	 * regulators, don't change the IO PAD PWR SWITCH.
	 */
	if (msm_host->caps_0 & CORE_VOLT_SUPPORT) {
		u32 new_config;
		/*
		 * We should unset IO PAD PWR switch only if the register write
		 * can set IO lines high and the regulator also switches to 3 V.
		 * Else, we should keep the IO PAD PWR switch set.
		 * This is applicable to certain targets where eMMC vccq supply
		 * is only 1.8V. In such targets, even during REQ_IO_HIGH, the
		 * IO PAD PWR switch must be kept set to reflect actual
		 * regulator voltage. This way, during initialization of
		 * controllers with only 1.8V, we will set the IO PAD bit
		 * without waiting for a REQ_IO_LOW.
		 */
		config = readl_relaxed(host->ioaddr +
				msm_offset->core_vendor_spec);
		new_config = config;

		if ((io_level & REQ_IO_HIGH) &&
				(msm_host->caps_0 & CORE_3_0V_SUPPORT) &&
				!msm_host->fake_core_3_0v_support) {
			if (msm_host->vbias_skip_wa)
				sdhci_msm_vbias_bypass_wa(host);
			else
				new_config &= ~CORE_IO_PAD_PWR_SWITCH;
		} else if ((io_level & REQ_IO_LOW) ||
				(msm_host->caps_0 & CORE_1_8V_SUPPORT)) {
			new_config |= CORE_IO_PAD_PWR_SWITCH;
		}

		if (config ^ new_config)
			writel_relaxed(new_config, host->ioaddr +
					msm_offset->core_vendor_spec);
	}

	if (io_level)
		msm_host->curr_io_level = io_level;

	dev_dbg(mmc_dev(mmc), "%s: %s: Handled IRQ(%d), irq_status=0x%x, ack=0x%x\n",
		mmc_hostname(msm_host->mmc), __func__, irq, irq_status,
		irq_ack);

	sdhci_msm_log_str(msm_host, "Handled IRQ(%d), irq_status=0x%x, ack=0x%x\n",
			irq, irq_status, irq_ack);
}

static irqreturn_t sdhci_msm_pwr_irq(int irq, void *data)
{
	struct sdhci_host *host = (struct sdhci_host *)data;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	sdhci_msm_handle_pwr_irq(host, irq);
	msm_host->pwr_irq_flag = 1;
	sdhci_msm_complete_pwr_irq_wait(msm_host);

	return IRQ_HANDLED;
}

static unsigned int sdhci_msm_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct clk *core_clk = msm_host->bulk_clks[0].clk;

	return clk_round_rate(core_clk, ULONG_MAX);
}

static unsigned int sdhci_msm_get_min_clock(struct sdhci_host *host)
{
	return SDHCI_MSM_MIN_CLOCK;
}

static unsigned int sdhci_msm_fmr_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct device *dev = mmc_dev(host->mmc);
	struct device *perf_dev;
	struct dev_pm_opp *opp;
	unsigned long max_freq = ULONG_MAX;

	if (!msm_host->pd_list) {
		dev_err(dev, "pd_list is NULL\n");
		return SDHCI_MSM_MIN_CLOCK;
	}

	perf_dev = msm_host->pd_list->pd_devs[DOMAIN_IDX_PERF];
	if (!perf_dev) {
		dev_err(dev, "perf_dev is NULL\n");
		return SDHCI_MSM_MIN_CLOCK;
	}

	opp = dev_pm_opp_find_freq_floor(perf_dev, &max_freq);
	if (IS_ERR(opp)) {
		dev_err(dev, "Failed to find opp for freq %lu\n", max_freq);
		return SDHCI_MSM_MIN_CLOCK;
	}

	dev_pm_opp_put(opp);

	return max_freq;
}

/*
 * __sdhci_msm_set_clock - sdhci_msm clock control.
 *
 * Description:
 * MSM controller does not use internal divider and
 * instead directly control the GCC clock as per
 * HW recommendation.
 **/
static void __sdhci_msm_set_clock(struct sdhci_host *host, unsigned int clock)
{
	u16 clk;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		return;

	/*
	 * MSM controller do not use clock divider.
	 * Thus read SDHCI_CLOCK_CONTROL and only enable
	 * clock with no divider value programmed.
	 */
	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	sdhci_enable_clk(host, clk);
}

/* sdhci_msm_set_clock - Called with (host->lock) spinlock held. */
static void sdhci_msm_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	if (!clock) {
		host->mmc->actual_clock = msm_host->clk_rate = 0;
		goto out;
	}

	sdhci_msm_hc_select_mode(host);

	msm_set_clock_rate_for_bus_mode(host, clock);
out:
	/* Vote on bus only with clock frequency or when changing clock
	 * frequency. No need to vote when setting clock frequency as 0
	 * because after setting clock at 0, we release host, which will
	 * eventually call host runtime suspend and unvoting would be
	 * taken care in runtime suspend call.
	 */
	if (!msm_host->skip_bus_bw_voting && clock)
		sdhci_msm_bus_voting(host, true);
	__sdhci_msm_set_clock(host, clock);
}

static void sdhci_msm_fmr_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	if (!clock) {
		host->mmc->actual_clock = msm_host->clk_rate = 0;
		goto out;
	}

	sdhci_msm_hc_select_mode(host);

	msm_fmr_set_clock_rate_for_bus_mode(host, clock);
out:
	__sdhci_msm_set_clock(host, clock);
}

static int sdhci_msm_fmr_power_state(struct device *dev, bool power_on)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct device *pwr_dev;
	int ret;

	if (!msm_host->pd_list) {
		dev_dbg(dev, "pd_list is NULL\n");
		return -ENODEV;
	}
	pwr_dev = msm_host->pd_list->pd_devs[DOMAIN_IDX_POWER];

	if (!pwr_dev) {
		dev_dbg(dev, "pwr_dev is NULL\n");
		return -ENODEV;
	}

	if (power_on)
		ret = pm_runtime_resume_and_get(pwr_dev);
	else
		ret = pm_runtime_put_sync(pwr_dev);

	if (ret)
		dev_dbg(dev, "Failed to switch power state(high=%d) ret=%d\n",
			power_on, ret);

	return ret;
}

/*****************************************************************************\
 *                                                                           *
 * Inline Crypto Engine (ICE) support                                        *
 *                                                                           *
\*****************************************************************************/

#ifdef CONFIG_MMC_CRYPTO

static int sdhci_msm_ice_init(struct sdhci_msm_host *msm_host,
			      struct cqhci_host *cq_host)
{
	struct mmc_host *mmc = msm_host->mmc;
	struct device *dev = mmc_dev(mmc);
#if IS_ENABLED(CONFIG_QTI_HW_KEY_MANAGER)
	struct resource *ice_hwkm_res;
#endif
	struct qcom_ice *ice;

	if (!(cqhci_readl(cq_host, CQHCI_CAP) & CQHCI_CAP_CS))
		return 0;

	ice = of_qcom_ice_get(dev);
	if (ice == ERR_PTR(-EOPNOTSUPP)) {
		dev_warn(dev, "Disabling inline encryption support\n");
		ice = NULL;
	}

#if IS_ENABLED(CONFIG_QTI_HW_KEY_MANAGER)
	ice_hwkm_res = platform_get_resource_byname(msm_host->pdev,
						    IORESOURCE_MEM,
						    "cqhci_ice_hwkm");
	if (!ice_hwkm_res) {
		dev_warn(dev, "ICE HWKM registers not found\n");
		goto disable;
	}
	msm_host->ice_hwkm_mem = devm_ioremap_resource(dev, ice_hwkm_res);
	if (IS_ERR(msm_host->ice_hwkm_mem)) {
		err = PTR_ERR(msm_host->ice_hwkm_mem);
		dev_err(dev, "Failed to map ICE HWKM registers; err=%d\n", err);
		return err;
	}
#endif

	if (IS_ERR_OR_NULL(ice))
		return PTR_ERR_OR_ZERO(ice);

	msm_host->ice = ice;
	mmc->caps2 |= MMC_CAP2_CRYPTO;

	return 0;
}

static void sdhci_msm_ice_enable(struct sdhci_msm_host *msm_host)
{
	int err = 0;
	if (msm_host->mmc->caps2 & MMC_CAP2_CRYPTO)
		err = qcom_ice_enable(msm_host->ice);

	if (err) {
		dev_warn(
			mmc_dev(msm_host->mmc),
			"Could not enable ICE err=%d, Disabling inline encryption capability.\n",
			err);
		msm_host->mmc->caps2 &= ~MMC_CAP2_CRYPTO;
	}
}

static __maybe_unused int sdhci_msm_ice_resume(struct sdhci_msm_host *msm_host)
{
	int err = 0;

	if (msm_host->mmc->caps2 & MMC_CAP2_CRYPTO)
		err = qcom_ice_resume(msm_host->ice);

	if (err) {
		dev_warn(
			mmc_dev(msm_host->mmc),
			"Could not resume ICE err=%d, Disabling inline encryption capability.\n",
			err);
		msm_host->mmc->caps2 &= ~MMC_CAP2_CRYPTO;
	}

	return err;
}

static __maybe_unused int sdhci_msm_ice_suspend(struct sdhci_msm_host *msm_host)
{
	if (msm_host->mmc->caps2 & MMC_CAP2_CRYPTO)
		return qcom_ice_suspend(msm_host->ice);

	return 0;
}

/*
 * Program a key into a QC ICE keyslot, or evict a keyslot.  QC ICE requires
 * vendor-specific SCM calls for this; it doesn't support the standard way.
 */
static int sdhci_msm_program_key(struct cqhci_host *cq_host,
				 const union cqhci_crypto_cfg_entry *cfg,
				 int slot)
{
	struct sdhci_host *host = mmc_priv(cq_host->mmc);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	union cqhci_crypto_cap_entry cap;

	/* Only AES-256-XTS has been tested so far. */
	cap = cq_host->crypto_cap_array[cfg->crypto_cap_idx];
	if (cap.algorithm_id != CQHCI_CRYPTO_ALG_AES_XTS ||
		cap.key_size != CQHCI_CRYPTO_KEY_SIZE_256)
		return -EINVAL;

	if (cfg->config_enable & CQHCI_CRYPTO_CONFIGURATION_ENABLE)
		return qcom_ice_program_key(msm_host->ice,
					    QCOM_ICE_CRYPTO_ALG_AES_XTS,
					    QCOM_ICE_CRYPTO_KEY_SIZE_256,
					    cfg->crypto_key,
					    cfg->data_unit_size, slot);
	else
		return qcom_ice_evict_key(msm_host->ice, slot);
}

void sdhci_msm_ice_disable(struct sdhci_msm_host *msm_host)
{
	if (!(msm_host->mmc->caps2 & MMC_CAP2_CRYPTO))
		return;
}
#else /* CONFIG_MMC_CRYPTO */

static inline int sdhci_msm_ice_init(struct sdhci_msm_host *msm_host,
				     struct cqhci_host *cq_host)
{
	return 0;
}

static inline void sdhci_msm_ice_enable(struct sdhci_msm_host *msm_host)
{
}

static inline __maybe_unused int
sdhci_msm_ice_resume(struct sdhci_msm_host *msm_host)
{
	return 0;
}

static inline __maybe_unused int
sdhci_msm_ice_suspend(struct sdhci_msm_host *msm_host)
{
	return 0;
}

void sdhci_msm_ice_disable(struct sdhci_msm_host *msm_host)
{
}
#endif /* !CONFIG_MMC_CRYPTO */

/*****************************************************************************\
 *                                                                           *
 * MSM Command Queue Engine (CQE)                                            *
 *                                                                           *
\*****************************************************************************/

static void sdhci_msm_log_irq(struct sdhci_host *host, u32 intmask)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	sdhci_msm_log_str(msm_host, "intmask: 0x%x\n", intmask);
}

static u32 sdhci_msm_cqe_irq(struct sdhci_host *host, u32 intmask)
{
	int cmd_error = 0;
	int data_error = 0;

	sdhci_msm_log_irq(host, intmask);

	if (!sdhci_cqe_irq(host, intmask, &cmd_error, &data_error))
		return intmask;

	cqhci_irq(host->mmc, intmask, cmd_error, data_error);
	return 0;
}

static void sdhci_msm_cqe_enable(struct mmc_host *mmc)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	sdhci_cqe_enable(mmc);
	sdhci_msm_ice_enable(msm_host);
}

static void sdhci_msm_cqe_disable(struct mmc_host *mmc, bool recovery)
{
	struct sdhci_host *host = mmc_priv(mmc);
	unsigned long flags;
	u32 ctrl;

	/*
	 * When CQE is halted, the legacy SDHCI path operates only
	 * on 16-byte descriptors in 64bit mode.
	 */
	if (host->flags & SDHCI_USE_64_BIT_DMA)
		host->desc_sz = 16;

	spin_lock_irqsave(&host->lock, flags);

	/*
	 * During CQE command transfers, command complete bit gets latched.
	 * So s/w should clear command complete interrupt status when CQE is
	 * either halted or disabled. Otherwise unexpected SDCHI legacy
	 * interrupt gets triggered when CQE is halted/disabled.
	 */
	ctrl = sdhci_readl(host, SDHCI_INT_ENABLE);
	ctrl |= SDHCI_INT_RESPONSE;
	sdhci_writel(host,  ctrl, SDHCI_INT_ENABLE);
	sdhci_writel(host, SDHCI_INT_RESPONSE, SDHCI_INT_STATUS);

	spin_unlock_irqrestore(&host->lock, flags);

	sdhci_cqe_disable(mmc, recovery);
}

static void sdhci_msm_set_timeout(struct sdhci_host *host, struct mmc_command *cmd)
{
	u32 count, start = 15;

	/*
	 * Qcom SoC hardware data timeout value was calculated
	 * using 4 * MCLK * 2^(count + 13). where MCLK = 1 / host->clock.
	 */

	host->timeout_clk = host->mmc->actual_clock ?
				host->mmc->actual_clock / 1000 :
				host->clock / 1000;
	host->timeout_clk /= 4;

	__sdhci_set_timeout(host, cmd);
	count = sdhci_readb(host, SDHCI_TIMEOUT_CONTROL);

	/*
	 * Update software timeout value if its value is less than hardware data
	 * timeout value.
	 */
	if (cmd && cmd->data && host->clock > 400000 &&
	    host->clock <= 50000000 &&
	    ((1 << (count + start)) > (10 * host->clock)))
		host->data_timeout = 22LL * NSEC_PER_SEC;
}

static const struct cqhci_host_ops sdhci_msm_cqhci_ops = {
	.enable		= sdhci_msm_cqe_enable,
	.disable	= sdhci_msm_cqe_disable,
	.enhanced_strobe_mask = sdhci_msm_enhanced_strobe_mask,
#ifdef CONFIG_MMC_CRYPTO
	.program_key	= sdhci_msm_program_key,
#endif
};

static int sdhci_msm_cqe_add_host(struct sdhci_host *host,
				struct platform_device *pdev)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct cqhci_host *cq_host;
	bool dma64;
	u32 cqcfg;
	int ret;

	/*
	 * When CQE is halted, SDHC operates only on 16byte ADMA descriptors.
	 * So ensure ADMA table is allocated for 16byte descriptors.
	 */
	if (host->caps & SDHCI_CAN_64BIT)
		host->alloc_desc_sz = 16;

	ret = sdhci_setup_host(host);
	if (ret)
		return ret;

	cq_host = cqhci_pltfm_init(pdev);
	if (IS_ERR(cq_host)) {
		ret = PTR_ERR(cq_host);
		dev_err(&pdev->dev, "cqhci-pltfm init: failed: %d\n", ret);
		goto cleanup;
	}

	msm_host->mmc->caps2 |= MMC_CAP2_CQE | MMC_CAP2_CQE_DCMD;
	cq_host->ops = &sdhci_msm_cqhci_ops;
	msm_host->cq_host = cq_host;
	cq_host->offset_changed = msm_host->cqhci_offset_changed;

	dma64 = host->flags & SDHCI_USE_64_BIT_DMA;

	ret = sdhci_msm_ice_init(msm_host, cq_host);
	if (ret)
		goto cleanup;

#if IS_ENABLED(CONFIG_MMC_CRYPTO_QTI)
	cq_host->ice = msm_host->ice;
#endif
	ret = cqhci_init(cq_host, host->mmc, dma64);
	if (ret) {
		dev_err(&pdev->dev, "%s: CQE init: failed (%d)\n",
				mmc_hostname(host->mmc), ret);
		goto cleanup;
	}

	/* Disable cqe reset due to cqe enable signal */
	cqcfg = cqhci_readl(cq_host, CQHCI_VENDOR_CFG1);
	cqcfg |= CQHCI_VENDOR_DIS_RST_ON_CQ_EN;
	cqhci_writel(cq_host, cqcfg, CQHCI_VENDOR_CFG1);

	/*
	 * SDHC expects 12byte ADMA descriptors till CQE is enabled.
	 * So limit desc_sz to 12 so that the data commands that are sent
	 * during card initialization (before CQE gets enabled) would
	 * get executed without any issues.
	 */
	if (host->flags & SDHCI_USE_64_BIT_DMA)
		host->desc_sz = 12;

	ret = __sdhci_add_host(host);
	if (ret)
		goto cleanup;

	dev_info(&pdev->dev, "%s: CQE init: success\n",
			mmc_hostname(host->mmc));
	return ret;

cleanup:
	sdhci_cleanup_host(host);
	return ret;
}

static void sdhci_msm_registers_save(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_offset = msm_host->offset;
	struct cqhci_host *cq_host = host->mmc->cqe_private;

	if (!msm_host->regs_restore.is_supported &&
			!msm_host->reg_store)
		return;

	msm_host->regs_restore.vendor_func = readl_relaxed(host->ioaddr +
		msm_offset->core_vendor_spec);
	msm_host->regs_restore.vendor_pwrctl_mask =
		readl_relaxed(host->ioaddr +
		msm_offset->core_pwrctl_mask);
	msm_host->regs_restore.vendor_func2 =
		readl_relaxed(host->ioaddr +
		msm_offset->core_vendor_spec_func2);
	msm_host->regs_restore.vendor_func3 =
		readl_relaxed(host->ioaddr +
		msm_offset->core_vendor_spec3);
	msm_host->regs_restore.hc_2c_2e =
		sdhci_readl(host, SDHCI_CLOCK_CONTROL);
	msm_host->regs_restore.hc_3c_3e =
		sdhci_readl(host, SDHCI_AUTO_CMD_STATUS);
	msm_host->regs_restore.vendor_pwrctl_ctl =
		readl_relaxed(host->ioaddr +
		msm_offset->core_pwrctl_ctl);
	msm_host->regs_restore.hc_38_3a =
		sdhci_readl(host, SDHCI_SIGNAL_ENABLE);
	msm_host->regs_restore.hc_34_36 =
		sdhci_readl(host, SDHCI_INT_ENABLE);
	msm_host->regs_restore.hc_28_2a =
		sdhci_readl(host, SDHCI_HOST_CONTROL);
	msm_host->regs_restore.vendor_caps_0 =
		readl_relaxed(host->ioaddr +
		msm_offset->core_vendor_spec_capabilities0);
	msm_host->regs_restore.hc_caps_1 =
		sdhci_readl(host, SDHCI_CAPABILITIES_1);
	msm_host->regs_restore.ext_fb_clk =
		sdhci_readl(host, VENDOR_SPEC_FUNC4);
	msm_host->regs_restore.testbus_config = readl_relaxed(host->ioaddr +
		msm_offset->core_testbus_config);
	msm_host->regs_restore.dll_config = readl_relaxed(host->ioaddr +
		msm_offset->core_dll_config);
	msm_host->regs_restore.dll_config2 = readl_relaxed(host->ioaddr +
		msm_offset->core_dll_config_2);
	msm_host->regs_restore.dll_config = readl_relaxed(host->ioaddr +
		msm_offset->core_dll_config);
	msm_host->regs_restore.dll_config3 = readl_relaxed(host->ioaddr +
		msm_offset->core_dll_config_3);
	msm_host->regs_restore.dll_usr_ctl = readl_relaxed(host->ioaddr +
		msm_offset->core_dll_usr_ctl);
	if (cq_host)
		msm_host->cqe_regs.cqe_vendor_cfg1 =
			cqhci_readl(cq_host, CQHCI_VENDOR_CFG1);

	msm_host->regs_restore.is_valid = true;

	pr_debug("%s: %s: registers saved. PWRCTL_MASK = 0x%x\n",
		mmc_hostname(host->mmc), __func__,
		readl_relaxed(host->ioaddr +
			msm_offset->core_pwrctl_mask));

	sdhci_msm_log_str(msm_host, "Registers saved\n");
}

static void sdhci_msm_registers_restore(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_offset = msm_host->offset;
	u32 irq_status;
	struct mmc_ios ios = host->mmc->ios;
	struct cqhci_host *cq_host = host->mmc->cqe_private;

	if ((!msm_host->regs_restore.is_supported ||
		!msm_host->regs_restore.is_valid) &&
		!msm_host->reg_store)
		return;

	writel_relaxed(0, host->ioaddr + msm_offset->core_pwrctl_mask);
	writel_relaxed(msm_host->regs_restore.vendor_func, host->ioaddr +
			msm_offset->core_vendor_spec);
	writel_relaxed(msm_host->regs_restore.vendor_func2,
			host->ioaddr +
			msm_offset->core_vendor_spec_func2);
	writel_relaxed(msm_host->regs_restore.vendor_func3,
			host->ioaddr +
			msm_offset->core_vendor_spec3);
	sdhci_writel(host, msm_host->regs_restore.hc_2c_2e,
			SDHCI_CLOCK_CONTROL);
	sdhci_writel(host, msm_host->regs_restore.hc_3c_3e,
			SDHCI_AUTO_CMD_STATUS);
	sdhci_writel(host, msm_host->regs_restore.hc_38_3a,
			SDHCI_SIGNAL_ENABLE);
	sdhci_writel(host, msm_host->regs_restore.hc_34_36,
			SDHCI_INT_ENABLE);
	sdhci_writel(host, msm_host->regs_restore.hc_28_2a,
			SDHCI_HOST_CONTROL);
	sdhci_writel(host, msm_host->regs_restore.ext_fb_clk,
			VENDOR_SPEC_FUNC4);
	writel_relaxed(msm_host->regs_restore.vendor_caps_0,
			host->ioaddr +
			msm_offset->core_vendor_spec_capabilities0);
	sdhci_writel(host, msm_host->regs_restore.hc_caps_1,
			SDHCI_CAPABILITIES_1);
	writel_relaxed(msm_host->regs_restore.testbus_config, host->ioaddr +
			msm_offset->core_testbus_config);
	msm_host->regs_restore.is_valid = false;

	/*
	 * Clear the PWRCTL_STATUS register.
	 * There is a rare HW scenario where the first clear pulse could be
	 * lost when actual reset and clear/read of status register is
	 * happening at a time. Hence, retry for at least 10 times to make
	 * sure status register is cleared. Otherwise, this will result in
	 * a spurious power IRQ resulting in system instability.
	 */
	irq_status = msm_host_readl(msm_host, host,
			msm_offset->core_pwrctl_status);

	irq_status &= INT_MASK;
	sdhci_msm_clear_pwrctl_status(host, irq_status);

	writel_relaxed(msm_host->regs_restore.vendor_pwrctl_ctl,
			host->ioaddr + msm_offset->core_pwrctl_ctl);
	writel_relaxed(msm_host->regs_restore.vendor_pwrctl_mask,
			host->ioaddr + msm_offset->core_pwrctl_mask);

	if (cq_host)
		cqhci_writel(cq_host, msm_host->cqe_regs.cqe_vendor_cfg1 &
				~CMDQ_SEND_STATUS_TRIGGER, CQHCI_VENDOR_CFG1);

	if ((ios.timing == MMC_TIMING_UHS_SDR50 &&
		host->flags & SDHCI_SDR50_NEEDS_TUNING) ||
		(((ios.timing == MMC_TIMING_MMC_HS400) ||
			(ios.timing == MMC_TIMING_MMC_HS200) ||
			(ios.timing == MMC_TIMING_UHS_SDR104))
			&& (ios.clock > CORE_FREQ_100MHZ))) {
		writel_relaxed(msm_host->regs_restore.dll_config2,
			host->ioaddr + msm_offset->core_dll_config_2);
		writel_relaxed(msm_host->regs_restore.dll_config3,
			host->ioaddr + msm_offset->core_dll_config_3);
		writel_relaxed(msm_host->regs_restore.dll_usr_ctl,
			host->ioaddr + msm_offset->core_dll_usr_ctl);
		writel_relaxed(msm_host->regs_restore.dll_config &
			~(CORE_DLL_RST | CORE_DLL_PDN),
			host->ioaddr + msm_offset->core_dll_config);

		msm_init_cm_dll(host, DLL_INIT_FROM_CX_COLLAPSE_EXIT);
		msm_config_cm_dll_phase(host, msm_host->saved_tuning_phase);
	}

	pr_debug("%s: %s: registers restored. PWRCTL_MASK = 0x%x\n",
		mmc_hostname(host->mmc), __func__,
		readl_relaxed(host->ioaddr +
			msm_offset->core_pwrctl_mask));

	sdhci_msm_log_str(msm_host, "Registers restored\n");
}

/*
 * Platform specific register write functions. This is so that, if any
 * register write needs to be followed up by platform specific actions,
 * they can be added here. These functions can go to sleep when writes
 * to certain registers are done.
 * These functions are relying on sdhci_set_ios not using spinlock.
 */
static int __sdhci_msm_check_write(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	u32 req_type = 0;

	switch (reg) {
	case SDHCI_HOST_CONTROL2:
		req_type = (val & SDHCI_CTRL_VDD_180) ? REQ_IO_LOW :
			REQ_IO_HIGH;
		break;
	case SDHCI_SOFTWARE_RESET:
		if (host->pwr && (val & SDHCI_RESET_ALL))
			req_type = REQ_BUS_OFF;
		break;
	case SDHCI_POWER_CONTROL:
		req_type = !val ? REQ_BUS_OFF : REQ_BUS_ON;
		break;
	case SDHCI_TRANSFER_MODE:
		msm_host->transfer_mode = val;
		break;
	case SDHCI_COMMAND:
		/*
		 * If a command does not have CRC field in its response, as per
		 * design, host controller shall clear the CRC error flag for
		 * it, which takes 3 MCLK's, i.e. 7.5us in the case of 400KHz.
		 * The next command, which requires CRC check to its response,
		 * should be sent after the CRC error flag of previous command
		 * gets cleaned.
		 */
		if ((val & SDHCI_CMD_CRC) &&
		    (msm_host->last_cmd & SDHCI_CMD_FLAGS_MASK) &&
		    !(msm_host->last_cmd & SDHCI_CMD_CRC))
			udelay(8);
		msm_host->last_cmd = val;
		if (!msm_host->use_cdr)
			break;
		if ((msm_host->transfer_mode & SDHCI_TRNS_READ) &&
		    !mmc_op_tuning(SDHCI_GET_CMD(val)))
			sdhci_msm_set_cdr(host, true);
		else
			sdhci_msm_set_cdr(host, false);
		break;
	}

	if (req_type) {
		msm_host->pwr_irq_flag = 0;
		/*
		 * Since this register write may trigger a power irq, ensure
		 * all previous register writes are complete by this point.
		 */
		mb();
	}
	return req_type;
}

/* This function may sleep*/
static void sdhci_msm_writew(struct sdhci_host *host, u16 val, int reg)
{
	u32 req_type = 0;

	req_type = __sdhci_msm_check_write(host, val, reg);
	writew_relaxed(val, host->ioaddr + reg);

	if (req_type)
		sdhci_msm_check_power_status(host, req_type);
}

/* This function may sleep*/
static void sdhci_msm_writeb(struct sdhci_host *host, u8 val, int reg)
{
	u32 req_type = 0;

	req_type = __sdhci_msm_check_write(host, val, reg);

	writeb_relaxed(val, host->ioaddr + reg);

	if (req_type)
		sdhci_msm_check_power_status(host, req_type);
}

static void sdhci_msm_set_regulator_caps(struct sdhci_msm_host *msm_host)
{
	struct mmc_host *mmc = msm_host->mmc;
	struct regulator *supply = mmc->supply.vqmmc;
	u32 caps = 0, config;
	struct sdhci_host *host = mmc_priv(mmc);
	const struct sdhci_msm_offset *msm_offset = msm_host->offset;

	if (!IS_ERR(mmc->supply.vqmmc)) {
		if (regulator_is_supported_voltage(supply, 1700000, 1950000))
			caps |= CORE_1_8V_SUPPORT;
		if (regulator_is_supported_voltage(supply, 2700000, 3600000))
			caps |= CORE_3_0V_SUPPORT;

		if (!caps)
			pr_warn("%s: 1.8/3V not supported for vqmmc\n",
					mmc_hostname(mmc));
	}

	if (caps) {
		/*
		 * Set the PAD_PWR_SWITCH_EN bit so that the PAD_PWR_SWITCH
		 * bit can be used as required later on.
		 */
		u32 io_level = msm_host->curr_io_level;

		config = readl_relaxed(host->ioaddr +
				msm_offset->core_vendor_spec);
		config |= CORE_IO_PAD_PWR_SWITCH_EN;

		if ((io_level & REQ_IO_HIGH) && (caps &	CORE_3_0V_SUPPORT))
			config &= ~CORE_IO_PAD_PWR_SWITCH;
		else if ((io_level & REQ_IO_LOW) || (caps & CORE_1_8V_SUPPORT))
			config |= CORE_IO_PAD_PWR_SWITCH;

		writel_relaxed(config,
				host->ioaddr + msm_offset->core_vendor_spec);
	}
	msm_host->caps_0 |= caps;
	pr_debug("%s: supported caps: 0x%08x\n", mmc_hostname(mmc), caps);
}

static int sdhci_msm_dt_get_array(struct device *dev, const char *prop_name,
				 u32 **bw_vecs, int *len, u32 size)
{
	int ret = 0;
	struct device_node *np = dev->of_node;
	size_t sz;
	u32 *arr = NULL;

	if (!of_get_property(np, prop_name, len)) {
		ret = -EINVAL;
		goto out;
	}
	sz = *len = *len / sizeof(*arr);
	if (sz <= 0 || (size > 0 && (sz > size))) {
		dev_err(dev, "%s invalid size\n", prop_name);
		ret = -EINVAL;
		goto out;
	}

	arr = devm_kzalloc(dev, sz * sizeof(*arr), GFP_KERNEL);
	if (!arr) {
		ret = -ENOMEM;
		goto out;
	}

	ret = of_property_read_u32_array(np, prop_name, arr, sz);
	if (ret < 0) {
		dev_err(dev, "%s failed reading array %d\n", prop_name, ret);
		goto out;
	}
	*bw_vecs = arr;
out:
	if (ret)
		*len = 0;
	return ret;
}

/* Returns required bandwidth in Bytes per Sec */
static unsigned long sdhci_get_bw_required(struct sdhci_host *host,
					struct mmc_ios *ios)
{
	unsigned long bw;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	bw = msm_host->clk_rate;

	if (ios->bus_width == MMC_BUS_WIDTH_4)
		bw /= 2;
	else if (ios->bus_width == MMC_BUS_WIDTH_1)
		bw /= 8;

	return bw;
}

static int sdhci_msm_bus_get_vote_for_bw(struct sdhci_msm_host *host,
					   unsigned int bw)
{
	struct sdhci_msm_bus_vote_data *bvd = host->bus_vote_data;

	const unsigned int *table = bvd->bw_vecs;
	unsigned int size = bvd->bw_vecs_size;
	int i;

	for (i = 0; i < size; i++) {
		if (bw <= table[i])
			return i;
	}

	return i - 1;
}

/*
 * Caller of this function should ensure that msm bus client
 * handle is not null.
 */
static inline int sdhci_msm_bus_set_vote(struct sdhci_msm_host *msm_host,
					     int vote)
{
	struct sdhci_host *host =  platform_get_drvdata(msm_host->pdev);
	struct sdhci_msm_bus_vote_data *bvd = msm_host->bus_vote_data;
	struct msm_bus_path *usecase = bvd->usecase;
	struct msm_bus_vectors *vec = usecase[vote].vec;
	int ddr_rc = 0, cpu_rc = 0;

	if (vote == bvd->curr_vote)
		return 0;

	pr_debug("%s: vote:%d sdhc_ddr ab:%llu ib:%llu cpu_sdhc ab:%llu ib:%llu\n",
			mmc_hostname(host->mmc), vote, vec[0].ab,
			vec[0].ib, vec[1].ab, vec[1].ib);

	if (bvd->sdhc_ddr)
		ddr_rc = icc_set_bw(bvd->sdhc_ddr, vec[0].ab, vec[0].ib);

	if (bvd->cpu_sdhc)
		cpu_rc = icc_set_bw(bvd->cpu_sdhc, vec[1].ab, vec[1].ib);

	if (ddr_rc || cpu_rc) {
		pr_err("%s: icc_set() failed\n",
			mmc_hostname(host->mmc));
		goto out;
	}
	bvd->curr_vote = vote;
out:
	return cpu_rc;
}

/*
 * This function cancels any scheduled delayed work and sets the bus
 * vote based on bw (bandwidth) argument.
 */
static void sdhci_msm_bus_get_and_set_vote(struct sdhci_host *host,
						unsigned int bw)
{
	int vote;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	if (!msm_host->bus_vote_data ||
		(!msm_host->bus_vote_data->sdhc_ddr &&
		!msm_host->bus_vote_data->cpu_sdhc))
		return;
	vote = sdhci_msm_bus_get_vote_for_bw(msm_host, bw);
	sdhci_msm_bus_set_vote(msm_host, vote);
}

static struct sdhci_msm_bus_vote_data *sdhci_msm_get_bus_vote_data(struct device
				       *dev, struct sdhci_msm_host *host)

{
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *of_node = dev->of_node;
	struct sdhci_msm_bus_vote_data *bvd = NULL;
	struct msm_bus_path *usecase = NULL;
	int ret = 0, i = 0, j, num_paths, len;
	const u32 *vec_arr = NULL;

	if (!pdev) {
		dev_err(dev, "Null platform device!\n");
		return NULL;
	}

	bvd = devm_kzalloc(dev, sizeof(*bvd), GFP_KERNEL);
	if (!bvd)
		return bvd;

	ret = sdhci_msm_dt_get_array(dev, "qcom,bus-bw-vectors-bps",
				&bvd->bw_vecs, &bvd->bw_vecs_size, 0);
	if (ret) {
		if (ret == -EINVAL) {
			dev_dbg(dev, "No dt property of bus bw. voting defined!\n");
			dev_dbg(dev, "Skipping Bus BW voting now!!\n");
			host->skip_bus_bw_voting = true;
		}
		goto out;
	}

	ret = of_property_read_string(of_node, "qcom,msm-bus,name",
					&bvd->name);
	if (ret) {
		dev_err(dev, "Bus name missing err:(%d)\n", ret);
		goto out;
	}

	ret = of_property_read_u32(of_node, "qcom,msm-bus,num-cases",
		&bvd->num_usecase);
	if (ret) {
		dev_err(dev, "num-usecases not found err:(%d)\n", ret);
		goto out;
	}

	usecase = devm_kzalloc(dev, (sizeof(struct msm_bus_path) *
				   bvd->num_usecase), GFP_KERNEL);
	if (!usecase)
		goto out;

	ret = of_property_read_u32(of_node, "qcom,msm-bus,num-paths",
				   &num_paths);
	if (ret) {
		dev_err(dev, "num_paths not found err:(%d)\n", ret);
		goto out;
	}

	vec_arr = of_get_property(of_node, "qcom,msm-bus,vectors-KBps", &len);
	if (!vec_arr) {
		dev_err(dev, "Vector array not found\n");
		goto out;
	}

	for (i = 0; i < bvd->num_usecase; i++) {
		usecase[i].num_paths = num_paths;
		usecase[i].vec = devm_kcalloc(dev, num_paths,
					      sizeof(struct msm_bus_vectors),
					      GFP_KERNEL);
		if (!usecase[i].vec)
			goto out;
		for (j = 0; j < num_paths; j++) {
			int idx = ((i * num_paths) + j) * 2;

			usecase[i].vec[j].ab = (u64)
				be32_to_cpu(vec_arr[idx]);
			usecase[i].vec[j].ib = (u64)
				be32_to_cpu(vec_arr[idx + 1]);
		}
	}

	bvd->usecase = usecase;
	return bvd;
out:
	bvd = NULL;
	return bvd;
}

static int sdhci_msm_bus_register(struct sdhci_msm_host *host,
				struct platform_device *pdev)
{
	struct sdhci_msm_bus_vote_data *bsd;
	struct device *dev = &pdev->dev;
	int ret = 0;

	bsd = sdhci_msm_get_bus_vote_data(dev, host);
	if (!bsd) {
		dev_err(&pdev->dev, "Failed to get bus_scale data\n");
		return -EINVAL;
	}
	host->bus_vote_data = bsd;

	bsd->sdhc_ddr = of_icc_get(&pdev->dev, "sdhc-ddr");
	if (IS_ERR_OR_NULL(bsd->sdhc_ddr)) {
		dev_info(&pdev->dev, "(%ld): failed getting %s path\n",
			PTR_ERR(bsd->sdhc_ddr), "sdhc-ddr");
		bsd->sdhc_ddr = NULL;
	}

	bsd->cpu_sdhc = of_icc_get(&pdev->dev, "cpu-sdhc");
	if (IS_ERR_OR_NULL(bsd->cpu_sdhc)) {
		dev_info(&pdev->dev, "(%ld): failed getting %s path\n",
			PTR_ERR(bsd->cpu_sdhc), "cpu-sdhc");
		bsd->cpu_sdhc = NULL;
	}

	return ret;
}

static void sdhci_msm_bus_unregister(struct device *dev,
				struct sdhci_msm_host *host)
{
	struct sdhci_msm_bus_vote_data *bsd = host->bus_vote_data;

	if (bsd->sdhc_ddr)
		icc_put(bsd->sdhc_ddr);

	if (bsd->cpu_sdhc)
		icc_put(bsd->cpu_sdhc);
}

static void sdhci_msm_bus_voting(struct sdhci_host *host, bool enable)
{
	struct mmc_ios *ios = &host->mmc->ios;
	unsigned int bw;

	if (enable) {
		bw = sdhci_get_bw_required(host, ios);
		sdhci_msm_bus_get_and_set_vote(host, bw);
	} else
		sdhci_msm_bus_get_and_set_vote(host, 0);
}

static void sdhci_msm_reset(struct sdhci_host *host, u8 mask)
{
	if ((host->mmc->caps2 & MMC_CAP2_CQE) && (mask & SDHCI_RESET_ALL))
		cqhci_deactivate(host->mmc);
	sdhci_reset(host, mask);
}

static int sdhci_msm_register_vreg(struct sdhci_msm_host *msm_host)
{
	int ret;

	ret = mmc_regulator_get_supply(msm_host->mmc);
	if (ret)
		return ret;

	sdhci_msm_set_regulator_caps(msm_host);

	return 0;
}

static int sdhci_msm_start_signal_voltage_switch(struct mmc_host *mmc,
				      struct mmc_ios *ios)
{
	struct sdhci_host *host = mmc_priv(mmc);
	u16 ctrl, status;

	/*
	 * Signal Voltage Switching is only applicable for Host Controllers
	 * v3.00 and above.
	 */
	if (host->version < SDHCI_SPEC_300)
		return 0;

	ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);

	switch (ios->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		if (!(host->flags & SDHCI_SIGNALING_330))
			return -EINVAL;

		/* Set 1.8V Signal Enable in the Host Control2 register to 0 */
		ctrl &= ~SDHCI_CTRL_VDD_180;
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		if (!(host->flags & SDHCI_SIGNALING_180))
			return -EINVAL;

		/* Enable 1.8V Signal Enable in the Host Control2 register */
		ctrl |= SDHCI_CTRL_VDD_180;
		break;

	default:
		return -EINVAL;
	}

	sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);

	/* Wait for 5ms */
	usleep_range(5000, 5500);

	/* regulator output should be stable within 5 ms */
	status = ctrl & SDHCI_CTRL_VDD_180;
	ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	if ((ctrl & SDHCI_CTRL_VDD_180) == status)
		return 0;

	dev_warn(mmc_dev(mmc), "%s: Regulator output did not became stable\n",
		mmc_hostname(mmc));

	return -EAGAIN;
}

#define MAX_TEST_BUS 60
#define DRIVER_NAME "sdhci_msm"
#define SDHCI_MSM_DUMP(f, x...) \
	pr_err("%s: " DRIVER_NAME ": " f, mmc_hostname(host->mmc), ## x)
#define DRV_NAME "cqhci-host"

static void sdhci_msm_cqe_dump_debug_ram(struct sdhci_host *host)
{
	int i = 0;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_offset = msm_host->offset;

	struct cqhci_host *cq_host;
	u32 version;
	u16 minor;
	int offset;

	if (msm_host->cq_host)
		cq_host = msm_host->cq_host;
	else
		return;

	version =  msm_host_readl(msm_host, host,
				 msm_offset->core_mci_version);
	minor = version & CORE_VERSION_TARGET_MASK;

	/* registers offset changed starting from 4.2.0 */
	offset = minor >= SDHCI_MSM_VER_420 ? 0 : 0x48;

	if (cq_host->offset_changed)
		offset += CQE_V5_VENDOR_CFG;
	pr_err("---- Debug RAM dump ----\n");
	pr_err(DRV_NAME ": Debug RAM wrap-around: 0x%08x | Debug RAM overlap: 0x%08x\n",
	       cqhci_readl(cq_host, CQ_CMD_DBG_RAM_WA + offset),
	       cqhci_readl(cq_host, CQ_CMD_DBG_RAM_OL + offset));

	while (i < 16) {
		pr_err(DRV_NAME ": Debug RAM dump [%d]: 0x%08x\n", i,
		       cqhci_readl(cq_host, CQ_CMD_DBG_RAM + offset + (4 * i)));
		i++;
	}
	pr_err("-------------------------\n");
}

static void sdhci_msm_dump_vendor_regs(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_offset = msm_host->offset;

	int tbsel, tbsel2;
	int i, index = 0;
	u32 test_bus_val = 0;
	u32 debug_reg[MAX_TEST_BUS] = {0};

	SDHCI_MSM_DUMP("----------- VENDOR REGISTER DUMP -----------\n");

	if (msm_host->cq_host)
		sdhci_msm_cqe_dump_debug_ram(host);

	SDHCI_MSM_DUMP(
			"Data cnt: 0x%08x | Fifo cnt: 0x%08x | Int sts: 0x%08x\n",
		readl_relaxed(host->ioaddr + msm_offset->core_mci_data_cnt),
		readl_relaxed(host->ioaddr + msm_offset->core_mci_fifo_cnt),
		readl_relaxed(host->ioaddr + msm_offset->core_mci_status));
	SDHCI_MSM_DUMP(
			"DLL sts: 0x%08x | DLL cfg:  0x%08x | DLL cfg2: 0x%08x\n",
		readl_relaxed(host->ioaddr + msm_offset->core_dll_status),
		readl_relaxed(host->ioaddr + msm_offset->core_dll_config),
		readl_relaxed(host->ioaddr + msm_offset->core_dll_config_2));
	SDHCI_MSM_DUMP(
			"DLL cfg3: 0x%08x | DLL usr ctl:  0x%08x | DDR cfg: 0x%08x\n",
		readl_relaxed(host->ioaddr + msm_offset->core_dll_config_3),
		readl_relaxed(host->ioaddr + msm_offset->core_dll_usr_ctl),
		readl_relaxed(host->ioaddr + msm_offset->core_ddr_config));
	SDHCI_MSM_DUMP(
			"SDCC ver: 0x%08x | Vndr adma err : addr0: 0x%08x addr1: 0x%08x\n",
		readl_relaxed(host->ioaddr + msm_offset->core_mci_version),
		readl_relaxed(host->ioaddr +
				msm_offset->core_vendor_spec_adma_err_addr0),
		readl_relaxed(host->ioaddr +
				msm_offset->core_vendor_spec_adma_err_addr1));
	SDHCI_MSM_DUMP(
			"Vndr func: 0x%08x | Vndr func2 : 0x%08x Vndr func3: 0x%08x\n",
		readl_relaxed(host->ioaddr + msm_offset->core_vendor_spec),
		readl_relaxed(host->ioaddr +
			msm_offset->core_vendor_spec_func2),
		readl_relaxed(host->ioaddr + msm_offset->core_vendor_spec3));
	sdhci_msm_dump_pwr_ctrl_regs(host);

	/*
	 * tbsel indicates [2:0] bits and tbsel2 indicates [7:4] bits
	 * of core_testbus_config register.
	 *
	 * To select test bus 0 to 7 use tbsel and to select any test bus
	 * above 7 use (tbsel2 | tbsel) to get the test bus number. For eg,
	 * to select test bus 14, write 0x1E to core_testbus_config register
	 * i.e., tbsel2[7:4] = 0001, tbsel[2:0] = 110.
	 */
	for (tbsel2 = 0; tbsel2 < 7; tbsel2++) {
		for (tbsel = 0; tbsel < 8; tbsel++) {
			if (index >= MAX_TEST_BUS)
				break;
			test_bus_val =
			(tbsel2 << msm_offset->core_testbus_sel2_bit) |
				tbsel | msm_offset->core_testbus_ena;
			writel_relaxed(test_bus_val, host->ioaddr +
				msm_offset->core_testbus_config);
			debug_reg[index++] = readl_relaxed(host->ioaddr +
					msm_offset->core_sdcc_debug_reg);
		}
	}
	for (i = 0; i < MAX_TEST_BUS; i = i + 4)
		SDHCI_MSM_DUMP(
				" Test bus[%d to %d]: 0x%08x 0x%08x 0x%08x 0x%08x\n",
				i, i + 3, debug_reg[i], debug_reg[i+1],
				debug_reg[i+2], debug_reg[i+3]);

	msm_host->dbg_en = false;
}

static const struct sdhci_msm_variant_ops mci_var_ops = {
	.msm_readl_relaxed = sdhci_msm_mci_variant_readl_relaxed,
	.msm_writel_relaxed = sdhci_msm_mci_variant_writel_relaxed,
};

static const struct sdhci_msm_variant_ops v5_var_ops = {
	.msm_readl_relaxed = sdhci_msm_v5_variant_readl_relaxed,
	.msm_writel_relaxed = sdhci_msm_v5_variant_writel_relaxed,
};

static const struct sdhci_msm_variant_info sdhci_msm_mci_var = {
	.var_ops = &mci_var_ops,
	.offset = &sdhci_msm_mci_offset,
};

static const struct sdhci_msm_variant_info sdhci_msm_v5_var = {
	.mci_removed = true,
	.var_ops = &v5_var_ops,
	.offset = &sdhci_msm_v5_offset,
};

static const struct sdhci_msm_variant_info sdhci_msm_fmr_v5_var = {
	.mci_removed = true,
	.var_ops = &v5_var_ops,
	.offset = &sdhci_msm_v5_offset,
	.pd_data = {
		.pd_flags = PD_FLAG_DEV_LINK_ON,
		.pd_names = (const char * []) { "power", "perf" },
		.num_pd_names = 2,
	},
};

static const struct sdhci_msm_variant_info sdm845_sdhci_var = {
	.mci_removed = true,
	.restore_dll_config = true,
	.var_ops = &v5_var_ops,
	.offset = &sdhci_msm_v5_offset,
};

static const struct of_device_id sdhci_msm_dt_match[] = {
	/*
	 * Do not add new variants to the driver which are compatible with
	 * generic ones, unless they need customization.
	 */
	{.compatible = "qcom,sdhci-msm-v4", .data = &sdhci_msm_mci_var},
	{.compatible = "qcom,sdhci-msm-v5", .data = &sdhci_msm_v5_var},
	{.compatible = "qcom,sdm670-sdhci", .data = &sdm845_sdhci_var},
	{.compatible = "qcom,sdm845-sdhci", .data = &sdm845_sdhci_var},
	{.compatible = "qcom,sa7255-sdhci", .data = &sdhci_msm_fmr_v5_var},
	{},
};

MODULE_DEVICE_TABLE(of, sdhci_msm_dt_match);

static int sdhci_msm_gcc_reset(struct device *dev, struct sdhci_host *host)
{

	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct reset_control *reset = msm_host->core_reset;
	int ret = -EOPNOTSUPP;

	if (!reset)
		return dev_err_probe(dev, ret, "unable to acquire core_reset\n");

	ret = reset_control_assert(reset);
	if (ret)
		return dev_err_probe(dev, ret, "core_reset assert failed\n");

	/*
	 * The hardware requirement for delay between assert/deassert
	 * is at least 3-4 sleep clock (32.7KHz) cycles, which comes to
	 * ~125us (4/32768). To be on the safe side add 200us delay.
	 */
	usleep_range(200, 210);

	ret = reset_control_deassert(reset);
	if (ret)
		return dev_err_probe(dev, ret, "core_reset deassert failed\n");

	usleep_range(200, 210);

	return ret;
}

static void sdhci_msm_hw_reset(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct platform_device *pdev = msm_host->pdev;
	int ret = -EOPNOTSUPP;

	if (!msm_host->core_reset) {
		dev_err(&pdev->dev, "%s: failed, err = %d\n", __func__,
				ret);
		return;
	}

	msm_host->reg_store = true;
	sdhci_msm_registers_save(host);
	if (host->mmc->caps2 & MMC_CAP2_CQE) {
		host->mmc->cqe_ops->cqe_disable(host->mmc);
		host->mmc->cqe_enabled = false;
	}

	sdhci_msm_gcc_reset(&pdev->dev, host);
	sdhci_msm_registers_restore(host);
	msm_host->reg_store = false;

	sdhci_msm_log_str(msm_host, "HW reset done\n");
#if defined(CONFIG_SDC_QTI)
	if (host->mmc->card)
		mmc_power_cycle(host->mmc, host->mmc->card->ocr);
#endif
}

static const struct sdhci_ops sdhci_msm_ops = {
	.reset = sdhci_msm_reset,
	.set_clock = sdhci_msm_set_clock,
	.get_min_clock = sdhci_msm_get_min_clock,
	.get_max_clock = sdhci_msm_get_max_clock,
	.set_bus_width = sdhci_set_bus_width,
	.set_uhs_signaling = sdhci_msm_set_uhs_signaling,
	.get_max_timeout_count = sdhci_msm_get_max_timeout_count,
	.write_w = sdhci_msm_writew,
	.write_b = sdhci_msm_writeb,
	.irq	= sdhci_msm_cqe_irq,
	.dump_vendor_regs = sdhci_msm_dump_vendor_regs,
	.set_power = sdhci_set_power_noreg,
	.hw_reset = sdhci_msm_hw_reset,
	.set_timeout = sdhci_msm_set_timeout,
};

static const struct sdhci_ops sdhci_msm_fmr_ops = {
	.reset = sdhci_and_cqhci_reset,
	.set_clock = sdhci_msm_fmr_set_clock,
	.get_min_clock = sdhci_msm_get_min_clock,
	.get_max_clock = sdhci_msm_fmr_get_max_clock,
	.set_bus_width = sdhci_set_bus_width,
	.set_uhs_signaling = sdhci_msm_set_uhs_signaling,
	.write_w = sdhci_msm_writew,
	.write_b = sdhci_msm_writeb,
	.irq	= sdhci_msm_cqe_irq,
	.dump_vendor_regs = sdhci_msm_dump_vendor_regs,
	.set_power = sdhci_set_power_noreg,
	.set_timeout = sdhci_msm_set_timeout,
};

static const struct sdhci_pltfm_data sdhci_msm_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_CARD_DETECTION |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12 |
		  SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK,

	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
	.ops = &sdhci_msm_ops,
};

static const struct sdhci_pltfm_data sdhci_msm_fmr_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_CARD_DETECTION |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12,

	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
	.ops = &sdhci_msm_fmr_ops,
};

static void sdhci_set_default_hw_caps(struct sdhci_msm_host *msm_host,
		struct sdhci_host *host)
{
	u32 config, version, caps = 0;
	u16 host_version, minor;
	u8 major;
	const struct sdhci_msm_offset *msm_offset =
					sdhci_priv_msm_offset(host);

	host_version = readw_relaxed((host->ioaddr + SDHCI_HOST_VERSION));
	dev_dbg(&msm_host->pdev->dev, "Host Version: 0x%x Vendor Version 0x%x\n",
		host_version, ((host_version & SDHCI_VENDOR_VER_MASK) >>
					SDHCI_VENDOR_VER_SHIFT));

	version =  msm_host_readl(msm_host, host,
				 msm_offset->core_mci_version);

	major = (version & CORE_VERSION_MAJOR_MASK) >>
			CORE_VERSION_MAJOR_SHIFT;
	minor = version & CORE_VERSION_TARGET_MASK;

	dev_dbg(&msm_host->pdev->dev, "MCI Version: 0x%08x, major: 0x%04x, minor: 0x%02x\n",
		version, major, minor);

	caps = readl_relaxed(host->ioaddr + SDHCI_CAPABILITIES);

	/*
	 * SDCC 5 controller with major version 1, minor version 0x34 and later
	 * with HS 400 mode support will use CM DLL instead of CDC LP 533 DLL.
	 */
	if ((major == 1) && (minor < 0x34))
		msm_host->use_cdclp533 = true;

	if (major == 1 && minor >= 0x42)
		msm_host->use_14lpp_dll_reset = true;

	/* Fake 3.0V support for SDIO devices which requires such voltage */
	if (msm_host->fake_core_3_0v_support) {
		caps |= CORE_3_0V_SUPPORT;
			writel_relaxed((readl_relaxed(host->ioaddr +
			SDHCI_CAPABILITIES) | caps), host->ioaddr +
			msm_offset->core_vendor_spec_capabilities0);
	}

	writel_relaxed(caps, host->ioaddr +
		msm_offset->core_vendor_spec_capabilities0);

	/* keep track of the value in SDHCI_CAPABILITIES */
	msm_host->caps_0 = caps;

	/* 7FF projects with 7nm DLL */
	if ((major == 1) && ((minor == 0x6e) || (minor == 0x71) ||
				(minor == 0x72)))
		msm_host->use_7nm_dll = true;

	/*
	 * Support for some capabilities is not advertised by newer
	 * controller versions and must be explicitly enabled.
	 */
	if (major >= 1 && minor != 0x11 && minor != 0x12) {
		config = readl_relaxed(host->ioaddr + SDHCI_CAPABILITIES);
		config |= SDHCI_CAN_VDD_300 | SDHCI_CAN_DO_8BIT;
		writel_relaxed(config, host->ioaddr +
				msm_offset->core_vendor_spec_capabilities0);
	}

	if (major == 1 && minor >= 0x49)
		msm_host->updated_ddr_cfg = true;

	/* For SDHC v5.0.0 onwards, ICE 3.0 specific registers are added
	 * in CQ register space, due to which few CQ registers are
	 * shifted. Set cqhci_offset_changed boolean to use updated address.
	 */
	if (major == 1 && minor >= 0x6B)
		msm_host->cqhci_offset_changed = true;

	if (major == 1 && minor >= 0x71)
		msm_host->uses_tassadar_dll = true;

	/* For SDHC v5.0.0 onwards, ICE 3.0 specific registers are added
	 * in CQ register space, due to which few CQ registers are
	 * shifted. Set cqhci_offset_changed boolean to use updated address.
	 */
	if (major == 1 && minor >= 0x6B)
		msm_host->cqhci_offset_changed = true;
}

static void sdhci_enable_ext_fb_clk(struct sdhci_host *host)
{
	u32 fb_clk;

	fb_clk = sdhci_readl(host, VENDOR_SPEC_FUNC4);
	fb_clk |= EXTERN_FB_CLK;
	sdhci_writel(host, fb_clk, VENDOR_SPEC_FUNC4);
}

static inline void sdhci_msm_get_of_property(struct platform_device *pdev,
		struct sdhci_host *host)
{
	struct device_node *node = pdev->dev.of_node;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	if (of_property_read_u32(node, "qcom,ddr-config",
				&msm_host->ddr_config))
		msm_host->ddr_config = DDR_CONFIG_POR_VAL;

	of_property_read_u32(node, "qcom,dll-config", &msm_host->dll_config);

	if (of_device_is_compatible(node, "qcom,msm8916-sdhci"))
		host->quirks2 |= SDHCI_QUIRK2_BROKEN_64_BIT_DMA;
}

/* Find cpu group qos from a given cpu */
static struct qos_cpu_group *cpu_to_group(struct sdhci_msm_qos_req *r, int cpu)
{
	int i;
	struct qos_cpu_group *g = r->qcg;

	if (cpu < 0 || cpu > num_possible_cpus())
		return NULL;

	for (i = 0; i < r->num_groups; i++, g++) {
		if (cpumask_test_cpu(cpu, &g->mask))
			return &r->qcg[i];
	}

	return NULL;
}

/*
 * Function to put qos vote. This takes qos cpu group of
 * host and type of vote as input
 */
static int sdhci_msm_update_qos_constraints(struct qos_cpu_group *qcg,
							enum constraint type)
{
	unsigned int vote;
	int cpu, err;
	struct dev_pm_qos_request *qos_req = qcg->qos_req;

	if (type == QOS_MAX)
		vote = S32_MAX;
	else
		vote = qcg->votes[type];

	if (qcg->curr_vote == vote)
		return 0;

	sdhci_msm_log_str(qcg->host, "mask: 0x%lx type: %d vote: %u\n",
			qcg->mask.bits[0], type, vote);

	for_each_cpu(cpu, &qcg->mask) {
		err = dev_pm_qos_update_request(qos_req, vote);
		if (err < 0)
			return err;
		++qos_req;
	}

	if (type == QOS_MAX)
		qcg->voted = false;
	else
		qcg->voted = true;

	qcg->curr_vote = vote;

	return 0;
}

/* Unregister pm qos requests */
static int remove_group_qos(struct qos_cpu_group *qcg)
{
	int err, cpu;
	struct dev_pm_qos_request *qos_req = qcg->qos_req;

	for_each_cpu(cpu, &qcg->mask) {
		if (!dev_pm_qos_request_active(qos_req)) {
			++qos_req;
			continue;
		}
		err = dev_pm_qos_remove_request(qos_req);
		if (err < 0)
			return err;
		qos_req++;
	}

	return 0;
}

/* Register pm qos request */
static int add_group_qos(struct qos_cpu_group *qcg, enum constraint type)
{
	int cpu, err;
	struct dev_pm_qos_request *qos_req = qcg->qos_req;

	for_each_cpu(cpu, &qcg->mask) {
		memset(qos_req, 0,
				sizeof(struct dev_pm_qos_request));
		err = dev_pm_qos_add_request(get_cpu_device(cpu),
				qos_req,
				DEV_PM_QOS_RESUME_LATENCY,
				type);
		if (err < 0)
			return err;
		qos_req++;
	}
	return 0;
}

/* Function to remove pm qos vote */
static void sdhci_msm_unvote_qos_all(struct sdhci_msm_host *msm_host)
{
	struct sdhci_msm_qos_req *qos_req = msm_host->sdhci_qos;
	struct qos_cpu_group *qcg;
	int i, err;

	if (!qos_req)
		return;
	qcg = qos_req->qcg;
	for (i = 0; ((i < qos_req->num_groups) && qcg->initialized); i++,
								qcg++) {
		err = sdhci_msm_update_qos_constraints(qcg, QOS_MAX);
		if (err)
			dev_err(&msm_host->pdev->dev,
				"Failed (%d) removing qos vote(%d)\n", err, i);
	}
}

/* Function to vote pmqos from sdcc. */
static void sdhci_msm_vote_pmqos(struct mmc_host *mmc, int cpu)
{
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct qos_cpu_group *qcg;

	if (!msm_host->sdhci_qos) {
		dev_dbg(&msm_host->pdev->dev, "QoS instance is uninitialized\n");
		return;
	}

	qcg = cpu_to_group(msm_host->sdhci_qos, cpu);
	if (!qcg) {
		dev_dbg(&msm_host->pdev->dev, "QoS group is undefined\n");
		return;
	}

	if (qcg->voted)
		return;

	if (sdhci_msm_update_qos_constraints(qcg, QOS_PERF))
		dev_err(&qcg->host->pdev->dev, "%s: update qos - failed\n",
				__func__);
	dev_dbg(&msm_host->pdev->dev, "Voted pmqos - cpu: %d\n", cpu);
}

static unsigned int sdhci_msm_get_sup_clk_rate(struct sdhci_host *host,
						u32 req_clk)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct clk *core_clk = msm_host->bulk_clks[0].clk;
	unsigned int sup_clk = -1;

	if (req_clk < sdhci_msm_get_min_clock(host)) {
		sup_clk = sdhci_msm_get_min_clock(host);
		return sup_clk;
	}

	sup_clk = clk_round_rate(core_clk, clk_get_rate(core_clk));

	if (host->clock != msm_host->clk_rate)
		sup_clk = sup_clk / 2;

	return sup_clk;
}

/**
 * sdhci_msm_irq_affinity_notify - Callback for affinity changes
 * @notify: context as to what irq was changed
 * @mask: the new affinity mask
 *
 * This is a callback function used by the irq_set_affinity_notifier function
 * so that we may register to receive changes to the irq affinity masks.
 */
static void
sdhci_msm_irq_affinity_notify(struct irq_affinity_notify *notify,
		const cpumask_t *mask)
{
	struct sdhci_msm_host *msm_host =
		container_of(notify, struct sdhci_msm_host, affinity_notify);
	struct platform_device *pdev = msm_host->pdev;
	struct sdhci_msm_qos_req *qos_req = msm_host->sdhci_qos;
	struct qos_cpu_group *qcg;
	int i, err;

	if (!qos_req)
		return;
	/*
	 * If device is in suspend mode, just update the active mask,
	 * vote would be taken care when device resumes.
	 */
	msm_host->sdhci_qos->active_mask = cpumask_first(mask);
	if (pm_runtime_status_suspended(&pdev->dev))
		return;

	/* Cancel previous scheduled work and unvote votes */
	qcg = qos_req->qcg;
	for (i = 0; i < qos_req->num_groups; i++, qcg++) {
		err = sdhci_msm_update_qos_constraints(qcg, QOS_MAX);
		if (err)
			pr_err("%s: Failed (%d) removing qos vote of grp(%d)\n",
					mmc_hostname(msm_host->mmc), err, i);
	}

	sdhci_msm_vote_pmqos(msm_host->mmc,
			msm_host->sdhci_qos->active_mask);
}

/**
 * sdhci_msm_irq_affinity_release - Callback for affinity notifier release
 * @ref: internal core kernel usage
 *
 * This is a callback function used by the irq_set_affinity_notifier function
 * to inform the current notification subscriber that they will no longer
 * receive notifications.
 */
static void
inline sdhci_msm_irq_affinity_release(struct kref __always_unused *ref)
{ }

/* Function for settig up qos based on parsed dt entries */
static int sdhci_msm_setup_qos(struct sdhci_msm_host *msm_host)
{
	struct platform_device *pdev = msm_host->pdev;
	struct sdhci_msm_qos_req *qr = msm_host->sdhci_qos;
	struct qos_cpu_group *qcg = NULL;
	struct mmc_host *mmc = msm_host->mmc;
	struct sdhci_host *host = mmc_priv(mmc);
	int i, err;

	if (!msm_host->sdhci_qos)
		return 0;

	qcg = qr->qcg;

	/* Affine irq to first set of mask */
	WARN_ON(irq_set_affinity_hint(host->irq, &qcg->mask));

	/* Setup notifier for case of affinity change/migration */
	msm_host->affinity_notify.notify = sdhci_msm_irq_affinity_notify;
	msm_host->affinity_notify.release = sdhci_msm_irq_affinity_release;
	irq_set_affinity_notifier(host->irq, &msm_host->affinity_notify);

	for (i = 0; i < qr->num_groups; i++, qcg++) {
		qcg->qos_req = kcalloc(cpumask_weight(&qcg->mask),
				sizeof(struct dev_pm_qos_request),
				GFP_KERNEL);
		if (!qcg->qos_req) {
			dev_err(&pdev->dev, "Memory allocation failed\n");
			if (!i)
				return -ENOMEM;
			goto free_mem;
		}
		err = add_group_qos(qcg, S32_MAX);
		if (err < 0) {
			dev_err(&pdev->dev, "Fail (%d) add qos-req: grp-%d\n",
					err, i);
			if (!i) {
				kfree(qcg->qos_req);
				return err;
			}
			goto free_mem;
		}
		qcg->initialized = true;
		dev_dbg(&pdev->dev, "%s: qcg: %p | mask: 0x%lx\n",
				 __func__, qcg, qcg->mask.bits[0]);
	}

	/* Vote pmqos during setup for first set of mask*/
	sdhci_msm_update_qos_constraints(qr->qcg, QOS_PERF);
	qr->active_mask = cpumask_first(&qr->qcg->mask);
	return 0;

free_mem:
	while (i--) {
		kfree(qcg->qos_req);
		qcg--;
	}

	return err;
}

/*
 * QoS init function. It parses dt entries and intializes data
 * structures.
 */
static void sdhci_msm_qos_init(struct sdhci_msm_host *msm_host)
{
	struct platform_device *pdev = msm_host->pdev;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *group_node;
	struct sdhci_msm_qos_req *qr;
	struct qos_cpu_group *qcg;
	int i, err, mask = 0;

	qr = kzalloc(sizeof(*qr), GFP_KERNEL);
	if (!qr)
		return;

	msm_host->sdhci_qos = qr;

	/* find numbers of qos child node present */
	qr->num_groups = of_get_available_child_count(np);
	dev_dbg(&pdev->dev, "num-groups: %d\n", qr->num_groups);
	if (!qr->num_groups) {
		dev_err(&pdev->dev, "QoS groups undefined\n");
		kfree(qr);
		msm_host->sdhci_qos = NULL;
		return;
	}
	qcg = kzalloc(sizeof(*qcg) * qr->num_groups, GFP_KERNEL);
	if (!qcg) {
		msm_host->sdhci_qos = NULL;
		kfree(qr);
		return;
	}

	/*
	 * Assign qos cpu group/cluster to host qos request and
	 * read child entries of qos node
	 */
	qr->qcg = qcg;
	for_each_available_child_of_node(np, group_node) {
		err = of_property_read_u32(group_node, "mask", &mask);
		if (err) {
			dev_dbg(&pdev->dev, "Error reading group mask: %d\n",
					err);
			continue;
		}

		qcg->mask.bits[0] = mask & cpu_possible_mask->bits[0];
		if (!qcg->mask.bits[0]) {
			dev_err(&pdev->dev, "Invalid group mask, use default\n");
			qcg->mask.bits[0] = cpu_possible_mask->bits[0];
		}

		err = of_property_count_u32_elems(group_node, "vote");
		if (err <= 0) {
			dev_err(&pdev->dev, "1 vote is needed, bailing out\n");
			goto out_vote_err;
		}
		qcg->votes = kmalloc(sizeof(*qcg->votes) * err, GFP_KERNEL);
		if (!qcg->votes)
			goto out_vote_err;
		for (i = 0; i < err; i++) {
			if (of_property_read_u32_index(group_node, "vote", i,
						&qcg->votes[i]))
				goto out_vote_err;
		}
		qcg->host = msm_host;
		++qcg;
	}
	err = sdhci_msm_setup_qos(msm_host);
	if (!err)
		return;
	dev_err(&pdev->dev, "Failed to setup PM QoS.\n");

out_vote_err:
	for (i = 0, qcg = qr->qcg; i < qr->num_groups; i++, qcg++)
		kfree(qcg->votes);
	kfree(qr->qcg);
	kfree(qr);
	msm_host->sdhci_qos = NULL;
}

static ssize_t dbg_state_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	bool v;

	if (!capable(CAP_SYS_ADMIN))
		return -EACCES;

	if (kstrtobool(buf, &v))
		return -EINVAL;

	msm_host->dbg_en = v;

	return count;
}

static ssize_t dbg_state_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

#if defined(CONFIG_SDHCI_MSM_DBG)
	msm_host->dbg_en = true;
#endif

	return scnprintf(buf, PAGE_SIZE, "%d\n", msm_host->dbg_en);
}

static DEVICE_ATTR_RW(dbg_state);

static struct attribute *sdhci_msm_sysfs_attrs[] = {
	&dev_attr_dbg_state.attr,
	NULL
};

static const struct attribute_group sdhci_msm_sysfs_group = {
	.name = "qcom",
	.attrs = sdhci_msm_sysfs_attrs,
};

static int sdhci_msm_init_sysfs(struct device *dev)
{
	int ret;

	ret = sysfs_create_group(&dev->kobj, &sdhci_msm_sysfs_group);
	if (ret)
		dev_err(dev, "%s: Failed to create qcom sysfs group (err = %d)\n",
				 __func__, ret);

	return ret;
}

static ssize_t show_sdhci_msm_pm_qos(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	return scnprintf(buf, PAGE_SIZE, "%u\n", msm_host->pm_qos_delay);
}

static ssize_t store_sdhci_msm_pm_qos(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	uint32_t value;

	if (!kstrtou32(buf, 0, &value)) {
		msm_host->pm_qos_delay = value;
		dev_info(dev, "set pm qos work delay (%u)\n", value);
	}

	return count;
}

static void sdhci_msm_init_sysfs_gating_qos(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	int ret;

	msm_host->pm_qos.show = show_sdhci_msm_pm_qos;
	msm_host->pm_qos.store = store_sdhci_msm_pm_qos;
	sysfs_attr_init(&msm_host->pm_qos.attr);
	msm_host->pm_qos.attr.name = "pm_qos";
	msm_host->pm_qos.attr.mode = 0644;
	ret = device_create_file(dev, &msm_host->pm_qos);
	if (ret) {
		pr_err("%s: %s: failed creating pm qos attr: %d\n",
				mmc_hostname(host->mmc), __func__, ret);
	}
}

static void sdhci_msm_setup_pm(struct platform_device *pdev,
			struct sdhci_msm_host *msm_host)
{
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	if (!(msm_host->mmc->caps & MMC_CAP_SYNC_RUNTIME_PM)) {
		pm_runtime_set_autosuspend_delay(&pdev->dev,
					 MSM_MMC_AUTOSUSPEND_DELAY_MS);
		pm_runtime_use_autosuspend(&pdev->dev);
	}
}

static int sdhci_msm_setup_ice_clk(struct sdhci_msm_host *msm_host,
						struct platform_device *pdev)
{
	int ret = 0;
	struct clk *clk;

	msm_host->bulk_clks[2].clk = NULL;

	/* Setup SDC ICE clock */
	clk = devm_clk_get(&pdev->dev, "ice_core");
	if (!IS_ERR(clk)) {
		msm_host->bulk_clks[2].clk = clk;

		/* Set maximum clock rate for ice clk */
		ret = clk_set_rate(clk, msm_host->ice_clk_max);
		if (ret) {
			dev_err(&pdev->dev, "ICE_CLK rate set failed (%d) for %u\n",
				ret, msm_host->ice_clk_max);
			return ret;
		}

		msm_host->ice_clk_rate = msm_host->ice_clk_max;
	}

	return ret;
}

static void sdhci_msm_set_caps(struct sdhci_msm_host *msm_host)
{
	msm_host->mmc->caps |= MMC_CAP_AGGRESSIVE_PM;
	msm_host->mmc->caps |= MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_NEED_RSP_BUSY;
}
/* RUMI W/A for SD card */
static void sdhci_msm_set_rumi_bus_mode(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	const struct sdhci_msm_offset *msm_offset = msm_host->offset;
	u32 config = readl_relaxed(host->ioaddr +
			msm_offset->core_vendor_spec_capabilities1);

	if (!(host->mmc->caps & MMC_CAP_NONREMOVABLE)) {
		config &= ~(VS_CAPABILITIES_SDR_104_SUPPORT |
				VS_CAPABILITIES_DDR_50_SUPPORT |
				VS_CAPABILITIES_SDR_50_SUPPORT);
		writel_relaxed(config, host->ioaddr +
				msm_offset->core_vendor_spec_capabilities1);
	}
}

static bool is_bootdevice_sdhci = true;

static bool sdhci_qcom_read_boot_config(struct platform_device *pdev)
{
	u8 *buf;
	size_t len;
	struct nvmem_cell *cell;
	int boot_device_type, data;
	struct device *dev = &pdev->dev;

	cell = nvmem_cell_get(dev, "boot_conf");
	if (IS_ERR(cell)) {
		dev_warn(dev, "nvmem cell get failed\n");
		goto ret;
	}

	buf = (u8 *)nvmem_cell_read(cell, &len);
	if (IS_ERR(buf)) {
		dev_warn(dev, "nvmem cell read failed\n");
		goto ret_put_nvmem;
	}

	/*
	 * Boot_device_type value is passed from the dtsi node
	 */
	if (of_property_read_u32(dev->of_node, "boot_device_type",
				&boot_device_type)) {
		dev_warn(dev, "boot_device_type not present\n");
		goto ret_free_buf;
	}

	/*
	 * Storage boot device fuse is present in QFPROM_RAW_OEM_CONFIG_ROW0_LSB
	 * this fuse is blown by bootloader and pupulated in boot_config
	 * register[1:5] - hence shift read data by 1 and mask it with 0x1f.
	 */
	data = *buf >> 1 & 0x1f;

	/*
	 * The value in the boot_device_type in dtsi node should match with the
	 * value read from the register for the probe to continue.
	 */
	is_bootdevice_sdhci = (data == boot_device_type) ? true : false;

	if (!is_bootdevice_sdhci)
		dev_err(dev, "boot dev in reg = 0x%x boot dev in dtsi = 0x%x\n",
				data, boot_device_type);

ret_free_buf:
	kfree(buf);
ret_put_nvmem:
	nvmem_cell_put(cell);
ret:
	return is_bootdevice_sdhci;
}

static int sdhci_msm_setup_clocks_and_bus(struct sdhci_msm_host *msm_host)
{
	struct sdhci_host *host = mmc_priv(msm_host->mmc);
	struct platform_device *pdev = msm_host->pdev;
	struct clk *clk;
	int ret;

	/* Setup SDCC bus voter clock. */
	msm_host->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (!IS_ERR(msm_host->bus_clk)) {
		/* Vote for max. clk rate for max. performance */
		ret = clk_set_rate(msm_host->bus_clk, INT_MAX);
		if (ret)
			return ret;
		ret = clk_prepare_enable(msm_host->bus_clk);
		if (ret)
			return ret;
	}

	/* Setup main peripheral bus clock */
	clk = devm_clk_get(&pdev->dev, "iface");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(&pdev->dev, "Peripheral clk setup failed (%d)\n", ret);
		goto bus_clk_disable;
	}
	msm_host->bulk_clks[1].clk = clk;

	/* Setup SDC MMC clock */
	clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(&pdev->dev, "SDC MMC clk setup failed (%d)\n", ret);
		goto bus_clk_disable;
	}
	msm_host->bulk_clks[0].clk = clk;

	/* Check for optional interconnect paths */
	ret = dev_pm_opp_of_find_icc_paths(&pdev->dev, NULL);
	if (ret)
		goto bus_clk_disable;

	msm_host->opp_token = dev_pm_opp_set_clkname(&pdev->dev, "core");
	if (msm_host->opp_token <= 0) {
		ret = msm_host->opp_token;
		goto bus_clk_disable;
	}

	/* OPP table is optional */
	ret = dev_pm_opp_of_add_table(&pdev->dev);
	if (!ret) {
		msm_host->has_opp_table = true;
	} else if (ret != -ENODEV) {
		dev_err(&pdev->dev, "Invalid OPP table in Device tree\n");
		goto opp_cleanup;
	}

	/* Vote for maximum clock rate for maximum performance */
	ret = dev_pm_opp_set_rate(&pdev->dev, INT_MAX);
	if (ret)
		dev_warn(&pdev->dev, "core clock boost failed\n");

	ret = sdhci_msm_setup_ice_clk(msm_host, pdev);
	if (ret)
		goto bus_clk_disable;

	clk = devm_clk_get(&pdev->dev, "cal");
	if (IS_ERR(clk))
		clk = NULL;
	msm_host->bulk_clks[3].clk = clk;

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(msm_host->bulk_clks),
				      msm_host->bulk_clks);
	if (ret)
		goto opp_cleanup;
	ret = qcom_clk_set_flags(msm_host->bulk_clks[0].clk,
			CLKFLAG_NORETAIN_MEM);
	if (ret)
		dev_err(&pdev->dev, "Failed to set core clk NORETAIN_MEM: %d\n",
				ret);
	ret = qcom_clk_set_flags(msm_host->bulk_clks[2].clk,
			CLKFLAG_RETAIN_MEM);
	if (ret)
		dev_err(&pdev->dev, "Failed to set ice clk RETAIN_MEM: %d\n",
				ret);
	/*
	 * xo clock is needed for FLL feature of cm_dll.
	 * In case if xo clock is not mentioned in DT, warn and proceed.
	 */
	msm_host->xo_clk = devm_clk_get(&pdev->dev, "xo");
	if (IS_ERR(msm_host->xo_clk)) {
		ret = PTR_ERR(msm_host->xo_clk);
		dev_warn(&pdev->dev, "TCXO clk not present (%d)\n", ret);
	}

	ret = sdhci_msm_bus_register(msm_host, pdev);
	if (ret && !msm_host->skip_bus_bw_voting) {
		dev_err(&pdev->dev, "Bus registration failed (%d)\n", ret);
		goto clk_disable;
	}

	if (!msm_host->skip_bus_bw_voting)
		sdhci_msm_bus_voting(host, true);

	/*
	 * The flag enable_ext_fb_clk is true means select external
	 * FB clock while false means select internal FB clock.
	 */
	if (msm_host->enable_ext_fb_clk)
		sdhci_enable_ext_fb_clk(host);

	return 0;

clk_disable:
	clk_bulk_disable_unprepare(ARRAY_SIZE(msm_host->bulk_clks),
				   msm_host->bulk_clks);
opp_cleanup:
	if (msm_host->has_opp_table)
		dev_pm_opp_of_remove_table(&pdev->dev);
	dev_pm_opp_put_clkname(msm_host->opp_token);
bus_clk_disable:
	if (!IS_ERR(msm_host->bus_clk))
		clk_disable_unprepare(msm_host->bus_clk);

	return ret;

}

static int sdhci_msm_setup_pwr_irq(struct sdhci_msm_host *msm_host)
{
	struct sdhci_host *host = mmc_priv(msm_host->mmc);
	struct platform_device *pdev = msm_host->pdev;
	int ret;

	/*
	 * Power on reset state may trigger power irq if previous status of
	 * PWRCTL was either BUS_ON or IO_HIGH_V. So before enabling pwr irq
	 * interrupt in GIC, any pending power irq interrupt should be
	 * acknowledged. Otherwise power irq interrupt handler would be
	 * fired prematurely.
	 */
	sdhci_msm_handle_pwr_irq(host, 0);

	/*
	 * Ensure that above writes are propogated before interrupt enablement
	 * in GIC.
	 */
	mb();

	/* Setup IRQ for handling power/voltage tasks with PMIC */
	msm_host->pwr_irq = platform_get_irq_byname(pdev, "pwr_irq");
	if (msm_host->pwr_irq < 0) {
		ret = msm_host->pwr_irq;
		return ret;
	}

	sdhci_msm_init_pwr_irq_wait(msm_host);
	/* Enable pwr irq interrupts */
	msm_host_writel(msm_host, INT_MASK, host,
			msm_host->offset->core_pwrctl_mask);

	ret = devm_request_threaded_irq(&pdev->dev, msm_host->pwr_irq, NULL,
					sdhci_msm_pwr_irq, IRQF_ONESHOT,
					dev_name(&pdev->dev), host);
	if (ret) {
		dev_err(&pdev->dev, "Request IRQ failed (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int sdhci_msm_parse_sdio(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dep_np;
	struct platform_device *dep_pdev;

	dep_np = of_parse_phandle(dev->of_node, "sdhc-msm-sdio", 0);
	if (!dep_np)
		return 0;

	dep_pdev = of_find_device_by_node(dep_np);
	of_node_put(dep_np);

	if (!dep_pdev || !dep_pdev->dev.driver || !dep_pdev->dev.driver_data) {
		dev_warn(dev, "SDIO dependent driver probe not complete\n");
		if (dep_pdev)
			platform_device_put(dep_pdev);
		return -EPROBE_DEFER;
	}

	platform_device_put(dep_pdev);
	return 0;
}

static void sdhci_msm_parse_pinctrl(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	msm_host->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(msm_host->pinctrl)) {
		dev_dbg(dev, "Failed to get pinctrl\n");
		msm_host->pinctrl = NULL;
		return;
	}

	msm_host->pinctrl_state_default =
		pinctrl_lookup_state(msm_host->pinctrl, "default");
	if (IS_ERR(msm_host->pinctrl_state_default)) {
		dev_dbg(dev, "Failed to lookup default state\n");
		msm_host->pinctrl_state_default = NULL;
	}

	msm_host->pinctrl_state_gpio =
		pinctrl_lookup_state(msm_host->pinctrl, "gpio");
	if (IS_ERR(msm_host->pinctrl_state_gpio)) {
		dev_dbg(dev, "Failed to lookup gpio state\n");
		msm_host->pinctrl_state_gpio = NULL;
	}
}

static int sdhci_msm_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_msm_host *msm_host;
	int ret;
	u32 config;
	const struct sdhci_msm_offset *msm_offset;
	const struct sdhci_msm_variant_info *var_info;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	bool fw_managed;

	if (of_property_read_bool(node, "non-removable") && !sdhci_qcom_read_boot_config(pdev)) {
		dev_err(dev, "SDHCI is not boot dev.\n");
		return 0;
	}

	ret = sdhci_msm_parse_sdio(pdev);
	if (ret)
		return ret;

	var_info = of_device_get_match_data(&pdev->dev);
	if (!var_info) {
		dev_err(&pdev->dev, "Compatible string not found\n");
		return -ENODEV;
	}
	fw_managed = var_info->pd_data.pd_names != NULL;

	/*
	 * Based on variant info, differentiate platform initialization
	 * for local and remote managed resource platforms.
	 */
	if (fw_managed)
		host = sdhci_pltfm_init(pdev, &sdhci_msm_fmr_pdata, sizeof(*msm_host));
	else
		host = sdhci_pltfm_init(pdev, &sdhci_msm_pdata, sizeof(*msm_host));
	if (IS_ERR(host))
		return PTR_ERR(host);

	host->sdma_boundary = 0;
	pltfm_host = sdhci_priv(host);
	msm_host = sdhci_pltfm_priv(pltfm_host);
	msm_host->mmc = host->mmc;
	msm_host->pdev = pdev;

	sdhci_msm_init_sysfs(dev);
#if defined(CONFIG_SDHCI_MSM_DBG)
	msm_host->dbg_en = true;
#endif

	msm_host->sdhci_msm_ipc_log_ctx = ipc_log_context_create(SDHCI_MSM_MAX_LOG_SZ,
							dev_name(&host->mmc->class_dev), 0);
	if (!msm_host->sdhci_msm_ipc_log_ctx)
		dev_warn(dev, "IPC Log init - failed\n");
	/**
	 * System resume triggers a card detect interrupt even when there's no
	 * card inserted. Core layer acquires the registered wakeup source for
	 * 5s thus preventing system suspend for 5s at least.
	 * Disable this wakesource until this is sorted out.
	 */
	if ((host->mmc->ws) && !(host->mmc->caps & MMC_CAP_NONREMOVABLE)) {
		wakeup_source_unregister(host->mmc->ws);
		host->mmc->ws = NULL;
	}

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto pltfm_free;

	if (pdev->dev.of_node) {
		ret = of_alias_get_id(pdev->dev.of_node, "mmc");
		if (ret < 0)
			dev_err(&pdev->dev, "get slot index failed %d\n", ret);
		else if (ret <= 2)
			sdhci_slot[ret] = msm_host;
	}

	/*
	 * Based on the compatible string, load the required msm host info from
	 * the data associated with the version info.
	 */
	msm_host->mci_removed = var_info->mci_removed;
	msm_host->restore_dll_config = var_info->restore_dll_config;
	msm_host->var_ops = var_info->var_ops;
	msm_host->offset = var_info->offset;
	msm_host->fw_managed = fw_managed;

	msm_offset = msm_host->offset;

	sdhci_get_of_property(pdev);
	sdhci_msm_get_of_property(pdev, host);

	msm_host->saved_tuning_phase = INVALID_TUNING_PHASE;

	if (msm_host->fw_managed) {
		ret = dev_pm_domain_attach_list(&pdev->dev, &var_info->pd_data,
				&msm_host->pd_list);
		if (ret < 0) {
			dev_err(&pdev->dev, "Domain attach failed with err=%d\n", ret);
			goto pltfm_free;
		}

		ret = sdhci_msm_fmr_power_state(&pdev->dev, true);
		if (ret) {
			dev_err(&pdev->dev, "sdhci_msm_power_state failed with err=%d\n", ret);
			dev_pm_domain_detach_list(msm_host->pd_list);
			goto pltfm_free;
		}

		/*
		 * Skip initializing clocks, regulators and reset in kernel
		 * for firmware managed platforms.
		 */
		goto skip_resource_init;
	}

	ret = sdhci_msm_populate_pdata(dev, msm_host);
	if (ret) {
		dev_err(&pdev->dev, "DT parsing error\n");
		goto pltfm_free;
	}

	sdhci_msm_gcc_reset(&pdev->dev, host);
	msm_host->regs_restore.is_supported =
		of_property_read_bool(dev->of_node,
			"qcom,restore-after-cx-collapse");

	ret = sdhci_msm_setup_clocks_and_bus(msm_host);
	if (ret)
		goto pltfm_free;

	/* Setup regulators */
	ret = sdhci_msm_vreg_init(&pdev->dev, msm_host, true);
	if (ret) {
		dev_err(&pdev->dev, "Regulator setup failed (%d)\n", ret);
		goto bus_clk_deinit;
	}

	if (!msm_host->mci_removed) {
		msm_host->core_mem = devm_platform_ioremap_resource(pdev, 1);
		if (IS_ERR(msm_host->core_mem)) {
			ret = PTR_ERR(msm_host->core_mem);
			goto vreg_deinit;
		}
	}

skip_resource_init:
	/* Reset the vendor spec register to power on reset state */
	writel_relaxed(CORE_VENDOR_SPEC_POR_VAL,
			host->ioaddr + msm_offset->core_vendor_spec);

	/* Ensure SDHCI FIFO is enabled by disabling alternative FIFO */
	config = readl_relaxed(host->ioaddr + msm_offset->core_vendor_spec3);
	config &= ~CORE_FIFO_ALT_EN;
	writel_relaxed(config, host->ioaddr + msm_offset->core_vendor_spec3);

	if (!msm_host->mci_removed) {
		/* Set HC_MODE_EN bit in HC_MODE register */
		msm_host_writel(msm_host, HC_MODE_EN, host,
				msm_offset->core_hc_mode);
		config = msm_host_readl(msm_host, host,
				msm_offset->core_hc_mode);
		config |= FF_CLK_SW_RST_DIS;
		msm_host_writel(msm_host, config, host,
				msm_offset->core_hc_mode);
	}

	if (of_property_read_bool(node, "is_rumi"))
		sdhci_msm_set_rumi_bus_mode(host);

	sdhci_set_default_hw_caps(msm_host, host);

	ret = sdhci_msm_register_vreg(msm_host);
	if (ret)
		goto vreg_deinit;

	ret = sdhci_msm_setup_pwr_irq(msm_host);
	if (ret)
		goto vreg_deinit;

	sdhci_msm_set_caps(msm_host);

	sdhci_msm_setup_pm(pdev, msm_host);

	host->mmc_host_ops.start_signal_voltage_switch =
		sdhci_msm_start_signal_voltage_switch;
	host->mmc_host_ops.execute_tuning = sdhci_msm_execute_tuning;

	if (of_device_is_compatible(node, "lenovo,bh201-sdhci-v1")) {
		pr_info("BH201 patch is applied for hostname: %s, devname: %s\n",
			mmc_hostname(host->mmc), dev_name(dev));
		sdhci_bh201_parse(host->mmc);
		host->mmc_host_ops.execute_tuning = sdhci_bht_execute_tuning;
		host->mmc_host_ops.init_card = bht_load;
	}

	msm_host->workq = create_workqueue("sdhci_msm_generic_swq");
	if (!msm_host->workq)
		dev_err(&pdev->dev, "Generic swq creation failed\n");

	msm_host->pm_qos_delay = MSM_MMC_AUTOSUSPEND_DELAY_MS;
	/* Initialize pmqos */
	sdhci_msm_qos_init(msm_host);
	/* Initialize sysfs entries */
	sdhci_msm_init_sysfs_gating_qos(dev);

	if (of_property_read_bool(node, "supports-cqe"))
		ret = sdhci_msm_cqe_add_host(host, pdev);
	else
		ret = sdhci_add_host(host);
	if (ret)
		goto pm_runtime_disable;

	if (msm_host->ggc.bh201_used) {
		ret = sdhci_bh201_create_sysfs(host->mmc);
		if (ret)
			pr_info("Failed to create BH201 sysfs interface: %d\n", ret);
	}

	sdhci_msm_parse_pinctrl(dev);

	/*
	 * Set platfm_init_done only after sdhci_add_host().
	 * So that we don't turn off vqmmc while we reset sdhc as
	 * part of sdhci_add_host().
	 */
	msm_host->pltfm_init_done = true;

	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_autosuspend(&pdev->dev);

	return 0;

pm_runtime_disable:
	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

vreg_deinit:
	sdhci_msm_vreg_init(&pdev->dev, msm_host, false);
bus_clk_deinit:
	if (!msm_host->skip_bus_bw_voting) {
		sdhci_msm_bus_get_and_set_vote(host, 0);
		sdhci_msm_bus_unregister(&pdev->dev, msm_host);
	}
	clk_bulk_disable_unprepare(ARRAY_SIZE(msm_host->bulk_clks),
				   msm_host->bulk_clks);
	if (msm_host->has_opp_table)
		dev_pm_opp_of_remove_table(&pdev->dev);
	dev_pm_opp_put_clkname(msm_host->opp_token);
	if (!IS_ERR(msm_host->bus_clk))
		clk_disable_unprepare(msm_host->bus_clk);
pltfm_free:
	sdhci_pltfm_free(pdev);
	return ret;
}

static void sdhci_msm_remove(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_msm_host *msm_host;
	struct sdhci_msm_qos_req *r;
	struct qos_cpu_group *qcg;
	struct device_node *np = pdev->dev.of_node;
	int i;
	int dead;

	if (of_property_read_bool(np, "non-removable") &&
			!is_bootdevice_sdhci) {
		dev_err(&pdev->dev, "SDHCI is not boot dev.\n");
		return;
	}

	host = platform_get_drvdata(pdev);
	pltfm_host = sdhci_priv(host);
	msm_host = sdhci_pltfm_priv(pltfm_host);
	r = msm_host->sdhci_qos;

	dead = (readl_relaxed(host->ioaddr + SDHCI_INT_STATUS) ==
		    0xffffffff);

	if (msm_host->fw_managed) {
		sdhci_msm_fmr_power_state(&pdev->dev, false);
		dev_pm_domain_detach_list(msm_host->pd_list);
	}

	if (msm_host->ggc.bh201_used)
		sdhci_bh201_remove_sysfs(host->mmc);

	sdhci_remove_host(host, dead);

	sdhci_msm_vreg_init(&pdev->dev, msm_host, false);

	if (msm_host->has_opp_table)
		dev_pm_opp_of_remove_table(&pdev->dev);
	dev_pm_opp_put_clkname(msm_host->opp_token);
	pm_runtime_get_sync(&pdev->dev);

	/* Add delay to complete resume where qos vote is scheduled */
	if (!r)
		goto skip_removing_qos;
	qcg = r->qcg;
	msleep(50);
	for (i = 0; i < r->num_groups; i++, qcg++) {
		sdhci_msm_update_qos_constraints(qcg, QOS_MAX);
		remove_group_qos(qcg);
	}
	destroy_workqueue(msm_host->workq);

skip_removing_qos:
	pm_runtime_disable(&pdev->dev);
	pm_runtime_put_noidle(&pdev->dev);

	clk_bulk_disable_unprepare(ARRAY_SIZE(msm_host->bulk_clks),
				   msm_host->bulk_clks);
	if (!IS_ERR(msm_host->bus_clk))
		clk_disable_unprepare(msm_host->bus_clk);
	if (!msm_host->skip_bus_bw_voting) {
		sdhci_msm_bus_get_and_set_vote(host, 0);
		sdhci_msm_bus_unregister(&pdev->dev, msm_host);
	}
	sdhci_pltfm_free(pdev);
}

void sdhci_msm_toggle_dat1_gpio(struct sdio_func *func)
{
	struct sdhci_host *host = mmc_priv(func->card->host);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct device *dev = mmc_dev(host->mmc);
	int gpio_num;
	int ret;

	if (!msm_host->pinctrl || !msm_host->pinctrl_state_gpio ||
			IS_ERR(msm_host->pinctrl) || IS_ERR(msm_host->pinctrl_state_gpio)) {
		dev_dbg(dev, "Invalid pinctrl\n");
		return;
	}

	/* Switch pinctrl to GPIO mode */
	ret = pinctrl_select_state(msm_host->pinctrl, msm_host->pinctrl_state_gpio);
	if (ret) {
		dev_err(dev, "Failed to switch to gpio state: %d\n", ret);
		return;
	}

	/* Find the GPIO number for DAT1 */
	gpio_num = of_get_named_gpio(dev->of_node, "qcom,dat1-gpio", 0);
	if (gpio_num < 0) {
		dev_err(dev, "Failed to get DAT1 GPIO from DT\n");
		return;
	}

	ret = gpio_request(gpio_num, "sdio-dat1");
	if (ret) {
		dev_err(dev, "Failed to request gpio: %d\n", ret);
		return;
	}

	/* Toggle GPIO */
	gpio_direction_output(gpio_num, 1); /* Set HIGH */
	msleep(20);
	gpio_set_value(gpio_num, 0); /* Set LOW */
	msleep(20);

	/* Free the GPIO after operation */
	gpio_free(gpio_num);

	/* Optionally switch back to default SDIO mode */
	ret = pinctrl_select_state(msm_host->pinctrl, msm_host->pinctrl_state_default);
	if (ret)
		dev_err(dev, "Failed to switch back to default state: %d\n", ret);

	dev_dbg(dev, "Successfully toggled SDIO DAT1 via GPIO\n");
}
EXPORT_SYMBOL_GPL(sdhci_msm_toggle_dat1_gpio);

static __maybe_unused int sdhci_msm_runtime_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct sdhci_msm_qos_req *qos_req = msm_host->sdhci_qos;
	unsigned long flags;

	/* For firmware managed models, skip suspend sequence */
	if (msm_host->fw_managed)
		return 0;
	spin_lock_irqsave(&host->lock, flags);
	host->runtime_suspended = true;
	spin_unlock_irqrestore(&host->lock, flags);
	sdhci_msm_log_str(msm_host, "Enter\n");
	if (!qos_req)
		goto skip_qos;
	sdhci_msm_unvote_qos_all(msm_host);

skip_qos:
	sdhci_msm_registers_save(host);
	dev_pm_opp_set_rate(&msm_host->pdev->dev, 0);
	clk_bulk_disable_unprepare(ARRAY_SIZE(msm_host->bulk_clks),
					msm_host->bulk_clks);
	sdhci_msm_log_str(msm_host, "Clocks gated\n");
	sdhci_msm_bus_voting(host, false);
	return sdhci_msm_ice_suspend(msm_host);
}

static __maybe_unused int sdhci_msm_runtime_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);
	struct sdhci_msm_qos_req *qos_req = msm_host->sdhci_qos;
	unsigned long flags;
	int ret;

	sdhci_msm_log_str(msm_host, "Enter\n");

	/* For firmware managed models, skip resume sequence */
	if (msm_host->fw_managed)
		return 0;

	sdhci_msm_bus_voting(host, true);
	dev_pm_opp_set_rate(dev, msm_host->clk_rate);
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(msm_host->bulk_clks),
					       msm_host->bulk_clks);
	if (ret) {
		dev_err(dev, "Failed to enable clocks %d\n", ret);
		sdhci_msm_bus_voting(host, false);
		return ret;
	}

	sdhci_msm_registers_restore(host);
	sdhci_msm_toggle_fifo_write_clk(host);
	/*
	 * Whenever core-clock is gated dynamically, it's needed to
	 * restore the SDR DLL settings when the clock is ungated.
	 */
	if (msm_host->restore_dll_config && msm_host->clk_rate)
		sdhci_msm_restore_sdr_dll_config(host);

	if (!qos_req)
		goto skip_qos;

	sdhci_msm_vote_pmqos(msm_host->mmc,
			msm_host->sdhci_qos->active_mask);

skip_qos:
	ret = sdhci_msm_ice_resume(msm_host);
	if (ret)
		return ret;

	spin_lock_irqsave(&host->lock, flags);
	host->runtime_suspended = false;
	spin_unlock_irqrestore(&host->lock, flags);

	return ret;
}

static int sdhci_msm_suspend_late(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = sdhci_pltfm_priv(pltfm_host);

	sdhci_msm_log_str(msm_host, "Enter\n");
	return 0;
}

static int sdhci_msm_wrapper_suspend_late(struct device *dev)
{
	struct device_node *np = dev->of_node;

	if (of_property_read_bool(np, "non-removable") &&
			!is_bootdevice_sdhci) {
		dev_info(dev, "SDHCI is not boot dev.\n");
		return 0;
	}

	return sdhci_msm_suspend_late(dev);
}

static const struct dev_pm_ops sdhci_msm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(sdhci_msm_wrapper_suspend_late, NULL)
	SET_RUNTIME_PM_OPS(sdhci_msm_runtime_suspend,
			   sdhci_msm_runtime_resume,
			   NULL)
};

// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bayhub Technologies, Inc. BH201 SDHCI bridge IC for
 * VENDOR SDHCI platform driver source file
 *
 * Copyright (c) 2012-2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/mmc/card.h>
#include <linux/mmc/sd.h>

#define TRUE 1
#define FALSE 0

static const unsigned int tran_exp[] = {
	10000,		100000,		1000000,	10000000,
	0,		0,		0,		0
};

static const unsigned char tran_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

static const unsigned int taac_exp[] = {
	1,	10,	100,	1000,	10000,	100000,	1000000, 10000000,
};

static const unsigned int taac_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

#define UNSTUFF_BITS(resp,start,size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})
#define	SD_FNC_AM_SDR50		0x2
#define	SD_FNC_AM_SDR104	0x3
#define BIT_PASS_MASK  (0x7ff)
#define SDR104_MANUAL_INJECT 0x3ff
#define SDR50_MANUAL_INJECT  0x77f

#define	TRUNING_RING_IDX(x)  ((x) % TUNING_PHASE_SIZE)
#define	GET_IDX_VALUE(tb, x)  (tb & (1 << (x)))
#define	GENERATE_IDX_VALUE(x)  (1 << (x))
#define	GET_TRUNING_RING_IDX_VALUE(tb, x)  (tb & (1 << TRUNING_RING_IDX(x)))
#define	GENERATE_TRUNING_RING_IDX_VALUE(x)  (1 << TRUNING_RING_IDX(x))
#define	MAX_CFG_BIT_VAL (383)
#define MMC_CMD_RETRIES 3
static int gg_select_card_spec(struct sdhci_host *host);
static int bht_mmc_app_cmd(struct mmc_host *host, struct mmc_card *card)
{
	struct mmc_command cmd = { 0 };
	int ret;

	if (WARN_ON(card && card->host != host))
		return -EINVAL;
	cmd.opcode = MMC_APP_CMD;
	cmd.arg = card ? card->rca << 16 : 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 |
		(card ? MMC_CMD_AC : MMC_CMD_BCR);
	ret = mmc_wait_for_cmd(host, &cmd, 0);
	if (ret)
		return ret;
	if (!mmc_host_is_spi(host) && !(cmd.resp[0] & R1_APP_CMD))
		return -EOPNOTSUPP;
	return 0;
}
static int sd_tuning_sw(struct sdhci_host *host);

static struct sdhci_msm_host* get_vendor_host(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *vendor_host = sdhci_pltfm_priv(pltfm_host);
	return vendor_host;
}

static struct sdhci_host* get_sdhci_host(struct mmc_host *mmc)
{
	return mmc_priv(mmc);
}

static int init_flg;
static u32 g_card_cid[4] = {0};
static void cfg_bit_2_bt(int max_bit, int tar, int *byt, int *bit)
{
	struct rl_bit_lct cfg_bit_map[6] = {
		{0, 6},	{1, 5},	{2, 4},
		{3, 2},	{4, 1},	{5, 0},
	};

	*byt = (max_bit - tar) / 6;
	*bit = cfg_bit_map[(max_bit - tar) % 6].rl_bits;
}

static u32 cfg_read_bits_ofs_mask(u8 *cfg, struct t_gg_reg_strt *bts)
{
	u32 rv = 0;
	u32 msk = bts->mask;
	int byt = 0, bit = 0;
	int i = 0;

	do {
		cfg_bit_2_bt(MAX_CFG_BIT_VAL, bts->ofs + i, &byt, &bit);
		if (cfg[byt] & (1 << bit))
			rv |= 1 << i;

		i++;
		msk >>= 1;
		if (msk == 0)
			break;
	} while (1);
	return rv;
}

static void cfg_write_bits_ofs_mask(u8 *cfg, struct t_gg_reg_strt *bts, u32 w_value)
{
	u32 wv = w_value & bts->mask;
	u32 msk = bts->mask;
	int byt = 0, bit = 0;
	int i = 0;

	do {
		cfg_bit_2_bt(MAX_CFG_BIT_VAL, bts->ofs + i, &byt, &bit);
		if (wv & 1)
			cfg[byt] |= (u8) (1 << bit);
		else
			cfg[byt] &= (u8) (~(1 << bit));

		i++;
		wv >>= 1;
		msk >>= 1;
		if (msk == 0)
			break;
	} while (1);
}

static void ram_bit_2_bt(int tar, int *byt, int *bit)
{
	*byt = tar / 8;
	*bit = tar % 8;
}

static void set_gg_reg_cur_val(struct ggc_platform_t  *ggc, u8 *data, u8 len)
{

	memcpy(&ggc->_gg_reg_cur[0], data, len);

}

static void get_gg_reg_cur_val(struct ggc_platform_t  *ggc, u8 *data, u8 len)
{

	memcpy(data, &ggc->_gg_reg_cur[0], len);

}

static u32 read_ram_bits_ofs_mask(u8 *cfg, struct t_gg_reg_strt *bts)
{
	u32 rv = 0;
	u32 msk = bts->mask;
	int byt = 0, bit = 0;
	int i = 0;

	do {
		ram_bit_2_bt(bts->ofs + i, &byt, &bit);
		if (cfg[byt] & (1 << bit))
			rv |= 1 << i;

		i++;
		msk >>= 1;
		if (msk == 0)
			break;

	} while (1);
	return rv;
}

static void host_cmddat_line_reset(struct sdhci_host *host)
{
	if (host->ops && host->ops->reset)
		host->ops->reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
}

static int driver_send_command(struct sdhci_host *host)
{
	int ret = 0;
	int err;
	struct mmc_host *mmc = host->mmc;
	struct mmc_command cmd = {0};

	cmd.opcode = MMC_SELECT_CARD;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;
	err = mmc_wait_for_cmd(mmc, &cmd, 3);
	if (err)
		pr_err("---- CMD7 FAILE0 ----\n");
	else
		ret = 1;

	return ret;
}

static int driver_send_command24(struct sdhci_host *host, u32 *cfg_data, int data_len)
{
	struct mmc_host *mmc = host->mmc;

	u8 *data1 = kzalloc(PAGE_SIZE, GFP_KERNEL);
	struct mmc_request mrq = {0};
	struct mmc_command cmd = { 0 };
	struct mmc_data data = { 0 };
	struct scatterlist sg;

	if (!data1)
		return -ENOMEM;
	if (data_len < 0 || data_len > 512) {
		kfree(data1);
		return -EINVAL;
	}
	memcpy(data1, cfg_data, data_len);
	sg_init_one(&sg, data1, 512);

	cmd.opcode = MMC_WRITE_BLOCK;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	data.blksz = 512;
	data.blocks = 1;
	data.flags = MMC_DATA_WRITE;
	data.timeout_ns = 1000 * 1000 * 1000;
	data.sg = &sg;
	data.sg_len = 1;
	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = NULL;
	mmc_wait_for_req(mmc, &mrq);

	kfree(data1);
	return cmd.error ? cmd.error : data.error;
}

static int bht_mmc_send_cxd_native(struct mmc_host *host, u32 arg, u32 *cxd, int opcode)
{
	int err;
	struct mmc_command cmd = {};

	cmd.opcode = opcode;
	cmd.arg = arg;
	cmd.flags = MMC_RSP_R2 | MMC_CMD_AC;

	err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
	if (err)
		return err;

	memcpy(cxd, cmd.resp, sizeof(u32) * 4);

	return 0;
}

static int bht_mmc_send_csd(struct mmc_card *card, u32 *csd)
{
	return bht_mmc_send_cxd_native(card->host, card->rca << 16,	csd,
				MMC_SEND_CSD);
}

static int bht_mmc_decode_csd(struct mmc_card *card)
{
	struct mmc_csd *csd = &card->csd;
	unsigned int e, m, a, b;
	u32 *resp = card->raw_csd;

	csd->structure = UNSTUFF_BITS(resp, 126, 2);
	if (csd->structure == 0) {
		pr_err("%s: unrecognised CSD structure version %d\n",
			mmc_hostname(card->host), csd->structure);
		return -EINVAL;
	}

	csd->mmca_vsn	 = UNSTUFF_BITS(resp, 122, 4);
	m = UNSTUFF_BITS(resp, 115, 4);
	e = UNSTUFF_BITS(resp, 112, 3);
	csd->taac_ns	 = (taac_exp[e] * taac_mant[m] + 9) / 10;
	csd->taac_clks	 = UNSTUFF_BITS(resp, 104, 8) * 100;

	m = UNSTUFF_BITS(resp, 99, 4);
	e = UNSTUFF_BITS(resp, 96, 3);
	csd->max_dtr	  = tran_exp[e] * tran_mant[m];
	csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

	e = UNSTUFF_BITS(resp, 47, 3);
	m = UNSTUFF_BITS(resp, 62, 12);
	csd->capacity	  = (1 + m) << (e + 2);

	csd->read_blkbits = UNSTUFF_BITS(resp, 80, 4);
	csd->read_partial = UNSTUFF_BITS(resp, 79, 1);
	csd->write_misalign = UNSTUFF_BITS(resp, 78, 1);
	csd->read_misalign = UNSTUFF_BITS(resp, 77, 1);
	csd->dsr_imp = UNSTUFF_BITS(resp, 76, 1);
	csd->r2w_factor = UNSTUFF_BITS(resp, 26, 3);
	csd->write_blkbits = UNSTUFF_BITS(resp, 22, 4);
	csd->write_partial = UNSTUFF_BITS(resp, 21, 1);

	if (csd->write_blkbits >= 9) {
		a = UNSTUFF_BITS(resp, 42, 5);
		b = UNSTUFF_BITS(resp, 37, 5);
		csd->erase_size = (a + 1) * (b + 1);
		csd->erase_size <<= csd->write_blkbits - 9;
	}

	return 0;
}

static int bht_mmc_sd_get_csd(struct mmc_host *host, struct mmc_card *card)
{
	int err;

	err = bht_mmc_send_csd(card, card->raw_csd);
	if (err)
		return err;

	err = bht_mmc_decode_csd(card);
	if (err)
		return err;

	return 0;
}

static int bht_mmc_send_relative_addr(struct mmc_host *host, unsigned int *rca)
{
	int err;
	struct mmc_command cmd = {};

	cmd.opcode = 3;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R6 | MMC_CMD_BCR;

	err = mmc_wait_for_cmd(host, &cmd, 3);
	if (err)
		return err;

	*rca = cmd.resp[0] >> 16;

	return 0;
}

static int bht_mmc_app_send_scr(struct mmc_card *card)
{
	int err;
	struct mmc_request mrq = {};
	struct mmc_command cmd = {};
	struct mmc_data data = {};
	struct scatterlist sg;
	__be32 *scr;

	err = bht_mmc_app_cmd(card->host, card);
	if (err)
		return err;

	scr = kmalloc(sizeof(card->raw_scr), GFP_KERNEL);
	if (!scr)
		return -ENOMEM;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = SD_APP_SEND_SCR;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = 8;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, scr, 8);

	mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(card->host, &mrq);

	card->raw_scr[0] = be32_to_cpu(scr[0]);
	card->raw_scr[1] = be32_to_cpu(scr[1]);

	kfree(scr);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

static int bht_mmc_decode_scr(struct mmc_card *card)
{
	struct sd_scr *scr = &card->scr;
	unsigned int scr_struct;
	u32 resp[4];

	resp[3] = card->raw_scr[1];
	resp[2] = card->raw_scr[0];

	scr_struct = UNSTUFF_BITS(resp, 60, 4);
	if (scr_struct != 0) {
		pr_err("%s: unrecognised SCR structure version %d\n",
			mmc_hostname(card->host), scr_struct);
		return -EINVAL;
	}

	scr->sda_vsn = UNSTUFF_BITS(resp, 56, 4);
	scr->bus_widths = UNSTUFF_BITS(resp, 48, 4);
	if (scr->sda_vsn == SCR_SPEC_VER_2)

		scr->sda_spec3 = UNSTUFF_BITS(resp, 47, 1);

	if (scr->sda_spec3) {
		scr->sda_spec4 = UNSTUFF_BITS(resp, 42, 1);
		scr->sda_specx = UNSTUFF_BITS(resp, 38, 4);
	}

	if (UNSTUFF_BITS(resp, 55, 1))
		card->erased_byte = 0xFF;
	else
		card->erased_byte = 0x0;

	if (scr->sda_spec3)
		scr->cmds = UNSTUFF_BITS(resp, 32, 2);

	if (!(scr->bus_widths & SD_SCR_BUS_WIDTH_1) ||
	    !(scr->bus_widths & SD_SCR_BUS_WIDTH_4)) {
		pr_err("%s: invalid bus width\n", mmc_hostname(card->host));
		return -EINVAL;
	}

	return 0;
}

static int bht_mmc_sd_switch(struct mmc_card *card, int mode, int group,
	u8 value, u8 *resp)
{
	struct mmc_request mrq = {};
	struct mmc_command cmd = {};
	struct mmc_data data = {};
	struct scatterlist sg;

	mode = !!mode;
	value &= 0xF;

	mrq.cmd = &cmd;
	mrq.data = &data;

	cmd.opcode = SD_SWITCH;
	cmd.arg = mode << 31 | 0x00FFFFFF;
	cmd.arg &= ~(0xF << (group * 4));
	cmd.arg |= value << (group * 4);
	cmd.flags = MMC_RSP_SPI_R1 | MMC_RSP_R1 | MMC_CMD_ADTC;

	data.blksz = 64;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.sg = &sg;
	data.sg_len = 1;

	sg_init_one(&sg, resp, 64);

	mmc_set_data_timeout(&data, card);

	mmc_wait_for_req(card->host, &mrq);

	if (cmd.error)
		return cmd.error;
	if (data.error)
		return data.error;

	return 0;
}

static int bht_mmc_read_switch(struct mmc_card *card)
{
	int err;
	u8 *status;

	if (card->scr.sda_vsn < SCR_SPEC_VER_1)
		return 0;

	if (!(card->csd.cmdclass & CCC_SWITCH)) {
		pr_warn("%s: card lacks mandatory switch function, performance might suffer\n",
			mmc_hostname(card->host));
		return 0;
	}

	status = kmalloc(64, GFP_KERNEL);
	if (!status)
		return -ENOMEM;

	err = bht_mmc_sd_switch(card, 0, 0, 0, status);
	if (err) {

		if (err != -EINVAL && err != -ENOSYS && err != -EFAULT)
			goto out;

		pr_warn("%s: problem reading Bus Speed modes\n",
			mmc_hostname(card->host));
		err = 0;

		goto out;
	}

	if (status[13] & SD_MODE_HIGH_SPEED)
		card->sw_caps.hs_max_dtr = HIGH_SPEED_MAX_DTR;

	if (card->scr.sda_spec3) {
		card->sw_caps.sd3_bus_mode = status[13];

		card->sw_caps.sd3_drv_type = status[9];
		card->sw_caps.sd3_curr_limit = status[7] | status[6] << 8;
	}

out:
	kfree(status);

	return err;
}

static int bht_mmc_wait_for_app_cmd(struct mmc_host *host, struct mmc_card *card,
				struct mmc_command *cmd)
{
	struct mmc_request mrq = {};
	int i, err = -EIO;

	for (i = 0; i <= MMC_CMD_RETRIES; i++) {
		err = bht_mmc_app_cmd(host, card);
		if (err) {

			if (mmc_host_is_spi(host)) {
				if (cmd->resp[0] & R1_SPI_ILLEGAL_COMMAND)
					break;
			}
			continue;
		}

		memset(&mrq, 0, sizeof(struct mmc_request));

		memset(cmd->resp, 0, sizeof(cmd->resp));
		cmd->retries = 0;

		mrq.cmd = cmd;
		cmd->data = NULL;

		mmc_wait_for_req(host, &mrq);

		err = cmd->error;
		if (!cmd->error)
			break;

		if (mmc_host_is_spi(host)) {
			if (cmd->resp[0] & R1_SPI_ILLEGAL_COMMAND)
				break;
		}
	}

	return err;
}

static int bht_mmc_app_set_bus_width(struct mmc_card *card, int width)
{
	struct mmc_command cmd = {};

	cmd.opcode = SD_APP_SET_BUS_WIDTH;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	switch (width) {
	case MMC_BUS_WIDTH_1:
		cmd.arg = SD_BUS_WIDTH_1;
		break;
	case MMC_BUS_WIDTH_4:
		cmd.arg = SD_BUS_WIDTH_4;
		break;
	default:
		return -EINVAL;
	}

	return bht_mmc_wait_for_app_cmd(card->host, card, &cmd);
}

static int bht_mmc_app_set_clr_card_detect(struct mmc_card *card)
{
       struct mmc_command cmd = {};

       cmd.opcode = 42;
       cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

       return bht_mmc_wait_for_app_cmd(card->host, card, &cmd);
}

static int bht_mmc_send_io_op_cond(struct mmc_host *host, u32 ocr, u32 *rocr)
{
	struct mmc_command cmd = {};
	int i, err = 0;

	cmd.opcode = 5;
	cmd.arg = ocr;
	cmd.flags = MMC_RSP_SPI_R4 | MMC_RSP_R4 | MMC_CMD_BCR;

	for (i = 100; i; i--) {
		err = mmc_wait_for_cmd(host, &cmd, MMC_CMD_RETRIES);
		if (err)
			break;

		if (ocr == 0)
			break;

		if (mmc_host_is_spi(host)) {

			if (cmd.resp[1] & MMC_CARD_BUSY)
				break;
		} else {
			if (cmd.resp[0] & MMC_CARD_BUSY)
				break;
		}

		err = -ETIMEDOUT;
	}

	if (rocr)
		*rocr = cmd.resp[mmc_host_is_spi(host) ? 1 : 0];

	return err;
}
static int wait_dll_lock(struct sdhci_host *host)
{
	return sd_tuning_sw(host);
}

static int bht_update_cfg(struct mmc_host *mmc_host, struct mmc_card *card, u32 *cfg_data, int data_len)
{
	int ret = 0;
	struct sdhci_host *host;

	host = get_sdhci_host(mmc_host);
	if(host->ops && host->ops->set_bus_width)
		host->ops->set_bus_width(host, MMC_BUS_WIDTH_4);
	else
		sdhci_set_bus_width(host, MMC_BUS_WIDTH_4);

	if (host->ops && host->ops->reset)
		host->ops->reset(host, SDHCI_RESET_CMD|SDHCI_RESET_DATA);

	if (!driver_send_command(host))
		pr_err("--send cmd7   error--\n");

	ret = driver_send_command24(host, cfg_data, data_len);

	if (!driver_send_command(host))
		pr_err("--send cmd7  error--\n");

	if(host->ops && host->ops->set_bus_width)
		host->ops->set_bus_width(host, MMC_BUS_WIDTH_1);
	else
		sdhci_set_bus_width(host, MMC_BUS_WIDTH_1);
	return ret;
}

static int bht_load_hw_inject(struct mmc_host *mmc_host, struct mmc_card *card,
						u32 *cfg_data, int data_len, u32 sel200, u32 sel100)
{
	struct sdhci_host *host = get_sdhci_host(mmc_host);
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	u32 gg_hw_inj[16] = GGC_CFG_DATA;

	gg_hw_inj[1] = 0x7364032;

	if (vendor_host->ggc.bh201_sdr104_selb_hw_inject)
		gg_hw_inj[11] = vendor_host->ggc.bh201_sdr104_selb_hw_inject;
	else
		gg_hw_inj[11] = 0x77316200;

	if (vendor_host->ggc.bh201_sdr50_selb_hw_inject)
		gg_hw_inj[12] = vendor_host->ggc.bh201_sdr50_selb_hw_inject;
	else
		gg_hw_inj[12] = 0x00725777;

	pr_info("gg_hw_inj[11] = 0x%08x, gg_hw_inj[12] = 0x%08x, gg_hw_inj[15] = 0x%08x \n",gg_hw_inj[11], gg_hw_inj[12], gg_hw_inj[15]);

	return bht_update_cfg(mmc_host, card, gg_hw_inj, data_len);
}

static void sdhci_bh201_parse(struct mmc_host *mmc_host)
{
	struct sdhci_host *host = get_sdhci_host(mmc_host);
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	struct device_node *np = vendor_host->pdev->dev.of_node;
	struct t_gg_reg_strt index_array[] ={
		{ 14, 0xffffffff, 0 }, { 46, 0xffffffff, 0 },
		{ 205, 0xffffffff, 0 }, { 237, 0xffffffff, 0 },
		{ 141, 0xf, 0 }, { 145, 0xf, 0 },
		{ 83, 0xfff, 0 }, { 95, 0xfff, 0 },
		{ 126, 0xf, 0 }, { 130, 0xf, 0 },
		{ 140, 0xf, 0 }, { 144, 0xf, 0 },
		{ 183, 0x1, 0 }, { 184, 0x1, 0 },
		{ 171, 0x1, 0 }, { 172, 0x1, 0 },
		{ 173, 0x3f, 0 }, { 357, 0x1, 0 },
		{ 93, 0x7ff, 0 }, { 81, 0x7ff, 0 },
	};

	vendor_host->ggc.pha_stas_rx_low32 = index_array[0];
	vendor_host->ggc.pha_stas_rx_high32 = index_array[1];
	vendor_host->ggc.pha_stas_tx_low32 = index_array[2];
	vendor_host->ggc.pha_stas_tx_high32 = index_array[3];
	vendor_host->ggc.dll_sela_after_mask = index_array[4];
	vendor_host->ggc.dll_selb_after_mask = index_array[5];

	vendor_host->ggc.dll_delay_100m_backup = index_array[6];
	vendor_host->ggc.dll_delay_200m_backup = index_array[7];

	vendor_host->ggc.dll_sela_100m_cfg = index_array[8];
	vendor_host->ggc.dll_sela_200m_cfg = index_array[9];
	vendor_host->ggc.dll_selb_100m_cfg = index_array[10];
	vendor_host->ggc.dll_selb_200m_cfg = index_array[11];
	vendor_host->ggc.dll_selb_100m_cfg_en = index_array[12];
	vendor_host->ggc.dll_selb_200m_cfg_en = index_array[13];
	vendor_host->ggc.internl_tuning_en_100m = index_array[14];
	vendor_host->ggc.internl_tuning_en_200m = index_array[15];
	vendor_host->ggc.cmd19_cnt_cfg = index_array[16];

	vendor_host->ggc.inject_failure_for_tuning_enable_cfg = index_array[17];
	vendor_host->ggc.inject_failure_for_200m_tuning_cfg = index_array[18];
	vendor_host->ggc.inject_failure_for_100m_tuning_cfg = index_array[19];

	vendor_host->ggc.bh201_used = 1;
	vendor_host->ggc.cur_sd_bus_speed = 0;
	vendor_host->ggc.bh201_drive_strength = 0;
	vendor_host->ggc.bh201_sdr50_sela_sw_inject = 0;
	vendor_host->ggc.bh201_sdr50_selb_hw_inject = 0;
	vendor_host->ggc.bh201_sdr104_selb_hw_inject = 0;

	host->flags |= SDHCI_SDR50_NEEDS_TUNING;
	host->mmc_host_ops.init_card = bht_load;
	ggc_chip_init(host);

	if (of_find_property(np, "bh201_drive_strength", NULL))
		of_property_read_u32_index(np, "bh201_drive_strength", 0,
			&vendor_host->ggc.bh201_drive_strength);
	if (of_find_property(np, "bh201_sdr50_sela_sw_inject", NULL))
		of_property_read_u32_index(np, "bh201_sdr50_sela_sw_inject", 0,
			&vendor_host->ggc.bh201_sdr50_sela_sw_inject);
	if (of_find_property(np, "bh201_sdr50_selb_hw_inject", NULL))
		of_property_read_u32_index(np, "bh201_sdr50_selb_hw_inject", 0,
			&vendor_host->ggc.bh201_sdr50_selb_hw_inject);
	if (of_find_property(np, "bh201_sdr104_selb_hw_inject", NULL))
		of_property_read_u32_index(np, "bh201_sdr104_selb_hw_inject", 0,
			&vendor_host->ggc.bh201_sdr104_selb_hw_inject);

}

static int card_deselect_card(struct sdhci_host *host)
{
	int ret = -1;
	int err;
	struct mmc_host *mmc = host->mmc;
	struct mmc_command cmd = { 0 };

	cmd.opcode = MMC_SELECT_CARD;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;

	err = mmc_wait_for_cmd(mmc, &cmd, 3);
	if (err)
		pr_err("BHT ERR:%s: ---- CMD7 FAIL: err = %d ----\n", __func__, err);
	else
		ret = 0;

	return ret;
}

static bool enter_exit_emulator_mode(struct sdhci_host *host, bool b_enter)
{
	bool ret = FALSE;
	u8 times = b_enter ? 2 : 1;
	u8 i = 0;

	for (i = 0; i < times; i++) {
		ret = card_deselect_card(host);
		if (ret)
			break;
	}
	return ret;
}

static bool _gg_emulator_read_only(struct sdhci_host *host,
				   u8 *in_data, u32 datalen)
{
	struct mmc_host *mmc = host->mmc;
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	int rc = 0;
	u8 *data1 = kzalloc(PAGE_SIZE, GFP_KERNEL);
	struct mmc_request mrq = { 0 };
	struct mmc_command cmd = { 0 };
	struct mmc_data data = { 0 };
	struct scatterlist sg;

	if (!data1) {
		pr_info("BHT MSG:gg read no memory\n");
		rc = -ENOMEM;
		goto out;
	}

	sg_init_one(&sg, data1, 512);

	cmd.opcode = MMC_READ_SINGLE_BLOCK;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	data.blksz = 512;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;
	data.timeout_ns = 1000 * 1000 * 1000;
	data.sg = &sg;
	data.sg_len = 1;
	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = NULL;

	mmc_wait_for_req(mmc, &mrq);
	memcpy(in_data, data1, datalen);

	kfree(data1);

	if (cmd.error || data.error) {

		if (cmd.error) {
			if (cmd.error == -EILSEQ) {
				pr_err("BHT ERR:cmd error 0x%x\n", cmd.error);
				vendor_host->ggc.sdr50_notuning_crc_error_flag = 1;
			}
		}

		if (data.error) {
			if (data.error == -EILSEQ) {
				pr_err("BHT ERR:data error 0x%x\n", data.error);
				vendor_host->ggc.sdr50_notuning_crc_error_flag = 1;
			}
		}
		rc = -1;
		goto out;
	}

out:
	return rc;
}

static int gg_select_card_spec(struct sdhci_host *host)
{
	int err;
	struct mmc_command cmd = { 0 };
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	cmd.opcode = MMC_SELECT_CARD;

	if (host->mmc->card) {
		cmd.arg = host->mmc->card->rca << 16;
		cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	} else if (vendor_host->ggc.card){
		cmd.arg = vendor_host->ggc.card->rca << 16;
		cmd.flags = MMC_RSP_R1B | MMC_CMD_AC;
	} else {
		pr_err("Can't get valid RCA, deselect card instead of select card\n");
	}

	err = mmc_wait_for_cmd(host->mmc, &cmd, 0);
	if (err) {
		if (-EILSEQ == err) {
			host_cmddat_line_reset(host);
			{
				struct mmc_command cmd = { 0 };

				cmd.opcode = 5;
				cmd.arg = 0;
				cmd.flags =
				    MMC_RSP_SPI_R4 | MMC_RSP_R4 | MMC_CMD_BCR;

				mmc_wait_for_cmd(host->mmc, &cmd, 0);
			}
			pr_err("BHT ERR:%s: CMD7 CRC\n", __func__);
			host_cmddat_line_reset(host);
			return 0;
		}
		if (-ETIMEDOUT == err) {
			pr_err("BHT ERR:%s: CMD7 timeout\n", __func__);
			host_cmddat_line_reset(host);
			return err;
		}
		return 0;
	}

	return 0;
}

static bool gg_emulator_read_ext(struct sdhci_host *host, bool *card_status,
					bool *read_status, u8 *data, u32 datalen)
{
	bool ret = FALSE;
	bool card_ret = TRUE;
	bool rd_ret = FALSE;

	ret = (enter_exit_emulator_mode(host, TRUE) == 0) ? TRUE : FALSE;
	if (!ret)
		goto exit;

	rd_ret = (_gg_emulator_read_only(host, data, datalen) == 0) ? TRUE : FALSE;

	ret = (enter_exit_emulator_mode(host, FALSE) == 0) ? TRUE : FALSE;
	if (!ret)
		goto exit;

	card_ret = (gg_select_card_spec(host) == 0) ? TRUE : FALSE;

	if (!rd_ret)
		pr_err("BHT ERR:GGC read status error\n");

exit:
	if (!card_ret) {
		pr_err("BHT ERR:GGC Emulator exit Fail!!\n");
		ret = FALSE;
	}

	if (card_status)
		*card_status = ret;

	if (read_status)
		*read_status = rd_ret;

	if (rd_ret && !ret)
		pr_err("BHT ERR:data read ok, but exit NG\n");
	else if (!rd_ret && ret)
		pr_err("BHT ERR:data read NG, but exit ok\n");

	return ret;
}

static void _status_bit_2_bt(int tar, int *byt, int *bit)
{
	*byt = tar / 8;
	*bit = tar % 8;
}

static u32 _read_status_data_read_register(u8 *cfg, struct t_gg_reg_strt *bts)
{
	u32 rv = 0;
	u32 msk = bts->mask;
	int byt = 0, bit = 0;
	int i = 0;

	do {
		_status_bit_2_bt(bts->ofs + i, &byt, &bit);
		if (cfg[byt] & (1 << bit))
			rv |= 1 << i;

		i++;
		msk >>= 1;
		if (msk == 0)
			break;
	} while (1);
	return rv;
}

static bool ggc_read_registers_ext(struct sdhci_host *host,
		bool *card_status, bool *read_status,
		struct t_gg_reg_strt *gg_reg_arr, u8 num)
{
	u8 get_idx = 0;
	bool ret = FALSE;
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	struct ggc_platform_t *ggc = &vendor_host->ggc;

	if (read_status)
		*read_status = FALSE;
	if (card_status)
		*card_status = FALSE;

	memset(ggc->_cur_read_buf, 0, 512);
	ret = gg_emulator_read_ext(host, card_status, read_status, ggc->_cur_read_buf, 512);
	if (read_status == FALSE)
		goto exit;

	for (get_idx = 0; get_idx < num; get_idx++)
		(gg_reg_arr + get_idx)->value =
			_read_status_data_read_register(ggc->_cur_read_buf, (gg_reg_arr + get_idx));

exit:
	return ret;
}

static bool gg_emulator_read(struct sdhci_host *host, u8 *data, u32 datalen)
{
	bool ret = FALSE;
	bool rd_ret = FALSE;

	ret = enter_exit_emulator_mode(host, TRUE);
	if (ret)
		goto exit;

	rd_ret = _gg_emulator_read_only(host, data, datalen);

	ret = enter_exit_emulator_mode(host, FALSE);
	if (ret)
		goto exit;

	ret = gg_select_card_spec(host);

exit:
	if (rd_ret)
		pr_err("BHT ERR:GGC read status error\n");

	if (ret)
		pr_err("BHT ERR:GGC Emulator exit Fail!!\n");

	if (rd_ret == 0 && ret) {
		pr_err("BHT ERR:data read ok, but exit NG\n");
		ret = 0;
	}

	if (rd_ret && ret == 0) {
		pr_err("BHT ERR:data read NG, but exit ok\n");
		ret = -1;
	}

	return ret ? FALSE : TRUE;
}

static bool _ggc_emulator_write_only(struct sdhci_host *host,
				     u8 *in_data, u32 datalen)
{
	struct mmc_host *mmc = host->mmc;
	int rc = 0;
	u8 *data1 = kzalloc(PAGE_SIZE, GFP_KERNEL);
	struct mmc_request mrq = { 0 };
	struct mmc_command cmd = { 0 };
	struct mmc_data data = { 0 };
	struct scatterlist sg;
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	if (!data1) {
		pr_info("BHT MSG:gg write no memory\n");
		rc = -ENOMEM;
		goto out;
	}

	memcpy(data1, in_data, datalen);
	sg_init_one(&sg, data1, 512);

	cmd.opcode = MMC_WRITE_BLOCK;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;
	data.blksz = 512;
	data.blocks = 1;
	data.flags = MMC_DATA_WRITE;
	data.timeout_ns = 1000 * 1000 * 1000;
	data.sg = &sg;
	data.sg_len = 1;
	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = NULL;

	mmc_wait_for_req(mmc, &mrq);

	if (cmd.error == -EILSEQ)
		vendor_host->ggc.sdr50_notuning_crc_error_flag = 1;

	kfree(data1);
out:
	return rc;
}

static bool gg_emulator_write(struct sdhci_host *host, u8 *data, u32 datalen)
{
	bool ret = FALSE;
	bool wr_ret = FALSE;
	u32 i = 0;
	u32 reg;

	ret = enter_exit_emulator_mode(host, TRUE);
	if (ret)
		goto exit;

	pr_info("BHT MSG:%s: dump config data\n", __func__);
	for (i = 0; i < (datalen/sizeof(u32)); i++)	{
		memcpy(&reg, data+i*sizeof(u32), sizeof(u32));
		pr_info("BHT MSG:\tggc_reg32[%03d]=0x%08x\n", i, reg);
	}

	_ggc_emulator_write_only(host, data, datalen);
	wr_ret = TRUE;

	ret = enter_exit_emulator_mode(host, FALSE);
	if (ret)
		goto exit;

	ret = (gg_select_card_spec(host) == 0) ? TRUE : FALSE;
exit:
	if (wr_ret == FALSE)
		ret = FALSE;

	if (ret == FALSE)
		pr_err("BHT ERR:%s: GGC Emulator Write Fail!!\n", __func__);

	return ret;
}

static void get_gg_reg_def(struct sdhci_host *host, u8 *data)
{
	u32 gg_sw_def[16] = GGC_CFG_DATA;

	memcpy(data, (u8 *)&(gg_sw_def[0]), sizeof(gg_sw_def));
}

static bool get_gg_reg_cur(struct sdhci_host *host, u8 *data,
		    struct t_gg_reg_strt *gg_reg_arr, u8 num)
{
	u8 get_idx = 0;
	bool ret = FALSE;

	memset(data, 0, 512);
	ret = gg_emulator_read(host, data, 512);

	if (ret == FALSE)
		goto exit;

	for (get_idx = 0; get_idx < num; get_idx++) {
		(gg_reg_arr + get_idx)->value =
		    read_ram_bits_ofs_mask(data, (gg_reg_arr + get_idx));
	}
exit:
	return ret;
}

static void chg_gg_reg_cur_val(struct ggc_platform_t  *ggc, u8 *data,
	struct t_gg_reg_strt *gg_reg_arr, u8 num, bool b_sav_chg)
{
	u8 chg_idx = 0;

	for (chg_idx = 0; chg_idx < num; chg_idx++) {

		cfg_write_bits_ofs_mask(data, (gg_reg_arr + chg_idx),
					(gg_reg_arr + chg_idx)->value);
	}

	if (b_sav_chg)
		set_gg_reg_cur_val(ggc, data, 64);
}

static void log_bin(u32 n)
{
	int i = 0;
	u8 tb[33] = { 0 };

	for (i = 0; i < 32; i++) {
		if (n & (1 << i))
			tb[i] = '1';
		else
			tb[i] = '0';
	}
	pr_info("BHT MSG:bin:%s\n", tb);
}

static void phase_str(u8 *tb, u32 n)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (n & (1 << i))
			tb[i] = '1';
		else
			tb[i] = '0';
	}
	tb[TUNING_PHASE_SIZE] = 0;
}

static int get_bit_number(u32 n)
{
	int i = 0;
	int cnt = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (n & (1 << i))
			cnt++;
	}
	return cnt;
}

static bool gg_emulator_write_ext(struct sdhci_host *host, bool *card_status, u8 *data, u32 datalen)
{
	bool ret = FALSE;
	bool wr_ret = FALSE;

	ret = enter_exit_emulator_mode(host, TRUE);
	if (ret)
		goto exit;

	_ggc_emulator_write_only(host, data, datalen);
	wr_ret = TRUE;

	ret = enter_exit_emulator_mode(host, FALSE);
	if (ret)
		goto exit;

	ret = (gg_select_card_spec(host) == 0) ? TRUE : FALSE;
	if (ret == FALSE) {
		if (card_status)
			*card_status = FALSE;
	}

exit:
	if (wr_ret == FALSE)
		ret = FALSE;

	if (ret == FALSE)
		pr_err("BHT ERR:%s: GGC Emulator Write Fail!!\n", __func__);

	return ret;
}

static bool ggc_set_output_tuning_phase_ext(struct sdhci_host *host, bool *card_status, int sela, int selb)
{
	bool ret = TRUE;
	u8 data[512] = { 0 };
	struct t_gg_reg_strt gg_reg_arr[8];
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	get_gg_reg_cur_val(&vendor_host->ggc, data, 64);
	memcpy(&gg_reg_arr[0], &vendor_host->ggc.dll_sela_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.dll_sela_200m_cfg, sizeof(struct t_gg_reg_strt));
	gg_reg_arr[0].value = sela;
	gg_reg_arr[1].value = sela;
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.dll_selb_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[3], &vendor_host->ggc.dll_selb_200m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[4], &vendor_host->ggc.dll_selb_100m_cfg_en, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[5], &vendor_host->ggc.dll_selb_200m_cfg_en, sizeof(struct t_gg_reg_strt));
	gg_reg_arr[2].value = selb;
	gg_reg_arr[3].value = selb;
	gg_reg_arr[4].value = 1;
	gg_reg_arr[5].value = 1;
	memcpy(&gg_reg_arr[6], &vendor_host->ggc.internl_tuning_en_100m,
		  sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[7], &vendor_host->ggc.internl_tuning_en_200m,
		  sizeof(struct t_gg_reg_strt));
	gg_reg_arr[6].value = 1;
	gg_reg_arr[7].value = 1;
	if (card_status)
		*card_status = TRUE;
	chg_gg_reg_cur_val(&vendor_host->ggc, data, gg_reg_arr, 8, TRUE);
	ret = gg_emulator_write_ext(host, card_status, data, 512);
	return ret;
}

static bool gg_fix_output_tuning_phase(struct sdhci_host *host, int sela, int selb)
{
	u8 data[512] = { 0 };
	struct t_gg_reg_strt gg_reg_arr[10];
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	pr_info("BHT MSG:### %s - sela dll: %x, selb dll: %x\n", __func__, sela,
		selb);

	get_gg_reg_cur_val(&vendor_host->ggc, data, 64);

	memcpy(&gg_reg_arr[0], &vendor_host->ggc.dll_sela_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.dll_sela_200m_cfg, sizeof(struct t_gg_reg_strt));
	gg_reg_arr[0].value = sela;
	gg_reg_arr[1].value = sela;
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.dll_selb_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[3], &vendor_host->ggc.dll_selb_200m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[4], &vendor_host->ggc.dll_selb_100m_cfg_en, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[5], &vendor_host->ggc.dll_selb_200m_cfg_en, sizeof(struct t_gg_reg_strt));
	gg_reg_arr[2].value = selb;
	gg_reg_arr[3].value = selb;
	gg_reg_arr[4].value = 1;
	gg_reg_arr[5].value = 1;
	memcpy(&gg_reg_arr[6], &vendor_host->ggc.internl_tuning_en_100m,
		  sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[7], &vendor_host->ggc.internl_tuning_en_200m,
		  sizeof(struct t_gg_reg_strt));
	gg_reg_arr[6].value = 0;
	gg_reg_arr[7].value = 0;

	chg_gg_reg_cur_val(&vendor_host->ggc, data, gg_reg_arr, 8, TRUE);

	return gg_emulator_write(host, data, 512);
}

static void gen_array_data(u32 low32, u32 high32, u32 *ptw)
{
	u8 tu_res_per[6][TUNING_PHASE_SIZE];
	u8 i = 0, j = 0;
	u8 i_mode = 0;
	u32 tw = 0;

	memset(tu_res_per, 1, sizeof(tu_res_per));
	for (i = 0; i < 64; i++) {
		u32 tmp_data = (i < 32) ? low32 : high32;

		tu_res_per[i / TUNING_PHASE_SIZE][i % TUNING_PHASE_SIZE] =
		    (tmp_data & (1 << (i % 32))) >> (i % 32);
	}

	for (i_mode = 0; i_mode < TUNING_PHASE_SIZE; i_mode++) {
		for (j = 0; j < 6; j++) {
			if (tu_res_per[j][i_mode] != 0)
				tw |= (1 << i_mode);
			else {
				tw &= ~(1 << i_mode);
				break;
			}
		}
	}
	if (ptw)
		*ptw = tw;
}

static bool sw_calc_tuning_result(struct sdhci_host *host, u32 *tx_selb,
			   u32 *all_selb, u64 *raw_tx_selb)
{
	bool ret = FALSE;
	u8 data[512] = { 0 };
	u32 selb_status_tx_low32 = 0, selb_status_tx_high32 = 0;
	u32 selb_status_ggc_low32 = 0, selb_status_ggc_high32 = 0;
	struct t_gg_reg_strt gg_reg_arr[6];
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	memcpy(&gg_reg_arr[0], &vendor_host->ggc.pha_stas_tx_low32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.pha_stas_tx_high32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.pha_stas_rx_low32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[3], &vendor_host->ggc.pha_stas_rx_high32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[4], &vendor_host->ggc.dll_sela_after_mask, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[5], &vendor_host->ggc.dll_selb_after_mask, sizeof(struct t_gg_reg_strt));

	ret = get_gg_reg_cur(host, data, gg_reg_arr, 6);

	if (ret == TRUE) {
		selb_status_tx_low32 = gg_reg_arr[0].value;
		pr_info("BHT MSG:[205-236]:\n");
		log_bin(selb_status_tx_low32);
		pr_info("BHT MSG:[237-268]:\n");
		selb_status_tx_high32 = gg_reg_arr[1].value;
		log_bin(selb_status_tx_high32);

		pr_info("BHT MSG:[14-45]:\n");
		selb_status_ggc_low32 = gg_reg_arr[2].value;
		log_bin(selb_status_ggc_low32);
		pr_info("BHT MSG:[46-77]:\n");
		selb_status_ggc_high32 = gg_reg_arr[3].value;
		log_bin(selb_status_ggc_high32);
		pr_info("BHT MSG:dll sela after mask=%xh\n", gg_reg_arr[4].value);
		pr_info("BHT MSG:dll selb after mask=%xh\n", gg_reg_arr[5].value);

		if (raw_tx_selb) {
			*raw_tx_selb = gg_reg_arr[1].value;
			(*raw_tx_selb) <<= 32;
			*raw_tx_selb += gg_reg_arr[0].value;
			pr_info("BHT MSG:raw_tx_selb:%llxh\n", *raw_tx_selb);
		}

		if (tx_selb) {
			gen_array_data(gg_reg_arr[0].value, gg_reg_arr[1].value,
				       tx_selb);
			pr_info("BHT MSG:tx_selb:%xh\n", *tx_selb);
		}
		if (all_selb) {
			gen_array_data(gg_reg_arr[2].value, gg_reg_arr[3].value,
				       all_selb);
			pr_info("BHT MSG:all_selb:%xh\n", *all_selb);
		}
	}

	return ret;
}

static bool gg_tuning_result(struct sdhci_host *host, u32 *tx_selb, u32 *all_selb,
			u64 *raw_tx_selb)
{

	host_cmddat_line_reset(host);
	return sw_calc_tuning_result(host, tx_selb, all_selb, raw_tx_selb);
}

static u64 GENERATE_64_IDX_VALUE(int sft)
{
	u64 val = 1;

	return val << sft;
}

static bool is_bus_mode_sdr104(struct sdhci_host *host)
{
	bool ret = FALSE;

	if (host->timing == MMC_TIMING_UHS_SDR104)
		ret = TRUE;

	return ret;
}

static bool _check_bus_mode(struct sdhci_host *host)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	struct ggc_platform_t *ggc = &vendor_host->ggc;

	if (is_bus_mode_sdr104(host))
		ggc->cur_bus_mode = &vendor_host->ggc.sdr104;
	else
		ggc->cur_bus_mode = &vendor_host->ggc.sdr50;

	return true;
}

static void tx_selb_failed_history_update(struct sdhci_host *host, u32 val)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	_check_bus_mode(host);

	vendor_host->ggc.cur_bus_mode->tx_selb_failed_history &= val;
}

static void tx_selb_failed_history_reset(struct sdhci_host *host)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	vendor_host->ggc.sdr50.tx_selb_failed_history = BIT_PASS_MASK;
	vendor_host->ggc.sdr104.tx_selb_failed_history = BIT_PASS_MASK;
}

static u32 tx_selb_failed_history_get(struct sdhci_host *host)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	_check_bus_mode(host);

	return vendor_host->ggc.cur_bus_mode->tx_selb_failed_history;
}

static void tx_selb_failed_tb_update(struct sdhci_host *host, int sela, u32 val)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	_check_bus_mode(host);
	vendor_host->ggc.cur_bus_mode->tx_selb_tb[sela] &= val;
}

static void tx_selb_failed_tb_reset(struct sdhci_host *host)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	memset(&vendor_host->ggc.sdr104.tx_selb_tb, 0xff,
			sizeof(vendor_host->ggc.sdr104.tx_selb_tb));
	memset(&vendor_host->ggc.sdr50.tx_selb_tb, 0xff,
			sizeof(vendor_host->ggc.sdr50.tx_selb_tb));
}

static u32 tx_selb_failed_tb_get(struct sdhci_host *host, int sela)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	u32 value = 0;

	_check_bus_mode(host);

	if (is_bus_mode_sdr104(host))
		value = vendor_host->ggc.sdr104.tx_selb_tb[sela];
	else
		value = vendor_host->ggc.sdr50.tx_selb_tb[sela];

	return value;
}

static void all_selb_failed_tb_update(struct sdhci_host *host, int sela, u32 val)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	_check_bus_mode(host);
	vendor_host->ggc.cur_bus_mode->all_selb_tb[sela] &= val;
}

static void all_selb_failed_tb_reset(struct sdhci_host *host)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	memset(vendor_host->ggc.sdr104.all_selb_tb, 0xff,
			sizeof(vendor_host->ggc.sdr104.all_selb_tb));
	memset(vendor_host->ggc.sdr50.all_selb_tb, 0xff,
			sizeof(vendor_host->ggc.sdr50.all_selb_tb));
}

static u32 all_selb_failed_tb_get(struct sdhci_host *host, int sela)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	u32 val;

	_check_bus_mode(host);

	val = vendor_host->ggc.cur_bus_mode->all_selb_tb[sela];

	return val;
}

static void chk_phase_window(u8 *tuning_win, u8 *mid_val, u8 *max_pass_win)
{
	u8 tuning_pass[TUNING_PHASE_SIZE + 32];
	u8 tuning_pass_start[TUNING_PHASE_SIZE + 32];
	u8 tuning_pass_num_max = 0;
	u8 first_0 = 0;
	u8 i = 0, j = 0;
	u8 i_mode = 0, selb_mode = 0;

	memset(tuning_pass, 1, sizeof(tuning_pass));
	memset(tuning_pass_start, 1, sizeof(tuning_pass_start));

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (tuning_win[i] == 0) {
			first_0 = i;
			break;
		}
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		i_mode = (first_0 + i) % TUNING_PHASE_SIZE;
		if (tuning_win[i_mode] == 1)
			tuning_pass[j]++;
		else if (tuning_pass[j])
			j++;
		if (tuning_pass[j] == 1)
			tuning_pass_start[j] = i_mode;
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (tuning_pass_num_max < tuning_pass[i]) {
			tuning_pass_num_max = tuning_pass[i];
			i_mode = i;
		}
	}

	if (tuning_pass_num_max == 0)
		pr_err
		    ("###### Get max pass window fail, there is no any pass phase!!\n");
	else {
		*max_pass_win = tuning_pass_num_max - 1;
		tuning_pass_num_max /= 2;
		selb_mode = tuning_pass_start[i_mode] + tuning_pass_num_max;
		if ((*max_pass_win % 2 == 0))
			selb_mode += 1;
		selb_mode %= TUNING_PHASE_SIZE;
	}

	*mid_val = selb_mode;
}

static void dump_array(u8 *tb)
{
	int i = 0;
	u8 str[12] = { 0 };

	for (i = 0; i < TUNING_PHASE_SIZE; i++)
		str[i] = tb[i] + '0';

}

static void bits_generate_array(u8 *tb, u32 v)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if ((v & (1 << i)))
			tb[i] = 1;
		else
			tb[i] = 0;
	}
	dump_array(tb);
}

static int ggc_get_selx_weight(u32 val)
{
	int i = 0;
	int cnt = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(val, i))
			cnt++;
	}
	return cnt;
}

static void tx_selb_calculate_valid_phase_range(u32 val, int *start,
						int *pass_cnt)
{
	int i = 0, flg = 0;
	*pass_cnt = ggc_get_selx_weight(val);
	for (i = 0; i < (TUNING_PHASE_SIZE * 2); i++) {
		if ((GET_TRUNING_RING_IDX_VALUE(val, i)) == 0 && (flg == 0))
			flg = 1;
		if ((flg == 1) && GET_TRUNING_RING_IDX_VALUE(val, i)) {
			*start = TRUNING_RING_IDX(i);
			break;
		}
	}
}

static bool ggc_update_default_selb_phase_tuning_cnt(struct sdhci_host *host, int selb,
					      int tuning_cnt)
{
	struct t_gg_reg_strt gg_reg_arr[3];
	u8 data[512] = { 0 };
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	get_gg_reg_cur_val(&vendor_host->ggc, data, 64);

	pr_info("BHT MSG:%s selb:%xh,tuning_cnt:%xh\n", __func__, selb,
		 tuning_cnt);
	memcpy(&gg_reg_arr[0], &vendor_host->ggc.dll_selb_100m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.dll_selb_200m_cfg, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.cmd19_cnt_cfg, sizeof(struct t_gg_reg_strt));

	gg_reg_arr[0].value = selb;
	gg_reg_arr[1].value = selb;
	gg_reg_arr[2].value = tuning_cnt;
	chg_gg_reg_cur_val(&vendor_host->ggc, data, gg_reg_arr, 3, TRUE);

	return TRUE;
}

static void _ggc_update_cur_setting_for_sw_selb_tuning(struct sdhci_host *host,
						       u32 val)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	int start = 0, pass_cnt = 0;

	tx_selb_calculate_valid_phase_range(val, &start, &pass_cnt);
	pr_info("BHT MSG:%s %x %x %x\n", __func__, val, start, pass_cnt);
	ggc_update_default_selb_phase_tuning_cnt(host, start, pass_cnt);
	vendor_host->ggc.ggc_sw_selb_tuning_first_selb = start;
}

static int sdhci_bht_sdr104_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	struct mmc_host *mmc = host->mmc;

	return sdhci_msm_execute_tuning(mmc, opcode);
}

static int sdhci_bht_sdr50_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	return sdhci_msm_execute_tuning(host->mmc, opcode);
}

static int sd_tuning_sw(struct sdhci_host *host)
{
	int ret = 0;

	if (is_bus_mode_sdr104(host))
		ret = sdhci_bht_sdr104_execute_tuning(host, 0x13);
	else
		ret = sdhci_bht_sdr50_execute_tuning(host, 0x13);

	return ret;
}

static bool sd_gg_tuning_status(struct sdhci_host *host,
			 u32 *tx_selb, u32 *all_selb, u64 *raw_tx_selb,
			 bool *status_ret, bool *first_cmd19_status)
{
	bool ret = TRUE;
	int err = sd_tuning_sw(host);

	ret = err == 0 ? TRUE : FALSE;
	if (err == -ETIMEDOUT) {
		ret = FALSE;
		if (first_cmd19_status)
			*first_cmd19_status = false;
		goto exit;
	}

	if (status_ret) {
		*status_ret =
			gg_tuning_result(host, tx_selb, all_selb,
						raw_tx_selb);
	} else {
		gg_tuning_result(host, 0, 0, 0);
	}

exit:
	return ret;
}

static bool ggc_sd_tuning(struct sdhci_host *host,
			 bool *first_cmd19_status)
{
	bool ret = TRUE;
	int err = sd_tuning_sw(host);

	ret = err == 0 ? TRUE : FALSE;
	if (err == -ETIMEDOUT) {
		ret = FALSE;
		if (first_cmd19_status)
			*first_cmd19_status = false;
		goto exit;
	}

exit:
	return ret;
}

static int get_config_sela_setting(struct sdhci_host *host)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	if (is_bus_mode_sdr104(host))
		return vendor_host->ggc.def_sela_200m;
	else
		return vendor_host->ggc.def_sela_100m;
}

static int get_config_selb_setting(struct sdhci_host *host)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	if (is_bus_mode_sdr104(host))
		return vendor_host->ggc.def_selb_200m;
	else
		return vendor_host->ggc.def_selb_100m;
}

static void get_default_setting(struct sdhci_host *host, u8 *data)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	vendor_host->ggc.def_sela_100m =
		cfg_read_bits_ofs_mask(data, &vendor_host->ggc.dll_sela_100m_cfg);
	vendor_host->ggc.def_sela_200m =
		cfg_read_bits_ofs_mask(data, &vendor_host->ggc.dll_sela_200m_cfg);
	vendor_host->ggc.def_selb_100m =
		cfg_read_bits_ofs_mask(data, &vendor_host->ggc.dll_selb_100m_cfg);
	vendor_host->ggc.def_selb_200m =
		cfg_read_bits_ofs_mask(data, &vendor_host->ggc.dll_selb_200m_cfg);
}

static int get_left_one_sel(int base)
{
	if (base == 0)
		return 0xa;
	else
		return base - 1;
}

static int get_right_one_sel(int base)
{
	if (base == 0xa)
		return 0x0;
	else
		return base + 1;
}

static int update_selb(struct sdhci_host *host, int target_selb)
{
	return target_selb;
}

static int ggc_get_10case_0_index(u32 val)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(val, i) == 0
		    && GET_IDX_VALUE(val,
				     TRUNING_RING_IDX(i + TUNING_PHASE_SIZE -
						      1))) {
			return i;
		}
	}

	return -1;
}

static u32 ggc_get_01case_0_index(u32 val)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(val, i) == 0
		    && GET_IDX_VALUE(val, TRUNING_RING_IDX(i + 1))) {
			return i;
		}
	}

	return -1;
}

static int ggc_get_next_1_index(u32 val, int pos)
{
	int i = 0;

	pos = pos % TUNING_PHASE_SIZE;
	for (i = 0; i < TUNING_PHASE_SIZE; i++)	{
		if (GET_IDX_VALUE(val, (pos+i)%TUNING_PHASE_SIZE))
			break;
	}
	if (GET_IDX_VALUE(val, (pos+i)%TUNING_PHASE_SIZE))
		return (pos+i)%TUNING_PHASE_SIZE;
	else
		return -1;
}

static u32 ggc_get_01case_1_index(u32 val)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(val, i) == 0
			&& GET_IDX_VALUE(val, TRUNING_RING_IDX(i + 1))) {
			return TRUNING_RING_IDX(i + 1);
		}
	}

	return -1;
}

static int ggc_get_first_0_index(u32 val)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(val, i) == 0)
			return i;
	}
	pr_info("BHT MSG:oops-not find 0 index\n");
	return 0;
}

static int _tx_selb_inject_policy(int tx_selb, int org_selb)
{
	int group_pos[TUNING_PHASE_SIZE+1][3];
	int group_cnt = 0;
	int max_len_group = 0;
	int max_len = 0;
	int i, j, cnt;
	int zero_start, zero_end, sel;

	if ((org_selb & BIT_PASS_MASK) != BIT_PASS_MASK) {
		sel = tx_selb;
		zero_start = ggc_get_10case_0_index(sel);
		sel &=
		    ~GENERATE_TRUNING_RING_IDX_VALUE(get_left_one_sel
						     (zero_start));
		zero_end = ggc_get_01case_0_index(sel);
		sel &=
		    ~GENERATE_TRUNING_RING_IDX_VALUE(get_right_one_sel
						     (zero_end));
		if (sel != (sel & tx_selb)) {
			pr_err("BHT ERR:!!!=============\n\n\n");
			pr_err
			    ("tx selb reinject exception case :not adjacent phase\n");
			pr_err("BHT ERR:selb_failed range:%xh  ,new tx_selb:%x\n",
			       org_selb, tx_selb);
			pr_err("BHT ERR:\n\n!!!=============\n");
		}
		org_selb &= tx_selb;
	} else {
		cnt = ggc_get_selx_weight(~tx_selb);
		pr_info("BHT MSG:%d\n", cnt);
		switch (cnt) {
		case 1:
			i = ggc_get_first_0_index(tx_selb);
			tx_selb &=
			    ~GENERATE_TRUNING_RING_IDX_VALUE(get_right_one_sel
							     (i));
			tx_selb &=
			    ~GENERATE_TRUNING_RING_IDX_VALUE(get_left_one_sel
							     (i));

			break;
		case 2:
			i = ggc_get_10case_0_index(tx_selb);
			tx_selb &=
			    ~GENERATE_TRUNING_RING_IDX_VALUE(get_left_one_sel
							     (i));
			i = ggc_get_01case_0_index(tx_selb);
			tx_selb &=
			    ~GENERATE_TRUNING_RING_IDX_VALUE(get_right_one_sel
							     (i));
			break;
		default:
			pr_info("BHT MSG:>= 3 point case\n");
		}
		org_selb &= tx_selb;
	}

	pr_info("BHT MSG:will check continuous 0bits: 0x%x\n", org_selb);

	memset(group_pos, 0, sizeof(group_pos));
	for (i = ggc_get_01case_1_index(org_selb);
		i < TUNING_PHASE_SIZE && i >= 0 && group_cnt < TUNING_PHASE_SIZE;) {
		for (j = 1; j < TUNING_PHASE_SIZE; j++)	{
			if (GET_TRUNING_RING_IDX_VALUE(org_selb, i+j) != 0)
				continue;
			else
				break;
		}
		group_pos[group_cnt][0] = i;
		group_pos[group_cnt][1] = (i + j - 1) % TUNING_PHASE_SIZE;
		group_pos[group_cnt][2] = j;
		group_cnt++;
		if (group_pos[group_cnt-1][0] > group_pos[group_cnt-1][1])
			break;
		i = ggc_get_next_1_index(org_selb, (i+j)%TUNING_PHASE_SIZE);
		for (j = 0; j < group_cnt; j++)	{
			if (i == group_pos[j][0])
				break;
		}
		if (j < group_cnt)
			break;
	}

	if (group_cnt > 1) {
		pr_err("BHT ERR:After inject, selb 0x%x has %d continuous 0 bits\n",
				org_selb, group_cnt);

		for (i = 0; i < group_cnt; i++) {
			if (max_len < group_pos[i][2]) {
				max_len = group_pos[i][2];
				max_len_group = i;
			}
		}
		for (i = (group_pos[max_len_group][1] + 1) % TUNING_PHASE_SIZE;
			i != group_pos[max_len_group][0]; i = (i+1)%TUNING_PHASE_SIZE) {
			org_selb &= ~(1 << i);
		}
		pr_err("BHT ERR:After merge incontious 0 group, selb changed to 0x%x\n", org_selb);
	} else if (group_cnt > 0) {
		pr_err("BHT ERR:After merge incontious 0 group, selb = 0x%x\n", org_selb);
	} else {
		pr_err("BHT ERR:selb 0x%x has no bit is 0\n", org_selb);
	}

	return org_selb;
}

static void tx_selb_inject_policy(struct sdhci_host *host, int tx_selb)
{

	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	pr_info("BHT MSG:before inject, failed ragen 0x%x, tx_selb 0x%x\n",
			vendor_host->ggc.ggc_cmd_tx_selb_failed_range, tx_selb);
	vendor_host->ggc.ggc_cmd_tx_selb_failed_range =
		_tx_selb_inject_policy(tx_selb, vendor_host->ggc.ggc_cmd_tx_selb_failed_range);
	tx_selb_failed_history_update(host, vendor_host->ggc.ggc_cmd_tx_selb_failed_range);
	pr_info("BHT MSG:after inject %xh range:%xh\n", tx_selb,
		 vendor_host->ggc.ggc_cmd_tx_selb_failed_range);
	if (is_bus_mode_sdr104(host))
		vendor_host->ggc.sdr104.fail_phase = vendor_host->ggc.ggc_cmd_tx_selb_failed_range;
	else
		vendor_host->ggc.sdr50.fail_phase = vendor_host->ggc.ggc_cmd_tx_selb_failed_range;
}

static int get_selb_failure_point(int start, u64 raw_tx_selb, int tuning_cnt)
{
	int last = start + (tuning_cnt - 1);
	int i = 0;
	int j = 0;
	int phase = start;
	int vct = BIT_PASS_MASK;

	pr_info("BHT MSG:%s start:%d tuning_cnt:%d\n", __func__, start,
			 tuning_cnt);

	for (i = 0; i < tuning_cnt; i++) {
		if ((raw_tx_selb & GENERATE_64_IDX_VALUE(last - i)) == 0)
			break;
	}
	if (i == tuning_cnt) {
		phase = last % TUNING_PHASE_SIZE;
		vct &= (~(1 << phase));
		goto exit;
	}

	for (i = 0; i < tuning_cnt; i++) {
		if ((raw_tx_selb & GENERATE_64_IDX_VALUE(last - i)) != 0)
			break;
	}
	for (j = i - 2; j >= 0; j--)
		raw_tx_selb |= (1 << (last - j));

	for (j = 0; j < tuning_cnt; j++) {
		if (0 == (raw_tx_selb & GENERATE_64_IDX_VALUE(last - j)))
			vct &= (~(1 << (last-j)));
	}

exit:
	pr_info("BHT MSG:%s: after adjust raw_tx_selb: 0x%llx, vct 0x%x\n",
			 __func__, raw_tx_selb, vct);

	return vct;
}

static bool selx_failure_point_exist(u32 val)
{
	return (val & BIT_PASS_MASK) != BIT_PASS_MASK;
}

static int _bits_vct_get_left_index(int base)
{
	return TRUNING_RING_IDX(base + TUNING_PHASE_SIZE - 1);
}

static int _get_best_window_phase(u32 vct, int *pmax_pass_win, int shif_left_flg)
{
	u8 tuning_win[TUNING_PHASE_SIZE] = { 0 };
	u8 tuning_pass[TUNING_PHASE_SIZE];
	int tuning_pass_start[TUNING_PHASE_SIZE];
	int tuning_pass_num_max = 0;
	int first_0 = 0;
	int i = 0, j = 0;
	int i_mode = 0, selb_mode = 0;

	memset(tuning_pass, 0, sizeof(tuning_pass));
	memset(tuning_pass_start, 0, sizeof(tuning_pass_start));

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(vct, i))
			tuning_win[i] = 1;
		else
			tuning_win[i] = 0;
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (tuning_win[i] == 0) {
			first_0 = i;
			break;
		}
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		i_mode = TRUNING_RING_IDX(first_0 + i);
		if (tuning_win[i_mode] == 1)
			tuning_pass[j]++;
		else if (tuning_pass[j])
			j++;
		if (tuning_pass[j] == 1)
			tuning_pass_start[j] = i_mode;
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (tuning_pass_num_max < tuning_pass[i]) {
			tuning_pass_num_max = tuning_pass[i];
			i_mode = i;
		}
	}

	if (tuning_pass_num_max == 0) {
		pr_err("BHT ERR:###### Get max pass window fail, there is no any pass phase!!\n");
		selb_mode = 0;
	} else {
		if (tuning_pass_num_max % 2)	{
			selb_mode = tuning_pass_start[i_mode] + (tuning_pass_num_max - 1) / 2;
		} else {
			selb_mode = tuning_pass_start[i_mode] + (tuning_pass_num_max) / 2;
			if (shif_left_flg) {
				selb_mode = _bits_vct_get_left_index(selb_mode);
				pr_info("BHT MSG:shift left index\n");
			}
		}
		selb_mode = TRUNING_RING_IDX(selb_mode);
	}
	if (pmax_pass_win)
		*pmax_pass_win = tuning_pass_num_max;

	return selb_mode;
}

static int get_best_window_phase(u32 vct, int *pmax_pass_win)
{
	return _get_best_window_phase(vct, pmax_pass_win, 0);
}

static int _ggc_get_suitable_selb_for_next_tuning(struct sdhci_host *host)
{
	int selb = 0;
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	if (selx_failure_point_exist(vendor_host->ggc.ggc_cmd_tx_selb_failed_range)) {
		selb = vendor_host->ggc.ggc_sw_selb_tuning_first_selb;
	} else {
		u32 inj_tx_selb = BIT_PASS_MASK;

		pr_info("BHT MSG:manual inject for all pass case\n");
		if (is_bus_mode_sdr104(host))
			inj_tx_selb &= SDR104_MANUAL_INJECT;
		else
			inj_tx_selb &= SDR50_MANUAL_INJECT;

		pr_info("BHT MSG:manual inject for all pass case, inj_tx_selb=0x%x\n",
				inj_tx_selb);
		selb = get_best_window_phase(inj_tx_selb, NULL);
		pr_info("BHT MSG:select selb %d for all pass case\n", selb);
	}
	return selb;
}

static void ggc_reset_selx_failed_tb(struct sdhci_host *host)
{
	tx_selb_failed_tb_reset(host);
	all_selb_failed_tb_reset(host);
	tx_selb_failed_history_reset(host);
}

static void _ggc_reset_sela_tuning_result(struct sdhci_msm_host *host)
{
	int i = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++)
		host->ggc.ggc_sela_tuning_result[i] = NO_TUNING;
}

static void _ggc_reset_tuning_result_for_dll(struct sdhci_host *host)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	ggc_reset_selx_failed_tb(host);
	vendor_host->ggc.ggc_cmd_tx_selb_failed_range = BIT_PASS_MASK;
	vendor_host->ggc.selx_tuning_done_flag = 0;
	_ggc_reset_sela_tuning_result(vendor_host);
}

static int ggc_get_tuning_cnt_from_buffer(struct sdhci_host *host)
{
	int cnt = 0;
	u8 data[512] = { 0 };
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	get_gg_reg_cur_val(&vendor_host->ggc, data, 64);
	cnt = (int)cfg_read_bits_ofs_mask(data, &vendor_host->ggc.cmd19_cnt_cfg);

	pr_info("BHT MSG:tuning cnt=%d\n", cnt);
	return cnt;
}

static bool ggc_hw_inject_ext(struct sdhci_host *host, bool *card_status,
			u32 sel200, u32 sel100, bool writetobh201)
{
	bool ret = TRUE;
	u8 data[512] = { 0 };
	struct t_gg_reg_strt gg_reg_arr[10];
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	pr_info("BHT MSG:%s sel200:%xh,sel100:%xh\n", __func__, sel200, sel100);
	get_gg_reg_cur_val(&vendor_host->ggc, data, 64);
	memcpy(&gg_reg_arr[0], &vendor_host->ggc.inject_failure_for_tuning_enable_cfg,
				sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.inject_failure_for_200m_tuning_cfg,
				sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.inject_failure_for_100m_tuning_cfg,
				sizeof(struct t_gg_reg_strt));
	gg_reg_arr[0].value = 1;
	gg_reg_arr[1].value = sel200;
	gg_reg_arr[2].value = sel100;

	chg_gg_reg_cur_val(&vendor_host->ggc, data, gg_reg_arr, 3, TRUE);
	if (writetobh201)
		ret = gg_emulator_write_ext(host, card_status, data, 512);
	else {
		u32 i = 0;
		u32 reg;

		pr_info("BHT MSG:%s: dump config data instead write to bh201\n", __func__);
		for (i = 0; i < 16; i++) {
			memcpy(&reg, data+i*sizeof(u32), sizeof(u32));
			pr_info("BHT MSG:    ggc_reg32[%03d]=0x%08x\n", i, reg);
		}
	}
	return ret;
}

static bool _ggc_hw_inject_may_recursive(struct sdhci_host *host, u32 sel200,
			u32 sel100, int max_recur, bool writetobh201)
{
	bool ret = TRUE, card_status = TRUE;
	int selb = BIT_PASS_MASK;
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	ret = ggc_hw_inject_ext(host, &card_status, vendor_host->ggc.ggc_cmd_tx_selb_failed_range,
			vendor_host->ggc.ggc_cmd_tx_selb_failed_range, writetobh201);
	pr_info("BHT MSG:ret:%x\n", ret);
	if ((ret == FALSE) && (card_status == FALSE)) {
		pr_info("BHT MSG:inject again when hw inject\n");
		selb &= ~GENERATE_IDX_VALUE(vendor_host->ggc.ggc_sw_selb_tuning_first_selb);
		tx_selb_inject_policy(host, selb);
		_ggc_update_cur_setting_for_sw_selb_tuning(host,
			vendor_host->ggc.ggc_cmd_tx_selb_failed_range);

		if (((11 - get_bit_number(vendor_host->ggc.ggc_cmd_tx_selb_failed_range)) >= 5)) {
			pr_err("BHT ERR:pass windows too small,reinit recursive\n");
			return FALSE;
		}

		if (max_recur--)
			return _ggc_hw_inject_may_recursive(host,
			vendor_host->ggc.ggc_cmd_tx_selb_failed_range,
			vendor_host->ggc.ggc_cmd_tx_selb_failed_range, max_recur, writetobh201);
		else
			return FALSE;
	} else
		return TRUE;
}

static bool ggc_hw_inject_may_recursive(struct sdhci_host *host, u32 sel200,
			u32 sel100, bool writetobh201)
{
	return _ggc_hw_inject_may_recursive(host, sel200, sel100, 4, writetobh201);
}

static bool get_next_dll_voltage(int cur, int *next, u32 *dll_voltage_unlock_cnt,
			int *dll_voltage_scan_map)
{
	int min_idx = 0, cur_cnt = 0, next_cnt = 0;
	int cur_flg = 0;
	int i = 0;
	u8 ret = 0;

	pr_err("BHT ERR:dll_voltage_unlock_cnt:%x %x %x %x\n",
			dll_voltage_unlock_cnt[0], dll_voltage_unlock_cnt[1],
			dll_voltage_unlock_cnt[2], dll_voltage_unlock_cnt[3]);
	pr_err("BHT ERR:dll_voltage_scan_map:%x %x %x %x\n",
			dll_voltage_scan_map[0], dll_voltage_scan_map[1],
			dll_voltage_scan_map[2], dll_voltage_scan_map[3]);
	for (i = 1; i < 4; i++) {
		if (cur_flg == 0) {
			if (dll_voltage_scan_map[(cur + i) % 4] != 0)
				continue;
			cur_cnt = dll_voltage_unlock_cnt[(cur + i) % 4];
			cur_flg = 1;
			min_idx = (cur + i) % 4;
			continue;
		} else {
			if (dll_voltage_scan_map[(cur + i) % 4] != 0)
				continue;
			next_cnt = dll_voltage_unlock_cnt[(cur + i) % 4];
			if (cur_cnt > next_cnt) {
				cur_cnt = next_cnt;
				min_idx = (cur + i) % 4;
			}
		}
	}
	if (cur_flg == 0) {
		pr_err("BHT ERR:no find available voltage\n");
		ret = FALSE;
	} else {
		*next = min_idx;
		pr_err("BHT ERR:next available voltage %d\n", min_idx);
		ret = TRUE;
	}
	return ret;
}

static bool ggc_sw_calc_tuning_result(struct sdhci_host *host, bool *card_status,
			       bool *read_status, u32 *tx_selb, u32 *all_selb, u64 *raw_tx_selb)
{
	bool ret = FALSE;
	bool card_ret = FALSE;
	bool read_ret = FALSE;
	u32 selb_status_tx_low32 = 0, selb_status_tx_high32 = 0;
	u32 selb_status_ggc_low32 = 0, selb_status_ggc_high32 = 0;
	struct t_gg_reg_strt gg_reg_arr[8] = {{0}};
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	memcpy(&gg_reg_arr[0], &vendor_host->ggc.pha_stas_tx_low32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[1], &vendor_host->ggc.pha_stas_tx_high32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[2], &vendor_host->ggc.pha_stas_rx_low32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[3], &vendor_host->ggc.pha_stas_rx_high32, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[4], &vendor_host->ggc.dll_sela_after_mask, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[5], &vendor_host->ggc.dll_selb_after_mask, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[6], &vendor_host->ggc.dll_delay_100m_backup, sizeof(struct t_gg_reg_strt));
	memcpy(&gg_reg_arr[7], &vendor_host->ggc.dll_delay_200m_backup, sizeof(struct t_gg_reg_strt));

	ret = ggc_read_registers_ext(host, &card_ret, &read_ret, gg_reg_arr, 8);
	if (read_ret == TRUE) {
		selb_status_tx_low32 = gg_reg_arr[0].value;
		pr_info("BHT MSG:[205-236]:\n");
		log_bin(selb_status_tx_low32);
		pr_info("BHT MSG:[237-268]:\n");
		selb_status_tx_high32 = gg_reg_arr[1].value;
		log_bin(selb_status_tx_high32);

		pr_info("BHT MSG:[14-45]:\n");
		selb_status_ggc_low32 = gg_reg_arr[2].value;
		log_bin(selb_status_ggc_low32);
		pr_info("BHT MSG:[46-77]:\n");
		selb_status_ggc_high32 = gg_reg_arr[3].value;
		log_bin(selb_status_ggc_high32);
		pr_info("BHT MSG:dll  sela after mask=%xh", gg_reg_arr[4].value);
		pr_info("BHT MSG:dll  selb after mask=%xh", gg_reg_arr[5].value);

		if (raw_tx_selb) {
			*raw_tx_selb = gg_reg_arr[1].value;
			(*raw_tx_selb) <<= 32;
			*raw_tx_selb += gg_reg_arr[0].value;
			pr_info("BHT MSG:raw_tx_selb:%llxh\n", *raw_tx_selb);
		}

		if (tx_selb) {
			gen_array_data(gg_reg_arr[0].value, gg_reg_arr[1].value,
				       tx_selb);
			pr_info("BHT MSG:tx_selb:%xh\n", *tx_selb);
		}
		if (all_selb) {
			gen_array_data(gg_reg_arr[2].value, gg_reg_arr[3].value,
				       all_selb);
			pr_info("BHT MSG:all_selb:%xh\n", *all_selb);
		}
	}

	if (read_status)
		(*read_status) = read_ret;
	if (card_status)
		(*card_status) = card_ret;

	if (card_status && read_status)
		pr_info("BHT MSG:card_status,read_status:%x %x\n", *card_status, *read_status);
	return ret;
}

static bool _ggc_calc_cur_sela_tuning_result(struct sdhci_host *host, int cur_sela, int start_selb)
{
	bool read_status = FALSE;
	bool card_status = FALSE;
	bool ret = TRUE;
	u32 tx_selb, all_selb;
	u64 raw_tx_selb = 0;
	bool retuning_flg = FALSE;
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	int selb;
	struct ggc_platform_t *ggc = &vendor_host->ggc;
	enum tuning_stat_et *psela_tuning_result = ggc->ggc_sela_tuning_result;

	ret = ggc_sw_calc_tuning_result(host, &card_status,
			&read_status, &tx_selb, &all_selb, &raw_tx_selb);

	if (card_status == FALSE) {
		if (read_status == TRUE) {
			selb = get_selb_failure_point(start_selb, raw_tx_selb,
					ggc_get_tuning_cnt_from_buffer(host));
			pr_info("BHT MSG:inject selb %03x for CMD7 read timeout\n", selb);
			tx_selb_inject_policy(host, selb);
		} else {
			pr_info("BHT MSG:read status failedA!!\n");
			pr_info("BHT MSG:============ %s dll:%xh failed ============\n",
					__func__, cur_sela);
		}
		ret = FALSE;
		goto exit;
	} else {
		if (read_status == TRUE) {
			if (selx_failure_point_exist(tx_selb)) {
				if ((11-get_bit_number(tx_selb)) <= 3) {
					tx_selb_inject_policy(host, tx_selb);
					all_selb_failed_tb_update(host, cur_sela, all_selb);
					tx_selb_failed_tb_update(host, cur_sela, tx_selb);
					tx_selb_failed_history_update(host, tx_selb);
				} else if (get_bit_number(tx_selb) == 0) {
					selb = get_selb_failure_point(start_selb, raw_tx_selb,
							ggc_get_tuning_cnt_from_buffer(host));
					tx_selb_inject_policy(host, selb);
					all_selb_failed_tb_update(host, cur_sela, all_selb);
					tx_selb_failed_tb_update(host, cur_sela, selb);
					tx_selb_failed_history_update(host, selb);
					retuning_flg = TRUE;
				} else {
					tx_selb_inject_policy(host, tx_selb);
					all_selb_failed_tb_update(host, cur_sela, all_selb);
					tx_selb_failed_tb_update(host, cur_sela, tx_selb);
					tx_selb_failed_history_update(host, tx_selb);
					retuning_flg = TRUE;
				}

				_ggc_update_cur_setting_for_sw_selb_tuning(host,
						ggc->ggc_cmd_tx_selb_failed_range);
				ggc_hw_inject_may_recursive(host, ggc->ggc_cmd_tx_selb_failed_range,
						ggc->ggc_cmd_tx_selb_failed_range, TRUE);
			} else {
				all_selb_failed_tb_update(host, cur_sela, all_selb);
				tx_selb_failed_tb_update(host, cur_sela, tx_selb);
				tx_selb_failed_history_update(host, tx_selb);
			}

			if (retuning_flg == TRUE) {
				pr_info("BHT MSG:== %s dll:%xh need retuning ==\n",
						__func__, cur_sela);
				psela_tuning_result[cur_sela] = RETUNING_CASE;
			} else {
				pr_info("BHT MSG:== %s dll:%xh pass ==\n",
						__func__, cur_sela);
				psela_tuning_result[cur_sela] = OUTPUT_PASS_TYPE;
			}
		} else {
			pr_info("BHT MSG:read status failed!!\n");
			psela_tuning_result[cur_sela] = READ_STATUS_FAIL_TYPE;
			all_selb_failed_tb_update(host, cur_sela, 0);
			pr_info("BHT MSG:== %s dll:%xh failed ==\n",
					__func__, cur_sela);
		}
	}
exit:
	return ret;
}

static bool _ggc_output_tuning(struct sdhci_host *host, u8 *selb_pass_win)
{
	int cur_sela = 0, dll_sela_cnt = 0;
	int dll_sela_basis = 0;
	bool ret = FALSE;
	u8 win_tb[12] = { 0 };
	u8 win_mid = 0;
	u8 win_max = 0;
	u32 tx_tmp = 0;
	int target_sela = 0;
	int target_selb = 0;
	u32 all_sela, tx_selb, all_selb;
	u64 raw_tx_selb;
	bool status_ret = FALSE;
	int cur_selb = 0;
	int tuning_error_type[16] = { 0 };
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	struct ggc_platform_t *ggc = &vendor_host->ggc;
	enum tuning_stat_et *psela_tuning_result = ggc->ggc_sela_tuning_result;
	int i = 0;
	u32 idx_r, idx_c;
	u32 min_pos = 0;
	u32 all_selb_ar[TUNING_PHASE_SIZE] = { 0 };
	u32 pass_cnt[TUNING_PHASE_SIZE] = { 0 };
	u32 cfg = 0;
	int rescan_selb_cnt = 0;
	int returning_selb_cnt = 0;
	bool first_cmd19_sta = TRUE;
	int next = 0;
	bool card_status = TRUE;
	int selb =	BIT_PASS_MASK;
	u8 all_str[TUNING_PHASE_SIZE + 1],tx_str[TUNING_PHASE_SIZE + 1];

	vendor_host->ggc.driver_strength_reinit_flg = 0;
	vendor_host->ggc.dll_unlock_reinit_flg = 0;
	vendor_host->ggc.sdr50_notuning_crc_error_flag = 0;
	dll_sela_basis = get_config_sela_setting(host);
	cur_selb = get_config_selb_setting(host);

	{
		if(wait_dll_lock(host))
			pr_err("BHT MSG:wait for DLL lock failed\n");
		bht_mmc_send_io_op_cond(host->mmc,0,NULL);
	}

	if (ggc->tuning_cmd7_timeout_reinit_flg) {
		ggc->tuning_cmd7_timeout_reinit_flg = 0;
		dll_sela_basis = vendor_host->ggc.ggc_cur_sela;
		cur_selb = vendor_host->ggc.ggc_sw_selb_tuning_first_selb;
		pr_info
		("BHT MSG:Tuning start sela: 0x%x, selb: 0x%x where CMD7 timeout\n",
				dll_sela_basis, cur_selb);
	}

	for (dll_sela_cnt = 0; dll_sela_cnt < TUNING_PHASE_SIZE; dll_sela_cnt++) {
		rescan_selb_cnt = 0;
		returning_selb_cnt = 0;
		cur_sela =
				(dll_sela_cnt + dll_sela_basis) % TUNING_PHASE_SIZE;
		ggc->ggc_cur_sela = cur_sela;
		pr_info("BHT MSG:== %s select sela dll: %x, selb dll: %x ==\n",
				__func__, cur_sela, cur_selb);
		if (psela_tuning_result[cur_sela] != NO_TUNING) {
			pr_info("BHT MSG:sela tuning=%d already tuning,so skip it\n", cur_sela);
			continue;
		}
rescan_selb:
		host_cmddat_line_reset(host);

		if (dll_sela_cnt == 0) {
			if (!selx_failure_point_exist
					(vendor_host->ggc.ggc_cmd_tx_selb_failed_range)) {
				rescan_selb_cnt = 3;
				pr_info("BHT MSG:no need rescan case\n");
			}
			status_ret = FALSE;
			ret = ggc_sd_tuning(host, &first_cmd19_sta);

			if (first_cmd19_sta == FALSE) {
				next = 0;
				_check_bus_mode(host);
				ggc->cur_bus_mode->dll_voltage_unlock_cnt
							[ggc->cur_dll_voltage_idx]++;
				ggc->dll_voltage_scan_map[ggc->cur_dll_voltage_idx] = 1;
				if (get_next_dll_voltage(ggc->cur_dll_voltage_idx, &next,
					ggc->cur_bus_mode->dll_voltage_unlock_cnt,
					ggc->dll_voltage_scan_map) == TRUE)
					ggc->cur_dll_voltage_idx = next;
				else
					ggc->cur_dll_voltage_idx =
							(ggc->cur_dll_voltage_idx + 1) % 4;

				pr_err("BHT ERR:first cmd19 timeout\n");
				vendor_host->ggc.dll_unlock_reinit_flg = 1;
				_ggc_reset_tuning_result_for_dll(host);
				ret = FALSE;
				goto exit;
			}
		} else if ((is_bus_mode_sdr104(host) == FALSE)
				&& vendor_host->ggc.sdr50_notuning_sela_inject_flag == 1
				&& !GET_IDX_VALUE(vendor_host->ggc.sdr50_notuning_sela_rx_inject,
				cur_sela)) {
			pr_info("BHT MSG:skip %d\n", cur_sela);
			tuning_error_type[cur_sela] = READ_STATUS_FAIL_TYPE;
			goto cur_sela_failed;
		} else {
			ret = ggc_set_output_tuning_phase_ext(host, &card_status,
				cur_sela, update_selb(host, cur_selb));
			if (ret == FALSE || card_status == FALSE) {
				pr_err
				("BHT ERR: output_tuning fail at phase %d,ret=%d,card_status=%d\n",
						cur_sela, ret, card_status);
				if (card_status == FALSE) {
					selb = BIT_PASS_MASK;
					selb &= ~GENERATE_IDX_VALUE(cur_selb);
					pr_err("BHT ERR:inject selb %d for update sela/selb fail\n",
							selb);
					tx_selb_inject_policy(host, selb);
					_ggc_update_cur_setting_for_sw_selb_tuning(host,
						ggc->ggc_cmd_tx_selb_failed_range);
					ggc_hw_inject_may_recursive(host,
							ggc->ggc_cmd_tx_selb_failed_range,
							ggc->ggc_cmd_tx_selb_failed_range, TRUE);

					if (((11 - get_bit_number(
						ggc->ggc_cmd_tx_selb_failed_range))	>= 5)) {
						u8 current_ds = (u8)(ggc->_gg_reg_cur[15] >> 28);

						pr_err("BHT ERR:pass windows too small\n");

						if(vendor_host->ggc.tuning_fail_count == 0)
						{
							if (current_ds < 7)
								vendor_host->ggc.driver_strength_reinit_flg = 6;

							else
								vendor_host->ggc.driver_strength_reinit_flg = 7;

							ggc->_gg_reg_cur[15] &= 0x0F0FFFFF;
							ggc->_gg_reg_cur[15] |=
							(vendor_host->ggc.driver_strength_reinit_flg << 28)
							| (vendor_host->ggc.driver_strength_reinit_flg << 20);
							pr_err("BHT ERR:will change driver strength from %d to %d\n",
							current_ds,
							vendor_host->ggc.driver_strength_reinit_flg);
						}
						else if(is_bus_mode_sdr104(host))
						{
							vendor_host->ggc.degrade = 1;
							host->mmc->caps &= ~(MMC_CAP_UHS_DDR50 | MMC_CAP_UHS_SDR104);
							pr_info("BHT MSG: disable SDR104 and DDR50, caps 0x%x\n",host->mmc->caps);
						} else
						{
							vendor_host->ggc.degrade = 1;
							host->mmc->caps &= ~(MMC_CAP_UHS);
							pr_info("BHT MSG: disable UHS mode, caps 0x%x\n",host->mmc->caps);
						}
						ret = FALSE;
						goto exit;
					}
					cur_selb = _ggc_get_suitable_selb_for_next_tuning(host);
				}
				psela_tuning_result[cur_sela] = RETUNING_CASE;
				goto retuning_case;
			}
			ret = ggc_sd_tuning(host, NULL);
		}

		if (ret == FALSE) {
			pr_err("BHT ERR:Error when output_tuning, fail at phase %d\n",
					cur_sela);
			psela_tuning_result[cur_sela] = TUNING_FAIL_TYPE;
			all_selb_failed_tb_update(host, cur_sela, 0);
			continue;
		}

		ret = _ggc_calc_cur_sela_tuning_result(host, cur_sela, cur_selb);

		if ((11 - get_bit_number(vendor_host->ggc.ggc_cmd_tx_selb_failed_range)) >= 5)
		{
			u8 current_ds = (u8)(ggc->_gg_reg_cur[15] >> 28);

			pr_err("BHT ERR:pass windows too small after result calculate, reinit\n");

			if(vendor_host->ggc.tuning_fail_count == 0)
			{
				if (current_ds < 7)
					vendor_host->ggc.driver_strength_reinit_flg = 6;

				else
					vendor_host->ggc.driver_strength_reinit_flg = 7;

				ggc->_gg_reg_cur[15] &= 0x0F0FFFFF;
				ggc->_gg_reg_cur[15] |=
					(vendor_host->ggc.driver_strength_reinit_flg << 28)
					| (vendor_host->ggc.driver_strength_reinit_flg << 20);

				pr_err("BHT ERR:will change driver strength from %d to %d\n",
							current_ds,
							vendor_host->ggc.driver_strength_reinit_flg);
			}
			else if(is_bus_mode_sdr104(host))
			{
				vendor_host->ggc.degrade = 1;
				host->mmc->caps &= ~(MMC_CAP_UHS_DDR50 | MMC_CAP_UHS_SDR104);
				pr_info("BHT MSG: disable SDR104 and DDR50, caps 0x%x\n",host->mmc->caps);
			} else
			{
				vendor_host->ggc.degrade = 1;
				host->mmc->caps &= ~(MMC_CAP_UHS);
				pr_info("BHT MSG: disable UHS mode, caps 0x%x\n",host->mmc->caps);
			}
			ret = FALSE;
			goto exit;
		}

		if (ret == FALSE) {
			pr_err("BHT ERR:cmd7 timeout fail,reinit\n");
			vendor_host->ggc.tuning_cmd7_timeout_reinit_flg = 1;

			_ggc_update_cur_setting_for_sw_selb_tuning(host,
							ggc->ggc_cmd_tx_selb_failed_range);
			ggc_hw_inject_may_recursive(host, ggc->ggc_cmd_tx_selb_failed_range,
							ggc->ggc_cmd_tx_selb_failed_range, FALSE);
			if ((11 - get_bit_number(vendor_host->ggc.ggc_cmd_tx_selb_failed_range))
										>= 5) {
				u8 current_ds = (u8)(ggc->_gg_reg_cur[15] >> 28);

				pr_err("BHT ERR:pass windows too small, driver strength reinit\n");

				vendor_host->ggc.tuning_cmd7_timeout_reinit_flg = 0;

				if(vendor_host->ggc.tuning_fail_count == 0)
				{
					if (current_ds < 7)
						vendor_host->ggc.driver_strength_reinit_flg = 6;

					else
						vendor_host->ggc.driver_strength_reinit_flg = 7;

					ggc->_gg_reg_cur[15] &= 0x0F0FFFFF;
					ggc->_gg_reg_cur[15] |=
						(vendor_host->ggc.driver_strength_reinit_flg << 28)
						| (vendor_host->ggc.driver_strength_reinit_flg << 20);
					pr_err("BHT ERR:will change driver strength from %d to %d\n",
							current_ds,
							vendor_host->ggc.driver_strength_reinit_flg);
				}
				else if(is_bus_mode_sdr104(host))
				{
					vendor_host->ggc.degrade = 1;
					host->mmc->caps &= ~(MMC_CAP_UHS_DDR50 | MMC_CAP_UHS_SDR104);
					pr_info("BHT MSG: disable SDR104 and DDR50, caps 0x%x\n",host->mmc->caps);
				} else
				{
					vendor_host->ggc.degrade = 1;
					host->mmc->caps &= ~(MMC_CAP_UHS);
					pr_info("BHT MSG: disable UHS mode, caps 0x%x\n",host->mmc->caps);
				}
				ret = FALSE;
				goto exit;
			}
			goto exit;
		}

		cur_selb = _ggc_get_suitable_selb_for_next_tuning(host);

		pr_info("BHT MSG:===ot  sela:%xh pass ===\n", cur_sela);
		rescan_selb_cnt++;
		if ((rescan_selb_cnt < 3) &&
			(selx_failure_point_exist(vendor_host->ggc.ggc_cmd_tx_selb_failed_range))) {
			pr_info("BHT MSG:rescan cnt %d, ggc_cmd_tx_selb_failed_range=0x%x\n",
						rescan_selb_cnt,
						vendor_host->ggc.ggc_cmd_tx_selb_failed_range);
			goto rescan_selb;
		}

retuning_case:
		if (psela_tuning_result[cur_sela] == RETUNING_CASE) {
			returning_selb_cnt++;
			if (returning_selb_cnt < 3) {
				rescan_selb_cnt = 0;
				pr_info("BHT MSG:retuning %d\n", rescan_selb_cnt);
				goto rescan_selb;
			} else {
				psela_tuning_result[cur_sela] = SET_PHASE_FAIL_TYPE;
				all_selb_failed_tb_update(host, cur_sela, 0);
				continue;
			}
		}

		goto next_dll_sela;

cur_sela_failed:
		pr_info("BHT MSG:read status failedB\n");
		all_selb = 0;
		all_selb_failed_tb_update(host, cur_sela, all_selb);
		pr_info("BHT MSG:===ot  sela:%xh failed ===\n", cur_sela);
next_dll_sela:
		if ((is_bus_mode_sdr104(host) == FALSE)
				&& vendor_host->ggc.sdr50_notuning_crc_error_flag) {
			u32 fp = 0;

			fp = GENERATE_IDX_VALUE(cur_sela);
			fp |= GENERATE_IDX_VALUE((cur_sela + 1) % TUNING_PHASE_SIZE);
			fp |= GENERATE_IDX_VALUE((cur_sela + 10) % TUNING_PHASE_SIZE);
			vendor_host->ggc.sdr50_notuning_sela_rx_inject &= ~fp;
			vendor_host->ggc.sdr50_notuning_sela_inject_flag = 1;
			pr_info("BHT MSG:sdr50_notuning_sela_rx_inject:%x\n",
						vendor_host->ggc.sdr50_notuning_sela_rx_inject);
			ret = FALSE;
			goto exit;
		};
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		phase_str(all_str, all_selb_failed_tb_get(host, i));
		phase_str(tx_str, tx_selb_failed_tb_get(host, i));
		pr_info
				("BHT MSG:DLL sela[%x]  all selb: %s   tx selb: %s [%xh,%xh] %s\n",
					i, all_str, tx_str,
					all_selb_failed_tb_get(host, i),
					tx_selb_failed_tb_get(host, i),
					op_dbg_str[tuning_error_type[i]]);

	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++)
		all_selb_ar[i] = all_selb_failed_tb_get(host, i);

	for (idx_c = 0; idx_c < TUNING_PHASE_SIZE; idx_c++) {
		for (idx_r = 0; idx_r < TUNING_PHASE_SIZE;
					idx_r++) {
			pass_cnt[idx_c] +=	((all_selb_ar[idx_r] >>
					((idx_r + idx_c) % TUNING_PHASE_SIZE)) & 0x01);
		}
		if (idx_c == 0)
			min_pos = 0;
		else if (pass_cnt[idx_c] < pass_cnt[min_pos])
			min_pos = idx_c;
	}
	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		all_selb_ar[i] &= ~(1 << (min_pos + i) % TUNING_PHASE_SIZE);
		all_selb_failed_tb_update(host, i, all_selb_ar[i]);
	}

	tx_selb = tx_selb_failed_history_get(host);

	pr_info("inject sw selb & merge tx_selb failed point to all_selb\n");
	for (i = 0; i < TUNING_PHASE_SIZE; i++)
		all_selb_failed_tb_update(host, i, tx_selb);

	pr_info("BHT MSG:inject sw sela failed point to all_selb\n");
	if (is_bus_mode_sdr104(host))
		cfg = 0x7ff;
	else
		cfg = 0x7ff;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		if (GET_IDX_VALUE(cfg, i) == 0)
			all_selb_failed_tb_update(host, i, 0);
	}

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		phase_str(all_str, all_selb_failed_tb_get(host, i));
		phase_str(tx_str, tx_selb_failed_tb_get(host, i));
		pr_info("BHT MSG:DLL sela[%x]  all selb: %s tx selb: %s [%xh,%xh] %s\n",
				i, all_str, tx_str,
				all_selb_failed_tb_get(host, i),
				tx_selb_failed_tb_get(host, i),
				op_dbg_str[tuning_error_type[i]]);
	}

	tx_selb = tx_selb_failed_history_get(host);
	tx_selb &= 0x7ff;
	tx_tmp = tx_selb;

	pr_info("BHT MSG:---selb merge---\n");
	if ((tx_selb&0x7ff) == 0x7ff) {
		if (is_bus_mode_sdr104(host)) {
			u32 cfg = SDR104_MANUAL_INJECT;

			tx_selb &= cfg;
			pr_info("tx selb:%xh SDR104 inject:%xh merge tx_selb:%xh\n",
						tx_tmp, cfg, tx_selb);
		} else {
			u32 cfg = SDR50_MANUAL_INJECT;

			tx_selb &= cfg;
			pr_info("tx selb:%xh SDR50 inject:%xh merge tx_selb:%xh\n",
						tx_tmp, cfg, tx_selb);
		}
	}

	if (tx_selb == 0) {
		pr_err("all failed, force fixed phase sela selb to default\n");
		target_selb =
				get_config_selb_setting(host);
		target_sela =
				get_config_sela_setting(host);
		goto final;
	}
	phase_str(win_tb, tx_selb);
	pr_info("BHT MSG:######### tx selb[%xh] 11 bit: %s\n",
						tx_selb, win_tb);
	bits_generate_array(win_tb, tx_selb);
	chk_phase_window(win_tb, &win_mid, &win_max);
	target_selb = win_mid;

	all_sela = 0;

	for (i = 0; i < TUNING_PHASE_SIZE; i++) {
		u32 all_selb = all_selb_failed_tb_get(host, i);

		phase_str(win_tb, all_selb);
		pr_info("######### all_selb[%xh] 11 bit: %s\n",
				all_selb, win_tb);
		bits_generate_array(win_tb, all_selb);
		chk_phase_window(win_tb, &win_mid,
					&win_max);
		*selb_pass_win = win_max;
		if (all_selb & (1 << target_selb))
			all_sela |= 1 << i;
	}

	phase_str(win_tb, all_sela);
	pr_info("######### all_sela[%xh] 11 bit: %s\n",
			all_sela, win_tb);
	bits_generate_array(win_tb, all_sela);
	chk_phase_window(win_tb, &win_mid, &win_max);
	target_sela = win_mid;

final:

	gg_fix_output_tuning_phase(host, target_sela,
						target_selb);

	ret = sd_gg_tuning_status(host, &tx_selb,
			&all_selb, &raw_tx_selb, &status_ret, NULL);
	if (ret == FALSE) {
		pr_err("Error when output_tuning,  sd_tuning fail\n");
		ret = FALSE;
		goto exit;
	}

	phase_str(win_tb, all_selb);
	pr_info("######### all_selb[%xh] 11 bit: %s\n",
			all_selb, win_tb);
	bits_generate_array(win_tb, all_selb);
	chk_phase_window(win_tb, &win_mid, &win_max);
	*selb_pass_win = win_max;

	vendor_host->ggc.selx_tuning_done_flag = 1;

exit:
	pr_info("BHT MSG:exit:%s  %d\n", __func__, ret);
	return ret;
}

static void ggc_tuning_result_reset(struct sdhci_host *host)
{
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);

	pr_info("BHT MSG:%s: will clear all tuning results\n", __func__);
	_ggc_reset_tuning_result_for_dll(host);

	vendor_host->ggc.sdr50.bus_mode = SD_FNC_AM_SDR50;
	vendor_host->ggc.sdr104.bus_mode = SD_FNC_AM_SDR104;
	vendor_host->ggc.driver_strength_reinit_flg = 0;
	vendor_host->ggc.cur_bus_mode = NULL;
	vendor_host->ggc.dll_unlock_reinit_flg = 0;
	vendor_host->ggc.dll_unlock_reinit_flg = 0;
	vendor_host->ggc.tuning_cmd7_timeout_reinit_flg = 0;
	vendor_host->ggc.tuning_cmd7_timeout_reinit_cnt = 0;
	vendor_host->ggc.sdr50_notuning_sela_inject_flag = 1;
	vendor_host->ggc.sdr50_notuning_crc_error_flag = 0;
	vendor_host->ggc.degrade = 0;

	if (vendor_host->ggc.bh201_sdr50_sela_sw_inject)
		vendor_host->ggc.sdr50_notuning_sela_rx_inject =
						vendor_host->ggc.bh201_sdr50_sela_sw_inject;
	else
		vendor_host->ggc.sdr50_notuning_sela_rx_inject = 0x47F;
}

static void ggc_chip_init(struct sdhci_host *host)
{
	u8 data[512] = { 0 };
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	if (init_flg == 0) {
		ggc_tuning_result_reset(host);
		get_gg_reg_def(host, data);
		get_default_setting(host, data);
		set_gg_reg_cur_val(&vendor_host->ggc, data, 64);
		init_flg = 1;
	}
}

static int sdhci_bht_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct sdhci_host *host = get_sdhci_host(mmc);
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	u8 tw = 0;
	int ret = 0;

	if (vendor_host->ggc.bh201_used) {
		pr_info("BHT MSG:enter bht tuning\n");
		if (host->clock < CORE_FREQ_100MHZ) {
			pr_info("BHT MSG:%d less 100Mhz,no tuning\n", host->clock);
			goto exit;
		}

		if (vendor_host->ggc.tuning_in_progress) {
			pr_info("BHT MSG:tuning_in_progress\n");
			goto exit;
		}
		vendor_host->ggc.tuning_in_progress = true;

		if(vendor_host->ggc.origin_cap == 0)
			vendor_host->ggc.origin_cap = host->mmc->caps;

		if (vendor_host->ggc.selx_tuning_done_flag) {
			pr_info("BHT MSG:GGC tuning is done, just do vendor host tuning\n");
			if (is_bus_mode_sdr104(host))
				ret = sdhci_bht_sdr104_execute_tuning(host, 0x13);
			else
				ret = sdhci_bht_sdr50_execute_tuning(host, 0x13);

			if(ret)
			{
				pr_info("BHT MSG:GGC tuning is done, but do vendor host tuning failed\n");
				ggc_tuning_result_reset(host);
			}
		} else {
			ret = !_ggc_output_tuning(host, &tw);
		}
		vendor_host->ggc.tuning_in_progress = false;

	} else
		ret = sdhci_msm_execute_tuning(mmc, opcode);
	pr_info("BHT MSG:GGC tuning_fail_count %d\n",vendor_host->ggc.tuning_fail_count);
	if(vendor_host->ggc.tuning_fail_count >= 2)
	{
		vendor_host->ggc.degrade = 1;
		host->mmc->caps &= ~(MMC_CAP_UHS);
		pr_info("BHT MSG: disable UHS mode, caps 0x%x\n",host->mmc->caps);
	}
	if(ret)
		vendor_host->ggc.tuning_fail_count++;
	else
		vendor_host->ggc.tuning_fail_count = 0;

	if(ret && vendor_host->ggc.tuning_fail_count < 5) {

		pr_info("BHT MSG: finit is %dHz\n",host->mmc->f_init);
		if (host->ops->get_min_clock && host->ops->get_min_clock(host) == host->mmc->f_init)
        {
			pr_info("BHT MSG: Invoke card init from bayhub tuning routine\n");
			queue_delayed_work(system_freezable_wq, &mmc->detect, HZ);
		}
	}

exit:

	vendor_host->ggc.card = NULL;
	return ret;
}

static void bht_load(struct mmc_host *mmc_host, struct mmc_card *card)
{
	static const int s_dll_voltage_cfg[4][2] = {
		{0x30503106, 0x64141711},
		{0x31503106, 0x64141711},
		{0x30503106, 0x64141751},
		{0x31503106, 0x64141751},
	};

	struct sdhci_host *host = get_sdhci_host(mmc_host);
	struct sdhci_msm_host *vendor_host = get_vendor_host(host);
	u32 i = 0;
	u32 reg;
	u32 gg_sw_def[16] = GGC_CFG_DATA;
	u8 data[512] = { 0 };

	if (vendor_host->ggc.bh201_used) {
		pr_info("%s: Load BHT patch\n", mmc_hostname(mmc_host));
		if (vendor_host->ggc.degrade) {
			ggc_tuning_result_reset(host);
		}

		bht_mmc_send_relative_addr(mmc_host, &card->rca);

		vendor_host->ggc.card = card;

		if(memcmp(g_card_cid, card->raw_cid, sizeof(g_card_cid)) != 0)
		{
			pr_info("card cid is different, card is changed\n");
			ggc_tuning_result_reset(host);
			memcpy(g_card_cid,card->raw_cid,sizeof(g_card_cid));
		}

		if (vendor_host->ggc.dll_unlock_reinit_flg) {
			int cur_dll_voltage_idx = vendor_host->ggc.cur_dll_voltage_idx;

			pr_info("update dll voltage cfg for dll unlock reinit flg: idx=%d\n",
					cur_dll_voltage_idx);
			ggc_tuning_result_reset(host);
			gg_sw_def[8] = s_dll_voltage_cfg[cur_dll_voltage_idx][0];
			gg_sw_def[9] = s_dll_voltage_cfg[cur_dll_voltage_idx][1];
		}
		if (vendor_host->ggc.driver_strength_reinit_flg) {
			u8 driver_strength_reinit_flg = vendor_host->ggc.driver_strength_reinit_flg;

			pr_info("%s: driver strength should be init to %d\n",
				mmc_hostname(mmc_host), driver_strength_reinit_flg);
			ggc_tuning_result_reset(host);
			if (vendor_host->ggc.driver_strength_reinit_flg <= 7) {
				gg_sw_def[15] &= 0x0f0fffff;
				gg_sw_def[15] |= (driver_strength_reinit_flg << 28)
						| (driver_strength_reinit_flg << 20);
			}
		} else {
			if (vendor_host->ggc.bh201_drive_strength)
				gg_sw_def[15] = vendor_host->ggc.bh201_drive_strength;
		}
		driver_send_command(host);
		if (vendor_host->ggc.tuning_cmd7_timeout_reinit_flg == 0
					&& vendor_host->ggc.selx_tuning_done_flag == 0) {
			pr_debug("gg_sw_def[8] = 0x%08x, gg_sw_def[9]=0x%08x, gg_sw_def[15]=%08x\n",
					gg_sw_def[8], gg_sw_def[9], gg_sw_def[15]);
			if (bht_load_hw_inject(mmc_host, card, gg_sw_def,
					sizeof(gg_sw_def), 0x3ff, 0x77f)) {
				pr_info("%s: %s: bht_load_hw_inject fail, exit bh201 init. \n",
					mmc_hostname(mmc_host), __func__);
				goto out;
			}
			if (bht_update_cfg(mmc_host, card, gg_sw_def, sizeof(gg_sw_def))) {
				pr_info("%s: %s: bht_update_cfg fail, exit bh201 init. \n",
					mmc_hostname(mmc_host), __func__);
				goto out;
			}
			set_gg_reg_cur_val(&vendor_host->ggc, (u8 *)gg_sw_def, sizeof(gg_sw_def));
			pr_debug("gg_sw_def[8] = 0x%08x, gg_sw_def[9]=0x%08x, gg_sw_def[15]=%08x\n",
					gg_sw_def[8], gg_sw_def[9], gg_sw_def[15]);
		} else {
			if (vendor_host->ggc.selx_tuning_done_flag)
				pr_info("%s: write previous inject results to bh201 for tuning done\n",
					mmc_hostname(mmc_host));
			if (vendor_host->ggc.tuning_cmd7_timeout_reinit_flg)
				pr_info("%s: write previous inject results to bh201 for cmd7 timeout flag is set\n",
					mmc_hostname(mmc_host));

			get_gg_reg_cur_val(&vendor_host->ggc, data, sizeof(gg_sw_def));

			pr_info("%s: dump config data before write to bh201\n", __func__);
			for (i = 0; i < 16; i++) {
				memcpy(&reg, data+i*sizeof(u32), sizeof(u32));
				pr_info("ggc_reg32[%03d]=0x%08x\n", i, reg);
			}
			if (bht_update_cfg(mmc_host, card, (u32 *)data, sizeof(data)))
				goto out;

		}

		bht_mmc_sd_get_csd(mmc_host, card);
		gg_select_card_spec(host);
		bht_mmc_app_send_scr(card);
		bht_mmc_decode_scr(card);
		bht_mmc_read_switch(card);
		bht_mmc_app_set_bus_width(card, MMC_BUS_WIDTH_1);
		bht_mmc_app_set_clr_card_detect(card);
		driver_send_command(host);
	}	return;
out:
	vendor_host->ggc.bh201_used = 0;
	vendor_host->ggc.card = NULL;
}

static ssize_t bh201_drive_strength_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct sdhci_msm_host *msm_host = get_vendor_host(mmc_priv(mmc));

	return sysfs_emit(buf, "0x%08x\n", msm_host->ggc.bh201_drive_strength);
}

static ssize_t bh201_drive_strength_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mmc_host *mmc = container_of(dev, struct mmc_host, class_dev);
	struct sdhci_msm_host *msm_host = get_vendor_host(mmc_priv(mmc));
	u32 value;
	int ret;

	ret = kstrtou32(buf, 0, &value);
	if (ret)
		return ret;
	msm_host->ggc.bh201_drive_strength = value;
	pr_info("%s: bh201_drive_strength updated to 0x%08x\n",
		mmc_hostname(mmc), value);
	return count;
}
static DEVICE_ATTR_RW(bh201_drive_strength);

static struct attribute *sdhci_bh201_attrs[] = {
	&dev_attr_bh201_drive_strength.attr,
	NULL,
};
static const struct attribute_group sdhci_bh201_attr_group = {
	.attrs = sdhci_bh201_attrs,
};

static int sdhci_bh201_create_sysfs(struct mmc_host *mmc)
{
	return sysfs_create_group(&mmc->class_dev.kobj, &sdhci_bh201_attr_group);
}

static void sdhci_bh201_remove_sysfs(struct mmc_host *mmc)
{
	sysfs_remove_group(&mmc->class_dev.kobj, &sdhci_bh201_attr_group);
}

static struct platform_driver sdhci_msm_driver = {
	.probe = sdhci_msm_probe,
	.remove_new = sdhci_msm_remove,
	.driver = {
		   .name = "sdhci_msm",
		   .probe_type = PROBE_PREFER_ASYNCHRONOUS,
		   .of_match_table = sdhci_msm_dt_match,
		   .pm = &sdhci_msm_pm_ops,
	},
};

module_platform_driver(sdhci_msm_driver);

MODULE_DESCRIPTION("Qualcomm Secure Digital Host Controller Interface driver");
MODULE_LICENSE("GPL v2");
