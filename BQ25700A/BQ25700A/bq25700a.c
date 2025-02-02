/* Copyright 2018 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * TI BQ25700A battery charger driver.
 */

#include "battery.h"
#include "battery_smart.h"
#include "BQ25700A.h"
#include "charge_ramp.h"
#include "charge_state_v2.h"
#include "charger.h"
#include "common.h"
#include "console.h"
#include "hooks.h"
#include "i2c.h"
#include "task.h"
#include "system.h"
#include "timer.h"
#include "util.h"

#ifndef CONFIG_CHARGER_NARROW_VDC
#error "BQ25700A is a NVDC charger, please enable CONFIG_CHARGER_NARROW_VDC."
#endif

/*
 * Delay required from taking the BQ25700A out of low power mode and having the
 * correct value in register 0x3E for VSYS_MIN voltage. The length of the delay
 * was determined by experiment. Less than 12 msec was not enough of delay, so
 * the value here is set to 20 msec to have plenty of margin.
 */
#define BQ25700A_VDDA_STARTUP_DELAY_MSEC 20

/* Sense resistor configurations and macros */
#define DEFAULT_SENSE_RESISTOR 10

#ifdef CONFIG_CHARGER_SENSE_RESISTOR_AC_BQ25700A
	#undef CONFIG_CHARGER_SENSE_RESISTOR_AC
	#define CONFIG_CHARGER_SENSE_RESISTOR_AC \
		CONFIG_CHARGER_SENSE_RESISTOR_AC_BQ25700A
#endif


#define INPUT_RESISTOR_RATIO \
	((CONFIG_CHARGER_SENSE_RESISTOR_AC) / DEFAULT_SENSE_RESISTOR)
#define REG_TO_INPUT_CURRENT(REG) ((REG + 1) * 50 / INPUT_RESISTOR_RATIO)
#define INPUT_CURRENT_TO_REG(CUR) (((CUR) * INPUT_RESISTOR_RATIO / 50) - 1)

#define CHARGING_RESISTOR_RATIO \
	((CONFIG_CHARGER_SENSE_RESISTOR) / DEFAULT_SENSE_RESISTOR)
#define REG_TO_CHARGING_CURRENT(REG) ((REG) / CHARGING_RESISTOR_RATIO)
#define CHARGING_CURRENT_TO_REG(CUR) ((CUR) * CHARGING_RESISTOR_RATIO)

/* Console output macros */
#define CPRINTF(format, args...) cprintf(CC_CHARGER, format, ## args)

#ifdef CONFIG_CHARGER_BQ25700A_IDCHG_LIMIT_MA
/*
 * If this config option is defined, then the BQ25700A needs to remain in
 * performance mode when the AP is in S0. Performance mode is active whenever AC
 * power is connected or when the EN_LWPWR bit in ChargeOption0 is clear.
 */
static uint32_t BQ25700A_perf_mode_req;
static struct mutex BQ25700A_perf_mode_mutex;
#endif

/* Charger parameters */
static const struct charger_info BQ25700A_charger_info = {
	.name         = "BQ25700A",
	.voltage_max  = 19200,
	.voltage_min  = 1024,
	.voltage_step = 8,
	.current_max  = 8128 / CHARGING_RESISTOR_RATIO,
	.current_min  = 64 / CHARGING_RESISTOR_RATIO,
	.current_step = 64 / CHARGING_RESISTOR_RATIO,
	.input_current_max  = 6400 / INPUT_RESISTOR_RATIO,
	.input_current_min  = 50 / INPUT_RESISTOR_RATIO,
	.input_current_step = 50 / INPUT_RESISTOR_RATIO,
};

static enum ec_error_list BQ25700A_get_option(int chgnum, int *option);
static enum ec_error_list BQ25700A_set_option(int chgnum, int option);

static inline enum ec_error_list raw_read16(int chgnum, int offset, int *value)
{
	return i2c_read16(chg_chips[chgnum].i2c_port,
			  chg_chips[chgnum].i2c_addr_flags,
			  offset, value);
}

static inline enum ec_error_list raw_write16(int chgnum, int offset, int value)
{
	return i2c_write16(chg_chips[chgnum].i2c_port,
			   chg_chips[chgnum].i2c_addr_flags,
			   offset, value);
}

#if defined(CONFIG_CHARGE_RAMP_HW) || \
	defined(CONFIG_USB_PD_VBUS_MEASURE_CHARGER)
static int BQ25700A_get_low_power_mode(int chgnum, int *mode)
{
	int rv;
	int reg;

	rv = raw_read16(chgnum, BQ25700A_REG_CHARGE_OPTION_0, &reg);
	if (rv)
		return rv;

	*mode = !!(reg & BQ25700A_CHARGE_OPTION_0_LOW_POWER_MODE);

	return EC_SUCCESS;
}

static int BQ25700A_set_low_power_mode(int chgnum, int enable)
{
	int rv;
	int reg;

	rv = raw_read16(chgnum, BQ25700A_REG_CHARGE_OPTION_0, &reg);
	if (rv)
		return rv;

#ifdef CONFIG_CHARGER_BQ25700A_IDCHG_LIMIT_MA
	mutex_lock(&BQ25700A_perf_mode_mutex);
	/*
	 * Performance mode means not in low power mode. The bit that controls
	 * this is EN_LWPWR in ChargeOption0. The 'enable' param in this
	 * function is refeerring to low power mode, so enabling low power mode
	 * means disabling performance mode and vice versa.
	 */
	if (enable)
		BQ25700A_perf_mode_req &= ~(1 << task_get_current());
	else
		BQ25700A_perf_mode_req |= (1 << task_get_current());
	enable = !BQ25700A_perf_mode_req;
#endif

	if (enable)
		reg |= BQ25700A_CHARGE_OPTION_0_LOW_POWER_MODE;
	else
		reg &= ~BQ25700A_CHARGE_OPTION_0_LOW_POWER_MODE;

	rv = raw_write16(chgnum, BQ25700A_REG_CHARGE_OPTION_0, reg);
#ifdef CONFIG_CHARGER_BQ25700A_IDCHG_LIMIT_MA
	mutex_unlock(&BQ25700A_perf_mode_mutex);
#endif
	if (rv)
		return rv;

	return EC_SUCCESS;
}

static int BQ25700A_adc_start(int chgnum, int adc_en_mask)
{
	int reg;
	int mode;
	int tries_left = 8;

	/* Save current mode to restore same state after ADC read */
	if (BQ25700A_get_low_power_mode(chgnum, &mode))
		return EC_ERROR_UNKNOWN;

	/* Exit low power mode so ADC conversion takes typical time */
	if (BQ25700A_set_low_power_mode(chgnum, 0))
		return EC_ERROR_UNKNOWN;

	/*
	 * Turn on the ADC for one reading. Note that adc_en_mask
	 * maps to bit[7:0] in ADCOption register.
	 */
	reg = (adc_en_mask & BQ25700A_ADC_OPTION_EN_ADC_ALL) |
	      BQ25700A_ADC_OPTION_ADC_START;
	if (raw_write16(chgnum, BQ25700A_REG_ADC_OPTION, reg))
		return EC_ERROR_UNKNOWN;

	/*
	 * Wait until the ADC operation completes. The spec says typical
	 * conversion time is 10 msec. If low power mode isn't exited first,
	 * then the conversion time jumps to ~60 msec.
	 */
	do {
		msleep(2);
		raw_read16(chgnum, BQ25700A_REG_ADC_OPTION, &reg);
	} while (--tries_left && (reg & BQ25700A_ADC_OPTION_ADC_START));

	/* ADC reading attempt complete, go back to low power mode */
	if (BQ25700A_set_low_power_mode(chgnum, mode))
		return EC_ERROR_UNKNOWN;

	/* Could not complete read */
	if (reg & BQ25700A_ADC_OPTION_ADC_START)
		return EC_ERROR_TIMEOUT;

	return EC_SUCCESS;
}
#endif

static void BQ25700A_init(int chgnum)
{
	int reg;
	int vsys;
	int rv;

	/*
	 * Reset registers to their default settings. There is no reset pin for
	 * this chip so without a full power cycle, some registers may not be at
	 * their default values. Note, need to save the POR value of
	 * MIN_SYSTEM_VOLTAGE register prior to setting the reset so that the
	 * correct value is preserved. In order to have the correct value read,
	 * the BQ25700A must not be in low power mode, otherwise the VDDA rail
	 * may not be powered if AC is not connected. Note, this reset is only
	 * required when running out of RO and not following sysjump to RW.
	 */
	if (!system_is_in_rw()) {
		rv = BQ25700A_set_low_power_mode(chgnum, 0);
		/* Allow enough time for VDDA to be powered */
		msleep(BQ25700A_VDDA_STARTUP_DELAY_MSEC);
		rv |= raw_read16(chgnum, BQ25700A_REG_MIN_SYSTEM_VOLTAGE, &vsys);
		rv |= raw_read16(chgnum, BQ25700A_REG_CHARGE_OPTION_3, &reg);
		if (!rv) {
			reg |= BQ25700A_CHARGE_OPTION_3_RESET_REG;
			/* Set all registers to default values */
			raw_write16(chgnum, BQ25700A_REG_CHARGE_OPTION_3, reg);
			/* Restore VSYS_MIN voltage to POR reset value */
			raw_write16(chgnum, BQ25700A_REG_MIN_SYSTEM_VOLTAGE,
				    vsys);
		}
		/* Reenable low power mode */
		BQ25700A_set_low_power_mode(chgnum, 1);
	}

	if (!raw_read16(chgnum, BQ25700A_REG_PROCHOT_OPTION_1, &reg)) {
		/* Disable VDPM prochot profile at initialization */
		reg &= ~BQ25700A_PROCHOT_PROFILE_VDPM;
		/*
		 * Enable PROCHOT to be asserted with VSYS min detection. Note
		 * that when no battery is present, then VSYS will be set to the
		 * value in register 0x3E (MinSysVoltage) which means that when
		 * no battery is present prochot will continuosly be asserted.
		 */
		reg |= BQ25700A_PROCHOT_PROFILE_VSYS;
#ifdef CONFIG_CHARGER_BQ25700A_IDCHG_LIMIT_MA
		/*
		 * Set the IDCHG limit who's value is defined in the config
		 * option in mA. Also, enable IDCHG trigger for prochot.
		 */
		reg &= ~BQ25700A_PROCHOT_IDCHG_VTH_MASK;
		/*
		 * IDCHG limit is in 512 mA steps. Note there is a 128 mA offset
		 * so the actual IDCHG limit will be the value stored in bits
		 * 15:10 + 128 mA.
		 */
		reg |= ((CONFIG_CHARGER_BQ25700A_IDCHG_LIMIT_MA << 1) &
			BQ25700A_PROCHOT_IDCHG_VTH_MASK);
		reg |= BQ25700A_PROCHOT_PROFILE_IDCHG;
#endif
		raw_write16(chgnum, BQ25700A_REG_PROCHOT_OPTION_1, reg);
	}

	/* Reduce ILIM from default of 150% to 105% */
	if (!raw_read16(chgnum, BQ25700A_REG_PROCHOT_OPTION_0, &reg)) {
		reg &= ~BQ25700A_PROCHOT0_ILIM_VTH_MASK;
		raw_write16(chgnum, BQ25700A_REG_PROCHOT_OPTION_0, reg);
	}

	/*
	 * Reduce peak power mode overload and relax cycle time from default 20
	 * msec to the minimum of 5 msec.
	 */
	if (!raw_read16(chgnum, BQ25700A_REG_CHARGE_OPTION_2, &reg)) {
		reg &= ~BQ25700A_CHARGE_OPTION_2_TMAX_MASK;
		raw_write16(chgnum, BQ25700A_REG_CHARGE_OPTION_2, reg);
	}
}

/* Charger interfaces */
static const struct charger_info *BQ25700A_get_info(int chgnum)
{
	return &BQ25700A_charger_info;
}

static enum ec_error_list BQ25700A_post_init(int chgnum)
{
	/*
	 * Note: BQ25700A power on reset state is:
	 *	watch dog timer     = 175 sec
	 *	input current limit = ~1/2 maximum setting
	 *	charging voltage    = 0 mV
	 *	charging current    = 0 mA
	 *	discharge on AC     = disabled
	 */

	return EC_SUCCESS;
}

static enum ec_error_list BQ25700A_get_status(int chgnum, int *status)
{
	int rv;
	int option;

	rv = BQ25700A_get_option(chgnum, &option);
	if (rv)
		return rv;

	/* Default status */
	*status = CHARGER_LEVEL_2;

	if (option & BQ25700A_CHARGE_OPTION_0_CHRG_INHIBIT)
		*status |= CHARGER_CHARGE_INHIBITED;

	return EC_SUCCESS;
}

static enum ec_error_list BQ25700A_set_mode(int chgnum, int mode)
{
	int rv;
	int option;

	rv = BQ25700A_get_option(chgnum, &option);
	if (rv)
		return rv;

	if (mode & CHARGER_CHARGE_INHIBITED)
		option |= BQ25700A_CHARGE_OPTION_0_CHRG_INHIBIT;
	else
		option &= ~BQ25700A_CHARGE_OPTION_0_CHRG_INHIBIT;

	return BQ25700A_set_option(chgnum, option);
}

static enum ec_error_list BQ25700A_enable_otg_power(int chgnum, int enabled)
{
	/* This is controlled with the EN_OTG pin. Support not added yet. */
	return EC_ERROR_UNIMPLEMENTED;
}

static enum ec_error_list BQ25700A_set_otg_current_voltage(int chgum,
							  int output_current,
							  int output_voltage)
{
	/* Add when needed. */
	return EC_ERROR_UNIMPLEMENTED;
}

static enum ec_error_list BQ25700A_get_current(int chgnum, int *current)
{
	int rv, reg;

	rv = raw_read16(chgnum, BQ25700A_REG_CHARGE_CURRENT, &reg);
	if (!rv)
		*current = REG_TO_CHARGING_CURRENT(reg);

	return rv;
}

static enum ec_error_list BQ25700A_set_current(int chgnum, int current)
{
	return raw_write16(chgnum, BQ25700A_REG_CHARGE_CURRENT,
		CHARGING_CURRENT_TO_REG(current));
}

/* Get/set charge voltage limit in mV */
static enum ec_error_list BQ25700A_get_voltage(int chgnum, int *voltage)
{
	return raw_read16(chgnum, BQ25700A_REG_MAX_CHARGE_VOLTAGE, voltage);
}

static enum ec_error_list BQ25700A_set_voltage(int chgnum, int voltage)
{
	return raw_write16(chgnum, BQ25700A_REG_MAX_CHARGE_VOLTAGE, voltage);
}

/* Discharge battery when on AC power. */
static enum ec_error_list BQ25700A_discharge_on_ac(int chgnum, int enable)
{
	int rv, option;

	rv = BQ25700A_get_option(chgnum, &option);
	if (rv)
		return rv;

	if (enable)
		option |= BQ25700A_CHARGE_OPTION_0_EN_LEARN;
	else
		option &= ~BQ25700A_CHARGE_OPTION_0_EN_LEARN;

	return BQ25700A_set_option(chgnum, option);
}

static enum ec_error_list BQ25700A_set_input_current_limit(int chgnum,
							  int input_current)
{
	int num_steps = INPUT_CURRENT_TO_REG(input_current);

	return raw_write16(chgnum, BQ25700A_REG_IIN_HOST, num_steps <<
			  BQ25700A_CHARGE_IIN_BIT_0FFSET);
}

static enum ec_error_list BQ25700A_get_input_current_limit(int chgnum,
							  int *input_current)
{
	int rv, reg;

	/*
	 * IIN_DPM register reflects the actual input current limit programmed
	 * in the register, either from host or from ICO. After ICO, the
	 * current limit used by DPM regulation may differ from the IIN_HOST
	 * register settings.
	 */
	rv = raw_read16(chgnum, BQ25700A_REG_IIN_DPM, &reg);
	if (!rv)
		*input_current =
			REG_TO_INPUT_CURRENT((reg >>
					      BQ25700A_CHARGE_IIN_BIT_0FFSET));

	return rv;
}

static enum ec_error_list BQ25700A_manufacturer_id(int chgnum, int *id)
{
	return raw_read16(chgnum, BQ25700A_REG_MANUFACTURER_ID, id);
}

static enum ec_error_list BQ25700A_device_id(int chgnum, int *id)
{
	return raw_read16(chgnum, BQ25700A_REG_DEVICE_ADDRESS, id);
}

#ifdef CONFIG_USB_PD_VBUS_MEASURE_CHARGER
static enum ec_error_list BQ25700A_get_vbus_voltage(int chgnum, int port,
						   int *voltage)
{
	int reg, rv;

	rv = BQ25700A_adc_start(chgnum, BQ25700A_ADC_OPTION_EN_ADC_VBUS);
	if (rv)
		goto error;

	/* Read ADC value */
	rv = raw_read16(chgnum, BQ25700A_REG_ADC_VBUS_PSYS, &reg);
	if (rv)
		goto error;

	reg >>= BQ25700A_ADC_VBUS_STEP_BIT_OFFSET;
	/*
	 * LSB => 64mV.
	 * Return 0 when VBUS <= 3.2V as ADC can't measure it.
	 */
	*voltage = reg ?
	       (reg * BQ25700A_ADC_VBUS_STEP_MV + BQ25700A_ADC_VBUS_BASE_MV) : 0;

error:
	if (rv)
		CPRINTF("Could not read VBUS ADC! Error: %d\n", rv);
	return rv;
}
#endif

static enum ec_error_list BQ25700A_get_option(int chgnum, int *option)
{
	/* There are 4 option registers, but we only need the first for now. */
	return raw_read16(chgnum, BQ25700A_REG_CHARGE_OPTION_0, option);
}

static enum ec_error_list BQ25700A_set_option(int chgnum, int option)
{
	/* There are 4 option registers, but we only need the first for now. */
	return raw_write16(chgnum, BQ25700A_REG_CHARGE_OPTION_0, option);
}

#ifdef CONFIG_CHARGE_RAMP_HW

static void BQ25700A_chg_ramp_handle(void)
{
	int ramp_curr;
	int chgnum = 0;

	if (IS_ENABLED(CONFIG_OCPC))
		chgnum = charge_get_active_chg_chip();

	/*
	 * Once the charge ramp is stable write back the stable ramp
	 * current to the host input current limit register
	 */
	ramp_curr = chg_ramp_get_current_limit();
	if (chg_ramp_is_stable()) {
		if (ramp_curr &&
		    !charger_set_input_current_limit(chgnum, ramp_curr))
			CPRINTF("BQ25700A: stable ramp current=%d\n", ramp_curr);
	} else {
		CPRINTF("BQ25700A: ICO stall, ramp current=%d\n", ramp_curr);
	}
	/*
	 * Disable ICO mode. When ICO mode is active the input current limit is
	 * given by the value in register IIN_DPM (0x22)
	 */
	charger_set_hw_ramp(0);
}
DECLARE_DEFERRED(BQ25700A_chg_ramp_handle);

static enum ec_error_list BQ25700A_set_hw_ramp(int chgnum, int enable)
{
	int option3_reg, option2_reg, rv;

	rv = raw_read16(chgnum, BQ25700A_REG_CHARGE_OPTION_3, &option3_reg);
	if (rv)
		return rv;
	rv = raw_read16(chgnum, BQ25700A_REG_CHARGE_OPTION_2, &option2_reg);
	if (rv)
		return rv;

	if (enable) {
		/*
		 * ICO mode can only be used when a battery is present. If there
		 * is no battery, or if the battery has not recovered yet from
		 * cutoff, then enabling ICO mode will lead to VSYS
		 * dropping out.
		 */
		if (!battery_is_present() || (battery_get_disconnect_state() !=
					      BATTERY_NOT_DISCONNECTED)) {
			CPRINTF("BQ25700A: no battery, skip ICO enable\n");
			return EC_ERROR_UNKNOWN;
		}

		/* Set InputVoltage register to BC1.2 minimum ramp voltage */
		rv = raw_write16(chgnum, BQ25700A_REG_INPUT_VOLTAGE,
			BQ25700A_BC12_MIN_VOLTAGE_MV);
		if (rv)
			return rv;

		/*  Enable ICO algorithm */
		option3_reg |= BQ25700A_CHARGE_OPTION_3_EN_ICO_MODE;

		/* 0b: Input current limit is set by BQ25700A_REG_IIN_HOST */
		option2_reg &= ~BQ25700A_CHARGE_OPTION_2_EN_EXTILIM;

		/* Charge ramp may take up to 2s to settle down */
		hook_call_deferred(&BQ25700A_chg_ramp_handle_data, (4 * SECOND));
	} else {
		/*  Disable ICO algorithm */
		option3_reg &= ~BQ25700A_CHARGE_OPTION_3_EN_ICO_MODE;

		/*
		 * 1b: Input current limit is set by the lower value of
		 * ILIM_HIZ pin and BQ25700A_REG_IIN_HOST
		 */
		option2_reg |= BQ25700A_CHARGE_OPTION_2_EN_EXTILIM;
	}

	rv = raw_write16(chgnum, BQ25700A_REG_CHARGE_OPTION_2, option2_reg);
	if (rv)
		return rv;
	return raw_write16(chgnum, BQ25700A_REG_CHARGE_OPTION_3, option3_reg);
}

static int BQ25700A_ramp_is_stable(int chgnum)
{
	int reg;

	if (raw_read16(chgnum, BQ25700A_REG_CHARGER_STATUS, &reg))
		return 0;

	return reg & BQ25700A_CHARGE_STATUS_ICO_DONE;
}

static int BQ25700A_ramp_get_current_limit(int chgnum)
{
	int reg, rv;

	rv = raw_read16(chgnum, BQ25700A_REG_IIN_DPM, &reg);
	if (rv) {
		CPRINTF("Could not read iin_dpm current limit! Error: %d\n",
			rv);
		return 0;
	}

	return ((reg >> BQ25700A_IIN_DPM_BIT_SHIFT) * BQ25700A_IIN_DPM_STEP_MA +
		BQ25700A_IIN_DPM_STEP_MA);
}
#endif /* CONFIG_CHARGE_RAMP_HW */

#ifdef CONFIG_CHARGER_BQ25700A_IDCHG_LIMIT_MA
/* Called on AP S5 -> S3  and S3/S0iX -> S0 transition */
static void BQ25700A_chipset_startup(void)
{
	BQ25700A_set_low_power_mode(CHARGER_SOLO, 0);
}
DECLARE_HOOK(HOOK_CHIPSET_STARTUP, BQ25700A_chipset_startup, HOOK_PRIO_DEFAULT);
DECLARE_HOOK(HOOK_CHIPSET_RESUME, BQ25700A_chipset_startup, HOOK_PRIO_DEFAULT);


/* Called on AP S0 -> S0iX/S3 or S3 -> S5 transition */
static void BQ25700A_chipset_suspend(void)
{
	BQ25700A_set_low_power_mode(CHARGER_SOLO, 1);
}
DECLARE_HOOK(HOOK_CHIPSET_SUSPEND, BQ25700A_chipset_suspend, HOOK_PRIO_DEFAULT);
DECLARE_HOOK(HOOK_CHIPSET_SHUTDOWN, BQ25700A_chipset_suspend, HOOK_PRIO_DEFAULT);
#endif

#ifdef CONFIG_CMD_CHARGER_DUMP
static int console_BQ25700A_dump_regs(int argc, char **argv)
{
	int i;
	int val;
	int chgnum = 0;
	char *e;

	/* Dump all readable registers on BQ25700A. */
	static const uint8_t regs[] = {
		BQ25700A_REG_CHARGE_OPTION_0,
		BQ25700A_REG_CHARGE_CURRENT,
		BQ25700A_REG_MAX_CHARGE_VOLTAGE,
		BQ25700A_REG_CHARGE_OPTION_1,
		BQ25700A_REG_CHARGE_OPTION_2,
		BQ25700A_REG_CHARGE_OPTION_3,
		BQ25700A_REG_PROCHOT_OPTION_0,
		BQ25700A_REG_PROCHOT_OPTION_1,
		BQ25700A_REG_ADC_OPTION,
		BQ25700A_REG_CHARGER_STATUS,
		BQ25700A_REG_PROCHOT_STATUS,
		BQ25700A_REG_IIN_DPM,
		BQ25700A_REG_ADC_VBUS_PSYS,
		BQ25700A_REG_ADC_IBAT,
		BQ25700A_REG_ADC_CMPIN_IIN,
		BQ25700A_REG_ADC_VSYS_VBAT,
		BQ25700A_REG_PROCHOT_OPTION_1,
		BQ25700A_REG_OTG_VOLTAGE,
		BQ25700A_REG_OTG_CURRENT,
		BQ25700A_REG_INPUT_VOLTAGE,
		BQ25700A_REG_MIN_SYSTEM_VOLTAGE,
		BQ25700A_REG_IIN_HOST,
		BQ25700A_REG_MANUFACTURER_ID,
		BQ25700A_REG_DEVICE_ADDRESS,
	};
	if (argc >= 2) {
		chgnum = strtoi(argv[1], &e, 10);
		if (*e)
			return EC_ERROR_PARAM1;
	}

	for (i = 0; i < ARRAY_SIZE(regs); ++i) {
		if (raw_read16(chgnum, regs[i], &val))
			continue;
		ccprintf("BQ25700A REG 0x%02x:  0x%04x\n", regs[i], val);
	}


	return 0;
}
DECLARE_CONSOLE_COMMAND(charger_dump, console_BQ25700A_dump_regs,
			"charger_dump <chgnum>",
			"Dump all charger registers");

#endif /* CONFIG_CMD_CHARGER_DUMP */

const struct charger_drv BQ25700A_drv = {
	.init = &BQ25700A_init,
	.post_init = &BQ25700A_post_init,
	.get_info = &BQ25700A_get_info,
	.get_status = &BQ25700A_get_status,
	.set_mode = &BQ25700A_set_mode,
	.enable_otg_power = &BQ25700A_enable_otg_power,
	.set_otg_current_voltage = &BQ25700A_set_otg_current_voltage,
	.get_current = &BQ25700A_get_current,
	.set_current = &BQ25700A_set_current,
	.get_voltage = &BQ25700A_get_voltage,
	.set_voltage = &BQ25700A_set_voltage,
	.discharge_on_ac = &BQ25700A_discharge_on_ac,
	.get_vbus_voltage = &BQ25700A_get_vbus_voltage,
	.set_input_current_limit = &BQ25700A_set_input_current_limit,
	.get_input_current_limit = &BQ25700A_get_input_current_limit,
	.manufacturer_id = &BQ25700A_manufacturer_id,
	.device_id = &BQ25700A_device_id,
	.get_option = &BQ25700A_get_option,
	.set_option = &BQ25700A_set_option,
	.set_hw_ramp = &BQ25700A_set_hw_ramp,
	.ramp_is_stable = &BQ25700A_ramp_is_stable,
	.ramp_get_current_limit = &BQ25700A_ramp_get_current_limit,
};
