/**************************************************************************
*   Copyright (C) 2012 by Andreas Fritiofson                              *
*   andreas.fritiofson@gmail.com                                          *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
***************************************************************************/

/**
 * @file
 * JTAG adapters based on the FT2232 full and high speed USB parts are
 * popular low cost JTAG debug solutions.  Many FT2232 based JTAG adapters
 * are discrete, but development boards may integrate them as alternatives
 * to more capable (and expensive) third party JTAG pods.
 *
 * JTAG uses only one of the two communications channels ("MPSSE engines")
 * on these devices.  Adapters based on FT4232 parts have four ports/channels
 * (A/B/C/D), instead of just two (A/B).
 *
 * Especially on development boards integrating one of these chips (as
 * opposed to discrete pods/dongles), the additional channels can be used
 * for a variety of purposes, but OpenOCD only uses one channel at a time.
 *
 *  - As a USB-to-serial adapter for the target's console UART ...
 *    which may be able to support ROM boot loaders that load initial
 *    firmware images to flash (or SRAM).
 *
 *  - On systems which support ARM's SWD in addition to JTAG, or instead
 *    of it, that second port can be used for reading SWV/SWO trace data.
 *
 *  - Additional JTAG links, e.g. to a CPLD or * FPGA.
 *
 * FT2232 based JTAG adapters are "dumb" not "smart", because most JTAG
 * request/response interactions involve round trips over the USB link.
 * A "smart" JTAG adapter has intelligence close to the scan chain, so it
 * can for example poll quickly for a status change (usually taking on the
 * order of microseconds not milliseconds) before beginning a queued
 * transaction which require the previous one to have completed.
 *
 * There are dozens of adapters of this type, differing in details which
 * this driver needs to understand.  Those "layout" details are required
 * as part of FT2232 driver configuration.
 *
 * This code uses information contained in the MPSSE specification which was
 * found here:
 * http://www.ftdichip.com/Documents/AppNotes/AN2232C-01_MPSSE_Cmnd.pdf
 * Hereafter this is called the "MPSSE Spec".
 *
 * The datasheet for the ftdichip.com's FT2232D part is here:
 * http://www.ftdichip.com/Documents/DataSheets/DS_FT2232D.pdf
 *
 * Also note the issue with code 0x4b (clock data to TMS) noted in
 * http://developer.intra2net.com/mailarchive/html/libftdi/2009/msg00292.html
 * which can affect longer JTAG state paths.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <jtag/interface.h>
#include <jtag/swd.h>
#include <transport/transport.h>
#include <helper/time_support.h>

#if IS_CYGWIN == 1
#include <windows.h>
#endif

#include <assert.h>

/* FTDI access library includes */
#include "mpsse.h"

#define JTAG_MODE (LSB_FIRST | POS_EDGE_IN | NEG_EDGE_OUT)
#define JTAG_MODE_ALT (LSB_FIRST | NEG_EDGE_IN | NEG_EDGE_OUT)
#define SWD_MODE (LSB_FIRST | POS_EDGE_IN | NEG_EDGE_OUT)

/* The number of SWD transactions which must pass before exiting wait mode. */
#define FTDI_SWD_WAIT_COUNT 100

static char *ftdi_device_desc;
static char *ftdi_serial;
static char *ftdi_location;
static uint8_t ftdi_channel;
static uint8_t ftdi_jtag_mode = JTAG_MODE;

static bool swd_mode;

#define MAX_USB_IDS 8
/* vid = pid = 0 marks the end of the list */
static uint16_t ftdi_vid[MAX_USB_IDS + 1] = { 0 };
static uint16_t ftdi_pid[MAX_USB_IDS + 1] = { 0 };

static struct mpsse_ctx *mpsse_ctx;

struct signal {
	const char *name;
	uint16_t data_mask;
	uint16_t oe_mask;
	bool invert_data;
	bool invert_oe;
	struct signal *next;
};

static struct signal *signals;

/* Bit sizes for the SWD packets. */
#define SWD_CMD_SZ          8
#define SWD_TRN_W_TO_R_SZ   1
#define SWD_ACK_SZ          3
#define SWD_TRN_R_TO_W_SZ   1
#define SWD_DATA_SZ         32
#define SWD_PARITY_SZ       1

/* FIXME: Where to store per-instance data? We need an SWD context.
 *
 * The ack, data and parity could be kept in one array to save some space, but
 * this requires shifting to put the data in place and docode it.  This shifting
 * takes quite a bit of time, so instead it is all justified, to save the processing
 * time.
 */
static struct swd_cmd_queue_entry {
	uint8_t cmd;                    /* Holds the command. */
	uint8_t ack;                    /* Holds the 3 bit ack. */
	uint8_t data[sizeof(uint32_t)]; /* Holds the 32 bits of data. */
	uint8_t parity;                 /* Holds the 1 bit of data. */
	uint32_t *dst;                  /* Points to the location to place read data. */
} *swd_cmd_queue;
static size_t swd_cmd_queue_length;
static size_t swd_cmd_queue_alloced;
static int queued_retval;
static int freq;

/* If during SWD a part returns a WAIT response, the wait count is setup.  For
 * the next FTID_SWD_WAIT_COUNT transactions, wait is checked for on each
 * transaction.  If the count is not reset, then the code reverts back to queued
 * mode.  This is done since queued mode is faster and for some parts WAIT only
 * seems to happen during the init sequence.
 */
static unsigned int ftdi_swd_wait_remaining = 0;

static uint16_t output;
static uint16_t direction;

static uint16_t jtag_output_init;
static uint16_t jtag_direction_init;

static int ftdi_swd_switch_seq(enum swd_special_seq seq);
static void ftdi_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk);

static struct signal *find_signal_by_name(const char *name)
{
	for (struct signal *sig = signals; sig; sig = sig->next) {
		if (strcmp(name, sig->name) == 0)
			return sig;
	}
	return NULL;
}

static struct signal *create_signal(const char *name)
{
	struct signal **psig = &signals;
	while (*psig)
		psig = &(*psig)->next;

	*psig = calloc(1, sizeof(**psig));
	if (*psig == NULL)
		return NULL;

	(*psig)->name = strdup(name);
	if ((*psig)->name == NULL) {
		free(*psig);
		*psig = NULL;
	}
	return *psig;
}

static int ftdi_set_signal(const struct signal *s, char value)
{
	bool data;
	bool oe;

	if (s->data_mask == 0 && s->oe_mask == 0) {
		LOG_ERROR("interface doesn't provide signal '%s'", s->name);
		return ERROR_FAIL;
	}
	switch (value) {
	case '0':
		data = s->invert_data;
		oe = !s->invert_oe;
		break;
	case '1':
		if (s->data_mask == 0) {
			LOG_ERROR("interface can't drive '%s' high", s->name);
			return ERROR_FAIL;
		}
		data = !s->invert_data;
		oe = !s->invert_oe;
		break;
	case 'z':
	case 'Z':
		if (s->oe_mask == 0) {
			LOG_ERROR("interface can't tri-state '%s'", s->name);
			return ERROR_FAIL;
		}
		data = s->invert_data;
		oe = s->invert_oe;
		break;
	default:
		assert(0 && "invalid signal level specifier");
		return ERROR_FAIL;
	}

	uint16_t old_output = output;
	uint16_t old_direction = direction;

	output = data ? output | s->data_mask : output & ~s->data_mask;
	if (s->oe_mask == s->data_mask)
		direction = oe ? direction | s->oe_mask : direction & ~s->oe_mask;
	else
		output = oe ? output | s->oe_mask : output & ~s->oe_mask;

	if ((output & 0xff) != (old_output & 0xff) || (direction & 0xff) != (old_direction & 0xff))
		mpsse_set_data_bits_low_byte(mpsse_ctx, output & 0xff, direction & 0xff);
	if ((output >> 8 != old_output >> 8) || (direction >> 8 != old_direction >> 8))
		mpsse_set_data_bits_high_byte(mpsse_ctx, output >> 8, direction >> 8);

	return ERROR_OK;
}


/**
 * Function move_to_state
 * moves the TAP controller from the current state to a
 * \a goal_state through a path given by tap_get_tms_path().  State transition
 * logging is performed by delegation to clock_tms().
 *
 * @param goal_state is the destination state for the move.
 */
static void move_to_state(tap_state_t goal_state)
{
	tap_state_t start_state = tap_get_state();

	/*	goal_state is 1/2 of a tuple/pair of states which allow convenient
		lookup of the required TMS pattern to move to this state from the
		start state.
	*/

	/* do the 2 lookups */
	uint8_t tms_bits  = tap_get_tms_path(start_state, goal_state);
	int tms_count = tap_get_tms_path_len(start_state, goal_state);
	assert(tms_count <= 8);

	DEBUG_JTAG_IO("start=%s goal=%s", tap_state_name(start_state), tap_state_name(goal_state));

	/* Track state transitions step by step */
	for (int i = 0; i < tms_count; i++)
		tap_set_state(tap_state_transition(tap_get_state(), (tms_bits >> i) & 1));

	mpsse_clock_tms_cs_out(mpsse_ctx,
		&tms_bits,
		0,
		tms_count,
		false,
		ftdi_jtag_mode);
}

static int ftdi_speed(int speed)
{
	int retval;
	retval = mpsse_set_frequency(mpsse_ctx, speed);

	if (retval < 0) {
		LOG_ERROR("couldn't set FTDI TCK speed");
		return retval;
	}

	if (!swd_mode && speed >= 10000000 && ftdi_jtag_mode != JTAG_MODE_ALT)
		LOG_INFO("ftdi: if you experience problems at higher adapter clocks, try "
			 "the command \"ftdi_tdo_sample_edge falling\"");
	return ERROR_OK;
}

static int ftdi_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;
}

static int ftdi_khz(int khz, int *jtag_speed)
{
	if (khz == 0 && !mpsse_is_high_speed(mpsse_ctx)) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

static void ftdi_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %s is not a stable end state", tap_state_name(state));
		exit(-1);
	}
}

static void ftdi_execute_runtest(struct jtag_command *cmd)
{
	int i;
	uint8_t zero = 0;

	DEBUG_JTAG_IO("runtest %i cycles, end in %s",
		cmd->cmd.runtest->num_cycles,
		tap_state_name(cmd->cmd.runtest->end_state));

	if (tap_get_state() != TAP_IDLE)
		move_to_state(TAP_IDLE);

	/* TODO: Reuse ftdi_execute_stableclocks */
	i = cmd->cmd.runtest->num_cycles;
	while (i > 0) {
		/* there are no state transitions in this code, so omit state tracking */
		unsigned this_len = i > 7 ? 7 : i;
		mpsse_clock_tms_cs_out(mpsse_ctx, &zero, 0, this_len, false, ftdi_jtag_mode);
		i -= this_len;
	}

	ftdi_end_state(cmd->cmd.runtest->end_state);

	if (tap_get_state() != tap_get_end_state())
		move_to_state(tap_get_end_state());

	DEBUG_JTAG_IO("runtest: %i, end in %s",
		cmd->cmd.runtest->num_cycles,
		tap_state_name(tap_get_end_state()));
}

static void ftdi_execute_statemove(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("statemove end in %s",
		tap_state_name(cmd->cmd.statemove->end_state));

	ftdi_end_state(cmd->cmd.statemove->end_state);

	/* shortest-path move to desired end state */
	if (tap_get_state() != tap_get_end_state() || tap_get_end_state() == TAP_RESET)
		move_to_state(tap_get_end_state());
}

/**
 * Clock a bunch of TMS (or SWDIO) transitions, to change the JTAG
 * (or SWD) state machine. REVISIT: Not the best method, perhaps.
 */
static void ftdi_execute_tms(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("TMS: %d bits", cmd->cmd.tms->num_bits);

	/* TODO: Missing tap state tracking, also missing from ft2232.c! */
	mpsse_clock_tms_cs_out(mpsse_ctx,
		cmd->cmd.tms->bits,
		0,
		cmd->cmd.tms->num_bits,
		false,
		ftdi_jtag_mode);
}

static void ftdi_execute_pathmove(struct jtag_command *cmd)
{
	tap_state_t *path = cmd->cmd.pathmove->path;
	int num_states  = cmd->cmd.pathmove->num_states;

	DEBUG_JTAG_IO("pathmove: %i states, current: %s  end: %s", num_states,
		tap_state_name(tap_get_state()),
		tap_state_name(path[num_states-1]));

	int state_count = 0;
	unsigned bit_count = 0;
	uint8_t tms_byte = 0;

	DEBUG_JTAG_IO("-");

	/* this loop verifies that the path is legal and logs each state in the path */
	while (num_states--) {

		/* either TMS=0 or TMS=1 must work ... */
		if (tap_state_transition(tap_get_state(), false)
		    == path[state_count])
			buf_set_u32(&tms_byte, bit_count++, 1, 0x0);
		else if (tap_state_transition(tap_get_state(), true)
			 == path[state_count]) {
			buf_set_u32(&tms_byte, bit_count++, 1, 0x1);

			/* ... or else the caller goofed BADLY */
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid "
				"TAP state transition",
				tap_state_name(tap_get_state()),
				tap_state_name(path[state_count]));
			exit(-1);
		}

		tap_set_state(path[state_count]);
		state_count++;

		if (bit_count == 7 || num_states == 0) {
			mpsse_clock_tms_cs_out(mpsse_ctx,
					&tms_byte,
					0,
					bit_count,
					false,
					ftdi_jtag_mode);
			bit_count = 0;
		}
	}
	tap_set_end_state(tap_get_state());
}

static void ftdi_execute_scan(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("%s type:%d", cmd->cmd.scan->ir_scan ? "IRSCAN" : "DRSCAN",
		jtag_scan_type(cmd->cmd.scan));

	/* Make sure there are no trailing fields with num_bits == 0, or the logic below will fail. */
	while (cmd->cmd.scan->num_fields > 0
			&& cmd->cmd.scan->fields[cmd->cmd.scan->num_fields - 1].num_bits == 0) {
		cmd->cmd.scan->num_fields--;
		LOG_DEBUG("discarding trailing empty field");
	}

	if (cmd->cmd.scan->num_fields == 0) {
		LOG_DEBUG("empty scan, doing nothing");
		return;
	}

	if (cmd->cmd.scan->ir_scan) {
		if (tap_get_state() != TAP_IRSHIFT)
			move_to_state(TAP_IRSHIFT);
	} else {
		if (tap_get_state() != TAP_DRSHIFT)
			move_to_state(TAP_DRSHIFT);
	}

	ftdi_end_state(cmd->cmd.scan->end_state);

	struct scan_field *field = cmd->cmd.scan->fields;
	unsigned scan_size = 0;

	for (int i = 0; i < cmd->cmd.scan->num_fields; i++, field++) {
		scan_size += field->num_bits;
		DEBUG_JTAG_IO("%s%s field %d/%d %d bits",
			field->in_value ? "in" : "",
			field->out_value ? "out" : "",
			i,
			cmd->cmd.scan->num_fields,
			field->num_bits);

		if (i == cmd->cmd.scan->num_fields - 1 && tap_get_state() != tap_get_end_state()) {
			/* Last field, and we're leaving IRSHIFT/DRSHIFT. Clock last bit during tap
			 * movement. This last field can't have length zero, it was checked above. */
			mpsse_clock_data(mpsse_ctx,
				field->out_value,
				0,
				field->in_value,
				0,
				field->num_bits - 1,
				ftdi_jtag_mode);
			uint8_t last_bit = 0;
			if (field->out_value)
				bit_copy(&last_bit, 0, field->out_value, field->num_bits - 1, 1);
			uint8_t tms_bits = 0x01;
			mpsse_clock_tms_cs(mpsse_ctx,
					&tms_bits,
					0,
					field->in_value,
					field->num_bits - 1,
					1,
					last_bit,
					ftdi_jtag_mode);
			tap_set_state(tap_state_transition(tap_get_state(), 1));
			mpsse_clock_tms_cs_out(mpsse_ctx,
					&tms_bits,
					1,
					1,
					last_bit,
					ftdi_jtag_mode);
			tap_set_state(tap_state_transition(tap_get_state(), 0));
		} else
			mpsse_clock_data(mpsse_ctx,
				field->out_value,
				0,
				field->in_value,
				0,
				field->num_bits,
				ftdi_jtag_mode);
	}

	if (tap_get_state() != tap_get_end_state())
		move_to_state(tap_get_end_state());

	DEBUG_JTAG_IO("%s scan, %i bits, end in %s",
		(cmd->cmd.scan->ir_scan) ? "IR" : "DR", scan_size,
		tap_state_name(tap_get_end_state()));
}

static void ftdi_execute_reset(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("reset trst: %i srst %i",
		cmd->cmd.reset->trst, cmd->cmd.reset->srst);

	if (cmd->cmd.reset->trst == 1
	    || (cmd->cmd.reset->srst
		&& (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
		tap_set_state(TAP_RESET);

	struct signal *trst = find_signal_by_name("nTRST");
	if (cmd->cmd.reset->trst == 1) {
		if (trst)
			ftdi_set_signal(trst, '0');
		else
			LOG_ERROR("Can't assert TRST: nTRST signal is not defined");
	} else if (trst && jtag_get_reset_config() & RESET_HAS_TRST &&
			cmd->cmd.reset->trst == 0) {
		if (jtag_get_reset_config() & RESET_TRST_OPEN_DRAIN)
			ftdi_set_signal(trst, 'z');
		else
			ftdi_set_signal(trst, '1');
	}

	struct signal *srst = find_signal_by_name("nSRST");
	if (cmd->cmd.reset->srst == 1) {
		if (srst)
			ftdi_set_signal(srst, '0');
		else
			LOG_ERROR("Can't assert SRST: nSRST signal is not defined");
	} else if (srst && jtag_get_reset_config() & RESET_HAS_SRST &&
			cmd->cmd.reset->srst == 0) {
		if (jtag_get_reset_config() & RESET_SRST_PUSH_PULL)
			ftdi_set_signal(srst, '1');
		else
			ftdi_set_signal(srst, 'z');
	}

	DEBUG_JTAG_IO("trst: %i, srst: %i",
		cmd->cmd.reset->trst, cmd->cmd.reset->srst);
}

static void ftdi_execute_sleep(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("sleep %" PRIi32, cmd->cmd.sleep->us);

	mpsse_flush(mpsse_ctx);
	jtag_sleep(cmd->cmd.sleep->us);
	DEBUG_JTAG_IO("sleep %" PRIi32 " usec while in %s",
		cmd->cmd.sleep->us,
		tap_state_name(tap_get_state()));
}

static void ftdi_execute_stableclocks(struct jtag_command *cmd)
{
	/* this is only allowed while in a stable state.  A check for a stable
	 * state was done in jtag_add_clocks()
	 */
	int num_cycles = cmd->cmd.stableclocks->num_cycles;

	/* 7 bits of either ones or zeros. */
	uint8_t tms = tap_get_state() == TAP_RESET ? 0x7f : 0x00;

	/* TODO: Use mpsse_clock_data with in=out=0 for this, if TMS can be set to
	 * the correct level and remain there during the scan */
	while (num_cycles > 0) {
		/* there are no state transitions in this code, so omit state tracking */
		unsigned this_len = num_cycles > 7 ? 7 : num_cycles;
		mpsse_clock_tms_cs_out(mpsse_ctx, &tms, 0, this_len, false, ftdi_jtag_mode);
		num_cycles -= this_len;
	}

	DEBUG_JTAG_IO("clocks %i while in %s",
		cmd->cmd.stableclocks->num_cycles,
		tap_state_name(tap_get_state()));
}

static void ftdi_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
		case JTAG_RESET:
			ftdi_execute_reset(cmd);
			break;
		case JTAG_RUNTEST:
			ftdi_execute_runtest(cmd);
			break;
		case JTAG_TLR_RESET:
			ftdi_execute_statemove(cmd);
			break;
		case JTAG_PATHMOVE:
			ftdi_execute_pathmove(cmd);
			break;
		case JTAG_SCAN:
			ftdi_execute_scan(cmd);
			break;
		case JTAG_SLEEP:
			ftdi_execute_sleep(cmd);
			break;
		case JTAG_STABLECLOCKS:
			ftdi_execute_stableclocks(cmd);
			break;
		case JTAG_TMS:
			ftdi_execute_tms(cmd);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered: %d", cmd->type);
			break;
	}
}

static int ftdi_execute_queue(void)
{
	/* blink, if the current layout has that feature */
	struct signal *led = find_signal_by_name("LED");
	if (led)
		ftdi_set_signal(led, '1');

	for (struct jtag_command *cmd = jtag_command_queue; cmd; cmd = cmd->next) {
		/* fill the write buffer with the desired command */
		ftdi_execute_command(cmd);
	}

	if (led)
		ftdi_set_signal(led, '0');

	int retval = mpsse_flush(mpsse_ctx);
	if (retval != ERROR_OK)
		LOG_ERROR("error while flushing MPSSE queue: %d", retval);

	return retval;
}

static int ftdi_initialize(void)
{
	if (tap_get_tms_path_len(TAP_IRPAUSE, TAP_IRPAUSE) == 7)
		LOG_DEBUG("ftdi interface using 7 step jtag state transitions");
	else
		LOG_DEBUG("ftdi interface using shortest path jtag state transitions");

	for (int i = 0; ftdi_vid[i] || ftdi_pid[i]; i++) {
		mpsse_ctx = mpsse_open(&ftdi_vid[i], &ftdi_pid[i], ftdi_device_desc,
				ftdi_serial, ftdi_location, ftdi_channel);
		if (mpsse_ctx)
			break;
	}

	if (!mpsse_ctx)
		return ERROR_JTAG_INIT_FAILED;

	output = jtag_output_init;
	direction = jtag_direction_init;

	if (swd_mode) {
		struct signal *sig = find_signal_by_name("SWD_EN");
		if (!sig) {
			LOG_ERROR("SWD mode is active but SWD_EN signal is not defined");
			return ERROR_JTAG_INIT_FAILED;
		}
		/* A dummy SWD_EN would have zero mask */
		if (sig->data_mask)
			ftdi_set_signal(sig, '1');
	}

	mpsse_set_data_bits_low_byte(mpsse_ctx, output & 0xff, direction & 0xff);
	mpsse_set_data_bits_high_byte(mpsse_ctx, output >> 8, direction >> 8);

	mpsse_loopback_config(mpsse_ctx, false);

	freq = mpsse_set_frequency(mpsse_ctx, jtag_get_speed_khz() * 1000);

	return mpsse_flush(mpsse_ctx);
}

static int ftdi_quit(void)
{
	mpsse_close(mpsse_ctx);

	free(swd_cmd_queue);

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_device_desc_command)
{
	if (CMD_ARGC == 1) {
		if (ftdi_device_desc)
			free(ftdi_device_desc);
		ftdi_device_desc = strdup(CMD_ARGV[0]);
	} else {
		LOG_ERROR("expected exactly one argument to ftdi_device_desc <description>");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_serial_command)
{
	if (CMD_ARGC == 1) {
		if (ftdi_serial)
			free(ftdi_serial);
		ftdi_serial = strdup(CMD_ARGV[0]);
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

#ifdef HAVE_LIBUSB_GET_PORT_NUMBERS
COMMAND_HANDLER(ftdi_handle_location_command)
{
	if (CMD_ARGC == 1) {
		if (ftdi_location)
			free(ftdi_location);
		ftdi_location = strdup(CMD_ARGV[0]);
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}
#endif

COMMAND_HANDLER(ftdi_handle_channel_command)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], ftdi_channel);
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_layout_init_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], jtag_output_init);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], jtag_direction_init);

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_layout_signal_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	bool invert_data = false;
	uint16_t data_mask = 0;
	bool invert_oe = false;
	uint16_t oe_mask = 0;
	for (unsigned i = 1; i < CMD_ARGC; i += 2) {
		if (strcmp("-data", CMD_ARGV[i]) == 0) {
			invert_data = false;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], data_mask);
		} else if (strcmp("-ndata", CMD_ARGV[i]) == 0) {
			invert_data = true;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], data_mask);
		} else if (strcmp("-oe", CMD_ARGV[i]) == 0) {
			invert_oe = false;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], oe_mask);
		} else if (strcmp("-noe", CMD_ARGV[i]) == 0) {
			invert_oe = true;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], oe_mask);
		} else if (!strcmp("-alias", CMD_ARGV[i]) ||
			   !strcmp("-nalias", CMD_ARGV[i])) {
			if (!strcmp("-nalias", CMD_ARGV[i]))
				invert_data = true;
			struct signal *sig = find_signal_by_name(CMD_ARGV[i + 1]);
			if (!sig) {
				LOG_ERROR("signal %s is not defined", CMD_ARGV[i + 1]);
				return ERROR_FAIL;
			}
			data_mask = sig->data_mask;
			oe_mask = sig->oe_mask;
			invert_oe = sig->invert_oe;
			invert_data ^= sig->invert_data;
		} else {
			LOG_ERROR("unknown option '%s'", CMD_ARGV[i]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	struct signal *sig;
	sig = find_signal_by_name(CMD_ARGV[0]);
	if (!sig)
		sig = create_signal(CMD_ARGV[0]);
	if (!sig) {
		LOG_ERROR("failed to create signal %s", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	sig->invert_data = invert_data;
	sig->data_mask = data_mask;
	sig->invert_oe = invert_oe;
	sig->oe_mask = oe_mask;

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_set_signal_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct signal *sig;
	sig = find_signal_by_name(CMD_ARGV[0]);
	if (!sig) {
		LOG_ERROR("interface configuration doesn't define signal '%s'", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	switch (*CMD_ARGV[1]) {
	case '0':
	case '1':
	case 'z':
	case 'Z':
		/* single character level specifier only */
		if (CMD_ARGV[1][1] == '\0') {
			ftdi_set_signal(sig, *CMD_ARGV[1]);
			break;
		}
	default:
		LOG_ERROR("unknown signal level '%s', use 0, 1 or z", CMD_ARGV[1]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return mpsse_flush(mpsse_ctx);
}

COMMAND_HANDLER(ftdi_handle_vid_pid_command)
{
	if (CMD_ARGC > MAX_USB_IDS * 2) {
		LOG_WARNING("ignoring extra IDs in ftdi_vid_pid "
			"(maximum is %d pairs)", MAX_USB_IDS);
		CMD_ARGC = MAX_USB_IDS * 2;
	}
	if (CMD_ARGC < 2 || (CMD_ARGC & 1)) {
		LOG_WARNING("incomplete ftdi_vid_pid configuration directive");
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		/* remove the incomplete trailing id */
		CMD_ARGC -= 1;
	}

	unsigned i;
	for (i = 0; i < CMD_ARGC; i += 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i], ftdi_vid[i >> 1]);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], ftdi_pid[i >> 1]);
	}

	/*
	 * Explicitly terminate, in case there are multiples instances of
	 * ftdi_vid_pid.
	 */
	ftdi_vid[i >> 1] = ftdi_pid[i >> 1] = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(ftdi_handle_tdo_sample_edge_command)
{
	Jim_Nvp *n;
	static const Jim_Nvp nvp_ftdi_jtag_modes[] = {
		{ .name = "rising", .value = JTAG_MODE },
		{ .name = "falling", .value = JTAG_MODE_ALT },
		{ .name = NULL, .value = -1 },
	};

	if (CMD_ARGC > 0) {
		n = Jim_Nvp_name2value_simple(nvp_ftdi_jtag_modes, CMD_ARGV[0]);
		if (n->name == NULL)
			return ERROR_COMMAND_SYNTAX_ERROR;
		ftdi_jtag_mode = n->value;

	}

	n = Jim_Nvp_value2name_simple(nvp_ftdi_jtag_modes, ftdi_jtag_mode);
	command_print(CMD_CTX, "ftdi samples TDO on %s edge of TCK", n->name);

	return ERROR_OK;
}

static const struct command_registration ftdi_command_handlers[] = {
	{
		.name = "ftdi_device_desc",
		.handler = &ftdi_handle_device_desc_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB device description of the FTDI device",
		.usage = "description_string",
	},
	{
		.name = "ftdi_serial",
		.handler = &ftdi_handle_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the FTDI device",
		.usage = "serial_string",
	},
#ifdef HAVE_LIBUSB_GET_PORT_NUMBERS
	{
		.name = "ftdi_location",
		.handler = &ftdi_handle_location_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB bus location of the FTDI device",
		.usage = "<bus>:port[,port]...",
	},
#endif
	{
		.name = "ftdi_channel",
		.handler = &ftdi_handle_channel_command,
		.mode = COMMAND_CONFIG,
		.help = "set the channel of the FTDI device that is used as JTAG",
		.usage = "(0-3)",
	},
	{
		.name = "ftdi_layout_init",
		.handler = &ftdi_handle_layout_init_command,
		.mode = COMMAND_CONFIG,
		.help = "initialize the FTDI GPIO signals used "
			"to control output-enables and reset signals",
		.usage = "data direction",
	},
	{
		.name = "ftdi_layout_signal",
		.handler = &ftdi_handle_layout_signal_command,
		.mode = COMMAND_ANY,
		.help = "define a signal controlled by one or more FTDI GPIO as data "
			"and/or output enable",
		.usage = "name [-data mask|-ndata mask] [-oe mask|-noe mask] [-alias|-nalias name]",
	},
	{
		.name = "ftdi_set_signal",
		.handler = &ftdi_handle_set_signal_command,
		.mode = COMMAND_EXEC,
		.help = "control a layout-specific signal",
		.usage = "name (1|0|z)",
	},
	{
		.name = "ftdi_vid_pid",
		.handler = &ftdi_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of the FTDI device",
		.usage = "(vid pid)* ",
	},
	{
		.name = "ftdi_tdo_sample_edge",
		.handler = &ftdi_handle_tdo_sample_edge_command,
		.mode = COMMAND_ANY,
		.help = "set which TCK clock edge is used for sampling TDO "
			"- default is rising-edge (Setting to falling-edge may "
			"allow signalling speed increase)",
		.usage = "(rising|falling)",
	},
	COMMAND_REGISTRATION_DONE
};

static int create_default_signal(const char *name, uint16_t data_mask)
{
	struct signal *sig = create_signal(name);
	if (!sig) {
		LOG_ERROR("failed to create signal %s", name);
		return ERROR_FAIL;
	}
	sig->invert_data = false;
	sig->data_mask = data_mask;
	sig->invert_oe = false;
	sig->oe_mask = 0;

	return ERROR_OK;
}

static int create_signals(void)
{
	if (create_default_signal("TCK", 0x01) != ERROR_OK)
		return ERROR_FAIL;
	if (create_default_signal("TDI", 0x02) != ERROR_OK)
		return ERROR_FAIL;
	if (create_default_signal("TDO", 0x04) != ERROR_OK)
		return ERROR_FAIL;
	if (create_default_signal("TMS", 0x08) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

static int ftdi_swd_init(void)
{
	LOG_INFO("FTDI SWD mode enabled");
	swd_mode = true;

	if (create_signals() != ERROR_OK)
		return ERROR_FAIL;

	swd_cmd_queue_alloced = 10;
	swd_cmd_queue = malloc(swd_cmd_queue_alloced * sizeof(*swd_cmd_queue));

	return swd_cmd_queue != NULL ? ERROR_OK : ERROR_FAIL;
}

static void ftdi_swd_swdio_en(bool enable)
{
	struct signal *oe = find_signal_by_name("SWDIO_OE");
	if (oe)
		ftdi_set_signal(oe, enable ? '1' : '0');
}

static size_t ftdi_add_turnaround(bool rnw)
{
	static bool last_rnw = 0;
	uint8_t data = 0;

	if (last_rnw != rnw) {
		if (rnw)
			ftdi_swd_swdio_en(rnw != 0 ? false : true);
		/* No members of swd_queue_entry are used here since the value is
		 * never read back.
		 */
		mpsse_clock_data_out(mpsse_ctx, &data, 0, 1, SWD_MODE);
		if (!rnw)
			ftdi_swd_swdio_en(rnw != 0 ? false : true);
		last_rnw = rnw;
		return 1;
	}
	return 0;
}

/**
 * @brief Let the ftdi driver know that an SWD wait response was encountered.
 *
 * When this happens the driver is moved from queued mode to wait mode.  In
 * queued mode as many commands as requested are queued and then sent out when
 * the upper level driver requets a flush (ftdi_swd_run_queue()).
 *
 * In wait mode, each transaction flushes the queue so that the response from
 * the target can be checked.  If a WAIT response is received, the command is
 * retransmitted as defined in the ARM spec.
 */
void ftdi_wait_encountered(void)
{
	if (ftdi_swd_wait_remaining == 0)
		LOG_DEBUG("Wait encountered, switching to WAIT mode.");
	ftdi_swd_wait_remaining = FTDI_SWD_WAIT_COUNT;
}

/*
 * Flush the buffer if in wait mode and return true.  If in queue mode, then
 * return false, and do not flush.
 */
static inline bool ftdi_flush_if_wait_mode(void)
{
	if (ftdi_swd_wait_remaining != 0) {
		queued_retval = mpsse_flush(mpsse_ctx);
		return true;
	}
	return false;
}

/*
 * Replays the queue from start to end.  A copy of the queue is made, since if
 * this is the first element in the queue it would be overwritten.
 *
 * Note: This should only be called after the mode is switched to wait mode.
 */
static int ftdi_replay_queue(size_t start)
{
	struct swd_cmd_queue_entry *replay_queue;
	int retval = ERROR_FAIL;
	size_t i;
	size_t num_entries = swd_cmd_queue_length-start;

	if (ftdi_swd_wait_remaining != 0) {
		ftdi_swd_wait_remaining += num_entries;
		replay_queue = (struct swd_cmd_queue_entry *)malloc(sizeof(struct swd_cmd_queue_entry)*num_entries);
		if (replay_queue != NULL) {
			memcpy(replay_queue, &swd_cmd_queue[start], sizeof(struct swd_cmd_queue_entry)*num_entries);
			for (i = 0; i < num_entries; i++) {
				ftdi_swd_queue_cmd(replay_queue[i].cmd, replay_queue[i].dst,
				                   le_to_h_u32(replay_queue[i].data), 0);
			}
		}
	}
	return retval;
}

/**
 * @brief Flush the MPSSE queue and process the SWD transaction queue
 *
 * @return ERROR_OK - No problems encountered or an error code.
 */
static int _ftdi_swd_run_queue(void)
{
	LOG_DEBUG("Executing %zu queued transactions", swd_cmd_queue_length);
	int retval;
	struct signal *led = find_signal_by_name("LED");

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skipping due to previous errors: %d", queued_retval);
		goto skip;
	}

	/* A transaction must be followed by another transaction or at least 8 idle cycles to
	 * ensure that data is clocked through the AP. */
	mpsse_clock_data_out(mpsse_ctx, NULL, 0, 8, SWD_MODE);

	/* Terminate the "blink", if the current layout has that feature */
	if (led)
		ftdi_set_signal(led, '0');

	queued_retval = mpsse_flush(mpsse_ctx);
	if (queued_retval != ERROR_OK) {
		LOG_ERROR("MPSSE failed");
		goto skip;
	}

	if (ftdi_swd_wait_remaining == 0) {
		for (size_t i = 0; i < swd_cmd_queue_length; i++) {
			unsigned int ack = swd_cmd_queue[i].ack;
			uint32_t data = le_to_h_u32(swd_cmd_queue[i].data);

			LOG_DEBUG("%s %s %s reg %" PRIx32 " = %08" PRIx32,
					  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
					  swd_cmd_queue[i].cmd & SWD_CMD_APnDP ? "AP" : "DP",
					  swd_cmd_queue[i].cmd & SWD_CMD_RnW ? "read" : "write",
					  (swd_cmd_queue[i].cmd & SWD_CMD_A32) >> 1,
					  data);

			if (ack != SWD_ACK_OK) {
				queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
				if (ack == SWD_ACK_WAIT) {
					ftdi_wait_encountered();
					ftdi_replay_queue(i);
				}
				goto skip;

			} else if (swd_cmd_queue[i].cmd & SWD_CMD_RnW) {
				int parity = (int)swd_cmd_queue[i].parity&0x01;

				if (parity != parity_u32(data)) {
					LOG_ERROR("SWD Read data parity mismatch (data: %08" PRIx32 " - %" PRIx32 ")", data, parity);
					queued_retval = ERROR_FAIL;
					goto skip;
				}

				if (swd_cmd_queue[i].dst != NULL)
					*swd_cmd_queue[i].dst = data;
			}
		}
	}
skip:
	swd_cmd_queue_length = 0;
	retval = queued_retval;
	queued_retval = ERROR_OK;

	/* Queue a new "blink" */
	if (led && retval == ERROR_OK)
		ftdi_set_signal(led, '1');

	return retval;
}

static int ftdi_swd_run_queue(void)
{
	return _ftdi_swd_run_queue();
}

static void ftdi_read_cmd_data(uint32_t *dst)
{
	int parity;
	uint32_t data;

	swd_cmd_queue[swd_cmd_queue_length].dst = dst;
	swd_cmd_queue[swd_cmd_queue_length].parity = 0;
	/* 32 data bits, one parity bit and 8 clock cycles before it can go idle (see section 5.5.1). */
	mpsse_clock_data_in(mpsse_ctx,
	                    (uint8_t *)&swd_cmd_queue[swd_cmd_queue_length].data,
	                    0, SWD_DATA_SZ, SWD_MODE);
	mpsse_clock_data_in(mpsse_ctx,
	                    &swd_cmd_queue[swd_cmd_queue_length].parity,
	                    0, SWD_PARITY_SZ, SWD_MODE);
	/* Return to write. */
	ftdi_add_turnaround(0);

	if (ftdi_flush_if_wait_mode())
	{
		if (queued_retval != ERROR_OK) {
			LOG_ERROR("MPSSE failed");
		}
		else
		{
			data = le_to_h_u32(swd_cmd_queue[swd_cmd_queue_length].data);
			parity = swd_cmd_queue[swd_cmd_queue_length].parity&0x01;
			if (dst != NULL) {
				if (parity != parity_u32(data)) {
					LOG_ERROR("SWD Read data parity mismatch");
					queued_retval = ERROR_FAIL;
				}
				*dst = data;
			}
			LOG_DEBUG("%s %s reg %X = %08" PRIx32,
					  swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_APnDP ? "AP" : "DP",
					  swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_RnW ? "read" : "write",
					  (swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_A32) >> 1,
					  data);
		}
	}
}

static void ftdi_send_cmd_data(uint32_t data)
{
	ftdi_add_turnaround(0);
	/* 32 data bits, one parity bit and 8 bits before the clock can go idle. */
	h_u32_to_le(swd_cmd_queue[swd_cmd_queue_length].data, data);
	swd_cmd_queue[swd_cmd_queue_length].parity = (uint8_t)parity_u32(data);
	mpsse_clock_data_out(mpsse_ctx, (uint8_t *)&swd_cmd_queue[swd_cmd_queue_length].data, 0, SWD_DATA_SZ, SWD_MODE);
	mpsse_clock_data_out(mpsse_ctx, &swd_cmd_queue[swd_cmd_queue_length].parity, 0, SWD_PARITY_SZ, SWD_MODE);
	if (ftdi_flush_if_wait_mode()) {
		LOG_DEBUG("%s %s reg %X = %08"PRIx32,
				  swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_APnDP ? "AP" : "DP",
				  swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_RnW ? "read" : "write",
				  (swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_A32) >> 1,
				  data);
		if (queued_retval != ERROR_OK) {
			LOG_ERROR("MPSSE failed");
		}
	}
}

static int ftdi_send_cmd_wait_ack(uint8_t cmd)
{
	int ack = SWD_ACK_OK;

	swd_cmd_queue[swd_cmd_queue_length].cmd = cmd | SWD_CMD_START | SWD_CMD_PARK;
	LOG_DEBUG("%s cmd: %x (%s %s reg %x)",
	          ftdi_swd_wait_remaining > 0 ? "Sending" : "Queueing",
	          (unsigned int)swd_cmd_queue[swd_cmd_queue_length].cmd,
	          swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_APnDP ? "AP" : "DP",
	          swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_RnW ? "read" : "write",
			  (swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_A32) >> 1);
	/* Make sure in write mode. */
	ftdi_add_turnaround(0);
	mpsse_clock_data_out(mpsse_ctx, &swd_cmd_queue[swd_cmd_queue_length].cmd, 0, SWD_CMD_SZ, SWD_MODE);
	/* Switch to read. */
	ftdi_add_turnaround(1);
	swd_cmd_queue[swd_cmd_queue_length].ack = 7;
	mpsse_clock_data_in(mpsse_ctx, &swd_cmd_queue[swd_cmd_queue_length].ack, 0, SWD_ACK_SZ, SWD_MODE);

	if (ftdi_flush_if_wait_mode()) {
		ack = swd_cmd_queue[swd_cmd_queue_length].ack;
		if (queued_retval != ERROR_OK) {
			LOG_ERROR("MPSSE failed");
			/* Force no retries with an invalid ACK. */
			return(0x7);
		}
		else
		{
			LOG_DEBUG("%s (%02x) %s %s reg %X",
					  ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
					  (unsigned int)ack,
					  swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_APnDP ? "AP" : "DP",
					  swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_RnW ? "read" : "write",
					  (swd_cmd_queue[swd_cmd_queue_length].cmd & SWD_CMD_A32) >> 1);
			if (ack != SWD_ACK_OK) {
				queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
			}
		}
	}
	return ack;
}

static void ftdi_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk)
{
	unsigned int ack;
	unsigned int retry;
	const uint8_t zero_data[5] = { 0, 0, 0, 0, 0 };
	uint32_t reg;

	queued_retval = ERROR_OK;
	/* Move one step closer to going back to queued mode, if in wait mode. */
	if (ftdi_swd_wait_remaining != 0) {
		ftdi_swd_wait_remaining--;
	}
	else {
		if (swd_cmd_queue_length >= swd_cmd_queue_alloced) {
			/* Not enough room in the queue. Run the queue and increase its size for next time.
			 * Note that it's not possible to avoid running the queue here, because mpsse contains
			 * pointers into the queue which may be invalid after the realloc. */
			queued_retval = ftdi_swd_run_queue();
			struct swd_cmd_queue_entry *q = realloc(swd_cmd_queue, swd_cmd_queue_alloced * 2 * sizeof(*swd_cmd_queue));
			if (q != NULL) {
				swd_cmd_queue = q;
				swd_cmd_queue_alloced *= 2;
				LOG_DEBUG("Increased SWD command queue to %zu elements", swd_cmd_queue_alloced);
			}
		}
		if (queued_retval != ERROR_OK)
			return;
	}

	retry = 0;
	do {
		ack = ftdi_send_cmd_wait_ack(cmd);
		/*
		 * If ACK was either FAULT or WAIT and overrun detection is on a data phase is required.  If
		 * the ACK is not recoginized, then simply abort.
		 */
		switch (ack) {
			case SWD_ACK_OK:
				if (cmd & SWD_CMD_RnW) {
					/* Queue a read transaction */
					ftdi_read_cmd_data(dst);
				} else {
					/* Queue a write transaction */
					ftdi_send_cmd_data(data);
				}
				break;
			case SWD_ACK_WAIT:
				ftdi_wait_encountered();
			case SWD_ACK_FAULT:
				mpsse_clock_data_out(mpsse_ctx, zero_data, 0, 33, SWD_MODE);
				ftdi_add_turnaround(0);
				/* Find out what cause the fault and clear it out. */
				if (ack == SWD_ACK_FAULT) {
					(void)ftdi_send_cmd_wait_ack(swd_cmd(true, false, DP_CTRL_STAT));
					ftdi_read_cmd_data(&reg);
					LOG_DEBUG("Fault detected CTRL_STAT: %08" PRIx32, reg);
					(void)ftdi_send_cmd_wait_ack(swd_cmd(false, false, DP_ABORT));
					ftdi_send_cmd_data(STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR);
				}
				break;
			default:
				/* Invalid, perform a shift data just in case. */
				mpsse_clock_data_out(mpsse_ctx, zero_data, 0, 33, SWD_MODE);
				ftdi_add_turnaround(0);
				break;
		}
	} while ((ack != SWD_ACK_OK) && (retry++ < 100));

    if (ack != SWD_ACK_OK) {
	    queued_retval = ack == SWD_ACK_WAIT ? ERROR_WAIT : ERROR_FAIL;
	}
	/* Insert idle cycles after AP accesses to avoid WAIT */
	if (cmd & SWD_CMD_APnDP)
		mpsse_clock_data_out(mpsse_ctx, NULL, 0, ap_delay_clk, SWD_MODE);
	if (ftdi_swd_wait_remaining == 0)
		swd_cmd_queue_length++;
}

static void ftdi_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RnW);
	ftdi_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

static void ftdi_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RnW));
	ftdi_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static int_least32_t ftdi_swd_frequency(int_least32_t hz)
{
	if (hz > 0)
		freq = mpsse_set_frequency(mpsse_ctx, hz);

	return freq;
}

static int ftdi_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		mpsse_clock_data_out(mpsse_ctx, swd_seq_line_reset, 0, swd_seq_line_reset_len, SWD_MODE);
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		mpsse_clock_data_out(mpsse_ctx, swd_seq_jtag_to_swd, 0, swd_seq_jtag_to_swd_len, SWD_MODE);
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		mpsse_clock_data_out(mpsse_ctx, swd_seq_swd_to_jtag, 0, swd_seq_swd_to_jtag_len, SWD_MODE);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static const struct swd_driver ftdi_swd = {
	.init = ftdi_swd_init,
	.frequency = ftdi_swd_frequency,
	.switch_seq = ftdi_swd_switch_seq,
	.read_reg = ftdi_swd_read_reg,
	.write_reg = ftdi_swd_write_reg,
	.run = ftdi_swd_run_queue,
};

static const char * const ftdi_transports[] = { "jtag", "swd", NULL };

struct jtag_interface ftdi_interface = {
	.name = "ftdi",
	.supported = DEBUG_CAP_TMS_SEQ,
	.commands = ftdi_command_handlers,
	.transports = ftdi_transports,
	.swd = &ftdi_swd,

	.init = ftdi_initialize,
	.quit = ftdi_quit,
	.speed = ftdi_speed,
	.speed_div = ftdi_speed_div,
	.khz = ftdi_khz,
	.execute_queue = ftdi_execute_queue,
};
