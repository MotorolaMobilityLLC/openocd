/***************************************************************************
 *   Copyright (C) 2016 Motorola Mobility LLC                              *
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
/*
 * SPI flash driver for use on the STM32 micro controllers.
 *
 * Before this code can be used the GPIO and clock clock tree must be setup.
 * Due to the differences between different versions of the STM32 parts it is
 * assumed the GPIO and clock are already setup and the clock does not exceed
 * 96MHz.  While the parts differ each seems to have an RCC enable bit which
 * must be enabled before using this code.  See the config file
 * mot_mdk_muc_common.cfg for an example.
 */

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif
#include "genspi.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/target.h>

#define STM32SPI_CR1_OFS      0x00
#define STM32SPI_CR1_SSM      (1<<9)
#define STM32SPI_CR1_SSI      (1<<8)
#define STM32SPI_CR1_SPE      (1<<6)
#define STM32SPI_CR1_BR_POS   3
#define STM32SPI_CR1_BR_MSK   (0x7<<STM32SPI_CR1_BR_POS)
#define STM32SPI_CR1_MSTR     (1<<2)
#define STM32SPI_CR1_CPOL     (1<<1)
#define STM32SPI_CR1_CPHA     (1<<0)

#define STM32SPI_CR2_OFS      0x04
#define STM32SPI_CR2_FRXTH    (1<<12)
#define STM32SPI_CR2_DS_MSK   (0xf<<8)
#define STM32SPI_CR2_DS_8BIT  (0x7<<8)
#define STM32SPI_CR2_FRF      (1<<4)
#define STM32SPI_CR2_SSOE     (1<<2)

#define STM32SPI_SR_OFS       0x08
#define STM32SPI_SR_FTLVL_MSK (0x3<<11)
#define STM32SPI_SR_FRLVL_MSK (0x3<<9)
#define STM32SPI_SR_BSY       (1<<7)
#define STM32SPI_SR_TXE       (1<<1)
#define STM32SPI_SR_RXNE      (1<<0)

#define STM32SPI_DR_OFS       0x0c

#define STM32GPIO_BSRR_OFS    0x18

#define DEBUG_IO              1

#define STM32SPI_CMD_SZ       6
#define STM32SPI_DATA_SZ      4096

/* See contrib/loaders/flash/stm32spi.S. */
static const uint8_t stm32spi_flash_code[] =
{
							   /* stm32spi_flash:                                                   */
	0xe9, 0xb1,                /*    0:   b1e9        cbz r1, 3e <done>                             */
	0x13, 0xb1,                /*    2:   b113        cbz r3, a <enable_spi>                        */
	0xbf, 0xf8, 0x34, 0x60,    /*    4:   f8bf 6034   ldrh.w  r6, [pc, #52]   ; 3c <wdog_key_value> */
	0x1e, 0x80,                /*    8:   801e        strh    r6, [r3, #0]                          */
							   /* enable_spi:                                                       */
	0x16, 0x88,                /*    a:   8816        ldrh    r6, [r2, #0]                          */
	0x46, 0xf0, 0x40, 0x06,    /*    c:   f046 0640   orr.w   r6, r6, #64 ; 0x40                    */
	0x16, 0x80,                /*   10:   8016        strh    r6, [r2, #0]                          */
	0x04, 0x46,                /*   12:   4604        mov r4, r0                                    */
	0x0d, 0x46,                /*   14:   460d        mov r5, r1                                    */
							   /* copy_tx_bytes:                                                    */
	0x39, 0xb1,                /*   16:   b139        cbz r1, 28 <copy_rx_bytes>                    */
	0x16, 0x89,                /*   18:   8916        ldrh    r6, [r2, #8]                          */
	0x16, 0xf0, 0x02, 0x0f,    /*   1a:   f016 0f02   tst.w   r6, #2                                */
	0x1e, 0xbf,                /*   1e:   bf1e        ittt    ne                                    */
	0x10, 0xf8, 0x01, 0x7b,    /*   20:   f810 7b01   ldrbne.w    r7, [r0], #1                      */
	0x17, 0x73,                /*   24:   7317        strbne  r7, [r2, #12]                         */
	0x01, 0x39,                /*   26:   3901        subne   r1, #1                                */
							   /* copy_rx_bytes:                                                    */
	0x4d, 0xb1,                /*   28:   b14d        cbz r5, 3e <done>                             */
	0x16, 0x89,                /*   2a:   8916        ldrh    r6, [r2, #8]                          */
	0x16, 0xf0, 0x01, 0x0f,    /*   2c:   f016 0f01   tst.w   r6, #1                                */
	0x1e, 0xbf,                /*   30:   bf1e        ittt    ne                                    */
	0x17, 0x7b,                /*   32:   7b17        ldrbne  r7, [r2, #12]                         */
	0x04, 0xf8, 0x01, 0x7b,    /*   34:   f804 7b01   strbne.w    r7, [r4], #1                      */
	0x01, 0x3d,                /*   38:   3d01        subne   r5, #1                                */
	0xec, 0xe7,                /*   3a:   e7ec        b.n 16 <copy_tx_bytes>                        */
							   /* wdog_key_value:                                                   */
	0xaa, 0xaa,                /*   3c:   aaaa        .short  0xaaaa                                */
							   /* done:                                                             */
	0x17, 0x89,                /*   3e:   8917        ldrh    r7, [r2, #8]                          */
	0x17, 0xf0, 0x80, 0x0f,    /*   40:   f017 0f80   tst.w   r7, #128    ; 0x80                    */
	0xfb, 0xd1,                /*   44:   d1fb        bne.n   3e <done>                             */
	0x17, 0x88,                /*   46:   8817        ldrh    r7, [r2, #0]                          */
	0x27, 0xf0, 0x40, 0x07,    /*   48:   f027 0740   bic.w   r7, r7, #64 ; 0x40                    */
	0x17, 0x80,                /*   4c:   8017        strh    r7, [r2, #0]                          */
	0x00, 0xbe,                /*   4e:   be00        bkpt    0x0000                                */
};

struct stm32spi_algo_s
{
	struct working_area *buffer;
	bool buffer_allocated;
	struct working_area *code;
	bool code_allocated;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
};

static int stm32spi_cs(struct flash_bank *bank, bool enable)
{
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	uint32_t tmpreg;
	int retval = ERROR_OK;

	/* Toggle the chip select only if a GPIO chip select address is setup. */
	if (genspi_info->gpio_pin >= 0) {
		/* Active low chip select, the standard for most SPI flash parts. */
		tmpreg = (1 << genspi_info->gpio_pin) << (enable ? 16 : 0);
		LOG_DEBUG("Setting chip select with %08x = %08x", genspi_info->gpio_base+STM32GPIO_BSRR_OFS, tmpreg);
		retval = target_write_u32(bank->target, genspi_info->gpio_base+STM32GPIO_BSRR_OFS, tmpreg);
		if (retval != ERROR_OK)
			LOG_ERROR("Unable to %s SPI chip select.\n", enable ? "enable" : "disable");
	}
	return retval;
}

static void stm32spi_code_working_area_freed(struct target *target, struct working_area *wa, void *cb_data)
{
	struct flash_bank *bank = (struct flash_bank *)cb_data;
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	struct stm32spi_algo_s *stm32spi_algo = genspi_info->driver_priv;

	destroy_reg_param(&stm32spi_algo->reg_params[0]);
	destroy_reg_param(&stm32spi_algo->reg_params[1]);
	destroy_reg_param(&stm32spi_algo->reg_params[2]);
	destroy_reg_param(&stm32spi_algo->reg_params[3]);
	stm32spi_algo->code_allocated = false;
}

static void stm32spi_buffer_working_area_freed(struct target *target, struct working_area *wa, void *cb_data)
{
	struct flash_bank *bank = (struct flash_bank *)cb_data;
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	struct stm32spi_algo_s *stm32spi_algo = genspi_info->driver_priv;

	stm32spi_algo->buffer_allocated = false;
}

static int stm32spi_allocte_wa(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	struct stm32spi_algo_s *stm32spi_algo = genspi_info->driver_priv;
	uint16_t cr1;
	int retval = ERROR_OK;

	if (stm32spi_algo == NULL) {
		LOG_ERROR("Driver private data not allocated.");
		return ERROR_FAIL;
	}

	if (stm32spi_algo->code_allocated == false) {
		/*
		 * Get the working areas needed for the algorithm.  This assumes that
		 * probe will be called once per halt.
		 */
		if (target_alloc_working_area_cb(target,
										sizeof(stm32spi_flash_code),
										stm32spi_code_working_area_freed,
										bank,
										&stm32spi_algo->code) != ERROR_OK) {
			LOG_ERROR("A working area of at least %zu bytes is required for flash operation.",
					  sizeof(stm32spi_flash_code)+STM32SPI_CMD_SZ+STM32SPI_DATA_SZ);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		stm32spi_algo->code_allocated = true;
		retval = target_write_buffer(target, stm32spi_algo->code->address,
									sizeof(stm32spi_flash_code),
									stm32spi_flash_code);

		init_reg_param(&stm32spi_algo->reg_params[0], "r0", 32, PARAM_IN);     /* Buffer address */
		init_reg_param(&stm32spi_algo->reg_params[1], "r1", 32, PARAM_IN);     /* Size in bytes */
		init_reg_param(&stm32spi_algo->reg_params[2], "r2", 32, PARAM_IN);     /* SPI base address */
		init_reg_param(&stm32spi_algo->reg_params[3], "r3", 32, PARAM_IN);     /* Watchdog base address */

		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to download flash code to working area.");
			target_free_working_area(target, stm32spi_algo->code);
			return retval;
		}
	}

	if (stm32spi_algo->buffer_allocated == false) {
		if (target_alloc_working_area_cb(target,
										STM32SPI_CMD_SZ+STM32SPI_DATA_SZ,
										stm32spi_buffer_working_area_freed,
										bank,
										&stm32spi_algo->buffer) != ERROR_OK) {
			LOG_ERROR("A working area of at least %zu bytes is required for flash operation.",
					sizeof(stm32spi_flash_code)+STM32SPI_CMD_SZ+STM32SPI_DATA_SZ);
			target_free_working_area(target, stm32spi_algo->code);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		stm32spi_algo->buffer_allocated = true;
		/* Set up for communication with most SPI flash parts. */
		cr1 = ((0 << STM32SPI_CR1_BR_POS) & STM32SPI_CR1_BR_MSK) | STM32SPI_CR1_MSTR;
		if (genspi_info->gpio_pin >= 0)
			cr1 |= STM32SPI_CR1_SSI | STM32SPI_CR1_SSM;

		retval = target_write_u16(target, genspi_info->reg_base+STM32SPI_CR1_OFS, cr1);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to write to the SPI confg 1 register.");
			return retval;
		}
		retval = target_write_u16(target, genspi_info->reg_base+STM32SPI_CR2_OFS,
								  (STM32SPI_CR2_FRXTH |
								   STM32SPI_CR2_DS_8BIT |
								   STM32SPI_CR2_SSOE));
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to write to the SPI confg 2 register.");
			return retval;
		}
		/* Make sure the chip select starts off disabled if configured. */
		retval = stm32spi_cs(bank, false);
	}
	return retval;
}

static int stm32spi_send_cmd(struct flash_bank *bank,
						   uint8_t *cmd,
						   size_t cmd_bytes,
						   uint8_t *data,
						   size_t data_bytes)
{
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	struct stm32spi_algo_s *stm32spi_algo = genspi_info->driver_priv;
	int retval;
#if defined(DEBUG_IO)
	uint8_t *buf;
	unsigned int i;
	char hexstr[12+3*16+1];
	char alpstr[16+1];
#endif

	if (cmd_bytes > STM32SPI_CMD_SZ) {
		LOG_ERROR("Too many command bytes (%zu) for TSB SPI.", cmd_bytes);
		return ERROR_BUF_TOO_SMALL;
	}
	if (data_bytes > STM32SPI_DATA_SZ) {
		LOG_ERROR("Too many data bytes (%zu) for TSB SPI.", data_bytes);
		return ERROR_BUF_TOO_SMALL;
	}

	retval = stm32spi_allocte_wa(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unable to allocate working areas needed for SPI.");
		return retval;
	}
	stm32spi_algo->armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	stm32spi_algo->armv7m_info.core_mode = ARM_MODE_THREAD;

	buf_set_u32(stm32spi_algo->reg_params[0].value, 0, 32, stm32spi_algo->buffer->address);
	buf_set_u32(stm32spi_algo->reg_params[1].value, 0, 32, cmd_bytes+data_bytes);
	buf_set_u32(stm32spi_algo->reg_params[2].value, 0, 32, genspi_info->reg_base);
	buf_set_u32(stm32spi_algo->reg_params[3].value, 0, 32, genspi_info->watchdog_base);

	/* Copy the command into the buffer. */
	if ((cmd_bytes > 0) && (cmd != NULL)) {
		retval = target_write_buffer(bank->target,
									 stm32spi_algo->buffer->address,
									 cmd_bytes,
									 cmd);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to copy command into target buffer.");
			return retval;
		}
	}

	/* Copy the data into the buffer. */
	if ((data_bytes > 0) && (data != NULL)) {
		retval = target_write_buffer(bank->target,
									 stm32spi_algo->buffer->address+cmd_bytes,
									 data_bytes,
									 data);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to copy command data into target buffer.");
			return retval;
		}
	}

#if defined(DEBUG_IO)
	/* DEBUG: Read the data back and display it. */
	buf = (uint8_t *)malloc(data_bytes+cmd_bytes);
	target_read_buffer(bank->target, stm32spi_algo->buffer->address, cmd_bytes+data_bytes, buf);
	for (i = 0; i < cmd_bytes+data_bytes; i++) {
		if ((i % 16) == 0) {
			sprintf(hexstr, "w %08x: ", stm32spi_algo->buffer->address+i);
			alpstr[0] = '\0';
		}
		sprintf(hexstr, "%s %02x", hexstr, buf[i]);
		sprintf(alpstr, "%s%c", alpstr, isprint(buf[i]) ? buf[i] : '.');
		if (((i & 0xf) == 0xf) || (i == cmd_bytes+data_bytes-1))
			LOG_DEBUG("%s %s", hexstr, alpstr);
	}
#endif
	/* Enable the chip select if required by the config. */
	retval = stm32spi_cs(bank, true);
	if (retval != ERROR_OK)
		return retval;
	/* Send the data. */
	LOG_DEBUG("Running flash algorithm.");
	retval = target_run_algorithm(bank->target,
								  0,
								  NULL,
								  ARRAY_SIZE(stm32spi_algo->reg_params),
								  stm32spi_algo->reg_params,
								  stm32spi_algo->code->address,
								  stm32spi_algo->code->address+sizeof(stm32spi_flash_code)-2,
								  1000,
								  &stm32spi_algo->armv7m_info);
	LOG_DEBUG("Flash algorithm exited.");
	if (retval != ERROR_OK) {
		LOG_ERROR("Error executing flash command algorithm.");
		return retval;
	}

	retval = stm32spi_cs(bank, false);
	if (retval != ERROR_OK)
		return retval;

#if defined(DEBUG_IO)
	/* DEBUG: Read the data back and display it. */
	target_read_buffer(bank->target, stm32spi_algo->buffer->address, cmd_bytes+data_bytes, buf);
	hexstr[0] = '\0';
	alpstr[0] = '\0';
	for (i = 0; i < cmd_bytes+data_bytes; i++) {
		if ((i % 16) == 0) {
			sprintf(hexstr, "r %08x: ", stm32spi_algo->buffer->address+i);
			alpstr[0] = '\0';
		}
		sprintf(hexstr, "%s %02x", hexstr, buf[i]);
		sprintf(alpstr, "%s%c", alpstr, isprint(buf[i]) ? buf[i] : '.');
		if (((i & 0xf) == 0xf) || (i == cmd_bytes+data_bytes-1))
			LOG_DEBUG("%s %s", hexstr, alpstr);
	}
	free(buf);
#endif

	/* Copy the command response. */
	if ((cmd_bytes > 0) && (cmd != NULL)) {
		retval = target_read_buffer(bank->target,
									stm32spi_algo->buffer->address,
									cmd_bytes,
									cmd);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to copy command response from target buffer.");
			return retval;
		}
	}
	/* Copy the data response. */
	if ((data_bytes > 0) && (data != NULL)) {
		retval = target_read_buffer(bank->target,
									stm32spi_algo->buffer->address+cmd_bytes,
									data_bytes,
									data);
		if (retval != ERROR_OK)
			LOG_ERROR("Unable to copy command data from target buffer.");
	}
	return retval;
}

static int stm32spi_init(struct flash_bank *bank)
{
	struct genspi_flash_bank *genspi_info = bank->driver_priv;

	if (genspi_info->driver_priv == NULL) {
		genspi_info->driver_priv = malloc(sizeof(struct stm32spi_algo_s));
		if (genspi_info->driver_priv == NULL) {
			LOG_ERROR("Unable to allocate driver memory.");
			return ERROR_FAIL;
		}
		memset(genspi_info->driver_priv, 0, sizeof(struct stm32spi_algo_s));
	}
	return ERROR_OK;
}

static const struct genspi_fops stm32spi_fops = {
	STM32SPI_DATA_SZ,
	stm32spi_send_cmd,
	stm32spi_init
};

static int stm32spi_probe(struct flash_bank *bank)
{
	return genspi_probe(bank, &stm32spi_fops);
}

static int stm32spi_auto_probe(struct flash_bank *bank)
{
	return genspi_auto_probe(bank, &stm32spi_fops);
}

struct flash_driver stm32spi_flash = {
	.name = "stm32spi",
	.usage = "<spi_reg_base_address> [chip_select_gpio_base chip_select_pin]",
	.flash_bank_command = genspi_flash_bank_command,
	.erase = genspi_flash_erase,
	.protect = NULL,
	.write = genspi_flash_write,
	.read = genspi_flash_read,
	.probe = stm32spi_probe,
	.auto_probe = stm32spi_auto_probe,
	.erase_check = genspi_flash_erase_check,
	.protect_check = genspi_protect_check,
	.info = genspi_get_info,
};
