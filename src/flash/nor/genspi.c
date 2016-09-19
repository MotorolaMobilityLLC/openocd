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
 * Generic utilities used for SPI flash drivers.  To use this call probe with
 * a time of generic function pointers.  This file will handle the generic
 * parts of the communication with OpenOCD as well as the SPI flash.  The
 * provided functions will be called when necessary.
 *
 * This was derived from mrlvqspi.c.
 */

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include "genspi.h"

#define CHIP_ERASE_TIMEOUT  (100)
#define CHIP_STATUS_TIMEOUT (10)

static int genspi_wait_busy(struct flash_bank *bank, unsigned int timeout)
{
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	uint8_t cmd_buf[1];
	uint8_t status[1];
	int retval;

	do {
		cmd_buf[0] = SPIFLASH_READ_STATUS;
		status[0] = 0x00;
		retval = genspi_info->fops->spi_send_cmd(bank, cmd_buf, sizeof(cmd_buf), status, sizeof(status));
		if (((status[0] & SPIFLASH_BSY_BIT) == 0) &&
		    (retval == ERROR_OK)) {
			return ERROR_OK;
		}
		alive_sleep(1);
	} while (timeout-- > 0);
	LOG_ERROR("Timed out waiting for busy.");
	return ERROR_WAIT;
}

static int genspi_wait_write_enable(struct flash_bank *bank, unsigned int timeout)
{
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	uint8_t cmd_buf[1];
	uint8_t status[1];
	int retval;

	do {
		cmd_buf[0] = SPIFLASH_READ_STATUS;
		status[0] = 0x00;
		retval = genspi_info->fops->spi_send_cmd(bank, cmd_buf, sizeof(cmd_buf), status, sizeof(status));
		if (((status[0] & SPIFLASH_WE_BIT) != 0) &&
		    (retval == ERROR_OK)) {
			return ERROR_OK;
		}
		alive_sleep(1);
	} while (timeout-- > 0);
	LOG_ERROR("Timed out waiting for write enable.");
	return ERROR_WAIT;
}

static int genspi_set_write_status(struct flash_bank *bank, bool mode)
{
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	int retval;
	uint8_t cmd_buf[1] = { SPIFLASH_WRITE_ENABLE };

	if (!mode) {
		cmd_buf[0] = SPIFLASH_WRITE_DISABLE;
	}
	retval = genspi_wait_busy(bank, CHIP_STATUS_TIMEOUT);
	if (retval == ERROR_OK) {
		retval = genspi_info->fops->spi_send_cmd(bank, cmd_buf, 1, NULL, 0);
	}
	if (retval == ERROR_OK) {
		retval = genspi_wait_write_enable(bank, CHIP_STATUS_TIMEOUT);
	}
	return retval;
}

int genspi_flash_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	uint8_t cmd_buf[4];
	int retval = ERROR_OK;
	int sector;

	LOG_DEBUG("Erase from sector %d to sector %d.", first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector range invalid (%d-%d).", first, last);
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(genspi_info->probed)) {
		LOG_ERROR("Flash bank not probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected.", sector);
			return ERROR_FAIL;
		}
	}

	/*
	 * If we're erasing the entire chip and the flash supports
	 * it, use a bulk erase instead of going sector-by-sector.
	 */
	if ((first == 0) && (last == (bank->num_sectors - 1)) &&
	    (genspi_info->dev->chip_erase_cmd != genspi_info->dev->erase_cmd)) {
		LOG_DEBUG("Chip supports the bulk erase command."\
		" Will use bulk erase instead of sector-by-sector erase.");
		/* Enable write access. */
		retval = genspi_set_write_status(bank, true);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to enable writes to flash part.");
			return retval;
		}
		cmd_buf[0] = genspi_info->dev->chip_erase_cmd;
		retval = genspi_info->fops->spi_send_cmd(bank, cmd_buf, 1, NULL, 0);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to send bulk erase command to flash part.");
			return retval;
		}
		retval = genspi_wait_busy(bank, CHIP_ERASE_TIMEOUT);
		if (retval == ERROR_OK) {
			return retval;
		} else
			LOG_WARNING("Bulk flash erase failed.  Falling back to sector-by-sector erase.");
	}

	for (sector = first; sector <= last; sector++) {
		/* Enable write access. */
		retval = genspi_set_write_status(bank, true);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to enable writes to flash part.");
			return retval;
		}
		cmd_buf[0] = genspi_info->dev->erase_cmd;
		cmd_buf[1] = (bank->sectors[sector].offset >> 16) & 0xff;
		cmd_buf[2] = (bank->sectors[sector].offset >> 8) & 0xff;
		cmd_buf[3] = (bank->sectors[sector].offset >> 0) & 0xff;
		LOG_DEBUG("Erasing %d bytes starting at 0x%08" PRIx32  ".", bank->sectors[sector].size, bank->sectors[sector].offset);
		retval = genspi_info->fops->spi_send_cmd(bank, cmd_buf, sizeof(cmd_buf), NULL, 0);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to send block erase command to flash part.");
			return retval;
		}
		retval = genspi_wait_busy(bank, CHIP_ERASE_TIMEOUT);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to erase block at address: %08" PRIx32, bank->sectors[sector].offset);
			return retval;
		}
	}
	retval = genspi_wait_busy(bank, CHIP_STATUS_TIMEOUT);
	return retval;
}

int genspi_flash_write(struct flash_bank *bank, const uint8_t *buffer,
                       uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	uint32_t bytes_written;
	uint32_t chunk;
	uint8_t cmd_buf[4];
	int sector;
	int retval;

	LOG_DEBUG("offset=0x%08" PRIx32 " count=0x%08" PRIx32, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(genspi_info->probed)) {
		LOG_ERROR("Flash bank not probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (offset + count > genspi_info->dev->size_in_bytes) {
		LOG_WARNING("Data goes past the last address.  Extra %lu bytes discarded.",
		            count - genspi_info->dev->size_in_bytes);
		count = genspi_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/*
		 * Start offset in or before this sector?
		 * End offset in or behind this sector?
		 */
		if ((offset < (bank->sectors[sector].offset + bank->sectors[sector].size)) &&
		    ((offset + count - 1) >= bank->sectors[sector].offset) &&
		    bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected.", sector);
			return ERROR_FAIL;
		}
	}

	/* Enable write access. */
	retval = genspi_set_write_status(bank, true);
	if (retval != ERROR_OK) {
		return retval;
	}

    bytes_written = 0;
    while (count) {
		/* Enable write access. */
		retval = genspi_set_write_status(bank, true);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to enable writes to flash part.");
			return retval;
		}

		/* Setup the command. */
		cmd_buf[0] = SPIFLASH_PAGE_PROGRAM;

		/* Setup the address. */
		cmd_buf[1] = (offset >> 16) & 0xff;
		cmd_buf[2] = (offset >> 8) & 0xff;
		cmd_buf[3] = (offset >> 0) & 0xff;

		/* Send the command and data.   This will trash the contents of buffer. */
		chunk = genspi_info->fops->max_data_bytes;
		if (chunk > genspi_info->dev->pagesize) {
			/* Make sure chunk does not go past the end of the page. */
			chunk = genspi_info->dev->pagesize-(offset%genspi_info->dev->pagesize);
		}

		if (chunk > count) {
			chunk = count;
		}

		retval = genspi_info->fops->spi_send_cmd(bank, cmd_buf, sizeof(cmd_buf), (uint8_t *)&buffer[bytes_written], chunk);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to program page at address: 0x%08" PRIx32 ".", offset);
			return retval;
		}
		bytes_written += chunk;
		offset += chunk;
		count -= chunk;
	}
	return genspi_wait_busy(bank, CHIP_ERASE_TIMEOUT);
}

int genspi_flash_read(struct flash_bank *bank, uint8_t *buffer,
				      uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	uint32_t bytes_copied;
	uint32_t chunk;
	uint8_t cmd_buf[5];
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!(genspi_info->probed)) {
		LOG_ERROR("Flash bank not probed.");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	retval = genspi_wait_busy(bank, CHIP_ERASE_TIMEOUT);
	if (retval != ERROR_OK) {
		LOG_ERROR("Busy check before read timed out.");
		return retval;
	}

	bytes_copied = 0;
	while (count)
	{
		/* Setup the command. */
		cmd_buf[0] = SPIFLASH_FAST_READ;

		/* Setup the address. */
		cmd_buf[1] = (offset >> 16) & 0xff;
		cmd_buf[2] = (offset >> 8) & 0xff;
		cmd_buf[3] = (offset >> 0) & 0xff;

		/* One dummy byte must be included before the data is read. */
		cmd_buf[4] = 0x00;

		/* Determine how many bytes to copy. */
		chunk = genspi_info->fops->max_data_bytes;
		if (chunk > count) {
			chunk = count;
		}

		/* Send the command and get the response. */
		memset(&buffer[bytes_copied], 0, chunk);
		retval = genspi_info->fops->spi_send_cmd(bank, cmd_buf, sizeof(cmd_buf), &buffer[bytes_copied], chunk);
		if (retval != ERROR_OK) {
			LOG_ERROR("Unable to read data at address: 0x%08" PRIx32 ".", offset);
			return retval;
		}
		bytes_copied += chunk;
		offset += chunk;
		count -= chunk;
	}
	return ERROR_OK;
}

int genspi_probe(struct flash_bank *bank, const struct genspi_fops *fops)
{
	struct target *target = bank->target;
	struct genspi_flash_bank *genspi_info = bank->driver_priv;
	uint32_t id = 0;
	int retval;
	struct flash_sector *sectors;
	uint8_t rx_buf[3];
	uint8_t cmd_buf[1];

	/* If we've already probed, we should be fine to skip this time. */
	if (genspi_info->probed) {
		return ERROR_OK;
	}

    if ((fops == NULL) ||
        (fops->spi_send_cmd == NULL)) {
        return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted.");
		return ERROR_TARGET_NOT_HALTED;
	}

    genspi_info->fops = fops;
	genspi_info->bank_num = bank->bank_number;

	if (fops->spi_init) {
		retval = fops->spi_init(bank);
		if (retval != ERROR_OK) {
			return retval;
		}
	}

	/* Read the flash JEDEC ID. */
	memset(rx_buf, 0x00, sizeof(rx_buf));
	cmd_buf[0] = SPIFLASH_READ_ID;
    retval = fops->spi_send_cmd(bank, cmd_buf, 1, rx_buf, sizeof(rx_buf));
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("ID is 0x%02" PRIx8 " 0x%02" PRIx8 " 0x%02" PRIx8,
					rx_buf[0], rx_buf[1], rx_buf[2]);

	id = rx_buf[2] << 16 | rx_buf[1] << 8 | rx_buf[0];

	genspi_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			genspi_info->dev = p;
			break;
		}

	if (!genspi_info->dev) {
		LOG_ERROR("Unknown flash device ID 0x%08" PRIx32, id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' ID 0x%08" PRIx32,
		genspi_info->dev->name, genspi_info->dev->device_id);

	/* Set correct size value. */
	bank->size = genspi_info->dev->size_in_bytes;

	/* Create and fill sectors array. */
	bank->num_sectors =
		genspi_info->dev->size_in_bytes / genspi_info->dev->sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("Insufficient memory.");
		return ERROR_FAIL;
	}

	for (int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset =
				sector * genspi_info->dev->sectorsize;
		sectors[sector].size = genspi_info->dev->sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	genspi_info->probed = 1;

	return ERROR_OK;
}

int genspi_auto_probe(struct flash_bank *bank, const struct genspi_fops *fops)
{
	struct genspi_flash_bank *genspi_info = bank->driver_priv;

	if (genspi_info->probed)
		return ERROR_OK;
	return genspi_probe(bank, fops);
}

int genspi_flash_erase_check(struct flash_bank *bank)
{
	/* Not implemented yet */
	return ERROR_OK;
}

int genspi_protect_check(struct flash_bank *bank)
{
	/* Not implemented yet */
	return ERROR_OK;
}

int genspi_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct genspi_flash_bank *genspi_info = bank->driver_priv;

	if (!(genspi_info->probed)) {
		snprintf(buf, buf_size,
			"\nSPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nSPI flash information:\n"
		"  Device \'%s\' ID 0x%08" PRIx32 "\n",
		genspi_info->dev->name, genspi_info->dev->device_id);

	return ERROR_OK;
}

__FLASH_BANK_COMMAND(genspi_flash_bank_command)
{
	struct genspi_flash_bank *genspi_info;

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	genspi_info = malloc(sizeof(struct genspi_flash_bank));
	if (genspi_info == NULL) {
		LOG_ERROR("insufficient memory");
		return ERROR_FAIL;
	}
	memset(genspi_info, 0, sizeof(*genspi_info));
	genspi_info->gpio_pin = -1;

	/* Get SPI controller register map base address */
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], genspi_info->reg_base);
	bank->driver_priv = genspi_info;
	genspi_info->probed = 0;
	LOG_DEBUG("Flash setup with SPI register base address: 0x%08x", genspi_info->reg_base);
	/* See if a GPIO was specified to use as Chip Select. */
	if (CMD_ARGC > 7) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], genspi_info->gpio_base);
		COMMAND_PARSE_NUMBER(s32, CMD_ARGV[8], genspi_info->gpio_pin);
		LOG_DEBUG("Flash Chip Select setup with base address: 0x%08x and pin %d",
		          genspi_info->gpio_base, genspi_info->gpio_pin);
	}
	/* Get the watchdog base address if one was specified. */
	if (CMD_ARGC > 9) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[9], genspi_info->watchdog_base);
		LOG_DEBUG("Watchdog setup with base address: 0x%08x.", genspi_info->watchdog_base);
	}

	return ERROR_OK;
}
