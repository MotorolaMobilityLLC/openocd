
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
#include "imp.h"
#include "spi.h"

struct genspi_flash_bank {
    int probed;
    uint32_t bank_num;
    uint32_t reg_base;
    uint32_t gpio_base;
    int32_t gpio_pin;
    const struct genspi_fops *fops;
    const struct flash_device *dev;
    void *driver_priv;
};

struct genspi_fops {
    uint32_t max_data_bytes;
    int (*spi_send_cmd)(struct flash_bank *bank, uint8_t *opcode, size_t cmd_bytes, uint8_t *data, size_t data_bytes);
    int (*spi_init)(struct flash_bank *bank);
};

int genspi_flash_erase(struct flash_bank *bank, int first, int last);
int genspi_flash_write(struct flash_bank *bank, const uint8_t *buffer,
                       uint32_t offset, uint32_t count);
int genspi_flash_read(struct flash_bank *bank, uint8_t *buffer,
				      uint32_t offset, uint32_t count);
int genspi_probe(struct flash_bank *bank, const struct genspi_fops *fops);
int genspi_auto_probe(struct flash_bank *bank, const struct genspi_fops *fops);
int genspi_flash_erase_check(struct flash_bank *bank);
int genspi_protect_check(struct flash_bank *bank);
int genspi_get_info(struct flash_bank *bank, char *buf, int buf_size);
__FLASH_BANK_COMMAND(genspi_flash_bank_command);

