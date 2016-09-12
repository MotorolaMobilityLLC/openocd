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
 * SPI flash driver for use on the Motorola mods high speed bridge IC.
 */

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif
#include "genspi.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/target.h>

/*
 * ES2/ES3 HSB Registers.
 */
#define HSBSPI_SCM_BASE                             0x40000000
#define HSBSPI_SCM_CLOCK_ENABLE0                    (HSBSPI_SCM_BASE+0x0300)
#define HSBSPI_SCM_CLOCK_ENABLE0_SPI_SCLK           (1<<24)
#define HSBSPI_SCM_CLOCK_ENABLE0_SPI_PCLK           (1<<23)

#define HSBSPI_SCM_PINSHARE                         (HSBSPI_SCM_BASE+0x0800)
#define HSBSPI_SCM_PINSHARE_SPI_DATA_GPIO           (1<<6)
#define HSBSPI_SCM_PINSHARE_SPI_CS0_GPIO            (1<<7)

#define HSBSPI_GPIO_BASE                            0x40003000
#define HSBSPI_GPIO_ODATASET_OFS                    0x0008
#define HSBSPI_GPIO_ODATACLR_OFS                    0x000C
#define HSBSPI_GPIO_DIROUT_OFS                      0x0014
#define HSBSPI_GPIO_DIRIN_OFS                       0x0018

#define HSBSPI_GPIO_SPI_MOSI                        (1<<13)
#define HSBSPI_GPIO_SPI_MISO                        (1<<14)
#define HSBSPI_GPIO_SPI_CS0                         (1<<15)

/* ES3 SPI Registers. */
#define HSBSPI_CTRL0                                0x0000
#define HSBSPI_CTRL0_DFS32_POS                      16
#define HSBSPI_CTRL0_DFS32_MSK                      (0x1f<<HSBSPI_CTRL0_DFS32_POS)
#define HSBSPI_CTRL0_CFS_POS                        12
#define HSBSPI_CTRL0_CFS_MSK                        (0x0f<<HSBSPI_CTRL0_CFS_POS)
#define HSBSPI_CTRL0_TMOD_POS                       8
#define HSBSPI_CTRL0_TMOD_MSK                       (0x03<<HSBSPI_CTRL0_TMOD_POS)
#define HSBSPI_CTRL0_TMOD_TX_RX                     (0x00<<HSBSPI_CTRL0_TMOD_POS)
#define HSBSPI_CTRL0_SCPOL_POS                      7
#define HSBSPI_CTRL0_SCPOL_MSK                      (0x01<<HSBSPI_CTRL0_SCPOL_POS)
#define HSBSPI_CTRL0_SCPH_POS                       6
#define HSBSPI_CTRL0_SCPH_MSK                       (0x01<<HSBSPI_CTRL0_SCPH_POS)
#define HSBSPI_CTRL0_FRF_POS                        4
#define HSBSPI_CTRL0_FRF_MSK                        (0x03<<HSBSPI_CTRL0_FRF_POS)
#define HSBSPI_CTRL0_FRF_MOT                        (0x02<<HSBSPI_CTRL0_FRF_POS)

#define HSBSPI_SSIENR                               0x0008
#define HSBSPI_SSIENR_SSI_EN_POS                    0
#define HSBSPI_SSIENR_SSI_EN_MSK                    (1<<HSBSPI_SSIENR_SSI_EN_POS)

#define HSBSPI_SER                                  0x0010
#define HSBSPI_SER_POS                              0
#define HSBSPI_SER_MSK                              (0x3<<HSBSPI_SER_POS)

#define HSBSPI_BAUDR                                0x0014
#define HSBSPI_BAUDR_POS                            0
#define HSBSPI_BAUDR_MSK                            (0xffff<<HSBSPI_BAUDR_POS)

#define HSBSPI_SR                                   0x0028
#define HSBSPI_SR_DCOL_POS                          6
#define HSBSPI_SR_DCOL_MSK                          (0x01<<HSBSPI_SR_DCOL_POS)
#define HSBSPI_SR_RFF_POS                           4
#define HSBSPI_SR_RFF_MSK                           (0x01<<HSBSPI_SR_RFF_POS)
#define HSBSPI_SR_RFNE_POS                          3
#define HSBSPI_SR_RFNE_MSK                          (0x01<<HSBSPI_SR_RFNE_POS)
#define HSBSPI_SR_TFE_POS                           2
#define HSBSPI_SR_TFE_MSK                           (0x01<<HSBSPI_SR_TFE_POS)
#define HSBSPI_SR_TFNF_POS                          1
#define HSBSPI_SR_TFNF_MSK                          (0x01<<HSBSPI_SR_TFNF_POS)
#define HSBSPI_SR_BUSY_POS                          0
#define HSBSPI_SR_BUSY_MSK                          (0x01<<HSBSPI_SR_BUSY_POS)

#define HSBSPI_DR0                                  0x0060
#define HSBSPI_DR0_POS                              0
#define HSBSPI_DR0_MSK                              (0xffffffff<<HSBSPI_DR0_POS)

#define HSBSPI_CMD_SZ                               6
#define HSBSPI_DATA_SZ                              4096

static const uint8_t hsbspi_flash_code[] =
{
                               /* hsbspi_flash:                                                 */
    0x01, 0xb3,                /*    0:   b301        cbz r1, 44 <done>                         */
    0xdf, 0xf8, 0x3c, 0x60,    /*    2:   f8df 603c   ldr.w   r6, [pc, #60]   ; 40 <spi_ctrl_0> */
    0x16, 0x60,                /*    6:   6016        str r6, [r2, #0]                          */
    0x4f, 0xf0, 0x00, 0x06,    /*    8:   f04f 0600   mov.w   r6, #0                            */
    0xd6, 0x62,                /*    c:   62d6        str r6, [r2, #44]   ; 0x2c                */
    0x96, 0x68,                /*    e:   6896        ldr r6, [r2, #8]                          */
    0x46, 0xf0, 0x01, 0x06,    /*   10:   f046 0601   orr.w   r6, r6, #1                        */
    0x96, 0x60,                /*   14:   6096        str r6, [r2, #8]                          */
    0x04, 0x46,                /*   16:   4604        mov r4, r0                                */
    0x0d, 0x46,                /*   18:   460d        mov r5, r1                                */
                               /* copy_tx_bytes:                                                */
    0x39, 0xb1,                /*   1a:   b139        cbz r1, 2c <copy_rx_bytes>                */
    0x96, 0x6a,                /*   1c:   6a96        ldr r6, [r2, #40]   ; 0x28                */
    0x16, 0xf0, 0x02, 0x0f,    /*   1e:   f016 0f02   tst.w   r6, #2                            */
    0x1e, 0xbf,                /*   22:   bf1e        ittt    ne                                */
    0x10, 0xf8, 0x01, 0x7b,    /*   24:   f810 7b01   ldrbne.w    r7, [r0], #1                  */
    0x17, 0x66,                /*   28:   6617        strne   r7, [r2, #96]   ; 0x60            */
    0x01, 0x39,                /*   2a:   3901        subne   r1, #1                            */
                               /* copy_rx_bytes:                                                */
    0x55, 0xb1,                /*   2c:   b155        cbz r5, 44 <done>                         */
    0x96, 0x6a,                /*   2e:   6a96        ldr r6, [r2, #40]   ; 0x28                */
    0x16, 0xf0, 0x08, 0x0f,    /*   30:   f016 0f08   tst.w   r6, #8                            */
    0x1e, 0xbf,                /*   34:   bf1e        ittt    ne                                */
    0x17, 0x6e,                /*   36:   6e17        ldrne   r7, [r2, #96]   ; 0x60            */
    0x04, 0xf8, 0x01, 0x7b,    /*   38:   f804 7b01   strbne.w    r7, [r4], #1                  */
    0x01, 0x3d,                /*   3c:   3d01        subne   r5, #1                            */
    0xec, 0xe7,                /*   3e:   e7ec        b.n 1a <copy_tx_bytes>                    */
                               /* spi_ctrl_0:                                                   */
    0x00, 0x00, 0x07, 0x00,    /*   40:   00070000    .word   0x00070000                        */
                               /* done:                                                         */
    0x97, 0x68,                /*   44:   6897        ldr r7, [r2, #8]                          */
    0x27, 0xf0, 0x01, 0x07,    /*   46:   f027 0701   bic.w   r7, r7, #1                        */
    0x97, 0x60,                /*   4a:   6097        str r7, [r2, #8]                          */
    0x00, 0xbe,                /*   4c:   be00        bkpt    0x0000                            */
};

struct hsbspi_algo_s
{
    struct working_area *buffer;
    bool buffer_allocated;
    struct working_area *code;
    bool code_allocated;
    struct reg_param reg_params[3];
    struct armv7m_algorithm armv7m_info;
};

static int hsbspi_cs(struct flash_bank *bank, bool enable)
{
    struct genspi_flash_bank *genspi_info = bank->driver_priv;
    uint32_t regofs;
    int retval = ERROR_OK;

    /* Toggle the chip select only if a GPIO chip select address is setup. */
    if (genspi_info->gpio_pin >= 0) {
        /* Active low chip select, the standard for most SPI flash parts. */
        regofs = HSBSPI_GPIO_ODATASET_OFS;
        if (enable) {
            regofs = HSBSPI_GPIO_ODATACLR_OFS;
        }
        LOG_DEBUG("Setting chip select with %08x = %08x",
                  genspi_info->gpio_base+regofs,
                  1 << genspi_info->gpio_pin);
        retval = target_write_u32(bank->target,
                                  genspi_info->gpio_base+regofs,
                                  1 << genspi_info->gpio_pin);
        if (retval != ERROR_OK) {
            LOG_ERROR("Unable to %s SPI chip select.\n", enable ? "enable" : "disable");
        }
    }
    return retval;
}

static void hsbspi_code_working_area_freed(struct target *target, struct working_area *wa, void *cb_data)
{
    struct flash_bank *bank = (struct flash_bank *)cb_data;
    struct genspi_flash_bank *genspi_info = bank->driver_priv;
    struct hsbspi_algo_s *hsbspi_algo = genspi_info->driver_priv;

    destroy_reg_param(&hsbspi_algo->reg_params[0]);
    destroy_reg_param(&hsbspi_algo->reg_params[1]);
    destroy_reg_param(&hsbspi_algo->reg_params[2]);
    hsbspi_algo->code_allocated = false;
}

static void hsbspi_buffer_working_area_freed(struct target *target, struct working_area *wa, void *cb_data)
{
    struct flash_bank *bank = (struct flash_bank *)cb_data;
    struct genspi_flash_bank *genspi_info = bank->driver_priv;
    struct hsbspi_algo_s *hsbspi_algo = genspi_info->driver_priv;

    hsbspi_algo->buffer_allocated = false;
}

static int hsbspi_alloc_wa(struct flash_bank *bank)
{
    struct target *target = bank->target;
    struct genspi_flash_bank *genspi_info = bank->driver_priv;
    struct hsbspi_algo_s *hsbspi_algo = genspi_info->driver_priv;
    uint32_t tmp_reg;
    int retval;

    if (hsbspi_algo->code_allocated == false) {
        /*
         * Get the working areas needed for the algorithm.  This assumes that
         * probe will be called once per halt.
         */
        if (target_alloc_working_area_cb(target,
                                         sizeof(hsbspi_flash_code),
                                         hsbspi_code_working_area_freed,
                                         bank,
                                         &hsbspi_algo->code) != ERROR_OK) {
            LOG_ERROR("A working area of at least %zu bytes is required for flash operation.",
                      sizeof(hsbspi_flash_code)+HSBSPI_CMD_SZ+HSBSPI_DATA_SZ);
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
        }
        hsbspi_algo->code_allocated = true;
        retval = target_write_buffer(target, hsbspi_algo->code->address,
                                     sizeof(hsbspi_flash_code),
                                     hsbspi_flash_code);

        init_reg_param(&hsbspi_algo->reg_params[0], "r0", 32, PARAM_IN);     /* buffer address */
        init_reg_param(&hsbspi_algo->reg_params[1], "r1", 32, PARAM_IN);     /* size in bytes */
        init_reg_param(&hsbspi_algo->reg_params[2], "r2", 32, PARAM_IN);     /* SPI register base address */

        if (retval != ERROR_OK) {
            LOG_ERROR("Unable to download flash code to working area.");
            target_free_working_area(target, hsbspi_algo->code);
            return retval;
        }
    }

    if (hsbspi_algo->buffer_allocated == false) {
        if (target_alloc_working_area_cb(target,
                                         HSBSPI_CMD_SZ+HSBSPI_DATA_SZ,
                                         hsbspi_buffer_working_area_freed,
                                         bank,
                                         &hsbspi_algo->buffer) != ERROR_OK) {
            LOG_ERROR("A working area of at least %zu bytes is required for flash operation.",
                      sizeof(hsbspi_flash_code)+HSBSPI_CMD_SZ+HSBSPI_DATA_SZ);
            target_free_working_area(target, hsbspi_algo->code);
            return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
        }
        hsbspi_algo->buffer_allocated = true;
        /* Setup the clock. */
        retval = target_read_u32(target, HSBSPI_SCM_CLOCK_ENABLE0, &tmp_reg);
        if (retval != ERROR_OK) {
            return retval;
        }
        tmp_reg |= HSBSPI_SCM_CLOCK_ENABLE0_SPI_SCLK | HSBSPI_SCM_CLOCK_ENABLE0_SPI_PCLK;
        retval = target_write_u32(target, HSBSPI_SCM_CLOCK_ENABLE0, tmp_reg);
        if (retval != ERROR_OK) {
            return retval;
        }

        /* Set the SPI rate to the max and start the clock. */
        retval = target_write_u32(target, genspi_info->reg_base+HSBSPI_BAUDR, 2);
        if (retval != ERROR_OK) {
            return retval;
        }

        /* Set the chip select. */
        retval = target_write_u32(target, genspi_info->reg_base+HSBSPI_SER, 1);
        if (retval != ERROR_OK) {
            return retval;
        }

        /* Make sure the pins are configured correctly. */
        retval = target_read_u32(target, HSBSPI_SCM_PINSHARE, &tmp_reg);
        if (retval != ERROR_OK) {
            return retval;
        }
        if (genspi_info->gpio_pin >= 0) {
            tmp_reg &= ~(HSBSPI_SCM_PINSHARE_SPI_DATA_GPIO);
            tmp_reg |= HSBSPI_SCM_PINSHARE_SPI_CS0_GPIO;
        }
        else {
            tmp_reg &= ~(HSBSPI_SCM_PINSHARE_SPI_DATA_GPIO | HSBSPI_SCM_PINSHARE_SPI_CS0_GPIO);
        }
        retval = target_write_u32(target, HSBSPI_SCM_PINSHARE, tmp_reg);
        if (retval != ERROR_OK) {
            return retval;
        }

        /* Set the chip select to high to disable the part. */
        retval = hsbspi_cs(bank, false);
        if (retval != ERROR_OK) {
            return retval;
        }

        /*
         * Setup the GPIO as inputs or outputs:
         *   GPIO13  SPI Data Out
         *   GPIO14  SPI Data In
         *   GPIO15  SPI CS
         * SPICLK is not shared with a GPIO.
         */
        retval = target_write_u32(target,
                                  genspi_info->gpio_base+HSBSPI_GPIO_DIROUT_OFS,
                                  HSBSPI_GPIO_SPI_MOSI | HSBSPI_GPIO_SPI_CS0);
        if (retval != ERROR_OK) {
            return retval;
        }
        retval = target_write_u32(target,
                                  genspi_info->gpio_base+HSBSPI_GPIO_DIRIN_OFS,
                                  HSBSPI_GPIO_SPI_MISO);
        if (retval != ERROR_OK) {
            return retval;
        }
    }
    return retval;
}

static int hsbspi_send_cmd(struct flash_bank *bank,
                           uint8_t *cmd,
                           size_t cmd_bytes,
                           uint8_t *data,
                           size_t data_bytes)
{
    struct genspi_flash_bank *genspi_info = bank->driver_priv;
    struct hsbspi_algo_s *hsbspi_algo = genspi_info->driver_priv;
    int retval;
#if defined(DEBUG_IO)
    uint8_t *buf;
    unsigned int i;
    char hexstr[12+3*16+1];
    char alpstr[16+1];
#endif

    if (cmd_bytes > HSBSPI_CMD_SZ) {
        LOG_ERROR("Too many command bytes (%zu) for TSB SPI.", cmd_bytes);
        return ERROR_BUF_TOO_SMALL;
    }
    if (data_bytes > HSBSPI_DATA_SZ) {
        LOG_ERROR("Too many data bytes (%zu) for TSB SPI.", data_bytes);
        return ERROR_BUF_TOO_SMALL;
    }

    retval = hsbspi_alloc_wa(bank);
    if (retval != ERROR_OK) {
        LOG_ERROR("Unable to allocate working areas needed for SPI.");
        return retval;
    }
	hsbspi_algo->armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	hsbspi_algo->armv7m_info.core_mode = ARM_MODE_THREAD;

	buf_set_u32(hsbspi_algo->reg_params[0].value, 0, 32, hsbspi_algo->buffer->address);
	buf_set_u32(hsbspi_algo->reg_params[1].value, 0, 32, cmd_bytes+data_bytes);
	buf_set_u32(hsbspi_algo->reg_params[2].value, 0, 32, genspi_info->reg_base);

    /* Copy the command into the buffer. */
    if ((cmd_bytes > 0) && (cmd != NULL)) {
        retval = target_write_buffer(bank->target,
                                     hsbspi_algo->buffer->address,
                                     cmd_bytes,
                                     cmd);
        if (retval != ERROR_OK) {
            return retval;
        }
    }

    /* Copy the data into the buffer. */
    if ((data_bytes > 0) && (data != NULL)) {
        retval = target_write_buffer(bank->target,
                                     hsbspi_algo->buffer->address+cmd_bytes,
                                     data_bytes,
                                     data);
        if (retval != ERROR_OK) {
            return retval;
        }
    }

#if defined(DEBUG_IO)
    /* DEBUG: Read the data back and display it. */
    buf = (uint8_t *)malloc(data_bytes+cmd_bytes);
    target_read_buffer(bank->target, hsbspi_algo->buffer->address, cmd_bytes+data_bytes, buf);
    for (i = 0; i < cmd_bytes+data_bytes; i++) {
        if ((i % 16) == 0) {
            sprintf(hexstr, "w %08x: ", hsbspi_algo->buffer->address+i);
            alpstr[0] = '\0';
        }
        sprintf(hexstr, "%s %02x", hexstr, buf[i]);
        sprintf(alpstr, "%s%c", alpstr, isprint(buf[i]) ? buf[i] : '.');
        if (((i & 0xf) == 0xf) || (i == cmd_bytes+data_bytes-1)) {
            LOG_DEBUG("%s %s", hexstr, alpstr);
        }
    }
#endif

    /* Enable the chip select. */
    retval = hsbspi_cs(bank, true);
    if (retval != ERROR_OK) {
        return retval;
    }

    /* Send the data. */
    LOG_DEBUG("Running flash algorithm.");
    retval = target_run_algorithm(bank->target,
                                  0,
                                  NULL,
                                  ARRAY_SIZE(hsbspi_algo->reg_params),
                                  hsbspi_algo->reg_params,
                                  hsbspi_algo->code->address,
                                  hsbspi_algo->code->address+sizeof(hsbspi_flash_code)-2,
                                  1000,
                                  &hsbspi_algo->armv7m_info);
    LOG_DEBUG("Flash algorithm exited.");
    if (retval != ERROR_OK) {
        LOG_ERROR("Error executing flash command algorithm.");
        return retval;
    }

    /* Disable the chip select. */
    retval = hsbspi_cs(bank, false);
    if (retval != ERROR_OK) {
        return retval;
    }

#if defined(DEBUG_IO)
    /* DEBUG: Read the data back and display it. */
    target_read_buffer(bank->target, hsbspi_algo->buffer->address, cmd_bytes+data_bytes, buf);
    hexstr[0] = '\0';
    alpstr[0] = '\0';
    for (i = 0; i < cmd_bytes+data_bytes; i++) {
        if ((i % 16) == 0) {
            sprintf(hexstr, "r %08x: ", hsbspi_algo->buffer->address+i);
            alpstr[0] = '\0';
        }
        sprintf(hexstr, "%s %02x", hexstr, buf[i]);
        sprintf(alpstr, "%s%c", alpstr, isprint(buf[i]) ? buf[i] : '.');
        if (((i & 0xf) == 0xf) || (i == cmd_bytes+data_bytes-1)) {
            LOG_DEBUG("%s %s", hexstr, alpstr);
        }
    }
    free(buf);
#endif

    /* Copy the command response. */
    if ((cmd_bytes > 0) && (cmd != NULL)) {
        retval = target_read_buffer(bank->target,
                                    hsbspi_algo->buffer->address,
                                    cmd_bytes,
                                    cmd);
        if (retval != ERROR_OK) {
            return retval;
        }
    }
    /* Copy the data response. */
    if ((data_bytes > 0) && (data != NULL)) {
        retval = target_read_buffer(bank->target,
                                    hsbspi_algo->buffer->address+cmd_bytes,
                                    data_bytes,
                                    data);
    }
    return retval;
}

static int hsbspi_init(struct flash_bank *bank)
{
    struct genspi_flash_bank *genspi_info = bank->driver_priv;

    genspi_info->driver_priv = malloc(sizeof(struct hsbspi_algo_s));
    if (genspi_info->driver_priv == NULL) {
        return(ERROR_FAIL);
    }
    memset(genspi_info->driver_priv, 0, sizeof(struct hsbspi_algo_s));
    return ERROR_OK;
}

static const struct genspi_fops hsbspi_fops =
{
    HSBSPI_DATA_SZ,
    hsbspi_send_cmd,
    hsbspi_init
};

static int hsbspi_probe(struct flash_bank *bank)
{
    return genspi_probe(bank, &hsbspi_fops);
}

static int hsbspi_auto_probe(struct flash_bank *bank)
{
    return genspi_auto_probe(bank, &hsbspi_fops);
}

struct flash_driver hsbspi_flash = {
    .name = "hsbspi",
    .usage = "<spi_reg_base_address> [chip_select_gpio_base chip_select_pin]",
    .flash_bank_command = genspi_flash_bank_command,
    .erase = genspi_flash_erase,
    .protect = NULL,
    .write = genspi_flash_write,
    .read = genspi_flash_read,
    .probe = hsbspi_probe,
    .auto_probe = hsbspi_auto_probe,
    .erase_check = genspi_flash_erase_check,
    .protect_check = genspi_protect_check,
    .info = genspi_get_info,
};
