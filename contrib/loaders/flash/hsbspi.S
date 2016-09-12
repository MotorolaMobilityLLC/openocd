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

    .text
    .syntax unified
    .cpu cortex-m3
    .thumb
    .thumb_func
/*
 * To assemble:
 *   arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -c contrib/loaders/flash/hsbspi.S
 *   tools/asm_to_c.pl hsbspi.o 0
 *   Copy the output into the correct source file.
 */

/* ES3 SPI Registers. */
#define HSBSPI_CTRL0                                0x0000
#define HSBSPI_CTRL0_DFS32_POS                      16
#define HSBSPI_CTRL0_DFS32_MSK                      (0x1f<<HSBSPI_CTRL0_DFS32_POS)
#define HSBSPI_CTRL0_DFS32(x)                       (((x-1)&0x1f)<<HSBSPI_CTRL0_DFS32_POS)
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
#define HSBSPI_CTRL0_FRF_MOTSPI                     (0x00<<HSBSPI_CTRL0_FRF_POS)

#define HSBSPI_CTRL1                                0x0004
#define HSBSPI_CTRL1_NDF_POS                        0
#define HSBSPI_CTRL1_NDF_MSK                        (0xffff<<HSBSPI_CTRL0_NDF_POS)

#define HSBSPI_SSIENR                               0x0008
#define HSBSPI_SSIENR_SSI_EN_POS                    0
#define HSBSPI_SSIENR_SSI_EN_MSK                    (1<<HSBSPI_SSIENR_SSI_EN_POS)

#define HSBSPI_SER                                  0x0010
#define HSBSPI_SER_POS                              0
#define HSBSPI_SER_MSK                              (0x3<<HSBSPI_SER_POS)

#define HSBSPI_BAUDR                                0x0014
#define HSBSPI_BAUDR_POS                            0
#define HSBSPI_BAUDR_MSK                            (0xffff<<HSBSPI_BAUDR_POS)

#define HSBSPI_IMR                                  0x002c

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

/*
 * Params:
 *   r0 - workarea start
 *   r1 - tx count (bytes)
 *   r2 - SPI Register base address
 * Clobbered:
 *   r4 - rx pointer
 *   r5 - rx count
 *   r6 - tmp (status register)
 *   r7 - tmp (copy)
 */

hsbspi_flash:
    cbz     r1,done

    /* Setup and enable the SPI. */
    ldr     r6,spi_ctrl_0
    str     r6,[r2,#HSBSPI_CTRL0]
    mov     r6,#0
    str     r6,[r2,HSBSPI_IMR]
    ldr     r6,[r2,#HSBSPI_SSIENR]
    orr     r6,r6,#HSBSPI_SSIENR_SSI_EN_MSK
    str     r6,[r2,#HSBSPI_SSIENR]

    mov     r4,r0
    mov     r5,r1

copy_tx_bytes:
    cbz     r1,copy_rx_bytes
    ldr     r6,[r2,#HSBSPI_SR]
    tst     r6,#HSBSPI_SR_TFNF_MSK
    ittt    ne
    ldrbne  r7,[r0],#1
    strne   r7,[r2,#HSBSPI_DR0]
    subne   r1,r1,#1

copy_rx_bytes:
    cbz     r5,done
    ldr     r6,[r2,#HSBSPI_SR]
    tst     r6,#HSBSPI_SR_RFNE_MSK
    ittt    ne
    ldrne   r7,[r2,HSBSPI_DR0]
    strbne  r7,[r4],#1
    subne   r5,r5,#1
    b       copy_tx_bytes

    /* Place the constant data before the end so no special processing needs to
       be done to calculate the address of the bkpt instruction.  For this it
       needs to be the last instruction. */
spi_ctrl_0:
    .word    HSBSPI_CTRL0_DFS32(8) | HSBSPI_CTRL0_FRF_MOTSPI

done:
    /* Disable the SPI. */
    ldr     r7,[r2,#HSBSPI_SSIENR]
    bic     r7,#HSBSPI_SSIENR_SSI_EN_MSK
    str     r7,[r2,#HSBSPI_SSIENR]
    bkpt    #0

