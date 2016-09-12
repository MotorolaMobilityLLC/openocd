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
    .cpu cortex-m4
    .thumb
    .thumb_func
/*
 * To assemble:
 *   arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -c contrib/loaders/flash/stm32spi.S
 *   tools/asm_to_c.pl stm32spi.o 0
 *   Copy the output into the correct source file.
 */

#define STM32SPI_CR1_OFS      0x00
#define STM32SPI_CR1_SPE      (1<<6)
#define STM32SPI_CR1_BR_POS   3
#define STM32SPI_CR1_BR_MSK   (0x7<<STM32SPI_SPI_CR1_BR_POS)
#define STM32SPI_CR1_MSTR     (1<<2)
#define STM32SPI_CR1_CPOL     (1<<1)
#define STM32SPI_CR1_CPHA     (1<<0)

#define STM32SPI_CR2_OFS      0x04
#define STM32SPI_CR2_DS_MSK   (0xf<<8)
#define STM32SPI_CR2_DS_8BIT  (0x7<<8)
#define STM32SPI_CR2_FRF      (1<<4)
#define STM32SPI_CR2_SSOE     (1<<2)

#define STM32SPI_SR_OFS        0x08
#define STM32SPI_SR_FTLVL_MSK (0x3<<11)
#define STM32SPI_SR_FRLVL_MSK (0x3<<9)
#define STM32SPI_SR_BSY       (1<<7)
#define STM32SPI_SR_TXE       (1<<1)
#define STM32SPI_SR_RXNE      (1<<0)

#define STM32SPI_DR_OFS       0x0c

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

stm32spi_flash:
    cbz     r1,done

    /* Enable the SPI. */
    ldrh    r6,[r2,#STM32SPI_CR1_OFS]
    orr     r6,r6,#STM32SPI_CR1_SPE
    strh    r6,[r2,#STM32SPI_CR1_OFS]

    mov     r4,r0
    mov     r5,r1

copy_tx_bytes:
    cbz     r1,copy_rx_bytes
    ldrh    r6,[r2,#STM32SPI_SR_OFS]
    tst     r6,#STM32SPI_SR_TXE
    ittt    ne
    ldrbne  r7,[r0],#1
    strbne  r7,[r2,#STM32SPI_DR_OFS]
    subne   r1,r1,#1

copy_rx_bytes:
    cbz     r5,done
    ldrh    r6,[r2,#STM32SPI_SR_OFS]
    tst     r6,#STM32SPI_SR_RXNE
    ittt    ne
    ldrbne  r7,[r2,#STM32SPI_DR_OFS]
    strbne  r7,[r4],#1
    subne   r5,r5,#1
    b       copy_tx_bytes

done:
    /* Wait until not busy. */
    ldrh    r7,[r2,#STM32SPI_SR_OFS]
    tst     r7,#STM32SPI_SR_BSY
    bne     done

    /* Disable the SPI. */
    ldrh    r7,[r2,#STM32SPI_CR1_OFS]
    bic     r7,#STM32SPI_CR1_SPE
    strh    r7,[r2,#STM32SPI_CR1_OFS]
    bkpt    #0

