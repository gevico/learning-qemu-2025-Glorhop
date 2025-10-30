/*
 * QEMU model of the G233 SPI Controller
 *
 * Copyright (c) 2025 Learning QEMU
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_G233_SPI_H
#define HW_G233_SPI_H

#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"

#define TYPE_G233_SPI "g233-spi"
OBJECT_DECLARE_SIMPLE_TYPE(G233SPIState, G233_SPI)

/* Register offsets */
#define R_SPI_CR1       (0x00 / 4)
#define R_SPI_CR2       (0x04 / 4)
#define R_SPI_SR        (0x08 / 4)
#define R_SPI_DR        (0x0C / 4)
#define R_SPI_CSCTRL    (0x10 / 4)

#define G233_SPI_REG_NUM 5

/* SPI_CR1 bits */
#define SPI_CR1_SPE     (1 << 6)  /* SPI Enable */
#define SPI_CR1_MSTR    (1 << 2)  /* Master mode */

/* SPI_CR2 bits */
#define SPI_CR2_TXEIE   (1 << 7)  /* TXE interrupt enable */
#define SPI_CR2_RXNEIE  (1 << 6)  /* RXNE interrupt enable */
#define SPI_CR2_ERRIE   (1 << 5)  /* Error interrupt enable */
#define SPI_CR2_SSOE    (1 << 4)  /* SS output enable */

/* SPI_SR bits */
#define SPI_SR_BSY      (1 << 7)  /* Busy flag */
#define SPI_SR_OVERRUN  (1 << 3)  /* Overrun flag */
#define SPI_SR_UNDERRUN (1 << 2)  /* Underrun flag */
#define SPI_SR_TXE      (1 << 1)  /* Transmit buffer empty */
#define SPI_SR_RXNE     (1 << 0)  /* Receive buffer not empty */

/* SPI_CSCTRL bits */
#define SPI_CSCTRL_CS3_ACT  (1 << 7)  /* CS3 active */
#define SPI_CSCTRL_CS2_ACT  (1 << 6)  /* CS2 active */
#define SPI_CSCTRL_CS1_ACT  (1 << 5)  /* CS1 active */
#define SPI_CSCTRL_CS0_ACT  (1 << 4)  /* CS0 active */
#define SPI_CSCTRL_CS3_EN   (1 << 3)  /* CS3 enable */
#define SPI_CSCTRL_CS2_EN   (1 << 2)  /* CS2 enable */
#define SPI_CSCTRL_CS1_EN   (1 << 1)  /* CS1 enable */
#define SPI_CSCTRL_CS0_EN   (1 << 0)  /* CS0 enable */

#define G233_SPI_NUM_CS 4

struct G233SPIState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    SSIBus *spi;
    qemu_irq cs_lines[G233_SPI_NUM_CS];

    uint32_t regs[G233_SPI_REG_NUM];
    
    uint8_t tx_data;
    uint8_t rx_data;
    bool tx_fifo_empty;
    bool rx_fifo_full;
};

#endif /* HW_G233_SPI_H */
