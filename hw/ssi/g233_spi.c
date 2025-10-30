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

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/ssi/g233_spi.h"

static void g233_spi_update_cs(G233SPIState *s)
{
    uint32_t csctrl = s->regs[R_SPI_CSCTRL];
    
    for (int i = 0; i < G233_SPI_NUM_CS; i++) {
        bool enabled = csctrl & (1 << i);  /* CS_EN bits */
        bool active = csctrl & (1 << (i + 4));  /* CS_ACT bits */
        
        /* CS is active low, so invert the active signal */
        if (enabled) {
            qemu_set_irq(s->cs_lines[i], !active);
        } else {
            qemu_set_irq(s->cs_lines[i], 1);  /* Inactive (high) */
        }
    }
}

static void g233_spi_update_irq(G233SPIState *s)
{
    uint32_t sr = s->regs[R_SPI_SR];
    uint32_t cr2 = s->regs[R_SPI_CR2];
    bool irq = false;

    /* Check TXE interrupt */
    if ((sr & SPI_SR_TXE) && (cr2 & SPI_CR2_TXEIE)) {
        irq = true;
    }

    /* Check RXNE interrupt */
    if ((sr & SPI_SR_RXNE) && (cr2 & SPI_CR2_RXNEIE)) {
        irq = true;
    }

    /* Check error interrupts */
    if ((sr & (SPI_SR_OVERRUN | SPI_SR_UNDERRUN)) && (cr2 & SPI_CR2_ERRIE)) {
        irq = true;
    }

    qemu_set_irq(s->irq, irq ? 1 : 0);
}

static void g233_spi_reset(DeviceState *d)
{
    G233SPIState *s = G233_SPI(d);

    /* Reset all registers to default values */
    s->regs[R_SPI_CR1] = 0x00000000;
    s->regs[R_SPI_CR2] = 0x00000000;
    s->regs[R_SPI_SR] = SPI_SR_TXE;  /* TXE = 1 (buffer empty) */
    s->regs[R_SPI_DR] = 0x0000000C;  /* Reset value */
    s->regs[R_SPI_CSCTRL] = 0x00000000;

    s->tx_fifo_empty = true;
    s->rx_fifo_full = false;
    s->tx_data = 0;
    s->rx_data = 0;

    g233_spi_update_cs(s);
    g233_spi_update_irq(s);
}

static void g233_spi_do_transfer(G233SPIState *s, uint8_t tx_byte)
{
    uint8_t rx_byte;

    /* Mark as busy */
    s->regs[R_SPI_SR] |= SPI_SR_BSY;
    
    /* Only perform transfer if SPI is enabled and in master mode */
    if ((s->regs[R_SPI_CR1] & SPI_CR1_SPE) && 
        (s->regs[R_SPI_CR1] & SPI_CR1_MSTR)) {
        
        /* Perform SPI transfer */
        rx_byte = ssi_transfer(s->spi, tx_byte);
        
        /* Check for overrun: RXNE is still set when new data arrives */
        if (s->regs[R_SPI_SR] & SPI_SR_RXNE) {
            s->regs[R_SPI_SR] |= SPI_SR_OVERRUN;
        }
        
        /* Store received data */
        s->rx_data = rx_byte;
        s->regs[R_SPI_SR] |= SPI_SR_RXNE;  /* Receive buffer not empty */
    }

    /* Clear busy flag */
    s->regs[R_SPI_SR] &= ~SPI_SR_BSY;
    
    /* Update interrupt status */
    g233_spi_update_irq(s);
}

static uint64_t g233_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    G233SPIState *s = opaque;
    uint32_t r = 0;

    if (addr >= G233_SPI_REG_NUM * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad read at address 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        return 0;
    }

    addr >>= 2;
    
    switch (addr) {
    case R_SPI_DR:
        /* Reading DR returns received data and clears RXNE */
        if (s->regs[R_SPI_SR] & SPI_SR_RXNE) {
            r = s->rx_data;
            s->regs[R_SPI_SR] &= ~SPI_SR_RXNE;  /* Clear RXNE */
            g233_spi_update_irq(s);
        } else {
            r = s->regs[R_SPI_DR];
        }
        break;

    case R_SPI_SR:
        r = s->regs[R_SPI_SR];
        break;

    default:
        r = s->regs[addr];
        break;
    }

    return r;
}

static void g233_spi_write(void *opaque, hwaddr addr,
                           uint64_t val64, unsigned int size)
{
    G233SPIState *s = opaque;
    uint32_t value = val64;

    if (addr >= G233_SPI_REG_NUM * 4) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad write at addr=0x%"
                      HWADDR_PRIx " value=0x%x\n", __func__, addr, value);
        return;
    }

    addr >>= 2;
    
    switch (addr) {
    case R_SPI_CR1:
        s->regs[R_SPI_CR1] = value & 0x7F;  /* Only bits [6:0] are valid */
        break;

    case R_SPI_CR2:
        s->regs[R_SPI_CR2] = value & 0xFF;  /* Only bits [7:0] are valid */
        g233_spi_update_irq(s);
        break;

    case R_SPI_SR:
        /* Write 1 to clear error flags */
        if (value & SPI_SR_OVERRUN) {
            s->regs[R_SPI_SR] &= ~SPI_SR_OVERRUN;
        }
        if (value & SPI_SR_UNDERRUN) {
            s->regs[R_SPI_SR] &= ~SPI_SR_UNDERRUN;
        }
        g233_spi_update_irq(s);
        break;

    case R_SPI_DR:
        /* Writing to DR triggers a transfer if TXE is set */
        if (s->regs[R_SPI_SR] & SPI_SR_TXE) {
            s->tx_data = value & 0xFF;
            s->regs[R_SPI_SR] &= ~SPI_SR_TXE;  /* Clear TXE */
            
            /* Perform the transfer */
            g233_spi_do_transfer(s, s->tx_data);
            
            /* Mark TX buffer as empty again */
            s->regs[R_SPI_SR] |= SPI_SR_TXE;
            g233_spi_update_irq(s);
        }
        break;

    case R_SPI_CSCTRL:
        s->regs[R_SPI_CSCTRL] = value & 0xFF;  /* Only bits [7:0] are valid */
        g233_spi_update_cs(s);
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: invalid write to register 0x%"
                      HWADDR_PRIx " with 0x%x\n", __func__, addr << 2, value);
        break;
    }
}

static const MemoryRegionOps g233_spi_ops = {
    .read = g233_spi_read,
    .write = g233_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void g233_spi_realize(DeviceState *dev, Error **errp)
{
    G233SPIState *s = G233_SPI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    /* Create SPI bus */
    s->spi = ssi_create_bus(dev, "spi");

    /* Initialize IRQ */
    sysbus_init_irq(sbd, &s->irq);

    /* Initialize CS lines */
    for (int i = 0; i < G233_SPI_NUM_CS; i++) {
        sysbus_init_irq(sbd, &s->cs_lines[i]);
    }

    /* Initialize MMIO region */
    memory_region_init_io(&s->mmio, OBJECT(s), &g233_spi_ops, s,
                          TYPE_G233_SPI, 0x1000);
    sysbus_init_mmio(sbd, &s->mmio);
}

static void g233_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = g233_spi_realize;
    device_class_set_legacy_reset(dc, g233_spi_reset);
}

static const TypeInfo g233_spi_info = {
    .name           = TYPE_G233_SPI,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(G233SPIState),
    .class_init     = g233_spi_class_init,
};

static void g233_spi_register_types(void)
{
    type_register_static(&g233_spi_info);
}

type_init(g233_spi_register_types)
