/*
 * SJA1000 driver adaptation for some Advantech PCI CAN cards.
 *
 * With portions from ems_pci.c and Adaptek drivers.
 *
 * Copyright (C) 2007 Wolfgang Grandegger <wg@grandegger.com>
 * Copyright (C) 2008 Markus Plessing <plessing@ems-wuensche.com>
 * Copyright (C) 2008 Sebastian Haas <haas@ems-wuensche.com>
 * Copyright (C) 2015 Marko Kohtala <marko.kohtala@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/can/dev.h>

#include "sja1000.h"

#define DRV_NAME  "advantech_can_pci"

MODULE_AUTHOR("Marko Kohtala <marko.kohtala@gmail.com>");
MODULE_DESCRIPTION("Socket-CAN driver for Advantech PCI CAN cards");
MODULE_SUPPORTED_DEVICE("Advantech MIOe-3680 CAN card");
MODULE_LICENSE("GPL v2");

enum board_id {
	pci_1680,
	mic_3680,
	uno_2052,
	eamb_ph07,
	c001,
	c002,
	c004,
	c101,
	c102,
	c104,
	c201,
	c202,
	c204,
	c301,
	c302,
	c304,
};

struct card_data {
	const char *name;
	unsigned ports;
	unsigned iolength;
	unsigned port_space;
};

static const struct card_data card_data[] = {
	[pci_1680] = { "PCI-1680", 2, .port_space = 1 },
	[mic_3680] = { "MIC-3680", 2, .port_space = 1 },
	[uno_2052] = { "UNO-2052(E)", 2, .port_space = 1 },
	[eamb_ph07] = { "EAMB-PH07", 1, .port_space = 1 },
	[c001] = { "C001 CAN card (1 PORT)", 1, 0x100 },
	[c002] = { "C002 CAN card (2 PORT)", 2, 0x100 },
	[c004] = { "C004 CAN card (4 PORT)", 4, 0x100 },
	[c101] = { "C101 CAN card (1 PORT,support CANopen)", 1, 0x100 },
	[c102] = { "C102 CAN card (2 PORT,support CANopen)", 2, 0x100 },
	[c104] = { "C104 CAN card (4 PORT,support CANopen)", 4, 0x100 },
	[c201] = { "C201 CAN card (1 PORT)", 1, 0x400 },
	[c202] = { "C202 CAN card (2 PORT)", 2, 0x400 },
	[c204] = { "C204 CAN card (4 PORT)", 4, 0x400 },
	[c301] = { "C301 CAN card (1 PORT,support CANopen)", 1, 0x400 },
	[c302] = { "C302, MIOe-3680 (2 PORT,support CANopen)", 2, 0x400 },
	[c304] = { "C304 CAN card (4 PORT,support CANopen)", 4, 0x400 },
};

struct adv_pci_card {
	const struct card_data *card_data;
	void __iomem *bar_addr[2];	/* Maps for BARS */
	struct net_device *net_dev[4];
};

/* SJA1000 internal clock is divided by 2 from external clock */
#define ADV_PCI_CAN_CLOCK (16000000 / 2)

/* The board configuration is following:
 * RX1 is connected to ground.
 * TX1 is not connected, but we do not leave it floating.
 * CLKO forwards clock to PCI bridge.
 */
#define ADV_PCI_OCR         (OCR_TX0_PUSHPULL | OCR_TX1_PUSHPULL)

/* In the CDR register, enable comparator by-pass for lower latency
 * since we have external tranceiver.
 * The clock divider value is set for direct oscillator output because the
 * PCI bridge is driven by the second CLKOUT output.
 */
#define ADV_PCI_CDR             (CDR_CBP | CDR_CLKOUT_MASK)

static const struct pci_device_id adv_pci_tbl[] = {
	/* PCI io 2 ports BAR 2 and 3 offset 0 length 128 */
	{ PCI_VDEVICE(ADVANTECH, 0x1680), pci_1680 },
	{ PCI_VDEVICE(ADVANTECH, 0x3680), mic_3680 },
	{ PCI_VDEVICE(ADVANTECH, 0x2052), uno_2052 },
	/* PCI io 1 port BAR 2 offset 0 length 128 */
	{ PCI_VDEVICE(ADVANTECH, 0x1681), eamb_ph07 },
	/* PCI iomem 1-4 ports BAR 0 spacing 0x100 bytes length 0x100 */
	{ PCI_VDEVICE(ADVANTECH, 0xc001), c001 },
	{ PCI_VDEVICE(ADVANTECH, 0xc002), c002 },
	{ PCI_VDEVICE(ADVANTECH, 0xc004), c004 },
	{ PCI_VDEVICE(ADVANTECH, 0xc101), c101 },
	{ PCI_VDEVICE(ADVANTECH, 0xc102), c102 },
	{ PCI_VDEVICE(ADVANTECH, 0xc104), c104 },
	/* PCI iomem 1-4 ports BAR 0 spacing 0x400 bytes length 0x400 */
	{ PCI_VDEVICE(ADVANTECH, 0xc201), c201 },
	{ PCI_VDEVICE(ADVANTECH, 0xc202), c202 },
	{ PCI_VDEVICE(ADVANTECH, 0xc204), c204 },
	{ PCI_VDEVICE(ADVANTECH, 0xc301), c301 },
	{ PCI_VDEVICE(ADVANTECH, 0xc302), c302 }, /* MIOe-3680 */
	{ PCI_VDEVICE(ADVANTECH, 0xc304), c304 },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, adv_pci_tbl);

static u8 adv_read_reg_io(const struct sja1000_priv *priv, int port)
{
	return inb(priv->reg_base + port);
}

static void adv_write_reg_io(const struct sja1000_priv *priv, int port, u8 val)
{
	outb(val, priv->reg_base + port);
}

static int adv_alloc_io(struct adv_pci_card *card)
{
	struct card_data *card_data = card->card_data;
	int err = 0;

	for (i = 0; i < card_data->ports; ++i) {
		err = pci_request_region(pdev, 2 + i, DRV_NAME);
		if (err)
			break;
	}
	return err;
}

static int adv_alloc_io(struct adv_pci_card *card)
{
	struct card_data *card_data = card->card_data;
	int err = 0;

	for (i = 0; i < card_data->ports; ++i) {
		err = pci_request_region(pdev, 2 + i, DRV_NAME);
		if (err)
			break;
	}
	return err;
}

static u8 adv_read_reg(const struct sja1000_priv *priv, int port)
{
	/* FIXME allocated using pci_iomap, should use via ioreadb */
	return readb(priv->reg_base + 4 * port);
}

static void adv_write_reg(const struct sja1000_priv *priv, int port, u8 val)
{
	writeb(val, priv->reg_base + 4 * port);
}

static void adv_remove(struct pci_dev *pdev)
{
	struct adv_pci_card *card = pci_get_drvdata(pdev);
	struct net_device *dev;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(card->net_dev); i++) {
		dev = card->net_dev[i];
		if (dev) {
			netdev_info(dev, "Removing\n");
			unregister_sja1000dev(dev);
			free_sja1000dev(dev);
		}
	}

	pci_disable_msi(pdev);
	for (i = 0; i < ARRAY_SIZE(card->bar_addr); i++) {
		if (card->bar_addr[i])
			pci_iounmap(pdev, card->bar_addr[i]);
	}
	pci_disable_device(pdev);
	pci_release_region(pdev, 0);

	kfree(card);
}

static int adv_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct sja1000_priv *priv;
	struct net_device *dev;
	struct adv_pci_card *card;
	struct card_data *card_data;
	int err, i;

	/* Enabling PCI device */
	if (pci_enable_device(pdev) < 0) {
		dev_err(&pdev->dev, "Enabling PCI device failed\n");
		return -ENODEV;
	}

	/* Allocating card structures to hold addresses, ... */
	card = kzalloc(sizeof(*card), GFP_KERNEL);
	if (!card) {
		pci_disable_device(pdev);
		return -ENOMEM;
	}

	pci_set_drvdata(pdev, card);
	card_data = card->card_data = &card_data[ent->driver_data];

	if (card_data->port_space) {
	}
	else {
		err = pci_request_region(pdev, 0, DRV_NAME);
		if (err)
			goto failure_cleanup;

		card->bar_addr[0] = pci_iomap(pdev, 0, 0);
		if (!card->bar_addr[0]) {
			err = -ENOMEM;
			goto failure_cleanup;
		}
	}

#ifdef TEST_MSI	/* For some reason MSI was not received on MIOe-3680 */
	err = pci_enable_msi(pdev);
	if (err)
		dev_err(&pdev->dev, "Error %d enabling MSI.\n", err);
#endif

	/* Number of ports is in the PCI device ID lowest nibble */
	for (i = 0; i < card_data->ports & PORTS_MASK; ++i) {
		dev = alloc_sja1000dev(0);
		if (!dev)
			return -ENOMEM;

		card->net_dev[i] = dev;
		priv = netdev_priv(dev);
		priv->priv = card;
		priv->irq_flags = IRQF_SHARED;
		dev->irq = pdev->irq;
		if (card_data->port_space) {
			priv->reg_base = card->bar_addr[i];
			priv->read_reg = adv_read_reg_io;
			priv->write_reg = adv_write_reg_io;
		} else {
			priv->reg_base = card->bar_addr[0] +
				card_data->iolength * i;
			priv->read_reg = adv_read_reg;
			priv->write_reg = adv_write_reg;
		}
		priv->can.clock.freq = ADV_PCI_CAN_CLOCK;
		priv->ocr = ADV_PCI_OCR;
		priv->cdr = ADV_PCI_CDR;

		SET_NETDEV_DEV(dev, &pdev->dev);
		dev->dev_id = i;

		/* Register SJA1000 device */
		err = register_sja1000dev(dev);
		if (err) {
			dev_err(&pdev->dev,
				"Registering device failed (err=%d)\n", err);
			free_sja1000dev(dev);
			return err;
		}

		netdev_info(dev, "Channel #%d at 0x%p, irq %d\n",
			    i + 1, priv->reg_base, dev->irq);
		}
	}

	return 0;

failure_cleanup:
	dev_err(&pdev->dev, "Error: %d. Cleaning Up.\n", err);

	adv_remove(pdev);

	return err;
}

static struct pci_driver adv_pci_driver = {
	.name = DRV_NAME,
	.id_table = adv_pci_tbl,
	.probe = adv_probe,
	.remove = adv_remove,
};

module_pci_driver(adv_pci_driver);
