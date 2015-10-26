/*
 * SJA1000 driver adaptation for some Advantech PCI CAN cards.
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

/* With portions from ems_pci.c */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/can/dev.h>

#include "sja1000.h"

#define DRV_NAME  "advantech_can_pci"

MODULE_AUTHOR("Marko Kohtala <marko.kohtala@gmail.com>");
MODULE_DESCRIPTION("Socket-CAN driver for Advantech MIOe-3680 CAN cards");
MODULE_SUPPORTED_DEVICE("Advantech MIOe-3680 CAN card");
MODULE_LICENSE("GPL v2");

struct adv_pci_card {
	void __iomem *can_addr;
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
	/* PCI BAR 0 iomem 0x400 bytes per port 32-bit word for each reg */
	{ PCI_VDEVICE(ADVANTECH, 0xc201) },
	{ PCI_VDEVICE(ADVANTECH, 0xc202) },
	{ PCI_VDEVICE(ADVANTECH, 0xc204) },
	{ PCI_VDEVICE(ADVANTECH, 0xc301) },
	{ PCI_VDEVICE(ADVANTECH, 0xc302) }, /* MIOe-3680 */
	{ PCI_VDEVICE(ADVANTECH, 0xc304) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, adv_pci_tbl);

static u8 adv_read_reg(const struct sja1000_priv *priv, int port)
{
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
	pci_iounmap(pdev, card->can_addr);
	pci_disable_device(pdev);
	pci_release_region(pdev, 0);

	kfree(card);
}

static int adv_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct sja1000_priv *priv;
	struct net_device *dev;
	struct adv_pci_card *card;
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

	err = pci_request_region(pdev, 0, DRV_NAME);
	if (err)
		goto failure_cleanup;

	card->can_addr = pci_iomap(pdev, 0, 0);
	if (!card->can_addr) {
		err = -ENOMEM;
		goto failure_cleanup;
	}

#ifdef TEST_MSI	/* For some reason MSI was not received on MIOe-3680 */
	err = pci_enable_msi(pdev);
	if (err)
		dev_err(&pdev->dev, "Error %d enabling MSI.\n", err);
#endif

	/* Number of ports is in the PCI device ID lowest nibble */
	for (i = 0; i < (pdev->device & 0xf); ++i) {
		dev = alloc_sja1000dev(0);
		if (!dev) {
			err = -ENOMEM;
			goto failure_cleanup;
		}

		card->net_dev[i] = dev;
		priv = netdev_priv(dev);
		priv->priv = card;
		priv->irq_flags = IRQF_SHARED;
		dev->irq = pdev->irq;
		priv->reg_base = card->can_addr + 0x400 * i;
		priv->read_reg = adv_read_reg;
		priv->write_reg = adv_write_reg;
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
			goto failure_cleanup;
		}

		netdev_info(dev, "Channel #%d at 0x%p, irq %d\n",
			    i + 1, priv->reg_base, dev->irq);
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
