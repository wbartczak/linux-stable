// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co. Ltd.
 * Rockchip CAN driver
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>

#define DRV_NAME		"rk3568_can"

#if 0

#define CAN_MODE		0x00
#define RESET_MODE		0
#define WORK_MODE		BIT(0)
#define SELF_TEST_EN		BIT(2)
#define MODE_AUTO_RETX		BIT(10)

#define CAN_CMD			0x04
#define TX_REQ			BIT(0)

#define CAN_STATE		0x08
#define RX_BUF_FULL		BIT(0)
#define TX_BUF_FULL		BIT(1)
#define RX_PERIOD		BIT(2)
#define TX_PERIOD		BIT(3)
#define ERR_WARN		BIT(4)
#define BUS_OFF			BIT(5)

#define CAN_INT			0x0C

#define CAN_INT_MASK		0x10
#define RX_FINISH		BIT(0)
#define TX_FINISH		BIT(1)
#define ERR_WARN_INT		BIT(2)
#define RX_BUF_OV		BIT(3)
#define PASSIVE_ERR		BIT(4)
#define TX_LOSTARB		BIT(5)
#define BUS_ERR_INT		BIT(6)

/* Bit Timing Register */
#define CAN_BTT			0x18
#define MODE_3_SAMPLES		BIT(16)
#define BT_SJW_SHIFT		14
#define BT_SJW_MASK		GENMASK(15, 14)
#define BT_BRP_SHIFT		8
#define BT_BRP_MASK		GENMASK(13, 8)
#define BT_TSEG2_SHIFT		4
#define BT_TSEG2_MASK		GENMASK(6, 4)
#define BT_TSEG1_SHIFT		0
#define BT_TSEG1_MASK		GENMASK(3, 0)

#define CAN_LOSTARB_CODE	0x28

#define CAN_ERR_CODE		0x2c
#define ERR_TYPE_MASK		GENMASK(24, 22)
#define ERR_TYPE_SHIFT		22
#define BIT_ERR			0
#define STUFF_ERR		1
#define FORM_ERR		2
#define ACK_ERR			3
#define CRC_ERR			4
#define ERR_DIR_RX		BIT(21)
#define ERR_LOC_MASK		GENMASK(13, 0)

#define CAN_RX_ERR_CNT		0x34

#define CAN_TX_ERR_CNT		0x38

#define CAN_ID			0x3c

#define CAN_ID_MASK		0x40

#define CAN_TX_FRM_INFO		0x50
#define CAN_EFF			BIT(7)
#define CAN_RTR			BIT(6)
#define CAN_DLC_MASK		GENMASK(3, 0)
#define CAN_DLC(x)		((x) & GENMASK(3, 0))

#define CAN_TX_ID		0x54
#define CAN_TX_ID_MASK		0x1fffffff

#define CAN_TX_DATA1		0x58

#define CAN_TX_DATA2		0x5c

#define CAN_RX_FRM_INFO		0x60

#define CAN_RX_ID		0x64

#define CAN_RX_DATA1		0x68

#define CAN_RX_DATA2		0x6c

#define CAN_RX_FILTER_MASK	0x1fffffff

#define CAN_VERSION		0x70

struct rockchip_can {
	struct can_priv can;
	void __iomem *base;
	struct device *dev;
	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control *reset;
};

static const struct can_bittiming_const rockchip_can_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 128,
	.brp_inc = 2,
};

static void rockchip_can_write_cmdreg(struct rockchip_can *rcan, u8 val)
{
	writel(val, rcan->base + CAN_CMD);
}

static int set_reset_mode(struct net_device *ndev)
{
	struct rockchip_can *rcan = netdev_priv(ndev);

	reset_control_assert(rcan->reset);
	udelay(2);
	reset_control_deassert(rcan->reset);

	writel(0, rcan->base + CAN_MODE);

	return 0;
}

static int set_normal_mode(struct net_device *ndev)
{
	struct rockchip_can *rcan = netdev_priv(ndev);
	u32 val;

	val = readl(rcan->base + CAN_MODE);
	val |= WORK_MODE | MODE_AUTO_RETX;
	writel(val, rcan->base + CAN_MODE);

	return 0;
}

/* bittiming is called in reset_mode only */
static int rockchip_can_set_bittiming(struct net_device *ndev)
{
	struct rockchip_can *rcan = netdev_priv(ndev);
	struct can_bittiming *bt = &rcan->can.bittiming;
	u32 cfg;

	cfg = ((bt->sjw - 1) << BT_SJW_SHIFT) |
	      (((bt->brp >> 1) - 1) << BT_BRP_SHIFT) |
	      ((bt->phase_seg2 - 1) << BT_TSEG2_SHIFT) |
	      ((bt->prop_seg + bt->phase_seg1 - 1));
	if (rcan->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		cfg |= MODE_3_SAMPLES;

	writel(cfg, rcan->base + CAN_BTT);

	netdev_dbg(ndev, "setting BITTIMING=0x%08x  brp: %d bitrate:%d\n",
		   cfg, bt->brp, bt->bitrate);

	return 0;
}

static int rockchip_can_get_berr_counter(const struct net_device *ndev,
					 struct can_berr_counter *bec)
{
	struct rockchip_can *rcan = netdev_priv(ndev);
	int err;

	err = pm_runtime_get_sync(rcan->dev);
	if (err < 0) {
		netdev_err(ndev, "%s: pm_runtime_get failed(%d)\n",
			   __func__, err);
		return err;
	}

	bec->rxerr = readl(rcan->base + CAN_RX_ERR_CNT);
	bec->txerr = readl(rcan->base + CAN_TX_ERR_CNT);

	pm_runtime_put(rcan->dev);

	netdev_dbg(ndev, "%s\n", __func__);
	return 0;
}

static int rockchip_can_start(struct net_device *ndev)
{
	struct rockchip_can *rcan = netdev_priv(ndev);

	/* we need to enter the reset mode */
	set_reset_mode(ndev);

	writel(0, rcan->base + CAN_INT_MASK);

	/* RECEIVING FILTER, accept all */
	writel(0, rcan->base + CAN_ID);
	writel(CAN_RX_FILTER_MASK, rcan->base + CAN_ID_MASK);

	rockchip_can_set_bittiming(ndev);

	set_normal_mode(ndev);

	rcan->can.state = CAN_STATE_ERROR_ACTIVE;

	netdev_dbg(ndev, "%s\n", __func__);
	return 0;
}static int rockchip_can_stop(struct net_device *ndev)
{
	struct rockchip_can *rcan = netdev_priv(ndev);
	u32 val;

	rcan->can.state = CAN_STATE_STOPPED;
	/* we need to enter reset mode */
	set_reset_mode(ndev);

	/* disable all interrupts */
	val = RX_FINISH | TX_FINISH | ERR_WARN_INT |
	      RX_BUF_OV | PASSIVE_ERR | TX_LOSTARB |
	      BUS_ERR_INT;

	writel(val, rcan->base + CAN_INT_MASK);
	netdev_dbg(ndev, "%s\n", __func__);
	return 0;
}

static int rockchip_can_set_mode(struct net_device *ndev, enum can_mode mode)
{
	int err;

	netdev_dbg(ndev, "can set mode: 0x%x\n", mode);

	switch (mode) {
	case CAN_MODE_START:
		err = rockchip_can_start(ndev);
		if (err) {
			netdev_err(ndev, "starting CAN controller failed!\n");
			return err;
		}
		if (netif_queue_stopped(ndev))
			netif_wake_queue(ndev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/* transmit a CAN message
 * message layout in the sk_buff should be like this:
 * xx xx xx xx         ff         ll 00 11 22 33 44 55 66 77
 * [ can_id ] [flags] [len] [can data (up to 8 bytes]
 */
static int rockchip_can_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct rockchip_can *rcan = netdev_priv(ndev);
	struct can_frame *cf = (struct can_frame *)skb->data;
	canid_t id;
	u8 dlc;
	u32 fi;
	u32 data1 = 0, data2 = 0;

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(ndev);

	id = cf->can_id;
	dlc = cf->can_dlc;
	fi = dlc;

	if (id & CAN_RTR_FLAG) {
		fi |= CAN_RTR;
		fi &= ~CAN_DLC_MASK;
	}

	if (id & CAN_EFF_FLAG)
		fi |= CAN_EFF;

	rockchip_can_write_cmdreg(rcan, 0);

	writel(id & CAN_TX_ID_MASK, rcan->base + CAN_TX_ID);
	if (!(id & CAN_RTR_FLAG)) {
		data1 = le32_to_cpup((__le32 *)&cf->data[0]);
		data2 = le32_to_cpup((__le32 *)&cf->data[4]);
		writel(data1, rcan->base + CAN_TX_DATA1);
		writel(data2, rcan->base + CAN_TX_DATA2);
	}

	writel(fi, rcan->base + CAN_TX_FRM_INFO);
	can_put_echo_skb(skb, ndev, 0);

	rockchip_can_write_cmdreg(rcan, TX_REQ);
	netdev_dbg(ndev, "TX: can_id:0x%08x dlc: %d mode: 0x%08x data: 0x%08x 0x%08x\n",
		   cf->can_id, cf->can_dlc, rcan->can.ctrlmode, data1, data2);

	return NETDEV_TX_OK;
}

static void rockchip_can_rx(struct net_device *ndev)
{
	struct rockchip_can *rcan = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	canid_t id;
	u8 fi;
	u32 data1 = 0, data2 = 0;

	/* create zero'ed CAN frame buffer */
	skb = alloc_can_skb(ndev, &cf);
	if (!skb)
		return;

	fi = readl(rcan->base + CAN_RX_FRM_INFO);
	cf->can_dlc = get_can_dlc(fi & CAN_DLC_MASK);
	id = readl(rcan->base + CAN_RX_ID);
	if (fi & CAN_EFF)
		id |= CAN_EFF_FLAG;

	/* remote frame ? */
	if (fi & CAN_RTR) {
		id |= CAN_RTR_FLAG;
	} else {
		data1 = readl(rcan->base + CAN_RX_DATA1);
		data2 = readl(rcan->base + CAN_RX_DATA2);
	}

	cf->can_id = id;
	*(__le32 *)(cf->data + 0) = cpu_to_le32(data1);
	*(__le32 *)(cf->data + 4) = cpu_to_le32(data2);

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);

	netdev_dbg(ndev, "%s can_id:0x%08x fi: 0x%08x dlc: %d data: 0x%08x 0x%08x\n",
		   __func__, cf->can_id, fi, cf->can_dlc,
		   data1, data2);
}

static void rockchip_can_clean_rx_info(struct rockchip_can *rcan)
{
	readl(rcan->base + CAN_RX_FRM_INFO);
	readl(rcan->base + CAN_RX_ID);
	readl(rcan->base + CAN_RX_DATA1);
	readl(rcan->base + CAN_RX_DATA2);
}

static int rockchip_can_err(struct net_device *ndev, u8 isr)
{
	struct rockchip_can *rcan = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	enum can_state state = rcan->can.state;
	enum can_state rx_state, tx_state;
	struct can_frame *cf;
	struct sk_buff *skb;
	unsigned int rxerr, txerr;
	u32 ecc, alc;
	u32 sta_reg;

	skb = alloc_can_err_skb(ndev, &cf);

	rxerr = readl(rcan->base + CAN_RX_ERR_CNT);
	txerr = readl(rcan->base + CAN_TX_ERR_CNT);
	sta_reg = readl(rcan->base + CAN_STATE);

	if (skb) {
		cf->data[6] = txerr;
		cf->data[7] = rxerr;
	}

	if (isr & RX_BUF_OV) {
		/* data overrun interrupt */
		netdev_dbg(ndev, "data overrun interrupt\n");
		if (likely(skb)) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		}
		stats->rx_over_errors++;
		stats->rx_errors++;

		/* reset the CAN IP by entering reset mode
		 * ignoring timeout error
		 */
		set_reset_mode(ndev);
		set_normal_mode(ndev);
	}

	if (isr & ERR_WARN_INT) {
		/* error warning interrupt */
		netdev_dbg(ndev, "error warning interrupt\n");

		if (sta_reg & BUS_OFF)
			state = CAN_STATE_BUS_OFF;
		else if (sta_reg & ERR_WARN)
			state = CAN_STATE_ERROR_WARNING;
		else
			state = CAN_STATE_ERROR_ACTIVE;
	}

	if (isr & BUS_ERR_INT) {
		/* bus error interrupt */
		netdev_dbg(ndev, "bus error interrupt\n");
		rcan->can.can_stats.bus_error++;
		stats->rx_errors++;

		if (likely(skb)) {
			ecc = readl(rcan->base + CAN_ERR_CODE);

			cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

			switch ((ecc & ERR_TYPE_MASK) >> ERR_TYPE_SHIFT) {
			case BIT_ERR:
				cf->data[2] |= CAN_ERR_PROT_BIT;
				break;
			case FORM_ERR:
				cf->data[2] |= CAN_ERR_PROT_FORM;
				break;
			case STUFF_ERR:
				cf->data[2] |= CAN_ERR_PROT_STUFF;
				break;
			default:
				cf->data[3] = ecc & ERR_LOC_MASK;
				break;
			}
			/* error occurred during transmission? */
			if ((ecc & ERR_DIR_RX) == 0)
				cf->data[2] |= CAN_ERR_PROT_TX;
		}
	}

	if (isr & PASSIVE_ERR) {
		/* error passive interrupt */
		netdev_dbg(ndev, "error passive interrupt\n");
		if (state == CAN_STATE_ERROR_PASSIVE)
			state = CAN_STATE_ERROR_WARNING;
		else
			state = CAN_STATE_ERROR_PASSIVE;
	}
	if (isr & TX_LOSTARB) {
		/* arbitration lost interrupt */
		netdev_dbg(ndev, "arbitration lost interrupt\n");
		alc = readl(rcan->base + CAN_LOSTARB_CODE);
		rcan->can.can_stats.arbitration_lost++;
		stats->tx_errors++;
		if (likely(skb)) {
			cf->can_id |= CAN_ERR_LOSTARB;
			cf->data[0] = alc;
		}
	}

	if (state != rcan->can.state) {
		tx_state = txerr >= rxerr ? state : 0;
		rx_state = txerr <= rxerr ? state : 0;

		if (likely(skb))
			can_change_state(ndev, cf, tx_state, rx_state);
		else
			rcan->can.state = state;
		if (state == CAN_STATE_BUS_OFF)
			can_bus_off(ndev);
	}

	if (likely(skb)) {
		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;
		netif_rx(skb);
	} else {
		return -ENOMEM;
	}

	return 0;
}

static irqreturn_t rockchip_can_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct rockchip_can *rcan = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	u8 err_int = ERR_WARN_INT | RX_BUF_OV | PASSIVE_ERR |
		     TX_LOSTARB | BUS_ERR_INT;
	u8 isr;

	isr = readl(rcan->base + CAN_INT);
	if (isr & TX_FINISH) {
		/* transmission complete interrupt */
		stats->tx_bytes += readl(rcan->base + CAN_TX_FRM_INFO) &
				   CAN_DLC_MASK;
		stats->tx_packets++;
		rockchip_can_write_cmdreg(rcan, 0);
		can_get_echo_skb(ndev, 0);
		netif_wake_queue(ndev);
	}

	if (isr & RX_FINISH)
		rockchip_can_rx(ndev);

	if (isr & err_int) {
		rockchip_can_clean_rx_info(rcan);
		if (rockchip_can_err(ndev, isr))
			netdev_err(ndev, "can't allocate buffer - clearing pending interrupts\n");
	}

	writel(isr, rcan->base + CAN_INT);
	rockchip_can_clean_rx_info(rcan);
	netdev_dbg(ndev, "isr: 0x%x\n", isr);
	return	IRQ_HANDLED;
}

static int rockchip_can_open(struct net_device *ndev)
{
	struct rockchip_can *rcan = netdev_priv(ndev);
	int err;

	/* common open */
	err = open_candev(ndev);
	if (err)
		return err;

	err = pm_runtime_get_sync(rcan->dev);
	if (err < 0) {
		netdev_err(ndev, "%s: pm_runtime_get failed(%d)\n",
			   __func__, err);
		goto exit;
	}

	err = rockchip_can_start(ndev);
	if (err) {
		netdev_err(ndev, "could not start CAN peripheral\n");
		goto exit_can_start;
	}

	netif_start_queue(ndev);

	netdev_dbg(ndev, "%s\n", __func__);
	return 0;

exit_can_start:
	pm_runtime_put(rcan->dev);
exit:
	close_candev(ndev);
	return err;
}

static int rockchip_can_close(struct net_device *ndev)
{
	struct rockchip_can *rcan = netdev_priv(ndev);

	netif_stop_queue(ndev);
	rockchip_can_stop(ndev);
	close_candev(ndev);
	pm_runtime_put(rcan->dev);

	netdev_dbg(ndev, "%s\n", __func__);
	return 0;
}

static const struct net_device_ops rockchip_can_netdev_ops = {
	.ndo_open = rockchip_can_open,
	.ndo_stop = rockchip_can_close,
	.ndo_start_xmit = rockchip_can_start_xmit,
};

/**
 * rockchip_can_suspend - Suspend method for the driver
 * @dev:	Address of the device structure
 *
 * Put the driver into low power mode.
 * Return: 0 on success and failure value on error
 */
static int __maybe_unused rockchip_can_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);
		rockchip_can_stop(ndev);
	}

	return pm_runtime_force_suspend(dev);
}

/**
 * rockchip_can_resume - Resume from suspend
 * @dev:	Address of the device structure
 *
 * Resume operation after suspend.
 * Return: 0 on success and failure value on error
 */
static int __maybe_unused rockchip_can_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret) {
		dev_err(dev, "pm_runtime_force_resume failed on resume\n");
		return ret;
	}

	if (netif_running(ndev)) {
		ret = rockchip_can_start(ndev);
		if (ret) {
			dev_err(dev, "rockchip_can_chip_start failed on resume\n");
			return ret;
		}

		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}

	return 0;
}

/**
 * rockchip_can_runtime_suspend - Runtime suspend method for the driver
 * @dev:	Address of the device structure
 *
 * Put the driver into low power mode.
 * Return: 0 always
 */
static int __maybe_unused rockchip_can_runtime_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rockchip_can *rcan = netdev_priv(ndev);

	clk_bulk_disable_unprepare(rcan->num_clks, rcan->clks);

	return 0;
}

/**
 * rockchip_can_runtime_resume - Runtime resume from suspend
 * @dev:	Address of the device structure
 *
 * Resume operation after suspend.
 * Return: 0 on success and failure value on error
 */
static int __maybe_unused rockchip_can_runtime_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rockchip_can *rcan = netdev_priv(ndev);
	int ret;

	ret = clk_bulk_prepare_enable(rcan->num_clks, rcan->clks);
	if (ret) {
		dev_err(dev, "Cannot enable clock.\n");
		return ret;
	}

	return 0;
}

static const struct dev_pm_ops rockchip_can_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rockchip_can_suspend, rockchip_can_resume)
	SET_RUNTIME_PM_OPS(rockchip_can_runtime_suspend,
			   rockchip_can_runtime_resume, NULL)
};


static int rockchip_can_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct rockchip_can *rcan;
	struct resource *res;
	void __iomem *addr;
	int err, irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "could not get a valid irq\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(addr))
		return -EBUSY;

	ndev = alloc_candev(sizeof(struct rockchip_can), 1);
	if (!ndev) {
		dev_err(&pdev->dev, "could not allocate memory for CAN device\n");
		return -ENOMEM;
	}

	ndev->netdev_ops = &rockchip_can_netdev_ops;
	ndev->irq = irq;
	ndev->flags |= IFF_ECHO;

	rcan = netdev_priv(ndev);

	/* register interrupt handler */
	err = devm_request_irq(&pdev->dev, ndev->irq, rockchip_can_interrupt,
			       0, ndev->name, ndev);
	if (err) {
		dev_err(&pdev->dev, "request_irq err: %d\n", err);
		return err;
	}

	rcan->reset = devm_reset_control_array_get(&pdev->dev, false, false);
	if (IS_ERR(rcan->reset)) {
		if (PTR_ERR(rcan->reset) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to get rcan reset lines\n");
		return PTR_ERR(rcan->reset);
	}

	rcan->num_clks = devm_clk_bulk_get_all(&pdev->dev, &rcan->clks);
	if (rcan->num_clks < 1) {
		dev_err(&pdev->dev, "bus clock not found\n");
		return -ENODEV;
	}

	rcan->dev = &pdev->dev;
	rcan->can.clock.freq = clk_get_rate(rcan->clks[0].clk);
	rcan->can.bittiming_const = &rockchip_can_bittiming_const;
	rcan->can.do_set_mode = rockchip_can_set_mode;
	rcan->can.do_get_berr_counter = rockchip_can_get_berr_counter;
	rcan->can.ctrlmode_supported = CAN_CTRLMODE_BERR_REPORTING |
				       CAN_CTRLMODE_LISTENONLY |
				       CAN_CTRLMODE_LOOPBACK |
				       CAN_CTRLMODE_3_SAMPLES;
	rcan->base = addr;
	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	pm_runtime_enable(&pdev->dev);
	err = pm_runtime_get_sync(&pdev->dev);
	if (err < 0) {
		dev_err(&pdev->dev, "%s: pm_runtime_get failed(%d)\n",
			__func__, err);
		goto err_pmdisable;
	}

	err = register_candev(ndev);
	if (err) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
			DRV_NAME, err);
		goto err_disableclks;
	}

	pm_runtime_put(&pdev->dev);

	return 0;

err_disableclks:
	pm_runtime_put(&pdev->dev);
err_pmdisable:
	pm_runtime_disable(&pdev->dev);
	free_candev(ndev);

	return err;
}

static int rockchip_can_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	unregister_netdev(ndev);
	pm_runtime_disable(&pdev->dev);
	free_candev(ndev);

	return 0;
}
#endif // 0


/* RK3568 CAN registers */
#define CAN_MODE                    	0x0000 /* CAN controller working mode. configure register */
#define CAN_CMD                     	0x0004 /* CAN command register */
#define CAN_STATE                   	0x0008 /* CAN state register */
#define CAN_INT                     	0x000c /* Interrupt state register */
#define CAN_INT_MASK                	0x0010 /* Interrupt enable registers */
#define CAN_DMA_CTRL                	0x0014 /* DMA mode control */
#define CAN_BITTIMING               	0x0018 /* Bit timing configure register */
#define CAN_ARBITFAIL               	0x0028 /* Arbit fail code register */
#define CAN_ERROR_CODE              	0x002c /* Error code register */
#define CAN_RXERRORCNT              	0x0034 /* Receive error counter */
#define CAN_TXERRORCNT              	0x0038 /* Transmit error counter */
#define CAN_IDCODE                  	0x003c /* CAN controller's identifier */
#define CAN_IDMASK                  	0x0040 /* Identification code bit mask register */
#define CAN_TXFRAMEINFO             	0x0050 /* TX frame information configuration register */
#define CAN_TXID                    	0x0054 /* CAN controller transmit ID */
#define CAN_TXDATA0                 	0x0058 /* CAN controller transmit DATA1 */
#define CAN_TXDATA1                 	0x005c /* CAN controller transmit DATA1 */
#define CAN_RXFRAMEINFO             	0x0060 /* RX frame information register. This register needs to be read (clear) before receiving the next frame. */
#define CAN_RXID                    	0x0064 /* CAN controller receive ID. This register needs to be read (clear) before receiving the next frame. */
#define CAN_RXDATA0                 	0x0068 /* Receive Data Reg0. This register needs to be read (clear) before receiving the next frame. */
#define CAN_RXDATA1                 	0x006c /* Receive Data Reg1. This register needs to be read (clear) before receiving the next frame. */
#define CAN_RTL_VERSION             	0x0070 /* CAN RTL version */
#define CAN_FD_NOMINAL_BITTIMING    	0x0100 /* CANFD nominal bit timing configure */
#define CAN_FD_DATA_BITTIMING       	0x0104 /* CANFD data bit timing configure */
#define CAN_TRANSMIT_DELAY_COMPENSATION 0x0108 /* Transmitter delay compensation configure */
#define CAN_TIMESTAMP_CTRL          	0x010c /* Timestamp counter configure */
#define CAN_TIMESTAMP               	0x0110 /* Timestamp counter */
#define CAN_TXEVENT_FIFO_CTRL       	0x0114 /* TX event FIFO configure */
#define CAN_RX_FIFO_CTRL            	0x0118 /* RX FIFO configure */
#define CAN_AFR_CTRL                	0x011c /* Acceptance filter register control */
#define CAN_IDCODE0                 	0x0120 /* CAN controller's identifier */
#define CAN_IDMASK0                 	0x0124 /* Identification code bit mask register */
#define CAN_IDCODE1                 	0x0128 /* CAN controller's identifier */
#define CAN_IDMASK1                 	0x012c /* Identification code bit mask register */
#define CAN_IDCODE2                 	0x0130 /* CAN controller's identifier */
#define CAN_IDMASK2                 	0x0134 /* Identification code bit mask register */
#define CAN_IDCODE3                 	0x0138 /* CAN controller's identifier */
#define CAN_IDMASK3                 	0x013c /* Identification code bit mask register */
#define CAN_IDCODE4                 	0x0140 /* CAN controller's identifier */
#define CAN_IDMASK4                 	0x0144 /* Identification code bit mask register */
#define CAN_FD_TXFRAMEINFO          	0x0200 /* CANFD TX frame information configuration register */
#define CAN_FD_TXID                 	0x0204 /* CAN controller transmit ID */
#define CAN_FD_TXDATA0              	0x0208 /* CANFD controller transmit DATA0 */
#define CAN_FD_TXDATA1              	0x020c /* CAN controller transmit DATA1 */
#define CAN_FD_TXDATA2              	0x0210 /* CAN controller transmit DATA2 */
#define CAN_FD_TXDATA3              	0x0214 /* CAN controller transmit DATA3 */
#define CAN_FD_TXDATA4              	0x0218 /* CAN controller transmit DATA4 */
#define CAN_FD_TXDATA5              	0x021c /* CAN controller transmit DATA5 */
#define CAN_FD_TXDATA6              	0x0220 /* CAN controller transmit DATA6 */
#define CAN_FD_TXDATA7              	0x0224 /* CAN controller transmit DATA7 */
#define CAN_FD_TXDATA8              	0x0228 /* CAN controller transmit DATA8 */
#define CAN_FD_TXDATA9              	0x022c /* CAN controller transmit DATA9 */
#define CAN_FD_TXDATA10             	0x0230 /* CAN controller transmit DATA10 */
#define CAN_FD_TXDATA11             	0x0234 /* CAN controller transmit DATA11 */
#define CAN_FD_TXDATA12             	0x0238 /* CAN controller transmit DATA12 */
#define CAN_FD_TXDATA13             	0x023c /* CAN controller transmit DATA13 */
#define CAN_FD_TXDATA14             	0x0240 /* CAN controller transmit DATA14 */
#define CAN_FD_TXDATA15             	0x0244 /* CAN controller transmit DATA15 */
#define CAN_FD_RXFRAMEINFO          	0x0300 /* CANFD RX frame information configuration register */
#define CAN_FD_RXID                 	0x0304 /* CAN controller receive ID */
#define CAN_FD_RXTIMESTAMP          	0x0308 /* CAN controller receive timestamp */
#define CAN_FD_RXDATA0              	0x030c /* Receive Data Reg0. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA1              	0x0310 /* Receive Data Reg1. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA2              	0x0314 /* Receive Data Reg2. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA3              	0x0318 /* Receive Data Reg3. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA4              	0x031c /* Receive Data Reg4. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA5              	0x0320 /* Receive Data Reg5. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA6              	0x0324 /* Receive Data Reg6. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA7              	0x0328 /* Receive Data Reg7. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA8              	0x032c /* Receive Data Reg8. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA9              	0x0330 /* Receive Data Reg9. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA10             	0x0334 /* Receive Data Reg10. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA11             	0x0338 /* Receive Data Reg11. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA12             	0x033c /* Receive Data Reg12. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA13             	0x0340 /* Receive Data Reg13. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA14             	0x0344 /* Receive Data Reg14. This register refreshes after receiving the next frame. */
#define CAN_FD_RXDATA15             	0x0348 /* Receive Data Reg15. This register refreshes after receiving the next frame. */
#define CAN_RX_FIFO_RDATA           	0x0400 /* This register gives the value read from RX FIFO */
#define CAN_TXE_FIFO_RDATA          	0x0500 /* This register gives the value read from TX event FIFO */

#define CAN_INT_MASK_ALL	0xefff
#define CAN_INT_MASK_NONE	0x0000

#define CAN_RX_FILTER_MASK	0x1fffffff
#define CAN_TX_ID_MASK		0x1fffffff

struct rk3568_data {
	struct can_priv can;
	struct device *dev;
	void __iomem *base;
	struct reset_control *reset;
	struct clk_bulk_data *clks;
	int num_clks;
};

/**
 * Reset CAN controller
 *
 */
static void rk3568_can_reset(struct rk3568_data *rd)
{
	/* This sequence resets whole CAN controller */
	reset_control_assert(rd->reset);
	udelay(5);
	reset_control_deassert(rd->reset);

	writew(0, rd->base + CAN_MODE);
}

static void rk3568_can_work(struct rk3568_data *rd)
{
	u16 mode = readw(rd->base + CAN_MODE);
	mode |= (1u << 0) | (1u << 10);

	writew(mode, rd->base + CAN_MODE);
}

static int rk3568_can_set_btt(struct net_device *ndev)
{
	struct rk3568_data *rd = netdev_priv(ndev);
	struct can_bittiming *btt = &rd->can.bittiming;

	u16 val = ((btt->sjw - 1) << 14) | (((btt->brp >> 1) - 1) << 8) |
		  /* rk3568 phase_seg1 is actuallty prop_seg + phase_seg1 */
		  ((btt->prop_seg + btt->phase_seg1 - 1)) |
		  ((btt->phase_seg2 - 1) << 4);
	if (rd->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		val |= 1 << 16;

	writew(val, rd->base + CAN_BITTIMING);

	dev_info(rd->dev, "setting bittiming: 0x%04x, brp: %u, bitrate: %u\n",
		 val, btt->brp, btt->bitrate);

	return 0;
}

static int rk3568_can_start(struct net_device *ndev)
{
	struct rk3568_data *rd = netdev_priv(ndev);

	/* Reset controller, ensure proper initial state */
	rk3568_can_reset(rd);
	/* Enable all interrupts */
	writel(CAN_INT_MASK_NONE, rd->base + CAN_INT_MASK)
	/* Accept all messages ids */
	writel(0, rd->base + CAN_ID);
	writel(CAN_RX_FILTER_MASK, rd->base + CAN_IDMASK);

	/* Set bittimings */
	rk3568_can_set_btt(ndev);
	rk3568_can_normal(rd);

	rd->can.state = CAN_STATE_ERROR_ACTIVE;
	dev_info(rd->dev, "CAN started!\n");

	return 0;
}

static int rk3568_can_open(struct net_device *ndev)
{
	struct rk3568_data *rd = netdev_priv(ndev);
	int ret;

	ret = open_candev(ndev);
	if (ret) {
		dev_err(rd->dev, "Can't open CAN device!\n");
		return ret;
	}

	rk3568_can_start(ndev); /* This one can't fail */

	netif_start_queue(ndev);

	dev_info(rd->dev, "CAN opened!\n");

	return 0;
}

static int rk3568_can_stop(struct net_device *ndev)
{
	struct rk3568_data *rd = netdev_priv(ndev);
	int ret;
	u16 ints;

	netif_stop_queue(ndev);
	rd->can.state = CAN_STATE_STOPPED;
	rk3568_can_reset(rd);
	// This is crap, interrupts shoud be received and cleared.
	writew(CAN_INT_MASK_ALL, rd->base + CAN_INT_MASK);
	close_candev(ndev);
	dev_info(rd->dev, "CAN stopped!\n");

	return 0;
}

static int rk3568_can_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct rk3568_data *rd = netdev_priv(ndev);
	struct can_frame *cf = (struct can_frame *)skb->data;
	u8 tx_frm_info = 0;
	u32 data1 = 0;
	u32 data2 = 0;

	if (can_dropped_invalid_skb(ndev, skb);
		return NETDEV_TX_OK;

	netif_stop_queue(ndev);

	/* Check frame properties */
	if (cf->can_id & CAN_EFF_FLAG)
		tx_frm_info |= 0x80;

	/* Determines also message length */
	if (!(cf->can_id & CAN_RTR_FLAG)) {
		tx_frm_info = cf->len; /* message length */
		/* Non RTR messages have data */
		data1 = __le32_to_cpup((__le32 *)&cf->data[0]);
		data2 = __le32_to_cpup((__le32 *)&cf->data[4]);
	} else {
		tx_frm_info |= 0x40;
	}

	/* Clear requests */
	writel(0, rd->base + CAN_CMD);
	/* Write ID */
	writel(cf->can_id & CAN_TX_ID_MASK, rd->base + CAN_TXID);
	/* Write data if available */
	if (!(cf->can_id & CAN_RTR_FLAG)) {
		writel(data1, rd->base + CAN_TXDATA0);
		writel(data2, rd->base + CAN_TXDATA1);
	}
	/* Write additional packet information */
	writeb(tx_frm_info, rd->base + CAN_TXFRAMEINFO);
	/* Start packet processing */
	can_put_echo_skb(skb, ndev, 0, 0);

	/* Request transfer */
	writel(1, rd->base + CAN_CMD);

	dev_info_ratelimited(rd->dev, "tx: can_id:0x%08x, dlc:%u, d0:0x%08x, d1:0x%08x\n",
	                     cf->can_id, cf->len, data1, data2);

	return NETDEV_TX_OK;
}

static const struct net_device_ops rk3568_can_netdev_ops = {
	.ndo_open = rk3568_can_open,
	.ndo_stop = rk3568_can_stop,
	.ndo_start_xmit = rk3568_can_start_xmit,
};

static void rk3568_can_rx(struct net_device *ndev)
{
	struct rk3568_data *rd = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;

	struct can_frame *cf;
	struct sk_buff *skb;

	u8 rx_frm_info;
	u8 len;

	skb = alloc_can_skb(ndev, &cf);
	if (!skb)
		return;

	rx_frm_info = readb(rd->base + CAN_RXFRAMEINFO);
	len = rx_frm_info & 0xf;
}

static irqreturn_t rk3568_can_intr(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct rk3568_data *rd = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;

	u8 intr = readb(rd->base + CAN_INT);

	/* TX Finish */
	if (intr & (1u << 1)) {
		u8 bytes = readb(rd->base + CAN_TXFRAMEINFO);

		/* Update statistics */
		atomic_add((bytes & 0xf), &stats->tx_bytes);
		atomic_inc(&stats->tx_packets);

		/* Clear request */
		writel(0, rd->base + CAN_CMD);
		/* Make skb available */
		can_get_echo_skb(ndev, 0, NULL);
		/* Resume sending data */
		netif_wake_queue(ndev);
	}

	if (intr & (1u << 0)) {

	}

	writeb(intr, rd->base + CAN_INT);

	return IRQ_HANDLED;
}

/* Bit timing structure */
static const struct can_bittiming_const rk3568_can_btt_const = {
        .name = DRV_NAME,
        .tseg1_min = 1,
        .tseg1_max = 16,
        .tseg2_min = 1,
        .tseg2_max = 8,
        .sjw_max = 4,
        .brp_min = 1,
        .brp_max = 128,
        .brp_inc = 2,
};

static int rk3568_can_set_mode(struct net_device *ndev, enum can_mode mode)
{
	struct rk2568_data *rd = netdev_priv(ndev);

	dev_info(rd->dev, "CAN Set mode %u\n", mode);

	switch (mode) {
	case CAN_MODE_START:
		rk3568_can_start(ndev);
		if (netif_queue_stopped(ndev)
			netif_wake_queue(ndev);
		break;

	case CAN_MODE_STOP:
		return -EOPNOTSUPP;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int rk3568_can_get_berr_counter(struct net_device *ndev,
				       struct can_berr_counter *bec)
{
	struct rk3568_data *rd = netdev_priv(ndev);

	bec->rxerr = readb(rd->base + CAN_RXERROR_CNT);
	bec->txerr = readb(rd->base + CAN_TXERROR_CNT);

	dev_info(rd->dev, "CAN get error counters rx:%u tx:%u\n",
		 bec->rxerr, bec->txerr);

	return 0;
}

static int rk3568_can_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct rk3568_data *rd;
	struct resource *res;
	void __iomem *addr;
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "could not get a valid irq\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(addr))
		return PTR_ERR(addr);

	ndev = alloc_candev(sizeof(struct rk3568_data), 1);
	if (!ndev) {
		dev_err(&pdev->dev, "Can't allocate nework device");
		return -ENOMEM;
	}

	ndev->irq = irq;
	ndev->base_addr = res->start;
	ndev->flags |= IFF_ECHO;

	rd = netdev_priv(ndev);
	rd->dev = &pdev->dev;
	rd->base = addr;

	/* Register all operations */
	ndev->netdev_ops = &rk3568_can_netdev_ops;

	rd->can.bittiming_const = &rk3568_can_btt_const;
        rd->can.do_set_mode = rk3568_can_set_mode;
	rd->can.do_get_berr_counter = rk3568_can_get_berr_counter;
	rd->can.ctrlmode_supported = CAN_CTRLMODE_BERR_REPORTING |
		                     CAN_CTRLMODE_3_SAMPLES;

	ret = devm_request_irq(&pdev->dev, irq, rk3568_can_intr, 0, "rk3568_can0", ndev);
	if (ret) {
		dev_err(&pdev->dev, "IRQ can't be registered");
		goto failure;
	}

	rd->reset = devm_reset_control_array_get(&pdev->dev, false, false);
	if (IS_ERR(rd->reset)) {
		int ret = PTR_ERR(rd->reset);

		/* Ensure that reset controller is up and running */
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get get reset lines!");

		goto failure;
	}

	rd->num_clks = devm_clk_bulk_get_all(&pdev->dev, &rd->clks);
	if (rd->num_clks < 1) {
		dev_err(&pdev->dev, "Clocks information not found!");

		if (rd->num_clks < 0)
			ret = rd->num_clks;
		else
			ret = -EINVAL;

		goto failure;
	}

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	ret = register_candev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "CAN driver can't be registered!");
		goto failure;
	}

	dev_info(&pdev->dev, "CAN driver probed");

	return 0;

failure:
	free_candev(ndev);

	return ret;
}

static int rk3568_can_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	unregister_candev(ndev);
	dev_info(&pdev->dev, "CAN driver removed");

	return 0;
}

static const struct of_device_id rk3568_can_of_match[] = {
	{ .compatible = "rk3568,can-1.0" },
	{},
};
MODULE_DEVICE_TABLE(of, rk3568_can_of_match);

static struct platform_driver rk3568_can_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = rk3568_can_of_match,
	},
	.probe = rk3568_can_probe,
	.remove = rk3568_can_remove,
};
module_platform_driver(rk3568_can_driver);

MODULE_AUTHOR("Wojciech Bartczak <wbartczak@gmail.com>");
MODULE_DESCRIPTION("RK3568 CAN Drivers");
MODULE_LICENSE("GPL v2");
