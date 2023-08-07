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
#include <linux/can/length.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include <linux/pm_runtime.h>

#define DRV_NAME		"rk3568_can"

/* RK3568 CAN registers */
#define CAN_MODE                    	0x0000 /* CAN controller working mode. configure register */

#define CAN_MODE_FD_CAN                 BIT(15)
#define CAN_MODE_DPEE                   BIT(14)
#define CAN_MODE_BRSD                   BIT(13)
#define CAN_MODE_SPACE_RX_MODE          BIT(12)
#define CAN_MODE_AUTO_BUS_ON            BIT(11)
#define CAN_MODE_AUTO_RETX_MODE         BIT(10)
#define CAN_MODE_OVERLOAD_MODE          BIT(9)
#define CAN_MODE_COVER_MODE             BIT(8)
#define CAN_MODE_RXSORT_MODE            BIT(7)
#define CAN_MODE_TXORDER_MODE           BIT(6)
#define CAN_MODE_RXSTX_MODE             BIT(5)
#define CAN_MODE_LOOPBACK_MODE          BIT(4)
#define CAN_MODE_SILENT_MODE            BIT(3)
#define CAN_MODE_SELF_TEST              BIT(2)
#define CAN_MODE_SLEEP_MODE             BIT(1)
#define CAN_MODE_WORK_MODE              BIT(0)

#define CAN_CMD                     	0x0004 /* CAN command register */

#define CAN_CMD_TX1_REQ			BIT(1)
#define CAN_CMD_TX0_REQ			BIT(0)	/* Transmit enable request buffer0 */

#define CAN_STATE                   	0x0008 /* CAN state register */

#define CAN_STATE_SLEEP			BIT(6)	/* CAN controller is in sleep state */
#define CAN_STATE_BUS_OFF		BIT(5)  /* CAN controller is in bus off state */
#define CAN_STATE_ERROR_WARN		BIT(4)  /* Error waring state for CAN controller */
#define CAN_STATE_TX_PERIOD		BIT(3)  /* Controller is transmiting data */
#define CAN_STATE_RX_PERIOD		BIT(2)  /* Contriller is receiving data */
#define CAN_STATE_TX_BUF_FULL		BIT(1)  /* TX buffer is full */
#define CAN_STATE_RX_BUF_FULL		BIT(0)  /* RX buffer is full */

#define CAN_INT                     	0x000c /* Interrupt state register */

#define CAN_INT_WAKEUP                          BIT(14)
#define CAN_INT_TX_EVENT_FIFO_FULL              BIT(13)
#define CAN_INT_TX_EVENT_FIFO_OVERFLOW          BIT(12)
#define CAN_INT_TIMESTAMP_COUNTER_OVERFLOW      BIT(11)
#define CAN_INT_BUS_OFF_RECOVERY                BIT(10)
#define CAN_INT_BUS_OFF                         BIT(9)
#define CAN_INT_RX_EVENT_FIFO_OVERFLOW          BIT(8)
#define CAN_INT_RX_EVENT_FIFO_FULL              BIT(7)
#define CAN_INT_ERROR_OFF                       BIT(6)
#define CAN_INT_TX_ARBIT_FAIL                   BIT(5)
#define CAN_INT_PASSIVE_ERROR                   BIT(4)
#define CAN_INT_OVERLOAD                        BIT(3)
#define CAN_INT_ERROR_WARNING                   BIT(2)
#define CAN_INT_TX_FINISH                       BIT(1)
#define CAN_INT_RX_FINISH                       BIT(0)

#define CAN_INT_MASK                	0x0010 /* Interrupt enable registers */
#define CAN_DMA_CTRL                	0x0014 /* DMA mode control */
#define CAN_BITTIMING               	0x0018 /* Bit timing configure register */
#define CAN_ARBITFAIL               	0x0028 /* Arbit fail code register */
#define CAN_ERROR_CODE              	0x002c /* Error code register */
#define CAN_RXERRORCNT              	0x0034 /* Receive error counter */
#define CAN_TXERRORCNT              	0x0039 /* Transmit error counter */
#define CAN_IDCODE                  	0x003c /* CAN controller's identifier */
#define CAN_IDMASK                  	0x0040 /* Identification code bit mask register */
#define CAN_TXFRMINFO             	0x0050 /* TX frame information configuration register */

#define CAN_FRMINFO_TX_EFF              BIT(7) /* Extende frame format */
#define CAN_FRMINFO_TX_RTR              BIT(6) /* Remote transfer request */
/* Data length encoded to DLC */
#define CAN_FRMINFO_DATA_LEN_MASK       GENMASK(3, 0)
#define CAN_FRMINFO_DATA_LEN(info)      ((info) & CAN_FRMINFO_DATA_LEN_MASK)

#define CAN_TXID                    	0x0054 /* CAN controller transmit ID */
#define CAN_TXDATA0                 	0x0058 /* CAN controller transmit DATA1 */
#define CAN_TXDATA1                 	0x005c /* CAN controller transmit DATA1 */
#define CAN_RXFRMINFO             	0x0060 /* RX frame information register. This register needs to be read (clear) before receiving the next frame. */

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

#define CAN_INT_MASK_ALL	0x0000efff
#define CAN_INT_MASK_NONE	0x00000000

#define CAN_RX_FILTER_MASK	0x1fffffff
#define CAN_TX_ID_MASK		0x1fffffff

struct rk3568_data {
	struct can_priv can;
	struct device *dev;
	void __iomem *base;
	struct reset_control *reset;
	struct clk *pclk;
	struct clk *baudclk;
};

/**
 * Reset CAN controller
 *
 */
static void rk3568_can_reset(struct rk3568_data *rd)
{
	/* This sequence resets whole CAN controller */
	reset_control_assert(rd->reset);
	udelay(2);
	reset_control_deassert(rd->reset);

	writel(0, rd->base + CAN_MODE);
}

static void rk3568_can_work(struct rk3568_data *rd)
{
	u32 mode = readl(rd->base + CAN_MODE);
	mode |= CAN_MODE_WORK_MODE | CAN_MODE_AUTO_RETX_MODE;
	//	| CAN_MODE_LOOPBACK_MODE | CAN_MODE_SELF_TEST;
	
	writel(mode, rd->base + CAN_MODE);
}

static int rk3568_can_set_btt(struct net_device *ndev)
{
	struct rk3568_data *rd = netdev_priv(ndev);
	struct can_bittiming *btt = &rd->can.bittiming;

	u32 val = ((btt->sjw - 1) << 14) | (((btt->brp >> 1) - 1) << 8) |
		  /* rk3568 phase_seg1 is actuallty prop_seg + phase_seg1 */
		  ((btt->prop_seg + btt->phase_seg1 - 1)) |
		  ((btt->phase_seg2 - 1) << 4);
	if (rd->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		val |= 1 << 16;

	writel(val, rd->base + CAN_BITTIMING);
	val = readl(rd->base + CAN_BITTIMING);

	dev_info(rd->dev, "setting bittiming: 0x%04x, brp: %u, bitrate: %u\n",
		 val, btt->brp, btt->bitrate);

	return 0;
}

static int rk3568_can_start(struct net_device *ndev)
{
	struct rk3568_data *rd = netdev_priv(ndev);
	int ret;

	/* Reset controller, ensure proper initial state */
	rk3568_can_reset(rd);
	
	/* Enable all interrupts */
	writel(0, rd->base + CAN_INT_MASK);

	/* Accept all messages ids */
	writel(0, rd->base + CAN_IDCODE);
	writel(CAN_RX_FILTER_MASK, rd->base + CAN_IDMASK);

	/* Set bittimings */
	rk3568_can_set_btt(ndev);

	rk3568_can_work(rd);

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

	ret = clk_enable(rd->baudclk);
	if (ret) {
		dev_err(rd->dev, "Can't enable baudclk\n");
		return ret;
	}

	ret = rk3568_can_start(ndev);
	if (ret)
		return ret;

	netif_start_queue(ndev);

	dev_info(rd->dev, "CAN opened!\n");

	return 0;
}

static int rk3568_can_stop(struct net_device *ndev)
{
	struct rk3568_data *rd = netdev_priv(ndev);

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
	u32 cmd;

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(ndev);

	/* Check frame properties */
	if (cf->can_id & CAN_EFF_FLAG)
		tx_frm_info |= CAN_FRMINFO_TX_EFF;

	/* Determines also message length */
	if (!(cf->can_id & CAN_RTR_FLAG)) {
		tx_frm_info = cf->len; /* message length */
		/* Non RTR messages have data */
		data1 = __le32_to_cpup((__le32 *)&cf->data[0]);
		data2 = __le32_to_cpup((__le32 *)&cf->data[4]);
	} else {
		tx_frm_info |= CAN_FRMINFO_TX_RTR;
	}

	/* Clear requests */
	cmd = readl(rd->base + CAN_CMD);
	writel(cmd & (~CAN_CMD_TX0_REQ), rd->base + CAN_CMD);
	
	/* Write ID */
	writel(cf->can_id & CAN_TX_ID_MASK, rd->base + CAN_TXID);
	writel(data1, rd->base + CAN_TXDATA0);
	writel(data2, rd->base + CAN_TXDATA1);
	/* Write additional packet information */
	writeb(tx_frm_info, rd->base + CAN_TXFRMINFO);
	/* Start packet processing */
	can_put_echo_skb(skb, ndev, 0, 0);

	/* Request transfer */
	cmd = readl(rd->base + CAN_CMD);
	writel(cmd | CAN_CMD_TX0_REQ, rd->base + CAN_CMD);

	dev_info_ratelimited(rd->dev, "tx: can_id:0x%08x, dlc:%u, d0:0x%08x, d1:0x%08x\n",
	                     cf->can_id, cf->len, data1, data2);

	return NETDEV_TX_OK;
}

static const struct net_device_ops rk3568_can_netdev_ops = {
	.ndo_open = rk3568_can_open,
	.ndo_stop = rk3568_can_stop,
	.ndo_start_xmit = rk3568_can_start_xmit,
};

static void rk3568_can_error(struct net_device *ndev, u32 intrs)
{
	struct rk3568_data *rd = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;

	struct can_frame *cf;
	struct sk_buff *skb;
	enum can_state state;

	u32 stater;
	canid_t id;
	u8 rx_frm_info;
	u8 len;
	u32 data1 = 0;
	u32 data2 = 0;
	u8 txerr;
	u8 rxerr;
	u32 errc;

	skb = alloc_can_err_skb(ndev, &cf);
	if (!skb) {
		dev_err_ratelimited(rd->dev, "Can't allocate skb for error!\n");
		return;
	}

	rx_frm_info = readb(rd->base + CAN_RXFRMINFO);
	id = readl(rd->base + CAN_RXID);
	len = can_cc_dlc2len(rx_frm_info & 0xf);

	data1 = readl(rd->base + CAN_RXDATA0);
	data2 = readl(rd->base + CAN_RXDATA1);

	txerr = readb(rd->base + CAN_TXERRORCNT);
	rxerr = readb(rd->base + CAN_RXERRORCNT);
	stater = readl(rd->base + CAN_STATE);

	stats->rx_packets++;
	stats->rx_bytes += len;
	
	cf->data[6] = txerr;
	cf->data[7] = rxerr;

	/* Overload/Overflow */
	if (intrs && BIT(3)) {
		id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;

		rk3568_can_reset(rd);
		rk3568_can_work(rd);
	}

	if (intrs && BIT(2)) {
		if (stater & BIT(5)) // Bus off
			state = CAN_STATE_BUS_OFF;
		else if (stater & BIT(4))
			state = CAN_STATE_ERROR_WARNING;
		else
			state = CAN_STATE_ERROR_ACTIVE;
	}

	if (intrs & BIT(6)) {
	//	make this can bus error stats->bus_errors++;
		stats->rx_errors++;
	  	errc = readl(rd->base + CAN_ERROR_CODE);
		id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

		switch ((errc && 0x1c000000) >> 26) {
		case 0: // BIT error
			cf->data[2] |= CAN_ERR_PROT_BIT;
			break;
		case 1: // BIT STUFF ERROR
			cf->data[2] |= CAN_ERR_PROT_STUFF;
			break;
		case 2: // FORM ERROR
			cf->data[2] |= CAN_ERR_PROT_FORM;
			break;
		default:
			dev_info_ratelimited(rd->dev, "Unknow error errc:0x%08x\n", errc);
		}
		if (errc && BIT(25) == 0)
			cf->data[2] |= CAN_ERR_PROT_TX;
	}

		
	cf->can_id = id;

	dev_info_ratelimited(rd->dev, "error handler intrs: 0x%08x\n", intrs);
}

static irqreturn_t rk3568_can_tx_finish(struct net_device *ndev)
{
	struct rk3568_data *rd = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	int ret;

	u32 tx_info = readl(rd->base + CAN_TXFRMINFO);
	u32 tx_cmd = readl(rd->base + CAN_CMD);
	unsigned long bytes = can_cc_dlc2len(CAN_FRMINFO_DATA_LEN(tx_info));
	
	/* Clear request flag */
	writel(tx_cmd & (~CAN_CMD_TX0_REQ), rd->base + CAN_CMD);

	ret = can_get_echo_skb(ndev, 0, NULL);
	if (!ret) {
		dev_info_ratelimited(rd->dev, "Incomming packet dropped\n");
	} else {
		stats->tx_bytes += bytes;
		stats->tx_packets++;
	}
	netif_wake_queue(ndev);

	return IRQ_HANDLED;
}

static void rk3568_can_rx_finish(struct net_device *ndev)
{
	struct rk3568_data *rd = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;

	struct can_frame *cf;
	struct sk_buff *skb;
	
	uint32_t rx_info;
	canid_t id;

	unsigned long len;
	u32 data1 = 0;
	u32 data2 = 0;

	skb = alloc_can_skb(ndev, &cf);
	if (!skb) {
		dev_info_ratelimited(rd->dev, "RX frame dropped skb=NULL\n");
		goto failure;
	}

	rx_info = readl(rd->base + CAN_RXFRMINFO);
	len = can_cc_dlc2len(CAN_FRMINFO_DATA_LEN(rx_info));
	id = readl(rd->base + CAN_RXID);

	if (rx_info & CAN_FRMINFO_TX_EFF)
		id |= CAN_EFF_FLAG;


	if (rx_info & CAN_FRMINFO_TX_RTR) {
		id |= CAN_RTR_FLAG;
		/* No data for RTR, but have to read  to clear int */
       		(void) readl(rd->base + CAN_RXDATA0);
		(void) readl(rd->base + CAN_RXDATA1);	
	} else {
		data1 = readl(rd->base + CAN_RXDATA0);
		data2 = readl(rd->base + CAN_RXDATA1);
	}

	cf->can_id = id;
	cf->len = len;
	*((__le32 *)&cf->data[0]) = cpu_to_le32(data1);
	*((__le32 *)&cf->data[4]) = cpu_to_le32(data2);

	stats->rx_packets++;
	stats->rx_bytes += cf->len;

	netif_rx(skb);	

	return;

failure:
	(void) readl(rd->base + CAN_RXFRMINFO);
	(void) readl(rd->base + CAN_RXID);
	(void) readl(rd->base + CAN_RXDATA0);
	(void) readl(rd->base + CAN_RXDATA1);
}

static irqreturn_t rk3568_can_intr(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct rk3568_data *rd = netdev_priv(ndev);

	const u32 err_intr_mask = 0x7ffc; /* All aside RX_FINISH and TX_FINISH */
	u32 intr = readl(rd->base + CAN_INT);
	irqreturn_t ret = IRQ_HANDLED;

	if (intr & CAN_INT_TX_FINISH)
		ret = rk3568_can_tx_finish(ndev);  

	if (intr & CAN_INT_RX_FINISH)
		rk3568_can_rx_finish(ndev);

	if (intr & err_intr_mask)
		rk3568_can_error(ndev, intr);

	writeb(intr, rd->base + CAN_INT);

	return ret;
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
	struct rk3568_data *rd = netdev_priv(ndev);

	dev_info(rd->dev, "CAN Set mode %u\n", mode);

	switch (mode) {
	case CAN_MODE_START:
		rk3568_can_start(ndev);
		if (netif_queue_stopped(ndev))
			netif_wake_queue(ndev);
		break;

	case CAN_MODE_STOP:
		return -EOPNOTSUPP;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int rk3568_can_get_berr_counter(const struct net_device *ndev,
				       struct can_berr_counter *bec)
{
	struct rk3568_data *rd = netdev_priv(ndev);

	bec->rxerr = readb(rd->base + CAN_RXERRORCNT);
	bec->txerr = readb(rd->base + CAN_TXERRORCNT);

	dev_info(rd->dev, "CAN get error counters rx:%u tx:%u\n",
		 bec->rxerr, bec->txerr);

	return 0;
}

static ssize_t settings_show(struct device *dev, struct device_attribute *atr, char *buf)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rk3568_data *rd = netdev_priv(ndev);

	u32 btt = readl(rd->base + CAN_BITTIMING);
	u32 mode = readl(rd->base + CAN_MODE);

	return sysfs_emit(buf, "mode:0x%x, btt:0x%x\n", mode, btt);
}
static DEVICE_ATTR_RO(settings);


static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct rk3568_data *rd = netdev_priv(ndev);

	u32 intr;
	u32 stater;
	u32 arbitfailr;
	u32 errcr;

	intr = readl(rd->base + CAN_INT);
	stater = readl(rd->base + CAN_STATE);
	arbitfailr = readl(rd->base + CAN_ARBITFAIL);
	errcr = readl(rd->base + CAN_ERROR_CODE);	

	return sysfs_emit(buf, "intr:0x%x, state:0x%x, arbt:0x%x, errc:0x%x\n",
			  intr, stater, arbitfailr, errcr);
}

static DEVICE_ATTR_RO(status);

static struct attribute *rk3568_can_attrs[] = {
	&dev_attr_settings.attr,
	&dev_attr_status.attr,
	NULL,
};

static const struct attribute_group rk3568_can_attr_group = {
	.attrs = rk3568_can_attrs,
};

static const struct attribute_group *rk3568_can_attr_groups[] = {
	&rk3568_can_attr_group,
	NULL,
};	

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
		dev_err(&pdev->dev, "Can't allocate nework device\n");
		return -ENOMEM;
	}

	ndev->irq = irq;
	ndev->base_addr = res->start;
	ndev->flags |= IFF_ECHO;

	rd = netdev_priv(ndev);
	rd->dev = &pdev->dev;
	rd->base = addr;

	ret = devm_request_irq(&pdev->dev, irq, rk3568_can_intr, 0, "rk3568_can0", ndev);
	if (ret) {
		dev_err(&pdev->dev, "IRQ can't be registered\n");
		goto failure;
	}

	rd->reset = devm_reset_control_array_get(&pdev->dev, false, false);
	if (IS_ERR(rd->reset)) {
		int ret = PTR_ERR(rd->reset);

		/* Ensure that reset controller is up and running */
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Failed to get get reset lines!\n");

		goto failure;
	}


	rd->pclk = of_clk_get(pdev->dev.of_node, 1);
	if (IS_ERR_OR_NULL(rd->pclk)) {
		ret = PTR_ERR(rd->pclk);
		if (ret == -EPROBE_DEFER)
			return ret;
			
		dev_err(&pdev->dev, "Can't find apb_pclk\n");
		goto failure;
	}

	rd->baudclk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR_OR_NULL(rd->baudclk)) {
		ret = PTR_ERR(rd->baudclk);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_err(&pdev->dev, "Can't find baudclk\n");
		goto failure;
	}

	/* Start pclk */
	ret = clk_prepare_enable(rd->pclk);
	if (ret) {
		dev_err(&pdev->dev, "Can't start pclk\n");
		goto failure;
	}

	/* Prepare baudclk, enable during open */
	ret = clk_prepare(rd->baudclk);
	if (ret) {
		dev_err(&pdev->dev, "Can't prepare baudclk\n");
		goto failure;
	}

	/* Register all operations */
	ndev->netdev_ops = &rk3568_can_netdev_ops;
	
	rd->can.clock.freq = clk_get_rate(rd->baudclk);
	rd->can.bittiming_const = &rk3568_can_btt_const;
        rd->can.do_set_mode = rk3568_can_set_mode;
	rd->can.do_get_berr_counter = rk3568_can_get_berr_counter;
	rd->can.ctrlmode_supported = CAN_CTRLMODE_BERR_REPORTING |
				     CAN_CTRLMODE_LISTENONLY |
				     CAN_CTRLMODE_LOOPBACK |
		                     CAN_CTRLMODE_3_SAMPLES;

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	ret = devm_device_add_groups(&pdev->dev, rk3568_can_attr_groups);
	if (ret) {
		dev_err(&pdev->dev, "Can't register extra attributes!\n");
		goto failure;
	}

	ret = register_candev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "CAN driver can't be registered!\n");
		goto failure;
	}

	dev_info(&pdev->dev, "CAN driver probed\n");

	return 0;

failure:
	free_candev(ndev);

	return ret;
}

static int rk3568_can_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	unregister_candev(ndev);
	dev_info(&pdev->dev, "CAN driver removed\n");

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
