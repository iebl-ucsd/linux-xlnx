// SPDX-License-Identifier: GPL-2.0-only
/*
 * Xilinx Axi Ethernet device driver
 *
 * Copyright (c) 2008 Nissin Systems Co., Ltd.,  Yoshio Kashiwagi
 * Copyright (c) 2005-2008 DLA Systems,  David H. Lynch Jr. <dhlii@dlasys.net>
 * Copyright (c) 2008-2009 Secret Lab Technologies Ltd.
 * Copyright (c) 2010 - 2011 Michal Simek <monstr@monstr.eu>
 * Copyright (c) 2010 - 2011 PetaLogix
 * Copyright (c) 2019 SED Systems, a division of Calian Ltd.
 * Copyright (c) 2010 - 2012 Xilinx, Inc. All rights reserved.
 *
 * This is a driver for the Xilinx Axi Ethernet which is used in the Virtex6
 * and Spartan6.
 *
 * TODO:
 *  - Add Axi Fifo support.
 *  - Factor out Axi DMA code into separate driver.
 *  - Test and fix basic multicast filtering.
 *  - Add support for extended multicast filtering.
 *  - Test basic VLAN support.
 *  - Add support for extended VLAN support.
 */

#include <linux/clk.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/phy.h>
#include <linux/mii.h>
#include <linux/ethtool.h>

#include "xilinx_axienet.h"

/* Descriptors defines for Tx and Rx DMA */
#define TX_BD_NUM_DEFAULT		128
#define RX_BD_NUM_DEFAULT		1024
#define TX_BD_NUM_MIN			(MAX_SKB_FRAGS + 1)
#define TX_BD_NUM_MAX			4096
#define RX_BD_NUM_MAX			4096

/* Must be shorter than length of ethtool_drvinfo.driver field to fit */
#define DRIVER_NAME		"xaxienet"
#define DRIVER_DESCRIPTION	"Xilinx Axi Ethernet driver"
#define DRIVER_VERSION		"1.00a"

#define AXIENET_REGS_N		40
#define AXIENET_TS_HEADER_LEN	8
#define XXVENET_TS_HEADER_LEN	4
#define MRMAC_TS_HEADER_LEN		16
#define MRMAC_TS_HEADER_WORDS   (MRMAC_TS_HEADER_LEN / 4)
#define NS_PER_SEC              1000000000ULL /* Nanoseconds per second */

#define MRMAC_RESET_DELAY	1 /* Delay in msecs*/

/* IEEE1588 Message Type field values  */
#define PTP_TYPE_SYNC		0
#define PTP_TYPE_PDELAY_REQ	2
#define PTP_TYPE_PDELAY_RESP	3
#define PTP_TYPE_OFFSET		42
/* SW flags used to convey message type for command FIFO handling */
#define MSG_TYPE_SHIFT			4
#define MSG_TYPE_SYNC_FLAG		((PTP_TYPE_SYNC + 1) << MSG_TYPE_SHIFT)
#define MSG_TYPE_PDELAY_RESP_FLAG	((PTP_TYPE_PDELAY_RESP + 1) << \
									 MSG_TYPE_SHIFT)

#ifdef CONFIG_XILINX_TSN_PTP
int axienet_phc_index = -1;
EXPORT_SYMBOL(axienet_phc_index);
#endif

void __iomem *mrmac_gt_pll;
EXPORT_SYMBOL(mrmac_gt_pll);

void __iomem *mrmac_gt_ctrl;
EXPORT_SYMBOL(mrmac_gt_ctrl);

int mrmac_pll_reg;
EXPORT_SYMBOL(mrmac_pll_reg);

int mrmac_pll_rst;
EXPORT_SYMBOL(mrmac_pll_rst);

/* Option table for setting up Axi Ethernet hardware options */
static struct axienet_option axienet_options[] = {
	/* Turn on jumbo packet support for both Rx and Tx */
	{
		.opt = XAE_OPTION_JUMBO,
		.reg = XAE_TC_OFFSET,
		.m_or = XAE_TC_JUM_MASK,
	}, {
		.opt = XAE_OPTION_JUMBO,
		.reg = XAE_RCW1_OFFSET,
		.m_or = XAE_RCW1_JUM_MASK,
	}, { /* Turn on VLAN packet support for both Rx and Tx */
		.opt = XAE_OPTION_VLAN,
		.reg = XAE_TC_OFFSET,
		.m_or = XAE_TC_VLAN_MASK,
	}, {
		.opt = XAE_OPTION_VLAN,
		.reg = XAE_RCW1_OFFSET,
		.m_or = XAE_RCW1_VLAN_MASK,
	}, { /* Turn on FCS stripping on receive packets */
		.opt = XAE_OPTION_FCS_STRIP,
		.reg = XAE_RCW1_OFFSET,
		.m_or = XAE_RCW1_FCS_MASK,
	}, { /* Turn on FCS insertion on transmit packets */
		.opt = XAE_OPTION_FCS_INSERT,
		.reg = XAE_TC_OFFSET,
		.m_or = XAE_TC_FCS_MASK,
	}, { /* Turn off length/type field checking on receive packets */
		.opt = XAE_OPTION_LENTYPE_ERR,
		.reg = XAE_RCW1_OFFSET,
		.m_or = XAE_RCW1_LT_DIS_MASK,
	}, { /* Turn on Rx flow control */
		.opt = XAE_OPTION_FLOW_CONTROL,
		.reg = XAE_FCC_OFFSET,
		.m_or = XAE_FCC_FCRX_MASK,
	}, { /* Turn on Tx flow control */
		.opt = XAE_OPTION_FLOW_CONTROL,
		.reg = XAE_FCC_OFFSET,
		.m_or = XAE_FCC_FCTX_MASK,
	}, { /* Turn on promiscuous frame filtering */
		.opt = XAE_OPTION_PROMISC,
		.reg = XAE_FMC_OFFSET,
		.m_or = XAE_FMC_PM_MASK,
	}, { /* Enable transmitter */
		.opt = XAE_OPTION_TXEN,
		.reg = XAE_TC_OFFSET,
		.m_or = XAE_TC_TX_MASK,
	}, { /* Enable receiver */
		.opt = XAE_OPTION_RXEN,
		.reg = XAE_RCW1_OFFSET,
		.m_or = XAE_RCW1_RX_MASK,
	},
	{}
};

/* Option table for setting up Axi Ethernet hardware options */
static struct xxvenet_option xxvenet_options[] = {
	{ /* Turn on FCS stripping on receive packets */
		.opt = XAE_OPTION_FCS_STRIP,
		.reg = XXV_RCW1_OFFSET,
		.m_or = XXV_RCW1_FCS_MASK,
	}, { /* Turn on FCS insertion on transmit packets */
		.opt = XAE_OPTION_FCS_INSERT,
		.reg = XXV_TC_OFFSET,
		.m_or = XXV_TC_FCS_MASK,
	}, { /* Enable transmitter */
		.opt = XAE_OPTION_TXEN,
		.reg = XXV_TC_OFFSET,
		.m_or = XXV_TC_TX_MASK,
	}, { /* Enable receiver */
		.opt = XAE_OPTION_RXEN,
		.reg = XXV_RCW1_OFFSET,
		.m_or = XXV_RCW1_RX_MASK,
	},
	{}
};

/* Option table for setting up MRMAC hardware options */
static struct xxvenet_option mrmacenet_options[] = {
	{ /* Turn on FCS stripping on receive packets */
		.opt = XAE_OPTION_FCS_STRIP,
		.reg = MRMAC_CONFIG_RX_OFFSET,
		.m_or = MRMAC_RX_DEL_FCS_MASK,
	}, { /* Turn on FCS insertion on transmit packets */
		.opt = XAE_OPTION_FCS_INSERT,
		.reg = MRMAC_CONFIG_TX_OFFSET,
		.m_or = MRMAC_TX_INS_FCS_MASK,
	}, { /* Enable transmitter */
		.opt = XAE_OPTION_TXEN,
		.reg = MRMAC_CONFIG_TX_OFFSET,
		.m_or = MRMAC_TX_EN_MASK,
	}, { /* Enable receiver */
		.opt = XAE_OPTION_RXEN,
		.reg = MRMAC_CONFIG_RX_OFFSET,
		.m_or = MRMAC_RX_EN_MASK,
	},
	{}
};

struct axienet_ethtools_stat {
	const char *name;
};

static struct axienet_ethtools_stat axienet_get_ethtools_strings_stats[] = {
	{ "tx_packets" },
	{ "rx_packets" },
	{ "tx_bytes" },
	{ "rx_bytes" },
	{ "tx_errors" },
	{ "rx_errors" },
};

/**
 * axienet_dma_bd_release - Release buffer descriptor rings
 * @ndev:	Pointer to the net_device structure
 *
 * This function is used to release the descriptors allocated in
 * axienet_dma_bd_init. axienet_dma_bd_release is called when Axi Ethernet
 * driver stop api is called.
 */
void axienet_dma_bd_release(struct net_device *ndev)
{
	int i;
	struct axienet_local *lp = netdev_priv(ndev);

#ifdef CONFIG_AXIENET_HAS_MCDMA
	for_each_tx_dma_queue(lp, i) {
		axienet_mcdma_tx_bd_free(ndev, lp->dq[i]);
	}
#endif
	for_each_rx_dma_queue(lp, i) {
#ifdef CONFIG_AXIENET_HAS_MCDMA
		axienet_mcdma_rx_bd_free(ndev, lp->dq[i]);
#else
		axienet_bd_free(ndev, lp->dq[i]);
#endif
	}
}

/**
 * axienet_dma_bd_init - Setup buffer descriptor rings for Axi DMA
 * @ndev:	Pointer to the net_device structure
 *
 * Return: 0, on success -ENOMEM, on failure -EINVAL, on default return
 *
 * This function is called to initialize the Rx and Tx DMA descriptor
 * rings. This initializes the descriptors with required default values
 * and is called when Axi Ethernet driver reset is called.
 */
static int axienet_dma_bd_init(struct net_device *ndev)
{
	int i, ret = -EINVAL;
	struct axienet_local *lp = netdev_priv(ndev);

#ifdef CONFIG_AXIENET_HAS_MCDMA
	for_each_tx_dma_queue(lp, i) {
		ret = axienet_mcdma_tx_q_init(ndev, lp->dq[i]);
		if (ret != 0)
			break;
	}
#endif
	for_each_rx_dma_queue(lp, i) {
#ifdef CONFIG_AXIENET_HAS_MCDMA
		ret = axienet_mcdma_rx_q_init(ndev, lp->dq[i]);
#else
		ret = axienet_dma_q_init(ndev, lp->dq[i]);
#endif
		if (ret != 0) {
			netdev_err(ndev, "%s: Failed to init DMA buf %d\n", __func__, ret);
			break;
		}
	}

	return ret;
}

/**
 * axienet_set_mac_address - Write the MAC address
 * @ndev:	Pointer to the net_device structure
 * @address:	6 byte Address to be written as MAC address
 *
 * This function is called to initialize the MAC address of the Axi Ethernet
 * core. It writes to the UAW0 and UAW1 registers of the core.
 */
void axienet_set_mac_address(struct net_device *ndev,
			     const void *address)
{
	struct axienet_local *lp = netdev_priv(ndev);

	if (address)
		ether_addr_copy(ndev->dev_addr, address);
	if (!is_valid_ether_addr(ndev->dev_addr))
		eth_hw_addr_random(ndev);

	if (lp->axienet_config->mactype != XAXIENET_1G &&
	    lp->axienet_config->mactype != XAXIENET_2_5G)
		return;

	/* Set up unicast MAC address filter set its mac address */
	axienet_iow(lp, XAE_UAW0_OFFSET,
		    (ndev->dev_addr[0]) |
		    (ndev->dev_addr[1] << 8) |
		    (ndev->dev_addr[2] << 16) |
		    (ndev->dev_addr[3] << 24));
	axienet_iow(lp, XAE_UAW1_OFFSET,
		    (((axienet_ior(lp, XAE_UAW1_OFFSET)) &
		      ~XAE_UAW1_UNICASTADDR_MASK) |
		     (ndev->dev_addr[4] |
		     (ndev->dev_addr[5] << 8))));
}

/**
 * netdev_set_mac_address - Write the MAC address (from outside the driver)
 * @ndev:	Pointer to the net_device structure
 * @p:		6 byte Address to be written as MAC address
 *
 * Return: 0 for all conditions. Presently, there is no failure case.
 *
 * This function is called to initialize the MAC address of the Axi Ethernet
 * core. It calls the core specific axienet_set_mac_address. This is the
 * function that goes into net_device_ops structure entry ndo_set_mac_address.
 */
static int netdev_set_mac_address(struct net_device *ndev, void *p)
{
	struct sockaddr *addr = p;

	axienet_set_mac_address(ndev, addr->sa_data);
	return 0;
}

/**
 * axienet_set_multicast_list - Prepare the multicast table
 * @ndev:	Pointer to the net_device structure
 *
 * This function is called to initialize the multicast table during
 * initialization. The Axi Ethernet basic multicast support has a four-entry
 * multicast table which is initialized here. Additionally this function
 * goes into the net_device_ops structure entry ndo_set_multicast_list. This
 * means whenever the multicast table entries need to be updated this
 * function gets called.
 */
void axienet_set_multicast_list(struct net_device *ndev)
{
	int i;
	u32 reg, af0reg, af1reg;
	struct axienet_local *lp = netdev_priv(ndev);

	if ((lp->axienet_config->mactype != XAXIENET_1G) || lp->eth_hasnobuf)
		return;

	if (ndev->flags & (IFF_ALLMULTI | IFF_PROMISC) ||
	    netdev_mc_count(ndev) > XAE_MULTICAST_CAM_TABLE_NUM) {
		/* We must make the kernel realize we had to move into
		 * promiscuous mode. If it was a promiscuous mode request
		 * the flag is already set. If not we set it.
		 */
		ndev->flags |= IFF_PROMISC;
		reg = axienet_ior(lp, XAE_FMC_OFFSET);
		reg |= XAE_FMC_PM_MASK;
		axienet_iow(lp, XAE_FMC_OFFSET, reg);
		dev_info(&ndev->dev, "Promiscuous mode enabled.\n");
	} else if (!netdev_mc_empty(ndev)) {
		struct netdev_hw_addr *ha;

		i = 0;
		netdev_for_each_mc_addr(ha, ndev) {
			if (i >= XAE_MULTICAST_CAM_TABLE_NUM)
				break;

			af0reg = (ha->addr[0]);
			af0reg |= (ha->addr[1] << 8);
			af0reg |= (ha->addr[2] << 16);
			af0reg |= (ha->addr[3] << 24);

			af1reg = (ha->addr[4]);
			af1reg |= (ha->addr[5] << 8);

			reg = axienet_ior(lp, XAE_FMC_OFFSET) & 0xFFFFFF00;
			reg |= i;

			axienet_iow(lp, XAE_FMC_OFFSET, reg);
			axienet_iow(lp, XAE_AF0_OFFSET, af0reg);
			axienet_iow(lp, XAE_AF1_OFFSET, af1reg);
			i++;
		}
	} else {
		reg = axienet_ior(lp, XAE_FMC_OFFSET);
		reg &= ~XAE_FMC_PM_MASK;

		axienet_iow(lp, XAE_FMC_OFFSET, reg);

		for (i = 0; i < XAE_MULTICAST_CAM_TABLE_NUM; i++) {
			reg = axienet_ior(lp, XAE_FMC_OFFSET) & 0xFFFFFF00;
			reg |= i;

			axienet_iow(lp, XAE_FMC_OFFSET, reg);
			axienet_iow(lp, XAE_AF0_OFFSET, 0);
			axienet_iow(lp, XAE_AF1_OFFSET, 0);
		}

		dev_info(&ndev->dev, "Promiscuous mode disabled.\n");
	}
}

/**
 * axienet_setoptions - Set an Axi Ethernet option
 * @ndev:	Pointer to the net_device structure
 * @options:	Option to be enabled/disabled
 *
 * The Axi Ethernet core has multiple features which can be selectively turned
 * on or off. The typical options could be jumbo frame option, basic VLAN
 * option, promiscuous mode option etc. This function is used to set or clear
 * these options in the Axi Ethernet hardware. This is done through
 * axienet_option structure .
 */
void axienet_setoptions(struct net_device *ndev, u32 options)
{
	int reg;
	struct axienet_local *lp = netdev_priv(ndev);
	struct axienet_option *tp = &axienet_options[0];

	while (tp->opt) {
		reg = ((axienet_ior(lp, tp->reg)) & ~(tp->m_or));
		if (options & tp->opt)
			reg |= tp->m_or;
		axienet_iow(lp, tp->reg, reg);
		tp++;
	}

	lp->options |= options;
}

static int __axienet_device_reset(struct axienet_local *lp)
{
	u32 value;
	int ret;

	/* Reset Axi DMA. This would reset Axi Ethernet core as well. The reset
	 * process of Axi DMA takes a while to complete as all pending
	 * commands/transfers will be flushed or completed during this
	 * reset process.
	 * Note that even though both TX and RX have their own reset register,
	 * they both reset the entire DMA core, so only one needs to be used.
	 */
	axienet_dma_out32(lp, XAXIDMA_TX_CR_OFFSET, XAXIDMA_CR_RESET_MASK);
	ret = read_poll_timeout(axienet_dma_in32, value,
				!(value & XAXIDMA_CR_RESET_MASK),
				DELAY_OF_ONE_MILLISEC, 50000, false, lp,
				XAXIDMA_TX_CR_OFFSET);
	if (ret) {
		dev_err(lp->dev, "%s: DMA reset timeout!\n", __func__);
		return ret;
	}

	/* Wait for PhyRstCmplt bit to be set, indicating the PHY reset has finished */
	ret = read_poll_timeout(axienet_ior, value,
				value & XAE_INT_PHYRSTCMPLT_MASK,
				DELAY_OF_ONE_MILLISEC, 50000, false, lp,
				XAE_IS_OFFSET);
	if (ret) {
		dev_err(lp->dev, "%s: timeout waiting for PhyRstCmplt\n", __func__);
		return ret;
	}

	return 0;
}

/**
 * axienet_device_reset - Reset and initialize the Axi Ethernet hardware.
 * @ndev:	Pointer to the net_device structure
 *
 * This function is called to reset and initialize the Axi Ethernet core. This
 * is typically called during initialization. It does a reset of the Axi DMA
 * Rx/Tx channels and initializes the Axi DMA BDs. Since Axi DMA reset lines
 * areconnected to Axi Ethernet reset lines, this in turn resets the Axi
 * Ethernet core. No separate hardware reset is done for the Axi Ethernet
 * core.
 * Returns 0 on success or a negative error number otherwise.
 */
static int axienet_device_reset(struct net_device *ndev)
{
	u32 axienet_status;
	struct axienet_local *lp = netdev_priv(ndev);
	int ret;

	ret = __axienet_device_reset(lp);
	if (ret)
		return ret;

	lp->max_frm_size = XAE_MAX_VLAN_FRAME_SIZE;
	lp->options |= XAE_OPTION_VLAN;
	lp->options &= (~XAE_OPTION_JUMBO);

	if ((ndev->mtu > XAE_MTU) &&
		(ndev->mtu <= XAE_JUMBO_MTU)) {
		lp->max_frm_size = ndev->mtu + VLAN_ETH_HLEN +
					XAE_TRL_SIZE;

		if (lp->max_frm_size <= lp->rxmem)
			lp->options |= XAE_OPTION_JUMBO;
	}

	ret = axienet_dma_bd_init(ndev);
	if (ret) {
		netdev_err(ndev, "%s: descriptor allocation failed\n",
			   __func__);
		return ret;
	}

	axienet_status = axienet_ior(lp, XAE_RCW1_OFFSET);
	axienet_status &= ~XAE_RCW1_RX_MASK;
	axienet_iow(lp, XAE_RCW1_OFFSET, axienet_status);

	axienet_status = axienet_ior(lp, XAE_IP_OFFSET);
	if (axienet_status & XAE_INT_RXRJECT_MASK)
		axienet_iow(lp, XAE_IS_OFFSET, XAE_INT_RXRJECT_MASK);
	axienet_iow(lp, XAE_IE_OFFSET, lp->eth_irq > 0 ?
		    XAE_INT_RECV_ERROR_MASK : 0);

	axienet_iow(lp, XAE_FCC_OFFSET, XAE_FCC_FCRX_MASK);

	/* Sync default options with HW but leave receiver and
	 * transmitter disabled.
	 */
	axienet_setoptions(ndev, lp->options &
			   ~(XAE_OPTION_TXEN | XAE_OPTION_RXEN));
	axienet_set_mac_address(ndev, NULL);
	axienet_set_multicast_list(ndev);
	axienet_setoptions(ndev, lp->options);

	netif_trans_update(ndev);

	return 0;
}

/**
 * axienet_free_tx_chain - Clean up a series of linked TX descriptors.
 * @ndev:	Pointer to the net_device structure
 * @first_bd:	Index of first descriptor to clean up
 * @nr_bds:	Number of descriptors to clean up, can be -1 if unknown.
 * @sizep:	Pointer to a u32 filled with the total sum of all bytes
 * 		in all cleaned-up descriptors. Ignored if NULL.
 *
 * Would either be called after a successful transmit operation, or after
 * there was an error when setting up the chain.
 * Returns the number of descriptors handled.
 */
static int axienet_free_tx_chain(struct net_device *ndev, u32 first_bd,
				 int nr_bds, u32 *sizep)
{
	struct axienet_local *lp = netdev_priv(ndev);
	struct axidma_bd *cur_p;
	int max_bds = nr_bds;
	unsigned int status;
	dma_addr_t phys;
	int i;

	if (max_bds == -1)
		max_bds = lp->tx_bd_num;

	for (i = 0; i < max_bds; i++) {
		cur_p = &lp->tx_bd_v[(first_bd + i) % lp->tx_bd_num];
		status = cur_p->status;

		/* If no number is given, clean up *all* descriptors that have
		 * been completed by the MAC.
		 */
		if (nr_bds == -1 && !(status & XAXIDMA_BD_STS_COMPLETE_MASK))
			break;

		/* Ensure we see complete descriptor update */
		dma_rmb();
		phys = desc_get_phys_addr(lp, cur_p);
		dma_unmap_single(ndev->dev.parent, phys,
				 (cur_p->cntrl & XAXIDMA_BD_CTRL_LENGTH_MASK),
				 DMA_TO_DEVICE);

		if (cur_p->skb && (status & XAXIDMA_BD_STS_COMPLETE_MASK))
			dev_consume_skb_irq(cur_p->skb);

		cur_p->app0 = 0;
		cur_p->app1 = 0;
		cur_p->app2 = 0;
		cur_p->app4 = 0;
		cur_p->skb = NULL;
		/* ensure our transmit path and device don't prematurely see status cleared */
		wmb();
		cur_p->cntrl = 0;
		cur_p->status = 0;

		if (sizep)
			*sizep += status & XAXIDMA_BD_STS_ACTUAL_LEN_MASK;
	}

	return i;
}

/**
 * axienet_check_tx_bd_space - Checks if a BD/group of BDs are currently busy
 * @lp:		Pointer to the axienet_local structure
 * @num_frag:	The number of BDs to check for
 *
 * Return: 0, on success
 *	    NETDEV_TX_BUSY, if any of the descriptors are not free
 *
 * This function is invoked before BDs are allocated and transmission starts.
 * This function returns 0 if a BD or group of BDs can be allocated for
 * transmission. If the BD or any of the BDs are not free the function
 * returns a busy status. This is invoked from axienet_start_xmit.
 */
static inline int axienet_check_tx_bd_space(struct axienet_local *lp,
					    int num_frag)
{
	struct axidma_bd *cur_p;

	/* Ensure we see all descriptor updates from device or TX IRQ path */
	rmb();
	cur_p = &lp->tx_bd_v[(lp->tx_bd_tail + num_frag) % lp->tx_bd_num];
	if (cur_p->cntrl)
		return NETDEV_TX_BUSY;
	return 0;
}

/**
 * axienet_start_xmit_done - Invoked once a transmit is completed by the
 * Axi DMA Tx channel.
 * @ndev:	Pointer to the net_device structure
 *
 * This function is invoked from the Axi DMA Tx isr to notify the completion
 * of transmit operation. It clears fields in the corresponding Tx BDs and
 * unmaps the corresponding buffer so that CPU can regain ownership of the
 * buffer. It finally invokes "netif_wake_queue" to restart transmission if
 * required.
 */
static void axienet_start_xmit_done(struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);
	u32 packets = 0;
	u32 size = 0;

	packets = axienet_free_tx_chain(ndev, lp->tx_bd_ci, -1, &size);

	lp->tx_bd_ci += packets;
	if (lp->tx_bd_ci >= lp->tx_bd_num)
		lp->tx_bd_ci -= lp->tx_bd_num;

	ndev->stats.tx_packets += packets;
	ndev->stats.tx_bytes += size;

	/* Matches barrier in axienet_start_xmit */
	smp_mb();

	if (!axienet_check_tx_bd_space(lp, MAX_SKB_FRAGS + 1))
		netif_wake_queue(ndev);
}

/**
 * axienet_start_xmit - Starts the transmission.
 * @skb:	sk_buff pointer that contains data to be Txed.
 * @ndev:	Pointer to net_device structure.
 *
 * Return: NETDEV_TX_OK, on success
 *	    NETDEV_TX_BUSY, if any of the descriptors are not free
 *
 * This function is invoked from upper layers to initiate transmission. The
 * function uses the next available free BDs and populates their fields to
 * start the transmission. Additionally if checksum offloading is supported,
 * it populates AXI Stream Control fields with appropriate values.
 */
static netdev_tx_t
axienet_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	u32 ii;
	u32 num_frag;
	u32 csum_start_off;
	u32 csum_index_off;
	skb_frag_t *frag;
	dma_addr_t tail_p, phys;
	struct axienet_local *lp = netdev_priv(ndev);
	struct axidma_bd *cur_p;
	u32 orig_tail_ptr = lp->tx_bd_tail;

	num_frag = skb_shinfo(skb)->nr_frags;
	cur_p = &lp->tx_bd_v[lp->tx_bd_tail];

	if (axienet_check_tx_bd_space(lp, num_frag + 1)) {
		/* Should not happen as last start_xmit call should have
		 * checked for sufficient space and queue should only be
		 * woken when sufficient space is available.
		 */
		netif_stop_queue(ndev);
		if (net_ratelimit())
			netdev_warn(ndev, "TX ring unexpectedly full\n");
		return NETDEV_TX_BUSY;
	}

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		if (lp->features & XAE_FEATURE_FULL_TX_CSUM) {
			/* Tx Full Checksum Offload Enabled */
			cur_p->app0 |= 2;
		} else if (lp->features & XAE_FEATURE_PARTIAL_RX_CSUM) {
			csum_start_off = skb_transport_offset(skb);
			csum_index_off = csum_start_off + skb->csum_offset;
			/* Tx Partial Checksum Offload Enabled */
			cur_p->app0 |= 1;
			cur_p->app1 = (csum_start_off << 16) | csum_index_off;
		}
	} else if (skb->ip_summed == CHECKSUM_UNNECESSARY) {
		cur_p->app0 |= 2; /* Tx Full Checksum Offload Enabled */
	}

	phys = dma_map_single(ndev->dev.parent, skb->data,
			      skb_headlen(skb), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(ndev->dev.parent, phys))) {
		if (net_ratelimit())
			netdev_err(ndev, "TX DMA mapping error\n");
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}
	desc_set_phys_addr(lp, phys, cur_p);
	cur_p->cntrl = skb_headlen(skb) | XAXIDMA_BD_CTRL_TXSOF_MASK;

	for (ii = 0; ii < num_frag; ii++) {
		if (++lp->tx_bd_tail >= lp->tx_bd_num)
			lp->tx_bd_tail = 0;
		cur_p = &lp->tx_bd_v[lp->tx_bd_tail];
		frag = &skb_shinfo(skb)->frags[ii];
		phys = dma_map_single(ndev->dev.parent,
				      skb_frag_address(frag),
				      skb_frag_size(frag),
				      DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(ndev->dev.parent, phys))) {
			if (net_ratelimit())
				netdev_err(ndev, "TX DMA mapping error\n");
			ndev->stats.tx_dropped++;
			axienet_free_tx_chain(ndev, orig_tail_ptr, ii + 1,
					      NULL);
			lp->tx_bd_tail = orig_tail_ptr;

			return NETDEV_TX_OK;
		}
		desc_set_phys_addr(lp, phys, cur_p);
		cur_p->cntrl = skb_frag_size(frag);
	}

	cur_p->cntrl |= XAXIDMA_BD_CTRL_TXEOF_MASK;
	cur_p->skb = skb;

	tail_p = lp->tx_bd_p + sizeof(*lp->tx_bd_v) * lp->tx_bd_tail;
	/* Start the transfer */
	axienet_dma_out_addr(lp, XAXIDMA_TX_TDESC_OFFSET, tail_p);
	if (++lp->tx_bd_tail >= lp->tx_bd_num)
		lp->tx_bd_tail = 0;

	/* Stop queue if next transmit may not have space */
	if (axienet_check_tx_bd_space(lp, MAX_SKB_FRAGS + 1)) {
		netif_stop_queue(ndev);

		/* Matches barrier in axienet_start_xmit_done */
		smp_mb();

		/* Space might have just been freed - check again */
		if (!axienet_check_tx_bd_space(lp, MAX_SKB_FRAGS + 1))
			netif_wake_queue(ndev);
	}

	return NETDEV_TX_OK;
}

/**
 * axienet_recv - Is called from Axi DMA Rx Isr to complete the received
 *		  BD processing.
 * @ndev:	Pointer to net_device structure.
 *
 * This function is invoked from the Axi DMA Rx isr to process the Rx BDs. It
 * does minimal processing and invokes "netif_rx" to complete further
 * processing.
 */
static void axienet_recv(struct net_device *ndev)
{
	u32 length;
	u32 csumstatus;
	u32 size = 0;
	u32 packets = 0;
	dma_addr_t tail_p = 0;
	struct axienet_local *lp = netdev_priv(ndev);
	struct sk_buff *skb, *new_skb;
	struct axidma_bd *cur_p;

	cur_p = &lp->rx_bd_v[lp->rx_bd_ci];

	while ((cur_p->status & XAXIDMA_BD_STS_COMPLETE_MASK)) {
		dma_addr_t phys;

		/* Ensure we see complete descriptor update */
		dma_rmb();

		skb = cur_p->skb;
		cur_p->skb = NULL;

		/* skb could be NULL if a previous pass already received the
		 * packet for this slot in the ring, but failed to refill it
		 * with a newly allocated buffer. In this case, don't try to
		 * receive it again.
		 */
		if (likely(skb)) {
			length = cur_p->app4 & 0x0000FFFF;

			phys = desc_get_phys_addr(lp, cur_p);
			dma_unmap_single(ndev->dev.parent, phys, lp->max_frm_size,
					 DMA_FROM_DEVICE);

			skb_put(skb, length);
			skb->protocol = eth_type_trans(skb, ndev);
			/*skb_checksum_none_assert(skb);*/
			skb->ip_summed = CHECKSUM_NONE;

			/* if we're doing Rx csum offload, set it up */
			if (lp->features & XAE_FEATURE_FULL_RX_CSUM) {
				csumstatus = (cur_p->app2 &
					      XAE_FULL_CSUM_STATUS_MASK) >> 3;
				if (csumstatus == XAE_IP_TCP_CSUM_VALIDATED ||
				    csumstatus == XAE_IP_UDP_CSUM_VALIDATED) {
					skb->ip_summed = CHECKSUM_UNNECESSARY;
				}
			} else if ((lp->features & XAE_FEATURE_PARTIAL_RX_CSUM) != 0 &&
				   skb->protocol == htons(ETH_P_IP) &&
				   skb->len > 64) {
				skb->csum = be32_to_cpu(cur_p->app3 & 0xFFFF);
				skb->ip_summed = CHECKSUM_COMPLETE;
			}

			netif_rx(skb);

			size += length;
			packets++;
		}

		new_skb = netdev_alloc_skb_ip_align(ndev, lp->max_frm_size);
		if (!new_skb)
			break;

		phys = dma_map_single(ndev->dev.parent, new_skb->data,
				      lp->max_frm_size,
				      DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(ndev->dev.parent, phys))) {
			if (net_ratelimit())
				netdev_err(ndev, "RX DMA mapping error\n");
			dev_kfree_skb(new_skb);
			break;
		}
		desc_set_phys_addr(lp, phys, cur_p);

		cur_p->cntrl = lp->max_frm_size;
		cur_p->status = 0;
		cur_p->skb = new_skb;

		/* Only update tail_p to mark this slot as usable after it has
		 * been successfully refilled.
		 */
		tail_p = lp->rx_bd_p + sizeof(*lp->rx_bd_v) * lp->rx_bd_ci;

		if (++lp->rx_bd_ci >= lp->rx_bd_num)
			lp->rx_bd_ci = 0;
		cur_p = &lp->rx_bd_v[lp->rx_bd_ci];
	}

	ndev->stats.rx_packets += packets;
	ndev->stats.rx_bytes += size;

	if (tail_p)
		axienet_dma_out_addr(lp, XAXIDMA_RX_TDESC_OFFSET, tail_p);
}

/**
 * axienet_tx_irq - Tx Done Isr.
 * @irq:	irq number
 * @_ndev:	net_device pointer
 *
 * Return: IRQ_HANDLED if device generated a TX interrupt, IRQ_NONE otherwise.
 *
 * This is the Axi DMA Tx done Isr. It invokes "axienet_start_xmit_done"
 * to complete the BD processing.
 */
static irqreturn_t axienet_tx_irq(int irq, void *_ndev)
{
	u32 cr;
	unsigned int status;
	struct net_device *ndev = _ndev;
	struct axienet_local *lp = netdev_priv(ndev);

	status = axienet_dma_in32(lp, XAXIDMA_TX_SR_OFFSET);
	if (status & (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK)) {
		axienet_dma_out32(lp, XAXIDMA_TX_SR_OFFSET, status);
		axienet_start_xmit_done(lp->ndev);
		goto out;
	}
	if (!(status & XAXIDMA_IRQ_ALL_MASK))
		return IRQ_NONE;
	if (status & XAXIDMA_IRQ_ERROR_MASK) {
		dev_err(&ndev->dev, "DMA Tx error 0x%x\n", status);
		dev_err(&ndev->dev, "Current BD is at: 0x%x%08x\n",
			(lp->tx_bd_v[lp->tx_bd_ci]).phys_msb,
			(lp->tx_bd_v[lp->tx_bd_ci]).phys);

		cr = axienet_dma_in32(lp, XAXIDMA_TX_CR_OFFSET);
		/* Disable coalesce, delay timer and error interrupts */
		cr &= (~XAXIDMA_IRQ_ALL_MASK);
		/* Write to the Tx channel control register */
		axienet_dma_out32(lp, XAXIDMA_TX_CR_OFFSET, cr);

		cr = axienet_dma_in32(lp, XAXIDMA_RX_CR_OFFSET);
		/* Disable coalesce, delay timer and error interrupts */
		cr &= (~XAXIDMA_IRQ_ALL_MASK);
		/* Write to the Rx channel control register */
		axienet_dma_out32(lp, XAXIDMA_RX_CR_OFFSET, cr);

		schedule_work(&lp->dma_err_task);
		axienet_dma_out32(lp, XAXIDMA_TX_SR_OFFSET, status);
	}
out:
	return IRQ_HANDLED;
}

/**
 * axienet_rx_irq - Rx Isr.
 * @irq:	irq number
 * @_ndev:	net_device pointer
 *
 * Return: IRQ_HANDLED if device generated a RX interrupt, IRQ_NONE otherwise.
 *
 * This is the Axi DMA Rx Isr. It invokes "axienet_recv" to complete the BD
 * processing.
 */
static irqreturn_t axienet_rx_irq(int irq, void *_ndev)
{
	u32 cr;
	unsigned int status;
	struct net_device *ndev = _ndev;
	struct axienet_local *lp = netdev_priv(ndev);

	status = axienet_dma_in32(lp, XAXIDMA_RX_SR_OFFSET);
	if (status & (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK)) {
		axienet_dma_out32(lp, XAXIDMA_RX_SR_OFFSET, status);
		axienet_recv(lp->ndev);
		goto out;
	}
	if (!(status & XAXIDMA_IRQ_ALL_MASK))
		return IRQ_NONE;
	if (status & XAXIDMA_IRQ_ERROR_MASK) {
		dev_err(&ndev->dev, "DMA Rx error 0x%x\n", status);
		dev_err(&ndev->dev, "Current BD is at: 0x%x%08x\n",
			(lp->rx_bd_v[lp->rx_bd_ci]).phys_msb,
			(lp->rx_bd_v[lp->rx_bd_ci]).phys);

		cr = axienet_dma_in32(lp, XAXIDMA_TX_CR_OFFSET);
		/* Disable coalesce, delay timer and error interrupts */
		cr &= (~XAXIDMA_IRQ_ALL_MASK);
		/* Finally write to the Tx channel control register */
		axienet_dma_out32(lp, XAXIDMA_TX_CR_OFFSET, cr);

		cr = axienet_dma_in32(lp, XAXIDMA_RX_CR_OFFSET);
		/* Disable coalesce, delay timer and error interrupts */
		cr &= (~XAXIDMA_IRQ_ALL_MASK);
		/* write to the Rx channel control register */
		axienet_dma_out32(lp, XAXIDMA_RX_CR_OFFSET, cr);

		schedule_work(&lp->dma_err_task);
		axienet_dma_out32(lp, XAXIDMA_RX_SR_OFFSET, status);
	}
out:
	return IRQ_HANDLED;
}

/**
 * axienet_eth_irq - Ethernet core Isr.
 * @irq:	irq number
 * @_ndev:	net_device pointer
 *
 * Return: IRQ_HANDLED if device generated a core interrupt, IRQ_NONE otherwise.
 *
 * Handle miscellaneous conditions indicated by Ethernet core IRQ.
 */
static irqreturn_t axienet_eth_irq(int irq, void *_ndev)
{
	struct net_device *ndev = _ndev;
	struct axienet_local *lp = netdev_priv(ndev);
	unsigned int pending;

	pending = axienet_ior(lp, XAE_IP_OFFSET);
	if (!pending)
		return IRQ_NONE;

	if (pending & XAE_INT_RXFIFOOVR_MASK)
		ndev->stats.rx_missed_errors++;

	if (pending & XAE_INT_RXRJECT_MASK)
		ndev->stats.rx_frame_errors++;

	axienet_iow(lp, XAE_IS_OFFSET, pending);
	return IRQ_HANDLED;
}

/**
 * axienet_open - Driver open routine.
 * @ndev:	Pointer to net_device structure
 *
 * Return: 0, on success.
 *	    non-zero error value on failure
 *
 * This is the driver open routine. It calls phy_start to start the PHY device.
 * It also allocates interrupt service routines, enables the interrupt lines
 * and ISR handling. Axi Ethernet core is reset through Axi DMA core. Buffer
 * descriptors are initialized.
 */
static int axienet_open(struct net_device *ndev)
{
	int ret = 0, i = 0;
	struct axienet_local *lp = netdev_priv(ndev);
	struct axienet_dma_q *q;
	u32 reg, err;
	struct phy_device *phydev = NULL;

	dev_dbg(&ndev->dev, "axienet_open()\n");

	ret  = axienet_device_reset(ndev);
	if (ret < 0) {
		dev_err(lp->dev, "axienet_device_reset failed\n");
		return ret;
	}

	if (lp->phy_node) {
		phydev = of_phy_connect(lp->ndev, lp->phy_node,
					axienet_adjust_link,
					lp->phy_flags,
					lp->phy_mode);

		if (!phydev)
			dev_err(lp->dev, "of_phy_connect() failed\n");
		else
			phy_start(phydev);
	}
	if (!lp->is_tsn) {
		/* Enable tasklets for Axi DMA error handling */
		for_each_rx_dma_queue(lp, i) {
#ifdef CONFIG_AXIENET_HAS_MCDMA
			tasklet_init(&lp->dma_err_tasklet[i],
				     axienet_mcdma_err_handler,
				     (unsigned long)lp->dq[i]);
#else
			tasklet_init(&lp->dma_err_tasklet[i],
				     axienet_dma_err_handler,
				     (unsigned long)lp->dq[i]);
#endif

			/* Enable NAPI scheduling before enabling Axi DMA Rx
			 * IRQ, or you might run into a race condition; the RX
			 * ISR disables IRQ processing before scheduling the
			 * NAPI function to complete the processing. If NAPI
			 * scheduling is (still) disabled at that time, no more
			 * RX IRQs will be processed as only the NAPI function
			 * re-enables them!
			 */
			napi_enable(&lp->napi[i]);
		}
		for_each_tx_dma_queue(lp, i) {
			struct axienet_dma_q *q = lp->dq[i];
#ifdef CONFIG_AXIENET_HAS_MCDMA
			/* Enable interrupts for Axi MCDMA Tx */
			ret = request_irq(q->tx_irq, axienet_mcdma_tx_irq,
					  IRQF_SHARED, ndev->name, ndev);
			if (ret)
				goto err_tx_irq;
#else
			/* Enable interrupts for Axi DMA Tx */
			ret = request_irq(q->tx_irq, axienet_tx_irq,
					  0, ndev->name, ndev);
			if (ret)
				goto err_tx_irq;
#endif
		}

		for_each_rx_dma_queue(lp, i) {
			struct axienet_dma_q *q = lp->dq[i];
#ifdef CONFIG_AXIENET_HAS_MCDMA
			/* Enable interrupts for Axi MCDMA Rx */
			ret = request_irq(q->rx_irq, axienet_mcdma_rx_irq,
					  IRQF_SHARED, ndev->name, ndev);
			if (ret)
				goto err_rx_irq;
#else
			/* Enable interrupts for Axi DMA Rx */
			ret = request_irq(q->rx_irq, axienet_rx_irq,
					  0, ndev->name, ndev);
			if (ret)
				goto err_rx_irq;
#endif
		}
	}

	if (lp->phy_mode == PHY_INTERFACE_MODE_USXGMII) {
		netdev_dbg(ndev, "RX reg: 0x%x\n",
			   axienet_ior(lp, XXV_RCW1_OFFSET));
		/* USXGMII setup at selected speed */
		reg = axienet_ior(lp, XXV_USXGMII_AN_OFFSET);
		reg &= ~USXGMII_RATE_MASK;
		netdev_dbg(ndev, "usxgmii_rate %d\n", lp->usxgmii_rate);
		switch (lp->usxgmii_rate) {
		case SPEED_1000:
			reg |= USXGMII_RATE_1G;
			break;
		case SPEED_2500:
			reg |= USXGMII_RATE_2G5;
			break;
		case SPEED_10:
			reg |= USXGMII_RATE_10M;
			break;
		case SPEED_100:
			reg |= USXGMII_RATE_100M;
			break;
		case SPEED_5000:
			reg |= USXGMII_RATE_5G;
			break;
		case SPEED_10000:
			reg |= USXGMII_RATE_10G;
			break;
		default:
			reg |= USXGMII_RATE_1G;
		}
		reg |= USXGMII_FD;
		reg |= (USXGMII_EN | USXGMII_LINK_STS);
		axienet_iow(lp, XXV_USXGMII_AN_OFFSET, reg);
		reg |= USXGMII_AN_EN;
		axienet_iow(lp, XXV_USXGMII_AN_OFFSET, reg);
		/* AN Restart bit should be reset, set and then reset as per
		 * spec with a 1 ms delay for a raising edge trigger
		 */
		axienet_iow(lp, XXV_USXGMII_AN_OFFSET,
			    reg & ~USXGMII_AN_RESTART);
		mdelay(1);
		axienet_iow(lp, XXV_USXGMII_AN_OFFSET,
			    reg | USXGMII_AN_RESTART);
		mdelay(1);
		axienet_iow(lp, XXV_USXGMII_AN_OFFSET,
			    reg & ~USXGMII_AN_RESTART);

		/* Check block lock bit to make sure RX path is ok with
		 * USXGMII initialization.
		 */
		err = readl_poll_timeout(lp->regs + XXV_STATRX_BLKLCK_OFFSET,
					 reg, (reg & XXV_RX_BLKLCK_MASK),
					 100, DELAY_OF_ONE_MILLISEC);
		if (err) {
			netdev_err(ndev, "%s: USXGMII Block lock bit not set",
				   __func__);
			ret = -ENODEV;
			goto err_eth_irq;
		}

		err = readl_poll_timeout(lp->regs + XXV_USXGMII_AN_STS_OFFSET,
					 reg, (reg & USXGMII_AN_STS_COMP_MASK),
					 1000000, DELAY_OF_ONE_MILLISEC);
		if (err) {
			netdev_err(ndev, "%s: USXGMII AN not complete",
				   __func__);
			ret = -ENODEV;
			goto err_eth_irq;
		}

		netdev_info(ndev, "USXGMII setup at %d\n", lp->usxgmii_rate);
	}

	if (lp->axienet_config->mactype == XAXIENET_MRMAC) {
		u32 val;

		/* Reset MRMAC */
		axienet_mrmac_reset(lp);

		mdelay(MRMAC_RESET_DELAY);
		/* Check for block lock bit to be set. This ensures that
		 * MRMAC ethernet IP is functioning normally.
		 */
		axienet_iow(lp, MRMAC_TX_STS_OFFSET, MRMAC_STS_ALL_MASK);
		axienet_iow(lp, MRMAC_RX_STS_OFFSET, MRMAC_STS_ALL_MASK);
		err = readx_poll_timeout(axienet_get_mrmac_blocklock, lp, val,
					 (val & MRMAC_RX_BLKLCK_MASK), 10, DELAY_OF_ONE_MILLISEC);
		if (err) {
			netdev_err(ndev, "MRMAC block lock not complete! Cross-check the MAC ref clock configuration\n");
			ret = -ENODEV;
			goto err_eth_irq;
		}
		netdev_info(ndev, "MRMAC setup at %d\n", lp->mrmac_rate);
		axienet_iow(lp, MRMAC_TICK_OFFSET, MRMAC_TICK_TRIGGER);
	}

	/* Enable interrupts for Axi Ethernet core (if defined) */
	if (!lp->eth_hasnobuf && (lp->axienet_config->mactype == XAXIENET_1G)) {
		ret = request_irq(lp->eth_irq, axienet_eth_irq, IRQF_SHARED,
				  ndev->name, ndev);
		if (ret)
			goto err_eth_irq;
	}

	netif_tx_start_all_queues(ndev);
	return 0;

err_eth_irq:
	while (i--) {
		q = lp->dq[i];
		free_irq(q->rx_irq, ndev);
	}
	i = lp->num_tx_queues;
err_rx_irq:
	while (i--) {
		q = lp->dq[i];
		free_irq(q->tx_irq, ndev);
	}
err_tx_irq:
	for_each_rx_dma_queue(lp, i)
		napi_disable(&lp->napi[i]);
	if (phydev)
		phy_disconnect(phydev);
	for_each_rx_dma_queue(lp, i)
		tasklet_kill(&lp->dma_err_tasklet[i]);
	dev_err(lp->dev, "request_irq() failed\n");
	return ret;
}

/**
 * axienet_stop - Driver stop routine.
 * @ndev:	Pointer to net_device structure
 *
 * Return: 0, on success.
 *
 * This is the driver stop routine. It calls phy_disconnect to stop the PHY
 * device. It also removes the interrupt handlers and disables the interrupts.
 * The Axi DMA Tx/Rx BDs are released.
 */
static int axienet_stop(struct net_device *ndev)
{
	u32 cr, sr;
	int count;
	u32 i;
	struct axienet_local *lp = netdev_priv(ndev);
	struct axienet_dma_q *q;

	dev_dbg(&ndev->dev, "axienet_close()\n");

	lp->axienet_config->setoptions(ndev, lp->options &
			   ~(XAE_OPTION_TXEN | XAE_OPTION_RXEN));

	if (!lp->is_tsn) {
		for_each_tx_dma_queue(lp, i) {
			q = lp->dq[i];
			cr = axienet_dma_in32(q, XAXIDMA_RX_CR_OFFSET);
			cr &= ~(XAXIDMA_CR_RUNSTOP_MASK | XAXIDMA_IRQ_ALL_MASK);
			axienet_dma_out32(q, XAXIDMA_RX_CR_OFFSET, cr);

			cr = axienet_dma_in32(q, XAXIDMA_TX_CR_OFFSET);
			cr &= ~(XAXIDMA_CR_RUNSTOP_MASK | XAXIDMA_IRQ_ALL_MASK);
			axienet_dma_out32(q, XAXIDMA_TX_CR_OFFSET, cr);

			axienet_iow(lp, XAE_IE_OFFSET, 0);

			/* Give DMAs a chance to halt gracefully */
			sr = axienet_dma_in32(q, XAXIDMA_RX_SR_OFFSET);
			for (count = 0; !(sr & XAXIDMA_SR_HALT_MASK) && count < 5; ++count) {
				msleep(20);
				sr = axienet_dma_in32(q, XAXIDMA_RX_SR_OFFSET);
			}

			sr = axienet_dma_in32(q, XAXIDMA_TX_SR_OFFSET);
			for (count = 0; !(sr & XAXIDMA_SR_HALT_MASK) && count < 5; ++count) {
				msleep(20);
				sr = axienet_dma_in32(q, XAXIDMA_TX_SR_OFFSET);
			}

			__axienet_device_reset(q);
			free_irq(q->tx_irq, ndev);
		}

		for_each_rx_dma_queue(lp, i) {
			q = lp->dq[i];
			netif_stop_queue(ndev);
			napi_disable(&lp->napi[i]);
			tasklet_kill(&lp->dma_err_tasklet[i]);
			free_irq(q->rx_irq, ndev);
		}
#ifdef CONFIG_XILINX_TSN_PTP
		if (lp->is_tsn) {
			free_irq(lp->ptp_tx_irq, ndev);
			free_irq(lp->ptp_rx_irq, ndev);
		}
#endif
		if ((lp->axienet_config->mactype == XAXIENET_1G) && !lp->eth_hasnobuf)
			free_irq(lp->eth_irq, ndev);

		if (ndev->phydev)
			phy_disconnect(ndev->phydev);

		if (!lp->is_tsn)
			axienet_dma_bd_release(ndev);
	}
	return 0;
}

/**
 * axienet_change_mtu - Driver change mtu routine.
 * @ndev:	Pointer to net_device structure
 * @new_mtu:	New mtu value to be applied
 *
 * Return: Always returns 0 (success).
 *
 * This is the change mtu driver routine. It checks if the Axi Ethernet
 * hardware supports jumbo frames before changing the mtu. This can be
 * called only when the device is not up.
 */
static int axienet_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct axienet_local *lp = netdev_priv(ndev);

	if (netif_running(ndev))
		return -EBUSY;

	if ((new_mtu + VLAN_ETH_HLEN +
		XAE_TRL_SIZE) > lp->rxmem)
		return -EINVAL;

	ndev->mtu = new_mtu;

	return 0;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/**
 * axienet_poll_controller - Axi Ethernet poll mechanism.
 * @ndev:	Pointer to net_device structure
 *
 * This implements Rx/Tx ISR poll mechanisms. The interrupts are disabled prior
 * to polling the ISRs and are enabled back after the polling is done.
 */
static void axienet_poll_controller(struct net_device *ndev)
{
	struct axienet_local *lp = netdev_priv(ndev);
	int i;

	for_each_tx_dma_queue(lp, i)
		disable_irq(lp->dq[i]->tx_irq);
	for_each_rx_dma_queue(lp, i)
		disable_irq(lp->dq[i]->rx_irq);

	for_each_rx_dma_queue(lp, i)
#ifdef CONFIG_AXIENET_HAS_MCDMA
		axienet_mcdma_rx_irq(lp->dq[i]->rx_irq, ndev);
#else
		axienet_rx_irq(lp->dq[i]->rx_irq, ndev);
#endif
	for_each_tx_dma_queue(lp, i)
#ifdef CONFIG_AXIENET_HAS_MCDMA
		axienet_mcdma_tx_irq(lp->dq[i]->tx_irq, ndev);
#else
		axienet_tx_irq(lp->dq[i]->tx_irq, ndev);
#endif
	for_each_tx_dma_queue(lp, i)
		enable_irq(lp->dq[i]->tx_irq);
	for_each_rx_dma_queue(lp, i)
		enable_irq(lp->dq[i]->rx_irq);
}
#endif

#if defined(CONFIG_XILINX_AXI_EMAC_HWTSTAMP) || defined(CONFIG_XILINX_TSN_PTP)
/**
 *  axienet_set_timestamp_mode - sets up the hardware for the requested mode
 *  @lp: Pointer to axienet local structure
 *  @config: the hwtstamp configuration requested
 *
 * Return: 0 on success, Negative value on errors
 */
static int axienet_set_timestamp_mode(struct axienet_local *lp,
				      struct hwtstamp_config *config)
{
	u32 regval;

#ifdef CONFIG_XILINX_TSN_PTP
	if (lp->is_tsn) {
		/* reserved for future extensions */
		if (config->flags)
			return -EINVAL;

		if (config->tx_type < HWTSTAMP_TX_OFF ||
		    config->tx_type > HWTSTAMP_TX_ONESTEP_SYNC)
			return -ERANGE;

		lp->ptp_ts_type = config->tx_type;

		/* On RX always timestamp everything */
		switch (config->rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			break;
		default:
			config->rx_filter = HWTSTAMP_FILTER_ALL;
		}
		return 0;
	}
#endif

	/* reserved for future extensions */
	if (config->flags)
		return -EINVAL;

	/* Read the current value in the MAC TX CTRL register */
	if (lp->axienet_config->mactype != XAXIENET_10G_25G &&
	    lp->axienet_config->mactype != XAXIENET_MRMAC)
		regval = axienet_ior(lp, XAE_TC_OFFSET);

	switch (config->tx_type) {
	case HWTSTAMP_TX_OFF:
		regval &= ~XAE_TC_INBAND1588_MASK;
		break;
	case HWTSTAMP_TX_ON:
		config->tx_type = HWTSTAMP_TX_ON;
		regval |= XAE_TC_INBAND1588_MASK;
		if (lp->axienet_config->mactype == XAXIENET_MRMAC)
			axienet_iow(lp, MRMAC_CFG1588_OFFSET, 0x0);
		break;
	case HWTSTAMP_TX_ONESTEP_SYNC:
		config->tx_type = HWTSTAMP_TX_ONESTEP_SYNC;
		regval |= XAE_TC_INBAND1588_MASK;
		if (lp->axienet_config->mactype == XAXIENET_MRMAC)
			axienet_iow(lp, MRMAC_CFG1588_OFFSET, MRMAC_ONE_STEP_EN);
		break;
	case HWTSTAMP_TX_ONESTEP_P2P:
		if (lp->axienet_config->mactype == XAXIENET_MRMAC) {
			config->tx_type = HWTSTAMP_TX_ONESTEP_P2P;
			axienet_iow(lp, MRMAC_CFG1588_OFFSET, MRMAC_ONE_STEP_EN);
		} else {
			return -ERANGE;
		}
		break;
	default:
		return -ERANGE;
	}

	if (lp->axienet_config->mactype != XAXIENET_10G_25G &&
	    lp->axienet_config->mactype != XAXIENET_MRMAC)
		axienet_iow(lp, XAE_TC_OFFSET, regval);

	/* Read the current value in the MAC RX RCW1 register */
	if (lp->axienet_config->mactype != XAXIENET_10G_25G &&
	    lp->axienet_config->mactype != XAXIENET_MRMAC)
		regval = axienet_ior(lp, XAE_RCW1_OFFSET);

	/* On RX always timestamp everything */
	switch (config->rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		regval &= ~XAE_RCW1_INBAND1588_MASK;
		break;
	default:
		config->rx_filter = HWTSTAMP_FILTER_ALL;
		regval |= XAE_RCW1_INBAND1588_MASK;
	}

	if (lp->axienet_config->mactype != XAXIENET_10G_25G &&
	    lp->axienet_config->mactype != XAXIENET_MRMAC)
		axienet_iow(lp, XAE_RCW1_OFFSET, regval);

	return 0;
}

/**
 * axienet_set_ts_config - user entry point for timestamp mode
 * @lp: Pointer to axienet local structure
 * @ifr: ioctl data
 *
 * Set hardware to the requested more. If unsupported return an error
 * with no changes. Otherwise, store the mode for future reference
 *
 * Return: 0 on success, Negative value on errors
 */
static int axienet_set_ts_config(struct axienet_local *lp, struct ifreq *ifr)
{
	struct hwtstamp_config config;
	int err;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	err = axienet_set_timestamp_mode(lp, &config);
	if (err)
		return err;

	/* save these settings for future reference */
	memcpy(&lp->tstamp_config, &config, sizeof(lp->tstamp_config));

	return copy_to_user(ifr->ifr_data, &config,
			    sizeof(config)) ? -EFAULT : 0;
}

/**
 * axienet_get_ts_config - return the current timestamp configuration
 * to the user
 * @lp: pointer to axienet local structure
 * @ifr: ioctl data
 *
 * Return: 0 on success, Negative value on errors
 */
static int axienet_get_ts_config(struct axienet_local *lp, struct ifreq *ifr)
{
	struct hwtstamp_config *config = &lp->tstamp_config;

	return copy_to_user(ifr->ifr_data, config,
			    sizeof(*config)) ? -EFAULT : 0;
}
#endif

/* Ioctl MII Interface */
static int axienet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
#if defined(CONFIG_XILINX_AXI_EMAC_HWTSTAMP) || defined(CONFIG_XILINX_TSN_PTP)
	struct axienet_local *lp = netdev_priv(dev);
#endif

	if (!netif_running(dev))
		return -EINVAL;

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		if (!dev->phydev)
			return -EOPNOTSUPP;
		return phy_mii_ioctl(dev->phydev, rq, cmd);
#if defined(CONFIG_XILINX_AXI_EMAC_HWTSTAMP) || defined(CONFIG_XILINX_TSN_PTP)
	case SIOCSHWTSTAMP:
		return axienet_set_ts_config(lp, rq);
	case SIOCGHWTSTAMP:
		return axienet_get_ts_config(lp, rq);
#endif
#ifdef CONFIG_XILINX_TSN_QBV
	case SIOCCHIOCTL:
		if (lp->qbv_regs)
			return axienet_set_schedule(dev, rq->ifr_data);
		return -EINVAL;
	case SIOC_GET_SCHED:
		if (lp->qbv_regs)
			return axienet_get_schedule(dev, rq->ifr_data);
		return -EINVAL;
#endif
#ifdef CONFIG_XILINX_TSN_QBR
	case SIOC_PREEMPTION_CFG:
		return axienet_preemption(dev, rq->ifr_data);
	case SIOC_PREEMPTION_CTRL:
		return axienet_preemption_ctrl(dev, rq->ifr_data);
	case SIOC_PREEMPTION_STS:
		return axienet_preemption_sts(dev, rq->ifr_data);
	case SIOC_PREEMPTION_COUNTER:
		return axienet_preemption_cnt(dev, rq->ifr_data);
#ifdef CONFIG_XILINX_TSN_QBV
	case SIOC_QBU_USER_OVERRIDE:
		return axienet_qbu_user_override(dev, rq->ifr_data);
	case SIOC_QBU_STS:
		return axienet_qbu_sts(dev, rq->ifr_data);
#endif
#endif

	default:
		return -EOPNOTSUPP;
	}
}

static const struct net_device_ops axienet_netdev_ops = {
#ifdef CONFIG_XILINX_TSN
	.ndo_open = axienet_tsn_open,
#else
	.ndo_open = axienet_open,
#endif
	.ndo_stop = axienet_stop,
#ifdef CONFIG_XILINX_TSN
	.ndo_start_xmit = axienet_tsn_xmit,
#else
	.ndo_start_xmit = axienet_start_xmit,
#endif
	.ndo_change_mtu	= axienet_change_mtu,
	.ndo_set_mac_address = netdev_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_rx_mode = axienet_set_multicast_list,
	.ndo_do_ioctl = axienet_ioctl,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = axienet_poll_controller,
#endif
};

/**
 * axienet_ethtools_get_drvinfo - Get various Axi Ethernet driver information.
 * @ndev:	Pointer to net_device structure
 * @ed:		Pointer to ethtool_drvinfo structure
 *
 * This implements ethtool command for getting the driver information.
 * Issue "ethtool -i ethX" under linux prompt to execute this function.
 */
static void axienet_ethtools_get_drvinfo(struct net_device *ndev,
					 struct ethtool_drvinfo *ed)
{
	strlcpy(ed->driver, DRIVER_NAME, sizeof(ed->driver));
	strlcpy(ed->version, DRIVER_VERSION, sizeof(ed->version));
}

/**
 * axienet_ethtools_get_regs_len - Get the total regs length present in the
 *				   AxiEthernet core.
 * @ndev:	Pointer to net_device structure
 *
 * This implements ethtool command for getting the total register length
 * information.
 *
 * Return: the total regs length
 */
static int axienet_ethtools_get_regs_len(struct net_device *ndev)
{
	return sizeof(u32) * AXIENET_REGS_N;
}

/**
 * axienet_ethtools_get_regs - Dump the contents of all registers present
 *			       in AxiEthernet core.
 * @ndev:	Pointer to net_device structure
 * @regs:	Pointer to ethtool_regs structure
 * @ret:	Void pointer used to return the contents of the registers.
 *
 * This implements ethtool command for getting the Axi Ethernet register dump.
 * Issue "ethtool -d ethX" to execute this function.
 */
static void axienet_ethtools_get_regs(struct net_device *ndev,
				      struct ethtool_regs *regs, void *ret)
{
	u32 *data = (u32 *)ret;
	size_t len = sizeof(u32) * AXIENET_REGS_N;
	struct axienet_local *lp = netdev_priv(ndev);

	regs->version = 0;
	regs->len = len;

	memset(data, 0, len);
	data[0] = axienet_ior(lp, XAE_RAF_OFFSET);
	data[1] = axienet_ior(lp, XAE_TPF_OFFSET);
	data[2] = axienet_ior(lp, XAE_IFGP_OFFSET);
	data[3] = axienet_ior(lp, XAE_IS_OFFSET);
	data[4] = axienet_ior(lp, XAE_IP_OFFSET);
	data[5] = axienet_ior(lp, XAE_IE_OFFSET);
	data[6] = axienet_ior(lp, XAE_TTAG_OFFSET);
	data[7] = axienet_ior(lp, XAE_RTAG_OFFSET);
	data[8] = axienet_ior(lp, XAE_UAWL_OFFSET);
	data[9] = axienet_ior(lp, XAE_UAWU_OFFSET);
	data[10] = axienet_ior(lp, XAE_TPID0_OFFSET);
	data[11] = axienet_ior(lp, XAE_TPID1_OFFSET);
	data[12] = axienet_ior(lp, XAE_PPST_OFFSET);
	data[13] = axienet_ior(lp, XAE_RCW0_OFFSET);
	data[14] = axienet_ior(lp, XAE_RCW1_OFFSET);
	data[15] = axienet_ior(lp, XAE_TC_OFFSET);
	data[16] = axienet_ior(lp, XAE_FCC_OFFSET);
	data[17] = axienet_ior(lp, XAE_EMMC_OFFSET);
	data[18] = axienet_ior(lp, XAE_RMFC_OFFSET);
	data[19] = axienet_ior(lp, XAE_MDIO_MC_OFFSET);
	data[20] = axienet_ior(lp, XAE_MDIO_MCR_OFFSET);
	data[21] = axienet_ior(lp, XAE_MDIO_MWD_OFFSET);
	data[22] = axienet_ior(lp, XAE_MDIO_MRD_OFFSET);
	data[23] = axienet_ior(lp, XAE_TEMAC_IS_OFFSET);
	data[24] = axienet_ior(lp, XAE_TEMAC_IP_OFFSET);
	data[25] = axienet_ior(lp, XAE_TEMAC_IE_OFFSET);
	data[26] = axienet_ior(lp, XAE_TEMAC_IC_OFFSET);
	data[27] = axienet_ior(lp, XAE_UAW0_OFFSET);
	data[28] = axienet_ior(lp, XAE_UAW1_OFFSET);
	data[29] = axienet_ior(lp, XAE_FMC_OFFSET);
	data[30] = axienet_ior(lp, XAE_AF0_OFFSET);
	data[31] = axienet_ior(lp, XAE_AF1_OFFSET);
	/* Support only single DMA queue */
	data[32] = axienet_dma_in32(lp->dq[0], XAXIDMA_TX_CR_OFFSET);
	data[33] = axienet_dma_in32(lp->dq[0], XAXIDMA_TX_SR_OFFSET);
	data[34] = axienet_dma_in32(lp->dq[0], XAXIDMA_TX_CDESC_OFFSET);
	data[35] = axienet_dma_in32(lp->dq[0], XAXIDMA_TX_TDESC_OFFSET);
	data[36] = axienet_dma_in32(lp->dq[0], XAXIDMA_RX_CR_OFFSET);
	data[37] = axienet_dma_in32(lp->dq[0], XAXIDMA_RX_SR_OFFSET);
	data[38] = axienet_dma_in32(lp->dq[0], XAXIDMA_RX_CDESC_OFFSET);
	data[39] = axienet_dma_in32(lp->dq[0], XAXIDMA_RX_TDESC_OFFSET);
}

static void axienet_ethtools_get_ringparam(struct net_device *ndev,
					   struct ethtool_ringparam *ering)
{
	struct axienet_local *lp = netdev_priv(ndev);

	ering->rx_max_pending = RX_BD_NUM_MAX;
	ering->rx_mini_max_pending = 0;
	ering->rx_jumbo_max_pending = 0;
	ering->tx_max_pending = TX_BD_NUM_MAX;
	ering->rx_pending = lp->rx_bd_num;
	ering->rx_mini_pending = 0;
	ering->rx_jumbo_pending = 0;
	ering->tx_pending = lp->tx_bd_num;
}

static int axienet_ethtools_set_ringparam(struct net_device *ndev,
					  struct ethtool_ringparam *ering)
{
	struct axienet_local *lp = netdev_priv(ndev);

	if (ering->rx_pending > RX_BD_NUM_MAX ||
	    ering->rx_mini_pending ||
	    ering->rx_jumbo_pending ||
	    ering->tx_pending < TX_BD_NUM_MIN ||
	    ering->tx_pending > TX_BD_NUM_MAX)
		return -EINVAL;

	if (netif_running(ndev))
		return -EBUSY;

	lp->rx_bd_num = ering->rx_pending;
	lp->tx_bd_num = ering->tx_pending;
	return 0;
}

/**
 * axienet_ethtools_get_pauseparam - Get the pause parameter setting for
 *				     Tx and Rx paths.
 * @ndev:	Pointer to net_device structure
 * @epauseparm:	Pointer to ethtool_pauseparam structure.
 *
 * This implements ethtool command for getting axi ethernet pause frame
 * setting. Issue "ethtool -a ethX" to execute this function.
 */
static void
axienet_ethtools_get_pauseparam(struct net_device *ndev,
				struct ethtool_pauseparam *epauseparm)
{
	u32 regval;
	struct axienet_local *lp = netdev_priv(ndev);

	epauseparm->autoneg  = 0;
	regval = axienet_ior(lp, XAE_FCC_OFFSET);
	epauseparm->tx_pause = regval & XAE_FCC_FCTX_MASK;
	epauseparm->rx_pause = regval & XAE_FCC_FCRX_MASK;
}

/**
 * axienet_ethtools_set_pauseparam - Set device pause parameter(flow control)
 *				     settings.
 * @ndev:	Pointer to net_device structure
 * @epauseparm:	Pointer to ethtool_pauseparam structure
 *
 * This implements ethtool command for enabling flow control on Rx and Tx
 * paths. Issue "ethtool -A ethX tx on|off" under linux prompt to execute this
 * function.
 *
 * Return: 0 on success, -EFAULT if device is running
 */
static int
axienet_ethtools_set_pauseparam(struct net_device *ndev,
				struct ethtool_pauseparam *epauseparm)
{
	u32 regval = 0;
	struct axienet_local *lp = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netdev_err(ndev,
			   "Please stop netif before applying configuration\n");
		return -EFAULT;
	}

	regval = axienet_ior(lp, XAE_FCC_OFFSET);
	if (epauseparm->tx_pause)
		regval |= XAE_FCC_FCTX_MASK;
	else
		regval &= ~XAE_FCC_FCTX_MASK;
	if (epauseparm->rx_pause)
		regval |= XAE_FCC_FCRX_MASK;
	else
		regval &= ~XAE_FCC_FCRX_MASK;
	axienet_iow(lp, XAE_FCC_OFFSET, regval);

	return 0;
}

/**
 * axienet_ethtools_get_coalesce - Get DMA interrupt coalescing count.
 * @ndev:	Pointer to net_device structure
 * @ecoalesce:	Pointer to ethtool_coalesce structure
 *
 * This implements ethtool command for getting the DMA interrupt coalescing
 * count on Tx and Rx paths. Issue "ethtool -c ethX" under linux prompt to
 * execute this function.
 *
 * Return: 0 always
 */
static int axienet_ethtools_get_coalesce(struct net_device *ndev,
					 struct ethtool_coalesce *ecoalesce)
{
	u32 regval = 0;
	struct axienet_local *lp = netdev_priv(ndev);
	struct axienet_dma_q *q;
	int i;

	for_each_rx_dma_queue(lp, i) {
		q = lp->dq[i];

		regval = axienet_dma_in32(q, XAXIDMA_RX_CR_OFFSET);
		ecoalesce->rx_max_coalesced_frames +=
						(regval & XAXIDMA_COALESCE_MASK)
						     >> XAXIDMA_COALESCE_SHIFT;
	}
	for_each_tx_dma_queue(lp, i) {
		q = lp->dq[i];
		regval = axienet_dma_in32(q, XAXIDMA_TX_CR_OFFSET);
		ecoalesce->tx_max_coalesced_frames +=
						(regval & XAXIDMA_COALESCE_MASK)
						     >> XAXIDMA_COALESCE_SHIFT;
	}
	return 0;
}

/**
 * axienet_ethtools_set_coalesce - Set DMA interrupt coalescing count.
 * @ndev:	Pointer to net_device structure
 * @ecoalesce:	Pointer to ethtool_coalesce structure
 *
 * This implements ethtool command for setting the DMA interrupt coalescing
 * count on Tx and Rx paths. Issue "ethtool -C ethX rx-frames 5" under linux
 * prompt to execute this function.
 *
 * Return: 0, on success, Non-zero error value on failure.
 */
static int axienet_ethtools_set_coalesce(struct net_device *ndev,
					 struct ethtool_coalesce *ecoalesce)
{
	struct axienet_local *lp = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netdev_err(ndev,
			   "Please stop netif before applying configuration\n");
		return -EFAULT;
	}

	if ((ecoalesce->rx_coalesce_usecs) ||
	    (ecoalesce->rx_coalesce_usecs_irq) ||
	    (ecoalesce->rx_max_coalesced_frames_irq) ||
	    (ecoalesce->tx_coalesce_usecs) ||
	    (ecoalesce->tx_coalesce_usecs_irq) ||
	    (ecoalesce->tx_max_coalesced_frames_irq) ||
	    (ecoalesce->stats_block_coalesce_usecs) ||
	    (ecoalesce->use_adaptive_rx_coalesce) ||
	    (ecoalesce->use_adaptive_tx_coalesce) ||
	    (ecoalesce->pkt_rate_low) ||
	    (ecoalesce->rx_coalesce_usecs_low) ||
	    (ecoalesce->rx_max_coalesced_frames_low) ||
	    (ecoalesce->tx_coalesce_usecs_low) ||
	    (ecoalesce->tx_max_coalesced_frames_low) ||
	    (ecoalesce->pkt_rate_high) ||
	    (ecoalesce->rx_coalesce_usecs_high) ||
	    (ecoalesce->rx_max_coalesced_frames_high) ||
	    (ecoalesce->tx_coalesce_usecs_high) ||
	    (ecoalesce->tx_max_coalesced_frames_high) ||
	    (ecoalesce->rate_sample_interval))
		return -EOPNOTSUPP;
	if (ecoalesce->rx_max_coalesced_frames)
		lp->coalesce_count_rx = ecoalesce->rx_max_coalesced_frames;
	if (ecoalesce->tx_max_coalesced_frames)
		lp->coalesce_count_tx = ecoalesce->tx_max_coalesced_frames;

	return 0;
}

#if defined(CONFIG_XILINX_AXI_EMAC_HWTSTAMP) || defined(CONFIG_XILINX_TSN_PTP)
/**
 * axienet_ethtools_get_ts_info - Get h/w timestamping capabilities.
 * @ndev:	Pointer to net_device structure
 * @info:	Pointer to ethtool_ts_info structure
 *
 * Return: 0, on success, Non-zero error value on failure.
 */
static int axienet_ethtools_get_ts_info(struct net_device *ndev,
					struct ethtool_ts_info *info)
{
	struct axienet_local *lp = netdev_priv(ndev);

	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;
	info->tx_types = (1 << HWTSTAMP_TX_OFF) | (1 << HWTSTAMP_TX_ON) |
			(1 << HWTSTAMP_TX_ONESTEP_SYNC) |
			(1 << HWTSTAMP_TX_ONESTEP_P2P);
	info->rx_filters = (1 << HWTSTAMP_FILTER_NONE) |
			   (1 << HWTSTAMP_FILTER_ALL);
	info->phc_index = lp->phc_index;

#ifdef CONFIG_XILINX_TSN_PTP
	info->phc_index = axienet_phc_index;
#endif
	return 0;
}
#endif

/**
 * axienet_ethtools_sset_count - Get number of strings that
 *				 get_strings will write.
 * @ndev:	Pointer to net_device structure
 * @sset:	Get the set strings
 *
 * Return: number of strings, on success, Non-zero error value on
 *	   failure.
 */
int axienet_ethtools_sset_count(struct net_device *ndev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
#ifdef CONFIG_AXIENET_HAS_MCDMA
		return axienet_sset_count(ndev, sset);
#else
		return AXIENET_ETHTOOLS_SSTATS_LEN;
#endif
	default:
		return -EOPNOTSUPP;
	}
}

/**
 * axienet_ethtools_get_stats - Get the extended statistics
 *				about the device.
 * @ndev:	Pointer to net_device structure
 * @stats:	Pointer to ethtool_stats structure
 * @data:	To store the statistics values
 *
 * Return: None.
 */
void axienet_ethtools_get_stats(struct net_device *ndev,
				struct ethtool_stats *stats,
				u64 *data)
{
	unsigned int i = 0;

	data[i++] = ndev->stats.tx_packets;
	data[i++] = ndev->stats.rx_packets;
	data[i++] = ndev->stats.tx_bytes;
	data[i++] = ndev->stats.rx_bytes;
	data[i++] = ndev->stats.tx_errors;
	data[i++] = ndev->stats.rx_missed_errors + ndev->stats.rx_frame_errors;

#ifdef CONFIG_AXIENET_HAS_MCDMA
	axienet_get_stats(ndev, stats, data);
#endif
}

/**
 * axienet_ethtools_strings - Set of strings that describe
 *			 the requested objects.
 * @ndev:	Pointer to net_device structure
 * @sset:	Get the set strings
 * @data:	Data of Transmit and Receive statistics
 *
 * Return: None.
 */
void axienet_ethtools_strings(struct net_device *ndev, u32 sset, u8 *data)
{
	int i;

	for (i = 0; i < AXIENET_ETHTOOLS_SSTATS_LEN; i++) {
		if (sset == ETH_SS_STATS)
			memcpy(data + i * ETH_GSTRING_LEN,
			       axienet_get_ethtools_strings_stats[i].name,
			       ETH_GSTRING_LEN);
	}
#ifdef CONFIG_AXIENET_HAS_MCDMA
	axienet_strings(ndev, sset, data);
#endif
}

static const struct ethtool_ops axienet_ethtool_ops = {
	.supported_coalesce_params = ETHTOOL_COALESCE_MAX_FRAMES,
	.get_drvinfo    = axienet_ethtools_get_drvinfo,
	.get_regs_len   = axienet_ethtools_get_regs_len,
	.get_regs       = axienet_ethtools_get_regs,
	.get_link       = ethtool_op_get_link,
	.get_ringparam	= axienet_ethtools_get_ringparam,
	.set_ringparam	= axienet_ethtools_set_ringparam,
	.get_pauseparam = axienet_ethtools_get_pauseparam,
	.set_pauseparam = axienet_ethtools_set_pauseparam,
	.get_coalesce   = axienet_ethtools_get_coalesce,
	.set_coalesce   = axienet_ethtools_set_coalesce,
	.get_link_ksettings = axienet_ethtools_get_link_ksettings,
	.set_link_ksettings = axienet_ethtools_set_link_ksettings,
};

static void axienet_validate(struct phylink_config *config,
			     unsigned long *supported,
			     struct phylink_link_state *state)
{
	struct net_device *ndev = to_net_dev(config->dev);
	struct axienet_local *lp = netdev_priv(ndev);
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	/* Only support the mode we are configured for */
	if (state->interface != PHY_INTERFACE_MODE_NA &&
	    state->interface != lp->phy_mode) {
		netdev_warn(ndev, "Cannot use PHY mode %s, supported: %s\n",
			    phy_modes(state->interface),
			    phy_modes(lp->phy_mode));
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		return;
	}

	phylink_set(mask, Autoneg);
	phylink_set_port_modes(mask);

	phylink_set(mask, Asym_Pause);
	phylink_set(mask, Pause);

	switch (state->interface) {
	case PHY_INTERFACE_MODE_NA:
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_GMII:
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		phylink_set(mask, 1000baseX_Full);
		phylink_set(mask, 1000baseT_Full);
		if (state->interface == PHY_INTERFACE_MODE_1000BASEX)
			break;
		fallthrough;
	case PHY_INTERFACE_MODE_MII:
		phylink_set(mask, 100baseT_Full);
		phylink_set(mask, 10baseT_Full);
	default:
		break;
	}

	bitmap_and(supported, supported, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
}

static void axienet_mac_pcs_get_state(struct phylink_config *config,
				      struct phylink_link_state *state)
{
	struct net_device *ndev = to_net_dev(config->dev);
	struct axienet_local *lp = netdev_priv(ndev);

	switch (state->interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		phylink_mii_c22_pcs_get_state(lp->pcs_phy, state);
		break;
	default:
		break;
	}
}

static void axienet_mac_an_restart(struct phylink_config *config)
{
	struct net_device *ndev = to_net_dev(config->dev);
	struct axienet_local *lp = netdev_priv(ndev);

	phylink_mii_c22_pcs_an_restart(lp->pcs_phy);
}

static void axienet_mac_config(struct phylink_config *config, unsigned int mode,
			       const struct phylink_link_state *state)
{
	struct net_device *ndev = to_net_dev(config->dev);
	struct axienet_local *lp = netdev_priv(ndev);
	int ret;

	switch (state->interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		ret = phylink_mii_c22_pcs_config(lp->pcs_phy, mode,
						 state->interface,
						 state->advertising);
		if (ret < 0)
			netdev_warn(ndev, "Failed to configure PCS: %d\n",
				    ret);
		break;

	default:
		break;
	}
}

static void axienet_mac_link_down(struct phylink_config *config,
				  unsigned int mode,
				  phy_interface_t interface)
{
	/* nothing meaningful to do */
}

static void axienet_mac_link_up(struct phylink_config *config,
				struct phy_device *phy,
				unsigned int mode, phy_interface_t interface,
				int speed, int duplex,
				bool tx_pause, bool rx_pause)
{
	struct net_device *ndev = to_net_dev(config->dev);
	struct axienet_local *lp = netdev_priv(ndev);
	u32 emmc_reg, fcc_reg;

	emmc_reg = axienet_ior(lp, XAE_EMMC_OFFSET);
	emmc_reg &= ~XAE_EMMC_LINKSPEED_MASK;

	switch (speed) {
	case SPEED_1000:
		emmc_reg |= XAE_EMMC_LINKSPD_1000;
		break;
	case SPEED_100:
		emmc_reg |= XAE_EMMC_LINKSPD_100;
		break;
	case SPEED_10:
		emmc_reg |= XAE_EMMC_LINKSPD_10;
		break;
	default:
		dev_err(&ndev->dev,
			"Speed other than 10, 100 or 1Gbps is not supported\n");
		break;
	}

	axienet_iow(lp, XAE_EMMC_OFFSET, emmc_reg);

	fcc_reg = axienet_ior(lp, XAE_FCC_OFFSET);
	if (tx_pause)
		fcc_reg |= XAE_FCC_FCTX_MASK;
	else
		fcc_reg &= ~XAE_FCC_FCTX_MASK;
	if (rx_pause)
		fcc_reg |= XAE_FCC_FCRX_MASK;
	else
		fcc_reg &= ~XAE_FCC_FCRX_MASK;
	axienet_iow(lp, XAE_FCC_OFFSET, fcc_reg);
}

static const struct phylink_mac_ops axienet_phylink_ops = {
	.validate = axienet_validate,
	.mac_pcs_get_state = axienet_mac_pcs_get_state,
	.mac_an_restart = axienet_mac_an_restart,
	.mac_config = axienet_mac_config,
	.mac_link_down = axienet_mac_link_down,
	.mac_link_up = axienet_mac_link_up,
};

/**
 * axienet_dma_err_handler - Work queue task for Axi DMA Error
 * @work:	pointer to work_struct
 *
 * Resets the Axi DMA and Axi Ethernet devices, and reconfigures the
 * Tx/Rx BDs.
 */
static void axienet_dma_err_handler(struct work_struct *work)
{
	u32 axienet_status;
	u32 cr, i;
	struct axienet_local *lp = container_of(work, struct axienet_local,
						dma_err_task);
	struct net_device *ndev = lp->ndev;
	struct axidma_bd *cur_p;

	axienet_setoptions(ndev, lp->options &
			   ~(XAE_OPTION_TXEN | XAE_OPTION_RXEN));
	/* Disable the MDIO interface till Axi Ethernet Reset is completed.
	 * When we do an Axi Ethernet reset, it resets the complete core
	 * including the MDIO. MDIO must be disabled before resetting
	 * and re-enabled afterwards.
	 * Hold MDIO bus lock to avoid MDIO accesses during the reset.
	 */
	mutex_lock(&lp->mii_bus->mdio_lock);
	axienet_mdio_disable(lp);
	__axienet_device_reset(lp);
	axienet_mdio_enable(lp);
	mutex_unlock(&lp->mii_bus->mdio_lock);

	for (i = 0; i < lp->tx_bd_num; i++) {
		cur_p = &lp->tx_bd_v[i];
		if (cur_p->cntrl) {
			dma_addr_t addr = desc_get_phys_addr(lp, cur_p);

			dma_unmap_single(ndev->dev.parent, addr,
					 (cur_p->cntrl &
					  XAXIDMA_BD_CTRL_LENGTH_MASK),
					 DMA_TO_DEVICE);
		}
		if (cur_p->skb)
			dev_kfree_skb_irq(cur_p->skb);
		cur_p->phys = 0;
		cur_p->phys_msb = 0;
		cur_p->cntrl = 0;
		cur_p->status = 0;
		cur_p->app0 = 0;
		cur_p->app1 = 0;
		cur_p->app2 = 0;
		cur_p->app3 = 0;
		cur_p->app4 = 0;
		cur_p->skb = NULL;
	}

	for (i = 0; i < lp->rx_bd_num; i++) {
		cur_p = &lp->rx_bd_v[i];
		cur_p->status = 0;
		cur_p->app0 = 0;
		cur_p->app1 = 0;
		cur_p->app2 = 0;
		cur_p->app3 = 0;
		cur_p->app4 = 0;
	}

	lp->tx_bd_ci = 0;
	lp->tx_bd_tail = 0;
	lp->rx_bd_ci = 0;

	/* Start updating the Rx channel control register */
	cr = axienet_dma_in32(lp, XAXIDMA_RX_CR_OFFSET);
	/* Update the interrupt coalesce count */
	cr = ((cr & ~XAXIDMA_COALESCE_MASK) |
	      (XAXIDMA_DFT_RX_THRESHOLD << XAXIDMA_COALESCE_SHIFT));
	/* Update the delay timer count */
	cr = ((cr & ~XAXIDMA_DELAY_MASK) |
	      (XAXIDMA_DFT_RX_WAITBOUND << XAXIDMA_DELAY_SHIFT));
	/* Enable coalesce, delay timer and error interrupts */
	cr |= XAXIDMA_IRQ_ALL_MASK;
	/* Finally write to the Rx channel control register */
	axienet_dma_out32(lp, XAXIDMA_RX_CR_OFFSET, cr);

	/* Start updating the Tx channel control register */
	cr = axienet_dma_in32(lp, XAXIDMA_TX_CR_OFFSET);
	/* Update the interrupt coalesce count */
	cr = (((cr & ~XAXIDMA_COALESCE_MASK)) |
	      (XAXIDMA_DFT_TX_THRESHOLD << XAXIDMA_COALESCE_SHIFT));
	/* Update the delay timer count */
	cr = (((cr & ~XAXIDMA_DELAY_MASK)) |
	      (XAXIDMA_DFT_TX_WAITBOUND << XAXIDMA_DELAY_SHIFT));
	/* Enable coalesce, delay timer and error interrupts */
	cr |= XAXIDMA_IRQ_ALL_MASK;
	/* Finally write to the Tx channel control register */
	axienet_dma_out32(lp, XAXIDMA_TX_CR_OFFSET, cr);

	/* Populate the tail pointer and bring the Rx Axi DMA engine out of
	 * halted state. This will make the Rx side ready for reception.
	 */
	axienet_dma_out_addr(lp, XAXIDMA_RX_CDESC_OFFSET, lp->rx_bd_p);
	cr = axienet_dma_in32(lp, XAXIDMA_RX_CR_OFFSET);
	axienet_dma_out32(lp, XAXIDMA_RX_CR_OFFSET,
			  cr | XAXIDMA_CR_RUNSTOP_MASK);
	axienet_dma_out_addr(lp, XAXIDMA_RX_TDESC_OFFSET, lp->rx_bd_p +
			     (sizeof(*lp->rx_bd_v) * (lp->rx_bd_num - 1)));

	/* Write to the RS (Run-stop) bit in the Tx channel control register.
	 * Tx channel is now ready to run. But only after we write to the
	 * tail pointer register that the Tx channel will start transmitting
	 */
	axienet_dma_out_addr(lp, XAXIDMA_TX_CDESC_OFFSET, lp->tx_bd_p);
	cr = axienet_dma_in32(lp, XAXIDMA_TX_CR_OFFSET);
	axienet_dma_out32(lp, XAXIDMA_TX_CR_OFFSET,
			  cr | XAXIDMA_CR_RUNSTOP_MASK);

	axienet_status = axienet_ior(lp, XAE_RCW1_OFFSET);
	axienet_status &= ~XAE_RCW1_RX_MASK;
	axienet_iow(lp, XAE_RCW1_OFFSET, axienet_status);

	axienet_status = axienet_ior(lp, XAE_IP_OFFSET);
	if (axienet_status & XAE_INT_RXRJECT_MASK)
		axienet_iow(lp, XAE_IS_OFFSET, XAE_INT_RXRJECT_MASK);
	axienet_iow(lp, XAE_IE_OFFSET, lp->eth_irq > 0 ?
		    XAE_INT_RECV_ERROR_MASK : 0);
	axienet_iow(lp, XAE_FCC_OFFSET, XAE_FCC_FCRX_MASK);

	/* Sync default options with HW but leave receiver and
	 * transmitter disabled.
	 */
	axienet_setoptions(ndev, lp->options &
			   ~(XAE_OPTION_TXEN | XAE_OPTION_RXEN));
	axienet_set_mac_address(ndev, NULL);
	axienet_set_multicast_list(ndev);
	axienet_setoptions(ndev, lp->options);
}

/**
 * axienet_probe - Axi Ethernet probe function.
 * @pdev:	Pointer to platform device structure.
 *
 * Return: 0, on success
 *	    Non-zero error value on failure.
 *
 * This is the probe routine for Axi Ethernet driver. This is called before
 * any other driver routines are invoked. It allocates and sets up the Ethernet
 * device. Parses through device tree and populates fields of
 * axienet_local. It registers the Ethernet device.
 */
static int axienet_probe(struct platform_device *pdev)
{
	int (*axienet_clk_init)(struct platform_device *pdev,
				struct clk **axi_aclk, struct clk **axis_clk,
				struct clk **ref_clk, struct clk **tmpclk) =
					axienet_clk_init;
	int ret = 0;
	struct device_node *np;
	struct axienet_local *lp;
	struct net_device *ndev;
	const void *mac_addr;
	struct resource *ethres;
	u32 value;
	u16 num_queues = XAE_MAX_QUEUES;
	bool is_tsn = false;

	is_tsn = of_property_read_bool(pdev->dev.of_node, "xlnx,tsn");
	ret = of_property_read_u16(pdev->dev.of_node, "xlnx,num-queues",
				   &num_queues);
	if (ret) {
		if (!is_tsn) {
#ifndef CONFIG_AXIENET_HAS_MCDMA
			num_queues = 1;
#endif
		}
	}
#ifdef CONFIG_XILINX_TSN
	if (is_tsn && (num_queues < XAE_TSN_MIN_QUEUES ||
		       num_queues > XAE_MAX_QUEUES))
		num_queues = XAE_MAX_QUEUES;
#endif

	ndev = alloc_etherdev_mq(sizeof(*lp), num_queues);
	if (!ndev)
		return -ENOMEM;

	platform_set_drvdata(pdev, ndev);
#ifdef CONFIG_XILINX_TSN
	bool slave = false;
	if (is_tsn) {
		slave = of_property_read_bool(pdev->dev.of_node,
					      "xlnx,tsn-slave");
		if (slave)
			snprintf(ndev->name, sizeof(ndev->name), "eth2");
		else
			snprintf(ndev->name, sizeof(ndev->name), "eth1");
	}
#endif

	SET_NETDEV_DEV(ndev, &pdev->dev);
	ndev->flags &= ~IFF_MULTICAST;  /* clear multicast */
	ndev->features = NETIF_F_SG;
	ndev->netdev_ops = &axienet_netdev_ops;
	ndev->ethtool_ops = &axienet_ethtool_ops;

	/* MTU range: 64 - 9000 */
	ndev->min_mtu = 64;
	ndev->max_mtu = XAE_JUMBO_MTU;

	lp = netdev_priv(ndev);
	lp->ndev = ndev;
	lp->dev = &pdev->dev;
	lp->options = XAE_OPTION_DEFAULTS;
	lp->rx_bd_num = RX_BD_NUM_DEFAULT;
	lp->tx_bd_num = TX_BD_NUM_DEFAULT;

	lp->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(lp->clk)) {
		ret = PTR_ERR(lp->clk);
		goto free_netdev;
	}
	ret = clk_prepare_enable(lp->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable clock: %d\n", ret);
		goto free_netdev;
	}

	/* Map device registers */
	ethres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lp->regs = devm_ioremap_resource(&pdev->dev, ethres);
	if (IS_ERR(lp->regs)) {
		dev_err(&pdev->dev, "could not map Axi Ethernet regs.\n");
		ret = PTR_ERR(lp->regs);
		goto cleanup_clk;
	}
	lp->regs_start = ethres->start;

	/* Setup checksum offload, but default to off if not specified */
	lp->features = 0;

	if (pdev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_node(axienet_of_match, pdev->dev.of_node);
		if (match && match->data) {
			lp->axienet_config = match->data;
			axienet_clk_init = lp->axienet_config->clk_init;
		}
	}

	ret = of_property_read_u32(pdev->dev.of_node, "xlnx,txcsum", &value);
	if (!ret) {
		dev_info(&pdev->dev, "TX_CSUM %d\n", value);

		switch (value) {
		case 1:
			lp->csum_offload_on_tx_path =
				XAE_FEATURE_PARTIAL_TX_CSUM;
			lp->features |= XAE_FEATURE_PARTIAL_TX_CSUM;
			/* Can checksum TCP/UDP over IPv4. */
			ndev->features |= NETIF_F_IP_CSUM | NETIF_F_SG;
			break;
		case 2:
			lp->csum_offload_on_tx_path =
				XAE_FEATURE_FULL_TX_CSUM;
			lp->features |= XAE_FEATURE_FULL_TX_CSUM;
			/* Can checksum TCP/UDP over IPv4. */
			ndev->features |= NETIF_F_IP_CSUM | NETIF_F_SG;
			break;
		default:
			lp->csum_offload_on_tx_path = XAE_NO_CSUM_OFFLOAD;
		}
	}
	ret = of_property_read_u32(pdev->dev.of_node, "xlnx,rxcsum", &value);
	if (!ret) {
		dev_info(&pdev->dev, "RX_CSUM %d\n", value);

		switch (value) {
		case 1:
			lp->csum_offload_on_rx_path =
				XAE_FEATURE_PARTIAL_RX_CSUM;
			lp->features |= XAE_FEATURE_PARTIAL_RX_CSUM;
			break;
		case 2:
			lp->csum_offload_on_rx_path =
				XAE_FEATURE_FULL_RX_CSUM;
			lp->features |= XAE_FEATURE_FULL_RX_CSUM;
			break;
		default:
			lp->csum_offload_on_rx_path = XAE_NO_CSUM_OFFLOAD;
		}
	}
	/* For supporting jumbo frames, the Axi Ethernet hardware must have
	 * a larger Rx/Tx Memory. Typically, the size must be large so that
	 * we can enable jumbo option and start supporting jumbo frames.
	 * Here we check for memory allocated for Rx/Tx in the hardware from
	 * the device-tree and accordingly set flags.
	 */
	of_property_read_u32(pdev->dev.of_node, "xlnx,rxmem", &lp->rxmem);

	/* Start with the proprietary, and broken phy_type */
	ret = of_property_read_u32(pdev->dev.of_node, "xlnx,phy-type", &value);
	if (!ret) {
		netdev_warn(ndev, "Please upgrade your device tree binary blob to use phy-mode");
		switch (value) {
		case XAE_PHY_TYPE_MII:
			lp->phy_mode = PHY_INTERFACE_MODE_MII;
			break;
		case XAE_PHY_TYPE_GMII:
			lp->phy_mode = PHY_INTERFACE_MODE_GMII;
			break;
		case XAE_PHY_TYPE_RGMII_2_0:
			lp->phy_mode = PHY_INTERFACE_MODE_RGMII_ID;
			break;
		case XAE_PHY_TYPE_SGMII:
			lp->phy_mode = PHY_INTERFACE_MODE_SGMII;
			break;
		case XAE_PHY_TYPE_1000BASE_X:
			lp->phy_mode = PHY_INTERFACE_MODE_1000BASEX;
			break;
		default:
			ret = -EINVAL;
			goto cleanup_clk;
		}
	} else {
		ret = of_get_phy_mode(pdev->dev.of_node, &lp->phy_mode);
		if (ret)
			goto cleanup_clk;
	}

	/* Find the DMA node, map the DMA registers, and decode the DMA IRQs */
	np = of_parse_phandle(pdev->dev.of_node, "axistream-connected", 0);
	if (np) {
		struct resource dmares;

		ret = of_address_to_resource(np, 0, &dmares);
		if (ret) {
			dev_err(&pdev->dev,
				"unable to get DMA resource\n");
			of_node_put(np);
			goto cleanup_clk;
		}
		lp->dma_regs = devm_ioremap_resource(&pdev->dev,
						     &dmares);
		lp->rx_irq = irq_of_parse_and_map(np, 1);
		lp->tx_irq = irq_of_parse_and_map(np, 0);
		of_node_put(np);
		lp->eth_irq = platform_get_irq_optional(pdev, 0);
	} else {
		/* Check for these resources directly on the Ethernet node. */
		struct resource *res = platform_get_resource(pdev,
							     IORESOURCE_MEM, 1);
		lp->dma_regs = devm_ioremap_resource(&pdev->dev, res);
		lp->rx_irq = platform_get_irq(pdev, 1);
		lp->tx_irq = platform_get_irq(pdev, 0);
		lp->eth_irq = platform_get_irq_optional(pdev, 2);
	}
	if (IS_ERR(lp->dma_regs)) {
		dev_err(&pdev->dev, "could not map DMA regs\n");
		ret = PTR_ERR(lp->dma_regs);
		goto cleanup_clk;
	}
	if ((lp->rx_irq <= 0) || (lp->tx_irq <= 0)) {
		dev_err(&pdev->dev, "could not determine irqs\n");
		ret = -ENOMEM;
		goto cleanup_clk;
	}

	/* Autodetect the need for 64-bit DMA pointers.
	 * When the IP is configured for a bus width bigger than 32 bits,
	 * writing the MSB registers is mandatory, even if they are all 0.
	 * We can detect this case by writing all 1's to one such register
	 * and see if that sticks: when the IP is configured for 32 bits
	 * only, those registers are RES0.
	 * Those MSB registers were introduced in IP v7.1, which we check first.
	 */
	if ((axienet_ior(lp, XAE_ID_OFFSET) >> 24) >= 0x9) {
		void __iomem *desc = lp->dma_regs + XAXIDMA_TX_CDESC_OFFSET + 4;

		iowrite32(0x0, desc);
		if (ioread32(desc) == 0) {	/* sanity check */
			iowrite32(0xffffffff, desc);
			if (ioread32(desc) > 0) {
				lp->features |= XAE_FEATURE_DMA_64BIT;
				addr_width = 64;
				dev_info(&pdev->dev,
					 "autodetected 64-bit DMA range\n");
			}
			iowrite32(0x0, desc);
		}
	}

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(addr_width));
	if (ret) {
		dev_err(&pdev->dev, "No suitable DMA available\n");
		goto cleanup_clk;
	}

	/* Check for Ethernet core IRQ (optional) */
	if (lp->eth_irq <= 0)
		dev_info(&pdev->dev, "Ethernet core IRQ not defined\n");

	/* Retrieve the MAC address */
	mac_addr = of_get_mac_address(pdev->dev.of_node);
	if (IS_ERR(mac_addr)) {
		dev_warn(&pdev->dev, "could not find MAC address property: %ld\n",
			 PTR_ERR(mac_addr));
		mac_addr = NULL;
	}
	axienet_set_mac_address(ndev, mac_addr);

	lp->coalesce_count_rx = XAXIDMA_DFT_RX_THRESHOLD;
	lp->coalesce_count_tx = XAXIDMA_DFT_TX_THRESHOLD;

	/* Reset core now that clocks are enabled, prior to accessing MDIO */
	ret = __axienet_device_reset(lp);
	if (ret)
		goto cleanup_clk;

	ret = axienet_mdio_setup(lp);
	if (ret)
		dev_warn(&pdev->dev,
			 "error registering MDIO bus: %d\n", ret);

	if (lp->phy_mode == PHY_INTERFACE_MODE_SGMII ||
	    lp->phy_mode == PHY_INTERFACE_MODE_1000BASEX) {
		lp->phy_node = of_parse_phandle(pdev->dev.of_node, "phy-handle", 0);
		if (!lp->phy_node) {
			dev_err(&pdev->dev, "phy-handle required for 1000BaseX/SGMII\n");
			ret = -EINVAL;
			goto cleanup_mdio;
		}
		lp->pcs_phy = of_mdio_find_device(lp->phy_node);
		if (!lp->pcs_phy) {
			ret = -EPROBE_DEFER;
			goto cleanup_mdio;
		}
		lp->phylink_config.pcs_poll = true;
	}

	lp->phylink_config.dev = &ndev->dev;
	lp->phylink_config.type = PHYLINK_NETDEV;

	lp->phylink = phylink_create(&lp->phylink_config, pdev->dev.fwnode,
				     lp->phy_mode,
				     &axienet_phylink_ops);
	if (IS_ERR(lp->phylink)) {
		ret = PTR_ERR(lp->phylink);
		dev_err(&pdev->dev, "phylink_create error (%i)\n", ret);
		goto cleanup_mdio;
	}

	ret = register_netdev(lp->ndev);
	if (ret) {
		dev_err(lp->dev, "register_netdev() error (%i)\n", ret);
		goto cleanup_phylink;
	}

	return 0;

cleanup_phylink:
	phylink_destroy(lp->phylink);

cleanup_mdio:
	if (lp->pcs_phy)
		put_device(&lp->pcs_phy->dev);
	if (lp->mii_bus)
		axienet_mdio_teardown(lp);
	of_node_put(lp->phy_node);

cleanup_clk:
	clk_disable_unprepare(lp->clk);

free_netdev:
	free_netdev(ndev);

	return ret;
}

static int axienet_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct axienet_local *lp = netdev_priv(ndev);

	unregister_netdev(ndev);

	if (lp->phylink)
		phylink_destroy(lp->phylink);

	if (lp->pcs_phy)
		put_device(&lp->pcs_phy->dev);

	axienet_mdio_teardown(lp);

	clk_disable_unprepare(lp->clk);

	of_node_put(lp->phy_node);
	lp->phy_node = NULL;

	free_netdev(ndev);

	return 0;
}

static void axienet_shutdown(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	rtnl_lock();
	netif_device_detach(ndev);

	if (netif_running(ndev))
		dev_close(ndev);

	rtnl_unlock();
}

static struct platform_driver axienet_driver = {
	.probe = axienet_probe,
	.remove = axienet_remove,
	.shutdown = axienet_shutdown,
	.driver = {
		 .name = "xilinx_axienet",
		 .of_match_table = axienet_of_match,
	},
};

module_platform_driver(axienet_driver);

MODULE_DESCRIPTION("Xilinx Axi Ethernet driver");
MODULE_AUTHOR("Xilinx");
MODULE_LICENSE("GPL");
