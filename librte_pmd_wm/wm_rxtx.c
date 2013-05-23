/*-
 * Copyright (c) 2013 Antti Kantee.  All rights reserved
 *
 * 2-clause BSD license
 */

/*-
 *   BSD LICENSE
 * 
 *   Copyright(c) 2010-2012 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without 
 *   modification, are permitted provided that the following conditions 
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright 
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright 
 *       notice, this list of conditions and the following disclaimer in 
 *       the documentation and/or other materials provided with the 
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its 
 *       contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *  version: DPDK.L.1.2.3-3
 */

#include <sys/queue.h>

#include <assert.h>
#include <endian.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdarg.h>
#include <inttypes.h>

#include <rte_interrupts.h>
#include <rte_byteorder.h>
#include <rte_common.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_pci.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_launch.h>
#include <rte_tailq.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_ring.h>
#include <rte_mempool.h>
#include <rte_malloc.h>
#include <rte_mbuf.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_prefetch.h>
#include <rte_udp.h>
#include <rte_tcp.h>
#include <rte_sctp.h>
#include <rte_string_fns.h>

#include "e1000_logs.h"
#include "e1000/e1000_api.h"
#include "wm_ethdev.h"

static inline struct rte_mbuf *
rte_rxmbuf_alloc(struct rte_mempool *mp)
{
	struct rte_mbuf *m;

	m = __rte_mbuf_raw_alloc(mp);
	__rte_mbuf_sanity_check_raw(m, RTE_MBUF_PKT, 0);
	return (m);
}

#define RTE_MBUF_DATA_DMA_ADDR(mb) \
	(uint64_t) ((mb)->buf_physaddr +		   \
			(uint64_t) ((char *)((mb)->pkt.data) -     \
			 	(char *)(mb)->buf_addr))

#define RTE_MBUF_DATA_DMA_ADDR_DEFAULT(mb) \
	(uint64_t) ((mb)->buf_physaddr + RTE_PKTMBUF_HEADROOM)

/**
 * Structure associated with each descriptor of the RX ring of a RX queue.
 */
struct wm_rx_entry {
	struct rte_mbuf *mbuf; /**< mbuf associated with RX descriptor. */
};

/**
 * Structure associated with each descriptor of the TX ring of a TX queue.
 */
struct wm_tx_entry {
	struct rte_mbuf *mbuf; /**< mbuf associated with TX desc, if any. */
	uint16_t next_id; /**< Index of next descriptor in ring. */
	uint16_t last_id; /**< Index of last scattered descriptor. */
};

/**
 * Structure associated with each RX queue.
 */
struct igb_rx_queue {
	struct rte_mempool  *mb_pool;   /**< mbuf pool to populate RX ring. */
	struct e1000_rx_desc *rx_ring; /**< RX ring virtual address. */
	uint64_t            rx_ring_phys_addr; /**< RX ring DMA address. */
	volatile uint32_t   *rdt_reg_addr; /**< RDT register address. */
	struct wm_rx_entry *sw_ring;   /**< address of RX software ring. */
	struct rte_mbuf *pkt_first_seg; /**< First segment of current packet. */
	struct rte_mbuf *pkt_last_seg;  /**< Last segment of current packet. */
	uint16_t            nb_rx_desc; /**< number of RX descriptors. */
	uint16_t            rx_tail;    /**< current value of RDT register. */
	uint16_t            nb_rx_hold; /**< number of held free RX desc. */
	uint16_t            rx_free_thresh; /**< max free RX desc to hold. */
	uint16_t            queue_id;   /**< RX queue index. */
	uint8_t             port_id;    /**< Device port identifier. */
	uint8_t             pthresh;    /**< Prefetch threshold register. */
	uint8_t             hthresh;    /**< Host threshold register. */
	uint8_t             wthresh;    /**< Write-back threshold register. */
	uint8_t             crc_len;    /**< 0 if CRC stripped, 4 otherwise. */
};

/**
 * Structure associated with each TX queue.
 */
struct igb_tx_queue {
	struct e1000_tx_desc *tx_ring; /**< TX ring address */
	uint64_t               tx_ring_phys_addr; /**< TX ring DMA address. */
	struct wm_tx_entry    *sw_ring; /**< virtual address of SW ring. */
	volatile uint32_t      *tdt_reg_addr; /**< Address of TDT register. */
	uint16_t               nb_tx_desc;    /**< number of TX descriptors. */
	uint16_t               tx_tail;  /**< Current value of TDT register. */
	uint16_t               tx_head;  /**< Index of first used TX descriptor. */
	uint16_t               queue_id; /**< TX queue index. */
	uint8_t                port_id;  /**< Device port identifier. */
	uint8_t                pthresh;  /**< Prefetch threshold register. */
	uint8_t                hthresh;  /**< Host threshold register. */
	uint8_t                wthresh;  /**< Write-back threshold register. */
	uint32_t               ctx_curr; /**< Current used hardware descriptor. */
	uint32_t               ctx_start;/**< Start context position for transmit queue. */
};

#if 1
#define RTE_PMD_USE_PREFETCH
#endif

#ifdef RTE_PMD_USE_PREFETCH
#define rte_wm_prefetch(p)	rte_prefetch0(p)
#else
#define rte_wm_prefetch(p)	do {} while(0)
#endif

#ifdef RTE_PMD_PACKET_PREFETCH
#define rte_packet_prefetch(p) rte_prefetch1(p)
#else
#define rte_packet_prefetch(p)	do {} while(0)
#endif

/*********************************************************************
 *
 *  TX function
 *
 **********************************************************************/

static inline uint32_t
tx_desc_cksum_flags_to_olinfo(uint16_t ol_flags)
{
	static const uint32_t l4_olinfo[2] = {0, E1000_ADVTXD_POPTS_TXSM};
	static const uint32_t l3_olinfo[2] = {0, E1000_ADVTXD_POPTS_IXSM};
	uint32_t tmp;

	tmp  = l4_olinfo[(ol_flags & PKT_TX_L4_MASK)  != PKT_TX_L4_NO_CKSUM];
	tmp |= l3_olinfo[(ol_flags & PKT_TX_IP_CKSUM) != 0];
	return tmp;
}

static inline uint32_t
tx_desc_vlan_flags_to_cmdtype(uint16_t ol_flags)
{
	static uint32_t vlan_cmd[2] = {0, E1000_ADVTXD_DCMD_VLE};
	return vlan_cmd[(ol_flags & PKT_TX_VLAN_PKT) != 0];
}

uint16_t
eth_wm_xmit_pkts(struct igb_tx_queue *txq, struct rte_mbuf **tx_pkts,
	       uint16_t nb_pkts)
{
	struct wm_tx_entry *sw_ring;
	struct wm_tx_entry *txe, *txn;
	volatile struct e1000_tx_desc *txr, *txd;
	struct rte_mbuf     *tx_pkt;
	struct rte_mbuf     *m_seg;
	uint64_t buf_dma_addr;
	uint32_t cmd_type_len;
	uint32_t pkt_len;
	uint16_t slen;
	uint16_t ol_flags;
	uint16_t tx_end;
	uint16_t tx_id;
	uint16_t tx_last;
	uint16_t nb_tx;
	uint16_t tx_ol_req;
	uint32_t new_ctx = 0;
	uint32_t ctx = 0;
	uint32_t vlan_macip_lens;

	sw_ring = txq->sw_ring;
	txr     = txq->tx_ring;
	tx_id   = txq->tx_tail;
	txe = &sw_ring[tx_id];

	for (nb_tx = 0; nb_tx < nb_pkts; nb_tx++) {
		tx_pkt = *tx_pkts++;
		pkt_len = tx_pkt->pkt.pkt_len;

		RTE_MBUF_PREFETCH_TO_FREE(txe->mbuf);

		/*
		 * The number of descriptors that must be allocated for a
		 * packet is the number of segments of that packet, plus 1
		 * Context Descriptor for the VLAN Tag Identifier, if any.
		 * Determine the last TX descriptor to allocate in the TX ring
		 * for the packet, starting from the current position (tx_id)
		 * in the ring.
		 */
		tx_last = (uint16_t) (tx_id + tx_pkt->pkt.nb_segs - 1);
		ol_flags = tx_pkt->ol_flags;

		if (tx_last >= txq->nb_tx_desc)
			tx_last = (uint16_t) (tx_last - txq->nb_tx_desc);

		PMD_TX_LOG(DEBUG, "port_id=%u queue_id=%u pktlen=%u"
			   " tx_first=%u tx_last=%u\n",
			   (unsigned) txq->port_id,
			   (unsigned) txq->queue_id,
			   (unsigned) pkt_len,
			   (unsigned) tx_id,
			   (unsigned) tx_last);

		/*
		 * Check if there are enough free descriptors in the TX ring
		 * to transmit the next packet.
		 * This operation is based on the two following rules:
		 *
		 *   1- Only check that the last needed TX descriptor can be
		 *      allocated (by construction, if that descriptor is free,
		 *      all intermediate ones are also free).
		 *
		 *      For this purpose, the index of the last TX descriptor
		 *      used for a packet (the "last descriptor" of a packet)
		 *      is recorded in the TX entries (the last one included)
		 *      that are associated with all TX descriptors allocated
		 *      for that packet.
		 *
		 *   2- Avoid to allocate the last free TX descriptor of the
		 *      ring, in order to never set the TDT register with the
		 *      same value stored in parallel by the NIC in the TDH
		 *      register, which makes the TX engine of the NIC enter
		 *      in a deadlock situation.
		 *
		 *      By extension, avoid to allocate a free descriptor that
		 *      belongs to the last set of free descriptors allocated
		 *      to the same packet previously transmitted.
		 */

		/*
		 * The "last descriptor" of the previously sent packet, if any,
		 * which used the last descriptor to allocate.
		 */
		tx_end = sw_ring[tx_last].last_id;

		/*
		 * The next descriptor following that "last descriptor" in the
		 * ring.
		 */
		tx_end = sw_ring[tx_end].next_id;

		/*
		 * The "last descriptor" associated with that next descriptor.
		 */
		tx_end = sw_ring[tx_end].last_id;

		/*
		 * Check that this descriptor is free.
		 */
		if (! (txr[tx_end].upper.fields.status & E1000_TXD_STAT_DD)) {
			if (nb_tx == 0)
				return (0);
			goto end_of_tx;
		}

		/*
		 * Set common flags of all TX Data Descriptors.
		 *
		 * The following bits must be set in all Data Descriptors:
		 *   - E1000_ADVTXD_DTYP_DATA
		 *   - E1000_ADVTXD_DCMD_DEXT
		 *
		 * The following bits must be set in the first Data Descriptor
		 * and are ignored in the other ones:
		 *   - E1000_ADVTXD_DCMD_IFCS
		 *   - E1000_ADVTXD_MAC_1588
		 *   - E1000_ADVTXD_DCMD_VLE
		 *
		 * The following bits must only be set in the last Data
		 * Descriptor:
		 *   - E1000_TXD_CMD_EOP
		 *
		 * The following bits can be set in any Data Descriptor, but
		 * are only set in the last Data Descriptor:
		 *   - E1000_TXD_CMD_RS
		 */

		cmd_type_len = E1000_TXD_CMD_IFCS;

		m_seg = tx_pkt;
		do {
			txn = &sw_ring[txe->next_id];
			txd = &txr[tx_id];

			if (txe->mbuf != NULL)
				rte_pktmbuf_free_seg(txe->mbuf);
			txe->mbuf = m_seg;

			/*
			 * Set up transmit descriptor.
			 */
			slen = (uint16_t) m_seg->pkt.data_len;
			buf_dma_addr = RTE_MBUF_DATA_DMA_ADDR(m_seg);

			txd->buffer_addr = buf_dma_addr;
			txd->lower.data =
			    rte_cpu_to_le_32(E1000_TXD_CMD_IFCS | slen);
			txd->upper.data = 0;

			txe->last_id = tx_last;
			tx_id = txe->next_id;
			txe = txn;
			m_seg = m_seg->pkt.next;
		} while (m_seg != NULL);

		/*
		 * The last packet data descriptor needs End Of Packet (EOP)
		 * and Report Status (RS).
		 */
		txd->lower.data |=
			rte_cpu_to_le_32(E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);
	}
 end_of_tx:
	rte_wmb();

	/*
	 * Set the Transmit Descriptor Tail (TDT).
	 */
	txq->tx_tail = tx_id;
	E1000_PCI_REG_WRITE(txq->tdt_reg_addr, tx_id);

#if 0
	PMD_TX_LOG(DEBUG, "port_id=%u queue_id=%u tx_tail=%u nb_tx=%u",
		   (unsigned) txq->port_id, (unsigned) txq->queue_id,
		   (unsigned) tx_id, (unsigned) nb_tx);
#endif

	return (nb_tx);
}

/*********************************************************************
 *
 *  RX functions
 *
 **********************************************************************/
static inline uint16_t
rx_desc_hlen_type_rss_to_pkt_flags(uint32_t hl_tp_rs)
{
	uint16_t pkt_flags;

	static uint16_t ip_pkt_types_map[16] = {
		0, PKT_RX_IPV4_HDR, PKT_RX_IPV4_HDR_EXT, PKT_RX_IPV4_HDR_EXT,
		PKT_RX_IPV6_HDR, 0, 0, 0,
		PKT_RX_IPV6_HDR_EXT, 0, 0, 0,
		PKT_RX_IPV6_HDR_EXT, 0, 0, 0,
	};

#if defined(RTE_LIBRTE_IEEE1588)
	static uint32_t ip_pkt_etqf_map[8] = {
		0, 0, 0, PKT_RX_IEEE1588_PTP,
		0, 0, 0, 0,
	};

	pkt_flags = (uint16_t) (hl_tp_rs & E1000_RXDADV_PKTTYPE_ETQF) ?
				ip_pkt_etqf_map[(hl_tp_rs >> 4) & 0x07] :
				ip_pkt_types_map[(hl_tp_rs >> 4) & 0x0F];
#else
	pkt_flags = (uint16_t) (hl_tp_rs & E1000_RXDADV_PKTTYPE_ETQF) ? 0 :
				ip_pkt_types_map[(hl_tp_rs >> 4) & 0x0F];
#endif
	return pkt_flags | (uint16_t) (((hl_tp_rs & 0x0F) == 0) ? 0 :
					PKT_RX_RSS_HASH);
}

static inline uint16_t
rx_desc_status_to_pkt_flags(uint32_t rx_status)
{
	uint16_t pkt_flags;

	/* Check if VLAN present */
	pkt_flags = (uint16_t) (rx_status & E1000_RXD_STAT_VP) ? PKT_RX_VLAN_PKT : 0;

#if defined(RTE_LIBRTE_IEEE1588)
	if (rx_status & E1000_RXD_STAT_TMST)
		pkt_flags = pkt_flags | PKT_RX_IEEE1588_TMST;
#endif
	return pkt_flags;
}

static inline uint16_t
rx_desc_error_to_pkt_flags(uint32_t rx_status)
{
	/*
	 * Bit 30: IPE, IPv4 checksum error
	 * Bit 29: L4I, L4I integrity error
	 */

	static uint16_t error_to_pkt_flags_map[4] = {
		0,  PKT_RX_L4_CKSUM_BAD, PKT_RX_IP_CKSUM_BAD,
		PKT_RX_IP_CKSUM_BAD | PKT_RX_L4_CKSUM_BAD
	};
	return error_to_pkt_flags_map[(rx_status >>
		E1000_RXD_ERR_CKSUM_BIT) & E1000_RXD_ERR_CKSUM_MSK];
}

uint16_t
eth_wm_recv_pkts(struct igb_rx_queue *rxq, struct rte_mbuf **rx_pkts,
	       uint16_t nb_pkts)
{
	struct e1000_rx_desc *rx_ring;
	struct e1000_rx_desc *rxdp;
	struct wm_rx_entry *sw_ring;
	struct wm_rx_entry *rxe;
	struct rte_mbuf *rxm;
	struct rte_mbuf *nmb;
	struct e1000_rx_desc rxd;
	uint64_t dma_addr;
	uint32_t staterr;
	uint32_t hlen_type_rss;
	uint16_t pkt_len;
	uint16_t rx_id;
	uint16_t nb_rx;
	uint16_t nb_hold;
	uint16_t pkt_flags;

	nb_rx = 0;
	nb_hold = 0;
	rx_id = rxq->rx_tail;
	rx_ring = rxq->rx_ring;
	sw_ring = rxq->sw_ring;
	while (nb_rx < nb_pkts) {
		/*
		 * The order of operations here is important as the DD status
		 * bit must not be read after any other descriptor fields.
		 * rx_ring and rxdp are pointing to volatile data so the order
		 * of accesses cannot be reordered by the compiler. If they were
		 * not volatile, they could be reordered which could lead to
		 * using invalid descriptor fields when read from rxd.
		 */
		rxdp = &rx_ring[rx_id];
		staterr = rxdp->status;
		if (! (staterr & E1000_RXD_STAT_DD))
			break;
		rxd = *rxdp;

		/*
		 * End of packet.
		 *
		 * If the E1000_RXD_STAT_EOP flag is not set, the RX packet is
		 * likely to be invalid and to be dropped by the various
		 * validation checks performed by the network stack.
		 *
		 * Allocate a new mbuf to replenish the RX ring descriptor.
		 * If the allocation fails:
		 *    - arrange for that RX descriptor to be the first one
		 *      being parsed the next time the receive function is
		 *      invoked [on the same queue].
		 *
		 *    - Stop parsing the RX ring and return immediately.
		 *
		 * This policy do not drop the packet received in the RX
		 * descriptor for which the allocation of a new mbuf failed.
		 * Thus, it allows that packet to be later retrieved if
		 * mbuf have been freed in the mean time.
		 * As a side effect, holding RX descriptors instead of
		 * systematically giving them back to the NIC may lead to
		 * RX ring exhaustion situations.
		 * However, the NIC can gracefully prevent such situations
		 * to happen by sending specific "back-pressure" flow control
		 * frames to its peer(s).
		 */
		PMD_RX_LOG(DEBUG, "\nport_id=%u queue_id=%u rx_id=%u "
			   "staterr=0x%x pkt_len=%u\n",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) staterr,
			   (unsigned) rte_le_to_cpu_16(rxd.length));

		nmb = rte_rxmbuf_alloc(rxq->mb_pool);
		if (nmb == NULL) {
			PMD_RX_LOG(DEBUG, "RX mbuf alloc failed port_id=%u "
				   "queue_id=%u\n", (unsigned) rxq->port_id,
				   (unsigned) rxq->queue_id);
			rte_eth_devices[rxq->port_id].data->rx_mbuf_alloc_failed++;
			break;
		}

		nb_hold++;
		rxe = &sw_ring[rx_id];
		rx_id++;
		if (rx_id == rxq->nb_rx_desc)
			rx_id = 0;

		/* Prefetch next mbuf while processing current one. */
		rte_wm_prefetch(sw_ring[rx_id].mbuf);

		/*
		 * When next RX descriptor is on a cache-line boundary,
		 * prefetch the next 4 RX descriptors and the next 8 pointers
		 * to mbufs.
		 */
		if ((rx_id & 0x3) == 0) {
			rte_wm_prefetch(&rx_ring[rx_id]);
			rte_wm_prefetch(&sw_ring[rx_id]);
		}

		rxm = rxe->mbuf;
		rxe->mbuf = nmb;
		dma_addr =
			rte_cpu_to_le_64(RTE_MBUF_DATA_DMA_ADDR_DEFAULT(nmb));
		rxdp->buffer_addr = dma_addr;

		/*
		 * Initialize the returned mbuf.
		 * 1) setup generic mbuf fields:
		 *    - number of segments,
		 *    - next segment,
		 *    - packet length,
		 *    - RX port identifier.
		 * 2) integrate hardware offload data, if any:
		 *    - RSS flag & hash,
		 *    - IP checksum flag,
		 *    - VLAN TCI, if any,
		 *    - error flags.
		 */
		pkt_len = (uint16_t) (rte_le_to_cpu_16(rxd.length) -
				      rxq->crc_len);
		rxm->pkt.data = (char*) rxm->buf_addr + RTE_PKTMBUF_HEADROOM;
		rte_packet_prefetch(rxm->pkt.data);
		rxm->pkt.nb_segs = 1;
		rxm->pkt.next = NULL;
		rxm->pkt.pkt_len = pkt_len;
		rxm->pkt.data_len = pkt_len;
		rxm->pkt.in_port = rxq->port_id;

		rxm->pkt.hash.rss = 0;
		hlen_type_rss = rte_le_to_cpu_32(0);
		/* Only valid if PKT_RX_VLAN_PKT set in pkt_flags */
		rxm->pkt.vlan_tci = rte_le_to_cpu_16(0);

		pkt_flags = rx_desc_hlen_type_rss_to_pkt_flags(hlen_type_rss);
		pkt_flags = (pkt_flags |
					rx_desc_status_to_pkt_flags(staterr));
		pkt_flags = (pkt_flags |
					rx_desc_error_to_pkt_flags(staterr));
		rxm->ol_flags = pkt_flags;

		/*
		 * Store the mbuf address into the next entry of the array
		 * of returned packets.
		 */
		rx_pkts[nb_rx++] = rxm;

		/* reset status so we get a fresh view of things next round */
		rxdp->status = 0;
	}
	rxq->rx_tail = rx_id;

	/*
	 * If the number of free RX descriptors is greater than the RX free
	 * threshold of the queue, advance the Receive Descriptor Tail (RDT)
	 * register.
	 * Update the RDT with the value of the last processed RX descriptor
	 * minus 1, to guarantee that the RDT register is never equal to the
	 * RDH register, which creates a "full" ring situtation from the
	 * hardware point of view...
	 */
	nb_hold = (uint16_t) (nb_hold + rxq->nb_rx_hold);
	if (nb_hold > rxq->rx_free_thresh) {
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_tail=%u "
			   "nb_hold=%u nb_rx=%u\n",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) nb_hold,
			   (unsigned) nb_rx);
		rx_id = (uint16_t) ((rx_id == 0) ?
				     (rxq->nb_rx_desc - 1) : (rx_id - 1));
		E1000_PCI_REG_WRITE(rxq->rdt_reg_addr, rx_id);
		nb_hold = 0;
	}
	rxq->nb_rx_hold = nb_hold;
	return (nb_rx);
}

/*
 * Rings setup and release.
 *
 * TDBA/RDBA should be aligned on 16 byte boundary. But TDLEN/RDLEN should be
 * multiple of 128 bytes. So we align TDBA/RDBA on 128 byte boundary.
 * This will also optimize cache line size effect.
 * H/W supports up to cache line size 128.
 */
#define IGB_ALIGN 128

/*
 * Maximum number of Ring Descriptors.
 *
 * Since RDLEN/TDLEN should be multiple of 128bytes, the number of ring
 * desscriptors should meet the following condition:
 *      (num_ring_desc * sizeof(struct e1000_rx/tx_desc)) % 128 == 0
 */
#define IGB_MIN_RING_DESC 32
#define IGB_MAX_RING_DESC 4096

static const struct rte_memzone *
ring_dma_zone_reserve(struct rte_eth_dev *dev, const char *ring_name,
		      uint16_t queue_id, uint32_t ring_size, int socket_id)
{
	char z_name[RTE_MEMZONE_NAMESIZE];
	const struct rte_memzone *mz;

	rte_snprintf(z_name, sizeof(z_name), "%s_%s_%d_%d",
			dev->driver->pci_drv.name, ring_name,
				dev->data->port_id, queue_id);
	mz = rte_memzone_lookup(z_name);
	if (mz)
		return mz;

	return rte_memzone_reserve_aligned(z_name, (uint64_t)ring_size,
			socket_id, 0, IGB_ALIGN);
}

static void
wm_tx_queue_release_mbufs(struct igb_tx_queue *txq)
{
	unsigned i;

	if (txq->sw_ring != NULL) {
		for (i = 0; i < txq->nb_tx_desc; i++) {
			if (txq->sw_ring[i].mbuf != NULL) {
				rte_pktmbuf_free_seg(txq->sw_ring[i].mbuf);
				txq->sw_ring[i].mbuf = NULL;
			}
		}
	}
}

static void
wm_tx_queue_release(struct igb_tx_queue *txq)
{
	wm_tx_queue_release_mbufs(txq);
        rte_free(txq->sw_ring);
        rte_free(txq);
}

int
wm_dev_tx_queue_alloc(struct rte_eth_dev *dev, uint16_t nb_queues)
{
	uint16_t i, old_nb_queues = dev->data->nb_tx_queues;
	struct igb_tx_queue **txq;

	if (dev->data->tx_queues == NULL) {
		dev->data->tx_queues = rte_zmalloc("ethdev->tx_queues",
				sizeof(struct igb_tx_queue *) * nb_queues,
							CACHE_LINE_SIZE);
		if (dev->data->tx_queues == NULL) {
			dev->data->nb_tx_queues = 0;
			return -ENOMEM;
		}
	} else {
		if (nb_queues < old_nb_queues)
			for (i = nb_queues; i < old_nb_queues; i++)
				wm_tx_queue_release(dev->data->tx_queues[i]);

		if (nb_queues != old_nb_queues) {
			txq = rte_realloc(dev->data->tx_queues,
				sizeof(struct igb_tx_queue *) * nb_queues,
							CACHE_LINE_SIZE);
			if (txq == NULL)
				return -ENOMEM;
			else
				dev->data->tx_queues = txq;
			if (nb_queues > old_nb_queues)
				memset(&(txq[old_nb_queues]), 0,
					sizeof(struct igb_tx_queue *) *
					(nb_queues - old_nb_queues));
		}
	}
	dev->data->nb_tx_queues = nb_queues;

	return 0;
}

static void
wm_reset_tx_queue_stat(struct igb_tx_queue *txq)
{
	txq->tx_head = 0;
	txq->tx_tail = 0;
	txq->ctx_curr = 0;
}

static void
wm_reset_tx_queue(struct igb_tx_queue *txq, struct rte_eth_dev *dev)
{
	struct wm_tx_entry *txe = txq->sw_ring;
	uint32_t size;
	uint16_t i, prev;
	struct e1000_hw *hw;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	size = sizeof(struct e1000_tx_desc) * txq->nb_tx_desc;
	/* Zero out HW ring memory */
	for (i = 0; i < size; i++) {
		((volatile char *)txq->tx_ring)[i] = 0;
	}

	/* Initialize ring entries */
	prev = txq->nb_tx_desc - 1;
	for (i = 0; i < txq->nb_tx_desc; i++) {
		struct e1000_tx_desc *txd = &(txq->tx_ring[i]);

		txd->upper.fields.status = E1000_TXD_STAT_DD;
		txe[i].mbuf = NULL;
		txe[i].last_id = i;
		txe[prev].next_id = i;
		prev = i;
	}

	wm_reset_tx_queue_stat(txq);
}

int
eth_wm_tx_queue_setup(struct rte_eth_dev *dev,
			 uint16_t queue_idx,
			 uint16_t nb_desc,
			 unsigned int socket_id,
			 const struct rte_eth_txconf *tx_conf)
{
	const struct rte_memzone *tz;
	struct igb_tx_queue *txq;
	struct e1000_hw     *hw;
	uint32_t size;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* just be really silly about it for now */
	if (nb_desc != 256) {
		return -EINVAL;
	}

	/*
	 * The tx_free_thresh and tx_rs_thresh values are not used in the 1G
	 * driver.
	 */
	if (tx_conf->tx_free_thresh != 0)
		RTE_LOG(WARNING, PMD,
			"The tx_free_thresh parameter is not "
			"used for the 1G driver.");
	if (tx_conf->tx_rs_thresh != 0)
		RTE_LOG(WARNING, PMD,
			"The tx_rs_thresh parameter is not "
			"used for the 1G driver.");
	if (tx_conf->tx_thresh.wthresh == 0)
		RTE_LOG(WARNING, PMD,
			"To improve 1G driver performance, consider setting "
			"the TX WTHRESH value to 4, 8, or 16.");

	/* Free memory prior to re-allocation if needed */
	if (dev->data->tx_queues[queue_idx] != NULL)
		wm_tx_queue_release(dev->data->tx_queues[queue_idx]);

	/* First allocate the tx queue data structure */
	txq = rte_zmalloc("ethdev TX queue", sizeof(struct igb_tx_queue),
							CACHE_LINE_SIZE);
	if (txq == NULL)
		return (-ENOMEM);

	size = sizeof(struct e1000_tx_desc) * nb_desc;
	tz = ring_dma_zone_reserve(dev, "tx_ring", queue_idx,
					size, socket_id);
	if (tz == NULL) {
		wm_tx_queue_release(txq);
		return (-ENOMEM);
	}

	txq->nb_tx_desc = nb_desc;
	txq->pthresh = tx_conf->tx_thresh.pthresh;
	txq->hthresh = tx_conf->tx_thresh.hthresh;
	txq->wthresh = tx_conf->tx_thresh.wthresh;
	txq->queue_id = queue_idx;
	txq->port_id = dev->data->port_id;

	txq->tdt_reg_addr = E1000_PCI_REG_ADDR(hw, E1000_TDT(queue_idx));
	txq->tx_ring_phys_addr = (uint64_t) tz->phys_addr;
	txq->tx_ring = (struct e1000_tx_desc *)tz->addr;

	/* Allocate software ring */
	txq->sw_ring = rte_zmalloc("txq->sw_ring",
				   sizeof(struct wm_tx_entry) * nb_desc,
				   CACHE_LINE_SIZE);
	if (txq->sw_ring == NULL) {
		wm_tx_queue_release(txq);
		return (-ENOMEM);
	}
	PMD_INIT_LOG(DEBUG, "sw_ring=%p hw_ring=%p dma_addr=0x%"PRIx64"\n",
		     txq->sw_ring, txq->tx_ring, txq->tx_ring_phys_addr);

	wm_reset_tx_queue(txq, dev);
	dev->tx_pkt_burst = eth_wm_xmit_pkts;
	dev->data->tx_queues[queue_idx] = txq;

	return (0);
}

static void
wm_rx_queue_release_mbufs(struct igb_rx_queue *rxq)
{
	unsigned i;

	if (rxq->sw_ring != NULL) {
		for (i = 0; i < rxq->nb_rx_desc; i++) {
			if (rxq->sw_ring[i].mbuf != NULL) {
				rte_pktmbuf_free_seg(rxq->sw_ring[i].mbuf);
				rxq->sw_ring[i].mbuf = NULL;
			}
		}
	}
}

static void
wm_rx_queue_release(struct igb_rx_queue *rxq)
{
	wm_rx_queue_release_mbufs(rxq);
	rte_free(rxq->sw_ring);
	rte_free(rxq);
}

int
wm_dev_rx_queue_alloc(struct rte_eth_dev *dev, uint16_t nb_queues)
{
	uint16_t i, old_nb_queues = dev->data->nb_rx_queues;
	struct igb_rx_queue **rxq;

	if (dev->data->rx_queues == NULL) {
		dev->data->rx_queues = rte_zmalloc("ethdev->rx_queues",
				sizeof(struct igb_rx_queue *) * nb_queues,
							CACHE_LINE_SIZE);
		if (dev->data->rx_queues == NULL) {
			dev->data->nb_rx_queues = 0;
			return -ENOMEM;
		}
	} else {
		for (i = nb_queues; i < old_nb_queues; i++) {
			wm_rx_queue_release(dev->data->rx_queues[i]);
			dev->data->rx_queues[i] = NULL;
		}
		if (nb_queues != old_nb_queues) {
			rxq = rte_realloc(dev->data->rx_queues,
				sizeof(struct igb_rx_queue *) * nb_queues,
							CACHE_LINE_SIZE);
			if (rxq == NULL)
				return -ENOMEM;
			else
				dev->data->rx_queues = rxq;
			if (nb_queues > old_nb_queues)
				memset(&(rxq[old_nb_queues]), 0,
					sizeof(struct igb_rx_queue *) *
					(nb_queues - old_nb_queues));
		}
	}
	dev->data->nb_rx_queues = nb_queues;

	return 0;
}

static void
wm_reset_rx_queue(struct igb_rx_queue *rxq)
{
	unsigned size;
	unsigned i;

	/* Zero out HW ring memory */
	size = sizeof(struct e1000_rx_desc) * rxq->nb_rx_desc;
	for (i = 0; i < size; i++) {
		((volatile char *)rxq->rx_ring)[i] = 0;
	}

	rxq->rx_tail = 0;
	rxq->pkt_first_seg = NULL;
	rxq->pkt_last_seg = NULL;
}

int
eth_wm_rx_queue_setup(struct rte_eth_dev *dev,
			 uint16_t queue_idx,
			 uint16_t nb_desc,
			 unsigned int socket_id,
			 const struct rte_eth_rxconf *rx_conf,
			 struct rte_mempool *mp)
{
	const struct rte_memzone *rz;
	struct igb_rx_queue *rxq;
	struct e1000_hw     *hw;
	unsigned int size;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	if (nb_desc != 256)
		return -EINVAL;

	/* Free memory prior to re-allocation if needed */
	if (dev->data->rx_queues[queue_idx] != NULL) {
		wm_rx_queue_release(dev->data->rx_queues[queue_idx]);
		dev->data->rx_queues[queue_idx] = NULL;
	}

	/* First allocate the RX queue data structure. */
	rxq = rte_zmalloc("ethdev RX queue", sizeof(struct igb_rx_queue),
			  CACHE_LINE_SIZE);
	if (rxq == NULL)
		return (-ENOMEM);
	rxq->mb_pool = mp;
	rxq->nb_rx_desc = nb_desc;
	rxq->pthresh = rx_conf->rx_thresh.pthresh;
	rxq->hthresh = rx_conf->rx_thresh.hthresh;
	rxq->wthresh = rx_conf->rx_thresh.wthresh;
	rxq->rx_free_thresh = rx_conf->rx_free_thresh;
	rxq->queue_id = queue_idx;
	rxq->port_id = dev->data->port_id;
	rxq->crc_len = (uint8_t) ((dev->data->dev_conf.rxmode.hw_strip_crc) ? 0 :
				  ETHER_CRC_LEN);

	size = sizeof(struct e1000_rx_desc) * nb_desc;
	rz = ring_dma_zone_reserve(dev, "rx_ring", queue_idx, size, socket_id);
	if (rz == NULL) {
		wm_rx_queue_release(rxq);
		return (-ENOMEM);
	}
	rxq->rdt_reg_addr = E1000_PCI_REG_ADDR(hw, E1000_RDT(queue_idx));
	rxq->rx_ring_phys_addr = (uint64_t) rz->phys_addr;
	rxq->rx_ring = (struct e1000_rx_desc *)rz->addr;

	/* Allocate software ring. */
	rxq->sw_ring = rte_zmalloc("rxq->sw_ring",
				   sizeof(struct wm_rx_entry) * nb_desc,
				   CACHE_LINE_SIZE);
	if (rxq->sw_ring == NULL) {
		wm_rx_queue_release(rxq);
		return (-ENOMEM);
	}
	PMD_INIT_LOG(DEBUG, "sw_ring=%p hw_ring=%p dma_addr=0x%"PRIx64"\n",
		     rxq->sw_ring, rxq->rx_ring, rxq->rx_ring_phys_addr);

	dev->data->rx_queues[queue_idx] = rxq;
	wm_reset_rx_queue(rxq);

	return 0;
}

void
wm_dev_clear_queues(struct rte_eth_dev *dev)
{
	uint16_t i;
	struct igb_tx_queue *txq;
	struct igb_rx_queue *rxq;

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		txq = dev->data->tx_queues[i];
		wm_tx_queue_release_mbufs(txq);
		wm_reset_tx_queue(txq, dev);
	}

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		rxq = dev->data->rx_queues[i];
		wm_rx_queue_release_mbufs(rxq);
		wm_reset_rx_queue(rxq);
	}
}

/**
 * Receive Side Scaling (RSS).
 * See section 7.1.1.7 in the following document:
 *     "Intel 82576 GbE Controller Datasheet" - Revision 2.45 October 2009
 *
 * Principles:
 * The source and destination IP addresses of the IP header and the source and
 * destination ports of TCP/UDP headers, if any, of received packets are hashed
 * against a configurable random key to compute a 32-bit RSS hash result.
 * The seven (7) LSBs of the 32-bit hash result are used as an index into a
 * 128-entry redirection table (RETA).  Each entry of the RETA provides a 3-bit
 * RSS output index which is used as the RX queue index where to store the
 * received packets.
 * The following output is supplied in the RX write-back descriptor:
 *     - 32-bit result of the Microsoft RSS hash function,
 *     - 4-bit RSS type field.
 */

/*
 * RSS random key supplied in section 7.1.1.7.3 of the Intel 82576 datasheet.
 * Used as the default key.
 */
static uint8_t rss_intel_key[40] = {
	0x6D, 0x5A, 0x56, 0xDA, 0x25, 0x5B, 0x0E, 0xC2,
	0x41, 0x67, 0x25, 0x3D, 0x43, 0xA3, 0x8F, 0xB0,
	0xD0, 0xCA, 0x2B, 0xCB, 0xAE, 0x7B, 0x30, 0xB4,
	0x77, 0xCB, 0x2D, 0xA3, 0x80, 0x30, 0xF2, 0x0C,
	0x6A, 0x42, 0xB7, 0x3B, 0xBE, 0xAC, 0x01, 0xFA,
};

static void
wm_rss_disable(struct rte_eth_dev *dev)
{
	struct e1000_hw *hw;
	uint32_t mrqc;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	mrqc = E1000_READ_REG(hw, E1000_MRQC);
	mrqc &= ~E1000_MRQC_ENABLE_MASK;
	E1000_WRITE_REG(hw, E1000_MRQC, mrqc);
}

static void
wm_rss_configure(struct rte_eth_dev *dev)
{
	struct e1000_hw *hw;
	uint8_t *hash_key;
	uint32_t rss_key;
	uint32_t mrqc;
	uint32_t shift;
	uint16_t rss_hf;
	uint16_t i;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	rss_hf = dev->data->dev_conf.rx_adv_conf.rss_conf.rss_hf;
	if (rss_hf == 0) /* Disable RSS. */ {
		wm_rss_disable(dev);
		return;
	}
	hash_key = dev->data->dev_conf.rx_adv_conf.rss_conf.rss_key;
	if (hash_key == NULL)
		hash_key = rss_intel_key; /* Default hash key. */

	/* Fill in RSS hash key. */
	for (i = 0; i < 10; i++) {
		rss_key  = hash_key[(i * 4)];
		rss_key |= hash_key[(i * 4) + 1] << 8;
		rss_key |= hash_key[(i * 4) + 2] << 16;
		rss_key |= hash_key[(i * 4) + 3] << 24;
		E1000_WRITE_REG_ARRAY(hw, E1000_RSSRK(0), i, rss_key);
	}

	/* Fill in redirection table. */
	shift = (hw->mac.type == e1000_82575) ? 6 : 0;
	for (i = 0; i < 128; i++) {
		union e1000_reta {
			uint32_t dword;
			uint8_t  bytes[4];
		} reta;
		uint8_t q_idx;

		q_idx = (uint8_t) ((dev->data->nb_rx_queues > 1) ?
				   i % dev->data->nb_rx_queues : 0);
		reta.bytes[i & 3] = (uint8_t) (q_idx << shift);
		if ((i & 3) == 3)
			E1000_WRITE_REG(hw, E1000_RETA(i >> 2), reta.dword);
	}

	/* Set configured hashing functions in MRQC register. */
	mrqc = E1000_MRQC_ENABLE_RSS_4Q; /* RSS enabled. */
	if (rss_hf & ETH_RSS_IPV4)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV4;
	if (rss_hf & ETH_RSS_IPV4_TCP)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV4_TCP;
	if (rss_hf & ETH_RSS_IPV6)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6;
	if (rss_hf & ETH_RSS_IPV6_EX)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6_EX;
	if (rss_hf & ETH_RSS_IPV6_TCP)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6_TCP;
	if (rss_hf & ETH_RSS_IPV6_TCP_EX)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6_TCP_EX;
	if (rss_hf & ETH_RSS_IPV4_UDP)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV4_UDP;
	if (rss_hf & ETH_RSS_IPV6_UDP)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6_UDP;
	if (rss_hf & ETH_RSS_IPV6_UDP_EX)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6_UDP_EX;
	E1000_WRITE_REG(hw, E1000_MRQC, mrqc);
}

/*********************************************************************
 *
 *  Enable receive unit.
 *
 **********************************************************************/

static int
wm_alloc_rx_queue_mbufs(struct igb_rx_queue *rxq)
{
	struct wm_rx_entry *rxe = rxq->sw_ring;
	uint64_t dma_addr;
	unsigned i;

	/* Initialize software ring entries. */
	for (i = 0; i < rxq->nb_rx_desc; i++) {
		struct e1000_rx_desc *rxd;
		struct rte_mbuf *mbuf = rte_rxmbuf_alloc(rxq->mb_pool);

		if (mbuf == NULL) {
			PMD_INIT_LOG(ERR, "RX mbuf alloc failed "
				"queue_id=%hu\n", rxq->queue_id);
			wm_rx_queue_release(rxq);
			return (-ENOMEM);
		}
		dma_addr =
			rte_cpu_to_le_64(RTE_MBUF_DATA_DMA_ADDR_DEFAULT(mbuf));
		rxd = &rxq->rx_ring[i];
		rxd->buffer_addr = dma_addr;
		rxd->buffer_addr = dma_addr;
		rxe[i].mbuf = mbuf;
	}

	return 0;
}

int
eth_wm_rx_init(struct rte_eth_dev *dev)
{
	struct e1000_hw     *hw;
	struct igb_rx_queue *rxq;
	struct rte_pktmbuf_pool_private *mbp_priv;
	uint32_t rctl;
	uint32_t rxcsum;
	uint32_t srrctl;
	uint16_t buf_size;
	uint16_t rctl_bsize;
	uint64_t bus_addr;
	uint32_t rxdctl;
	int ret;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/*
	 * Make sure receives are disabled while setting
	 * up the descriptor ring.
	 */
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);

	assert(hw->mac.type == e1000_82540);
	/* no interrupt throttling */
	E1000_WRITE_REG(hw, E1000_RADV, 0);
	E1000_WRITE_REG(hw, E1000_ITR, 0);

	/* Configure and enable each RX queue. */
	rctl_bsize = 0;
	dev->rx_pkt_burst = eth_wm_recv_pkts;

	rxq = dev->data->rx_queues[0];

	/* Allocate buffers for descriptor rings and set up queue */
	ret = wm_alloc_rx_queue_mbufs(rxq);
	if (ret) {
		wm_dev_clear_queues(dev);
		return ret;
	}

	/*
	 * Reset crc_len in case it was changed after queue setup by a
	 *  call to configure
	 */
	rxq->crc_len = (uint8_t)(dev->data->dev_conf.rxmode.hw_strip_crc
	    ?  0 : ETHER_CRC_LEN);

	bus_addr = rxq->rx_ring_phys_addr;
	E1000_WRITE_REG(hw, E1000_RDLEN(0),
	    rxq->nb_rx_desc * sizeof(struct e1000_rx_desc));
	E1000_WRITE_REG(hw, E1000_RDBAH(0), (uint32_t)(bus_addr >> 32));
	E1000_WRITE_REG(hw, E1000_RDBAL(0), (uint32_t)bus_addr);

	/*
	 * Configure RSS if device configured with multiple RX queues.
	 */
	if (dev->data->nb_rx_queues > 1)
		wm_rss_configure(dev);
	else
		wm_rss_disable(dev);

	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_LBM_NO |
		E1000_RCTL_RDMTS_HALF |
		(hw->mac.mc_filter_type << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_SZ_2048;

	/* Make sure VLAN Filters are off. */
	rctl &= ~E1000_RCTL_VFE;
	/* Don't store bad packets. */
	rctl &= ~E1000_RCTL_SBP;

	/* Enable Receives. */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);

	/*
	 * Setup the HW Rx Head and Tail Descriptor Pointers.
	 * This needs to be done after enable.
	 */
	rxq = dev->data->rx_queues[0];
	E1000_WRITE_REG(hw, E1000_RDH(0), 0);
	E1000_WRITE_REG(hw, E1000_RDT(0), rxq->nb_rx_desc - 1);

	return 0;
}

/*********************************************************************
 *
 *  Enable transmit unit.
 *
 **********************************************************************/
void
eth_wm_tx_init(struct rte_eth_dev *dev)
{
	struct e1000_hw     *hw;
	struct igb_tx_queue *txq;
	uint32_t tctl;
	uint32_t tipg;
	uint16_t i;
	uint64_t bus_addr;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Setup the Base and Length of the Tx Descriptor Rings. */
	txq = dev->data->tx_queues[i];
	bus_addr = txq->tx_ring_phys_addr;

	E1000_WRITE_REG(hw, E1000_TDLEN(i),
			txq->nb_tx_desc *
			sizeof(struct e1000_tx_desc));
	E1000_WRITE_REG(hw, E1000_TDBAH(i),
			(uint32_t)(bus_addr >> 32));
	E1000_WRITE_REG(hw, E1000_TDBAL(i), (uint32_t)bus_addr);

	/* Setup the HW Tx Head and Tail descriptor pointers. */
	E1000_WRITE_REG(hw, E1000_TDT(i), 0);
	E1000_WRITE_REG(hw, E1000_TDH(i), 0);

	assert(hw->mac.type == e1000_82540);
	tipg = DEFAULT_82543_TIPG_IPGT_COPPER;
	tipg |= DEFAULT_82543_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
	tipg |= DEFAULT_82543_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
	E1000_WRITE_REG(hw, E1000_TIPG, tipg);
	E1000_WRITE_REG(hw, E1000_TIDV, 0);

	/* Program the Transmit Control Register. */
	tctl = E1000_READ_REG(hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= (E1000_TCTL_PSP | E1000_TCTL_RTLC | E1000_TCTL_EN |
		 (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT));

	e1000_config_collision_dist(hw);

	/* This write will effectively turn on the transmit unit. */
	E1000_WRITE_REG(hw, E1000_TCTL, tctl);
}
