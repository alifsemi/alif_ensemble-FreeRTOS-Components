/*
 * FreeRTOS-Plus-TCP NetworkInterface Driver for Alif Ethernet MAC
 *
 * Author   : Silesh C V <silesh@alifsemi.com>
 *
 * Copyright (C) 2022 ALIF SEMICONDUCTOR
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ALIF SEMICONDUCTOR nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file mac_hw.h
 * @brief Header file for the Alif Ethernet MAC NetworkInterface Driver.
 */

#ifndef _MAC_HW_H_
#define _MAC_HW_H_

#include CMSIS_device_header
#include "system_utils.h"

#define RX_DESC_COUNT   12      /**< Rx DMA descriptor count */
#define TX_DESC_COUNT   12      /**< Tx DMA descriptor count */

#define ETH_BUF_SIZE    1536

/** @brief Rx/Tx Dma Descriptor. */
typedef struct {
    uint32_t ulDes0;
    uint32_t ulDes1;
    uint32_t ulDes2;
    uint32_t ulDes3;
} DmaDesc;

/** @brief Ethernet Configuration. */
typedef struct {
    uint8_t ucAutoNegCtrl;  /**< Enable/Disable autonegotiation. Can be set to AN_DISABLE or AN_ENABLE */
    uint8_t ucSpeed;        /**< Speed configuration. Legal values are ETH_SPEED_10M and ETH_SPEED_100M */
    uint8_t ucDuplex;       /**< Duplex configuration. Legal values are ETH_DUPLEX_HALF and ETH_DUPLEX_FULL */
} EthConfig;

#define AN_DISABLE                      0x0 /**< Disable autonegotiation. */
#define AN_ENABLE                       0x1 /**< Enable autonegotiation. */

#define ETH_SPEED_10M		            0x0 /**< 10Mbps Speed. */
#define ETH_SPEED_100M		            0x1 /**< 100mbps Speed. */

#define ETH_DUPLEX_HALF		            0x0 /**< Half Duplex. */
#define ETH_DUPLEX_FULL		            0x1 /**< Full Duplex. */


/* Select the PHY Ref clock as the ETH 50M clock */
#define SEL_PHY_REFCLK                  (1U << 0)

/** \brief MAC register map. */
typedef struct {
    uint32_t MAC_CONFIG;
    uint32_t MAC_EXT_CONFIG;
    uint32_t MAC_PACKET_FILTER;
    uint32_t RESERVED_0;
    uint32_t MAC_HASH_TAB_0;
    uint32_t MAC_HASH_TAB_1;
    uint32_t RESERVED_1[22];
    uint32_t MAC_Q0_TX_FLOW_CTRL;
    uint32_t RESERVED[7];
    uint32_t MAC_RX_FLOW_CTRL;
    uint32_t RESERVED_2[3];
    uint32_t MAC_RXQ_CTRL_0;
    uint32_t MAC_RXQ_CTRL_1;
    uint32_t MAC_RXQ_CTRL_2;
    uint32_t MAC_RXQ_CTRL_3;
    uint32_t MAC_INT_STATUS;
    uint32_t MAC_INT_ENABLE;
    uint32_t RESERVED_3[2];
    uint32_t MAC_PMT_CTRL_STS;
    uint32_t RESERVED_4[22];
    uint32_t MAC_HW_FEATURE_0;
    uint32_t MAC_HW_FEATURE_1;
    uint32_t MAC_HW_FEATURE_2;
    uint32_t MAC_HW_FEATURE_3;
    uint32_t RESERVED_5[53];
    uint32_t MAC_MDIO_ADDR;
    uint32_t MAC_MDIO_DATA;
    uint32_t RESERVED_6[62];
    uint32_t MAC_ADDR_HIGH_0;
    uint32_t MAC_ADDR_LOW_0;
    uint32_t RESERVED_7[510];
    uint32_t MAC_TIMESTAMP_CONTROL;
    uint32_t MAC_SUB_SEC_INCR;
    uint32_t MAC_SYS_TIME_SEC;
    uint32_t MAC_SYS_TIME_NANOSEC;
    uint32_t MAC_SYS_TIME_SEC_UPD;
    uint32_t MAC_SYS_TIME_NANOSEC_UPD;
    uint32_t MAC_TIMESTAMP_ADDEND;
    uint32_t RESERVED_8[25];
    uint32_t MAC_PPS_TARGET_SEC;
    uint32_t MAC_PPS_TARGET_NANOSEC;
    uint32_t RESERVED_9[94];
    uint32_t MTL_TXQ0_OP_MODE;
    uint32_t RESERVED_10[11];
    uint32_t MTL_RXQ0_OP_MODE;
    uint32_t RESERVED_11[179];
    uint32_t DMA_BUS_MODE;
    uint32_t DMA_SYS_BUS_MODE;
    uint32_t DMA_STATUS;
    uint32_t RESERVED_12[61];
    uint32_t DMA_CH0_CTRL;
    uint32_t DMA_CH0_TX_CTRL;
    uint32_t DMA_CH0_RX_CTRL;
    uint32_t RESERVED_13;
    uint32_t DMA_CH0_TX_BASE_ADDR_HIGH;
    uint32_t DMA_CH0_TX_BASE_ADDR;
    uint32_t DMA_CH0_RX_BASE_ADDR_HIGH;
    uint32_t DMA_CH0_RX_BASE_ADDR;
    uint32_t DMA_CH0_TX_END_ADDR;
    uint32_t RESERVED_14;
    uint32_t DMA_CH0_RX_END_ADDR;
    uint32_t DMA_CHO_TX_RING_LEN;
    uint32_t DMA_CH0_RX_RING_LEN;
    uint32_t DMA_CH0_INT_ENABLE;
    uint32_t RESERVED_15[10];
    uint32_t DMA_CH0_STATUS;
} DW_EQOS_REGS;

/**@brief DWC Eqos MAC device */
typedef struct {
    volatile DW_EQOS_REGS *pxRegs;              /**< Registers of the Ethernet MAC instance. */
    IRQn_Type xIrq;                             /**< IRQ number of the Ethernet MAC instance */
    DmaDesc *RxDescs;                           /**< Array of Rx DMA descriptors. */
    DmaDesc *TxDescs;                           /**< Array of Tx DMA descriptors. */
    NetworkBufferDescriptor_t **pxRxBufferDescs;/**< Array of Rx Buffer Descriptors. */
    uint32_t ulCurRxDescId;                     /**< Index of the current Rx DMA descriptor. */
    uint32_t ulCurTxDescId;                     /**< Index of the current Tx DMA descriptor. */
    EthConfig Config;                           /**< Configuration information (Speed/Duplex/Autonegotiation). */
} DwEqosDev;

/*  MAC register fields */

/* RX Queues Routing */
#define EQOS_MAC_RXQCTRL_AVCPQ_MASK	                    MASK(2, 0)
#define EQOS_MAC_RXQCTRL_AVCPQ_SHIFT	                0
#define EQOS_MAC_RXQCTRL_PTPQ_MASK	                    MASK(6, 4)
#define EQOS_MAC_RXQCTRL_PTPQ_SHIFT	                    4
#define EQOS_MAC_RXQCTRL_DCBCPQ_MASK	                MASK(10, 8)
#define EQOS_MAC_RXQCTRL_DCBCPQ_SHIFT	                8
#define EQOS_MAC_RXQCTRL_UPQ_MASK	                    MASK(14, 12)
#define EQOS_MAC_RXQCTRL_UPQ_SHIFT	                    12
#define EQOS_MAC_RXQCTRL_MCBCQ_MASK	                    MASK(18, 16)
#define EQOS_MAC_RXQCTRL_MCBCQ_SHIFT	                16
#define EQOS_MAC_RXQCTRL_MCBCQEN	                    BIT(20)
#define EQOS_MAC_RXQCTRL_MCBCQEN_SHIFT	                20
#define EQOS_MAC_RXQCTRL_TACPQE		                    BIT(21)
#define EQOS_MAC_RXQCTRL_TACPQE_SHIFT	                21
#define EQOS_MAC_RXQCTRL_FPRQ		                    MASK(26, 24)
#define EQOS_MAC_RXQCTRL_FPRQ_SHIFT	                    24

#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT                 0
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_MASK                  3
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_NOT_ENABLED           0
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB           2
#define EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_AV            1

#define EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT               16
#define EQOS_MAC_Q0_TX_FLOW_CTRL_PT_MASK                0xffff
#define EQOS_MAC_Q0_TX_FLOW_CTRL_TFE                    BIT(1)

#define EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT             16
#define EQOS_DMA_CH0_TX_CONTROL_TXPBL_MASK              0x3f
#define EQOS_DMA_CH0_TX_CONTROL_OSP                     BIT(4)
#define EQOS_DMA_CH0_TX_CONTROL_ST                      BIT(0)

#define EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT             16
#define EQOS_DMA_CH0_RX_CONTROL_RXPBL_MASK              0x3f
#define EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT              1
#define EQOS_DMA_CH0_RX_CONTROL_RBSZ_MASK               0x3fff
#define EQOS_DMA_CH0_RX_CONTROL_SR                      BIT(0)

#define EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT           16
#define EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_MASK            0xf
#define EQOS_DMA_SYSBUS_MODE_WR_OSR_LMT_SHIFT           24
#define EQOS_DMA_SYSBUS_MODE_EAME                       BIT(11)
#define EQOS_DMA_SYSBUS_MODE_ONEKBBE                    BIT(13)
#define EQOS_DMA_SYSBUS_MODE_BLEN16                     BIT(3)
#define EQOS_DMA_SYSBUS_MODE_BLEN8                      BIT(2)
#define EQOS_DMA_SYSBUS_MODE_BLEN4                      BIT(1)

/* MAC Packet Filtering */
#define EQOS_MAC_PACKET_FILTER_PR                       BIT(0)
#define EQOS_MAC_PACKET_FILTER_HMC                      BIT(2)
#define EQOS_MAC_PACKET_FILTER_PM                       BIT(4)
#define EQOS_MAC_PACKET_FILTER_DBF                      BIT(5)
#define EQOS_MAC_PACKET_FILTER_PCF                      BIT(7)
#define EQOS_MAC_PACKET_FILTER_HPF                      BIT(10)
#define EQOS_MAC_PACKET_FILTER_VTFE                     BIT(16)
#define EQOS_MAC_PACKET_FILTER_IPFE                     BIT(20)
#define EQOS_MAC_PACKET_FILTER_RA                       BIT(31)

#define EQOS_MAC_MAX_PERFECT_ADDRESSES                  128

/* MAC PMT Control Status */
#define EQOS_MAC_PMT_CTRL_STS_MGKPKTEN	                BIT(1)
#define EQOS_MAC_PMT_CTRL_STS_PWRDWN	                BIT(0)

/* MAC Flow Control RX */
#define EQOS_MAC_RX_FLOW_CTRL_RFE                       BIT(0)

/* MAC Flow Control TX */
#define EQOS_MAC_TX_FLOW_CTRL_TFE                       BIT(1)
#define EQOS_MAC_TX_FLOW_CTRL_PT_SHIFT                  16

/* MAC config */
#define EQOS_MAC_CONFIG_ARPEN                           BIT(31)
#define EQOS_MAC_CONFIG_SARC                            MASK(30, 28)
#define EQOS_MAC_CONFIG_SARC_SHIFT                      28
#define EQOS_MAC_CONFIG_IPC                             BIT(27)
#define EQOS_MAC_CONFIG_IPG                             MASK(26, 24)
#define EQOS_MAC_CONFIG_IPG_SHIFT                       24
#define EQOS_MAC_CONFIG_2K                              BIT(22)
#define EQOS_MAC_CONFIG_CST                             BIT(21)
#define EQOS_MAC_CONFIG_ACS                             BIT(20)
#define EQOS_MAC_CONFIG_WD                              BIT(19)
#define EQOS_MAC_CONFIG_BE                              BIT(18)
#define EQOS_MAC_CONFIG_JD                              BIT(17)
#define EQOS_MAC_CONFIG_JE                              BIT(16)
#define EQOS_MAC_CONFIG_PS                              BIT(15)
#define EQOS_MAC_CONFIG_FES                             BIT(14)
#define EQOS_MAC_CONFIG_FES_SHIFT                       14
#define EQOS_MAC_CONFIG_DM                              BIT(13)
#define EQOS_MAC_CONFIG_LM                              BIT(12)
#define EQOS_MAC_CONFIG_DCRS                            BIT(9)
#define EQOS_MAC_CONFIG_TE                              BIT(1)
#define EQOS_MAC_CONFIG_RE                              BIT(0)

#define EQOS_MAC_MDIO_ADDR_PA_SHIFT	                    21
#define EQOS_MAC_MDIO_ADDR_RDA_SHIFT	                16
#define EQOS_MAC_MDIO_ADDR_CR_SHIFT	                    8
/* MDC clock divisors for different CSR clk values */
#define EQOS_MAC_MDIO_ADDR_CR_60_100	                0
#define EQOS_MAC_MDIO_ADDR_CR_100_150	                1
#define EQOS_MAC_MDIO_ADDR_CR_20_35	                    2
#define EQOS_MAC_MDIO_ADDR_CR_35_60	                    3
#define EQOS_MAC_MDIO_ADDR_CR_150_250	                4
#define EQOS_MAC_MDIO_ADDR_CR_250_300	                5
#define EQOS_MAC_MDIO_ADDR_CR_300_500	                6
#define EQOS_MAC_MDIO_ADDR_CR_500_800   	            7
#define EQOS_MAC_MDIO_ADDR_SKAP		                    BIT(4)
#define EQOS_MAC_MDIO_ADDR_GOC_SHIFT	                2
#define EQOS_MAC_MDIO_ADDR_GOC_READ	                    3
#define EQOS_MAC_MDIO_ADDR_GOC_WRITE	                1
#define EQOS_MAC_MDIO_ADDR_C45E		                    BIT(1)
#define EQOS_MAC_MDIO_ADDR_GB		                    BIT(0)

#define EQOS_MAC_MDIO_DATA_GD_MASK                      0xffff

#define EQOS_MAC_ADDR_HIGH_AE                           BIT(31)

#define MAC_INT_EN_TSIE                                 BIT(12)
#define MAC_INT_EN_PMTIE                                BIT(4)

#define EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT           6
#define EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_MASK            0x1f
#define EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT           0
#define EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_MASK            0x1f

/*  MTL registers */
#define MTL_FRPE                                        BIT(15)
#define MTL_OPERATION_SCHALG_MASK                       MASK(6, 5)
#define MTL_OPERATION_SCHALG_WRR                        (0x0 << 5)
#define MTL_OPERATION_SCHALG_WFQ                        (0x1 << 5)
#define MTL_OPERATION_SCHALG_DWRR                       (0x2 << 5)
#define MTL_OPERATION_SCHALG_SP                         (0x3 << 5)
#define MTL_OPERATION_RAA                               BIT(2)
#define MTL_OPERATION_RAA_SP                            (0x0 << 2)
#define MTL_OPERATION_RAA_WSP                           (0x1 << 2)

#define MTL_OP_MODE_RSF                                 BIT(5)
#define MTL_OP_MODE_FEP		    	                    BIT(4)
#define MTL_OP_MODE_FUP		                        	BIT(3)
#define MTL_OP_MODE_TXQEN_MASK                          MASK(3, 2)
#define MTL_OP_MODE_TXQEN_AV                            BIT(2)
#define MTL_OP_MODE_TXQEN                               BIT(3)
#define MTL_OP_MODE_TSF                                 BIT(1)
#define MTL_OP_MODE_TTC_64B				                (1 << 4)

#define MTL_TXQ0_OPERATION_MODE_TQS_SHIFT               16
#define MTL_TXQ0_OPERATION_MODE_TQS_MASK                0x1ff

#define EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT          20
#define EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK           0x3ff


/* DMA Bus Mode bitmap */
#define DMA_BUS_MODE_SFT_RESET		                    BIT(0)

/* DMA SYS Bus Mode bitmap */
#define DMA_BUS_MODE_SPH		                        BIT(24)
#define DMA_BUS_MODE_PBL		                        BIT(16)
#define DMA_BUS_MODE_PBL_SHIFT		                    16
#define DMA_BUS_MODE_RPBL_SHIFT		                    16
#define DMA_BUS_MODE_MB			                        BIT(14)
#define DMA_BUS_MODE_FB			                        BIT(0)

/* DMA Interrupt top status */
#define DMA_STATUS_MAC			                        BIT(17)
#define DMA_STATUS_MTL			                        BIT(16)
#define DMA_STATUS_CHAN7		                        BIT(7)
#define DMA_STATUS_CHAN6		                        BIT(6)
#define DMA_STATUS_CHAN5		                        BIT(5)
#define DMA_STATUS_CHAN4		                        BIT(4)
#define DMA_STATUS_CHAN3		                        BIT(3)
#define DMA_STATUS_CHAN2		                        BIT(2)
#define DMA_STATUS_CHAN1		                        BIT(1)
#define DMA_STATUS_CHAN0		                        BIT(0)

/* definitions for DMA channels */
/* DMA Control X */
#define DMA_CONTROL_SPH                                 BIT(24)
#define DMA_CONTROL_MSS_MASK                            MASK(13, 0)

#define DMA_CHAN_CONTROL_PBLX8		                    BIT(16)

/* DMA Tx Channel X Control register defines */
#define DMA_CONTROL_EDSE                                BIT(28)
#define DMA_CONTROL_TSE                                 BIT(12)
#define DMA_CONTROL_OSP                                 BIT(4)
#define DMA_CONTROL_ST                                  BIT(0)

/* DMA Rx Channel X Control register defines */
#define DMA_CONTROL_SR                                  BIT(0)
#define DMA_RBSZ_MASK                                   MASK(14, 1)
#define DMA_RBSZ_SHIFT                                  1

/* Interrupt status per channel */
#define DMA_CHAN_STATUS_REB                             MASK(21, 19)
#define DMA_CHAN_STATUS_REB_SHIFT                       19
#define DMA_CHAN_STATUS_TEB                             MASK(18, 16)
#define DMA_CHAN_STATUS_TEB_SHIFT                       16
#define DMA_CHAN_STATUS_NIS                             BIT(15)
#define DMA_CHAN_STATUS_AIS                             BIT(14)
#define DMA_CHAN_STATUS_CDE                             BIT(13)
#define DMA_CHAN_STATUS_FBE                             BIT(12)
#define DMA_CHAN_STATUS_ERI                             BIT(11)
#define DMA_CHAN_STATUS_ETI                             BIT(10)
#define DMA_CHAN_STATUS_RWT                             BIT(9)
#define DMA_CHAN_STATUS_RPS                             BIT(8)
#define DMA_CHAN_STATUS_RBU                             BIT(7)
#define DMA_CHAN_STATUS_RI                              BIT(6)
#define DMA_CHAN_STATUS_TBU                             BIT(2)
#define DMA_CHAN_STATUS_TPS                             BIT(1)
#define DMA_CHAN_STATUS_TI                              BIT(0)

/* Interrupt enable bits per channel */
#define DMA_CHAN_INTR_ENA_NIE                           BIT(15)
#define DMA_CHAN_INTR_ENA_AIE                           BIT(14)
#define DMA_CHAN_INTR_ENA_CDE                           BIT(13)
#define DMA_CHAN_INTR_ENA_FBE                           BIT(12)
#define DMA_CHAN_INTR_ENA_ERE                           BIT(11)
#define DMA_CHAN_INTR_ENA_ETE                           BIT(10)
#define DMA_CHAN_INTR_ENA_RWE                           BIT(9)
#define DMA_CHAN_INTR_ENA_RSE                           BIT(8)
#define DMA_CHAN_INTR_ENA_RBUE                          BIT(7)
#define DMA_CHAN_INTR_ENA_RIE                           BIT(6)
#define DMA_CHAN_INTR_ENA_TBUE                          BIT(2)
#define DMA_CHAN_INTR_ENA_TSE                           BIT(1)
#define DMA_CHAN_INTR_ENA_TIE                           BIT(0)

/* Descriptor definitions */
/* TDES2 (read format) */
#define TDES2_BUFFER1_SIZE_MASK		                    MASK(13, 0)
#define TDES2_VLAN_TAG_MASK		                        MASK(15, 14)
#define TDES2_VLAN_TAG_SHIFT		                    14
#define TDES2_BUFFER2_SIZE_MASK		                    MASK(29, 16)
#define TDES2_BUFFER2_SIZE_MASK_SHIFT	                16
#define TDES3_IVTIR_MASK		                        MASK(19, 18)
#define TDES3_IVTIR_SHIFT		                        18
#define TDES3_IVLTV			                            BIT(17)
#define TDES2_TIMESTAMP_ENABLE		                    BIT(30)
#define TDES2_IVT_MASK			                        MASK(31, 16)
#define TDES2_IVT_SHIFT			                        16
#define TDES2_INTERRUPT_ON_COMPLETION	                BIT(31)

/* TDES3 (read format) */
#define TDES3_PACKET_SIZE_MASK		                    MASK(14, 0)
#define TDES3_VLAN_TAG			                        MASK(15, 0)
#define TDES3_VLTV			                            BIT(16)
#define TDES3_CHECKSUM_INSERTION_MASK	                MASK(17, 16)
#define TDES3_CHECKSUM_INSERTION_SHIFT	                16
#define TDES3_TCP_PKT_PAYLOAD_MASK	                    MASK(17, 0)
#define TDES3_TCP_SEGMENTATION_ENABLE	                BIT(18)
#define TDES3_HDR_LEN_SHIFT		                        19
#define TDES3_SLOT_NUMBER_MASK		                    MASK(22, 19)
#define TDES3_SA_INSERT_CTRL_MASK	                    MASK(25, 23)
#define TDES3_SA_INSERT_CTRL_SHIFT	                    23
#define TDES3_CRC_PAD_CTRL_MASK		                    MASK(27, 26)

/* Transmit checksum insertion control */
#define TX_CIC_FULL                                     3 /* Include IP header and pseudoheader */

/* TDES3 (write back format) */
#define TDES3_IP_HDR_ERROR		                        BIT(0)
#define TDES3_DEFERRED			                        BIT(1)
#define TDES3_UNDERFLOW_ERROR		                    BIT(2)
#define TDES3_EXCESSIVE_DEFERRAL	                    BIT(3)
#define TDES3_COLLISION_COUNT_MASK	                    MASK(7, 4)
#define TDES3_COLLISION_COUNT_SHIFT	                    4
#define TDES3_EXCESSIVE_COLLISION	                    BIT(8)
#define TDES3_LATE_COLLISION		                    BIT(9)
#define TDES3_NO_CARRIER		                        BIT(10)
#define TDES3_LOSS_CARRIER		                        BIT(11)
#define TDES3_PAYLOAD_ERROR		                        BIT(12)
#define TDES3_PACKET_FLUSHED		                    BIT(13)
#define TDES3_JABBER_TIMEOUT		                    BIT(14)
#define TDES3_ERROR_SUMMARY		                        BIT(15)
#define TDES3_TIMESTAMP_STATUS		                    BIT(17)
#define TDES3_TIMESTAMP_STATUS_SHIFT	                17

/* TDES3 context */
#define TDES3_CTXT_TCMSSV		                        BIT(26)

/* TDES3 Common */
#define TDES3_RS1V                                      BIT(26)
#define TDES3_RS1V_SHIFT                                26
#define TDES3_LAST_DESCRIPTOR                           BIT(28)
#define TDES3_LAST_DESCRIPTOR_SHIFT                     28
#define TDES3_FIRST_DESCRIPTOR                          BIT(29)
#define TDES3_CONTEXT_TYPE                              BIT(30)
#define TDES3_CONTEXT_TYPE_SHIFT                        30

/* TDES4 */
#define TDES4_LTV                                       BIT(31)
#define TDES4_LT                                        MASK(7, 0)

/* TDS3 use for both format (read and write back) */
#define TDES3_OWN                                       BIT(31)
#define TDES3_OWN_SHIFT                                 31

/* RDES0 (write back format) */
#define RDES0_VLAN_TAG_MASK                             MASK(15, 0)

/* RDES1 (write back format) */
#define RDES1_IP_PAYLOAD_TYPE_MASK                      MASK(2, 0)
#define RDES1_IP_HDR_ERROR                              BIT(3)
#define RDES1_IPV4_HEADER                               BIT(4)
#define RDES1_IPV6_HEADER                               BIT(5)
#define RDES1_IP_CSUM_BYPASSED                          BIT(6)
#define RDES1_IP_CSUM_ERROR                             BIT(7)
#define RDES1_PTP_MSG_TYPE_MASK                         MASK(11, 8)
#define RDES1_PTP_PACKET_TYPE                           BIT(12)
#define RDES1_PTP_VER                                   BIT(13)
#define RDES1_TIMESTAMP_AVAILABLE                       BIT(14)
#define RDES1_TIMESTAMP_AVAILABLE_SHIFT                 14
#define RDES1_TIMESTAMP_DROPPED                         BIT(15)
#define RDES1_IP_TYPE1_CSUM_MASK                        MASK(31, 16)

/* RDES2 (write back format) */
#define RDES2_L3_L4_HEADER_SIZE_MASK                    MASK(9, 0)
#define RDES2_VLAN_FILTER_STATUS                        BIT(15)
#define RDES2_SA_FILTER_FAIL                            BIT(16)
#define RDES2_DA_FILTER_FAIL                            BIT(17)
#define RDES2_HASH_FILTER_STATUS                        BIT(18)
#define RDES2_MAC_ADDR_MATCH_MASK                       MASK(26, 19)
#define RDES2_HASH_VALUE_MATCH_MASK                     MASK(26, 19)
#define RDES2_L3_FILTER_MATCH                           BIT(27)
#define RDES2_L4_FILTER_MATCH                           BIT(28)
#define RDES2_L3_L4_FILT_NB_MATCH_MASK                  MASK(27, 26)
#define RDES2_L3_L4_FILT_NB_MATCH_SHIFT                 26
#define RDES2_HL                                        MASK(9, 0)

/* RDES3 (write back format) */
#define RDES3_PACKET_SIZE_MASK                          MASK(14, 0)
#define RDES3_ERROR_SUMMARY                             BIT(15)
#define RDES3_PACKET_LEN_TYPE_MASK                      MASK(18, 16)
#define RDES3_DRIBBLE_ERROR                             BIT(19)
#define RDES3_RECEIVE_ERROR                             BIT(20)
#define RDES3_OVERFLOW_ERROR                            BIT(21)
#define RDES3_RECEIVE_WATCHDOG                          BIT(22)
#define RDES3_GIANT_PACKET                              BIT(23)
#define RDES3_CRC_ERROR                                 BIT(24)
#define RDES3_RDES0_VALID                               BIT(25)
#define RDES3_RDES1_VALID                               BIT(26)
#define RDES3_RDES2_VALID                               BIT(27)
#define RDES3_LAST_DESCRIPTOR                           BIT(28)
#define RDES3_FIRST_DESCRIPTOR                          BIT(29)
#define RDES3_CONTEXT_DESCRIPTOR                        BIT(30)
#define RDES3_CONTEXT_DESCRIPTOR_SHIFT                  30

/* RDES3 (read format) */
#define RDES3_BUFFER1_VALID_ADDR                        BIT(24)
#define RDES3_BUFFER2_VALID_ADDR                        BIT(25)
#define RDES3_INT_ON_COMPLETION_EN                      BIT(30)
#define RDES3_OWN                                       BIT(31)

#endif /* _MAC_HW_H_ */
