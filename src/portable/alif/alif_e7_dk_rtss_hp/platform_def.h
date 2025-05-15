/*
 * Copyright (c) 2017-2020, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PLATFORM_DEF_H
#define PLATFORM_DEF_H

#include <zephyr/sys/util.h> // for BIT macros

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION __attribute__((section("usb_dma_buf")))
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(32)))
#endif

// Base address of USB device controller
#define USB_REG(offset) (USB_CTRL_BASE + (offset))

/* === Register Offsets === */
#define GSBUSCFG0_REG_OFFSET    0xC100 /* Global SoC Bus Configuration Register 0 */
#define GSBUSCFG1_REG_OFFSET    0xC104 /* Global SoC Bus Configuration Register 1 */
#define GCTL_REG_OFFSET         0xC110 /* Global Core Control Register */
#define GSTS_REG_OFFSET         0xC118 /* Global Status Register */
#define GUCTL1_REG_OFFSET       0xC11C /* Global User Control Register 1 */
#define GSNPSID_REG_OFFSET      0xC120 /* Global Controller ID Register */
#define GUID_REG_OFFSET         0xC128 /* Global User ID Register */
#define GUCTL_REG_OFFSET        0xC12C /* Global User Control Register */
#define GBUSERRADDLO_REG_OFFSET 0xC130 /* Global SoC Bus Error Address Register—Low */
#define GBUSERRADDHI_REG_OFFSET 0xC134 /* Global SoC Bus Error Address Register—High */
#define GHWPARAMS_REG_OFFSET(n)                                                                    \
	(0xC140 + (n) * 0x4) /* Global Hardware Parameters Register 0..7                           \
			      */
#define GPRTBIMAP_HSLO_REG_OFFSET                                                                  \
	0xC180 /* Global High-Speed Port to Bus Instance Mapping (Low)                             \
		*/
#define GPRTBIMAP_HSHI_REG_OFFSET                                                                  \
	0xC184 /* Global High-Speed Port to Bus Instance Mapping (High)                            \
		*/
#define GPRTBIMAP_FSLO_REG_OFFSET                                                                  \
	0xC188 /* Global Full-Speed Port to Bus Instance Mapping (Low)                             \
		*/
#define GPRTBIMAP_FSHI_REG_OFFSET                                                                  \
	0xC18C                          /* Global Full-Speed Port to Bus Instance Mapping (High)   \
					 */
#define GUCTL2_REG_OFFSET        0xC19C /* Global User Control Register 2 */
#define GUSB2PHYCFG0_REG_OFFSET  0xC200 /* Global USB2 PHY Configuration Register 0 */
#define GEVNTADDRL0_REG_OFFSET   0xC400 /* Global Event Buffer Address (Low) */
#define GEVNTADDRHI0_REG_OFFSET  0xC404 /* Global Event Buffer Address (High) */
#define GEVNTSIZ0_REG_OFFSET     0xC408 /* Global Event Buffer Size Register */
#define GEVNTCOUNT0_REG_OFFSET   0xC40C /* Global Event Buffer Count Register */
#define GHWPARAMS8_REG_OFFSET    0xC600 /* Global Hardware Parameters Register 8 */
#define GTXFIFOPRIDEV_REG_OFFSET 0xC610 /* Global Device TX FIFO DMA Priority Register */
#define GTXFIFOPRIHST_REG_OFFSET 0xC618 /* Global Host TX FIFO DMA Priority Register */
#define GRXFIFOPRIHST_REG_OFFSET 0xC61C /* Global Host RX FIFO DMA Priority Register */
#define GFLADJ_REG_OFFSET        0xC630 /* Global Frame Length Adjustment Register */
#define DCFG_REG_OFFSET          0xC700 /* Device Configuration Register */
#define DCTL_REG_OFFSET          0xC704 /* Device Control Register */
#define DEVTEN_REG_OFFSET        0xC708 /* Device Event Enable Register */
#define DSTS_REG_OFFSET          0xC70C /* Device Status Register */
#define DGCMDPAR_REG_OFFSET      0xC710 /* Device Command Parameter Register */
#define DGCMD_REG_OFFSET         0xC714 /* Device Command Register */
#define DALEPENA_REG_OFFSET      0xC720 /* Active Endpoints Enable */

#define GUSB2RHBCCTL0_REG_OFFSET(n)                                                                \
	(0xC640 + (n) * 0x10) /* Global USB 2.0 Root Hub Control Register */
#define GEVNTADDRL_REG_OFFSET(n) (0xC648 + (n) * 0x10) /* Global Event Buffer Address (Low) */
#define GEVNTADDRH_REG_OFFSET(n) (0xC64C + (n) * 0x10) /* Global Event Buffer Address (High) */
#define DEPCMDPAR2N_OFFSET(n)                                                                      \
	(0xC800 + (n) * 0x10) /* Device Endpoint Command Parameter 2 Register */
#define DEPCMDPAR1N_OFFSET(n)                                                                      \
	(0xC804 + (n) * 0x10) /* Device Endpoint Command Parameter 1 Register */
#define DEPCMDPAR0N_OFFSET(n)                                                                      \
	(0xC808 + (n) * 0x10)                   /* Device Endpoint Command Parameter 0 Register */
#define DEPCMDN_OFFSET(n) (0xC80C + (n) * 0x10) /* Device Endpoint Command Register */

/* === Register Addresses (with base applied) === */
#define GSBUSCFG0_REG        USB_REG(GSBUSCFG0_REG_OFFSET)
#define GSBUSCFG1_REG        USB_REG(GSBUSCFG1_REG_OFFSET)
#define GCTL_REG             USB_REG(GCTL_REG_OFFSET)
#define GSTS_REG             USB_REG(GSTS_REG_OFFSET)
#define GUCTL1_REG           USB_REG(GUCTL1_REG_OFFSET)
#define GSNPSID_REG          USB_REG(GSNPSID_REG_OFFSET)
#define GUID_REG             USB_REG(GUID_REG_OFFSET)
#define GUCTL_REG            USB_REG(GUCTL_REG_OFFSET)
#define GEVNTCOUNT0_REG      USB_REG(GEVNTCOUNT0_REG_OFFSET)
#define GUSB2PHYCFG0_REG     USB_REG(GUSB2PHYCFG0_REG_OFFSET)
#define DCFG_REG             USB_REG(DCFG_REG_OFFSET)
#define DCTL_REG             USB_REG(DCTL_REG_OFFSET)
#define DEVTEN_REG           USB_REG(DEVTEN_REG_OFFSET)
#define DSTS_REG             USB_REG(DSTS_REG_OFFSET)
#define DGCMDPAR_REG         USB_REG(DGCMDPAR_REG_OFFSET)
#define DGCMD_REG            USB_REG(DGCMD_REG_OFFSET)
#define DALEPENA_REG         USB_REG(DALEPENA_REG_OFFSET)
#define GEVNTSIZ_REG(n)      USB_REG(GEVNTSIZ_REG_OFFSET(n))
#define GUSB2RHBCCTL0_REG(n) USB_REG(GUSB2RHBCCTL0_REG_OFFSET(n))
#define GEVNTADDRL_REG(n)    USB_REG(GEVNTADDRL_REG_OFFSET(n))
#define GEVNTADDRH_REG(n)    USB_REG(GEVNTADDRH_REG_OFFSET(n))
#define DEPCMDPAR2N(n)       USB_REG(DEPCMDPAR2N_OFFSET(n))
#define DEPCMDPAR1N(n)       USB_REG(DEPCMDPAR1N_OFFSET(n))
#define DEPCMDPAR0N(n)       USB_REG(DEPCMDPAR0N_OFFSET(n))
#define DEPCMDN(n)           USB_REG(DEPCMDN_OFFSET(n))
#define GBUSERRADDLO_REG     USB_REG(GBUSERRADDLO_REG_OFFSET)
#define GBUSERRADDHI_REG     USB_REG(GBUSERRADDHI_REG_OFFSET)
#define GPRTBIMAP_HSLO_REG   USB_REG(GPRTBIMAP_HSLO_REG_OFFSET)
#define GPRTBIMAP_HSHI_REG   USB_REG(GPRTBIMAP_HSHI_REG_OFFSET)
#define GPRTBIMAP_FSLO_REG   USB_REG(GPRTBIMAP_FSLO_REG_OFFSET)
#define GPRTBIMAP_FSHI_REG   USB_REG(GPRTBIMAP_FSHI_REG_OFFSET)
#define GUCTL2_REG           USB_REG(GUCTL2_REG_OFFSET)
#define GHWPARAMS8_REG       USB_REG(GHWPARAMS8_REG_OFFSET)
#define GTXFIFOPRIDEV_REG    USB_REG(GTXFIFOPRIDEV_REG_OFFSET)
#define GTXFIFOPRIHST_REG    USB_REG(GTXFIFOPRIHST_REG_OFFSET)
#define GRXFIFOPRIHST_REG    USB_REG(GRXFIFOPRIHST_REG_OFFSET)
#define GFLADJ_REG           USB_REG(GFLADJ_REG_OFFSET)
#define GEVNTADDRL0_REG      USB_REG(GEVNTADDRL0_REG_OFFSET)
#define GEVNTADDRHI0_REG     USB_REG(GEVNTADDRHI0_REG_OFFSET)
#define GEVNTSIZ0_REG        USB_REG(GEVNTSIZ0_REG_OFFSET)
#define GEVNTCOUNTO0_REG     USB_REG(GEVNTCOUNTO0_REG_OFFSET)
#define GHWPARAMS8_REG       USB_REG(GHWPARAMS8_REG_OFFSET)
#define GTXFIFOPRIDEV_REG    USB_REG(GTXFIFOPRIDEV_REG_OFFSET)
#define GTXFIFOPRIHST_REG    USB_REG(GTXFIFOPRIHST_REG_OFFSET)
#define GRXFIFOPRIHST_REG    USB_REG(GRXFIFOPRIHST_REG_OFFSET)
#define GFLADJ_REG           USB_REG(GFLADJ_REG_OFFSET)

/* Bit Fields */
#define GCTL_CORESOFTRESET BIT(11) /* Core Soft Reset */

#define GUSB2PHYCFG0_PHYSOFTRST               BIT(31) /* UTMI PHY soft reset */
#define GUSB2PHYCFG0_U2_FREECLK_EXISTS        BIT(30) /* USB 2.0 Free-running clock exists */
#define GUSB2PHYCFG0_ULPI_LPM_WITH_OPMODE_CHK BIT(29) /* Reserved, must be 0 */
#define GUSB2PHYCFG0_OVRD_FSLS_DISC_TIME      BIT(25) /* Override FS/LS disconnect timing */
#define GUSB2PHYCFG0_LSTRD(x)                 (((x) & 0x7) << 22) /* LS Rx-to-Tx turnaround time */
#define GUSB2PHYCFG0_LSIPD(x)                 (((x) & 0x7) << 19) /* LS Tx-to-Tx inter-packet delay */
#define GUSB2PHYCFG0_USBTRDTIM(x)             (((x) & 0xF) << 10) /* HS/FS turnaround time */
#define GUSB2PHYCFG0_USBTRDTIM_MASK           (0xF << 10)
#define GUSB2PHYCFG0_XCVRDLY                  BIT(9) /* Enable HS Chirp Transceiver Delay */
#define GUSB2PHYCFG0_ENBLSLPM                 BIT(8) /* Enable UTMI_SUSPEND, UTMI_SLEEP */
#define GUSB2PHYCFG0_PHYSEL                   BIT(7) /* USB2.0 PHY vs USB1.1 Transceiver Select (always 0) */
#define GUSB2PHYCFG0_SUSPENDUSB20             BIT(6) /* Enter suspend mode when set */
#define GUSB2PHYCFG0_FSINTF                   BIT(5) /* Full-Speed serial interface (always 0) */
#define GUSB2PHYCFG0_ULPI_UTMI_SEL            BIT(4) /* 0: UTMI+, 1: ULPI */
#define GUSB2PHYCFG0_PHYIF                    BIT(3) /* 0: 8-bit UTMI+, 1: 16-bit */
#define GUSB2PHYCFG0_TOUTCAL(x)               ((x) & 0x7) /* Timeout calibration for HS/FS interpacket */

#define GTXFIFOSIZn_REG(n) (0xC300 + ((n) * 0x04)) /* Global TX FIFO Size Register n */
#define GRXFIFOSIZn_REG(n) (0xC380 + ((n) * 0x04)) /* Global RX FIFO Size Register n */

#define DCFG_IGNSTRMPP    BIT(23)     /* Ignore Stream PP */
#define DCFG_LPMCAP       BIT(22)     /* LPM Capable */
#define DCFG_NUMP_MASK    (0xF << 17) /* Number of Receive Buffers */
#define DCFG_NUMP(x)      (((x) & 0xF) << 17)
#define DCFG_INTRNUM_MASK (0x1F << 12) /* Interrupt/EventQ number */
#define DCFG_INTRNUM(x)   (((x) & 0x1F) << 12)
#define DCFG_DEVADDR_MASK (0x7F << 3) /* Device USB Address */
#define DCFG_DEVADDR(x)   (((x) & 0x7F) << 3)
#define DCFG_SPEED_MASK   (0x7 << 0) /* Device Speed */
#define DCFG_SPEED(x)     (((x) & 0x7) << 0)
#define DCFG_SPEED_HIGH   (0x0 << 0) /* High-speed (default) */
#define DCFG_SPEED_FULL   (0x1 << 0) /* Full-speed */
#define DCFG_SPEED_SUPER  (0x4 << 0) /* SuperSpeed (USB 3.0) */

#define DCTL_CSFTRST        BIT(30) /* Core Soft Reset */
#define DCTL_RUN_STOP       BIT(31) /* 0x1 - start the device controller.*/
#define DCTL_CSS            BIT(16) /* Core Soft Reset Status */
#define DCTL_ULSTCHNGREQ(x) (((x) << 5) & 0x1E0U)
#define DCTL_TSTCTL(x)      (((x) << 1) & 0x1EU)
#define DCTL_ACCEPTU1ENA    BIT(9)
#define DCTL_INITU1ENA      BIT(10)
#define DCTL_ACCEPTU2ENA    BIT(11)
#define DCTL_INITU2ENA      BIT(12)

#define GSTS_CURMOD_MASK   GENMASK(1, 0) /* Current mode of operation */
#define GSTS_CURMOD_DEVICE 0x0           /* Device mode */
#define GSTS_CURMOD_HOST   0x1           /* Host mode */

#define GSTS_BUSERRADDRVLD BIT(4) /* Bus error address valid */
#define GSTS_CSRTIMEOUT    BIT(5) /* CSR timeout */
#define GSTS_DEVICE_IP     BIT(6) /* Device interrupt pending */
#define GSTS_HOST_IP       BIT(7) /* Host interrupt pending */

#define GSTS_ADP_IP  BIT(8)  /* Not used */
#define GSTS_BC_IP   BIT(9)  /* Not used */
#define GSTS_OTG_IP  BIT(10) /* Not used */
#define GSTS_SSIC_IP BIT(11) /* Not used */

#define GSTS_CBELT_MASK GENMASK(31, 20) /* Current BELT value (Host mode only) */

#define DEVTEN_DISSCONEVTEN        BIT(0)  /* Disconnection Event Enable */
#define DEVTEN_USBRSTEVTEN         BIT(1)  /* USB Reset Event Enable */
#define DEVTEN_CONNECTDONEEVTEN    BIT(2)  /* Connection Done Event Enable */
#define DEVTEN_ULSTCNGEN           BIT(3)  /* USB Link State Change Enable */
#define DEVTEN_WKUPEVTEN           BIT(4)  /* L2 or L1 Resume Detected Event */
#define DEVTEN_HIBERNATIONREQEVTEN BIT(5)  /* Hibernation Request Event Enable */
#define DEVTEN_U3L2L1SUSPEN        BIT(6)  /* L2 or L1 Suspend Event Enable */
#define DEVTEN_SOFTEVTEN           BIT(7)  /* Start of Frame Event Enable */
#define DEVTEN_L1SUSPEN            BIT(8)  /* L1 Suspend Event Enable */
#define DEVTEN_ERRATICERREVTEN     BIT(9)  /* Erratic Error Event Enable */
#define DEVTEN_VENDEVTSTRCVEDEN    BIT(10) /* Vendor Device Test LMP Received */
#define DEVTEN_RESERVED_11         BIT(11)
#define DEVTEN_RESERVED_12         BIT(12)
#define DEVTEN_L1WKUPEN            BIT(14) /* L1 Resume Detected Event Enable */
#define DEVTEN_RESERVED_15         BIT(15)
#define DEVTEN_ECCERREN            BIT(16) /* ECC Error Event Enable */

/* DALEPENA Register Bitfields */
#define EP0_OUT_EN BIT(0) /* Enable EP0 OUT */
#define EP0_IN_EN  BIT(1) /* Enable EP0 IN */

#define DEPCMD_CMDTYP_MASK   (0xF << 0) /* Command Type (CMDTYP) [3:0] */
#define DEPCMD_CMDTYP(x)     (((x) & 0xF) << 0)
#define DEPCMD_CMDIOC        BIT(8)  /* Command Interrupt on Completion (CMDIOC) [8] */
#define DEPCMD_CMDACT        BIT(10) /* Command Active (CMDACT) [10] */
#define DEPCMD_HIPRI_FORCERM BIT(11)
#define DEPCMD_STATUS_MASK   (0xF << 12) /* Command Status (CMDSTATUS) [15:12] */
#define DEPCMD_STATUS(x)     (((x) & 0xF) << 12)
#define DEPCMD_PARAM_MASK    (0xFFFF << 16)
#define DEPCMD_PARAM(x)      (((x) & 0xFFFF) << 16)

#define GEVNTCOUNT0_EVNTCOUNT_MASK    GENMASK(15, 0) /* Number of valid bytes in Event Buffer */
#define GEVNTCOUNT0_EVNTCOUNT(x)      ((x) & GEVNTCOUNT0_EVNTCOUNT_MASK)
#define GEVNTCOUNT0_EVNT_HANDLER_BUSY BIT(31) /* Event handler busy indicator */

/* PWR_CTRL Register @ Offset 0x08 */
#define PWR_CTRL_UPHY_ISO          BIT(17) /* USB PHY Isolation Enable [17] */
#define PWR_CTRL_UPHY_PWR_MASK     BIT(16) /* USB PHY Power Mask [16] */
#define PWR_CTRL_VPH_1P8_BYP_VAL   BIT(13) /* USB PHY VPH 1.8V Bypass Value [13] */
#define PWR_CTRL_VPH_1P8_BYP_EN    BIT(12) /* USB PHY VPH 1.8V Bypass Enable [12] */
#define PWR_CTRL_DPHY_PLL_ISO      BIT(9)  /* MIPI PLL Isolation Enable [9] */
#define PWR_CTRL_DPHY_PLL_PWR_MASK BIT(8)  /* MIPI PLL Power Mask [8] */
#define PWR_CTRL_RX_DPHY_ISO       BIT(5)  /* RX MIPI DPHY Isolation Enable [5] */
#define PWR_CTRL_RX_DPHY_PWR_MASK  BIT(4)  /* RX MIPI DPHY Power Mask [4] */
#define PWR_CTRL_TX_DPHY_ISO       BIT(1)  /* TX MIPI DPHY Isolation Enable [1] */
#define PWR_CTRL_TX_DPHY_PWR_MASK  BIT(0)  /* TX MIPI DPHY Power Mask [0] */

/* GSBUSCFG0 Register Bit Definitions */
#define GSBUSCFG0_INCRBRSTENA    BIT(0) /* INCR burst type enable */
#define GSBUSCFG0_INCR4BRSTENA   BIT(1) /* INCR4 burst type enable */
#define GSBUSCFG0_INCR8BRSTENA   BIT(2) /* INCR8 burst type enable */
#define GSBUSCFG0_INCR16BRSTENA  BIT(3) /* INCR16 burst type enable */
#define GSBUSCFG0_INCR32BRSTENA  BIT(4) /* INCR32 burst type enable */
#define GSBUSCFG0_INCR64BRSTENA  BIT(5) /* INCR64 burst type enable */
#define GSBUSCFG0_INCR128BRSTENA BIT(6) /* INCR128 burst type enable */
#define GSBUSCFG0_INCR256BRSTENA BIT(7) /* INCR256 burst type enable */

#define GSBUSCFG0_DESBIGEND BIT(10) /* Descriptor endianness: 0 = little, 1 = big */
#define GSBUSCFG0_DATBIGEND BIT(11) /* Data endianness: 0 = little, 1 = big */

#define GSBUSCFG0_DESWRREQINFO(x) (((x) & 0xF) << 16) /* Descriptor write request AHB/AXI info */
#define GSBUSCFG0_DATWRREQINFO(x) (((x) & 0xF) << 20) /* Data write request AHB/AXI info */
#define GSBUSCFG0_DESRDREQINFO(x) (((x) & 0xF) << 24) /* Descriptor read request AHB/AXI info */
#define GSBUSCFG0_DATRDREQINFO(x) (((x) & 0xF) << 28) /* Data read request AHB/AXI info */

#define GSBUSCFG0_REQINFO_MASK (0xFFFF << 16) /* Маска на все REQINFO поля */

/* GEVNTSIZ0 Register Bit Definitions */
#define GEVNTSIZ0_EVENTSIZ(x)                                                                      \
	((x) & 0xFFFF) /* Bits 0–15: Event buffer size (in bytes, must be multiple of 4) */
#define GEVNTSIZ0_EVENTSIZ_MASK  GENMASK(15, 0)
#define GEVNTSIZ0_RESERVED_MASK  GENMASK(30, 16)
#define GEVNTSIZ0_MASK           GENMASK(30, 0) /* Mask for all writable bits */
#define GEVNTSIZ0_EVNTINTRPTMASK BIT(31)        /* Event interrupt mask */

/* USB_CTRL2 Register @ Offset 0xAC (CLKCTL_PER_MST 0x4903_F000) */
#define USB_CTRL2_POR_RST_MASK BIT(8)      /* POR_RST_MASK: USB PHY PoR Reset Mask (bit 8) */
#define USB_CTRL2_PME_EN       BIT(6)      /* PME_EN: Power Management Enable (bit 6) */
#define USB_CTRL2_FLADJ_MASK   (0x3F << 0) /* FLADJ_30MHZ_REG: Frequency Adjust Val (bits [5:0]) */
#define USB_CTRL2_FLADJ_SET(x) (((x) & 0x3F) << 0)

// DEPEVT event types
enum {
	// reserved
	DEPEVT_XFERCOMPLETE = 1,
	DEPEVT_XFERINPROGRESS,
	DEPEVT_XFERNOTREADY,
	// not implemented
	DEPEVT_STREAMEVT = 6,
	DEPEVT_EPCMDCMPLT
} DEPEVT;

// DEVT event types
enum {
	DEVT_DISCONNEVT = 0,
	DEVT_USBRST,
	DEVT_CONNECTDONE,
	DEVT_ULSTCHNG,
	DEVT_WKUPEVT,
	// DEVT_HIBRQ not implemented
	DEVT_USBSUSP = 6,
	DEVT_SOF,
	DEVT_L1SUSP,
	DEVT_ERRTICERR,
	DEVT_CMDCMPLT,
	DEVT_EVNTOVERFLOW,
	DEVT_VNDDEVTSTRCVED,
	// reserved
	DEVT_L1RESM = 14,
	// reserved
	DEVT_ECCERR = 16
} DEVT;

/* DEPCMD command types */
enum {
	DEPCMD_RESERVED = 0,
	DEPCMD_DEPCFG,
	DEPCMD_DEPXFERCFG,
	DEPCMD_DEPGETSTATE,
	DEPCMD_DEPSSTALL,
	DEPCMD_DEPCSTALL,
	DEPCMD_DEPSTRTXFER,
	DEPCMD_DEPUPDXFER,
	DEPCMD_DEPENDXFER,
	DEPCMD_DEPSTARTCFG
} DEPCMD_CMDTYP;

enum {
	// reserved
	TRBCTL_NORMAL = 1,
	TRBCTL_CTL_SETUP,
	TRBCTL_CTL_STAT2,
	TRBCTL_CTL_STAT3,
	TRBCTL_CTL_DATA,
	TRBCTL_ISO_FIRST,
	TRBCTL_ISO,
	TRBCTL_LINK,
	TRBCTL_NORMAL_ZLP
} TRBCTL;

#endif /* PLATFORM_DEF_H */
