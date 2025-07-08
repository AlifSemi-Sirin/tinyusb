#ifndef DCD_ENSEMBLE_DEF_H
#define DCD_ENSEMBLE_DEF_H

#if CFG_TUSB_OS == OPT_OS_ZEPHYR

#include <zephyr/sys/util.h> // for BIT macros
#include <soc_common.h>
#include <soc_memory_map.h>    

#define USB_NODE DT_NODELABEL(usb0)
#define USB_CTRL_BASE DT_REG_ADDR(USB_NODE)
#define USB_IRQ_IRQn DT_IRQN(USB_NODE)

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

/* CLK_ENA Register Bit Definitions */
#define CLK_ENA_SYSPLL      BIT(0)  /* Enable SYSPLL_CLK */
#define CLK_ENA_CPUPLL      BIT(4)  /* Enable CPUPLL_CLK */
#define CLK_ENA_ES0         BIT(12) /* Enable RTSS_HP_CLK */
#define CLK_ENA_ES1         BIT(13) /* Enable RTSS_HE_CLK */
#define CLK_ENA_HFXO        BIT(18) /* Enable clock to the HFXO_OUT pin */
#define CLK_ENA_CLK160M     BIT(20) /* Enable 160M_CLK */
#define CLK_ENA_CLK100M     BIT(21) /* Enable 100M_CLK */
#define CLK_ENA_CLK20M      BIT(22) /* Enable USB_CLK and 10M_CLK */
#define CLK_ENA_CLK38P4M    BIT(23) /* Enable HFOSC_CLK */
#define CLK_ENA_CVM         BIT(24) /* Enable SRAM0 clock */
#define CLK_ENA_OCVM        BIT(28) /* Enable SRAM1 clock */

/* Peripheral Clock Enable Register Bit Definitions */
#define PERIPH_CLK_ENA_CPI_CKEN     BIT(0)  /* Enable clock for CPI */
#define PERIPH_CLK_ENA_DPI_CKEN     BIT(1)  /* Enable clock for DPI controller (CDC) */
#define PERIPH_CLK_ENA_DMA_CKEN     BIT(4)  /* Enable clock for DMA0 */
#define PERIPH_CLK_ENA_GPU_CKEN     BIT(8)  /* Enable clock for GPU2D */
#define PERIPH_CLK_ENA_ETH_CKEN     BIT(12) /* Enable clock for ETH */
#define PERIPH_CLK_ENA_SDC_CKEN     BIT(16) /* Enable clock for SDMMC */
#define PERIPH_CLK_ENA_USB_CKEN     BIT(20) /* Enable clock for USB */
#define PERIPH_CLK_ENA_CSI_CKEN     BIT(24) /* Enable clock for CSI */
#define PERIPH_CLK_ENA_DSI_CKEN     BIT(28) /* Enable clock for DSI */

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

/* USB_CTRL2 Register @ Offset 0xC19C Global User Control Register 2 */
#define GUCTL2_RST_ACTBITLATER  BIT(14) /* Reset Active Bit Later */

#endif

// Alif USB ID definitions
#define GSNPSID_HIGH            0x55330000  // Core Identification Number
#define GSNPSID_LOW             0x0000330B  // Release number 3.30b

// Alif USB2.0 turnaround time
#define GUSB2PHYCFG0_USBTRDTIM_16BIT    0x5 // MAC interface is 16-bit UTMI+
#define GUSB2PHYCFG0_USBTRDTIM_8BIT     0x9 // MAC interface is 8-bit UTMI+/ULPI

// Alif USB device speed settings
#define DCFG_DEVSPD_HIGH_SPEED  0x00        // High-speed 480MBit/s
#define DCFG_DEVSPD_FULL_SPEED  0x01        // Full-speed 12MBit/s

// 
#define USB_BASE                    0x48200000UL

// USB Registers Access Types
#define _rw volatile uint32_t
#define __w volatile uint32_t
#define __r volatile const uint32_t

volatile struct {
    union {
        _rw gsbuscfg0;              // Global SoC Bus Configuration Register 0 0xC100
        struct {
            _rw incrbrstena    : 1; // Undefined length INCR burst type enable
            _rw incr4brstena   : 1; // INCR4 burst type enable
            _rw incr8brstena   : 1; // INCR8 burst type enable
            _rw incr16brstena  : 1; // INCR16 burst type enable
            _rw incr32brstena  : 1; // INCR32 burst type enable
            _rw incr64brstena  : 1; // INCR64 burst type enable
            _rw incr128brstena : 1; // INCR128 burst type enable
            _rw incr256brstena : 1; // INCR256 burst type enable
            __r : 2;                // Reserved
            _rw desbigend      : 1; // Descriptor access is big endian
            _rw datbigend      : 1; // Data access is big endian
            __r : 4;                // Reserved
            _rw deswrreqinfo   : 4; // Descriptor write request info
            _rw datwrreqinfo   : 4; // Data write request info 
            _rw desrdreqinfo   : 4; // Descriptor read request info
            _rw datrdreqinfo   : 4; // Data read request info
        } gsbuscfg0_b;
    };

    union {
        _rw gsbuscfg1;
        struct {
            __r : 8;
            _rw pipetranslimit : 4;
            _rw en1kpage       : 1;
            __r : 19;
        } gsbuscfg1_b;
    };

    __r : 32; __r : 32;

    union {
        _rw gctl;   // Global Core Control Register 0xC110
        struct {
            _rw dsblclkgtng      : 1;   // Disable clock gating
            __r gblhibernationen : 1;   // Hibernation enable status at the global level
            __r : 1;                    // Reserved
            _rw disscramble      : 1;   // Disable scrambling
            _rw scaledown        : 2;   // Scale-down mode
            _rw ramclksel        : 2;   // RAM clock select
            __r : 2;                    // Reserved
            _rw sofitpsync       : 1;   // Reserved
            _rw coresoftreset    : 1;   // Core soft reset
            _rw prtcapdir        : 2;   // Port capability direction
            _rw frmscldwn        : 2;   // Frame scales down
            __r : 1;                    // Reserved
            _rw bypssetaddr      : 1;   // Bypass set address in Device mode
            __r : 14;                   // Reserved
        } gctl_b;
    };

    __r : 32;

    union {
        _rw gsts;
        struct {
            __r curmod        : 2;
            __r : 2;
            _rw buserraddrvld : 1;
            _rw csrtimeout    : 1;
            __r device_ip     : 1;
            __r host_ip       : 1;
            __r adp_ip        : 1;
            __r bc_ip         : 1;
            __r otg_ip        : 1;
            __r ssic_ip       : 1;
            __r : 8;
            __r cbelt         : 12;
        } gsts_b;
    };

    union {
        _rw guctl1;
        struct {
            _rw loa_filter_en             : 1;
            _rw ovrld_l1_susp_com         : 1;
            _rw hc_parchk_disable         : 1;
            _rw hc_errata_enable          : 1;
            _rw l1_susp_thrld_for_host    : 4;
            _rw l1_susp_thrld_en_for_host : 1;
            _rw dev_hs_nyet_bulk_spr      : 1;
            _rw resume_opmode_hs_host     : 1;
            __r : 1;
            _rw disusb2refclkgtng         : 1;
            __r : 2;
            _rw parkmode_disable_fsls     : 1;
            _rw parkmode_disable_hs       : 1;
            __r : 1;
            _rw nak_per_enh_hs            : 1;
            _rw nak_per_enh_fs            : 1;
            _rw dev_lsp_tail_lock_dis     : 1;
            _rw ip_gap_add_on             : 2;
            _rw dev_l1_exit_by_hw         : 1;
            __r : 2;
            _rw dev_trb_out_spr_ind       : 1;
            _rw tx_ipgap_linecheck_dis    : 1;
            _rw filter_se0_fsls_eop       : 1;
            __r : 1;
            _rw dev_decouple_l1l2_evt     : 1;
        } guctl1_b;
    };

    union {
        _rw gsnpsid;    // Global ID Register 0xC120
    };

    __r : 32;

    union {
        _rw guid;       // Global User ID Register 0xC128
        struct {        // Reset value is 0x12345678
            _rw userid : 32;
        } guid_b;
    };

    union { // <- host only, leaving unimplemented for now [FIXME]
        _rw guctl;
        struct {
        } guctl_b;
    };

    union {
        _rw gbuserraddrlo;
        struct {
            _rw buserraddr : 32;
        } gbuserraddrlo_b;
    };

    union {
        _rw gbuserraddrhi;
        struct {
            _rw buserraddr : 32;
        } gbuserraddrhi_b;
    };

    __r : 32; __r : 32;

    __r ghwparams[8];

    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32;

    // base + 0x9C here

    union {
        _rw guctl2;
        struct {
            __r : 11;
            _rw disablecfc         : 1;
            _rw enableepcacheevict : 1;
            __r : 1;
            _rw rst_actbitlater    : 1;
            __r : 4;
            _rw en_hp_pm_timer     : 7;
            __r : 6;
        } guctl2_b;
    };

    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;

    // base + 0x100 here

    union {
        _rw gusb2phycfg0;   // Global USB2 PHY Configuration Register 0xC200
        struct {
            _rw toutcal                  : 3;   // HS/FS timeout calibration
            _rw phyif                    : 1;   // PHY interface (8 or 16 bit)
            __r ulpi_utmi_sel            : 1;   // ULPI or UTMI+ selection
            __r fsintf                   : 1;   // Full-Speed interface select (allways 0)
            _rw suspendusb20             : 1;   // Suspend USB2.0 HS/FS/Ls PHY
            __w physel                   : 1;   // USB2.0 HS or USB1.1 FS
            _rw enblslpm                 : 1;   // Enable UTMI_SLEEP and UTMI_L1_SUSPEND_n
            _rw xcvrdly                  : 1;   // Transceiver Delay
            _rw usbtrdtim                : 4;   // USB2.0 turnaround time (in PHY clocks)
            __r : 5;                            // Reserved
            _rw lsipd                    : 3;   // LS inter-packet time
            _rw lstrd                    : 3;   // LS turnaround time
            _rw ovrd_fsls_disc_time      : 1;   // Overriding the FS/LS disconnect time to 32 us
            __r inv_sel_hsic             : 1;   // Reserved
            __r hsic_con_width_adj       : 2;   // Reserved
            _rw ulpi_lpm_with_opmode_chk : 1;   // Reserved (keep at 0)
            _rw u2_freeclk_exists        : 1;   // U2 Free Clock Esists
            _rw physoftrst               : 1;   // UTMI PHY soft reset
        } gusb2phycfg0_b;
    };

    __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;

    // base + 0x200 here

    union {
        _rw gtxfifosiz[4];      // Global Transmit FIFO Size Register 0xC300+(n*0x4)
        struct {
            _rw txfdep : 16;    // TxFIFO Depth (0xa on reset)
            _rw txfstaddr : 16; // TxFIFO RAM start address (0x271 on reset)
        } gtxfifosiz_b[4];
    };

    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;

    // base + 0x280 here

    union {
        _rw grxfifosiz[4];      // Global Receive FIFO Size Register 0xC380+(n*0x4)
        struct {
            _rw rxfdep : 16;    // RxFIFO depth (0x105 on reset)
            _rw rxfstaddr : 16; // RxFIFO RAM start address (0x271)
        } grxfifosiz_b[4];
    };

    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;

    // base + 0x300 here

    union {
        _rw gevntadrlo0;        // Global Event Buffer Address (Low) Register 0xC400
        struct {
            _rw evntadrlo : 32; // Event buffer address (Low)
        } gevntadrlo0_b;
    };

    union {
        _rw gevntadrhi0;        // Global Event Buffer Address (High) Register 0xC404
        struct {
            _rw evntadrhi : 32; // Event buffer address (High)
        } gevntadrhi0_b;
    };

    union {
        _rw gevntsiz0;
        struct {
            _rw eventsiz : 16;
            __r : 15;
            _rw evntintrptmask : 1;
        } gevntsiz0_b;
    };

    union {
        _rw gevntcount0;
        struct {
            _rw evntcount : 16;
            __r : 15;
            _rw evnt_handler_busy : 1;
        } gevntcount0_b;
    };

    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;

    // base + 0x500 here

    __r ghwparams8;

    __r : 32; __r : 32; __r : 32;

    // base + 0x510 here

    union {
        _rw gtxfifopridev;
        struct {
            _rw gtxfifopridev : 4;
            __r : 28;
        } gtxfifopridev_b;
    };

    __r : 32;

    union {
        _rw gtxfifoprihst;
        struct {
            _rw gtxfifoprihst : 2;
            __r : 30;
        } gtxfifoprihst_b;
    };

    union {
        _rw grxfifoprihst;
        struct {
            _rw grxfifoprihst : 2;
            __r : 30;
        } grxfifoprihst_b;
    };

    __r : 32; __r : 32; __r : 32; __r : 32;

    // base + 0x530 here

    union {
        _rw gfladj;
        struct {
            _rw gfladj_30mhz : 6;
            __r : 1;
            _rw gfladj_30mhz_sdbnd_sel : 1;
            _rw gfladj_refclk_fladj : 14;
            __r : 1;
            _rw gfladj_refclk_lpm_sel : 1;
            _rw gfladj_refclk_240mhz_decr : 7;
            _rw gfladj_refclk_240mhzdecr_pls1 : 1;
        } gfladj_b;
    };

    __r : 32; __r : 32; __r : 32;

    // base + 0x540 here

    union {
        _rw gusb2rhbctl0;
        struct {
            _rw ovrd_l1timeout : 4;
            __r : 28;
        } gusb2rhbctl0_b;
    };

} *ugbl = (void *) (USB_BASE + 0xC100); // USB GBL Registers

volatile struct {
    union {
        _rw dcfg;                   // Device Configuration Register 0xC700
        struct {
            _rw devspd : 3;         // Device Speed
            _rw devaddr : 7;        // Device Address
            __r : 2;                // Reserved
            _rw intrnum : 5;        // Interrupt number
            _rw nump : 5;           // Number of Receive Buffers
            _rw lpmcap : 1;         // LPM Capable
            _rw ignstrmpp : 1;      // Ignore Stream PP
            __r : 8;
        } dcfg_b;
    };

    union {
        _rw dctl;                   // Device Control register 0xC704
        struct {
            __r : 1;                // Reserved
            _rw tstctl : 4;         // Test Control
            __w ulstchngreq : 4;    // USB/Link state change request
            __r : 7;                // Reserved
            _rw css : 1;            // Controller Save State (CSS)
            _rw crs : 1;            // Controller Restore State (CRC)
            _rw l1hibernationen : 1;// L1 Hibernation Enable    
            _rw keepconnect : 1;    // Keep Connect
            _rw lpm_nyet_thres : 4; // LPM NYET Threshold
            _rw hirdthres : 5;      // HIRD Threshold
            __r : 1;                // Reserved
            _rw csftrst : 1;        // Core Soft Reset
            _rw run_stop : 1;       // Start/Stop the device controller operation
        } dctl_b;
    };

    union {
        _rw devten;                     // Device Event Enable Register 0xC708
        struct {
            _rw dissconnevten : 1;      // Disconnect Detected Event Enable
            _rw usbrstevten : 1;        // USB Reset Enable
            _rw connectdoneevten : 1;   // Connection Done Enable
            _rw ulstcngen : 1;          // USB/Link State Change Event Enable
            _rw wkupevten : 1;          // L2 or L2L1 Resume Detected Event Enable
            _rw hibernationreqevten : 1;// Generation of the Hibernation Request Event Enable
            _rw u3l2l1suspen : 1;       // L2 or L2L1 Suspend Event Enable
            _rw softevten : 1;          // Start of (micro)frame
            _rw l1suspen : 1;           // L1 Suspend Event Enable
            _rw errticerrevten : 1;     // Erratic Error Event Enable
            __r : 2;                    // Reserved
            _rw vendevtstrcvden : 1;    //Vendor Device Test LMP Received Event
            __r : 1;                    // Reserved
            _rw l1wkupevten : 1;        // L1 Resume Detected Event Enable
            __r : 1;                    // Reserved
            _rw eccerren : 1;           // ECC Error Enable
            __r : 15;                   // Reserved
        } devten_b;
    };

    union {
        _rw dsts;
        struct {
            __r connectspd : 3;
            __r soffn : 14;
            __r rxfifoempty : 1;
            __r usblnkst : 4;
            __r devctrlhlt : 1;
            __r coreidle : 1;
            __r sss : 1;
            __r rss : 1;
            __r : 2;
            _rw sre : 1;
            __r dcnrd : 1;
            __r : 2;
        } dsts_b;
    };

    union {
        _rw dgcmdpar;
        struct {
            _rw parameter : 32;
        } dgcmdpar_b;
    };

    union {
        _rw dgcmd;
        struct {
            _rw cmdtyp : 8;
            _rw cmdioc : 1;
            __r : 1;
            _rw cmdact : 1;
            __r : 1;
            __r cmdstatus : 4;
            __r : 16;
        } dgcmd_b;
    };

    __r : 32; __r : 32;

    union {
        _rw dalepena;   // Device Active USB Endpoint Enable Register 0xC720
        struct {
            _rw usbactep : 32;  // USB Active Endpoints
        } dalepena_b;
    };

    __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;

    // base + 0x100 here

    struct {
        union {
            _rw par2;       // DEPCMD Parameter 2 Register 0xC800+(n*0x10)
            struct {
                _rw parameter : 32;
            } par2_b;
        };

        union {
            _rw par1;       // DEPCMD Parameter 1 Register 0xC804+(n*0x10)
            struct {
                _rw parameter : 32;
            } par1_b;
        };

        union {
            _rw par0;       // DEPCMD Parameter 0 Register 0xC808+(n*0x10)
            struct {
                _rw parameter : 32;
            } par0_b;
        };

        union {
            _rw depcmd;                 // Device Physical Endpoint-n Command Register
            struct {                    //                          0xC80C+(n*0x10)
                _rw cmdtyp : 4;         // Command type
                __r : 4;                // Reserved 
                _rw cmdioc : 1;         // Command Interrupt on Complete
                __r : 1;                // Reserved
                _rw cmdact : 1;         // Command Active
                _rw hipri_forcerm : 1;  // HighPriority/ForceRM
                _rw cmdstatus : 4;      // Command Completion Status
                _rw commandparam : 16;  // Command Parameters or Event Parameters
            } depcmd_b;
        };
    } depcmd[8];

    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;
    __r : 32; __r : 32; __r : 32; __r : 32;

    // base + 0x300 here

    union {
        _rw dev_imod0;
        struct {
            _rw device_imodi : 16;
            _rw device_imocd : 16;
        } dev_imod0_b;
    };
} *udev = (void *) (USB_BASE + 0xC700);

// Device Event Buffer Structure
typedef union {
    _rw val : 32;

    struct {
        _rw sig : 1;    // 0 = DEPEVT
        _rw ep  : 5;    // Physical Endpoint Number (0–31)
        _rw evt : 4;    // Event Type
                        //      0x1 = XferComplete
                        //      0x2 = XferInProgress
                        //      0x3 = XferNotReady
                        //      0x6 = StreamEvt
                        //      0x7 = EPCmdCmplt
        __r : 2;        // Reserved
        _rw sts : 4;    // Event-specific status (meaning varies by type)
        _rw par : 16;   // Stream ID, IsoMicroFrameNum, etc.
    } depevt;           // Device Endpoint-Specific Event (DEPEVT)
    
    struct {
        _rw sig  : 8;   // 0x01 = DEVT
        _rw evt  : 5;   // Event Type (see below)
        __r : 3;        // Reserved
        _rw info : 9;   // Event-specific information (e.g., link state)
        __r : 7;        // Reserved
    } devt;             // Device Specific Events (DEVT)

} evt_t;

// Command type the SW driver is requesting the controller to perform
enum {
    CMDTYP_RESERVED = 0,
    CMDTYP_DEPCFG = 1,      // Set Endpoint Configuration 64- or 96-bit param
    CMDTYP_DEPXFERCFG = 2,  // Set Endpoint Transfare Resource Configuration: 32-bit param
    CMDTYP_DEPGETSTATE = 3, // Get Endpoint State: No param
    CMDTYP_DEPSSTALL = 4,   // Set Stall: No param 
    CMDTYP_DEPCSTALL = 5,   // Clear Stall: No param
    CMDTYP_DEPSTRTXFER = 6, // Start Transfer 64-bit param 
    CMDTYP_DEPUPDXFER = 7,  // Update Transfare: No param
    CMDTYP_DEPENDXFER = 8,  // End Transfer: No param
    CMDTYP_DEPSTARTCFG = 9  // Start New Configuration: No param
} DEPCMD_CMDTYP;

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

enum {
    // reserved
    DEPEVT_XFERCOMPLETE = 1,
    DEPEVT_XFERINPROGRESS,
    DEPEVT_XFERNOTREADY,
    // not implemented
    DEPEVT_STREAMEVT = 6,
    DEPEVT_EPCMDCMPLT
} DEPEVT;

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

#endif // DCD_ENSEMBLE_DEF_H