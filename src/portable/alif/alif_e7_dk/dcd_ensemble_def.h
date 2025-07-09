#ifndef DCD_ENSEMBLE_DEF_H
#define DCD_ENSEMBLE_DEF_H

#if CFG_TUSB_OS == OPT_OS_ZEPHYR

#include <zephyr/sys/util.h> // for BIT macros
#include <soc_common.h>
#include <soc_memory_map.h>    

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

/* USB_CTRL2 Register @ Offset 0xAC (CLKCTL_PER_MST 0x4903_F000) */
#define USB_CTRL2_POR_RST_MASK BIT(8)      /* POR_RST_MASK: USB PHY PoR Reset Mask (bit 8) */
// #define USB_CTRL2_PME_EN       BIT(6)      /* PME_EN: Power Management Enable (bit 6) */
// #define USB_CTRL2_FLADJ_MASK   (0x3F << 0) /* FLADJ_30MHZ_REG: Frequency Adjust Val (bits [5:0]) */
// #define USB_CTRL2_FLADJ_SET(x) (((x) & 0x3F) << 0)

#define USB_NODE DT_NODELABEL(usb0)
#define USB_IRQ_IRQn DT_IRQN(USB_NODE)

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

// Base address of USB device controller 
#define USB_BASE                0x48200000UL

// USB Registers Access Types
#define _rw volatile uint32_t
#define __w volatile uint32_t
#define __r volatile const uint32_t

// USB GBL Registers
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

} *ugbl = (void *) (USB_BASE + 0xC100);

volatile struct {
    union {
        _rw dcfg;                       // Device Configuration Register 0xC700
        struct {
            _rw devspd : 3;             // Device Speed
            _rw devaddr : 7;            // Device Address
            __r : 2;                    // Reserved
            _rw intrnum : 5;            // Interrupt number
            _rw nump : 5;               // Number of Receive Buffers
            _rw lpmcap : 1;             // LPM Capable
            _rw ignstrmpp : 1;          // Ignore Stream PP
            __r : 8;
        } dcfg_b;
    };

    union {
        _rw dctl;                       // Device Control register 0xC704
        struct {
            __r : 1;                    // Reserved
            _rw tstctl : 4;             // Test Control
            __w ulstchngreq : 4;        // USB/Link state change request
            __r : 7;                    // Reserved
            _rw css : 1;                // Controller Save State (CSS)
            _rw crs : 1;                // Controller Restore State (CRC)
            _rw l1hibernationen : 1;    // L1 Hibernation Enable    
            _rw keepconnect : 1;        // Keep Connect
            _rw lpm_nyet_thres : 4;     // LPM NYET Threshold
            _rw hirdthres : 5;          // HIRD Threshold
            __r : 1;                    // Reserved
            _rw csftrst : 1;            // Core Soft Reset
            _rw run_stop : 1;           // Start/Stop the device controller operation
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