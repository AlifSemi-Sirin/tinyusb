/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef DCD_ENSEMBLE_H_
#define DCD_ENSEMBLE_H_

/* Enable clock supply for USB */
#define PERIPH_CLK_ENA_USB_CKEN (1U << 20)

/* CGU CLK_ENA field definitions */
#define CLK_ENA_CLK160M    (1U << 20) /* Enable 160M_CLK */
#define CLK_ENA_CLK100M    (1U << 21) /* Enable 100M_CLK */
#define CLK_ENA_CLK20M     (1U << 22) /* Enable USB and 10M_CLK */
#define CLK_ENA_CLK38P4M   (1U << 23) /* Enable HFOSC_CLK */



// Option for tinyUSB porting
#define TUD_OPT_RHPORT 0

#define RTSS_FORCE_GLOBAL_CLEAN_INVALIDATE_THRESHOLD_SIZE (128*1024)

// Structure for USB TRB (Transfer Request Block)
typedef union {
	uint32_t val;
	struct {
		uint32_t is_devt: 1;      // [0]     0 = DEPEVT, 1 = DEVT
		uint32_t ep: 5;       // [5:1]   Physical Endpoint Number (0–31)
		uint32_t evt: 4;   // [9:6]   Event Type
					  //         0x1 = XferComplete
					  //         0x2 = XferInProgress
					  //         0x3 = XferNotReady
					  //         0x6 = StreamEvt
					  //         0x7 = EPCmdCmplt
		uint32_t reserved: 2;     // [11:10] Reserved
		uint32_t sts: 4; // [15:12] Event-specific status (meaning varies by type)
		uint32_t par: 16; // [31:16] Stream ID, IsoMicroFrameNum, etc.
	} depevt;

	struct {
		uint32_t sig: 8;  // [7:0]   Always 1/0 for DEVT/(Device-specific event marker)
		uint32_t evt: 5; // [12:8]  Event Type (see below)
		uint32_t reserved1: 3; // [15:13] Reserved
		uint32_t info: 9; // [24:16] Event-specific information (e.g., link state)
		uint32_t reserved2: 7;  // [31:25] Reserved
	} devt;
} evt_t;


// Prototype for the functions
bool RTSS_IsCacheClean_Required_by_Addr (volatile void *addr, int32_t size);


static inline void enable_usb_periph_clk(void)
{
	sys_set_bits(EXPMST_PERIPH_CLK_EN, PERIPH_CLK_ENA_USB_CKEN);
}

static inline void disable_usb_periph_clk(void)
{
	sys_clear_bits(EXPMST_PERIPH_CLK_EN, PERIPH_CLK_ENA_USB_CKEN);
}

static inline void enable_cgu_clk20m(void)
{
	sys_set_bits(CGU_CLK_ENA, CLK_ENA_CLK20M);
}

static inline void disable_cgu_clk20m(void)
{
	sys_clear_bits(CGU_CLK_ENA, CLK_ENA_CLK20M);
}

static inline void enable_usb_phy_power(void)
{
	sys_clear_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_PWR_MASK);
}

static inline void disable_usb_phy_power(void)
{
	sys_set_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_PWR_MASK);
}

static inline void enable_usb_phy_isolation(void)
{
	sys_set_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_ISO);
}

static inline void disable_usb_phy_isolation(void)
{
	sys_clear_bits(VBAT_PWR_CTRL, PWR_CTRL_UPHY_ISO);
}

static inline void usb_ctrl2_phy_power_on_reset_set()
{
	sys_set_bits(EXPMST_USB_CTRL2, USB_CTRL2_POR_RST_MASK);
}

static inline void usb_ctrl2_phy_power_on_reset_clear()
{
	sys_clear_bits(EXPMST_USB_CTRL2, USB_CTRL2_POR_RST_MASK);
}

static inline void sb_dc_alif_int_enable()
{
	irq_enable(USB_ALIF_IRQ);
}

static inline void sb_dc_alif_int_disable()
{
	irq_disable(USB_ALIF_IRQ);
}



/**
  \fn          void RTSS_CleanDCache_by_Addr (volatile void *addr, int32_t dsize)
  \brief       Add a wrapper on the CleanDcache APIs so that
               TCM regions are ignored.
  \param[in]   addr    address
  \param[in]   dsize   size of memory block (in number of bytes)
*/
__STATIC_FORCEINLINE
void RTSS_CleanDCache_by_Addr (volatile void *addr, int32_t dsize)
{
    if(RTSS_IsCacheClean_Required_by_Addr (addr, dsize))
    {
        /*
         * Considering the time required to Clean by address for more
         * than 128K size, it is better to do global clean.
         *
         * Perform the check for threshold size and decide.
         *
         */
        if (dsize < RTSS_FORCE_GLOBAL_CLEAN_INVALIDATE_THRESHOLD_SIZE)
            SCB_CleanDCache_by_Addr (addr, dsize);
        else
            SCB_CleanDCache ();
    }
    else
    {
        __DSB();
        __ISB();
    }
}



#endif /* DCD_ENSEMBLE_H_ */