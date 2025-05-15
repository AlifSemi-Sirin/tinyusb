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

#define RTSS_FORCE_GLOBAL_CLEAN_INVALIDATE_THRESHOLD_SIZE (128*1024)

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