/*
 * Copyright (C) ST-Ericsson SA 2010
 *
 * ST-Ericsson MCDE base driver
 *
 * Author: Marcus Lorentzon <marcus.xm.lorentzon@stericsson.com>
 * for ST-Ericsson.
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>

#include <linux/mfd/dbx500-prcmu.h>

#include <video/mcde.h>
#include "dsilink_regs.h"
#include "mcde_regs.h"
#include "mcde_debugfs.h"


/* MCDE channel states
 *
 * Allowed state transitions:
 *   IDLE <-> SUSPEND
 *   IDLE <-> DSI_READ
 *   IDLE <-> DSI_WRITE
 *   IDLE -> SETUP -> (WAIT_TE ->) RUNNING -> STOPPING1 -> STOPPING2 -> IDLE
 *   WAIT_TE -> STOPPED (for missing TE to allow re-enable)
 */
enum chnl_state {
	CHNLSTATE_SUSPEND,   /* HW in suspended mode, initial state */
	CHNLSTATE_IDLE,      /* Channel aquired, but not running, FLOEN==0 */
	CHNLSTATE_DSI_READ,  /* Executing DSI read */
	CHNLSTATE_DSI_WRITE, /* Executing DSI write */
	CHNLSTATE_SETUP,     /* Channel register setup to prepare for running */
	CHNLSTATE_WAIT_TE,   /* Waiting for BTA or external TE */
	CHNLSTATE_RUNNING,   /* Update started, FLOEN=1, FLOEN==1 */
	CHNLSTATE_STOPPING,  /* Stopping, FLOEN=0, FLOEN==1, awaiting VCMP */
	CHNLSTATE_STOPPED,   /* Stopped, after VCMP, FLOEN==0|1 */
};

enum dsi_lane_status {
	DSI_LANE_STATE_START	= 0x00,
	DSI_LANE_STATE_IDLE	= 0x01,
	DSI_LANE_STATE_WRITE	= 0x02,
	DSI_LANE_STATE_ULPM	= 0x03,
};

static int set_channel_state_atomic(struct mcde_chnl_state *chnl,
							enum chnl_state state);
static int set_channel_state_sync(struct mcde_chnl_state *chnl,
							enum chnl_state state);
static void stop_channel(struct mcde_chnl_state *chnl);
static int _mcde_chnl_enable(struct mcde_chnl_state *chnl);
static int _mcde_chnl_apply(struct mcde_chnl_state *chnl);
static void disable_flow(struct mcde_chnl_state *chnl);
static void enable_flow(struct mcde_chnl_state *chnl);
static void do_softwaretrig(struct mcde_chnl_state *chnl);
static void dsi_te_poll_req(struct mcde_chnl_state *chnl);
static void dsi_te_poll_set_timer(struct mcde_chnl_state *chnl,
		unsigned int timeout);
static void dsi_te_timer_function(unsigned long value);
static int wait_for_vcmp(struct mcde_chnl_state *chnl);
static int probe_hw(struct platform_device *pdev);
static void wait_for_flow_disabled(struct mcde_chnl_state *chnl);

#define OVLY_TIMEOUT 100
#define CHNL_TIMEOUT 100
#define FLOW_STOP_TIMEOUT 20
#define SCREEN_PPL_HIGH 1280
#define SCREEN_PPL_CEA2 720
#define SCREEN_LPF_CEA2 480
#define DSI_DELAY0_CEA2_ADD 10

#define MCDE_SLEEP_WATCHDOG 500
#define DSI_TE_NO_ANSWER_TIMEOUT_INIT 2500
#define DSI_TE_NO_ANSWER_TIMEOUT 250
#define DSI_WAIT_FOR_ULPM_STATE_MS 1
#define DSI_ULPM_STATE_NBR_OF_RETRIES 10
#define DSI_READ_TIMEOUT 200
#define DSI_WRITE_CMD_TIMEOUT 1000
#define DSI_READ_DELAY 5
#define DSI_READ_NBR_OF_RETRIES 2
#define MCDE_FLOWEN_MAX_TRIAL 60
#define MAX_CONSECUTIVE_CHNL0_TIMEOUTS 2

#define MCDE_VERSION_4_1_3 0x04010300
#define MCDE_VERSION_4_0_4 0x04000400
#define MCDE_VERSION_3_0_8 0x03000800
#define MCDE_VERSION_3_0_5 0x03000500
#define MCDE_VERSION_1_0_4 0x01000400

#define CLK_MCDE	"mcde"
#define CLK_DPI		"lcd"

static u8 *mcdeio;
static u8 **dsiio;
static struct platform_device *mcde_dev;
static u8 num_dsilinks;
static u8 num_channels;
static u8 num_overlays;
static int mcde_irq;
static u32 input_fifo_size;
static u32 output_fifo_ab_size;
static u32 output_fifo_c0c1_size;
static bool disable_ulpm;

static struct regulator *regulator_vana;
static struct regulator *regulator_mcde_epod;
static struct regulator *regulator_esram_epod;
static struct clk *clock_mcde;

/* TODO remove when all platforms support dsilp and dsihs clocks */
static struct clk *clock_dsi;
static struct clk *clock_dsi_lp;

static u8 mcde_is_enabled;
static struct delayed_work hw_timeout_work;
static u8 dsi_pll_is_enabled;
static u8 dsi_ifc_is_supported;
static u8 dsi_use_clk_framework;
static u32 mcde_clk_rate; /* In Hz */
static u32 chnl0_timeouts;
static bool apply_extra_oled_color_conv;

static struct mcde_oled_transform rgb_2_rgb_extra = {
	.matrix = {
		{0x1000, 0x0000, 0x0000},
		{0x0000, 0x1000, 0x0000},
		{0x0000, 0x0000, 0x1000},
	},
	.offset = {0x0000, 0x0000, 0x0000},
};

static struct mcde_oled_transform yuv240_2_rgb = {
	/* Note that in MCDE YUV 422 pixels come as VYU pixels */
	.matrix = {
		{0x1990, 0x12A0, 0x0000},
		{0x2D00, 0x12A0, 0x2640},
		{0x0000, 0x12A0, 0x1FFF},
	},
	.offset = {0x2DF0, 0x0870, 0x3150},
};

static struct mcde_oled_transform yuv240_2_rgb_extra = {
	/* Note that in MCDE YUV 422 pixels come as VYU pixels */
	.matrix = {
		{0x1990, 0x12A0, 0x0000},
		{0x2D00, 0x12A0, 0x2640},
		{0x0000, 0x12A0, 0x1FFF},
	},
	.offset = {0x2DF0, 0x0870, 0x3150},
};

static struct mutex mcde_hw_lock;
static inline void mcde_lock(const char *func, int line)
{
	mutex_lock(&mcde_hw_lock);
	dev_vdbg(&mcde_dev->dev, "Enter MCDE: %s:%d\n", func, line);
}

static inline void mcde_unlock(const char *func, int line)
{
	dev_vdbg(&mcde_dev->dev, "Exit MCDE: %s:%d\n", func, line);
	mutex_unlock(&mcde_hw_lock);
}

static inline bool mcde_trylock(const char *func, int line)
{
	bool locked = mutex_trylock(&mcde_hw_lock) == 1;
	if (locked)
		dev_vdbg(&mcde_dev->dev, "Enter MCDE: %s:%d\n", func, line);
	return locked;
}

static u8 mcde_dynamic_power_management = true;

static inline u32 dsi_rreg(int i, u32 reg)
{
	return readl(dsiio[i] + reg);
}
static inline void dsi_wreg(int i, u32 reg, u32 val)
{
	writel(val, dsiio[i] + reg);
}

#define dsi_rfld(__i, __reg, __fld) \
({ \
	const u32 mask = __reg##_##__fld##_MASK; \
	const u32 shift = __reg##_##__fld##_SHIFT; \
	((dsi_rreg(__i, __reg) & mask) >> shift); \
})

#define dsi_wfld(__i, __reg, __fld, __val) \
({ \
	const u32 mask = __reg##_##__fld##_MASK; \
	const u32 shift = __reg##_##__fld##_SHIFT; \
	const u32 oldval = dsi_rreg(__i, __reg); \
	const u32 newval = ((__val) << shift); \
	dsi_wreg(__i, __reg, (oldval & ~mask) | (newval & mask)); \
})

static inline u32 mcde_rreg(u32 reg)
{
	return readl(mcdeio + reg);
}
static inline void mcde_wreg(u32 reg, u32 val)
{
	writel(val, mcdeio + reg);
}


#define mcde_rfld(__reg, __fld) \
({ \
	const u32 mask = __reg##_##__fld##_MASK; \
	const u32 shift = __reg##_##__fld##_SHIFT; \
	((mcde_rreg(__reg) & mask) >> shift); \
})

#define mcde_wfld(__reg, __fld, __val) \
({ \
	const u32 mask = __reg##_##__fld##_MASK; \
	const u32 shift = __reg##_##__fld##_SHIFT; \
	const u32 oldval = mcde_rreg(__reg); \
	const u32 newval = ((__val) << shift); \
	mcde_wreg(__reg, (oldval & ~mask) | (newval & mask)); \
})

struct ovly_regs {
	bool enabled;
	bool dirty;
	bool dirty_buf;

	u8   ch_id;
	u32  baseaddress0;
	u32  baseaddress1;
	u8   bits_per_pixel;
	u8   bpp;
	bool bgr;
	bool bebo;
	bool opq;
	u8   col_conv;
	u8   alpha_source;
	u8   alpha_value;
	u8   pixoff;
	u16  ppl;
	u16  lpf;
	u16  cropx;
	u16  cropy;
	u16  xpos;
	u16  ypos;
	u8   z;
};

struct mcde_ovly_state {
	bool inuse;
	u8 idx; /* MCDE overlay index */
	struct mcde_chnl_state *chnl; /* Owner channel */
	bool dirty;
	bool dirty_buf;

	/* Staged settings */
	u32 paddr;
	u16 stride;
	enum mcde_ovly_pix_fmt pix_fmt;

	u16 src_x;
	u16 src_y;
	u16 dst_x;
	u16 dst_y;
	u16 dst_z;
	u16 w;
	u16 h;

	u8 alpha_source;
	u8 alpha_value;

	/* Applied settings */
	struct ovly_regs regs;
	atomic_t update_sbb;
};

static struct mcde_ovly_state *overlays;

struct chnl_regs {
	bool dirty;

	bool floen;
	u16  x;
	u16  y;
	u16  ppl;
	u16  lpf;
	u8   bpp;
	bool internal_clk; /* CLKTYPE field */
	u16  pcd;
	u8   clksel;
	u8   cdwin;
	u16 (*map_r)(u8);
	u16 (*map_g)(u8);
	u16 (*map_b)(u8);
	bool palette_enable;
	bool oled_enable;
	bool background_yuv;
	bool bcd;
	bool roten;
	u8   rotdir;
	u32  rotbuf1;
	u32  rotbuf2;
	u32  rotbufsize;

	/* Blending */
	u8 blend_ctrl;
	bool blend_en;
	u8 alpha_blend;

	/* DSI */
	u8 dsipacking;
};

struct col_regs {
	bool dirty;

	u16 y_red;
	u16 y_green;
	u16 y_blue;
	u16 cb_red;
	u16 cb_green;
	u16 cb_blue;
	u16 cr_red;
	u16 cr_green;
	u16 cr_blue;
	u16 off_y;
	u16 off_cb;
	u16 off_cr;
};

struct tv_regs {
	bool dirty;

	u16 dho; /* TV mode: left border width; destination horizontal offset */
		 /* LCD MODE: horizontal back porch */
	u16 alw; /* TV mode: right border width */
		 /* LCD mode: horizontal front porch */
	u16 hsw; /* horizontal synch width */
	u16 dvo; /* TV mode: top border width; destination horizontal offset */
		 /* LCD MODE: vertical back porch */
	u16 bsl; /* TV mode: bottom border width; blanking start line */
		 /* LCD MODE: vertical front porch */
	/* field 1 */
	u16 bel1; /* TV mode: field total vertical blanking lines */
		 /* LCD mode: vertical sync width */
	u16 fsl1; /* field vbp */
	/* field 2 */
	u16 bel2;
	u16 fsl2;
	u8 tv_mode;
	bool sel_mode_tv;
	bool inv_clk;
	bool interlaced_en;
	u32 lcdtim1;
};

struct oled_regs {
	bool dirty;

	u16 alfa_red;
	u16 alfa_green;
	u16 alfa_blue;
	u16 beta_red;
	u16 beta_green;
	u16 beta_blue;
	u16 gamma_red;
	u16 gamma_green;
	u16 gamma_blue;
	u16 off_red;
	u16 off_green;
	u16 off_blue;
};

struct mcde_chnl_state {
	bool enabled;
	bool reserved;
	enum mcde_chnl id;
	enum mcde_fifo fifo;
	struct mcde_port port;
	struct mcde_ovly_state *ovly0;
	struct mcde_ovly_state *ovly1;
	enum chnl_state state;
	wait_queue_head_t state_waitq;
	wait_queue_head_t vcmp_waitq;
	wait_queue_head_t vsync_waitq;
	atomic_t vcmp_cnt;
	u64 vcmp_cnt_wait;
	atomic_t vsync_cnt;
	u64 vsync_cnt_wait;
	bool oled_color_conversion;
	struct timer_list dsi_te_timer;
	struct clk *clk_dsi_lp;
	struct clk *clk_dsi_hs;
	struct clk *clk_dpi;

	enum mcde_display_power_mode power_mode;

	/* Staged settings */
	u16 (*map_r)(u8);
	u16 (*map_g)(u8);
	u16 (*map_b)(u8);
	bool palette_enable;
	struct mcde_video_mode vmode;
	enum mcde_display_rotation rotation;
	u32 rotbuf1;
	u32 rotbuf2;
	u32 rotbufsize;

	struct mcde_col_transform rgb_2_ycbcr;
	struct mcde_col_transform ycbcr_2_rgb;
	struct mcde_col_transform *transform;
	struct mcde_oled_transform *oled_transform;

	/* Blending */
	u8 blend_ctrl;
	bool blend_en;
	u8 alpha_blend;

	/* Applied settings */
	struct chnl_regs regs;
	struct col_regs  col_regs;
	struct tv_regs   tv_regs;
	struct oled_regs oled_regs;

	/* an interlaced digital TV signal generates a VCMP per field */
	bool vcmp_per_field;
	bool even_vcmp;

	bool formatter_updated;
	bool esram_is_enabled;

	bool first_frame_vsync_fix;
};

static struct mcde_chnl_state *channels;
/*
 * Wait for CSM_RUNNING, all data sent for display
 */
static inline void wait_while_dsi_running(int lnk)
{
	u8 counter = DSI_READ_TIMEOUT;
	while (dsi_rfld(lnk, DSI_CMD_MODE_STS, CSM_RUNNING) && --counter) {
		dev_vdbg(&mcde_dev->dev,
			"%s: DSI link %u read running state retry %u times\n"
			, __func__, lnk, (DSI_READ_TIMEOUT - counter));
		udelay(DSI_READ_DELAY);
	}
	WARN_ON(!counter);
	if (!counter)
		dev_warn(&mcde_dev->dev,
			"%s: DSI link %u read timeout!\n", __func__, lnk);
}

static void enable_clocks_and_power(struct platform_device *pdev)
{
	struct mcde_platform_data *pdata = pdev->dev.platform_data;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	/* VANA should be enabled before a DSS hard reset */
	if (regulator_vana)
		WARN_ON_ONCE(regulator_enable(regulator_vana));

	WARN_ON_ONCE(regulator_enable(regulator_mcde_epod));

	if (!dsi_use_clk_framework)
		pdata->platform_set_clocks();

	WARN_ON_ONCE(clk_enable(clock_mcde));
}

static void disable_clocks_and_power(struct platform_device *pdev)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	clk_disable(clock_mcde);

	WARN_ON_ONCE(regulator_disable(regulator_mcde_epod));

	if (regulator_vana)
		WARN_ON_ONCE(regulator_disable(regulator_vana));
}

static void update_mcde_registers(void)
{
	struct mcde_platform_data *pdata = mcde_dev->dev.platform_data;

	/* Setup output muxing */
	mcde_wreg(MCDE_CONF0,
		MCDE_CONF0_IFIFOCTRLWTRMRKLVL(7) |
		MCDE_CONF0_OUTMUX0(pdata->outmux[0]) |
		MCDE_CONF0_OUTMUX1(pdata->outmux[1]) |
		MCDE_CONF0_OUTMUX2(pdata->outmux[2]) |
		MCDE_CONF0_OUTMUX3(pdata->outmux[3]) |
		MCDE_CONF0_OUTMUX4(pdata->outmux[4]) |
		pdata->syncmux);

	mcde_wfld(MCDE_RISPP, VCMPARIS, 1);
	mcde_wfld(MCDE_RISPP, VCMPBRIS, 1);
	mcde_wfld(MCDE_RISPP, VCMPC0RIS, 1);
	mcde_wfld(MCDE_RISPP, VCMPC1RIS, 1);

	/*
	 * Enable VCMP interrupts for all channels
	 * and VSYNC0 and VSYNC1 capture interrupts
	 */
	mcde_wreg(MCDE_IMSCPP,
		MCDE_IMSCPP_VCMPAIM(true) |
		MCDE_IMSCPP_VCMPBIM(true) |
		MCDE_IMSCPP_VSCC0IM(true) |
		MCDE_IMSCPP_VSCC1IM(true) |
		MCDE_IMSCPP_VCMPC0IM(true) |
		MCDE_IMSCPP_VCMPC1IM(true));

	mcde_wreg(MCDE_IMSCCHNL, MCDE_IMSCCHNL_CHNLAIM(0xf));
	mcde_wreg(MCDE_IMSCERR, 0xFFFF01FF);
}

static void dsi_link_handle_reset(u8 link, bool release)
{
	u32 value;

	value = prcmu_read(DB8500_PRCM_DSI_SW_RESET);
	if (release) {
		switch (link) {
		case 0:
			value |= DB8500_PRCM_DSI_SW_RESET_DSI0_SW_RESETN;
			break;
		case 1:
			value |= DB8500_PRCM_DSI_SW_RESET_DSI1_SW_RESETN;
			break;
		case 2:
			value |= DB8500_PRCM_DSI_SW_RESET_DSI2_SW_RESETN;
			break;
		default:
			break;
		}
	} else {
		switch (link) {
		case 0:
			value &= ~DB8500_PRCM_DSI_SW_RESET_DSI0_SW_RESETN;
			break;
		case 1:
			value &= ~DB8500_PRCM_DSI_SW_RESET_DSI1_SW_RESETN;
			break;
		case 2:
			value &= ~DB8500_PRCM_DSI_SW_RESET_DSI2_SW_RESETN;
			break;
		default:
			break;
		}
	}
	prcmu_write(DB8500_PRCM_DSI_SW_RESET, value);
}

static void dsi_link_switch_byte_clk(u8 link, bool to_system_clock)
{
	u32 value;

	value = prcmu_read(DB8500_PRCM_DSI_GLITCHFREE_EN);
	if (to_system_clock) {
		switch (link) {
		case 0:
			value |= DB8500_PRCM_DSI_GLITCHFREE_EN_DSI0_BYTE_CLK;
			break;
		case 1:
			value |= DB8500_PRCM_DSI_GLITCHFREE_EN_DSI1_BYTE_CLK;
			break;
		case 2:
			value |= DB8500_PRCM_DSI_GLITCHFREE_EN_DSI2_BYTE_CLK;
			break;
		default:
			break;
		}
	} else {
		switch (link) {
		case 0:
			value &= ~DB8500_PRCM_DSI_GLITCHFREE_EN_DSI0_BYTE_CLK;
			break;
		case 1:
			value &= ~DB8500_PRCM_DSI_GLITCHFREE_EN_DSI1_BYTE_CLK;
			break;
		case 2:
			value &= ~DB8500_PRCM_DSI_GLITCHFREE_EN_DSI2_BYTE_CLK;
			break;
		default:
			break;
		}

	}
	prcmu_write(DB8500_PRCM_DSI_GLITCHFREE_EN, value);
	dsi_wfld(link, DSI_MCTL_PLL_CTL, PLL_OUT_SEL, to_system_clock);
}

static void dsi_link_handle_ulpm(struct mcde_port *port, bool enter_ulpm)
{
	u8 link = port->link;
	u8 num_data_lanes = port->phy.dsi.num_data_lanes;
	u8 nbr_of_retries = 0;
	u8 lane_state;

	/* Some panels does not support ULPM */
	if (disable_ulpm)
		return;

	/*
	 * The D-PHY protocol specifies the time to leave the ULP mode
	 * in ms. It will at least take 1 ms to exit ULPM.
	 * The ULPOUT time value is using number of system clock ticks
	 * divided by 1000. The system clock for the DSI link is the MCDE
	 * clock.
	 */
	dsi_wreg(link, DSI_MCTL_ULPOUT_TIME,
			DSI_MCTL_ULPOUT_TIME_CKLANE_ULPOUT_TIME(0x1FF) |
			DSI_MCTL_ULPOUT_TIME_DATA_ULPOUT_TIME(0x1FF));

	if (enter_ulpm) {
		lane_state = DSI_LANE_STATE_ULPM;
		dsi_link_switch_byte_clk(link, true);
	}

	dsi_wfld(link, DSI_MCTL_MAIN_EN, DAT1_ULPM_REQ, enter_ulpm);
	dsi_wfld(link, DSI_MCTL_MAIN_EN, DAT2_ULPM_REQ,
					enter_ulpm && num_data_lanes == 2);
	dsi_wfld(link, DSI_MCTL_MAIN_EN, CLKLANE_ULPM_REQ, enter_ulpm);

	if (!enter_ulpm) {
		lane_state = DSI_LANE_STATE_IDLE;
		dsi_link_switch_byte_clk(link, false);
	}

	/* Wait for data lanes to enter ULPM */
	while (dsi_rfld(link, DSI_MCTL_LANE_STS, DATLANE1_STATE)
						!= lane_state ||
		(dsi_rfld(link, DSI_MCTL_LANE_STS, DATLANE2_STATE)
						!= lane_state &&
							num_data_lanes > 1)) {
		mdelay(DSI_WAIT_FOR_ULPM_STATE_MS);
		if (nbr_of_retries++ == DSI_ULPM_STATE_NBR_OF_RETRIES) {
			dev_dbg(&mcde_dev->dev,
				"Could not enter correct state=%d (link=%d)!\n",
							lane_state, link);
			break;
		}
	}

	nbr_of_retries = 0;
	/* Wait for clock lane to enter ULPM */
	while (dsi_rfld(link, DSI_MCTL_LANE_STS, CLKLANE_STATE)
						!= lane_state) {
		mdelay(DSI_WAIT_FOR_ULPM_STATE_MS);
		if (nbr_of_retries++ == DSI_ULPM_STATE_NBR_OF_RETRIES) {
			dev_dbg(&mcde_dev->dev,
				"Could not enter correct state=%d (link=%d)!\n",
							lane_state, link);
			break;
		}
	}
}

static int dsi_link_enable(struct mcde_chnl_state *chnl)
{
	int ret = 0;
	u8 link = chnl->port.link;

	if (dsi_use_clk_framework) {
		WARN_ON_ONCE(clk_enable(chnl->clk_dsi_lp));
		WARN_ON_ONCE(clk_enable(chnl->clk_dsi_hs));
		dsi_link_handle_reset(link, true);
	} else {
		WARN_ON_ONCE(clk_enable(clock_dsi));
		WARN_ON_ONCE(clk_enable(clock_dsi_lp));

		if (!dsi_pll_is_enabled) {
			struct mcde_platform_data *pdata =
					mcde_dev->dev.platform_data;
			ret = pdata->platform_enable_dsipll();
			if (ret < 0) {
				dev_warn(&mcde_dev->dev, "%s: "
					"enable_dsipll failed ret = %d\n",
								__func__, ret);
				goto enable_dsipll_err;
			}
			dev_dbg(&mcde_dev->dev, "%s enable dsipll\n",
								__func__);
		}
		dsi_pll_is_enabled++;
	}

	dsi_wfld(link, DSI_MCTL_MAIN_DATA_CTL, LINK_EN, true);

	dev_dbg(&mcde_dev->dev, "DSI%d LINK_EN\n", link);

	return 0;

enable_dsipll_err:
	clk_disable(clock_dsi_lp);
	clk_disable(clock_dsi);
	return ret;
}

static void dsi_link_disable(struct mcde_chnl_state *chnl, bool suspend)
{
	wait_while_dsi_running(chnl->port.link);
	dsi_link_handle_ulpm(&chnl->port, true);
	if (dsi_use_clk_framework) {
		clk_disable(chnl->clk_dsi_lp);
		clk_disable(chnl->clk_dsi_hs);
	} else {
		if (dsi_pll_is_enabled && (--dsi_pll_is_enabled == 0)) {
			struct mcde_platform_data *pdata =
				    mcde_dev->dev.platform_data;
			dev_dbg(&mcde_dev->dev, "%s disable dsipll\n",
								__func__);
			pdata->platform_disable_dsipll();
		}
		clk_disable(clock_dsi);
		clk_disable(clock_dsi_lp);
	}
}

static void disable_mcde_hw(bool force_disable, bool suspend)
{
	int i;
	bool mcde_up = false;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (!mcde_is_enabled)
		return;

	for (i = 0; i < num_channels; i++) {
		struct mcde_chnl_state *chnl = &channels[i];
		if (force_disable || (chnl->enabled &&
					chnl->state != CHNLSTATE_RUNNING)) {
			stop_channel(chnl);
			if (chnl->formatter_updated) {
				if (chnl->port.type == MCDE_PORTTYPE_DSI)
					dsi_link_disable(chnl, suspend);
				else if (chnl->port.type == MCDE_PORTTYPE_DPI)
					clk_disable(chnl->clk_dpi);
				chnl->formatter_updated = false;
			}
			if (chnl->esram_is_enabled) {
				WARN_ON_ONCE(regulator_disable(
							regulator_esram_epod));
				chnl->esram_is_enabled = false;
			}
		} else if (chnl->enabled && chnl->state == CHNLSTATE_RUNNING) {
			mcde_up = true;
		}
	}

	if (mcde_up)
		return;

	free_irq(mcde_irq, &mcde_dev->dev);

	disable_clocks_and_power(mcde_dev);

	mcde_is_enabled = false;
}

static void dpi_video_mode_apply(struct mcde_chnl_state *chnl)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);
	chnl->tv_regs.interlaced_en = chnl->vmode.interlaced;

	chnl->tv_regs.sel_mode_tv = chnl->port.phy.dpi.tv_mode;
	if (chnl->tv_regs.sel_mode_tv) {
		/* TV mode */
		u32 bel;
		/* -4 since hsw is excluding SAV/EAV, 2 bytes each */
		chnl->tv_regs.hsw  = chnl->vmode.hbp + chnl->vmode.hfp - 4;
		/* vbp_field2 = vbp_field1 + 1 */
		chnl->tv_regs.fsl1 = chnl->vmode.vbp / 2;
		chnl->tv_regs.fsl2 = chnl->vmode.vbp - chnl->tv_regs.fsl1;
		/* +1 since vbp_field2 = vbp_field1 + 1 */
		bel = chnl->vmode.vbp + chnl->vmode.vfp;
		/* in TV mode: bel2 = bel1 + 1 */
		chnl->tv_regs.bel1 = bel / 2;
		chnl->tv_regs.bel2 = bel - chnl->tv_regs.bel1;
		if (chnl->port.phy.dpi.bus_width == 4)
			chnl->tv_regs.tv_mode = MCDE_TVCRA_TVMODE_SDTV_656P_BE;
		else
			chnl->tv_regs.tv_mode = MCDE_TVCRA_TVMODE_SDTV_656P;
		chnl->tv_regs.inv_clk = true;
	} else {
		/* LCD mode */
		u32 polarity;
		chnl->tv_regs.hsw  = chnl->vmode.hsw;
		chnl->tv_regs.dho  = chnl->vmode.hbp;
		chnl->tv_regs.alw  = chnl->vmode.hfp;
		chnl->tv_regs.bel1 = chnl->vmode.vsw;
		chnl->tv_regs.bel2 = chnl->tv_regs.bel1;
		chnl->tv_regs.dvo  = chnl->vmode.vbp;
		chnl->tv_regs.bsl  = chnl->vmode.vfp;
		chnl->tv_regs.fsl1 = 0;
		chnl->tv_regs.fsl2 = 0;
		polarity = chnl->port.phy.dpi.polarity;
		chnl->tv_regs.lcdtim1 = MCDE_LCDTIM1A_IHS(
				(polarity & DPI_ACT_LOW_HSYNC) != 0);
		chnl->tv_regs.lcdtim1 |= MCDE_LCDTIM1A_IVS(
				(polarity & DPI_ACT_LOW_VSYNC) != 0);
		chnl->tv_regs.lcdtim1 |= MCDE_LCDTIM1A_IOE(
				(polarity & DPI_ACT_LOW_DATA_ENABLE) != 0);
		chnl->tv_regs.lcdtim1 |= MCDE_LCDTIM1A_IPC(
				(polarity & DPI_ACT_ON_FALLING_EDGE) != 0);
	}
	chnl->tv_regs.dirty = true;
}

static void update_dpi_registers(enum mcde_chnl chnl_id, struct tv_regs *regs)
{
	u8 idx = chnl_id;

	dev_dbg(&mcde_dev->dev, "%s\n", __func__);
	mcde_wreg(MCDE_TVCRA + idx * MCDE_TVCRA_GROUPOFFSET,
			MCDE_TVCRA_SEL_MOD(regs->sel_mode_tv)             |
			MCDE_TVCRA_INTEREN(regs->interlaced_en)           |
			MCDE_TVCRA_IFIELD(0)                              |
			MCDE_TVCRA_TVMODE(regs->tv_mode)                  |
			MCDE_TVCRA_SDTVMODE(MCDE_TVCRA_SDTVMODE_Y0CBY1CR) |
			MCDE_TVCRA_CKINV(regs->inv_clk)                   |
			MCDE_TVCRA_AVRGEN(0));
	mcde_wreg(MCDE_TVBLUA + idx * MCDE_TVBLUA_GROUPOFFSET,
		MCDE_TVBLUA_TVBLU(MCDE_CONFIG_TVOUT_BACKGROUND_LUMINANCE) |
		MCDE_TVBLUA_TVBCB(MCDE_CONFIG_TVOUT_BACKGROUND_CHROMINANCE_CB)|
		MCDE_TVBLUA_TVBCR(MCDE_CONFIG_TVOUT_BACKGROUND_CHROMINANCE_CR));

	/* Vertical timing registers */
	mcde_wreg(MCDE_TVDVOA + idx * MCDE_TVDVOA_GROUPOFFSET,
					MCDE_TVDVOA_DVO1(regs->dvo) |
					MCDE_TVDVOA_DVO2(regs->dvo));
	mcde_wreg(MCDE_TVBL1A + idx * MCDE_TVBL1A_GROUPOFFSET,
					MCDE_TVBL1A_BEL1(regs->bel1) |
					MCDE_TVBL1A_BSL1(regs->bsl));
	mcde_wreg(MCDE_TVBL2A + idx * MCDE_TVBL1A_GROUPOFFSET,
					MCDE_TVBL2A_BEL2(regs->bel2) |
					MCDE_TVBL2A_BSL2(regs->bsl));
	mcde_wreg(MCDE_TVISLA + idx * MCDE_TVISLA_GROUPOFFSET,
					MCDE_TVISLA_FSL1(regs->fsl1) |
					MCDE_TVISLA_FSL2(regs->fsl2));

	/* Horizontal timing registers */
	mcde_wreg(MCDE_TVLBALWA + idx * MCDE_TVLBALWA_GROUPOFFSET,
				MCDE_TVLBALWA_LBW(regs->hsw) |
				MCDE_TVLBALWA_ALW(regs->alw));
	mcde_wreg(MCDE_TVTIM1A + idx * MCDE_TVTIM1A_GROUPOFFSET,
				MCDE_TVTIM1A_DHO(regs->dho));
	if (!regs->sel_mode_tv)
		mcde_wreg(MCDE_LCDTIM1A + idx * MCDE_LCDTIM1A_GROUPOFFSET,
								regs->lcdtim1);
	regs->dirty = false;
}

static void update_oled_registers(enum mcde_chnl chnl_id,
							struct oled_regs *regs)
{
	u8 idx = chnl_id;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);
	mcde_wreg(MCDE_OLEDCONV1A + idx * MCDE_OLEDCONV1A_GROUPOFFSET,
				MCDE_OLEDCONV1A_ALPHA_RED(regs->alfa_red) |
				MCDE_OLEDCONV1A_ALPHA_GREEN(regs->alfa_green));
	mcde_wreg(MCDE_OLEDCONV2A + idx * MCDE_OLEDCONV2A_GROUPOFFSET,
				MCDE_OLEDCONV2A_ALPHA_BLUE(regs->alfa_blue) |
				MCDE_OLEDCONV2A_BETA_RED(regs->beta_red));
	mcde_wreg(MCDE_OLEDCONV3A + idx * MCDE_OLEDCONV3A_GROUPOFFSET,
				MCDE_OLEDCONV3A_BETA_GREEN(regs->beta_green) |
				MCDE_OLEDCONV3A_BETA_BLUE(regs->beta_blue));
	mcde_wreg(MCDE_OLEDCONV4A + idx * MCDE_OLEDCONV4A_GROUPOFFSET,
				MCDE_OLEDCONV4A_GAMMA_RED(regs->gamma_red) |
				MCDE_OLEDCONV4A_GAMMA_GREEN(regs->gamma_green));
	mcde_wreg(MCDE_OLEDCONV5A + idx * MCDE_OLEDCONV5A_GROUPOFFSET,
				MCDE_OLEDCONV5A_GAMMA_BLUE(regs->gamma_blue) |
				MCDE_OLEDCONV5A_OFF_RED(regs->off_red));
	mcde_wreg(MCDE_OLEDCONV6A + idx * MCDE_OLEDCONV6A_GROUPOFFSET,
				MCDE_OLEDCONV6A_OFF_GREEN(regs->off_green) |
				MCDE_OLEDCONV6A_OFF_BLUE(regs->off_blue));
	regs->dirty = false;
}


static void update_col_registers(enum mcde_chnl chnl_id, struct col_regs *regs)
{
	u8 idx = chnl_id;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);
	mcde_wreg(MCDE_RGBCONV1A + idx * MCDE_RGBCONV1A_GROUPOFFSET,
				MCDE_RGBCONV1A_YR_RED(regs->y_red) |
				MCDE_RGBCONV1A_YR_GREEN(regs->y_green));
	mcde_wreg(MCDE_RGBCONV2A + idx * MCDE_RGBCONV2A_GROUPOFFSET,
				MCDE_RGBCONV2A_YR_BLUE(regs->y_blue) |
				MCDE_RGBCONV2A_CR_RED(regs->cr_red));
	mcde_wreg(MCDE_RGBCONV3A + idx * MCDE_RGBCONV3A_GROUPOFFSET,
				MCDE_RGBCONV3A_CR_GREEN(regs->cr_green) |
				MCDE_RGBCONV3A_CR_BLUE(regs->cr_blue));
	mcde_wreg(MCDE_RGBCONV4A + idx * MCDE_RGBCONV4A_GROUPOFFSET,
				MCDE_RGBCONV4A_CB_RED(regs->cb_red) |
				MCDE_RGBCONV4A_CB_GREEN(regs->cb_green));
	mcde_wreg(MCDE_RGBCONV5A + idx * MCDE_RGBCONV5A_GROUPOFFSET,
				MCDE_RGBCONV5A_CB_BLUE(regs->cb_blue) |
				MCDE_RGBCONV5A_OFF_RED(regs->off_cr));
	mcde_wreg(MCDE_RGBCONV6A + idx * MCDE_RGBCONV6A_GROUPOFFSET,
				MCDE_RGBCONV6A_OFF_GREEN(regs->off_y) |
				MCDE_RGBCONV6A_OFF_BLUE(regs->off_cb));
	regs->dirty = false;
}

/*
 * Update the double buffered overlay registers that contains
 * single buffered bits and include the single buffered bits.
 */
static void mcde_update_ovly_db_register_sbb(u8 idx, struct ovly_regs *regs,
		u8 col_conv)
{
	/* COLCCTRL, CKEYGEN, ALPHAPMEN are single buffered bits */
	mcde_wreg(MCDE_OVL0CR + idx * MCDE_OVL0CR_GROUPOFFSET,
		MCDE_OVL0CR_OVLEN(regs->enabled) |
		MCDE_OVL0CR_COLCCTRL(col_conv) |
		MCDE_OVL0CR_CKEYGEN(false) |
		MCDE_OVL0CR_ALPHAPMEN(false) |
		MCDE_OVL0CR_OVLF(false) |
		MCDE_OVL0CR_OVLR(false) |
		MCDE_OVL0CR_OVLB(false) |
		MCDE_OVL0CR_FETCH_ROPC(0) |
		MCDE_OVL0CR_STBPRIO(0) |
		MCDE_OVL0CR_BURSTSIZE_ENUM(8W) |
		/* TODO: enum, get from ovly */
		MCDE_OVL0CR_MAXOUTSTANDING_ENUM(8_REQ) |
		/* TODO: _8W, calculate? */
		MCDE_OVL0CR_ROTBURSTSIZE_ENUM(8W));
}

/*
 * Update the double buffered overlay registers that contains
 * single buffered bits and but maintain the single buffered bits.
 */
static void mcde_update_ovly_db_registers_not_sbb(u8 idx,
		struct ovly_regs *regs)
{
	u32 cr = mcde_rreg(MCDE_OVL0CR + idx * MCDE_OVL0CR_GROUPOFFSET);
	mcde_update_ovly_db_register_sbb(idx, regs,
			MCDE_REG2VAL(MCDE_OVL0CR, COLCCTRL, cr));
}

/* MCDE internal helpers */
static u8 portfmt2dsipacking(enum mcde_port_pix_fmt pix_fmt)
{
	switch (pix_fmt) {
	case MCDE_PORTPIXFMT_DSI_16BPP:
		return MCDE_DSIVID0CONF0_PACKING_RGB565;
	case MCDE_PORTPIXFMT_DSI_18BPP_PACKED:
		return MCDE_DSIVID0CONF0_PACKING_RGB666;
	case MCDE_PORTPIXFMT_DSI_18BPP:
	case MCDE_PORTPIXFMT_DSI_24BPP:
	default:
		return MCDE_DSIVID0CONF0_PACKING_RGB888;
	case MCDE_PORTPIXFMT_DSI_YCBCR422:
		return MCDE_DSIVID0CONF0_PACKING_HDTV;
	}
}

static u8 portfmt2bpp(enum mcde_port_pix_fmt pix_fmt)
{
	/* TODO: Check DPI spec *//* REVIEW: Remove or check */
	switch (pix_fmt) {
	case MCDE_PORTPIXFMT_DPI_16BPP_C1:
	case MCDE_PORTPIXFMT_DPI_16BPP_C2:
	case MCDE_PORTPIXFMT_DPI_16BPP_C3:
	case MCDE_PORTPIXFMT_DSI_16BPP:
	case MCDE_PORTPIXFMT_DSI_YCBCR422:
		return 16;
	case MCDE_PORTPIXFMT_DPI_18BPP_C1:
	case MCDE_PORTPIXFMT_DPI_18BPP_C2:
	case MCDE_PORTPIXFMT_DSI_18BPP_PACKED:
		return 18;
	case MCDE_PORTPIXFMT_DSI_18BPP:
	case MCDE_PORTPIXFMT_DPI_24BPP:
	case MCDE_PORTPIXFMT_DSI_24BPP:
		return 24;
	default:
		return 1;
	}
}

static u8 bpp2outbpp(u8 bpp)
{
	switch (bpp) {
	case 16:
		return MCDE_CRA1_OUTBPP_16BPP;
	case 18:
		return MCDE_CRA1_OUTBPP_18BPP;
	case 24:
		return MCDE_CRA1_OUTBPP_24BPP;
	default:
		return 0;
	}
}

static u8 portfmt2cdwin(enum mcde_port_pix_fmt pix_fmt)
{
	switch (pix_fmt) {
	case MCDE_PORTPIXFMT_DPI_16BPP_C1:
		return MCDE_CRA1_CDWIN_16BPP_C1;
	case MCDE_PORTPIXFMT_DPI_16BPP_C2:
		return MCDE_CRA1_CDWIN_16BPP_C2;
	case MCDE_PORTPIXFMT_DPI_16BPP_C3:
		return MCDE_CRA1_CDWIN_16BPP_C3;
	case MCDE_PORTPIXFMT_DPI_18BPP_C1:
		return MCDE_CRA1_CDWIN_18BPP_C1;
	case MCDE_PORTPIXFMT_DPI_18BPP_C2:
		return MCDE_CRA1_CDWIN_18BPP_C2;
	case MCDE_PORTPIXFMT_DPI_24BPP:
		return MCDE_CRA1_CDWIN_24BPP;
	default:
		/* only DPI formats are relevant */
		return 0;
	}
}

static u32 get_output_fifo_size(enum mcde_fifo fifo)
{
	u32 ret = 1; /* Avoid div by zero */

	switch (fifo) {
	case MCDE_FIFO_A:
	case MCDE_FIFO_B:
		ret = output_fifo_ab_size;
		break;
	case MCDE_FIFO_C0:
	case MCDE_FIFO_C1:
		ret = output_fifo_c0c1_size;
		break;
	default:
		dev_warn(&mcde_dev->dev, "Unsupported fifo");
		break;
	}
	return ret;
}

static inline u8 get_dsi_formatter_id(const struct mcde_port *port)
{
	if (dsi_ifc_is_supported)
		return 2 * port->link + port->ifc;
	else
		return port->link;
}

static struct mcde_chnl_state *find_channel_by_dsilink(int link)
{
	struct mcde_chnl_state *chnl = &channels[0];
	for (; chnl < &channels[num_channels]; chnl++)
		if (chnl->enabled && chnl->port.link == link &&
					chnl->port.type == MCDE_PORTTYPE_DSI)
			return chnl;
	return NULL;
}

static inline void mcde_handle_vsync(struct mcde_chnl_state *chnl)
{
	if (chnl->id == 0 && chnl0_timeouts != 0) {
		dev_info(&mcde_dev->dev,
			"%s: vsync received, chnl0_timeouts = %d\n",
			__func__, chnl0_timeouts);
	}
	if (!chnl->port.update_auto_trig &&
			chnl->port.type == MCDE_PORTTYPE_DSI &&
			chnl->state == CHNLSTATE_WAIT_TE) {
		set_channel_state_atomic(chnl, CHNLSTATE_RUNNING);
		switch (chnl->port.sync_src) {
		case MCDE_SYNCSRC_TE0:
			mcde_wfld(MCDE_CRC, SYCEN0, false);
			break;
		case MCDE_SYNCSRC_TE1:
			mcde_wfld(MCDE_CRC, SYCEN1, false);
			break;
		default:
			break;
		}
		disable_flow(chnl);
		set_channel_state_atomic(chnl, CHNLSTATE_STOPPING);
	} else if (chnl->port.update_auto_trig &&
			chnl->port.type == MCDE_PORTTYPE_DSI) {
		atomic_inc(&chnl->vsync_cnt);
		chnl->vcmp_cnt_wait = atomic_read(&chnl->vcmp_cnt) + 1;
		if (chnl->state == CHNLSTATE_STOPPING) {
			switch (chnl->port.sync_src) {
			case MCDE_SYNCSRC_TE0:
				mcde_wfld(MCDE_CRC, SYCEN0, false);
				break;
			case MCDE_SYNCSRC_TE1:
				mcde_wfld(MCDE_CRC, SYCEN1, false);
				break;
			default:
				break;
			}
			disable_flow(chnl);
		}
		wake_up_all(&chnl->vsync_waitq);
	}
}

static inline void mcde_handle_vcmp_state_stopping(struct mcde_chnl_state *chnl)
{
	bool change_state = true;
	if (chnl->port.update_auto_trig) {
		switch (chnl->port.sync_src) {
		case MCDE_SYNCSRC_TE0:
			change_state = !mcde_rfld(MCDE_CRC, SYCEN0);
			break;
		case MCDE_SYNCSRC_TE1:
			change_state = !mcde_rfld(MCDE_CRC, SYCEN1);
			break;
		case MCDE_SYNCSRC_OFF:
		case MCDE_SYNCSRC_BTA:
		case MCDE_SYNCSRC_TE_POLLING:
		default:
			break;
		}
	}
	if (change_state)
		set_channel_state_atomic(chnl, CHNLSTATE_STOPPED);
}

static inline void mcde_handle_vcmp(struct mcde_chnl_state *chnl)
{
	if (!chnl->vcmp_per_field ||
				(chnl->vcmp_per_field && chnl->even_vcmp)) {
		atomic_inc(&chnl->vcmp_cnt);
		chnl->vsync_cnt_wait = atomic_read(&chnl->vsync_cnt) + 1;
		if (chnl->state == CHNLSTATE_STOPPING)
			mcde_handle_vcmp_state_stopping(chnl);

		else if (chnl->id == 0)
			dev_warn(&mcde_dev->dev, "%s: Not in STOPPING state, "
				"current state = %d\n", __func__, chnl->state);
		wake_up_all(&chnl->vcmp_waitq);
	}
	chnl->even_vcmp = !chnl->even_vcmp;
	if (chnl->id == 0 && chnl0_timeouts != 0) {
		dev_info(&mcde_dev->dev, "%s: vcmp received, reset timeout "
							"counter\n", __func__);
		chnl0_timeouts = 0;
	}

	if (atomic_cmpxchg(&chnl->ovly0->update_sbb, 1, 0))
		mcde_update_ovly_db_register_sbb(chnl->ovly0->idx,
			&chnl->ovly0->regs, chnl->ovly0->regs.col_conv);

	if (atomic_cmpxchg(&chnl->ovly1->update_sbb, 1, 0))
		mcde_update_ovly_db_register_sbb(chnl->ovly1->idx,
			&chnl->ovly1->regs, chnl->ovly1->regs.col_conv);
}

static void handle_dsi_irq(struct mcde_chnl_state *chnl, int i)
{
	u32 irq_status = dsi_rfld(i, DSI_DIRECT_CMD_STS_FLAG, TE_RECEIVED_FLAG);
	if (irq_status) {
		dsi_wreg(i, DSI_DIRECT_CMD_STS_CLR,
				DSI_DIRECT_CMD_STS_CLR_TE_RECEIVED_CLR(true));
		dev_vdbg(&mcde_dev->dev, "BTA TE DSI%d\n", i);
		if (chnl->port.frame_trig == MCDE_TRIG_SW) {
			do_softwaretrig(chnl);
		} else {
			set_channel_state_atomic(chnl, CHNLSTATE_RUNNING);
			set_channel_state_atomic(chnl, CHNLSTATE_STOPPING);
		}
	}

	irq_status = dsi_rfld(i, DSI_CMD_MODE_STS_FLAG, ERR_NO_TE_FLAG);
	if (irq_status) {
		dsi_wreg(i, DSI_CMD_MODE_STS_CLR,
			DSI_CMD_MODE_STS_CLR_ERR_NO_TE_CLR(true));
		dev_warn(&mcde_dev->dev, "NO_TE DSI%d\n", i);
		set_channel_state_atomic(chnl, CHNLSTATE_STOPPED);
	}

	irq_status = dsi_rfld(i, DSI_DIRECT_CMD_STS, TRIGGER_RECEIVED);
	if (irq_status) {
		/* DSI TE polling answer received */
		dsi_wreg(i, DSI_DIRECT_CMD_STS_CLR,
			DSI_DIRECT_CMD_STS_CLR_TRIGGER_RECEIVED_CLR(true));

		/* Reset TE watchdog timer */
		if (chnl->port.sync_src == MCDE_SYNCSRC_TE_POLLING)
			dsi_te_poll_set_timer(chnl, DSI_TE_NO_ANSWER_TIMEOUT);
	}
}

static irqreturn_t mcde_irq_handler(int irq, void *dev)
{
	int i;
	u32 irq_status;
	u32 active_interrupts;

	active_interrupts = mcde_rreg(MCDE_AIS);

	if (active_interrupts & (MCDE_AIS_DSI0AI_MASK |
				MCDE_AIS_DSI1AI_MASK |
				MCDE_AIS_DSI2AI_MASK)) {
		for (i = 0; i < num_dsilinks; i++) {
			struct mcde_chnl_state *chnl_from_dsi;

			chnl_from_dsi = find_channel_by_dsilink(i);

			if (chnl_from_dsi == NULL)
				continue;

			handle_dsi_irq(chnl_from_dsi, i);
		}
	}

	if (active_interrupts & MCDE_AIS_MCDECHNLI_MASK) {
		irq_status = mcde_rreg(MCDE_MISCHNL);
		if (irq_status) {
			dev_err(&mcde_dev->dev,
					"chnl error=%.8x\n", irq_status);
			mcde_wreg(MCDE_RISCHNL, irq_status);
		}
	}
	if (active_interrupts & MCDE_AIS_MCDEERRI_MASK) {
		irq_status = mcde_rreg(MCDE_MISERR);
		if (irq_status) {
			dev_err(&mcde_dev->dev, "error=%.8x\n", irq_status);
			mcde_wreg(MCDE_RISERR, irq_status);
		}
	}

	if (active_interrupts & MCDE_AIS_MCDEPPI_MASK) {
		struct mcde_chnl_state *chnl;

		/* Handle channel irqs */
		irq_status = mcde_rreg(MCDE_MISPP);

		if (irq_status & MCDE_MISPP_VCMPAMIS_MASK)
			mcde_handle_vcmp(&channels[MCDE_CHNL_A]);
		if (irq_status & MCDE_MISPP_VCMPBMIS_MASK)
			mcde_handle_vcmp(&channels[MCDE_CHNL_B]);
		if (irq_status & MCDE_MISPP_VCMPC0MIS_MASK)
			mcde_handle_vcmp(&channels[MCDE_CHNL_C0]);
		if (irq_status & MCDE_MISPP_VCMPC1MIS_MASK)
			mcde_handle_vcmp(&channels[MCDE_CHNL_C1]);

		if (irq_status & MCDE_MISPP_VSCC0MIS_MASK) {
			for (chnl = channels; chnl < &channels[num_channels];
					chnl++) {
				if (chnl->port.sync_src == MCDE_SYNCSRC_TE0)
					mcde_handle_vsync(chnl);
			}
		}
		if (irq_status & MCDE_MISPP_VSCC1MIS_MASK) {
			for (chnl = channels; chnl < &channels[num_channels];
					chnl++) {
				if (chnl->port.sync_src == MCDE_SYNCSRC_TE1)
					mcde_handle_vsync(chnl);
			}
		}
		mcde_wreg(MCDE_RISPP, irq_status);
	}

	return IRQ_HANDLED;
}

/* Transitions allowed: WAIT_TE -> UPDATE -> STOPPING */
static int set_channel_state_atomic(struct mcde_chnl_state *chnl,
							enum chnl_state state)
{
	enum chnl_state chnl_state = chnl->state;

	dev_dbg(&mcde_dev->dev, "Channel state change"
		" (chnl=%d, old=%d, new=%d)\n", chnl->id, chnl_state, state);

	if ((chnl_state == CHNLSTATE_SETUP && state == CHNLSTATE_WAIT_TE) ||
	    (chnl_state == CHNLSTATE_SETUP && state == CHNLSTATE_RUNNING) ||
	    (chnl_state == CHNLSTATE_WAIT_TE && state == CHNLSTATE_RUNNING) ||
	    (chnl_state == CHNLSTATE_RUNNING && state == CHNLSTATE_STOPPING)) {
		/* Set wait TE, running, or stopping state */
		chnl->state = state;
		return 0;
	} else if ((chnl_state == CHNLSTATE_STOPPING &&
						state == CHNLSTATE_STOPPED) ||
		   (chnl_state == CHNLSTATE_WAIT_TE &&
						state == CHNLSTATE_STOPPED)) {
		/* Set stopped state */
		chnl->state = state;
		wake_up_all(&chnl->state_waitq);
		return 0;
	} else if (state == CHNLSTATE_IDLE) {
		/* Set idle state */
		WARN_ON_ONCE(chnl_state != CHNLSTATE_DSI_READ &&
			     chnl_state != CHNLSTATE_DSI_WRITE &&
			     chnl_state != CHNLSTATE_SUSPEND);
		chnl->state = state;
		wake_up_all(&chnl->state_waitq);
		return 0;
	} else {
		/* Invalid atomic state transition */
		dev_warn(&mcde_dev->dev, "Channel state change error (chnl=%d,"
			" old=%d, new=%d)\n", chnl->id, chnl_state, state);
		WARN_ON_ONCE(true);
		return -EINVAL;
	}
}

static void handle_chnl0_timeout(struct mcde_chnl_state *chnl)
{
	int ret;
	u32 data = 0;
	int len;

	if (chnl0_timeouts >= MAX_CONSECUTIVE_CHNL0_TIMEOUTS) {
		dev_info(&mcde_dev->dev,
			"%s: Nbr timeouts since last successful frame = %d\n",
			__func__, chnl0_timeouts);
		/* Something is wrong. Check display */
		/* Read Display Signal Mode reg (0x0E) */
		len = 1;
		ret = mcde_dsi_dcs_read(chnl, 0x0E, &data, &len);
		dev_info(&mcde_dev->dev,
			"%s: Read 0x0E ret = %d, data = 0x%x\n",
			__func__, ret, data);
		/* Read First display id byte (0xA1) */
		len = 1;
		ret = mcde_dsi_dcs_read(chnl, 0xA1, &data, &len);
		dev_info(&mcde_dev->dev,
			"%s: Read 0xA1 ret = %d, data = 0x%x\n",
			__func__, ret, data);
	}
}

/* LOCKING: mcde_hw_lock */
static int set_channel_state_sync(struct mcde_chnl_state *chnl,
							enum chnl_state state)
{
	int ret = 0;
	enum chnl_state chnl_state = chnl->state;

	dev_dbg(&mcde_dev->dev, "Channel state change"
		" (chnl=%d, old=%d, new=%d)\n", chnl->id, chnl->state, state);

	/* No change */
	if (chnl_state == state)
		return 0;

	/* Wait for IDLE before changing state */
	if (chnl_state != CHNLSTATE_IDLE) {
		ret = wait_event_timeout(chnl->state_waitq,
			/* STOPPED -> IDLE is manual, so wait for both */
			chnl->state == CHNLSTATE_STOPPED ||
			chnl->state == CHNLSTATE_IDLE,
						msecs_to_jiffies(CHNL_TIMEOUT));
		if (WARN_ON_ONCE(!ret)) {
			dev_warn(&mcde_dev->dev, "Wait for channel timeout "
						"(chnl=%d, curr=%d, new=%d)\n",
						chnl->id, chnl->state, state);
			chnl0_timeouts++;
		}
		chnl_state = chnl->state;
	}

	/* Do manual transition from STOPPED to IDLE */
	if (chnl_state == CHNLSTATE_STOPPED)
		wait_for_flow_disabled(chnl);

	/* State is IDLE, do transition to new state */
	chnl->state = state;

	return ret;
}

static int wait_for_vcmp(struct mcde_chnl_state *chnl)
{
	int ret;
	u64 w = chnl->vcmp_cnt_wait;
	ret = wait_event_timeout(chnl->vcmp_waitq,
			atomic_read(&chnl->vcmp_cnt) >= w,
			msecs_to_jiffies(CHNL_TIMEOUT));
	return ret;
}

static int wait_for_vsync(struct mcde_chnl_state *chnl)
{
	int ret;
	u64 w = chnl->vsync_cnt_wait;
	ret = wait_event_timeout(chnl->vsync_waitq,
			atomic_read(&chnl->vsync_cnt) >= w,
			msecs_to_jiffies(CHNL_TIMEOUT));
	return ret;
}

static void get_vid_operating_mode(const struct mcde_port *port,
		bool *burst_mode, bool *sync_is_pulse, bool *tvg_enable)
{
	switch (port->phy.dsi.vid_mode) {
	case NON_BURST_MODE_WITH_SYNC_EVENT:
		*burst_mode = false;
		*sync_is_pulse = false;
		*tvg_enable = false;
		break;
	case NON_BURST_MODE_WITH_SYNC_EVENT_TVG_ENABLED:
		*burst_mode = false;
		*sync_is_pulse = false;
		*tvg_enable = true;
		break;
	case BURST_MODE_WITH_SYNC_EVENT:
		*burst_mode = true;
		*sync_is_pulse = false;
		*tvg_enable = false;
		break;
	case BURST_MODE_WITH_SYNC_PULSE:
		*burst_mode = true;
		*sync_is_pulse = true;
		*tvg_enable = false;
		break;
	default:
		dev_err(&mcde_dev->dev, "Unsupported video mode");
		break;
	}
}

static void update_vid_static_registers(const struct mcde_port *port)
{
	u8 link = port->link;
	bool burst_mode, sync_is_pulse, tvg_enable;

	get_vid_operating_mode(port, &burst_mode, &sync_is_pulse, &tvg_enable);

	/* burst mode or non-burst mode */
	dsi_wfld(link, DSI_VID_MAIN_CTL, BURST_MODE, burst_mode);

	/* sync is pulse or event */
	dsi_wfld(link, DSI_VID_MAIN_CTL, SYNC_PULSE_ACTIVE, sync_is_pulse);
	dsi_wfld(link, DSI_VID_MAIN_CTL, SYNC_PULSE_HORIZONTAL, sync_is_pulse);

	/* disable video stream when using TVG */
	if (tvg_enable) {
		dsi_wfld(link, DSI_MCTL_MAIN_EN, IF1_EN, false);
		dsi_wfld(link, DSI_MCTL_MAIN_EN, IF2_EN, false);
	}

	/*
	 * behavior during blanking time
	 * 00: NULL packet 1x:LP 01:blanking-packet
	 */
	dsi_wfld(link, DSI_VID_MAIN_CTL, REG_BLKLINE_MODE, 1);

	/*
	 * behavior during eol
	 * 00: NULL packet 1x:LP 01:blanking-packet
	 */
	dsi_wfld(link, DSI_VID_MAIN_CTL, REG_BLKEOL_MODE, 2);

	/* time to perform LP->HS on D-PHY */
	dsi_wfld(link, DSI_VID_DPHY_TIME, REG_WAKEUP_TIME,
						port->phy.dsi.vid_wakeup_time);

	/*
	 * video stream starts on VSYNC packet
	 * and stops at the end of a frame
	 */
	dsi_wfld(link, DSI_VID_MAIN_CTL, VID_ID, port->phy.dsi.virt_id);
	dsi_wfld(link, DSI_VID_MAIN_CTL, START_MODE, 0);
	dsi_wfld(link, DSI_VID_MAIN_CTL, STOP_MODE, 0);

	/* 1: if1 in video mode, 0: if1 in command mode */
	dsi_wfld(link, DSI_MCTL_MAIN_DATA_CTL, IF1_MODE, 1);

	/* 1: enables the link, 0: disables the link */
	dsi_wfld(link, DSI_MCTL_MAIN_DATA_CTL, VID_EN, 1);
}

static int update_channel_static_registers(struct mcde_chnl_state *chnl)
{
	const struct mcde_port *port = &chnl->port;

	switch (chnl->fifo) {
	case MCDE_FIFO_A:
		mcde_wreg(MCDE_CHNL0MUXING + chnl->id *
			MCDE_CHNL0MUXING_GROUPOFFSET,
			MCDE_CHNL0MUXING_FIFO_ID_ENUM(FIFO_A));
		if (port->type == MCDE_PORTTYPE_DPI) {
			mcde_wfld(MCDE_CTRLA, FORMTYPE,
					MCDE_CTRLA_FORMTYPE_DPITV);
			mcde_wfld(MCDE_CTRLA, FORMID, port->link);
		} else if (port->type == MCDE_PORTTYPE_DSI) {
			mcde_wfld(MCDE_CTRLA, FORMTYPE,
					MCDE_CTRLA_FORMTYPE_DSI);
			mcde_wfld(MCDE_CTRLA, FORMID,
						get_dsi_formatter_id(port));
		}
		break;
	case MCDE_FIFO_B:
		mcde_wreg(MCDE_CHNL0MUXING + chnl->id *
			MCDE_CHNL0MUXING_GROUPOFFSET,
			MCDE_CHNL0MUXING_FIFO_ID_ENUM(FIFO_B));
		if (port->type == MCDE_PORTTYPE_DPI) {
			mcde_wfld(MCDE_CTRLB, FORMTYPE,
					MCDE_CTRLB_FORMTYPE_DPITV);
			mcde_wfld(MCDE_CTRLB, FORMID, port->link);
		} else if (port->type == MCDE_PORTTYPE_DSI) {
			mcde_wfld(MCDE_CTRLB, FORMTYPE,
					MCDE_CTRLB_FORMTYPE_DSI);
			mcde_wfld(MCDE_CTRLB, FORMID,
						get_dsi_formatter_id(port));
		}

		break;
	case MCDE_FIFO_C0:
		mcde_wreg(MCDE_CHNL0MUXING + chnl->id *
			MCDE_CHNL0MUXING_GROUPOFFSET,
			MCDE_CHNL0MUXING_FIFO_ID_ENUM(FIFO_C0));
		if (port->type == MCDE_PORTTYPE_DPI)
			return -EINVAL;
		mcde_wfld(MCDE_CTRLC0, FORMTYPE,
					MCDE_CTRLC0_FORMTYPE_DSI);
		mcde_wfld(MCDE_CTRLC0, FORMID, get_dsi_formatter_id(port));
		break;
	case MCDE_FIFO_C1:
		mcde_wreg(MCDE_CHNL0MUXING + chnl->id *
			MCDE_CHNL0MUXING_GROUPOFFSET,
			MCDE_CHNL0MUXING_FIFO_ID_ENUM(FIFO_C1));
		if (port->type == MCDE_PORTTYPE_DPI)
			return -EINVAL;
		mcde_wfld(MCDE_CTRLC1, FORMTYPE,
					MCDE_CTRLC1_FORMTYPE_DSI);
		mcde_wfld(MCDE_CTRLC1, FORMID, get_dsi_formatter_id(port));
		break;
	default:
		return -EINVAL;
	}

	/* Formatter */
	if (port->type == MCDE_PORTTYPE_DSI) {
		int i = 0;
		u8 idx;
		u8 lnk = port->link;

		idx = get_dsi_formatter_id(port);

		if (dsi_link_enable(chnl))
			goto failed_to_enable_link;

		if (port->sync_src == MCDE_SYNCSRC_TE_POLLING) {
			/* Enable DSI TE polling */
			dsi_te_poll_req(chnl);

			/* Set timer to detect non TE answer */
			dsi_te_poll_set_timer(chnl,
					DSI_TE_NO_ANSWER_TIMEOUT_INIT);
		} else {
			dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, BTA_EN, true);
			dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, READ_EN, true);
			dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, REG_TE_EN, true);
		}

		dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, HOST_EOT_GEN,
						port->phy.dsi.host_eot_gen);

		dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, DLX_REMAP_EN,
					port->phy.dsi.data_lanes_swap);

		dsi_wreg(lnk, DSI_MCTL_DPHY_STATIC,
			DSI_MCTL_DPHY_STATIC_UI_X4(port->phy.dsi.ui));
		dsi_wreg(lnk, DSI_DPHY_LANES_TRIM,
			DSI_DPHY_LANES_TRIM_DPHY_SPECS_90_81B_ENUM(0_90));
		dsi_wreg(lnk, DSI_MCTL_DPHY_TIMEOUT,
			DSI_MCTL_DPHY_TIMEOUT_CLK_DIV(0xf) |
			DSI_MCTL_DPHY_TIMEOUT_HSTX_TO_VAL(0x3fff) |
			DSI_MCTL_DPHY_TIMEOUT_LPRX_TO_VAL(0x3fff));
		dsi_wreg(lnk, DSI_MCTL_MAIN_PHY_CTL,
			DSI_MCTL_MAIN_PHY_CTL_WAIT_BURST_TIME(0xf) |
			DSI_MCTL_MAIN_PHY_CTL_CLK_ULPM_EN(true) |
			DSI_MCTL_MAIN_PHY_CTL_DAT1_ULPM_EN(true) |
			DSI_MCTL_MAIN_PHY_CTL_DAT2_ULPM_EN(true) |
			DSI_MCTL_MAIN_PHY_CTL_LANE2_EN(
				port->phy.dsi.num_data_lanes >= 2) |
			DSI_MCTL_MAIN_PHY_CTL_CLK_CONTINUOUS(
				port->phy.dsi.clk_cont));
		/* TODO: make enum */
		dsi_wfld(lnk, DSI_CMD_MODE_CTL, ARB_MODE, false);
		/* TODO: make enum */
		dsi_wfld(lnk, DSI_CMD_MODE_CTL, ARB_PRI, port->ifc == 1);
		dsi_wreg(lnk, DSI_MCTL_MAIN_EN,
			DSI_MCTL_MAIN_EN_PLL_START(true) |
			DSI_MCTL_MAIN_EN_CKLANE_EN(true) |
			DSI_MCTL_MAIN_EN_DAT1_EN(true) |
			DSI_MCTL_MAIN_EN_DAT2_EN(port->phy.dsi.num_data_lanes
				== 2) |
			DSI_MCTL_MAIN_EN_IF1_EN(port->ifc == 0) |
			DSI_MCTL_MAIN_EN_IF2_EN(port->ifc == 1));
		while (dsi_rfld(lnk, DSI_MCTL_MAIN_STS, CLKLANE_READY) == 0 ||
			dsi_rfld(lnk, DSI_MCTL_MAIN_STS, DAT1_READY) == 0 ||
			(dsi_rfld(lnk, DSI_MCTL_MAIN_STS, DAT2_READY) == 0 &&
					port->phy.dsi.num_data_lanes > 1)) {
			mdelay(1);
			if (i++ == 10) {
				dev_warn(&mcde_dev->dev,
					"DSI lane not ready (link=%d)!\n", lnk);
				goto dsi_link_error;
			}
		}

		dsi_link_handle_ulpm(&chnl->port, false);
		mcde_wreg(MCDE_DSIVID0CONF0 +
			idx * MCDE_DSIVID0CONF0_GROUPOFFSET,
			MCDE_DSIVID0CONF0_BLANKING(0) |
			MCDE_DSIVID0CONF0_VID_MODE(
				port->mode == MCDE_PORTMODE_VID) |
			MCDE_DSIVID0CONF0_CMD8(true) |
			MCDE_DSIVID0CONF0_BIT_SWAP(false) |
			MCDE_DSIVID0CONF0_BYTE_SWAP(false) |
			MCDE_DSIVID0CONF0_DCSVID_NOTGEN(true));

		if (port->mode == MCDE_PORTMODE_VID) {
			update_vid_static_registers(port);
		} else {
			if (port->ifc == 0)
				dsi_wfld(port->link, DSI_CMD_MODE_CTL, IF1_ID,
					port->phy.dsi.virt_id);
			else if (port->ifc == 1)
				dsi_wfld(port->link, DSI_CMD_MODE_CTL, IF2_ID,
					port->phy.dsi.virt_id);
		}
	}

	if (port->type == MCDE_PORTTYPE_DPI) {
		if (port->phy.dpi.lcd_freq != clk_round_rate(chnl->clk_dpi,
							port->phy.dpi.lcd_freq))
			dev_warn(&mcde_dev->dev, "Could not set lcd freq"
					" to %d\n", port->phy.dpi.lcd_freq);
		WARN_ON_ONCE(clk_set_rate(chnl->clk_dpi,
						port->phy.dpi.lcd_freq));
		WARN_ON_ONCE(clk_enable(chnl->clk_dpi));
	}

	mcde_wfld(MCDE_CR, MCDEEN, true);
	chnl->formatter_updated = true;

	dev_vdbg(&mcde_dev->dev, "Static registers setup, chnl=%d\n", chnl->id);

	return 0;
dsi_link_error:
	dsi_link_disable(chnl, true);
failed_to_enable_link:
	return -EINVAL;
}

static void mcde_chnl_oled_convert_apply(struct mcde_chnl_state *chnl,
					struct mcde_oled_transform *transform)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (chnl->oled_transform != transform) {
		chnl->oled_regs.alfa_red     = transform->matrix[0][0];
		chnl->oled_regs.alfa_green   = transform->matrix[0][1];
		chnl->oled_regs.alfa_blue    = transform->matrix[0][2];
		chnl->oled_regs.beta_red     = transform->matrix[1][0];
		chnl->oled_regs.beta_green   = transform->matrix[1][1];
		chnl->oled_regs.beta_blue    = transform->matrix[1][2];
		chnl->oled_regs.gamma_red    = transform->matrix[2][0];
		chnl->oled_regs.gamma_green  = transform->matrix[2][1];
		chnl->oled_regs.gamma_blue   = transform->matrix[2][2];
		chnl->oled_regs.off_red      = transform->offset[0];
		chnl->oled_regs.off_green    = transform->offset[1];
		chnl->oled_regs.off_blue     = transform->offset[2];
		chnl->oled_regs.dirty = true;

		chnl->oled_transform = transform;
	}

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);
}

void mcde_chnl_col_convert_apply(struct mcde_chnl_state *chnl,
					struct mcde_col_transform *transform)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (chnl->transform != transform) {

		chnl->col_regs.y_red     = transform->matrix[0][0];
		chnl->col_regs.y_green   = transform->matrix[0][1];
		chnl->col_regs.y_blue    = transform->matrix[0][2];
		chnl->col_regs.cb_red    = transform->matrix[1][0];
		chnl->col_regs.cb_green  = transform->matrix[1][1];
		chnl->col_regs.cb_blue   = transform->matrix[1][2];
		chnl->col_regs.cr_red    = transform->matrix[2][0];
		chnl->col_regs.cr_green  = transform->matrix[2][1];
		chnl->col_regs.cr_blue   = transform->matrix[2][2];
		chnl->col_regs.off_y     = transform->offset[0];
		chnl->col_regs.off_cb    = transform->offset[1];
		chnl->col_regs.off_cr    = transform->offset[2];
		chnl->col_regs.dirty = true;

		chnl->transform = transform;
	}

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);
}

static void setup_channel_and_overlay_color_conv(struct mcde_chnl_state *chnl,
						struct mcde_ovly_state *ovly)
{
	struct ovly_regs *regs = &ovly->regs;

	if (chnl->port.update_auto_trig &&
			chnl->port.type == MCDE_PORTTYPE_DSI) {
		if (ovly->pix_fmt == MCDE_OVLYPIXFMT_YCbCr422 &&
				regs->col_conv
					!= MCDE_OVL0CR_COLCCTRL_ENABLED_SAT) {

			regs->col_conv = MCDE_OVL0CR_COLCCTRL_ENABLED_SAT;
			mcde_chnl_col_convert_apply(chnl, &chnl->ycbcr_2_rgb);
		} else if (ovly->pix_fmt != MCDE_OVLYPIXFMT_YCbCr422 &&
				regs->col_conv
					!= MCDE_OVL0CR_COLCCTRL_DISABLED) {
			regs->col_conv = MCDE_OVL0CR_COLCCTRL_DISABLED;
		}
	}
}

static void chnl_ovly_pixel_format_apply(struct mcde_chnl_state *chnl,
						struct mcde_ovly_state *ovly)
{
	struct mcde_port *port = &chnl->port;
	struct ovly_regs *regs = &ovly->regs;

	/* Note: YUV -> YUV: blending YUV overlays will not make sense. */
	static struct mcde_col_transform crycb_2_ycbcr = {
		/* Note that in MCDE YUV 422 pixels come as VYU pixels */
		.matrix = {
			{0x0000, 0x0100, 0x0000},
			{0x0000, 0x0000, 0x0100},
			{0x0100, 0x0000, 0x0000},
		},
		.offset = {0, 0, 0},
	};


	if (port->type == MCDE_PORTTYPE_DPI && port->phy.dpi.tv_mode) {
		regs->col_conv = MCDE_OVL0CR_COLCCTRL_ENABLED_NO_SAT;
		if (ovly->pix_fmt != MCDE_OVLYPIXFMT_YCbCr422)
			mcde_chnl_col_convert_apply(chnl, &chnl->rgb_2_ycbcr);
		else
			mcde_chnl_col_convert_apply(chnl, &crycb_2_ycbcr);
	} else {
		if (port->pixel_format != MCDE_PORTPIXFMT_DSI_YCBCR422) {
			setup_channel_and_overlay_color_conv(chnl, ovly);
		} else {
			if (ovly->pix_fmt != MCDE_OVLYPIXFMT_YCbCr422)
				/* DSI: RGB -> YUV */
				mcde_chnl_col_convert_apply(chnl,
							&chnl->rgb_2_ycbcr);
			else
				/* DSI: YUV -> YUV */
				mcde_chnl_col_convert_apply(chnl,
							&crycb_2_ycbcr);
			regs->col_conv = MCDE_OVL0CR_COLCCTRL_ENABLED_NO_SAT;
		}
	}
}

static void update_overlay_registers(struct mcde_ovly_state *ovly,
			struct ovly_regs *regs,
			struct mcde_port *port, enum mcde_fifo fifo,
			s16 stride, bool interlaced,
			enum mcde_display_rotation rotation)
{
	/* TODO: fix clipping for small overlay */
	u8 idx = ovly->idx;
	u32 lmrgn = regs->cropx * regs->bits_per_pixel;
	u32 tmrgn = regs->cropy * stride;
	u32 ppl = regs->ppl;
	u32 lpf = regs->lpf;
	s32 ljinc = stride;
	u32 pixelfetchwtrmrklevel;
	u8  nr_of_bufs = 1;
	u32 sel_mod = MCDE_EXTSRC0CR_SEL_MOD_SOFTWARE_SEL;
	struct mcde_platform_data *pdata = mcde_dev->dev.platform_data;

	if (rotation == MCDE_DISPLAY_ROT_180_CCW) {
		ljinc = -ljinc;
		tmrgn += stride * (regs->lpf - 1) / 8;
	}

	/*
	 * Preferably most of this is done in some apply function instead of for
	 * every update. However lpf has a dependency on update_y.
	 */
	if (interlaced && port->type == MCDE_PORTTYPE_DSI) {
		nr_of_bufs = 2;
		lpf = lpf / 2;
		ljinc *= 2;
	}

	pixelfetchwtrmrklevel = pdata->pixelfetchwtrmrk[idx];
	if (pixelfetchwtrmrklevel == 0) {
		/* Not set: Use default value */
		switch (idx) {
		case 0:
			pixelfetchwtrmrklevel = MCDE_PIXFETCH_WTRMRKLVL_OVL0;
			break;
		case 1:
			pixelfetchwtrmrklevel = MCDE_PIXFETCH_WTRMRKLVL_OVL1;
			break;
		case 2:
			pixelfetchwtrmrklevel = MCDE_PIXFETCH_WTRMRKLVL_OVL2;
			break;
		case 3:
			pixelfetchwtrmrklevel = MCDE_PIXFETCH_WTRMRKLVL_OVL3;
			break;
		case 4:
			pixelfetchwtrmrklevel = MCDE_PIXFETCH_WTRMRKLVL_OVL4;
			break;
		case 5:
			pixelfetchwtrmrklevel = MCDE_PIXFETCH_WTRMRKLVL_OVL5;
			break;
		}
	}
	if (regs->enabled)
		dev_dbg(&mcde_dev->dev, "ovly%d pfwml:%d %dbpp\n", idx,
				pixelfetchwtrmrklevel, regs->bits_per_pixel);

	if (port->update_auto_trig && port->type == MCDE_PORTTYPE_DSI) {
		switch (port->sync_src) {
		case MCDE_SYNCSRC_OFF:
			sel_mod = MCDE_EXTSRC0CR_SEL_MOD_SOFTWARE_SEL;
			break;
		case MCDE_SYNCSRC_TE0:
		case MCDE_SYNCSRC_TE1:
		case MCDE_SYNCSRC_TE_POLLING:
		default:
			sel_mod = MCDE_EXTSRC0CR_SEL_MOD_AUTO_TOGGLE;
			break;
		}
	} else if (port->type == MCDE_PORTTYPE_DPI)
		sel_mod = MCDE_EXTSRC0CR_SEL_MOD_SOFTWARE_SEL;

	mcde_wreg(MCDE_EXTSRC0CONF + idx * MCDE_EXTSRC0CONF_GROUPOFFSET,
		MCDE_EXTSRC0CONF_BUF_ID(0) |
		MCDE_EXTSRC0CONF_BUF_NB(nr_of_bufs) |
		MCDE_EXTSRC0CONF_PRI_OVLID(idx) |
		MCDE_EXTSRC0CONF_BPP(regs->bpp) |
		MCDE_EXTSRC0CONF_BGR(regs->bgr) |
		MCDE_EXTSRC0CONF_BEBO(regs->bebo) |
		MCDE_EXTSRC0CONF_BEPO(false));
	mcde_wreg(MCDE_EXTSRC0CR + idx * MCDE_EXTSRC0CR_GROUPOFFSET,
		MCDE_EXTSRC0CR_SEL_MOD(sel_mod) |
		MCDE_EXTSRC0CR_MULTIOVL_CTRL_ENUM(PRIMARY) |
		MCDE_EXTSRC0CR_FS_DIV_DISABLE(false) |
		MCDE_EXTSRC0CR_FORCE_FS_DIV(false));
	if (port->update_auto_trig) {
		mcde_update_ovly_db_registers_not_sbb(idx, regs);
		atomic_set(&ovly->update_sbb, 1);
	} else {
		mcde_update_ovly_db_register_sbb(idx, regs,
			regs->col_conv);
	}
	mcde_wreg(MCDE_OVL0CONF + idx * MCDE_OVL0CONF_GROUPOFFSET,
		MCDE_OVL0CONF_PPL(ppl) |
		MCDE_OVL0CONF_EXTSRC_ID(idx) |
		MCDE_OVL0CONF_LPF(lpf));
	mcde_wreg(MCDE_OVL0CONF2 + idx * MCDE_OVL0CONF2_GROUPOFFSET,
		MCDE_OVL0CONF2_BP(regs->alpha_source) |
		MCDE_OVL0CONF2_ALPHAVALUE(regs->alpha_value) |
		MCDE_OVL0CONF2_OPQ(regs->opq) |
		MCDE_OVL0CONF2_PIXOFF(lmrgn & 63) |
		MCDE_OVL0CONF2_PIXELFETCHERWATERMARKLEVEL(
			pixelfetchwtrmrklevel));
	mcde_wreg(MCDE_OVL0LJINC + idx * MCDE_OVL0LJINC_GROUPOFFSET,
		ljinc);
	mcde_wreg(MCDE_OVL0CROP + idx * MCDE_OVL0CROP_GROUPOFFSET,
		MCDE_OVL0CROP_TMRGN(tmrgn) |
		MCDE_OVL0CROP_LMRGN(lmrgn >> 6));
	regs->dirty = false;

	dev_vdbg(&mcde_dev->dev, "Overlay registers setup, idx=%d\n", idx);
}

static void update_overlay_registers_on_the_fly(u8 idx, struct ovly_regs *regs)
{
	mcde_wreg(MCDE_OVL0COMP + idx * MCDE_OVL0COMP_GROUPOFFSET,
		MCDE_OVL0COMP_XPOS(regs->xpos) |
		MCDE_OVL0COMP_CH_ID(regs->ch_id) |
		MCDE_OVL0COMP_YPOS(regs->ypos) |
		MCDE_OVL0COMP_Z(regs->z));

	mcde_wreg(MCDE_EXTSRC0A0 + idx * MCDE_EXTSRC0A0_GROUPOFFSET,
		regs->baseaddress0);
	mcde_wreg(MCDE_EXTSRC0A1 + idx * MCDE_EXTSRC0A1_GROUPOFFSET,
		regs->baseaddress1);
	regs->dirty_buf = false;
}

static void do_softwaretrig(struct mcde_chnl_state *chnl)
{
	unsigned long flags;

	local_irq_save(flags);

	enable_flow(chnl);
	set_channel_state_atomic(chnl, CHNLSTATE_RUNNING);
	mcde_wreg(MCDE_CHNL0SYNCHSW +
		chnl->id * MCDE_CHNL0SYNCHSW_GROUPOFFSET,
		MCDE_CHNL0SYNCHSW_SW_TRIG(true));
	disable_flow(chnl);
	set_channel_state_atomic(chnl, CHNLSTATE_STOPPING);

	local_irq_restore(flags);

	dev_vdbg(&mcde_dev->dev, "Software TRIG on channel %d\n", chnl->id);
}

static void disable_flow(struct mcde_chnl_state *chnl)
{
	unsigned long flags;

	local_irq_save(flags);

	switch (chnl->id) {
	case MCDE_CHNL_A:
		mcde_wfld(MCDE_CRA0, FLOEN, false);
		break;
	case MCDE_CHNL_B:
		mcde_wfld(MCDE_CRB0, FLOEN, false);
		break;
	case MCDE_CHNL_C0:
		mcde_wfld(MCDE_CRC, C1EN, false);
		break;
	case MCDE_CHNL_C1:
		mcde_wfld(MCDE_CRC, C2EN, false);
		break;
	}

	local_irq_restore(flags);
}

static void stop_channel(struct mcde_chnl_state *chnl)
{
	const struct mcde_port *port = &chnl->port;
	bool dpi_lcd_mode;

	dev_vdbg(&mcde_dev->dev, "%s %d\n", __func__, chnl->state);

	if (!chnl->port.update_auto_trig ||
				chnl->state != CHNLSTATE_RUNNING) {
		set_channel_state_sync(chnl, CHNLSTATE_SUSPEND);
		return;

	}
	if (chnl->port.sync_src == MCDE_SYNCSRC_OFF) {
		disable_flow(chnl);
		set_channel_state_atomic(chnl, CHNLSTATE_STOPPING);
		/*
		 * Needs to manually trigger VCOMP after the channel is
		 * disabled.
		*/
		dpi_lcd_mode = (port->type == MCDE_PORTTYPE_DPI &&
					!chnl->port.phy.dpi.tv_mode);
		if (!dpi_lcd_mode)
			mcde_wreg(MCDE_SISPP, 1 << chnl->id);
	} else {
		set_channel_state_atomic(chnl, CHNLSTATE_STOPPING);
	}
	set_channel_state_sync(chnl, CHNLSTATE_SUSPEND);
	if (port->type == MCDE_PORTTYPE_DSI) {
		dsi_wfld(port->link, DSI_MCTL_MAIN_PHY_CTL, CLK_CONTINUOUS,
			false);
		if (port->sync_src == MCDE_SYNCSRC_TE_POLLING)
			del_timer(&chnl->dsi_te_timer);
	}
}

static void wait_for_flow_disabled(struct mcde_chnl_state *chnl)
{
	int i = 0;

	switch (chnl->id) {
	case MCDE_CHNL_A:
		for (i = 0; i < MCDE_FLOWEN_MAX_TRIAL; i++) {
			if (!mcde_rfld(MCDE_CRA0, FLOEN)) {
				dev_vdbg(&mcde_dev->dev,
					"Flow (A) disable after >= %d ms\n", i);
				break;
			}
			usleep_range(1000, 1500);
		}
		break;
	case MCDE_CHNL_B:
		for (i = 0; i < MCDE_FLOWEN_MAX_TRIAL; i++) {
			if (!mcde_rfld(MCDE_CRB0, FLOEN)) {
				dev_vdbg(&mcde_dev->dev,
				"Flow (B) disable after >= %d ms\n", i);
				break;
			}
			usleep_range(1000, 1500);
		}
		break;
	case MCDE_CHNL_C0:
		for (i = 0; i < MCDE_FLOWEN_MAX_TRIAL; i++) {
			if (!mcde_rfld(MCDE_CRC, C1EN)) {
				dev_vdbg(&mcde_dev->dev,
				"Flow (C1) disable after >= %d ms\n", i);
				break;
			}
			usleep_range(1000, 1500);
		}
		break;
	case MCDE_CHNL_C1:
		for (i = 0; i < MCDE_FLOWEN_MAX_TRIAL; i++) {
			if (!mcde_rfld(MCDE_CRC, C2EN)) {
				dev_vdbg(&mcde_dev->dev,
				"Flow (C2) disable after >= %d ms\n", i);
				break;
			}
			usleep_range(1000, 1500);
		}
		break;
	}
	if (i == MCDE_FLOWEN_MAX_TRIAL)
		dev_err(&mcde_dev->dev, "%s: channel %d timeout\n",
							__func__, chnl->id);
}

static void enable_flow(struct mcde_chnl_state *chnl)
{
	const struct mcde_port *port = &chnl->port;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (port->type == MCDE_PORTTYPE_DSI)
		dsi_wfld(port->link, DSI_MCTL_MAIN_PHY_CTL, CLK_CONTINUOUS,
				port->phy.dsi.clk_cont);

	/*
	 * When ROTEN is set, the FLOEN bit will also be set but
	 * the flow has to be started anyway.
	 */
	switch (chnl->id) {
	case MCDE_CHNL_A:
		WARN_ON_ONCE(mcde_rfld(MCDE_CRA0, FLOEN));
		mcde_wfld(MCDE_CRA0, ROTEN, chnl->regs.roten);
		mcde_wfld(MCDE_CRA0, FLOEN, true);
		break;
	case MCDE_CHNL_B:
		WARN_ON_ONCE(mcde_rfld(MCDE_CRB0, FLOEN));
		mcde_wfld(MCDE_CRB0, ROTEN, chnl->regs.roten);
		mcde_wfld(MCDE_CRB0, FLOEN, true);
		break;
	case MCDE_CHNL_C0:
		WARN_ON_ONCE(mcde_rfld(MCDE_CRC, C1EN));
		mcde_wfld(MCDE_CRC, C1EN, true);
		break;
	case MCDE_CHNL_C1:
		WARN_ON_ONCE(mcde_rfld(MCDE_CRC, C2EN));
		mcde_wfld(MCDE_CRC, C2EN, true);
		break;
	}
}

static void work_sleep_function(struct work_struct *ptr)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);
	if (mcde_trylock(__func__, __LINE__)) {
		if (mcde_dynamic_power_management)
			disable_mcde_hw(false, false);
		mcde_unlock(__func__, __LINE__);
	}
}

/* TODO get from register */
#define MCDE_CLK_FREQ_MHZ 160
static u32 get_pkt_div(u32 disp_ppl,
		struct mcde_port *port,
		enum mcde_fifo fifo)
{
	/*
	 * The lines can be split in several packets only on DSI CMD mode.
	 * In DSI VIDEO mode, 1 line = 1 packet.
	 * DPI is like DSI VIDEO (watermark = 1 line).
	 * DPI waits for fifo ready only for the first line of the first frame.
	 * If line is wider than fifo size, one can set watermark
	 * at fifo size, or set it to line size as watermark will be
	 * saturated at fifo size inside MCDE.
	 */
	switch (port->type) {
	case MCDE_PORTTYPE_DSI:
		if (port->mode == MCDE_PORTMODE_CMD)
			/* Equivalent of ceil(disp_ppl/fifo_size) */
			return (disp_ppl - 1) / get_output_fifo_size(fifo) + 1;
		else
			return 1;
		break;
	case MCDE_PORTTYPE_DPI:
		return 1;
		break;
	default:
		break;
	}
	return 1;
}

static void update_vid_horizontal_blanking(struct mcde_port *port,
		struct mcde_video_mode *vmode, bool sync_is_pulse, u8 bpp)
{
	int hfp, hbp, hsa;
	u8 link = port->link;

	/*
	 * vmode->hfp, vmode->hbp and vmode->hsw are given in pixels
	 * and must be re-calculated into bytes
	 *
	 * 6 + 2 is HFP header + checksum
	 */
	hfp = vmode->hfp * bpp - 6 - 2;
	if (sync_is_pulse) {
		/*
		 * 6 is HBP header + checksum
		 * 4 is RGB header + checksum
		 */
		hbp = vmode->hbp * bpp - 4 - 6;
		/*
		 * 6 is HBP header + checksum
		 * 4 is HSW packet bytes
		 * 4 is RGB header + checksum
		 */
		hsa = vmode->hsw * bpp - 4 - 4 - 6;
	} else {
		/*
		 * 6 is HBP header + checksum
		 * 4 is HSW packet bytes
		 * 4 is RGB header + checksum
		 */
		hbp = (vmode->hbp + vmode->hsw) * bpp - 4 - 4 - 6;
		/* HSA is not considered in this mode and set to 0 */
		hsa = 0;
	}
	if (hfp < 0) {
		hfp = 0;
		dev_warn(&mcde_dev->dev,
			"%s: negative calc for hfp, set to 0\n", __func__);
	}
	if (hbp < 0) {
		hbp = 0;
		dev_warn(&mcde_dev->dev,
			"%s: negative calc for hbp, set to 0\n", __func__);
	}
	if (hsa < 0) {
		hsa = 0;
		dev_warn(&mcde_dev->dev,
			"%s: negative calc for hsa, set to 0\n", __func__);
	}

	dsi_wfld(link, DSI_VID_HSIZE1, HFP_LENGTH, hfp);
	dsi_wfld(link, DSI_VID_HSIZE1, HBP_LENGTH, hbp);
	dsi_wfld(link, DSI_VID_HSIZE1, HSA_LENGTH, hsa);
}

static void update_vid_frame_parameters(struct mcde_port *port,
				struct mcde_video_mode *vmode, u8 bpp)
{
	u8 link = port->link;
	bool burst_mode, sync_is_pulse, tvg_enable;
	u32 hs_byte_clk, pck_len, blkline_pck, line_duration;
	u32 blkeol_pck, blkeol_duration;
	u8 pixel_mode;
	u8 rgb_header;

	get_vid_operating_mode(port, &burst_mode, &sync_is_pulse, &tvg_enable);

	dsi_wfld(link, DSI_VID_VSIZE, VFP_LENGTH, vmode->vfp);
	dsi_wfld(link, DSI_VID_VSIZE, VBP_LENGTH, vmode->vbp);
	dsi_wfld(link, DSI_VID_VSIZE, VSA_LENGTH, vmode->vsw);
	update_vid_horizontal_blanking(port, vmode, sync_is_pulse, bpp);

	dsi_wfld(link, DSI_VID_VSIZE, VACT_LENGTH, vmode->yres);
	dsi_wfld(link, DSI_VID_HSIZE2, RGB_SIZE, vmode->xres * bpp);

	/*
	 * The rgb_header identifies the pixel stream format,
	 * as described in the MIPI DSI Specification:
	 *
	 * 0x0E: Packed pixel stream, 16-bit RGB, 565 format
	 * 0x1E: Packed pixel stream, 18-bit RGB, 666 format
	 * 0x2E: Loosely Packed pixel stream, 18-bit RGB, 666 format
	 * 0x3E: Packed pixel stream, 24-bit RGB, 888 format
	 */
	switch (port->pixel_format) {
	case MCDE_PORTPIXFMT_DSI_16BPP:
		pixel_mode = 0;
		rgb_header = 0x0E;
		break;
	case MCDE_PORTPIXFMT_DSI_18BPP:
		pixel_mode = 2;
		rgb_header = 0x2E;
		break;
	case MCDE_PORTPIXFMT_DSI_18BPP_PACKED:
		pixel_mode = 1;
		rgb_header = 0x1E;
		break;
	case MCDE_PORTPIXFMT_DSI_24BPP:
		pixel_mode = 3;
		rgb_header = 0x3E;
		break;
	default:
		pixel_mode = 3;
		rgb_header = 0x3E;
		dev_warn(&mcde_dev->dev,
			"%s: invalid pixel format %d\n",
			__func__, port->pixel_format);
		break;
	}

	dsi_wfld(link, DSI_VID_MAIN_CTL, VID_PIXEL_MODE, pixel_mode);
	dsi_wfld(link, DSI_VID_MAIN_CTL, HEADER, rgb_header);

	if (tvg_enable) {
		/*
		 * with these settings, expect to see 64 pixels wide
		 * red and green vertical stripes on the screen when
		 * tvg_enable = 1
		 */
		dsi_wfld(link, DSI_MCTL_MAIN_DATA_CTL, TVG_SEL, 1);

		dsi_wfld(link, DSI_TVG_CTL, TVG_STRIPE_SIZE, 6);
		dsi_wfld(link, DSI_TVG_CTL, TVG_MODE, 2);
		dsi_wfld(link, DSI_TVG_CTL, TVG_STOPMODE, 2);
		dsi_wfld(link, DSI_TVG_CTL, TVG_RUN, 1);

		dsi_wfld(link, DSI_TVG_IMG_SIZE, TVG_NBLINE, vmode->yres);
		dsi_wfld(link, DSI_TVG_IMG_SIZE, TVG_LINE_SIZE,
							vmode->xres * bpp);

		dsi_wfld(link, DSI_TVG_COLOR1, COL1_BLUE, 0);
		dsi_wfld(link, DSI_TVG_COLOR1, COL1_GREEN, 0);
		dsi_wfld(link, DSI_TVG_COLOR1, COL1_RED, 0xFF);

		dsi_wfld(link, DSI_TVG_COLOR2, COL2_BLUE, 0);
		dsi_wfld(link, DSI_TVG_COLOR2, COL2_GREEN, 0xFF);
		dsi_wfld(link, DSI_TVG_COLOR2, COL2_RED, 0);
	}

	/*
	 * vid->pixclock is the time between two pixels (in picoseconds)
	 *
	 * hs_byte_clk is the amount of transferred bytes per lane and
	 * second (in MHz)
	 */
	hs_byte_clk = 1000000 / vmode->pixclock / 8;
	pck_len = 1000000 * hs_byte_clk / port->refresh_rate /
			(vmode->vsw + vmode->vbp + vmode->yres + vmode->vfp) *
						port->phy.dsi.num_data_lanes;

	/*
	 * 6 is header + checksum, header = 4 bytes, checksum = 2 bytes
	 * 4 is short packet for vsync/hsync
	 */
	if (sync_is_pulse)
		blkline_pck = pck_len - vmode->hsw - 6;
	else
		blkline_pck = pck_len - 4 - 6;

	line_duration = (blkline_pck + 6) / port->phy.dsi.num_data_lanes;
	blkeol_pck = pck_len -
		(vmode->hsw + vmode->hbp + vmode->xres + vmode->hfp) * bpp - 6;
	blkeol_duration = (blkeol_pck + 6) / port->phy.dsi.num_data_lanes;

	if (sync_is_pulse)
		dsi_wfld(link, DSI_VID_BLKSIZE2, BLKLINE_PULSE_PCK,
								blkline_pck);
	else
		dsi_wfld(link, DSI_VID_BLKSIZE1, BLKLINE_EVENT_PCK,
								blkline_pck);
	dsi_wfld(link, DSI_VID_DPHY_TIME, REG_LINE_DURATION, line_duration);
	if (burst_mode) {
		dsi_wfld(link, DSI_VID_BLKSIZE1, BLKEOL_PCK, blkeol_pck);
		dsi_wfld(link, DSI_VID_PCK_TIME, BLKEOL_DURATION,
							blkeol_duration);
		dsi_wfld(link, DSI_VID_VCA_SETTING1, MAX_BURST_LIMIT,
							blkeol_pck - 6);
		dsi_wfld(link, DSI_VID_VCA_SETTING2, EXACT_BURST_LIMIT,
								blkeol_pck);
	}
	if (sync_is_pulse)
		dsi_wfld(link, DSI_VID_VCA_SETTING2, MAX_LINE_LIMIT,
							blkline_pck - 6);
}

static void set_vsync_method(u8 idx, struct mcde_port *port)
{
	u32 out_synch_src = MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_FORMATTER;
	u32 src_synch = MCDE_CHNL0SYNCHMOD_SRC_SYNCH_HARDWARE;

	if (port->type == MCDE_PORTTYPE_DSI) {
		switch (port->frame_trig) {
		case MCDE_TRIG_HW:
			src_synch = MCDE_CHNL0SYNCHMOD_SRC_SYNCH_HARDWARE;
			break;
		case MCDE_TRIG_SW:
			src_synch = MCDE_CHNL0SYNCHMOD_SRC_SYNCH_SOFTWARE;
			break;
		default:
			src_synch = MCDE_CHNL0SYNCHMOD_SRC_SYNCH_HARDWARE;
			break;
		}

		switch (port->sync_src) {
		case MCDE_SYNCSRC_OFF:
			out_synch_src =
				MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_FORMATTER;
			break;
		case MCDE_SYNCSRC_TE0:
			out_synch_src = MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_TE0;
			if (src_synch ==
				MCDE_CHNL0SYNCHMOD_SRC_SYNCH_SOFTWARE) {
				dev_dbg(&mcde_dev->dev, "%s: badly configured "
						"frame sync, TE0 defaulting "
						"to hw frame trig\n", __func__);
				src_synch =
					MCDE_CHNL0SYNCHMOD_SRC_SYNCH_HARDWARE;
			}
			break;
		case MCDE_SYNCSRC_TE1:
			out_synch_src = MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_TE1;
			if (src_synch ==
				MCDE_CHNL0SYNCHMOD_SRC_SYNCH_SOFTWARE) {
				dev_dbg(&mcde_dev->dev, "%s: badly configured "
						"frame sync, TE1 defaulting "
						"to hw frame trig\n", __func__);
				src_synch =
					MCDE_CHNL0SYNCHMOD_SRC_SYNCH_HARDWARE;
			}
			break;
		case MCDE_SYNCSRC_BTA:
			out_synch_src =
				MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_FORMATTER;
			break;
		case MCDE_SYNCSRC_TE_POLLING:
			out_synch_src =
				MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_FORMATTER;
			if (src_synch ==
				MCDE_CHNL0SYNCHMOD_SRC_SYNCH_SOFTWARE) {
				dev_dbg(&mcde_dev->dev, "%s: badly configured "
					"frame sync, TE_POLLING defaulting "
						"to hw frame trig\n", __func__);
				src_synch =
					MCDE_CHNL0SYNCHMOD_SRC_SYNCH_HARDWARE;
			}
			break;
		default:
			out_synch_src =
				MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_FORMATTER;
			src_synch = MCDE_CHNL0SYNCHMOD_SRC_SYNCH_HARDWARE;
			dev_dbg(&mcde_dev->dev, "%s: no sync src selected, "
						"defaulting to DSI BTA with "
						"hw frame trig\n", __func__);
			break;
		}
	} else if (port->type == MCDE_PORTTYPE_DPI) {
		out_synch_src = MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC_FORMATTER;
		src_synch = port->update_auto_trig ?
					MCDE_CHNL0SYNCHMOD_SRC_SYNCH_HARDWARE :
					MCDE_CHNL0SYNCHMOD_SRC_SYNCH_SOFTWARE;
	}

	mcde_wreg(MCDE_CHNL0SYNCHMOD +
		idx * MCDE_CHNL0SYNCHMOD_GROUPOFFSET,
		MCDE_CHNL0SYNCHMOD_SRC_SYNCH(src_synch) |
		MCDE_CHNL0SYNCHMOD_OUT_SYNCH_SRC(out_synch_src));
}

void update_channel_registers(enum mcde_chnl chnl_id, struct chnl_regs *regs,
				struct mcde_port *port, enum mcde_fifo fifo,
				struct mcde_video_mode *video_mode)
{
	u8 idx = chnl_id;
	u32 fifo_wtrmrk = 0;
	u8 red;
	u8 green;
	u8 blue;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	/*
	 * Select appropriate fifo watermark.
	 * Watermark will be saturated at fifo size inside MCDE.
	 */
	fifo_wtrmrk = video_mode->xres /
		get_pkt_div(video_mode->xres, port, fifo);

	dev_vdbg(&mcde_dev->dev, "%s fifo_watermark=%d for chnl_id=%d\n",
		__func__, fifo_wtrmrk, chnl_id);

	switch (chnl_id) {
	case MCDE_CHNL_A:
		mcde_wfld(MCDE_CTRLA, FIFOWTRMRK, fifo_wtrmrk);
		break;
	case MCDE_CHNL_B:
		mcde_wfld(MCDE_CTRLB, FIFOWTRMRK, fifo_wtrmrk);
		break;
	case MCDE_CHNL_C0:
		mcde_wfld(MCDE_CTRLC0, FIFOWTRMRK, fifo_wtrmrk);
		break;
	case MCDE_CHNL_C1:
		mcde_wfld(MCDE_CTRLC1, FIFOWTRMRK, fifo_wtrmrk);
		break;
	default:
		break;
	}

	set_vsync_method(idx, port);

	mcde_wreg(MCDE_CHNL0CONF + idx * MCDE_CHNL0CONF_GROUPOFFSET,
		MCDE_CHNL0CONF_PPL(regs->ppl-1) |
		MCDE_CHNL0CONF_LPF(regs->lpf-1));
	mcde_wreg(MCDE_CHNL0STAT + idx * MCDE_CHNL0STAT_GROUPOFFSET,
		MCDE_CHNL0STAT_CHNLBLBCKGND_EN(true) |
		MCDE_CHNL0STAT_CHNLRD(true));
	if (regs->background_yuv) {
		red = 0x80;
		green = 0x10;
		blue = 0x80;
	} else {
		red = 0x00;
		green = 0x00;
		blue = 0x00;
	}
	mcde_wreg(MCDE_CHNL0BCKGNDCOL + idx * MCDE_CHNL0BCKGNDCOL_GROUPOFFSET,
		MCDE_CHNL0BCKGNDCOL_B(blue) |
		MCDE_CHNL0BCKGNDCOL_G(green) |
		MCDE_CHNL0BCKGNDCOL_R(red));

	if (chnl_id == MCDE_CHNL_A || chnl_id == MCDE_CHNL_B) {
		u32 mcde_crx1;
		u32 mcde_pal0x;
		u32 mcde_pal1x;
		if (chnl_id == MCDE_CHNL_A) {
			mcde_crx1 = MCDE_CRA1;
			mcde_pal0x = MCDE_PAL0A;
			mcde_pal1x = MCDE_PAL1A;
			mcde_wfld(MCDE_CRA0, PALEN, regs->palette_enable);
			mcde_wfld(MCDE_CRA0, OLEDEN, regs->oled_enable);
		} else {
			mcde_crx1 = MCDE_CRB1;
			mcde_pal0x = MCDE_PAL0B;
			mcde_pal1x = MCDE_PAL1B;
			mcde_wfld(MCDE_CRB0, PALEN, regs->palette_enable);
			mcde_wfld(MCDE_CRB0, OLEDEN, regs->oled_enable);
		}
		mcde_wreg(mcde_crx1,
			MCDE_CRA1_PCD(regs->pcd) |
			MCDE_CRA1_CLKSEL(regs->clksel) |
			MCDE_CRA1_CDWIN(regs->cdwin) |
			MCDE_CRA1_OUTBPP(bpp2outbpp(regs->bpp)) |
			MCDE_CRA1_BCD(regs->bcd) |
			MCDE_CRA1_CLKTYPE(regs->internal_clk));
		if (regs->palette_enable) {
			int i;
			for (i = 0; i < 256; i++) {
				mcde_wreg(mcde_pal0x,
					MCDE_PAL0A_GREEN(regs->map_g(i)) |
					MCDE_PAL0A_BLUE(regs->map_b(i)));
				mcde_wreg(mcde_pal1x,
					MCDE_PAL1A_RED(regs->map_r(i)));
			}
		}
	}

	/* Formatter */
	if (port->type == MCDE_PORTTYPE_DSI) {
		u8 fidx;
		u32 temp, packet;
		/* pkt_div is used to avoid underflow in output fifo for
		 * large packets */
		u32 pkt_div = 1;
		u32 dsi_delay0 = 0;
		u32 screen_ppl, screen_lpf;

		fidx = get_dsi_formatter_id(port);

		screen_ppl = video_mode->xres;
		screen_lpf = video_mode->yres;

		pkt_div = get_pkt_div(screen_ppl, port, fifo);

		if (video_mode->interlaced)
			screen_lpf /= 2;

		/* pkt_delay_progressive = pixelclock * htot /
		 * (1E12 / 160E6) / pkt_div */
		dsi_delay0 = (video_mode->pixclock) *
			(video_mode->xres + video_mode->hbp +
				video_mode->hfp) /
			(100000000 / ((mcde_clk_rate / 10000))) / pkt_div;

		if ((screen_ppl == SCREEN_PPL_CEA2) &&
				(screen_lpf == SCREEN_LPF_CEA2))
			dsi_delay0 += DSI_DELAY0_CEA2_ADD;

		temp = mcde_rreg(MCDE_DSIVID0CONF0 +
			fidx * MCDE_DSIVID0CONF0_GROUPOFFSET);
		mcde_wreg(MCDE_DSIVID0CONF0 +
			fidx * MCDE_DSIVID0CONF0_GROUPOFFSET,
			(temp & ~MCDE_DSIVID0CONF0_PACKING_MASK) |
			MCDE_DSIVID0CONF0_PACKING(regs->dsipacking));
		/* no extra command byte in video mode */
		if (port->mode == MCDE_PORTMODE_CMD)
			packet = ((screen_ppl / pkt_div * regs->bpp) >> 3) + 1;
		else
			packet = ((screen_ppl / pkt_div * regs->bpp) >> 3);
		mcde_wreg(MCDE_DSIVID0FRAME +
			fidx * MCDE_DSIVID0FRAME_GROUPOFFSET,
			MCDE_DSIVID0FRAME_FRAME(packet * pkt_div * screen_lpf));
		mcde_wreg(MCDE_DSIVID0PKT + fidx * MCDE_DSIVID0PKT_GROUPOFFSET,
			MCDE_DSIVID0PKT_PACKET(packet));
		mcde_wreg(MCDE_DSIVID0SYNC +
			fidx * MCDE_DSIVID0SYNC_GROUPOFFSET,
			MCDE_DSIVID0SYNC_SW(0) |
			MCDE_DSIVID0SYNC_DMA(0));
		mcde_wreg(MCDE_DSIVID0CMDW +
			fidx * MCDE_DSIVID0CMDW_GROUPOFFSET,
			MCDE_DSIVID0CMDW_CMDW_START(DCS_CMD_WRITE_START) |
			MCDE_DSIVID0CMDW_CMDW_CONTINUE(DCS_CMD_WRITE_CONTINUE));
		mcde_wreg(MCDE_DSIVID0DELAY0 +
			fidx * MCDE_DSIVID0DELAY0_GROUPOFFSET,
			MCDE_DSIVID0DELAY0_INTPKTDEL(dsi_delay0));
		mcde_wreg(MCDE_DSIVID0DELAY1 +
			fidx * MCDE_DSIVID0DELAY1_GROUPOFFSET,
			MCDE_DSIVID0DELAY1_TEREQDEL(0) |
			MCDE_DSIVID0DELAY1_FRAMESTARTDEL(0));
		/* Setup VSYNC capture */
		if (port->sync_src == MCDE_SYNCSRC_TE0) {
			mcde_wreg(MCDE_VSCRC0,
				MCDE_VSCRC0_VSDBL(0) |
				MCDE_VSCRC0_VSSEL_ENUM(VSYNC0) |
				MCDE_VSCRC0_VSPOL(port->vsync_polarity) |
				MCDE_VSCRC0_VSPDIV(port->vsync_clock_div) |
				MCDE_VSCRC0_VSPMAX(port->vsync_max_duration) |
				MCDE_VSCRC0_VSPMIN(port->vsync_min_duration));
		} else if (port->sync_src == MCDE_SYNCSRC_TE1) {
			mcde_wreg(MCDE_VSCRC1,
				MCDE_VSCRC1_VSDBL(0) |
				MCDE_VSCRC1_VSSEL_ENUM(VSYNC1) |
				MCDE_VSCRC1_VSPOL(port->vsync_polarity) |
				MCDE_VSCRC1_VSPDIV(port->vsync_clock_div) |
				MCDE_VSCRC1_VSPMAX(port->vsync_max_duration) |
				MCDE_VSCRC1_VSPMIN(port->vsync_min_duration));
		}

		if (port->mode == MCDE_PORTMODE_VID)
			update_vid_frame_parameters(port, video_mode,
								regs->bpp / 8);
	} else if (port->type == MCDE_PORTTYPE_DPI &&
						!port->phy.dpi.tv_mode) {
		/* DPI LCD Mode */
		if (chnl_id == MCDE_CHNL_A) {
			mcde_wreg(MCDE_SYNCHCONFA,
				MCDE_SYNCHCONFA_HWREQVEVENT_ENUM(
							ACTIVE_VIDEO) |
				MCDE_SYNCHCONFA_HWREQVCNT(
							video_mode->yres - 1) |
				MCDE_SYNCHCONFA_SWINTVEVENT_ENUM(
							ACTIVE_VIDEO) |
				MCDE_SYNCHCONFA_SWINTVCNT(
							video_mode->yres - 1));
		} else if (chnl_id == MCDE_CHNL_B) {
			mcde_wreg(MCDE_SYNCHCONFB,
				MCDE_SYNCHCONFB_HWREQVEVENT_ENUM(
							ACTIVE_VIDEO) |
				MCDE_SYNCHCONFB_HWREQVCNT(
							video_mode->yres - 1) |
				MCDE_SYNCHCONFB_SWINTVEVENT_ENUM(
							ACTIVE_VIDEO) |
				MCDE_SYNCHCONFB_SWINTVCNT(
							video_mode->yres - 1));
		}
	}

	if (regs->roten) {
		u32 stripwidth;
		u32 stripwidth_val;

		/* calc strip width, 32 bits used internally */
		stripwidth = regs->rotbufsize / (video_mode->xres * 4);
		if (stripwidth >= 32)
			stripwidth_val = MCDE_ROTACONF_STRIP_WIDTH_32PIX;
		else if (stripwidth >= 16)
			stripwidth_val = MCDE_ROTACONF_STRIP_WIDTH_16PIX;
		else if (stripwidth >= 8)
			stripwidth_val = MCDE_ROTACONF_STRIP_WIDTH_8PIX;
		else if (stripwidth >= 4)
			stripwidth_val = MCDE_ROTACONF_STRIP_WIDTH_4PIX;
		else
			stripwidth_val = MCDE_ROTACONF_STRIP_WIDTH_2PIX;
		dev_vdbg(&mcde_dev->dev, "%s stripwidth=%d\n", __func__,
						1 << (stripwidth_val + 1));
		mcde_wreg(MCDE_ROTADD0A + chnl_id * MCDE_ROTADD0A_GROUPOFFSET,
			regs->rotbuf1);
		mcde_wreg(MCDE_ROTADD1A + chnl_id * MCDE_ROTADD1A_GROUPOFFSET,
			regs->rotbuf2);
		mcde_wreg(MCDE_ROTACONF + chnl_id * MCDE_ROTACONF_GROUPOFFSET,
			MCDE_ROTACONF_ROTBURSTSIZE_ENUM(HW_8W) |
			MCDE_ROTACONF_ROTDIR(regs->rotdir) |
			MCDE_ROTACONF_STRIP_WIDTH(stripwidth_val) |
			MCDE_ROTACONF_RD_MAXOUT_ENUM(4_REQ) |
			MCDE_ROTACONF_WR_MAXOUT_ENUM(8_REQ));
	}

	/* Blending */
	if (chnl_id == MCDE_CHNL_A) {
		mcde_wfld(MCDE_CRA0, BLENDEN, regs->blend_en);
		mcde_wfld(MCDE_CRA0, BLENDCTRL, regs->blend_ctrl);
		mcde_wfld(MCDE_CRA0, ALPHABLEND, regs->alpha_blend);
	} else if (chnl_id == MCDE_CHNL_B) {
		mcde_wfld(MCDE_CRB0, BLENDEN, regs->blend_en);
		mcde_wfld(MCDE_CRB0, BLENDCTRL, regs->blend_ctrl);
		mcde_wfld(MCDE_CRB0, ALPHABLEND, regs->alpha_blend);
	}

	dev_vdbg(&mcde_dev->dev, "Channel registers setup, chnl=%d\n", chnl_id);
	regs->dirty = false;
}

static int enable_mcde_hw(void)
{
	int ret;
	int i;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	cancel_delayed_work(&hw_timeout_work);
	schedule_delayed_work(&hw_timeout_work,
					msecs_to_jiffies(MCDE_SLEEP_WATCHDOG));

	for (i = 0; i < num_channels; i++) {
		struct mcde_chnl_state *chnl = &channels[i];
		if (chnl->state == CHNLSTATE_SUSPEND) {
			/* Mark all registers as dirty */
			set_channel_state_atomic(chnl, CHNLSTATE_IDLE);
			chnl->ovly0->regs.dirty = true;
			chnl->ovly0->regs.dirty_buf = true;
			if (chnl->ovly1) {
				chnl->ovly1->regs.dirty = true;
				chnl->ovly1->regs.dirty_buf = true;
			}
			chnl->regs.dirty = true;
			chnl->col_regs.dirty = true;
			chnl->tv_regs.dirty = true;

			if (!mcde_is_enabled)
				chnl->first_frame_vsync_fix = true;

			chnl->oled_regs.dirty = true;

			atomic_set(&chnl->vcmp_cnt, 0);
			atomic_set(&chnl->vsync_cnt, 0);
			chnl->vsync_cnt_wait = 0;
			chnl->vcmp_cnt_wait = 0;
		}
	}

	if (mcde_is_enabled) {
		dev_vdbg(&mcde_dev->dev, "%s - already enabled\n", __func__);
		return 0;
	}

	enable_clocks_and_power(mcde_dev);

	ret = request_irq(mcde_irq, mcde_irq_handler, 0, "mcde",
							&mcde_dev->dev);
	if (ret) {
		dev_dbg(&mcde_dev->dev, "Failed to request irq (irq=%d)\n",
								mcde_irq);
		cancel_delayed_work(&hw_timeout_work);
		return -EINVAL;
	}

	update_mcde_registers();

	dev_vdbg(&mcde_dev->dev, "%s - enable done\n", __func__);

	mcde_is_enabled = true;
	return 0;
}

/* DSI */
static int mcde_dsi_direct_cmd_write(struct mcde_chnl_state *chnl,
			bool dcs, u8 cmd, u8 *data, int len)
{
	int i, ret = 0;
	u32 wrdat[4] = { 0, 0, 0, 0 };
	u32 settings;
	u8 link = chnl->port.link;
	u8 virt_id = chnl->port.phy.dsi.virt_id;
	u32 counter = DSI_WRITE_CMD_TIMEOUT;

	if (len > MCDE_MAX_DSI_DIRECT_CMD_WRITE ||
			chnl->port.type != MCDE_PORTTYPE_DSI)
		return -EINVAL;

	mcde_lock(__func__, __LINE__);

	_mcde_chnl_enable(chnl);
	if (enable_mcde_hw()) {
		mcde_unlock(__func__, __LINE__);
		return -EINVAL;
	}
	if (!chnl->formatter_updated)
		(void)update_channel_static_registers(chnl);

	set_channel_state_sync(chnl, CHNLSTATE_DSI_WRITE);

	if (dcs) {
		wrdat[0] = cmd;
		for (i = 1; i <= len; i++)
			wrdat[i>>2] |= ((u32)data[i-1] << ((i & 3) * 8));
	} else {
		/* no explicit cmd byte for generic_write, only params */
		for (i = 0; i < len; i++)
			wrdat[i>>2] |= ((u32)data[i] << ((i & 3) * 8));
	}

	settings = DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_NAT_ENUM(WRITE) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LONGNOTSHORT(len > 1) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_ID(virt_id) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_SIZE(len+1) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LP_EN(true);
	if (dcs) {
		if (len == 0)
			settings |= DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(
				DCS_SHORT_WRITE_0);
		else if (len == 1)
			settings |= DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(
				DCS_SHORT_WRITE_1);
		else
			settings |= DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(
				DCS_LONG_WRITE);
	} else {
		if (len == 0)
			settings |= DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(
				GENERIC_SHORT_WRITE_0);
		else if (len == 1)
			settings |= DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(
				GENERIC_SHORT_WRITE_1);
		else if (len == 2)
			settings |= DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(
				GENERIC_SHORT_WRITE_2);
		else
			settings |= DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(
				GENERIC_LONG_WRITE);
	}

	dsi_wreg(link, DSI_DIRECT_CMD_MAIN_SETTINGS, settings);
	dsi_wreg(link, DSI_DIRECT_CMD_WRDAT0, wrdat[0]);
	if (len >  3)
		dsi_wreg(link, DSI_DIRECT_CMD_WRDAT1, wrdat[1]);
	if (len >  7)
		dsi_wreg(link, DSI_DIRECT_CMD_WRDAT2, wrdat[2]);
	if (len > 11)
		dsi_wreg(link, DSI_DIRECT_CMD_WRDAT3, wrdat[3]);
	dsi_wreg(link, DSI_DIRECT_CMD_STS_CLR, ~0);
	dsi_wreg(link, DSI_CMD_MODE_STS_CLR, ~0);
	dsi_wreg(link, DSI_DIRECT_CMD_SEND, true);

	/* loop will normally run zero or one time until WRITE_COMPLETED */
	while (!dsi_rfld(link, DSI_DIRECT_CMD_STS, WRITE_COMPLETED)
			&& --counter)
		cpu_relax();

	if (!counter) {
		dev_err(&mcde_dev->dev,
			"%s: DSI write cmd 0x%x timeout on DSI link %u!\n",
			__func__, cmd, link);
		ret = -ETIME;
	} else {
		/* inform if >100 loops before command completion */
		if (counter < (DSI_WRITE_CMD_TIMEOUT-DSI_WRITE_CMD_TIMEOUT/10))
			dev_vdbg(&mcde_dev->dev,
				"%s: %u loops for DSI command %x completion\n",
				__func__, (DSI_WRITE_CMD_TIMEOUT - counter),
				cmd);

		dev_vdbg(&mcde_dev->dev, "DSI Write ok %x error %x\n",
			dsi_rreg(link, DSI_DIRECT_CMD_STS_FLAG),
			dsi_rreg(link, DSI_CMD_MODE_STS_FLAG));
	}

	set_channel_state_atomic(chnl, CHNLSTATE_IDLE);

	mcde_unlock(__func__, __LINE__);

	return ret;
}

int mcde_dsi_generic_write(struct mcde_chnl_state *chnl, u8* para, int len)
{
	return mcde_dsi_direct_cmd_write(chnl, false, 0, para, len);
}

int mcde_dsi_dcs_write(struct mcde_chnl_state *chnl, u8 cmd, u8* data, int len)
{
	return mcde_dsi_direct_cmd_write(chnl, true, cmd, data, len);
}

int mcde_dsi_dcs_read(struct mcde_chnl_state *chnl,
			u8 cmd, u32 *data, int *len)
{
	int ret = 0;
	u8 link = chnl->port.link;
	u8 virt_id = chnl->port.phy.dsi.virt_id;
	u32 settings;
	bool ok = false;
	bool error, ack_with_err;
	u8 nbr_of_retries = DSI_READ_NBR_OF_RETRIES;

	if (*len > MCDE_MAX_DCS_READ || chnl->port.type != MCDE_PORTTYPE_DSI)
		return -EINVAL;

	mcde_lock(__func__, __LINE__);

	_mcde_chnl_enable(chnl);
	if (enable_mcde_hw()) {
		mcde_unlock(__func__, __LINE__);
		return -EINVAL;
	}
	if (!chnl->formatter_updated)
		(void)update_channel_static_registers(chnl);

	set_channel_state_sync(chnl, CHNLSTATE_DSI_READ);

	dsi_wfld(link, DSI_MCTL_MAIN_DATA_CTL, BTA_EN, true);
	dsi_wfld(link, DSI_MCTL_MAIN_DATA_CTL, READ_EN, true);
	settings = DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_NAT_ENUM(READ) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LONGNOTSHORT(false) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_ID(virt_id) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_SIZE(1) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LP_EN(true) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(DCS_READ);
	dsi_wreg(link, DSI_DIRECT_CMD_MAIN_SETTINGS, settings);
	dsi_wreg(link, DSI_DIRECT_CMD_WRDAT0, cmd);

	do {
		u8 wait  = DSI_READ_TIMEOUT;
		dsi_wreg(link, DSI_DIRECT_CMD_STS_CLR, ~0);
		dsi_wreg(link, DSI_DIRECT_CMD_RD_STS_CLR, ~0);
		dsi_wreg(link, DSI_DIRECT_CMD_SEND, true);

		while (wait-- && !(error = dsi_rfld(link, DSI_DIRECT_CMD_STS,
					READ_COMPLETED_WITH_ERR)) &&
				!(ok = dsi_rfld(link, DSI_DIRECT_CMD_STS,
							READ_COMPLETED)))
			udelay(DSI_READ_DELAY);

		ack_with_err = dsi_rfld(link, DSI_DIRECT_CMD_STS,
						ACKNOWLEDGE_WITH_ERR_RECEIVED);
		if (ack_with_err)
			dev_warn(&mcde_dev->dev,
					"DCS Acknowledge Error Report %.4X\n",
				dsi_rfld(link, DSI_DIRECT_CMD_STS, ACK_VAL));
	} while (--nbr_of_retries && ack_with_err);

	if (ok) {
		int rdsize;
		u32 rddat;

		rdsize = dsi_rfld(link, DSI_DIRECT_CMD_RD_PROPERTY, RD_SIZE);
		rddat = dsi_rreg(link, DSI_DIRECT_CMD_RDDAT);
		if (rdsize < *len)
			dev_warn(&mcde_dev->dev, "DCS incomplete read %d<%d"
					" (%.8X)\n", rdsize, *len, rddat);
		*len = min(*len, rdsize);
		memcpy(data, &rddat, *len);
	} else {
 		u32 sts;
		sts = dsi_rreg(link, DSI_DIRECT_CMD_STS);
		dev_err(&mcde_dev->dev, "DCS read failed, err=%d, sts=%X\n",
				error, sts);
		if (sts == 1) {
			dev_err(&mcde_dev->dev, "Stop DSI clock lanes\n");
			dsi_wfld(link, DSI_MCTL_MAIN_PHY_CTL, FORCE_STOP_MODE,
									true);
			dsi_wfld(link, DSI_MCTL_MAIN_PHY_CTL,
						CLOCK_FORCE_STOP_MODE, true);
			udelay(20);
			dsi_wfld(link, DSI_MCTL_MAIN_PHY_CTL, FORCE_STOP_MODE,
									false);
			dsi_wfld(link, DSI_MCTL_MAIN_PHY_CTL,
						CLOCK_FORCE_STOP_MODE, false);
		}
		ret = -EIO;
	}

	dsi_wreg(link, DSI_CMD_MODE_STS_CLR, ~0);
	dsi_wreg(link, DSI_DIRECT_CMD_STS_CLR, ~0);

	set_channel_state_atomic(chnl, CHNLSTATE_IDLE);

	mcde_unlock(__func__, __LINE__);

	return ret;
}

/*
 * Set Maximum Return Packet size is a command that specifies the
 * maximum size of the payload transmitted from peripheral back to
 * the host processor.
 *
 * During power-on or reset sequence, the Maximum Return Packet Size
 * is set to a default value of one. In order to be able to use
 * mcde_dsi_dcs_read for reading more than 1 byte at a time, this
 * parameter should be set by the host processor to the desired value
 * in the initialization routine before commencing normal operation.
 */
int mcde_dsi_set_max_pkt_size(struct mcde_chnl_state *chnl)
{
	u32 settings;
	u8 link = chnl->port.link;
	u8 virt_id = chnl->port.phy.dsi.virt_id;

	if (chnl->port.type != MCDE_PORTTYPE_DSI)
		return -EINVAL;

	mcde_lock(__func__, __LINE__);

	if (enable_mcde_hw()) {
		mcde_unlock(__func__, __LINE__);
		return -EIO;
	}
	if (!chnl->formatter_updated)
		(void)update_channel_static_registers(chnl);

	set_channel_state_sync(chnl, CHNLSTATE_DSI_WRITE);

	/*
	 * Set Maximum Return Packet Size is a two-byte command packet
	 * that specifies the maximum size of the payload as u16 value.
	 * The order of bytes is: MaxSize LSB, MaxSize MSB
	 */
	settings = DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_NAT_ENUM(WRITE) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LONGNOTSHORT(false) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_ID(virt_id) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_SIZE(2) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LP_EN(true) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(
							SET_MAX_PKT_SIZE);
	dsi_wreg(link, DSI_DIRECT_CMD_MAIN_SETTINGS, settings);
	dsi_wreg(link, DSI_DIRECT_CMD_WRDAT0, MCDE_MAX_DCS_READ);
	dsi_wreg(link, DSI_DIRECT_CMD_SEND, true);

	set_channel_state_atomic(chnl, CHNLSTATE_IDLE);

	mcde_unlock(__func__, __LINE__);

	return 0;
}

static void dsi_te_poll_req(struct mcde_chnl_state *chnl)
{
	u8 lnk = chnl->port.link;
	const struct mcde_port *port = &chnl->port;

	dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, REG_TE_EN, false);
	if (port->ifc == 0)
		dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, IF1_TE_EN, true);
	if (port->ifc == 1)
		dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, IF2_TE_EN, true);
	dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, BTA_EN, true);
	dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, READ_EN, true);
	dsi_wfld(lnk, DSI_CMD_MODE_CTL, TE_TIMEOUT, 0x3FF);
	dsi_wfld(lnk, DSI_MCTL_MAIN_DATA_CTL, TE_POLLING_EN, true);
}

static void dsi_te_poll_set_timer(struct mcde_chnl_state *chnl,
		unsigned int timeout)
{
	mod_timer(&chnl->dsi_te_timer,
			jiffies +
			msecs_to_jiffies(timeout));
}

static void dsi_te_timer_function(unsigned long arg)
{
	struct mcde_chnl_state *chnl;
	u8 lnk;

	if (arg >= num_channels) {
		dev_err(&mcde_dev->dev, "%s invalid arg:%ld\n", __func__, arg);
		return;
	}

	chnl = &channels[arg];

	if (mcde_is_enabled && chnl->enabled && chnl->formatter_updated) {
		lnk = chnl->port.link;
		/* No TE answer; force stop */
		dsi_wfld(lnk, DSI_MCTL_MAIN_PHY_CTL, FORCE_STOP_MODE, true);
		udelay(20);
		dsi_wfld(lnk, DSI_MCTL_MAIN_PHY_CTL, FORCE_STOP_MODE, false);
		dev_info(&mcde_dev->dev, "DSI%d force stop\n", lnk);
		dsi_te_poll_set_timer(chnl, DSI_TE_NO_ANSWER_TIMEOUT);
	} else {
		dev_info(&mcde_dev->dev, "1:DSI force stop\n");
	}
}

static void dsi_te_request(struct mcde_chnl_state *chnl)
{
	u8 link = chnl->port.link;
	u8 virt_id = chnl->port.phy.dsi.virt_id;
	u32 settings;

	dev_vdbg(&mcde_dev->dev, "Request BTA TE, chnl=%d\n",
		chnl->id);

	set_channel_state_atomic(chnl, CHNLSTATE_WAIT_TE);

	dsi_wfld(link, DSI_MCTL_MAIN_DATA_CTL, BTA_EN, true);
	dsi_wfld(link, DSI_MCTL_MAIN_DATA_CTL, REG_TE_EN, true);
	dsi_wfld(link, DSI_CMD_MODE_CTL, TE_TIMEOUT, 0x3FF);
	settings = DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_NAT_ENUM(TE_REQ) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LONGNOTSHORT(false) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_ID(virt_id) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_SIZE(2) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_LP_EN(true) |
		DSI_DIRECT_CMD_MAIN_SETTINGS_CMD_HEAD_ENUM(DCS_SHORT_WRITE_1);
	dsi_wreg(link, DSI_DIRECT_CMD_MAIN_SETTINGS, settings);
	dsi_wreg(link, DSI_DIRECT_CMD_WRDAT0, DCS_CMD_SET_TEAR_ON);
	dsi_wreg(link, DSI_DIRECT_CMD_STS_CLR,
		DSI_DIRECT_CMD_STS_CLR_TE_RECEIVED_CLR(true));
	dsi_wfld(link, DSI_DIRECT_CMD_STS_CTL, TE_RECEIVED_EN, true);
	dsi_wreg(link, DSI_CMD_MODE_STS_CLR,
		DSI_CMD_MODE_STS_CLR_ERR_NO_TE_CLR(true));
	dsi_wfld(link, DSI_CMD_MODE_STS_CTL, ERR_NO_TE_EN, true);
	dsi_wreg(link, DSI_DIRECT_CMD_SEND, true);
}

/* MCDE channels */
static struct mcde_chnl_state *_mcde_chnl_get(enum mcde_chnl chnl_id,
	enum mcde_fifo fifo, const struct mcde_port *port)
{
	int i;
	struct mcde_chnl_state *chnl = NULL;
	struct mcde_platform_data *pdata = mcde_dev->dev.platform_data;

	static struct mcde_col_transform ycbcr_2_rgb = {
		/* Note that in MCDE YUV 422 pixels come as VYU pixels */
		.matrix = {
			{0xff30, 0x012a, 0xff9c},
			{0x0000, 0x012a, 0x0204},
			{0x0199, 0x012a, 0x0000},
		},
		.offset = {0x0088, 0xfeeb, 0xff21},
	};

	static struct mcde_col_transform rgb_2_ycbcr = {
		.matrix = {
			{0x0042, 0x0081, 0x0019},
			{0xffda, 0xffb6, 0x0070},
			{0x0070, 0xffa2, 0xffee},
		},
		.offset = {0x0010, 0x0080, 0x0080},
	};

	/* Allocate channel */
	for (i = 0; i < num_channels; i++) {
		if (chnl_id == channels[i].id)
			chnl = &channels[i];
	}
	if (!chnl) {
		dev_dbg(&mcde_dev->dev, "Invalid channel, chnl=%d\n", chnl_id);
		return ERR_PTR(-EINVAL);
	}
	if (chnl->reserved) {
		dev_dbg(&mcde_dev->dev, "Channel in use, chnl=%d\n", chnl_id);
		return ERR_PTR(-EBUSY);
	}

	chnl->port = *port;
	chnl->fifo = fifo;
	chnl->formatter_updated = false;
	chnl->ycbcr_2_rgb = ycbcr_2_rgb;
	chnl->rgb_2_ycbcr = rgb_2_ycbcr;
	chnl->oled_color_conversion = false;

	chnl->blend_en = true;
	chnl->blend_ctrl = MCDE_CRA0_BLENDCTRL_SOURCE;
	chnl->alpha_blend = 0xFF;
	chnl->rotbuf1 = pdata->rotbuf1;
	chnl->rotbuf2 = pdata->rotbuf2;
	chnl->rotbufsize = pdata->rotbufsize;

	_mcde_chnl_apply(chnl);
	chnl->reserved = true;

	if (chnl->port.type == MCDE_PORTTYPE_DPI) {
		chnl->clk_dpi = clk_get(&mcde_dev->dev, CLK_DPI);
		if (chnl->port.phy.dpi.tv_mode)
			chnl->vcmp_per_field = true;
	} else if (chnl->port.type == MCDE_PORTTYPE_DSI &&
							dsi_use_clk_framework) {
		char dsihs_name[10];
		char dsilp_name[10];

		sprintf(dsihs_name, "dsihs%d", port->link);
		sprintf(dsilp_name, "dsilp%d", port->link);

		chnl->clk_dsi_lp = clk_get(&mcde_dev->dev, dsilp_name);
		chnl->clk_dsi_hs = clk_get(&mcde_dev->dev, dsihs_name);
		if (port->phy.dsi.lp_freq != clk_round_rate(chnl->clk_dsi_lp,
							port->phy.dsi.lp_freq))
			dev_warn(&mcde_dev->dev, "Could not set dsi lp freq"
					" to %d\n", port->phy.dsi.lp_freq);
		WARN_ON_ONCE(clk_set_rate(chnl->clk_dsi_lp,
							port->phy.dsi.lp_freq));
		if (port->phy.dsi.hs_freq != clk_round_rate(chnl->clk_dsi_hs,
							port->phy.dsi.hs_freq))
			dev_warn(&mcde_dev->dev, "Could not set dsi hs freq"
					" to %d\n", port->phy.dsi.hs_freq);
		WARN_ON_ONCE(clk_set_rate(chnl->clk_dsi_hs,
							port->phy.dsi.hs_freq));
	}
	return chnl;
}

static int _mcde_chnl_apply(struct mcde_chnl_state *chnl)
{
	bool roten = false;
	u8 rotdir = 0;

	if (chnl->rotation == MCDE_DISPLAY_ROT_90_CCW) {
		roten = true;
		rotdir = MCDE_ROTACONF_ROTDIR_CCW;
	} else if (chnl->rotation == MCDE_DISPLAY_ROT_90_CW) {
		roten = true;
		rotdir = MCDE_ROTACONF_ROTDIR_CW;
	}
	/* REVIEW: 180 deg? */

	chnl->regs.bpp = portfmt2bpp(chnl->port.pixel_format);
	chnl->regs.roten = roten;
	chnl->regs.rotdir = rotdir;
	chnl->regs.rotbuf1 = chnl->rotbuf1;
	chnl->regs.rotbuf2 = chnl->rotbuf2;
	chnl->regs.rotbufsize = chnl->rotbufsize;
	chnl->regs.palette_enable = chnl->palette_enable;
	chnl->regs.map_r = chnl->map_r;
	chnl->regs.map_g = chnl->map_g;
	chnl->regs.map_b = chnl->map_b;
	if (chnl->port.type == MCDE_PORTTYPE_DSI) {
		chnl->regs.clksel = MCDE_CRA1_CLKSEL_MCDECLK;
		chnl->regs.dsipacking =
				portfmt2dsipacking(chnl->port.pixel_format);
	} else if (chnl->port.type == MCDE_PORTTYPE_DPI) {
		if (chnl->port.phy.dpi.tv_mode) {
			chnl->regs.internal_clk = false;
			chnl->regs.bcd = true;
			if (chnl->id == MCDE_CHNL_A)
				chnl->regs.clksel = MCDE_CRA1_CLKSEL_TV1CLK;
			else
				chnl->regs.clksel = MCDE_CRA1_CLKSEL_TV2CLK;
		} else {
			chnl->regs.internal_clk = true;
			chnl->regs.clksel = MCDE_CRA1_CLKSEL_CLKPLL72;
			chnl->regs.cdwin =
					portfmt2cdwin(chnl->port.pixel_format);
			chnl->regs.bcd = (chnl->port.phy.dpi.clock_div < 2);
			if (!chnl->regs.bcd)
				chnl->regs.pcd =
					chnl->port.phy.dpi.clock_div - 2;
		}
		dpi_video_mode_apply(chnl);
	}

	chnl->regs.blend_ctrl = chnl->blend_ctrl;
	chnl->regs.blend_en = chnl->blend_en;
	chnl->regs.alpha_blend = chnl->alpha_blend;

	chnl->regs.dirty = true;

	dev_vdbg(&mcde_dev->dev, "Channel applied, chnl=%d\n", chnl->id);
	return 0;
}

static void setup_channel(struct mcde_chnl_state *chnl)
{
	set_channel_state_sync(chnl, CHNLSTATE_SETUP);

	if (chnl->port.type == MCDE_PORTTYPE_DPI && chnl->tv_regs.dirty)
		update_dpi_registers(chnl->id, &chnl->tv_regs);

	/*
	 * For command mode displays using external sync (TE0/TE1), the first
	 * frame need special treatment to avoid garbage on the panel.
	 * This mechanism is placed here because it needs the chnl_state and
	 * modifies settings before they are committed to the registers.
	 */
	if (!chnl->port.update_auto_trig && chnl->first_frame_vsync_fix) {
		switch (chnl->port.sync_src) {
		case MCDE_SYNCSRC_TE0:
		case MCDE_SYNCSRC_TE1:
			/* Save requested mode. */
			chnl->port.requested_sync_src = chnl->port.sync_src;
			chnl->port.requested_frame_trig = chnl->port.frame_trig;
			/*
			 * Temporarily set other mode.
			 * Requested mode will be set at next frame.
			 */
			chnl->port.sync_src = MCDE_SYNCSRC_OFF;
			chnl->port.frame_trig = MCDE_TRIG_SW;
			break;
		default:
			/* No vsync switch needed. */
			chnl->first_frame_vsync_fix = false;
			break;
		}
	}

	if (chnl->id == MCDE_CHNL_A || chnl->id == MCDE_CHNL_B) {
		if (chnl->col_regs.dirty)
			update_col_registers(chnl->id, &chnl->col_regs);
		if (chnl->oled_regs.dirty)
			update_oled_registers(chnl->id, &chnl->oled_regs);
	}
	if (chnl->regs.dirty)
		update_channel_registers(chnl->id, &chnl->regs, &chnl->port,
						chnl->fifo, &chnl->vmode);
}

static void chnl_update_continous(struct mcde_chnl_state *chnl,
						bool tripple_buffer)
{
	if (chnl->state == CHNLSTATE_RUNNING) {
		if (!tripple_buffer)
			wait_for_vcmp(chnl);
	}

	if (atomic_cmpxchg(&chnl->ovly0->update_sbb, 1, 0))
		mcde_update_ovly_db_register_sbb(chnl->ovly0->idx,
			&chnl->ovly0->regs, chnl->ovly0->regs.col_conv);
	if (atomic_cmpxchg(&chnl->ovly1->update_sbb, 1, 0))
		mcde_update_ovly_db_register_sbb(chnl->ovly1->idx,
			&chnl->ovly1->regs, chnl->ovly1->regs.col_conv);

	if (chnl->state == CHNLSTATE_RUNNING)
		return;

	setup_channel(chnl);
	if (chnl->port.sync_src == MCDE_SYNCSRC_TE0)
		mcde_wfld(MCDE_CRC, SYCEN0, true);
	else if (chnl->port.sync_src == MCDE_SYNCSRC_TE1)
		mcde_wfld(MCDE_CRC, SYCEN1, true);

	enable_flow(chnl);
	set_channel_state_atomic(chnl, CHNLSTATE_RUNNING);
}

static void chnl_update_non_continous(struct mcde_chnl_state *chnl)
{
	/* Commit settings to registers */
	setup_channel(chnl);
	if (chnl->port.type != MCDE_PORTTYPE_DSI)
		return;

	switch (chnl->port.sync_src) {
	case MCDE_SYNCSRC_OFF:
		if (chnl->port.frame_trig == MCDE_TRIG_SW) {
			do_softwaretrig(chnl);
			if (chnl->first_frame_vsync_fix) {
				/* restore requested vsync mode */
				chnl->port.sync_src =
					chnl->port.requested_sync_src;
				chnl->port.frame_trig =
					chnl->port.requested_frame_trig;
				chnl->regs.dirty = true;
				chnl->first_frame_vsync_fix = false;
				dev_vdbg(&mcde_dev->dev,
					"SWITCH TO TE0 DSIx\n");
			}
		} else {
			enable_flow(chnl);
			set_channel_state_atomic(chnl, CHNLSTATE_RUNNING);
			disable_flow(chnl);
			set_channel_state_atomic(chnl, CHNLSTATE_STOPPING);
		}
		dev_vdbg(&mcde_dev->dev, "Chnl update (no sync), chnl=%d\n",
				chnl->id);
		break;
	case MCDE_SYNCSRC_BTA:
		if (chnl->power_mode == MCDE_DISPLAY_PM_ON) {
			dsi_te_request(chnl);
		} else {
			if (chnl->port.frame_trig == MCDE_TRIG_SW)
				do_softwaretrig(chnl);
		}
		if (chnl->port.frame_trig == MCDE_TRIG_HW) {
			/*
			 * During BTA TE the MCDE block will be stalled,
			 * once the TE is received the DMA trig will
			 * happen
			 */
			enable_flow(chnl);
			disable_flow(chnl);
		}
		break;
	case MCDE_SYNCSRC_TE0:
		set_channel_state_atomic(chnl, CHNLSTATE_WAIT_TE);
		enable_flow(chnl);
		mcde_wfld(MCDE_CRC, SYCEN0, true);
		break;
	case MCDE_SYNCSRC_TE1:
		set_channel_state_atomic(chnl, CHNLSTATE_WAIT_TE);
		enable_flow(chnl);
		mcde_wfld(MCDE_CRC, SYCEN1, true);
		break;
	case MCDE_SYNCSRC_TE_POLLING:
	default:
		break;
	}
}

static void update_oled_conversion(struct mcde_chnl_state *chnl,
		struct mcde_ovly_state *ovly)
{
	struct mcde_ovly_state *ovly0 = chnl->ovly0;
	struct mcde_ovly_state *ovly1 = chnl->ovly1;
	static struct mcde_oled_transform yuv240_2_rgb = {
		/* Note that in MCDE YUV 422 pixels come as VYU pixels */
		.matrix = {
			{0x1990, 0x12A0, 0x0000},
			{0x2D00, 0x12A0, 0x2640},
			{0x0000, 0x12A0, 0x1FFF},
		},
		.offset = {0x2DF0, 0x0870, 0x3150},
	};
	static struct mcde_col_transform rgb_2_yuv240 = {
		/* Note that in MCDE YUV 422 pixels come as VYU pixels */
		.matrix = {
			{0x0042, 0x0081, 0x0019},
			{0xffda, 0xffb5, 0x0071},
			{0x0070, 0xffa2, 0xffee},
		},
		.offset = {0x0010, 0x0080, 0x0080},
	};

	/* Never configure the OLED matrix for these cases */
	if (chnl->port.type == MCDE_PORTTYPE_DPI &&
			chnl->port.phy.dpi.tv_mode)
		return;
	if (chnl->port.pixel_format == MCDE_PORTPIXFMT_DSI_YCBCR422)
		return;
	if (chnl->port.update_auto_trig &&
			chnl->port.type == MCDE_PORTTYPE_DSI)
		return;

	/* Check oled/color conversion state and setup overlays */
	if (!chnl->oled_color_conversion &&
		((ovly0 != NULL && ovly0->regs.enabled && ovly0->paddr != 0 &&
				ovly0->pix_fmt == MCDE_OVLYPIXFMT_YCbCr422) ||
		(ovly1 != NULL && ovly1->regs.enabled && ovly1->paddr != 0 &&
				ovly1->pix_fmt == MCDE_OVLYPIXFMT_YCbCr422))) {
		chnl->oled_color_conversion = true;
		if (apply_extra_oled_color_conv) {
			dev_dbg(&mcde_dev->dev, "%s: yuv240_2_rgb_extra\n",
								__func__);
			mcde_chnl_oled_convert_apply(chnl,
						&yuv240_2_rgb_extra);
		} else {
			dev_dbg(&mcde_dev->dev, "%s: yuv240_2_rgb\n", __func__);
			mcde_chnl_oled_convert_apply(chnl, &yuv240_2_rgb);
		}
		mcde_chnl_col_convert_apply(chnl, &rgb_2_yuv240);
		if (ovly0 != NULL && ovly0->regs.enabled && ovly0->paddr != 0 &&
				ovly0->pix_fmt == MCDE_OVLYPIXFMT_YCbCr422) {
			dev_dbg(&mcde_dev->dev, "%s: ovly0 YUV\n", __func__);
			ovly0->regs.col_conv = MCDE_OVL0CR_COLCCTRL_DISABLED;
			ovly0->regs.dirty = true;
			ovly1->regs.col_conv = MCDE_OVL0CR_COLCCTRL_ENABLED_SAT;
			ovly1->regs.dirty = true;
		} else {
			dev_dbg(&mcde_dev->dev, "%s: ovly1 YUV\n", __func__);
			ovly0->regs.col_conv = MCDE_OVL0CR_COLCCTRL_ENABLED_SAT;
			ovly0->regs.dirty = true;
			ovly1->regs.col_conv = MCDE_OVL0CR_COLCCTRL_DISABLED;
			ovly1->regs.dirty = true;
		}
		chnl->regs.oled_enable = true;
		chnl->regs.background_yuv = true;
		chnl->regs.dirty = true;
	} else if (!chnl->oled_color_conversion && ovly->regs.enabled &&
			apply_extra_oled_color_conv &&
			!chnl->regs.oled_enable) {
		dev_dbg(&mcde_dev->dev, "%s: rgb_2_rgb_extra\n", __func__);
		mcde_chnl_oled_convert_apply(chnl, &rgb_2_rgb_extra);
		chnl->regs.oled_enable = true;
		chnl->regs.dirty = true;
	} else if (chnl->oled_color_conversion &&
			(!ovly0->regs.enabled ||
				ovly0->pix_fmt != MCDE_OVLYPIXFMT_YCbCr422) &&
			(!ovly1->regs.enabled ||
				ovly1->pix_fmt != MCDE_OVLYPIXFMT_YCbCr422)) {
		/* Turn off if no overlay needs YUV conv */
		dev_dbg(&mcde_dev->dev, "%s: disable oled\n", __func__);
		chnl->oled_color_conversion = false;
		ovly0->regs.col_conv = MCDE_OVL0CR_COLCCTRL_DISABLED;
		ovly0->regs.dirty = true;
		ovly1->regs.col_conv = MCDE_OVL0CR_COLCCTRL_DISABLED;
		ovly1->regs.dirty = true;
		chnl->regs.background_yuv = false;
		chnl->regs.oled_enable = false;
		if (apply_extra_oled_color_conv) {
			dev_dbg(&mcde_dev->dev, "%s: rgb_2_rgb_extra at off\n",
								__func__);
			mcde_chnl_oled_convert_apply(chnl, &rgb_2_rgb_extra);
			chnl->regs.oled_enable = true;
		}
		chnl->regs.dirty = true;
	}
}

static void chnl_update_overlay(struct mcde_chnl_state *chnl,
						struct mcde_ovly_state *ovly)
{
	if (!ovly)
		return;

	if (ovly->regs.dirty_buf) {
		if (!chnl->port.update_auto_trig)
			set_channel_state_sync(chnl, CHNLSTATE_SETUP);
		else
			wait_for_vsync(chnl);
		update_overlay_registers_on_the_fly(ovly->idx, &ovly->regs);
		mcde_debugfs_overlay_update(chnl->id, ovly != chnl->ovly0);
	}

	/* Test and set oled and color conversion state if necessary */
	update_oled_conversion(chnl, ovly);

	if (ovly->regs.dirty) {
		if (!chnl->port.update_auto_trig)
			set_channel_state_sync(chnl, CHNLSTATE_SETUP);
		chnl_ovly_pixel_format_apply(chnl, ovly);
		update_overlay_registers(ovly, &ovly->regs, &chnl->port,
			chnl->fifo, ovly->stride,
			chnl->vmode.interlaced, chnl->rotation);
		if (chnl->id == MCDE_CHNL_A || chnl->id == MCDE_CHNL_B) {
			update_oled_registers(chnl->id, &chnl->oled_regs);
			update_col_registers(chnl->id, &chnl->col_regs);
		}
	}
}

static int _mcde_chnl_update(struct mcde_chnl_state *chnl,
					bool tripple_buffer)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	/* TODO: lock & make wait->trig async */
	if (!chnl->enabled)
		return -EINVAL;

	if (chnl->port.update_auto_trig && tripple_buffer)
		wait_for_vcmp(chnl);

	/* For now the update area will always be full screen */
	chnl->regs.x   = 0;
	chnl->regs.y   = 0;
	/*
	 * regs.ppl and regs.lpf are values that is used before
	 * the rotation in MCDE. So if the channel is doing rotation
	 * then flip the values.
	 *
	 * vmode.xres and vmode.yres are in the current resolution of the
	 * screen.
	 */
	if ((chnl->rotation == MCDE_DISPLAY_ROT_90_CCW) ||
			(chnl->rotation == MCDE_DISPLAY_ROT_90_CW)) {

		chnl->regs.ppl = chnl->vmode.yres;
		chnl->regs.lpf = chnl->vmode.xres;
	} else {
		chnl->regs.ppl = chnl->vmode.xres;
		chnl->regs.lpf = chnl->vmode.yres;
	}

	if (chnl->port.type == MCDE_PORTTYPE_DPI &&
						chnl->port.phy.dpi.tv_mode) {
		/* subtract border */
		chnl->regs.ppl -= chnl->tv_regs.dho + chnl->tv_regs.alw;
		/* subtract double borders, ie. for both fields */
		chnl->regs.lpf -= 2 * (chnl->tv_regs.dvo + chnl->tv_regs.bsl);
	} else if (chnl->port.type == MCDE_PORTTYPE_DSI &&
			chnl->vmode.interlaced)
		chnl->regs.lpf /= 2;

	chnl_update_overlay(chnl, chnl->ovly0);
	chnl_update_overlay(chnl, chnl->ovly1);

	if (chnl->port.update_auto_trig)
		chnl_update_continous(chnl, tripple_buffer);
	else
		chnl_update_non_continous(chnl);

	dev_vdbg(&mcde_dev->dev, "Channel updated, chnl=%d\n", chnl->id);
	mcde_debugfs_channel_update(chnl->id);
	return 0;
}

static int _mcde_chnl_enable(struct mcde_chnl_state *chnl)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);
	chnl->enabled = true;
	chnl->first_frame_vsync_fix = true;
	return 0;
}

/* API entry points */
/* MCDE channels */
struct mcde_chnl_state *mcde_chnl_get(enum mcde_chnl chnl_id,
			enum mcde_fifo fifo, const struct mcde_port *port)
{
	struct mcde_chnl_state *chnl;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);
	chnl = _mcde_chnl_get(chnl_id, fifo, port);
	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);

	return chnl;
}

int mcde_chnl_set_pixel_format(struct mcde_chnl_state *chnl,
					enum mcde_port_pix_fmt pix_fmt)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (!chnl->reserved)
		return -EINVAL;
	chnl->port.pixel_format = pix_fmt;

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);

	return 0;
}

int mcde_chnl_set_palette(struct mcde_chnl_state *chnl,
					struct mcde_palette_table *palette)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (!chnl->reserved)
		return -EINVAL;
	if (palette != NULL) {
		chnl->map_r = palette->map_col_ch0;
		chnl->map_g = palette->map_col_ch1;
		chnl->map_b = palette->map_col_ch2;
		chnl->palette_enable = true;
	} else {
		chnl->map_r = NULL;
		chnl->map_g = NULL;
		chnl->map_b = NULL;
		chnl->palette_enable = false;
	}

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);
	return 0;
}

void mcde_chnl_set_col_convert(struct mcde_chnl_state *chnl,
					struct mcde_col_transform *transform,
					enum   mcde_col_convert    convert)
{
	switch (convert) {
	case MCDE_CONVERT_RGB_2_YCBCR:
		memcpy(&chnl->rgb_2_ycbcr, transform,
				sizeof(struct mcde_col_transform));
		/* force update: */
		if (chnl->transform == &chnl->rgb_2_ycbcr) {
			chnl->transform = NULL;
			chnl->ovly0->dirty = true;
			chnl->ovly1->dirty = true;
		}
		break;
	case MCDE_CONVERT_YCBCR_2_RGB:
		memcpy(&chnl->ycbcr_2_rgb, transform,
				sizeof(struct mcde_col_transform));
		/* force update: */
		if (chnl->transform == &chnl->ycbcr_2_rgb) {
			chnl->transform = NULL;
			chnl->ovly0->dirty = true;
			chnl->ovly1->dirty = true;
		}
		break;
	default:
		/* Trivial transforms are handled internally */
		dev_warn(&mcde_dev->dev,
			"%s: unsupported col convert\n", __func__);
		break;
	}
}

int mcde_chnl_set_video_mode(struct mcde_chnl_state *chnl,
					struct mcde_video_mode *vmode)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (chnl == NULL || vmode == NULL)
		return -EINVAL;

	chnl->vmode = *vmode;

	chnl->ovly0->dirty = true;
	if (chnl->ovly1)
		chnl->ovly1->dirty = true;

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);

	return 0;
}
EXPORT_SYMBOL(mcde_chnl_set_video_mode);

int mcde_chnl_set_rotation(struct mcde_chnl_state *chnl,
					enum mcde_display_rotation rotation)
{
	dev_vdbg(&mcde_dev->dev, "%s, rotation=%d\n", __func__, rotation);

	if (!chnl->reserved)
		return -EINVAL;

	if ((rotation == MCDE_DISPLAY_ROT_90_CW ||
			rotation == MCDE_DISPLAY_ROT_90_CCW) &&
			(chnl->id != MCDE_CHNL_A && chnl->id != MCDE_CHNL_B))
		return -EINVAL;

	chnl->rotation = rotation;

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);

	return 0;
}

bool mcde_chnl_is_rotated_90(struct mcde_chnl_state *chnl)
{
	if (chnl->rotation == MCDE_DISPLAY_ROT_90_CCW ||
			chnl->rotation == MCDE_DISPLAY_ROT_90_CW)
		return true;
	else
		return false;
}

int mcde_chnl_set_power_mode(struct mcde_chnl_state *chnl,
				enum mcde_display_power_mode power_mode)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (!chnl->reserved)
		return -EINVAL;

	chnl->power_mode = power_mode;

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);

	return 0;
}

int mcde_chnl_apply(struct mcde_chnl_state *chnl)
{
	int ret ;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (!chnl->reserved)
		return -EINVAL;

	mcde_lock(__func__, __LINE__);
	ret = _mcde_chnl_apply(chnl);
	mcde_unlock(__func__, __LINE__);

	dev_vdbg(&mcde_dev->dev, "%s exit with ret %d\n", __func__, ret);

	return ret;
}

void mcde_chnl_set_dirty(struct mcde_chnl_state *chnl)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (!chnl->reserved)
		return;

	mcde_lock(__func__, __LINE__);
	chnl->regs.dirty = true;
	mcde_unlock(__func__, __LINE__);

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);
}

void mcde_chnl_update_sync_src(struct mcde_chnl_state *chnl,
			       enum mcde_sync_src src)
{
	mcde_lock(__func__, __LINE__);
	chnl->port.sync_src = src;
	mcde_unlock(__func__, __LINE__);
}

int mcde_chnl_update(struct mcde_chnl_state *chnl,
					bool tripple_buffer)
{
	int ret;
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (!chnl->reserved)
		return -EINVAL;

	mcde_lock(__func__, __LINE__);
	enable_mcde_hw();
	if (!chnl->formatter_updated)
		(void)update_channel_static_registers(chnl);

	if (chnl->regs.roten && !chnl->esram_is_enabled) {
		WARN_ON_ONCE(regulator_enable(regulator_esram_epod));
		chnl->esram_is_enabled = true;
	} else if (!chnl->regs.roten && chnl->esram_is_enabled) {
		WARN_ON_ONCE(regulator_disable(regulator_esram_epod));
		chnl->esram_is_enabled = false;
	}

	ret = _mcde_chnl_update(chnl, tripple_buffer);

	mcde_unlock(__func__, __LINE__);


	if (chnl->id == 0)
		handle_chnl0_timeout(chnl);

	dev_vdbg(&mcde_dev->dev, "%s exit with ret %d\n", __func__, ret);

	return ret;
}

void mcde_chnl_put(struct mcde_chnl_state *chnl)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (chnl->enabled) {
		stop_channel(chnl);
		cancel_delayed_work(&hw_timeout_work);
		disable_mcde_hw(false, true);
		chnl->enabled = false;
	}

	chnl->reserved = false;
	if (chnl->port.type == MCDE_PORTTYPE_DPI) {
		clk_put(chnl->clk_dpi);
		if (chnl->port.phy.dpi.tv_mode) {
			chnl->vcmp_per_field = false;
			chnl->even_vcmp = false;
		}
	} else if (chnl->port.type == MCDE_PORTTYPE_DSI) {
		if (dsi_use_clk_framework) {
			clk_put(chnl->clk_dsi_lp);
			clk_put(chnl->clk_dsi_hs);
		}
	}

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);
}

void mcde_chnl_stop_flow(struct mcde_chnl_state *chnl)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	mcde_lock(__func__, __LINE__);
	if (mcde_is_enabled && chnl->enabled)
		stop_channel(chnl);
	mcde_unlock(__func__, __LINE__);

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);
}

void mcde_chnl_enable(struct mcde_chnl_state *chnl)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	mcde_lock(__func__, __LINE__);
	_mcde_chnl_enable(chnl);
	mcde_unlock(__func__, __LINE__);

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);
}

void mcde_chnl_disable(struct mcde_chnl_state *chnl)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	mcde_lock(__func__, __LINE__);
	cancel_delayed_work(&hw_timeout_work);
	/* The channel must be stopped before it is disabled */
	WARN_ON_ONCE(chnl->state == CHNLSTATE_RUNNING);
	disable_mcde_hw(false, true);
	chnl->enabled = false;
	mcde_unlock(__func__, __LINE__);

	dev_vdbg(&mcde_dev->dev, "%s exit\n", __func__);
}

void mcde_disable_ulpm_support(bool disable)
{
	mcde_lock(__func__, __LINE__);
	if (disable)
		disable_ulpm = true;
	else
		disable_ulpm = false;
	mcde_unlock(__func__, __LINE__);
}

void set_rgb_extra_matrix(struct mcde_oled_transform *matrix)
{
	/* Only reset oled matrix for channel 0 */
	struct mcde_chnl_state *chnl = &channels[0];

	mcde_lock(__func__, __LINE__);

	if (chnl)
		chnl->oled_transform = NULL;
	rgb_2_rgb_extra = *matrix;
	mcde_unlock(__func__, __LINE__);
}

struct mcde_oled_transform *get_rgb_extra_matrix(void)
{
	return &rgb_2_rgb_extra;
}

void set_yuv_extra_matrix(struct mcde_oled_transform *matrix)
{
	/* Only reset oled matrix for channel 0 */
	struct mcde_chnl_state *chnl = &channels[0];

	mcde_lock(__func__, __LINE__);
	if (chnl)
		chnl->oled_transform = NULL;
	yuv240_2_rgb_extra = *matrix;
	mcde_unlock(__func__, __LINE__);
}

struct mcde_oled_transform *get_yuv_extra_matrix(void)
{
	return &yuv240_2_rgb_extra;
}

void mcde_extra_oled_conversion(bool enable)
{
	struct mcde_chnl_state *chnl = &channels[0];

	apply_extra_oled_color_conv = enable;
	if (apply_extra_oled_color_conv) {
		if (chnl->regs.background_yuv) {
			dev_dbg(&mcde_dev->dev, "%s: yuv240_2_rgb_extra\n",
								__func__);
			mcde_chnl_oled_convert_apply(chnl, &yuv240_2_rgb_extra);
		} else {
			dev_dbg(&mcde_dev->dev, "%s: rgb_2_rgb_extra\n",
								__func__);
			mcde_chnl_oled_convert_apply(chnl, &rgb_2_rgb_extra);
		}
	} else {
		if (chnl->regs.background_yuv) {
			dev_dbg(&mcde_dev->dev, "%s: yuv240_2_rgb\n", __func__);
			mcde_chnl_oled_convert_apply(chnl, &yuv240_2_rgb);
		} else {
			dev_dbg(&mcde_dev->dev, "%s: disable oled\n", __func__);
			chnl->regs.oled_enable = false;
		}
	}
}

/* MCDE overlays */
struct mcde_ovly_state *mcde_ovly_get(struct mcde_chnl_state *chnl)
{
	struct mcde_ovly_state *ovly;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (!chnl->reserved)
		return ERR_PTR(-EINVAL);

	if (!chnl->ovly0->inuse)
		ovly = chnl->ovly0;
	else if (chnl->ovly1 && !chnl->ovly1->inuse)
		ovly = chnl->ovly1;
	else
		ovly = ERR_PTR(-EBUSY);

	if (!IS_ERR(ovly)) {
		ovly->inuse = true;
		ovly->paddr = 0;
		ovly->stride = 0;
		ovly->pix_fmt = MCDE_OVLYPIXFMT_RGB565;
		ovly->src_x = 0;
		ovly->src_y = 0;
		ovly->dst_x = 0;
		ovly->dst_y = 0;
		ovly->dst_z = 0;
		ovly->w = 0;
		ovly->h = 0;
		ovly->alpha_value = 0xFF;
		ovly->alpha_source = MCDE_OVL1CONF2_BP_PER_PIXEL_ALPHA;
		ovly->dirty = true;
		mcde_ovly_apply(ovly);
	}

	return ovly;
}

void mcde_ovly_put(struct mcde_ovly_state *ovly)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	if (!ovly->inuse)
		return;
	if (ovly->regs.enabled) {
		ovly->paddr = 0;
		ovly->dirty = true;
		mcde_ovly_apply(ovly);/* REVIEW: API call calling API call! */
	}
	ovly->inuse = false;
}

void mcde_ovly_set_source_buf(struct mcde_ovly_state *ovly, u32 paddr)
{
	if (!ovly->inuse)
		return;

	ovly->dirty = paddr == 0 || ovly->paddr == 0;
	ovly->dirty_buf = true;

	ovly->paddr = paddr;
}

void mcde_ovly_set_source_info(struct mcde_ovly_state *ovly,
	u32 stride, enum mcde_ovly_pix_fmt pix_fmt)
{
	if (!ovly->inuse)
		return;

	ovly->stride = stride;
	ovly->pix_fmt = pix_fmt;
	ovly->dirty = true;
}

void mcde_ovly_set_source_area(struct mcde_ovly_state *ovly,
	u16 x, u16 y, u16 w, u16 h)
{
	if (!ovly->inuse)
		return;

	ovly->src_x = x;
	ovly->src_y = y;
	ovly->w = w;
	ovly->h = h;
	ovly->dirty = true;
}

void mcde_ovly_set_dest_pos(struct mcde_ovly_state *ovly, u16 x, u16 y, u8 z)
{
	if (!ovly->inuse)
		return;

	ovly->dst_x = x;
	ovly->dst_y = y;
	ovly->dst_z = z;
	ovly->dirty = true;
}

void mcde_ovly_apply(struct mcde_ovly_state *ovly)
{
	if (!ovly->inuse)
		return;

	mcde_lock(__func__, __LINE__);

	if (ovly->dirty || ovly->dirty_buf) {
		ovly->regs.ch_id = ovly->chnl->id;
		ovly->regs.enabled = ovly->paddr != 0;
		ovly->regs.baseaddress0 = ovly->paddr;
		ovly->regs.baseaddress1 =
					ovly->regs.baseaddress0 + ovly->stride;
		ovly->regs.dirty_buf = true;
		ovly->dirty_buf = false;
	}
	if (!ovly->dirty) {
		mcde_unlock(__func__, __LINE__);
		return;
	}

	switch (ovly->pix_fmt) {/* REVIEW: Extract to table */
	case MCDE_OVLYPIXFMT_RGB565:
		ovly->regs.bits_per_pixel = 16;
		ovly->regs.bpp = MCDE_EXTSRC0CONF_BPP_RGB565;
		ovly->regs.bgr = false;
		ovly->regs.bebo = false;
		ovly->regs.opq = true;
		break;
	case MCDE_OVLYPIXFMT_RGBA5551:
		ovly->regs.bits_per_pixel = 16;
		ovly->regs.bpp = MCDE_EXTSRC0CONF_BPP_IRGB1555;
		ovly->regs.bgr = false;
		ovly->regs.bebo = false;
		ovly->regs.opq = false;
		break;
	case MCDE_OVLYPIXFMT_RGBA4444:
		ovly->regs.bits_per_pixel = 16;
		ovly->regs.bpp = MCDE_EXTSRC0CONF_BPP_ARGB4444;
		ovly->regs.bgr = false;
		ovly->regs.bebo = false;
		ovly->regs.opq = false;
		break;
	case MCDE_OVLYPIXFMT_RGB888:
		ovly->regs.bits_per_pixel = 24;
		ovly->regs.bpp = MCDE_EXTSRC0CONF_BPP_RGB888;
		ovly->regs.bgr = false;
		ovly->regs.bebo = false;
		ovly->regs.opq = true;
		break;
	case MCDE_OVLYPIXFMT_RGBX8888:
		ovly->regs.bits_per_pixel = 32;
		ovly->regs.bpp = MCDE_EXTSRC0CONF_BPP_XRGB8888;
		ovly->regs.bgr = false;
		ovly->regs.bebo = true;
		ovly->regs.opq = true;
		break;
	case MCDE_OVLYPIXFMT_RGBA8888:
		ovly->regs.bits_per_pixel = 32;
		ovly->regs.bpp = MCDE_EXTSRC0CONF_BPP_ARGB8888;
		ovly->regs.bgr = false;
		ovly->regs.bebo = false;
		ovly->regs.opq = false;
		break;
	case MCDE_OVLYPIXFMT_YCbCr422:
		ovly->regs.bits_per_pixel = 16;
		ovly->regs.bpp = MCDE_EXTSRC0CONF_BPP_YCBCR422;
		ovly->regs.bgr = false;
		ovly->regs.bebo = false;
		ovly->regs.opq = true;
		break;
	default:
		break;
	}

	ovly->regs.ppl = ovly->w;
	ovly->regs.lpf = ovly->h;
	ovly->regs.cropx = ovly->src_x;
	ovly->regs.cropy = ovly->src_y;
	ovly->regs.xpos = ovly->dst_x;
	ovly->regs.ypos = ovly->dst_y;
	ovly->regs.z = ovly->dst_z > 0; /* 0 or 1 */
	if (!ovly->chnl->oled_color_conversion)
		ovly->regs.col_conv = MCDE_OVL0CR_COLCCTRL_DISABLED;
	ovly->regs.alpha_source = ovly->alpha_source;
	ovly->regs.alpha_value = ovly->alpha_value;

	ovly->regs.dirty = true;
	ovly->dirty = false;

	mcde_unlock(__func__, __LINE__);

	dev_vdbg(&mcde_dev->dev, "Overlay applied, idx=%d chnl=%d\n",
						ovly->idx, ovly->chnl->id);
}

static int init_clocks_and_power(struct platform_device *pdev)
{
	int ret = 0;
	struct mcde_platform_data *pdata = pdev->dev.platform_data;

	if (pdata->regulator_mcde_epod_id) {
		regulator_mcde_epod = regulator_get(&pdev->dev,
				pdata->regulator_mcde_epod_id);
		if (IS_ERR(regulator_mcde_epod)) {
			ret = PTR_ERR(regulator_mcde_epod);
			dev_warn(&pdev->dev,
				"%s: Failed to get regulator '%s'\n",
				__func__, pdata->regulator_mcde_epod_id);
			regulator_mcde_epod = NULL;
			return ret;
		}
	} else {
		dev_warn(&pdev->dev, "%s: No mcde regulator id supplied\n",
								__func__);
		return -EINVAL;
	}

	if (pdata->regulator_esram_epod_id) {
		regulator_esram_epod = regulator_get(&pdev->dev,
				pdata->regulator_esram_epod_id);
		if (IS_ERR(regulator_esram_epod)) {
			ret = PTR_ERR(regulator_esram_epod);
			dev_warn(&pdev->dev,
				"%s: Failed to get regulator '%s'\n",
				__func__, pdata->regulator_esram_epod_id);
			regulator_esram_epod = NULL;
			goto regulator_esram_err;
		}
	} else {
		dev_warn(&pdev->dev, "%s: No esram regulator id supplied\n",
								__func__);
	}

	if (pdata->regulator_vana_id) {
		regulator_vana = regulator_get(&pdev->dev,
				pdata->regulator_vana_id);
		if (IS_ERR(regulator_vana)) {
			ret = PTR_ERR(regulator_vana);
			dev_warn(&pdev->dev,
				"%s: Failed to get regulator '%s'\n",
				__func__, pdata->regulator_vana_id);
			regulator_vana = NULL;
			goto regulator_vana_err;
		}
	} else {
		dev_dbg(&pdev->dev, "%s: No vana regulator id supplied\n",
								__func__);
	}

	if (!dsi_use_clk_framework) {
		clock_dsi = clk_get(&pdev->dev, pdata->clock_dsi_id);
		if (IS_ERR(clock_dsi))
			dev_dbg(&pdev->dev, "%s: Failed to get clock '%s'\n",
						__func__, pdata->clock_dsi_id);

		clock_dsi_lp = clk_get(&pdev->dev, pdata->clock_dsi_lp_id);
		if (IS_ERR(clock_dsi_lp))
			dev_dbg(&pdev->dev, "%s: Failed to get clock '%s'\n",
					__func__, pdata->clock_dsi_lp_id);
	}

	clock_mcde = clk_get(&pdev->dev, CLK_MCDE);
	if (IS_ERR(clock_mcde)) {
		ret = PTR_ERR(clock_mcde);
		dev_warn(&pdev->dev, "%s: Failed to get mcde_clk\n", __func__);
		goto clk_mcde_err;
	}

	return ret;

clk_mcde_err:
	if (!dsi_use_clk_framework) {
		clk_put(clock_dsi_lp);
		clk_put(clock_dsi);
	}

	if (regulator_vana)
		regulator_put(regulator_vana);
regulator_vana_err:
	if (regulator_esram_epod)
		regulator_put(regulator_esram_epod);
regulator_esram_err:
	regulator_put(regulator_mcde_epod);
	return ret;
}

void mcde_hw_chnl_print(struct mcde_chnl_state *chnl)
{
	struct device *dev = &mcde_dev->dev;

	dev_info(dev, "enabled = %d\n", chnl->enabled);
	dev_info(dev, "reserved = %d\n", chnl->reserved);
	dev_info(dev, "id = %d\n", chnl->id);
	dev_info(dev, "fifo = %d\n", chnl->fifo);
	dev_info(dev, "reserved = %d\n", chnl->reserved);
	dev_info(dev, "struct port = %p\n", &chnl->port);
	dev_info(dev, "struct ovly0 = %p\n", chnl->ovly0);
	dev_info(dev, "struct ovly1 = %p\n", chnl->ovly1);
	dev_info(dev, "state = %d\n", chnl->state);
	dev_info(dev, "state_waitq = %p\n", &chnl->state_waitq);
	dev_info(dev, "vcmp_waitq = %p\n", &chnl->vcmp_waitq);
	dev_info(dev, "vcmp_cnt = %d\n", atomic_read(&chnl->vcmp_cnt));
	dev_info(dev, "struct dsi_te_timer = %p\n", &chnl->dsi_te_timer);
	dev_info(dev, "struct clk_dsi_lp = %p\n", chnl->clk_dsi_lp);
	dev_info(dev, "struct clk_dsi_hs = %p\n", chnl->clk_dsi_hs);
	dev_info(dev, "struct clk_dpi = %p\n", chnl->clk_dpi);
	dev_info(dev, "power_mode = %d\n", chnl->power_mode);
	dev_info(dev, "map_r = %p\n", &chnl->map_r);
	dev_info(dev, "map_g = %p\n", &chnl->map_g);
	dev_info(dev, "map_b = %p\n", &chnl->map_b);
	dev_info(dev, "palette_enable = %d\n", chnl->palette_enable);
	dev_info(dev, "struct vmode = %p\n", &chnl->vmode);
	dev_info(dev, "rotation = %d\n", chnl->rotation);
	dev_info(dev, "rotbuf1 = 0x%x\n", chnl->rotbuf1);
	dev_info(dev, "rotbuf2 = 0x%x\n", chnl->rotbuf2);
	dev_info(dev, "rotbufsize = %d\n", chnl->rotbufsize);
	dev_info(dev, "struct rgb_2_ycbcr = %p\n", &chnl->rgb_2_ycbcr);
	dev_info(dev, "struct ycbcr_2_rgb = %p\n", &chnl->ycbcr_2_rgb);
	dev_info(dev, "struct transform = %p\n", chnl->transform);
	dev_info(dev, "blend_ctrl = 0x%x\n", chnl->blend_ctrl);
	dev_info(dev, "blend_en = 0x%x\n", chnl->blend_en);
	dev_info(dev, "alpha_blend = 0x%x\n", chnl->alpha_blend);
	dev_info(dev, "struct regs = %p\n", &chnl->regs);
	dev_info(dev, "struct col_regs = %p\n", &chnl->col_regs);
	dev_info(dev, "struct tv_regs = %p\n", &chnl->tv_regs);
	dev_info(dev, "vcmp_per_field = %d\n", chnl->vcmp_per_field);
	dev_info(dev, "even_vcmp = %d\n", chnl->even_vcmp);
	dev_info(dev, "formatter_updated = %d\n", chnl->formatter_updated);
	dev_info(dev, "esram_is_enabled = %d\n", chnl->esram_is_enabled);
}

void mcde_hw_ovly_print(struct mcde_ovly_state *ovly)
{
	struct device *dev = &mcde_dev->dev;

	dev_info(dev, "inuse = %d\n", ovly->inuse);
	dev_info(dev, "idx = %d\n", ovly->idx);
	dev_info(dev, "struct chnl = %p\n", ovly->chnl);
	dev_info(dev, "dirty = %d\n", ovly->dirty);
	dev_info(dev, "dirty_buf = %d\n", ovly->dirty_buf);
	dev_info(dev, "paddr = 0x%x\n", ovly->paddr);
	dev_info(dev, "stride = %d\n", ovly->stride);
	dev_info(dev, "pix_fmt = %d\n", ovly->pix_fmt);
	dev_info(dev, "src_x = %d\n", ovly->src_x);
	dev_info(dev, "src_y = %d\n", ovly->src_y);
	dev_info(dev, "dst_x = %d\n", ovly->dst_x);
	dev_info(dev, "dst_y = %d\n", ovly->dst_y);
	dev_info(dev, "dst_z = %d\n", ovly->dst_z);
	dev_info(dev, "w = %d\n", ovly->w);
	dev_info(dev, "h = %d\n", ovly->h);
	dev_info(dev, "alpha_source = %d\n", ovly->alpha_source);
	dev_info(dev, "alpha_value = %d\n", ovly->alpha_value);

	dev_info(dev, "regs.enabled = %d\n", ovly->regs.enabled);
	dev_info(dev, "regs.dirty = %d\n", ovly->regs.dirty);
	dev_info(dev, "regs.dirty_buf = %d\n", ovly->regs.dirty_buf);
	dev_info(dev, "regs.ch_id = %d\n", ovly->regs.ch_id);
	dev_info(dev, "regs.baseaddress0 = 0x%x\n", ovly->regs.baseaddress0);
	dev_info(dev, "regs.baseaddress1 = 0x%x\n", ovly->regs.baseaddress1);
	dev_info(dev, "regs.bits_per_pixel = %d\n", ovly->regs.bits_per_pixel);
	dev_info(dev, "regs.bpp = %d\n", ovly->regs.bpp);
	dev_info(dev, "regs.bgr = %d\n", ovly->regs.bgr);
	dev_info(dev, "regs.bebo = %d\n", ovly->regs.bebo);
	dev_info(dev, "regs.opq = %d\n", ovly->regs.opq);
	dev_info(dev, "regs.col_conv = %d\n", ovly->regs.col_conv);
	dev_info(dev, "regs.alpha_source = %d\n", ovly->regs.alpha_source);
	dev_info(dev, "regs.alpha_value = %d\n", ovly->regs.alpha_value);
	dev_info(dev, "regs.pixoff = %d\n", ovly->regs.pixoff);
	dev_info(dev, "regs.ppl = %d\n", ovly->regs.ppl);
	dev_info(dev, "regs.lpf = %d\n", ovly->regs.lpf);
	dev_info(dev, "regs.cropx = %d\n", ovly->regs.cropx);
	dev_info(dev, "regs.cropy = %d\n", ovly->regs.cropy);
	dev_info(dev, "regs.xpos = %d\n", ovly->regs.xpos);
	dev_info(dev, "regs.ypos = %d\n", ovly->regs.ypos);
	dev_info(dev, "regs.z = %d\n", ovly->regs.z);
}

static void remove_clocks_and_power(struct platform_device *pdev)
{
	/* REVIEW: Release only if exist */
	/* REVIEW: Remove make sure MCDE is done */
	if (!dsi_use_clk_framework) {
		clk_put(clock_dsi_lp);
		clk_put(clock_dsi);
	}
	clk_put(clock_mcde);
	if (regulator_vana)
		regulator_put(regulator_vana);
	regulator_put(regulator_mcde_epod);
	regulator_put(regulator_esram_epod);
}

static int probe_hw(struct platform_device *pdev)
{
	int i;
	int ret;
	u32 pid;
	struct resource *res;

	dev_info(&mcde_dev->dev, "Probe HW\n");

	/* Get MCDE HW version */
	regulator_enable(regulator_mcde_epod);
	clk_enable(clock_mcde);
	pid = mcde_rreg(MCDE_PID);

	dev_info(&mcde_dev->dev, "MCDE HW revision 0x%.8X\n", pid);

	clk_disable(clock_mcde);
	regulator_disable(regulator_mcde_epod);

	switch (pid) {
	case MCDE_VERSION_3_0_8:
		num_dsilinks = 3;
		num_channels = 4;
		num_overlays = 6;
		dsi_ifc_is_supported = true;
		input_fifo_size = 128;
		output_fifo_ab_size = 640;
		output_fifo_c0c1_size = 160;
		dsi_use_clk_framework = true;
		dev_info(&mcde_dev->dev, "db8500 V2 HW\n");
		break;
	case MCDE_VERSION_4_0_4:
		num_dsilinks = 2;
		num_channels = 2;
		num_overlays = 3;
		input_fifo_size = 80;
		output_fifo_ab_size = 320;
		dsi_ifc_is_supported = false;
		dsi_use_clk_framework = false;
		dev_info(&mcde_dev->dev, "db5500 V2 HW\n");
		break;
	case MCDE_VERSION_4_1_3:
		num_dsilinks = 3;
		num_channels = 4;
		num_overlays = 6;
		dsi_ifc_is_supported = true;
		input_fifo_size = 192;
		output_fifo_ab_size = 640;
		output_fifo_c0c1_size = 160;
		dsi_use_clk_framework = false;
		dev_info(&mcde_dev->dev, "db9540 V1 HW\n");
		break;
	case MCDE_VERSION_3_0_5:
		/* Intentional */
	case MCDE_VERSION_1_0_4:
		/* Intentional */
	default:
		dev_err(&mcde_dev->dev, "Unsupported HW version\n");
		ret = -ENOTSUPP;
		goto unsupported_hw;
		break;
	}

	channels = kzalloc(num_channels * sizeof(struct mcde_chnl_state),
								GFP_KERNEL);
	if (!channels) {
		ret = -ENOMEM;
		goto failed_channels_alloc;
	}

	overlays = kzalloc(num_overlays * sizeof(struct mcde_ovly_state),
								GFP_KERNEL);
	if (!overlays) {
		ret = -ENOMEM;
		goto failed_overlays_alloc;
	}

	dsiio = kzalloc(num_dsilinks * sizeof(*dsiio), GFP_KERNEL);
	if (!dsiio) {
		ret = -ENOMEM;
		goto failed_dsi_alloc;
	}

	for (i = 0; i < num_dsilinks; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1+i);
		if (!res) {
			dev_dbg(&pdev->dev, "No DSI%d io defined\n", i);
			ret = -EINVAL;
			goto failed_get_dsi_io;
		}
		dsiio[i] = ioremap(res->start, res->end - res->start + 1);
		if (!dsiio[i]) {
			dev_dbg(&pdev->dev, "MCDE DSI%d iomap failed\n", i);
			ret = -EINVAL;
			goto failed_map_dsi_io;
		}
		dev_info(&pdev->dev, "MCDE DSI%d iomap: 0x%.8X->0x%.8X\n",
			i, (u32)res->start, (u32)dsiio[i]);
	}

	/* Init MCDE */
	for (i = 0; i < num_overlays; i++)
		overlays[i].idx = i;

	channels[0].ovly0 = &overlays[0];
	channels[0].ovly1 = &overlays[1];
	channels[1].ovly0 = &overlays[2];

	if (pid == MCDE_VERSION_3_0_8) {
		channels[1].ovly1 = &overlays[3];
		channels[2].ovly0 = &overlays[4];
		channels[3].ovly0 = &overlays[5];
	}

	mcde_debugfs_create(&mcde_dev->dev);
	for (i = 0; i < num_channels; i++) {
		channels[i].id = i;

		channels[i].ovly0->chnl = &channels[i];
		if (channels[i].ovly1)
			channels[i].ovly1->chnl = &channels[i];

		init_waitqueue_head(&channels[i].state_waitq);
		init_waitqueue_head(&channels[i].vcmp_waitq);
		init_waitqueue_head(&channels[i].vsync_waitq);
		init_timer(&channels[i].dsi_te_timer);
		channels[i].dsi_te_timer.function =
					dsi_te_timer_function;
		channels[i].dsi_te_timer.data = i;

		mcde_debugfs_channel_create(i, &channels[i]);
		mcde_debugfs_overlay_create(i, 0, channels[i].ovly0);
		if (channels[i].ovly1)
			mcde_debugfs_overlay_create(i, 1, channels[i].ovly1);
	}
	mcde_clk_rate = clk_get_rate(clock_mcde);
	dev_info(&mcde_dev->dev, "MCDE_CLK is %d Hz\n", mcde_clk_rate);

	return 0;

failed_map_dsi_io:
	for (i = 0; i < num_dsilinks; i++) {
		if (dsiio[i])
			iounmap(dsiio[i]);
	}
failed_get_dsi_io:
	kfree(dsiio);
	dsiio = NULL;
failed_dsi_alloc:
	kfree(overlays);
	overlays = NULL;
failed_overlays_alloc:
	kfree(channels);
	channels = NULL;
unsupported_hw:
failed_channels_alloc:
	num_dsilinks = 0;
	num_channels = 0;
	num_overlays = 0;
	return ret;
}

static int __devinit mcde_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct mcde_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_dbg(&pdev->dev, "No platform data\n");
		return -EINVAL;
	}

	mcde_dev = pdev;

	/* Hook up irq */
	mcde_irq = platform_get_irq(pdev, 0);
	if (mcde_irq <= 0) {
		dev_dbg(&pdev->dev, "No irq defined\n");
		ret = -EINVAL;
		goto failed_irq_get;
	}

	/* Map I/O */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_dbg(&pdev->dev, "No MCDE io defined\n");
		ret = -EINVAL;
		goto failed_get_mcde_io;
	}
	mcdeio = ioremap(res->start, res->end - res->start + 1);
	if (!mcdeio) {
		dev_dbg(&pdev->dev, "MCDE iomap failed\n");
		ret = -EINVAL;
		goto failed_map_mcde_io;
	}
	dev_info(&pdev->dev, "MCDE iomap: 0x%.8X->0x%.8X\n",
		(u32)res->start, (u32)mcdeio);

	ret = init_clocks_and_power(pdev);
	if (ret < 0) {
		dev_warn(&pdev->dev, "%s: init_clocks_and_power failed\n"
					, __func__);
		goto failed_init_clocks;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&hw_timeout_work, work_sleep_function);

	ret = probe_hw(pdev);
	if (ret)
		goto failed_probe_hw;

	ret = enable_mcde_hw();
	if (ret)
		goto failed_mcde_enable;

	return 0;

failed_mcde_enable:
failed_probe_hw:
	remove_clocks_and_power(pdev);
failed_init_clocks:
	iounmap(mcdeio);
failed_map_mcde_io:
failed_get_mcde_io:
failed_irq_get:
	return ret;
}

static int __devexit mcde_remove(struct platform_device *pdev)
{
	struct mcde_chnl_state *chnl = &channels[0];

	for (; chnl < &channels[num_channels]; chnl++) {
		if (del_timer(&chnl->dsi_te_timer))
			dev_vdbg(&mcde_dev->dev,
				"%s dsi timer could not be stopped\n"
				, __func__);
	}

	remove_clocks_and_power(pdev);
	return 0;
}

#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)
static int mcde_resume(struct platform_device *pdev)
{
	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	mcde_lock(__func__, __LINE__);

	if (enable_mcde_hw()) {
		mcde_unlock(__func__, __LINE__);
		return -EINVAL;
	}

	mcde_unlock(__func__, __LINE__);

	return 0;
}

static int mcde_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret;

	dev_vdbg(&mcde_dev->dev, "%s\n", __func__);

	mcde_lock(__func__, __LINE__);

	cancel_delayed_work(&hw_timeout_work);

	if (!mcde_is_enabled) {
		mcde_unlock(__func__, __LINE__);
		return 0;
	}
	disable_mcde_hw(true, true);

	mcde_unlock(__func__, __LINE__);

	return ret;
}
#endif

static struct platform_driver mcde_driver = {
	.probe = mcde_probe,
	.remove = mcde_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_PM)
	.suspend = mcde_suspend,
	.resume = mcde_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
	.driver = {
		.name	= "mcde",
	},
};

int __init mcde_init(void)
{
	mutex_init(&mcde_hw_lock);
	return platform_driver_register(&mcde_driver);
}

void mcde_exit(void)
{
	/* REVIEW: shutdown MCDE? */
	platform_driver_unregister(&mcde_driver);
}
