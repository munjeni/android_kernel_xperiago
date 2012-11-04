mcde-objs		+= mcde_mod.o
mcde-objs		+= mcde_hw.o
mcde-objs		+= mcde_dss.o
mcde-objs		+= mcde_display.o
mcde-objs		+= mcde_bus.o
ifdef CONFIG_FB_MCDE
mcde-objs		+= mcde_fb.o
endif
mcde-objs		+= mcde_debugfs.o
obj-$(CONFIG_MCDE)	+= mcde.o

obj-$(CONFIG_MCDE_DISPLAY_DSI)			+= dsilink.o
obj-$(CONFIG_MCDE_DISPLAY_DSI)			+= dsilink_v2.o
obj-$(CONFIG_MCDE_DISPLAY_DSI)			+= dsilink_v3.o
obj-$(CONFIG_MCDE_DISPLAY_GENERIC_DSI)		+= display-generic_dsi.o
obj-$(CONFIG_MCDE_DISPLAY_SAMSUNG_S6D16D0)	+= display-samsung_s6d16d0.o
obj-$(CONFIG_MCDE_DISPLAY_SONY_ACX424AKP_DSI)   += display-sony_acx424akp_dsi.o
obj-$(CONFIG_MCDE_DISPLAY_VUIB500_DPI)		+= display-vuib500-dpi.o
obj-$(CONFIG_MCDE_DISPLAY_AV8100)		+= display-av8100.o
obj-$(CONFIG_DISPLAY_FICTIVE)			+= display-fictive.o

obj-$(CONFIG_MCDE_DISPLAY_LOGO)					+= logo.o

obj-$(CONFIG_MCDE_DISPLAY_PANEL_DSI)				+= display-panel_dsi.o
obj-$(CONFIG_MCDE_DISPLAY_NT35560_DSI_SONY_ACX424AKM)		+= display-nt35560_dsi_sony_acx424akm.o
obj-$(CONFIG_MCDE_DISPLAY_NT35560_DSI_HITACHI_DX09D09VM)	+= display-nt35560_dsi_hitachi_dx09d09vm.o
obj-$(CONFIG_MCDE_DISPLAY_R61408_DSI_SEIKO_RB443)		+= display-r61408_dsi_seiko_rb443.o
obj-$(CONFIG_MCDE_DISPLAY_R61408_DSI_SAMSUNG_LMS347TF01)		+= display-r61408_dsi_samsung_lms347tf01.o
obj-$(CONFIG_MCDE_DISPLAY_R61529_DSI_SEIKO_RJ248)		+= display-r61529_dsi_seiko_rj248.o
obj-$(CONFIG_MCDE_DISPLAY_R61529_DSI_HITACHI_TX09D115VM)	+= display-r61529_dsi_hitachi_tx09d115vm.o
obj-$(CONFIG_MCDE_DISPLAY_OTM9601_DSI_SONY_ACX438AKM)		+= display-otm9601_dsi_sony_acx438akm.o
obj-$(CONFIG_MCDE_DISPLAY_R61408_DSI_HITACHI_TX09D113)		+= display-r61408_dsi_hitachi_tx09d113.o
obj-$(CONFIG_MCDE_DISPLAY_R61408_DSI_SONY_L5F31178)		+= display-r61408_dsi_sony_l5f31178.o

ifeq ($(CONFIG_DEBUG_FS),y)
	mcde-objs						+= display-panel_dsi_debugfs.o
endif

ifdef CONFIG_FB_MCDE_DEBUG
EXTRA_CFLAGS += -DDEBUG
endif
ifdef CONFIG_FB_MCDE_VDEBUG
EXTRA_CFLAGS += -DVERBOSE_DEBUG
endif
