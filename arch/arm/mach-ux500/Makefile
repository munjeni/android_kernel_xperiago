#
# Makefile for the linux kernel, UX500 machine.
#

obj-y				:= clock.o cpu.o devices.o dcache.o \
				   devices-common.o id.o pins.o \
				   usb.o reboot_reasons.o timer.o \
				   uart-db8500.o clock-debug.o
obj-y				+= pm/ test/



ifeq ($(CONFIG_UX500_SOC_DB5500), y)
obj-$(CONFIG_UX500_SOC_DBX500)	+= cpu-db5500.o dma-db5500.o \
				   devices-db5500.o clock-db5500.o
obj-$(CONFIG_TOUCHSCREEN_CYTTSP_SPI)    += board-u5500-cyttsp.o
board-mcde-objs			+= board-u5500-mcde.o
endif
ifeq ($(CONFIG_UX500_SOC_DB8500), y)

obj-$(CONFIG_UX500_SOC_DBX500)	+= cpu-db8500.o devices-db8500.o \
				   clock-db8500.o dma-db8500.o cpu-db9500.o
board-mcde-objs			+= board-rio-grande-mcde.o
endif
obj-$(CONFIG_MACH_U8500)	+= board-mop500.o board-mop500-sdi.o \
				board-mop500-regulators.o \
				board-mop500-uib.o board-mop500-stuib.o \
				board-mop500-u8500uib.o board-mop500-pins.o \
				board-mop500-msp.o board-mop500-bm.o \
				board-mop500-mmio.o board-mop500-wlan.o \
				board-mop500-cyttsp.o \
				board-mop500-vibra.o

obj-$(CONFIG_MACH_U8500_KUMQUAT)  += board-rio-grande.o board-rio-grande-sdi.o \
				board-mop500-regulators.o \
				gpio-kumquat.o \
				board-mop500-msp.o board-mop500-bm.o \
				board-rio-grande-mmio.o board-mop500-wlan.o \
				simple_remote-rio-grande.o \
				input-kumquat.o leds-kumquat.o \
				display-rio-grande.o sim_detect_semc.o \
				charger-kumquat.o \
				board-mop500-vibra.o

obj-$(CONFIG_MACH_U8500_LOTUS)  += board-rio-grande.o board-rio-grande-sdi.o \
				board-mop500-regulators.o \
				gpio-lotus.o \
				board-mop500-msp.o board-mop500-bm.o \
				board-rio-grande-mmio.o board-mop500-wlan.o \
				simple_remote-rio-grande.o \
				input-lotus.o leds-lotus.o \
				display-rio-grande_r2.o sim_detect_semc.o \
				charger-lotus.o \
				board-mop500-vibra.o

obj-$(CONFIG_MACH_U8500_NYPON)  += board-rio-grande.o board-rio-grande-sdi.o \
				board-mop500-regulators.o \
				gpio-nypon.o \
				board-mop500-msp.o board-mop500-bm.o \
				board-rio-grande-mmio.o board-mop500-wlan.o \
				simple_remote-rio-grande_r2.o \
				input-nypon.o leds-nypon.o \
				display-nypon.o sim_detect_semc.o \
				charger-nypon.o \
				board-mop500-vibra-nypon.o

obj-$(CONFIG_MACH_U8500_PDP)  += board-rio-grande.o board-rio-grande-sdi.o \
				board-mop500-regulators.o charger-rio-grande.o \
				gpio-pdp.o \
				board-mop500-msp.o board-mop500-bm.o \
				board-mop500-wlan.o \
				board-mop500-vibra.o

obj-$(CONFIG_MACH_U8500_PDP_R2)  += board-rio-grande.o board-rio-grande-sdi.o \
				board-mop500-regulators.o charger-rio-grande.o \
				gpio-pdp_r2.o \
				board-mop500-msp.o board-mop500-bm.o \
				board-rio-grande-mmio.o board-mop500-wlan.o \
				simple_remote-rio-grande_r2.o \
				input-rio-grande-pdp.o leds-rio-grande-pdp.o \
				display-rio-grande_r2.o \
				board-mop500-vibra.o

obj-$(CONFIG_MACH_U8500_PEPPER)  += board-rio-grande.o board-rio-grande-sdi.o \
				board-mop500-regulators.o \
				gpio-pepper.o \
				board-mop500-msp.o board-mop500-bm.o \
				board-rio-grande-mmio.o board-mop500-wlan.o \
				simple_remote-rio-grande.o \
				input-pepper.o leds-pepper.o \
				display-rio-grande.o sim_detect_semc.o \
				board-mop500-vibra-pepper.o \
				charger-pepper.o

obj-$(CONFIG_MACH_U5500)	+= board-u5500.o board-u5500-sdi.o \
				   board-u5500-regulators.o \
				   board-u5500-pins.o \
				   board-u5500-wlan.o
obj-$(CONFIG_MCDE)		+= board-mcde.o
obj-$(CONFIG_U5500_MMIO)	+= board-u5500-mmio.o
obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o
obj-$(CONFIG_LOCAL_TIMERS)	+= localtimer.o
obj-$(CONFIG_U5500_MODEM_IRQ)	+= modem-irq-db5500.o
obj-$(CONFIG_TEE_UX500) 	+= tee_ux500.o product.o
obj-$(CONFIG_TEE_SVP)		+= tee_service_svp.o
obj-$(CONFIG_TEE_SVP)		+= tee_ta_start_modem_svp.o
obj-$(CONFIG_DB8500_MLOADER) 	+= mloader-db8500.o open_modem_shared_memory.o
obj-$(CONFIG_U5500_MLOADER)	+= mloader-db5500.o
obj-$(CONFIG_U8500_REGULATOR_DEBUG)	+= virt-regulator-u8500.o
obj-$(CONFIG_UX500_DEBUG_HWREG) 	+= hwreg.o
obj-$(CONFIG_HWMEM)			+= hwmem-int.o
obj-$(CONFIG_UX500_L2X0_PREFETCH_CTRL)	+= l2x0-prefetch.o
obj-$(CONFIG_AB5500_BM)			+= board-u5500-bm.o
obj-$(CONFIG_DBX500_PRCMU_DEBUG)	+= prcmu-debug.o
obj-$(CONFIG_UX500_DB_DUMP)		+= dbx500_dump.o
obj-$(CONFIG_UX500_DEBUG_LAST_IO)	+= debug-last-io.o
obj-$(CONFIG_RTC_HCTOHC)		+= hctohc.o
ifeq ($(CONFIG_UX500_SOC_DB8500), y)
obj-$(CONFIG_STM_TRACE)			+= board-mop500-stm.o
endif
obj-$(CONFIG_INPUT_AB8505_MICRO_USB_DETECT)	+= ssg_micro_usb_iddet.o abx_micro_usb_iddet.o

obj-$(CONFIG_SIMPLE_REMOTE_PLATFORM)    += simple_remote_ux500_pf.o

obj-$(CONFIG_SONY_SSM)			+= sony_ssm.o
