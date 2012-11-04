#This Makefile can be called to do an out-of-kernel build of tha mali kernel module.
#It is not used when mali is built from the kernel build system.
CROSS_COMPILE ?= arm-eabi-

ifeq ($(TARGET_PRODUCT),ste_u5500)
CONFIG_UX500_SOC_DB5500=y
export CONFIG_UX500_SOC_DB5500
endif


PWD:=$(shell pwd)

#if KERNEL_BUILD_DIR parameter is not set, assume there is a symlink to the kernel
KERNEL_BUILD_DIR?=$(PWD)/kernel

# android
ifneq (,$(findstring -android,$(PLATFORM)))
INSTALL_MOD_PATH?=$(ANDROID_OUT_TARGET_PRODUCT_DIRECTORY)/system
endif

# lbp
ifneq (,$(findstring -linux,$(PLATFORM)))
PREFIX?=$(MMROOT)/linux/install/cortexA9-linux-gnu-href_v1
INSTALL_MOD_PATH?=$(PREFIX)
endif

#if still no kernel module path is set, install the modules in a local folder
ifeq (,$(INSTALL_MOD_PATH))
INSTALL_MOD_PATH?=$(PWD)/out
endif

KO_FLAGS=KDIR=$(KERNEL_BUILD_DIR) CROSS_COMPILE=$(CROSS_COMPILE)

.PHONY: all entry check

all: mali-devicedrv install-mali

check:
	@if [ ! -d $(KERNEL_BUILD_DIR) ] ; then echo "Error: provide a path to your linux kernel through KERNEL_BUILD_DIR"; exit 1 ; fi

## Build ###############

mali-devicedrv: check
	$(MAKE) -C driver/src/devicedrv/mali $(KO_FLAGS) all

ump-devicedrv: check
	$(MAKE) -C driver/src/devicedrv/ump $(KO_FLAGS) all

mali_drm-devicedrv: check
	$(MAKE) -C x11/mali_drm/mali $(KO_FLAGS) all

## Clean ###############

clean-mali:
	$(MAKE) clean -C driver/src/devicedrv/mali $(KO_FLAGS)

clean-ump:
	$(MAKE) clean -C driver/src/devicedrv/ump $(KO_FLAGS)

clean-mali_drm:
	$(MAKE) clean -C x11/mali_drm/mali $(KO_FLAGS)

clean: clean-mali

realclean:
	rm -f driver/src/devicedrv/mali/common/*.o
	rm -f driver/src/devicedrv/mali/linux/*.o
	rm -f driver/src/devicedrv/mali/common/.*.cmd
	rm -f driver/src/devicedrv/mali/linux/.*.cmd
	rm -f driver/src/devicedrv/mali/.*.cmd
	rm -f driver/src/devicedrv/mali/*.c

distclean: realclean

## Install #############

install: install-mali

install-mali: mali-devicedrv
	$(MAKE) -C $(KERNEL_BUILD_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNELDIR=$(KERNEL_BUILD_DIR) INSTALL_MOD_PATH=$(INSTALL_MOD_PATH) M=$(PWD)/driver/src/devicedrv/mali modules_install

install-ump: ump-devicedrv
	$(MAKE) -C $(KERNEL_BUILD_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNELDIR=$(KERNEL_BUILD_DIR) INSTALL_MOD_PATH=$(INSTALL_MOD_PATH) M=$(PWD)/driver/src/devicedrv/ump modules_install

install-mali_drm: mali_drm-devicedrv
	$(MAKE) -C $(KERNEL_BUILD_DIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) KERNELDIR=$(KERNEL_BUILD_DIR) INSTALL_MOD_PATH=$(INSTALL_MOD_PATH) M=$(PWD)/x11/mali_drm/mali modules_install


printenv:
	@echo "\033[0;44m"
	@echo "ANDROID_OUT_TARGET_PRODUCT_DIRECTORY: " $(ANDROID_OUT_TARGET_PRODUCT_DIRECTORY)
	@echo "ANDROID_BSP_ROOT: " $(ANDROID_BSP_ROOT)
	@echo "KERNEL_BUILD_DIR: " $(KERNEL_BUILD_DIR)
	@echo "MM_INSTALL_DIR: " $(MM_INSTALL_DIR)
	@echo "PLATFORM: " $(PLATFORM)
	@echo "PREFIX: " $(PREFIX)
	@echo "MMROOT: " $(MMROOT)
	@echo "INSTALL_MOD_PATH: " $(INSTALL_MOD_PATH)
	@echo "FOO: " $(FOO)
	@echo "VARIANT: " $(VARIANT)
	@echo "\033[0m"

