#Android makefile to build kernel as a part of Android Build
ifeq ($(TARGET_USE_ST_ERICSSON_KERNEL),true)

# Give other modules a nice, symbolic name to use as a dependent
# Yes, there are modules that cannot build unless the kernel has
# been built. Typical (only?) example: loadable kernel modules.
.PHONY: build-kernel clean-kernel

PRIVATE_KERNEL_ARGS := -C kernel ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) LOCALVERSION=+

PRIVATE_OUT := $(abspath $(PRODUCT_OUT)/system)

PATH := $(PATH):$(BOOT_PATH)/u-boot/tools:$(abspath $(UBOOT_OUTPUT)/tools)
export PATH

# For compat-wireless gits to compile with kernel
export STERICSSON_WLAN_BUILT_IN=y

# only do this if we are buidling out of tree
ifneq ($(KERNEL_OUTPUT),)
ifneq ($(KERNEL_OUTPUT), $(abspath $(TOP)/kernel))
PRIVATE_KERNEL_ARGS += O=$(KERNEL_OUTPUT)
endif
else
KERNEL_OUTPUT := $(call my-dir)
endif
KERNEL_SOURCE_PATH := $(call my-dir)

include $(CLEAR_VARS)

KERNEL_LIBPATH := $(KERNEL_OUTPUT)
LOCAL_PATH := $(KERNEL_LIBPATH)
LOCAL_SRC_FILES := vmlinux
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)

$(KERNEL_LIBPATH)/$(LOCAL_SRC_FILES): build-kernel

include $(BUILD_PREBUILT)

include $(CLEAR_VARS)

KERNEL_LIBPATH := $(KERNEL_OUTPUT)/arch/arm/boot
LOCAL_PATH := $(KERNEL_LIBPATH)
LOCAL_SRC_FILES := zImage
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)

$(KERNEL_LIBPATH)/$(LOCAL_SRC_FILES): build-kernel

include $(BUILD_PREBUILT)

# Configures, builds and installs the kernel. KERNEL_DEFCONFIG usually
# comes from the BoardConfig.mk file, but can be overridden on the
# command line or by an environment variable.
# If KERNEL_DEFCONFIG is set to 'local', configuration is skipped.
# This is useful if you want to play with your own, custom configuration.

build-kernel:

# only do this if we are buidling out of tree
ifneq ($(KERNEL_OUTPUT),)
ifneq ($(KERNEL_OUTPUT), $(abspath $(TOP)/kernel))
	@mkdir -p $(KERNEL_OUTPUT)
endif
endif

ifeq ($(KERNEL_DEFCONFIG),local)
	@echo Skipping kernel configuration, KERNEL_DEFCONFIG set to local
else
	$(MAKE) $(PRIVATE_KERNEL_ARGS) $(KERNEL_DEFCONFIG)
endif

	$(MAKE) $(PRIVATE_KERNEL_ARGS) zImage
ifeq ($(KERNEL_NO_MODULES),)
	$(MAKE) $(PRIVATE_KERNEL_ARGS) modules
	$(MAKE) $(PRIVATE_KERNEL_ARGS) INSTALL_MOD_PATH:=$(PRIVATE_OUT) modules_install
else
	@echo Skipping building of kernel modules, KERNEL_NO_MODULES set
endif
	cp -u $(KERNEL_OUTPUT)/vmlinux $(PRODUCT_OUT)
	cp -u $(KERNEL_LIBPATH)/zImage $(PRODUCT_OUT)


# Configures and runs menuconfig on the kernel based on
# KERNEL_DEFCONFIG given on commandline or in BoardConfig.mk.
# The build after running menuconfig must be run with
# KERNEL_DEFCONFIG=local to not override the configuration modification done.

menuconfig-kernel:
# only do this if we are buidling out of tree
ifneq ($(KERNEL_OUTPUT),)
ifneq ($(KERNEL_OUTPUT), $(abspath $(TOP)/kernel))
	@mkdir -p $(KERNEL_OUTPUT)
endif
endif

	$(MAKE) $(PRIVATE_KERNEL_ARGS) $(KERNEL_DEFCONFIG)
	$(MAKE) $(PRIVATE_KERNEL_ARGS) menuconfig

clean clobber : clean-kernel

clean-kernel:
	$(MAKE) $(PRIVATE_KERNEL_ARGS) clean

# SEMC: make kernelconfig target
KERNEL_OUT_CONFIG := $(KERNEL_OUTPUT)/.config
KERNEL_SOURCE_CONFIG := $(KERNEL_SOURCE_PATH)/arch/arm/configs/$(KERNEL_DEFCONFIG)

# Select choosen defconfig
install_defconfig_always:
$(KERNEL_OUT_CONFIG): install_defconfig_always
	@mkdir -p $(KERNEL_OUTPUT)
	$(hide) $(MAKE) -C $(KERNEL_SOURCE_PATH) O=$(KERNEL_OUTPUT)\
            ARCH=arm CROSS_COMPILE=arm-eabi- $(KERNEL_DEFCONFIG)

# Install selected defconfig, run menuconfig and then copy back
.PHONY: kernelconfig
kernelconfig: $(KERNEL_OUT_CONFIG)
	$(hide) $(MAKE) -C $(KERNEL_SOURCE_PATH) O=$(KERNEL_OUTPUT) \
	             ARCH=arm CROSS_COMPILE=arm-eabi- menuconfig
	$(hide) $(MAKE) -C $(KERNEL_SOURCE_PATH) O=$(KERNEL_OUTPUT) \
	             ARCH=arm KCONFIG_NOTIMESTAMP=true savedefconfig
	$(hide) mv $(KERNEL_OUTPUT)/defconfig $(KERNEL_SOURCE_CONFIG)


#Sparse changed files
.PHONY: kernel-sparse-changed
kernel-sparse-changed: $(KERNEL_OUT_CONFIG)
	$(hide) $(MAKE) -C $(KERNEL_SOURCE_PATH) O=$(KERNEL_OUTPUT) \
	             ARCH=arm CROSS_COMPILE=arm-eabi- C=1

#Sparse all files
.PHONY: kernel-sparse-all
kernel-sparse-all: $(KERNEL_OUT_CONFIG)
	$(hide) $(MAKE) -C $(KERNEL_SOURCE_PATH) O=$(KERNEL_OUTPUT) \
	             ARCH=arm CROSS_COMPILE=arm-eabi- C=2

#
# Rules for packing kernel into elf and sin
#
TARGET_PREBUILT_KERNEL := $(KERNEL_OUTPUT)/arch/arm/boot/zImage

$(PRODUCT_OUT)/cmdline.txt: device/semc/riogrande/BoardConfig.mk
	$(hide) echo -n '$(BOARD_KERNEL_CMDLINE)' > $@

$(PRODUCT_OUT)/kernel-unsigned.elf: $(TARGET_PREBUILT_KERNEL) $(PRODUCT_OUT)/ramdisk.img $(PRODUCT_OUT)/cmdline.txt | sin-tools
	$(hide) $(HOST_OUT_EXECUTABLES)/mkelf.py -o $@ \
		$(TARGET_PREBUILT_KERNEL)@$(BOARD_KERNEL_ADDR) \
		$(PRODUCT_OUT)/ramdisk.img@$(BOARD_RAMDISK_ADDR),ramdisk \
		$(PRODUCT_OUT)/cmdline.txt@cmdline

$(PRODUCT_OUT)/kernel-signed.elf: $(PRODUCT_OUT)/kernel-unsigned.elf $(PRODUCT_PARTITION_CONFIG) | sin-tools
	$(hide) $(HOST_OUT_EXECUTABLES)/semcsc.py -c $(PRODUCT_PARTITION_CONFIG) -p Kernel -t internal -i $< -o $@

$(PRODUCT_OUT)/kernel.si_: $(PRODUCT_OUT)/kernel-signed.elf $(PRODUCT_PARTITION_CONFIG) | sin-tools
	$(hide) $(HOST_OUT_EXECUTABLES)/mksin.py -c $(PRODUCT_PARTITION_CONFIG) -p Kernel -i $< -o $@

$(PRODUCT_OUT)/kernel.sin: $(PRODUCT_OUT)/kernel.si_ $(PRODUCT_PARTITION_CONFIG) | sin-tools
	@echo target SIN: $(notdir $@)
	$(hide) $(HOST_OUT_EXECUTABLES)/semcsc.py -c $(PRODUCT_PARTITION_CONFIG) -p Kernel -t external -i $< -o $@

#
# Add kernel to system wide PHONY target sin
#
.PHONY: sin

sin: $(PRODUCT_OUT)/kernel.sin

endif
