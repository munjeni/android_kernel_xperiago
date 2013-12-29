MALI_SUBFOLDER := mali400ko/driver/src/devicedrv/mali
MALI_FOLDER := $(srctree)/$(src)/$(MALI_SUBFOLDER)
ifeq ($(shell [ -d $(MALI_FOLDER) ] && echo "OK"), OK)
obj-$(CONFIG_GPU_MALI)  += $(MALI_SUBFOLDER)/
else
$(warning WARNING: mali: Could not find $(MALI_FOLDER) - mali device driver will not be built)
obj-n += ./
endif

UMP_SUBFOLDER := mali400ko/driver/src/devicedrv/ump
UMP_FOLDER := $(srctree)/$(src)/$(UMP_SUBFOLDER)
ifeq ($(shell [ -d $(UMP_FOLDER) ] && echo "OK"), OK)
obj-$(CONFIG_GPU_MALI)  += $(UMP_SUBFOLDER)/
else
$(warning WARNING: mali: Could not find $(UMP_FOLDER) - mali ump device driver will not be built)
obj-n += ./
endif
