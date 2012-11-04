zreladdr-y	:= 0x00008000
params_phys-y	:= 0x00000100
# munjeni - Lotus ramdisk is at 0x1000000 !
ifdef CONFIG_MACH_U8500_LOTUS
initrd_phys-y	:= 0x01000000
else
initrd_phys-y	:= 0x00800000
endif
