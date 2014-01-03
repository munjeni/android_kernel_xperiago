kexec_load-objs := \
	kexec.o \
	machine_kexec.o \
	core.o \
	relocate_kernel.o \
	sys.o \
	copypage-v6.o \
	driver_sys.o \
	../../../../arch/arm/mm/idmap.o \
	../../../../arch/arm/mm/proc-v7.o \
	../../../../arch/arm/mm/tlb-v7.o \
	../../../../arch/arm/mm/cache-v7.o \
	../../../../arch/arm/mm/proc-macros.o \
	../../../../arch/arm/mm/abort-ev7.o \
	../../../../arch/arm/mm/pabort-v7.o \
	../../../../arch/arm/kernel/sleep.o

obj-$(CONFIG_KEXEC_LOAD) += kexec_load.o
