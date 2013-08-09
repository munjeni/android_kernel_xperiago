/*
 * relocate_kernel.S - put the kernel image in place to boot
 */

#include <asm/kexec.h>

#ifdef CONFIG_KEXEC_HARDBOOT
#include <asm/memory.h>
#endif

	.globl relocate_new_kernel
relocate_new_kernel:

	ldr	r0,kexec_indirection_page
	ldr	r1,kexec_start_address

	/*
	 * If there is no indirection page (we are doing crashdumps)
	 * skip any relocation.
	 */
	cmp	r0, #0
	beq	2f

0:	/* top, read another word for the indirection page */
	ldr	r3, [r0],#4

	/* Is it a destination page. Put destination address to r4 */
	tst	r3,#1,0
	beq	1f
	bic	r4,r3,#1
	b	0b
1:
	/* Is it an indirection page */
	tst	r3,#2,0
	beq	1f
	bic	r0,r3,#2
	b	0b
1:

	/* are we done ? */
	tst	r3,#4,0
	beq	1f
	b	2f

1:
	/* is it source ? */
	tst	r3,#8,0
	beq	0b
	bic r3,r3,#8
	mov r6,#1024
9:
	ldr r5,[r3],#4
	str r5,[r4],#4
	subs r6,r6,#1
	bne 9b
	b 0b

2:
#ifdef CONFIG_KEXEC_HARDBOOT
	ldr	r0, kexec_hardboot
	teq	r0, #0
	bne	hardboot
#endif

	/* Jump to relocated kernel */
	mov lr,r1
	mov r0,#0
	ldr r1,kexec_mach_type
	ldr r2,kexec_boot_atags
	mov pc,lr

#ifdef CONFIG_KEXEC_HARDBOOT
hardboot:
	/* Stash boot arguments in hardboot page:
	 *  0: KEXEC_HB_PAGE_MAGIC
	 *  4: kexec_start_address
	 *  8: kexec_mach_type
	 * 12: kexec_boot_atags
	 */
	ldr	r0, =KEXEC_HB_PAGE_ADDR
	str	r1, [r0, #4]
	ldr	r1, kexec_mach_type
	str	r1, [r0, #8]
	ldr	r1, kexec_boot_atags
	str	r1, [r0, #12]
	ldr	r1, =KEXEC_HB_PAGE_MAGIC
	str	r1, [r0]

/*
 * Munjeni @ XDA 2013:
 * Our reboot function is based on db8500-prcmu.c and cpu.c
 * Used db8500_prcmu_system_reset function with combination with ux500_restart function.
 * Created new reboot function based on these two by this way:
 *
 * void reboot(u16 reset_code)
 * {
 * 	//0x5501 is SW_RESET_NORMAL reset reason
 * 	local_irq_disable();
 * 	local_fiq_disable();
 * 	writew(0x5501, (tcdm_base + PRCM_SW_RST_REASON));
 * 	writel(1, PRCM_APE_SOFTRST);
 * 	mdelay(1000);
 *
 * 	while (1)
 * 		;
 * }
 *
 * Hope thats ok???
 */
#if defined(CONFIG_MACH_U8500_LOTUS) || defined(CONFIG_MACH_U8500_PEPPER)
	MOV	R12, SP
	STMFD	SP!, {R4,R5,R11,R12,LR,PC}
	SUB	R11, R12, #4
	STMFD	SP!, {LR}

loc_5124:
	BL	__gnu_mcount_nc
	CPSID	I
	CPSID	F
	LDR	R4, =outer_cache
	LDR	R3, [R4,#0x18]
	CMP	R3, #0
	BEQ	loc_5148
	BLX	R3

loc_5148:
	LDR	R3, =tcdm_base_bkp
	MOVW	R2, #0x5501
	LDR	R3, [R3]
	ADD	R3, R3, #0xFF0
	STRH	R2, [R3,#8]
	LDR	R3, [R4,#0x18]
	CMP	R3, #0
	BEQ	loc_5170
	BLX	R3

loc_5170:
	LDR	R3, =_PRCMU_BASE
	MOVW	R4, #0x3E7
	MOV	R2, #1
	LDR	R3, [R3]
	STR	R2, [R3,#0x228]

loc_5184:
	MOVW	R0, #0x65B0
	SUB	R4, R4, #1
	MOVT	R0, #0x666
	BL	__const_udelay
	CMN	R4, #1
	BNE	loc_5184

loc_519C:
	B	loc_519C
#else
#error "No reboot method defined for hardboot."
#endif

	.ltorg
#endif
	.align

	.globl kexec_start_address
kexec_start_address:
	.long	0x0

	.globl kexec_indirection_page
kexec_indirection_page:
	.long	0x0

	.globl kexec_mach_type
kexec_mach_type:
	.long	0x0

	/* phy addr of the atags for the new kernel */
	.globl kexec_boot_atags
kexec_boot_atags:
	.long	0x0

#ifdef CONFIG_KEXEC_HARDBOOT
	.globl kexec_hardboot
kexec_hardboot:
	.long	0x0
#endif

relocate_new_kernel_end:

	.globl relocate_new_kernel_size
relocate_new_kernel_size:
	.long relocate_new_kernel_end - relocate_new_kernel


