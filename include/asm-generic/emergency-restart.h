#ifndef _ASM_GENERIC_EMERGENCY_RESTART_H
#define _ASM_GENERIC_EMERGENCY_RESTART_H

static inline void machine_emergency_restart(void)
{
#ifndef CONFIG_EMGNCY_EQ_PANIC
	machine_restart(NULL);
#else
	machine_restart("panic");
#endif
}

#endif /* _ASM_GENERIC_EMERGENCY_RESTART_H */
