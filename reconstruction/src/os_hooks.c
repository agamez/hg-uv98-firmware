/* uC/OS-II application hooks — empty implementations.
 *
 * The recovered firmware's hook usage is unknown (the original's hook
 * bodies live in application code we have not reconstructed); the kernel
 * calls these unconditionally, so the reconstruction provides no-op
 * versions. If a future session identifies real hook behaviour (e.g. the
 * statistics task wiring), these become the integration points. */
#include "ucos_ii.h"
#include "cpu_core.h"

void OSTaskCreateHook(OS_TCB *ptcb)  { (void)ptcb; }
void OSTaskDelHook(OS_TCB *ptcb)     { (void)ptcb; }
void OSTaskReturnHook(OS_TCB *ptcb)  { (void)ptcb; }
void OSTaskIdleHook(void)            { }
void OSTaskStatHook(void)            { }
void OSTaskSwHook(void)              { }
void OSTCBInitHook(OS_TCB *ptcb)     { (void)ptcb; }
void OSTimeTickHook(void)            { }

/* CPU_TS_TmrRd is referenced unconditionally by cpu_core.c's interrupt-
 * disable measurement helpers even when CPU_CFG_INT_DIS_MEAS_EN is
 * DEF_DISABLED (the measurement is never exercised at runtime). The DWT
 * cycle counter is not reconstructed in this effort, so the stub returns
 * zero. CPU_TS is CPU_TS32 (CPU_INT32U) per cpu_core.h. */
CPU_TS_TMR CPU_TS_TmrRd(void)        { return (CPU_TS_TMR)0; }
