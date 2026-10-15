/* uC/OS-II configuration for the HG-UV98 reconstruction.
 *
 * Base: Cfg/Template/os_cfg.h (Apache-2.0), adjusted to the recovered
 * application shape: 7 application tasks + idle, mailboxes enabled, all
 * other kernel objects off, 1 ms tick (session 7 evidence: OSTimeDly(500)
 * used for a ~500 ms poll cadence). */
#ifndef OS_CFG_H
#define OS_CFG_H

                                       /* ---------------------- MISCELLANEOUS ----------------------- */
#define OS_APP_HOOKS_EN           0u
#define OS_ARG_CHK_EN             0u
#define OS_CPU_HOOKS_EN           0u   /* hooks wired via vector table / port file     */

#define OS_DEBUG_EN               0u

#define OS_EVENT_MULTI_EN         0u
#define OS_EVENT_NAME_EN          1u   /* original names tasks/events (0x08024A6C)     */

#define OS_LOWEST_PRIO           63u

#define OS_MAX_EVENTS             8u   /* mailboxes + margins                          */
#define OS_MAX_FLAGS              0u
#define OS_MAX_MEM_PART           0u
#define OS_MAX_QS                 0u
#define OS_MAX_TASKS             10u   /* 7 app tasks + idle + margin                  */

#define OS_SCHED_LOCK_EN          1u

#define OS_TICK_STEP_EN           0u
#define OS_TICKS_PER_SEC        1000u   /* 1 ms tick (research log session 7)           */

#define OS_TLS_TBL_SIZE           0u

                                       /* --------------------- TASK STACK SIZE ---------------------- */
#define OS_TASK_TMR_STK_SIZE    128u

                                       /* --------------------- TASK MANAGEMENT ---------------------- */
#define OS_TASK_CHANGE_PRIO_EN    0u
#define OS_TASK_CREATE_EN         0u   /* original uses OSTaskCreateExt only           */
#define OS_TASK_CREATE_EXT_EN     1u
#define OS_TASK_DEL_EN            0u
#define OS_TASK_IDLE_STK_SIZE   128u
#define OS_TASK_PROFILE_EN        0u
#define OS_TASK_QUERY_EN          0u
#define OS_TASK_REG_TBL_SIZE      0u
#define OS_TASK_STAT_EN           0u   /* original has a custom stats-style task       */
#define OS_TASK_STAT_STK_CHK_EN   0u
#define OS_TASK_STAT_STK_SIZE   128u
#define OS_TASK_SUSPEND_EN        0u
#define OS_TASK_SW_HOOK_EN        0u
#define OS_TASK_NAME_EN           1u

                                       /* ----------------------- EVENT FLAGS ------------------------ */
#define OS_FLAG_EN                0u
#define OS_FLAG_ACCEPT_EN         0u
#define OS_FLAG_DEL_EN            0u
#define OS_FLAG_NAME_EN           0u
#define OS_FLAG_QUERY_EN          0u
#define OS_FLAG_WAIT_CLR_EN       0u
#define OS_FLAGS_NBITS           16u

                                       /* -------------------- MESSAGE MAILBOXES --------------------- */
#define OS_MBOX_EN                1u
#define OS_MBOX_ACCEPT_EN         0u
#define OS_MBOX_DEL_EN            0u
#define OS_MBOX_PEND_ABORT_EN     0u
#define OS_MBOX_POST_EN           1u
#define OS_MBOX_POST_OPT_EN       0u
#define OS_MBOX_QUERY_EN          0u

                                       /* -------------------- MEMORY MANAGEMENT --------------------- */
#define OS_MEM_EN                 0u
#define OS_MEM_NAME_EN            0u
#define OS_MEM_QUERY_EN           0u

                                       /* ------------------------- MUTUAL EXCLUSION ----------------- */
#define OS_MUTEX_EN               0u
#define OS_MUTEX_ACCEPT_EN        0u
#define OS_MUTEX_DEL_EN           0u
#define OS_MUTEX_QUERY_EN         0u

                                       /* --------------------------- MESSAGE QUEUES ----------------- */
#define OS_Q_EN                   0u
#define OS_Q_ACCEPT_EN            0u
#define OS_Q_DEL_EN               0u
#define OS_Q_FLUSH_EN             0u
#define OS_Q_PEND_ABORT_EN        0u
#define OS_Q_POST_EN              0u
#define OS_Q_POST_FRONT_EN        0u
#define OS_Q_POST_OPT_EN          0u
#define OS_Q_QUERY_EN             0u

                                       /* --------------------------- SEMAPHORES --------------------- */
#define OS_SEM_EN                 0u
#define OS_SEM_ACCEPT_EN          0u
#define OS_SEM_DEL_EN             0u
#define OS_SEM_PEND_ABORT_EN      0u
#define OS_SEM_QUERY_EN           0u
#define OS_SEM_SET_EN             0u

                                       /* --------------------- TIME MANAGEMENT ---------------------- */
#define OS_TIME_DLY_HMSM_EN       1u
#define OS_TIME_DLY_RESUME_EN     0u
#define OS_TIME_GET_SET_EN        0u
#define OS_TIME_TICK_HOOK_EN      0u

                                       /* --------------------------- TIMERS ------------------------- */
#define OS_TMR_EN                 0u
#define OS_TMR_CFG_MAX            0u
#define OS_TMR_CFG_NAME_EN        0u
#define OS_TMR_CFG_TICKS_PER_SEC  1u
#define OS_TMR_CFG_WHEEL_SIZE     1u

                                       /* ------------------------ TRACE / RECORDER ------------------ */
/* OS_TRACE_EN intentionally undefined: no trace recorder. */

#endif /* OS_CFG_H */
