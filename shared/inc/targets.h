#ifndef __TARGETS_H__
#define __TARGETS_H__
#ifdef __cplusplus
/* clang-format off */
extern "C"
{
/* clang-format on */
#endif /* Start C linkage */


/* log_trace */
#if defined(TARGET_MCU)
#define log_trace(fmt, ...) ; /* do nothing (this is just so it compiles) */
#else
#define log_trace(fmt, ...)                                                    \
    printf("[%s : %d in %s]\n>>> " fmt "\n", __FILE__, __LINE__, __func__,     \
           ##__VA_ARGS__)
#endif /* #if defined(TARGET_MCU) */

/* CONFIG_ASSERT */
#if defined(TARGET_MCU)
#if defined(DEBUG) && !defined(NDEBUG)
#define CONFIG_ASSERT(x)                                                       \
    do                                                                         \
    {                                                                          \
        if (!(x))                                                              \
        {                                                                      \
            while (1)                                                          \
            {                                                                  \
            }                                                                  \
        }                                                                      \
    } while (0)
#else
#define CONFIG_ASSERT(x) ; /* literally let compiler elide the check */
#endif                     /* #if defined(DEBUG)  && !defined(NDEBUG) */
#else                      /* TARGET_MCU NOT DEFINED */
#if defined(DEBUG) && !defined(NDEBUG)
#include <assert.h>
#define CONFIG_ASSERT(x) assert((x))
#else
/* CAST AS VOID TO PREVENT COMPILER WARNING OF UNUSED VAR */
#define CONFIG_ASSERT(x) (void)(x);
#endif /* #if defined(DEBUG)  && !defined(NDEBUG) */

#endif /* #if defined(TARGET_MCU) */

#ifdef __cplusplus
/* clang-format off */
}
/* clang-format on */
#endif /* End C linkage */
#endif /* __TARGETS_H__ */
