#ifndef __ADS7841E_H__
#define __ADS7841E_H__
#ifdef __cplusplus
/* clang-format off */
extern "C"
{
/* clang-format on */
#endif /* Start C linkage */

#include <stdint.h>
#include <limits.h>
#include <stdbool.h>

/* API retvals */
#define ADS7841_CONV_STATUS_ERROR (UINT16_MAX)
#define ADS7841_CONV_STATUS_BUSY (UINT16_MAX - 1)

typedef enum
{
    ADS7841_CHANNEL_0,
    ADS7841_CHANNEL_1,
    ADS7841_CHANNEL_2,
    ADS7841_CHANNEL_3,
} ADS7841_CHANNEL_t;

typedef enum
{
    ADS7841_CONVMODE_8,
    ADS7841_CONVMODE_12,
} ADS7841_CONVMODE_t;

typedef enum
{
    ADS7841_PWRMODE_lowpwr,
    ADS7841_PWRMODE_stayOn,
} ADS7841_PWRMODE_t;

/**
 * @brief Initialize the ADS7841 driver API
 *
 * @param ena_func function to drive the CS pin on ADS7841 low
 * @param dis_func function to drive the CS pin on ADS7841 high
 * @param mode one of ADS7841_PWRMODE_t. Determines device behaviour when not
 * being used.
 * @param conv_type one of ADS7841_CONVMODE_t. Set to 12 or 8 bit conversions.
 */
void ADS7841_driver_init(void (*ena_func)(void), void (*dis_func)(void),
                         ADS7841_PWRMODE_t mode, ADS7841_CONVMODE_t conv_type);


/**
 * @brief Deinitialize the ADS7841 driver API
 */
void ADS7841_driver_deinit(void);


/**
 * @brief Perform a digital->analog conversion with ADS7841 for a given
 * channel.
 *
 * @param ch the ADS7841 channel to perform a conversion on
 * @return uint16_t the digitized analog value (averaged by driver API)
 */
uint16_t ADS7841_measure_channel(ADS7841_CHANNEL_t ch);


#ifdef __cplusplus
/* clang-format off */
}
/* clang-format on */
#endif /* End C linkage */
#endif /* __ADS7841E_H__ */
