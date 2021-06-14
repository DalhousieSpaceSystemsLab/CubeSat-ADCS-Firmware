#ifndef __RINGBUF_H__
#define __RINGBUF_H__
#ifdef __cplusplus
/* clang-format off */
extern "C"
{
/* clang-format on */
#endif /* Start C linkage */

#include "bufferlib.h"

#if defined(BUFFERLIB_THREAD_SAFE)
#define RINGBUF_THREAD_SAFE
#endif /* #if defined(BUFFERLIB_THREAD_SAFE) */

/**********************************************************
 ** @CONFIGURABLE:                                       **
 ** uncomment this to allow overwrite when input wraps   **
 **********************************************************
 */
//          #define RINGBUF_INPUT_OVERRUN

#include <stdint.h>


#include "fat_pointer.h"

/**
 * @brief ringbuf constructor
 *
 * @param size size of ring buffer
 * @return ringbuf_t buffer handle
 */
buffer_instance_handle ringbuf_ctor_new(unsigned int size);


/**
 * @brief ringbuf destructor
 *
 * @param ringbuf buffer handle
 */
void ringbuf_dtor(buffer_instance_handle this);


/**
 * @brief threadsafe read of byte from ring buffer
 *
 * @param ringbuf buffer handle
 * @return data in buffer : address of current byte in ring buffer
 *         no data in buffer : NULL
 *
 * @note reading from the ring buffer advances the outptr
 * @note thread safe
 */
int ringbuf_read_next(buffer_instance_handle this);


/**
 * @brief thread safe write to ring buffer
 *
 * @param ringbuf buffer handle
 * @param byte byte to write
 *
 * @note thread safe
 * @note Will overwrite data
 */
int ringbuf_write_next(buffer_instance_handle this, char byte);


/**
 * @brief get the buffer size
 *
 * @param this buffer handle
 * @return unsigned int size of buffer
 */
unsigned int ringbuf_size(buffer_instance_handle this);

#ifdef __cplusplus
/* clang-format off */
}
/* clang-format on */
#endif /* End C linkage */
#endif /* __RINGBUF_H__ */
