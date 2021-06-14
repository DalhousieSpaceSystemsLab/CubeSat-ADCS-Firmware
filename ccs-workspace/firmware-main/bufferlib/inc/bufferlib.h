#ifndef __BUFFERLIB_H__
#define __BUFFERLIB_H__
#ifdef __cplusplus
/* clang-format off */
extern "C"
{
/* clang-format on */
#endif /* Start C linkage */

#include <stdlib.h>
#include <stdint.h>
#include <limits.h>


/**********************************************************
 ** @CONFIGURABLE:
 ** uncomment this to enable thread safety
 **********************************************************
 */
//          #define BUFFERLIB_THREAD_SAFE


/* read and write functions return the value of the byte just read/written */
/* thus, failure values need to be something that a byte cannot represent  */
#define BUFFERLIB_READ_FAILURE ((int)(UINT8_MAX + 1))
#define BUFFERLIB_WRITE_FAILURE ((int)(UINT8_MAX + 1))


typedef void *buffer_instance_handle;
typedef struct buffer_handle
{
    buffer_instance_handle this;
    int (*read_next)(buffer_instance_handle);
    int (*write_next)(buffer_instance_handle, char);
    unsigned int (*size)(buffer_instance_handle);
    void (*delete)(buffer_instance_handle);
} buffer_handle;


/**
 * @brief Construct a bufferlib handle that uses a ringbuf implementation
 *
 * @param size desired size of the buffer in bytes
 * @return buffer_handle on constructor success, otherwise, bufferhandle
 * initialized with NULL
 */
buffer_handle bufferlib_ringbuf_new(unsigned int size);


#ifdef __cplusplus
/* clang-format off */
}
/* clang-format on */
#endif /* End C linkage */
#endif /* __BUFFERLIB_H__ */
