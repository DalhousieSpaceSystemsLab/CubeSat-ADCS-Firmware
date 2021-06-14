/**
 * @file ring_buffer.c
 * @author Carl Mattatall (cmattatall2@gmail.com)
 * @brief Ring buffer module supporting thread safety
 * @version 0.1
 * @date 2020-12-20
 *
 * @copyright Copyright (c) 2020 carl mattatall
 */
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include <limits.h>

#include "ringbuf.h"

#if defined(RINGBUF_THREAD_SAFE)
#include <pthread.h>
#endif /* #if defined(RINGBUF_THREAD_SAFE) */


typedef struct
{
    char *             in_ptr;
    char *             out_ptr;
    unsigned int       bcnt; /* needed for overlap edgecase handling */
    struct fat_pointer buf;
#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_t inptr_val_lock;
    pthread_mutex_t outptr_val_lock;
    pthread_mutex_t bcnt_val_lock;
    pthread_mutex_t inptr_lock;
    pthread_mutex_t outptr_lock;
#endif /* #if defined(RINGBUF_THREAD_SAFE) */
} ringbuf_t;

static void         ringbuf_write_inptr_SAFE(ringbuf_t *ringbuf, char byte);
static void         ringbuf_write_outptr_SAFE(ringbuf_t *ringbuf, char byte);
static char         ringbuf_read_outptr_SAFE(ringbuf_t *ringbuf);
static char         ringbuf_read_inptr_SAFE(ringbuf_t *ringbuf);
static void         ringbuf_inc_inptr_SAFE(ringbuf_t *ringbuf);
static void         ringbuf_inc_outptr_SAFE(ringbuf_t *ringbuf);
static void         ringbuf_inc_bcnt_SAFE(ringbuf_t *ringbuf);
static void         ringbuf_dec_bcnt_SAFE(ringbuf_t *ringbuf);
static unsigned int ringbuf_peek_bcnt_SAFE(ringbuf_t *ringbuf);
static bool         ringbuf_ptr_equal_SAFE(ringbuf_t *ringbuf);


#if !defined(RINGBUF_INPUT_OVERRUN)
static bool ringbuf_is_full_SAFE(ringbuf_t *ringbuf);
#endif /*#if !defined(RINGBUF_INPUT_OVERRUN) */

/**
 * @brief check if ring buffer is empty
 *
 * @param ringbuf buffer handle
 * @return true if empty, otherwise false
 *
 * @note thread safe
 */
static bool ringbuf_is_empty_SAFE(ringbuf_t *ringbuf);


buffer_instance_handle ringbuf_ctor_new(unsigned int size)
{
    ringbuf_t *ringbuf = malloc(sizeof(*ringbuf));
    assert(ringbuf != NULL);

    ringbuf->buf.start = malloc(size);
    assert(ringbuf->buf.start != NULL);
    memset(ringbuf->buf.start, 0, size);
    ringbuf->in_ptr  = ringbuf->buf.start;
    ringbuf->out_ptr = ringbuf->buf.start;

    ringbuf->buf.size = size;
    ringbuf->bcnt     = 0;

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_init(&ringbuf->inptr_val_lock, NULL);
    pthread_mutex_init(&ringbuf->outptr_val_lock, NULL);
    pthread_mutex_init(&ringbuf->bcnt_val_lock, NULL);
    pthread_mutex_init(&ringbuf->inptr_lock, NULL);
    pthread_mutex_init(&ringbuf->outptr_lock, NULL);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    return (buffer_instance_handle)ringbuf;
}




void ringbuf_dtor(buffer_instance_handle this)
{
    if (this != NULL)
    {
        ringbuf_t *ringbuf = this;
        free(ringbuf->buf.start);
#if defined(RINGBUF_THREAD_SAFE)
        pthread_mutex_destroy(&ringbuf->inptr_val_lock);
        pthread_mutex_destroy(&ringbuf->outptr_val_lock);
        pthread_mutex_destroy(&ringbuf->bcnt_val_lock);
        pthread_mutex_destroy(&ringbuf->inptr_lock);
        pthread_mutex_destroy(&ringbuf->inptr_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */
        free(ringbuf);
    }
}


int ringbuf_read_next(buffer_instance_handle this)
{
    int        byte_read;
    ringbuf_t *ringbuf = this;

    if (ringbuf_is_empty_SAFE(ringbuf))
    {
        byte_read = BUFFERLIB_READ_FAILURE;
    }
    else
    {
        byte_read = ringbuf_read_outptr_SAFE(ringbuf);
    }

    if (byte_read != BUFFERLIB_READ_FAILURE)
    {
        ringbuf_inc_outptr_SAFE(ringbuf);
        ringbuf_dec_bcnt_SAFE(ringbuf);
    }

    return byte_read;
}


int ringbuf_write_next(buffer_instance_handle this, char byte)
{
    int        byte_written;
    ringbuf_t *ringbuf = this;

    if (ringbuf_is_full_SAFE(ringbuf))
    {
#if defined(RINGBUF_INPUT_OVERRUN)
        /* Overwrite outptr,
         * then advance BOTH inptr and outptr */
        /* Do not increase bcnt */
        *current_inptr = byte;
        ringbuf_inc_outptr_SAFE(ringbuf);
        ringbuf_inc_inptr_SAFE(ringbuf);
        byte_written = byte;
#else
        byte_written = BUFFERLIB_WRITE_FAILURE;
#endif /* #if defined(RINGBUF_INPUT_OVERRUN) */
    }
    else
    {
        ringbuf_write_inptr_SAFE(ringbuf, byte);
        ringbuf_inc_bcnt_SAFE(ringbuf);
        ringbuf_inc_inptr_SAFE(ringbuf);
        byte_written = byte;
    }

    return byte_written;
}


unsigned int ringbuf_size(buffer_instance_handle this)
{
    if (this == NULL)
    {
        return 0;
    }
    else
    {
        ringbuf_t *ringbuf = this;
        return ringbuf->buf.size;
    }
}


static void ringbuf_write_inptr_SAFE(ringbuf_t *ringbuf, char byte)
{

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_lock(&ringbuf->inptr_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    *ringbuf->in_ptr = byte;

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_unlock(&ringbuf->inptr_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

static void ringbuf_write_outptr_SAFE(ringbuf_t *ringbuf, char byte)
{

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_lock(&ringbuf->outptr_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    *ringbuf->out_ptr = byte;

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_unlock(&ringbuf->outptr_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */
}
#pragma GCC diagnostic pop


static char ringbuf_read_outptr_SAFE(ringbuf_t *ringbuf)
{

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_lock(&ringbuf->outptr_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    char byte = *ringbuf->out_ptr;

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_unlock(&ringbuf->outptr_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    return byte;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

static char ringbuf_read_inptr_SAFE(ringbuf_t *ringbuf)
{

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_lock(&ringbuf->inptr_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    char byte = *ringbuf->in_ptr;

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_unlock(&ringbuf->inptr_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    return byte;
}
#pragma GCC diagnostic pop


static void ringbuf_inc_inptr_SAFE(ringbuf_t *ringbuf)
{
    char *new_ptr;
    if ((ringbuf->in_ptr - ringbuf->buf.start) == (int)ringbuf->buf.size)
    {
        new_ptr = ringbuf->buf.start;
    }
    else
    {
        new_ptr = ringbuf->in_ptr + 1;
    }

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_lock(&ringbuf->inptr_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    ringbuf->in_ptr = new_ptr;

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_unlock(&ringbuf->inptr_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */
}


static void ringbuf_inc_outptr_SAFE(ringbuf_t *ringbuf)
{
    char *new_ptr;
    if ((ringbuf->out_ptr - ringbuf->buf.start) == (int)ringbuf->buf.size)
    {
        new_ptr = ringbuf->buf.start;
    }
    else
    {
        new_ptr = ringbuf->out_ptr + 1;
    }

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_lock(&ringbuf->outptr_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    ringbuf->out_ptr = new_ptr;

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_unlock(&ringbuf->outptr_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */
}


static void ringbuf_inc_bcnt_SAFE(ringbuf_t *ringbuf)
{

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_lock(&ringbuf->bcnt_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    if (ringbuf->bcnt < ringbuf->buf.size)
    {
        ringbuf->bcnt++;
    }

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_unlock(&ringbuf->bcnt_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */
}


static void ringbuf_dec_bcnt_SAFE(ringbuf_t *ringbuf)
{

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_lock(&ringbuf->bcnt_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    if (ringbuf->bcnt > 0)
    {
        ringbuf->bcnt--;
    }

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_unlock(&ringbuf->bcnt_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */
}


static unsigned int ringbuf_peek_bcnt_SAFE(ringbuf_t *ringbuf)
{
    unsigned int bcnt;
#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_lock(&ringbuf->bcnt_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    bcnt = ringbuf->bcnt;

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_unlock(&ringbuf->bcnt_val_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    return bcnt;
}


#if !defined(RINGBUF_INPUT_OVERRUN)
static bool ringbuf_is_full_SAFE(ringbuf_t *ringbuf)
{
    bool         is_full = false;
    unsigned int bcnt    = ringbuf_peek_bcnt_SAFE(ringbuf);
    if (bcnt == ringbuf->buf.size && ringbuf_ptr_equal_SAFE(ringbuf))
    {
        is_full = true;
    }
    return is_full;
}
#endif /* #if !defined(RINGBUF_INPUT_OVERRUN) */


static bool ringbuf_is_empty_SAFE(ringbuf_t *ringbuf)
{
    bool         is_empty = false;
    unsigned int bcnt     = ringbuf_peek_bcnt_SAFE(ringbuf);
    if (bcnt == 0 && ringbuf_ptr_equal_SAFE(ringbuf))
    {
        is_empty = true;
    }
    return is_empty;
}

static bool ringbuf_ptr_equal_SAFE(ringbuf_t *ringbuf)
{
    char *inptr;
    char *outptr;

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_lock(&ringbuf->outptr_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    outptr = ringbuf->out_ptr;

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_unlock(&ringbuf->outptr_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_lock(&ringbuf->inptr_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    inptr = ringbuf->in_ptr;

#if defined(RINGBUF_THREAD_SAFE)
    pthread_mutex_unlock(&ringbuf->inptr_lock);
#endif /* #if defined(RINGBUF_THREAD_SAFE) */

    return inptr == outptr;
}
