#ifndef __FAT_POINTER_H__
#define __FAT_POINTER_H__
#ifdef __cplusplus
/* clang-format off */
extern "C"
{
/* clang-format on */
#endif /* Start C linkage */

struct fat_pointer
{
    char *       start;
    unsigned int size;
};


#ifdef __cplusplus
/* clang-format off */
}
/* clang-format on */
#endif /* End C linkage */
#endif /* __FAT_POINTER_H__ */
