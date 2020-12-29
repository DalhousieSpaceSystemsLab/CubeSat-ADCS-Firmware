#ifndef __JSONS_H__
#define __JSONS_H__
#ifdef __cplusplus
/* clang-format off */
extern "C"
{
/* clang-format on */
#endif /* Start C linkage */

#include <stdint.h>

/**
 * @brief Parse a json and execute commands based on the key : value pairs
 *
 * @param json nul-terminated string in json format
 * @param json_len max length of the json (or its buffer)
 * @return int 0 == success.
 */
int json_parse(uint8_t *json, uint_least16_t json_len);

#ifdef __cplusplus
/* clang-format off */
}
/* clang-format on */
#endif /* End C linkage */
#endif /* __JSONS_H__ */
