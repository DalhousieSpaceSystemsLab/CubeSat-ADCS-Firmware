/**
 * @file jsons.c
 * @author Carl Mattatall (cmattatall2@gmail.com)
 * @brief JSON ADCS payload parsing file for ADCS firmware
 * @version 0.1
 * @date 2020-12-09
 *
 * @copyright Copyright (c) 2020 DSS - LORIS project
 *
 */
#include "targets.h"

#include <stdint.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "jtok.h"
#include "obc_interface.h"
#include "jsons.h"
#include "version.h"

#include "reaction_wheels.h"
#include "magnetorquers.h"

#define BASE_10 10
#define JSON_TKN_CNT 20
#define JSON_HANDLER_RETVAL_ERROR NULL

typedef uint_fast16_t token_index_t;

typedef void *         json_handler_retval;
typedef token_index_t *json_handler_args;
typedef json_handler_retval (*json_handler)(json_handler_args);

typedef struct
{
    char         key[25];
    json_handler handler;
} json_parse_table_item;


static jtok_tkn_t tkns[JSON_TKN_CNT];
static char       tmp_chrbuf[50];


/* JSON HANDLER DECLARATIONS */
static void *parse_hardware_json(json_handler_args args);
static void *parse_firmware_json(json_handler_args args);
static void  parse_rw_speed(json_handler_args args);
static void  parse_rw_current(json_handler_args args);
static void  parse_mqtr_volts(json_handler_args args);
static void  parse_sunSen(json_handler_args args);
static void  parse_magSen(json_handler_args args);
static void  parse_burnWire(json_handler_args args);
static void  parse_imu(json_handler_args args);
static void  parse_current(json_handler_args args);


/* JSON PARSE TABLE */
/* clang-format off */
static const json_parse_table_item json_parse_table[] = {
    {.key = "fwVersion",  .handler = parse_firmware_json},
    {.key = "hwVersion",  .handler = parse_hardware_json},
    {.key = "rw_speed",   .handler = parse_rw_speed},
    {.key = "rw_current", .handler = parse_rw_current},
    {.key = "mqtr_volts", .handler = parse_mqtr_volts},
    {.key = "sunSen",     .handler = parse_sunSen},
    {.key = "magSen",     .handler = parse_magSen},
    {.key = "burnWire",   .handler = parse_burnWire},
    {.key = "imu",        .handler = parse_imu},
    {.key = "current",    .handler = parse_current},
};
/* clang-format on */


int json_parse(uint8_t *json)
{
    CONFIG_ASSERT(json != NULL);

    int json_parse_status = 0;

    int jtok_retval = jtok_parse((char *)json, tkns, JSON_TKN_CNT);

    if (jtok_retval != JTOK_PARSE_STATUS_OK)
    {
        json_parse_status = jtok_retval;
        memset(tkns, 0, sizeof(tkns));
    }
    else
    {

        token_index_t t; /* token index */
        token_index_t k; /* key index for json table */
        t = 0;
        if (isValidJson(tkns, JSON_TKN_CNT))
        {
            /* Go through command table and check if we have a registered
             * command for the key */
            t++;

            unsigned int k_max =
                sizeof(json_parse_table) / sizeof(*json_parse_table);
            for (k = 0; k < k_max; k++)
            {
                /*
                 * If we have a command for the current key,
                 * execute the command handler
                 */
                if (jtok_tokcmp(json_parse_table[k].key, &tkns[t]))
                {
                    if (NULL != json_parse_table[k].handler)
                    {
                        json_handler_retval retval;
                        retval = json_parse_table[k].handler(&t);
                        if (retval == JSON_HANDLER_RETVAL_ERROR)
                        {
                            json_parse_status = -1;
                        }
                    }
                    break;
                }
            }

            /* No match with supported json keys */
            if (k >= k_max)
            {
                json_parse_status = -1;
            }
        }
        else
        {
            json_parse_status = 1;
        }
    }
    return json_parse_status;
}


static void *parse_firmware_json(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    *t += 1;
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        OBC_IF_printf("{\"fwVersion\" : %s}", FW_VERSION);
        return (void *)t;
    }
    else
    {
        return JSON_HANDLER_RETVAL_ERROR;
    }
}


static void *parse_hardware_json(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    *t += 1;
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        OBC_IF_printf("{\"hwVersion\" : %s}", HW_VERSION);
        return (void *)t;
    }
    else
    {
        return JSON_HANDLER_RETVAL_ERROR;
    }
}


static void parse_rw_speed(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* Advance json token index */


    if (jtok_tokcmp("read", &tkns[*t]))
    {
        memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));
        rw_config_to_string(tmp_chrbuf, sizeof(tmp_chrbuf));
        OBC_IF_printf("{\"rw_speed\": %s}", tmp_chrbuf);
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            *t += 1;
            if (tkns[*t].type == JTOK_ARRAY)
            {
                *t += 1; /* Advance token index to the first element of arr */
                const jtok_tkn_t *tkn;
                unsigned int      i = 0;

                /* Traverse sibling tree of array elements */
                do
                {
                    tkn = &tkns[*t];
                    memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));
                    jtok_tokcpy(tmp_chrbuf, sizeof(tmp_chrbuf), &tkns[*t]);
                    char *endptr = tmp_chrbuf;
                    int   new_speed;
                    new_speed = (int)strtoul(tmp_chrbuf, &endptr, BASE_10);
                    if (*endptr == '\0')
                    {
                        i++;
                        switch (i)
                        {
                            case 1:
                            {
                                rw_set_config(REAC_WHEEL_x, new_speed);
                            }
                            break;
                            case 2:
                            {
                                rw_set_config(REAC_WHEEL_y, new_speed);
                            }
                            break;
                            case 3:
                            {
                                rw_set_config(REAC_WHEEL_z, new_speed);
                            }
                            break;
                            default:
                            {
                                return JSON_HANDLER_RETVAL_ERROR;
                            }
                            break;
                        }
                    }
                    else
                    {
                        /*
                         * error parsing the value
                         * - couldn't reach end of token
                         */
                        return JSON_HANDLER_RETVAL_ERROR;
                    }
                    memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));

                    *t = tkn->sibling;
                } while (*t != NO_SIBLING_IDX);


                if (i != NUM_REACTION_WHEELS)
                {
                    /* Array didn't contain speed values for all the params
                     * eg : [ 12, 34] <-- missing third value for rw_z */
                    OBC_IF_printf("{\"rw_speed\": \"write error\"}");
                    return JSON_HANDLER_RETVAL_ERROR;
                }
                else
                {
                    OBC_IF_printf("{\"rw_speed\": \"set\"}");
                }
            }
            else
            {
                return JSON_HANDLER_RETVAL_ERROR;
            }
        }
        else
        {
            /*
             * we were missing data from the payload.
             * eg : { "rw_speed" : "write" } <-- notice "value" : [ NUMBERS ]
             * is missing
             */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {
        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void parse_rw_current(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* Advance json token index */

    if (jtok_tokcmp("read", &tkns[*t]))
    {
        int current_ma_x = rw_measure_current_ma(REAC_WHEEL_x);
        int current_ma_y = rw_measure_current_ma(REAC_WHEEL_y);
        int current_ma_z = rw_measure_current_ma(REAC_WHEEL_z);
        OBC_IF_printf("{\"rw_current\": [ %d, %d, %d]}", current_ma_x,
                      current_ma_y, current_ma_z);
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            *t += 1;
            if (tkns[*t].type == JTOK_ARRAY)
            {
                *t += 1; /* Advance token index to the first element of arr */
                const jtok_tkn_t *tkn;
                unsigned int      i = 0;

                /* Traverse sibling tree of array elements */
                do
                {
                    tkn = &tkns[*t];
                    memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));
                    jtok_tokcpy(tmp_chrbuf, sizeof(tmp_chrbuf), &tkns[*t]);
                    char *endptr = tmp_chrbuf;
                    int   new_speed;
                    new_speed = (int)strtoul(tmp_chrbuf, &endptr, BASE_10);
                    if (*endptr == '\0')
                    {
                        i++;
                        switch (i)
                        {
                            case 1:
                            {
                                rw_set_config(REAC_WHEEL_x, new_speed);
                            }
                            break;
                            case 2:
                            {
                                rw_set_config(REAC_WHEEL_y, new_speed);
                            }
                            break;
                            case 3:
                            {
                                rw_set_config(REAC_WHEEL_z, new_speed);
                            }
                            break;
                            default:
                            {
                                return JSON_HANDLER_RETVAL_ERROR;
                            }
                            break;
                        }
                    }
                    else
                    {
                        /*
                         * error parsing the value
                         * - couldn't reach end of token
                         */
                        return JSON_HANDLER_RETVAL_ERROR;
                    }
                    memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));

                    *t = tkn->sibling;
                } while (*t != NO_SIBLING_IDX);


                if (i != NUM_REACTION_WHEELS)
                {
                    /* Array didn't contain speed values for all the params
                     * eg : [ 12, 34] <-- missing third value for rw_z */
                    OBC_IF_printf("{\"rw_speed\": \"write error\"}");
                    return JSON_HANDLER_RETVAL_ERROR;
                }
                else
                {
                    OBC_IF_printf("{\"rw_speed\": \"set\"}");
                }
            }
            else
            {
                return JSON_HANDLER_RETVAL_ERROR;
            }
        }
        else
        {
            /*
             * we were missing data from the payload.
             * eg : { "rw_speed" : "write" } <-- notice "value" : [ NUMBERS ]
             * is missing
             */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {
        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void parse_mqtr_volts(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* Advance json token index */

    if (jtok_tokcmp("read", &tkns[*t]))
    {
    }
    else
    {
        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void parse_sunSen(json_handler_args args)
{

}


static void parse_magSen(json_handler_args args)
{
    
}


static void parse_burnWire(json_handler_args args)
{}


static void parse_imu(json_handler_args args)
{}


static void parse_current(json_handler_args args)
{}


#if 0
static void *parse_pwm_rw_x(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        pwm_t current_x_pwm = reacwheel_get_wheel_pwm(REACTION_WHEEL_x);
        OBC_IF_printf("{\"pwm_rw_x\" : %u}", current_x_pwm);
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            /* getting values from JSON is a little unelegant in C ... */
            *t += 1;


            memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));
            jtok_tokcpy(tmp_chrbuf, sizeof(tmp_chrbuf), &tkns[*t]);
            char *endptr    = tmp_chrbuf;
            pwm_t new_value = (pwm_t)strtoul(tmp_chrbuf, &endptr, BASE_10);
            if (*endptr != '\0')
            {
                /* error parsing the value - couldn't reach end of token */
                return JSON_HANDLER_RETVAL_ERROR;
            }
            else
            {
                reacwheel_set_wheel_pwm(REACTION_WHEEL_x, new_value);
                OBC_IF_printf("{\"pwm_rw_x\":\"written\"}");
            }
            memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));
        }
        else
        {
            /* we were missing data from the payload.
             * eg : { "pwm_rw_x" : "write" } <-- notice "value" : 55 is missing
             */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void *parse_pwm_rw_y(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        pwm_t current_y_pwm = reacwheel_get_wheel_pwm(REACTION_WHEEL_y);
        OBC_IF_printf("{\"pwm_rw_y\" : %u}", current_y_pwm);
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            /* getting values from JSON is a little unelegant in C ... */
            *t += 1;
            memset(tmp_chrbuf, '\0', sizeof(tmp_chrbuf));
            jtok_tokcpy(tmp_chrbuf, sizeof(tmp_chrbuf), &tkns[*t]);
            char *endptr    = tmp_chrbuf;
            pwm_t new_value = (pwm_t)strtoul(tmp_chrbuf, &endptr, BASE_10);
            if (*endptr != '\0')
            {
                /* error parsing the value - couldn't reach end of token */
                return JSON_HANDLER_RETVAL_ERROR;
            }
            else
            {
                reacwheel_set_wheel_pwm(REACTION_WHEEL_y, new_value);
                OBC_IF_printf("{\"pwm_rw_y\":\"written\"}");
            }
            memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));
        }
        else
        {
            /* we were missing data from the payload.
             * eg : { "pwm_rw_y" : "write" } <-- notice "value" : 55 is missing
             */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void *parse_pwm_rw_z(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        pwm_t current_z_pwm = reacwheel_get_wheel_pwm(REACTION_WHEEL_z);
        OBC_IF_printf("{\"pwm_rw_z\" : %u}", current_z_pwm);
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            /* getting values from JSON is a little unelegant in C ... */
            *t += 1;
            memset(tmp_chrbuf, '\0', sizeof(tmp_chrbuf));
            jtok_tokcpy(tmp_chrbuf, sizeof(tmp_chrbuf), &tkns[*t]);
            char *endptr    = tmp_chrbuf;
            pwm_t new_value = (pwm_t)strtoul(tmp_chrbuf, &endptr, BASE_10);
            if (*endptr != '\0')
            {
                /* error parsing the value - couldn't reach end of token */
                return JSON_HANDLER_RETVAL_ERROR;
            }
            else
            {
                reacwheel_set_wheel_pwm(REACTION_WHEEL_z, new_value);
                OBC_IF_printf("{\"pwm_rw_z\":\"written\"}");
            }
            memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));
        }
        else
        {
            /*
             * we were missing data from the payload.
             * eg : { "pwm_rw_z" : "write" } <-- notice "value" : 55 is missing
             */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void *parse_dir_rw_x(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        RW_DIR_t current_dir = reacwheel_get_wheel_pwm(REACTION_WHEEL_x);
        OBC_IF_printf("{\"dir_rw_x\": \"%s\"}", reacwheel_dir_str(current_dir));
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            /* getting values from JSON is a little unelegant in C ... */
            *t += 1;
            if (jtok_tokcmp("clock", &tkns[*t]))
            {
                reacwheel_set_wheel_dir(REACTION_WHEEL_x, RW_DIR_clockwise);
            }
            else if (jtok_tokcmp("antiClock", &tkns[*t]))
            {
                reacwheel_set_wheel_dir(REACTION_WHEEL_x, RW_DIR_anticlockwise);
            }
            else
            {
                /*
                 * We won't set value because we knows its an invalid command
                 * and can handle it here (rather than rely on caller)
                 */
                return JSON_HANDLER_RETVAL_ERROR;
            }

            OBC_IF_printf("{\"dir_rw_x\":\"written\"}");
        }
        else
        {
            /*
             * we were missing data from the payload.
             * eg : { "dir_rw_x" : "write" } <-- notice "value" missing
             */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void *parse_dir_rw_y(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        RW_DIR_t current_dir = reacwheel_get_wheel_pwm(REACTION_WHEEL_y);
        switch (current_dir)
        {
            case RW_DIR_clockwise:
            {
                OBC_IF_printf("{\"dir_rw_y\" : \"%s\"}", "clock");
            }
            break;
            case RW_DIR_anticlockwise:
            {
                OBC_IF_printf("{\"dir_rw_y\" : \"%s\"}", "antiClock");
            }
            break;
            case RW_DIR_invalid:
            {
                OBC_IF_printf("{\"dir_rw_y\" : \"%s\"}", "invalid");
            }
            break;
        }
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            /* getting values from JSON is a little unelegant in C ... */
            *t += 1;
            if (jtok_tokcmp("clock", &tkns[*t]))
            {
                reacwheel_set_wheel_dir(REACTION_WHEEL_y, RW_DIR_clockwise);
            }
            else if (jtok_tokcmp("antiClock", &tkns[*t]))
            {
                reacwheel_set_wheel_dir(REACTION_WHEEL_y, RW_DIR_anticlockwise);
            }
            else
            {
                /*
                 * We won't set value because we knows its an invalid command
                 * and can handle it here (rather than rely on caller)
                 */
                return JSON_HANDLER_RETVAL_ERROR;
            }

            OBC_IF_printf("{\"dir_rw_y\":\"written\"}");
        }
        else
        {
            /*
             * we were missing data from the payload.
             * eg : { "dir_rw_y" : "write" } <-- notice "value" missing
             */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void *parse_dir_rw_z(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        RW_DIR_t current_dir = reacwheel_get_wheel_pwm(REACTION_WHEEL_z);
        switch (current_dir)
        {
            case RW_DIR_clockwise:
            {
                OBC_IF_printf("{\"dir_rw_z\" : \"%s\"}", "clock");
            }
            break;
            case RW_DIR_anticlockwise:
            {
                OBC_IF_printf("{\"dir_rw_z\" : \"%s\"}", "antiClock");
            }
            break;
            case RW_DIR_invalid:
            {
                OBC_IF_printf("{\"dir_rw_z\" : \"%s\"}", "invalid");
            }
            break;
        }
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            /* getting values from JSON is a little unelegant in C ... */
            *t += 1;
            if (jtok_tokcmp("clock", &tkns[*t]))
            {
                reacwheel_set_wheel_dir(REACTION_WHEEL_z, RW_DIR_clockwise);
            }
            else if (jtok_tokcmp("antiClock", &tkns[*t]))
            {
                reacwheel_set_wheel_dir(REACTION_WHEEL_z, RW_DIR_anticlockwise);
            }
            else
            {
                /*
                 * We won't set value because we knows its an invalid command
                 * and can handle it here (rather than rely on caller)
                 */
                return JSON_HANDLER_RETVAL_ERROR;
            }

            OBC_IF_printf("{\"dir_rw_z\":\"written\"}");
        }
        else
        {
            /*
             * we were missing data from the payload.
             * eg : { "dir_rw_z" : "write" } <-- notice "value" missing
             */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void *parse_pwm_mqtr_x(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        /** @todo IMPLEMENT */
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            /* getting values from JSON is a little unelegant in C ... */
            *t += 1;
            memset(tmp_chrbuf, '\0', sizeof(tmp_chrbuf));
            jtok_tokcpy(tmp_chrbuf, sizeof(tmp_chrbuf), &tkns[*t]);
            char *endptr    = tmp_chrbuf;
            pwm_t new_value = (pwm_t)strtoul(tmp_chrbuf, &endptr, BASE_10);
            if (*endptr != '\0')
            {
                /* error parsing the value - couldn't reach end of token */
                return JSON_HANDLER_RETVAL_ERROR;
            }
            else
            {
                mqtr_set_pwm(MQTR_x, new_value);
                OBC_IF_printf("{\"mqtr_x\":\"written\"}");
            }
            memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));
        }
        else
        {
            /* we were missing data from the payload. */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void *parse_pwm_mqtr_y(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        /** @todo IMPLEMENT */
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            /* getting values from JSON is a little unelegant in C ... */
            *t += 1;
            memset(tmp_chrbuf, '\0', sizeof(tmp_chrbuf));
            jtok_tokcpy(tmp_chrbuf, sizeof(tmp_chrbuf), &tkns[*t]);
            char *endptr    = tmp_chrbuf;
            pwm_t new_value = (pwm_t)strtoul(tmp_chrbuf, &endptr, BASE_10);
            if (*endptr != '\0')
            {
                /* error parsing the value - couldn't reach end of token */
                return JSON_HANDLER_RETVAL_ERROR;
            }
            else
            {
                mqtr_set_pwm(MQTR_y, new_value);
                OBC_IF_printf("{\"mqtr_y\":\"written\"}");
            }
            memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));
        }
        else
        {
            /* we were missing data from the payload. */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void *parse_pwm_mqtr_z(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        /** @todo IMPLEMENT */
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            /* getting values from JSON is a little unelegant in C ... */
            *t += 1;
            memset(tmp_chrbuf, '\0', sizeof(tmp_chrbuf));
            jtok_tokcpy(tmp_chrbuf, sizeof(tmp_chrbuf), &tkns[*t]);
            char *endptr    = tmp_chrbuf;
            pwm_t new_value = (pwm_t)strtoul(tmp_chrbuf, &endptr, BASE_10);
            if (*endptr != '\0')
            {
                /* error parsing the value - couldn't reach end of token */
                return JSON_HANDLER_RETVAL_ERROR;
            }
            else
            {
                mqtr_set_pwm(MQTR_z, new_value);
                OBC_IF_printf("{\"mqtr_z\":\"written\"}");
            }
            memset(tmp_chrbuf, 0, sizeof(tmp_chrbuf));
        }
        else
        {
            /* we were missing data from the payload. */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void *parse_dir_mqtr_x(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        /** @todo IMPLEMENT */
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            *t += 1;
            if (jtok_tokcmp("clock", &tkns[*t]))
            {
                mqtr_set_dir(MQTR_x, RW_DIR_clockwise);
            }
            else if (jtok_tokcmp("antiClock", &tkns[*t]))
            {
                mqtr_set_dir(MQTR_x, RW_DIR_anticlockwise);
            }
            else
            {
                /*
                 * We won't set value because we knows its an invalid command
                 * and can handle it here (rather than rely on caller)
                 */
                return JSON_HANDLER_RETVAL_ERROR;
            }
        }
        else
        {
            /* we were missing data from the payload. */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void *parse_dir_mqtr_y(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        /** @todo IMPLEMENT */
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            *t += 1;
            if (jtok_tokcmp("clock", &tkns[*t]))
            {
                mqtr_set_dir(MQTR_y, RW_DIR_clockwise);
            }
            else if (jtok_tokcmp("antiClock", &tkns[*t]))
            {
                mqtr_set_dir(MQTR_y, RW_DIR_anticlockwise);
            }
            else
            {
                /*
                 * We won't set value because we knows its an invalid command
                 * and can handle it here (rather than rely on caller)
                 */
                return JSON_HANDLER_RETVAL_ERROR;
            }
        }
        else
        {
            /* we were missing data from the payload. */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}


static void *parse_dir_mqtr_z(json_handler_args args)
{
    token_index_t *t = (token_index_t *)args;
    CONFIG_ASSERT(*t < JSON_TKN_CNT);
    *t += 1; /* don't do ++ because * has higher precedence than ++ */
    if (jtok_tokcmp("read", &tkns[*t]))
    {
        /** @todo IMPLEMENT */
    }
    else if (jtok_tokcmp("write", &tkns[*t]))
    {
        *t += 1;
        if (jtok_tokcmp("value", &tkns[*t]))
        {
            *t += 1;
            if (jtok_tokcmp("clock", &tkns[*t]))
            {
                mqtr_set_dir(MQTR_z, RW_DIR_clockwise);
            }
            else if (jtok_tokcmp("antiClock", &tkns[*t]))
            {
                mqtr_set_dir(MQTR_z, RW_DIR_anticlockwise);
            }
            else
            {
                /*
                 * We won't set value because we knows its an invalid command
                 * and can handle it here (rather than rely on caller)
                 */
                return JSON_HANDLER_RETVAL_ERROR;
            }
        }
        else
        {
            /* we were missing data from the payload. */
            return JSON_HANDLER_RETVAL_ERROR;
        }
    }
    else
    {

        return JSON_HANDLER_RETVAL_ERROR;
    }
    return t;
}
#endif
