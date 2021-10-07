#ifndef __MAGNETOMETER_H__
#define __MAGNETOMETER_H__
#ifdef __cplusplus
/* clang-format off */
extern "C"
{
/* clang-format on */
#endif /* Start C linkage */

void MAGTOM_init(void);
int  MAGTOM_measurement_to_string(char *buf, int buflen);
void MAGTOM_reset(void);

typedef struct MAGTOM_measurement
{
    float x_BMAG;
    float y_BMAG;
    float z_BMAG;
    //int sensortime;
}MAGTOM_measurement_t;

MAGTOM_measurement_t MAGTOM_get_measurement(void);

#ifdef __cplusplus
/* clang-format off */
}
/* clang-format on */
#endif /* End C linkage */
#endif /* __MAGNETOMETER_H__ */
