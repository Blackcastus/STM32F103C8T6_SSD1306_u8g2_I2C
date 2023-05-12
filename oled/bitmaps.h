/*
* @Author: nhantt
* @Date:   2019-12-13
* @Last Modified by:   nhantt
* @Last Modified time: 2019-12-13
*/

#ifndef __BITMAP_H__
#define __BITMAP_H__

#include <stdint.h>
#include "main.h"

/* bitmap struct */
typedef struct {
	uint8_t width, height;
	const uint8_t *data;
} BITMAP;


/* variable declare*/
extern const BITMAP bmp_logo_ph;
extern const BITMAP bmp_neon_on;
extern const BITMAP bmp_neon_off;
extern const BITMAP bmp_uv_on;
extern const BITMAP bmp_uv_off;
extern const BITMAP bmp_fan_origin;
extern const BITMAP bmp_fan_rotate;
extern const BITMAP bmp_socket_on;
extern const BITMAP bmp_socket_off;
extern const BITMAP bmp_temp;
extern const BITMAP bmp_uv1_on;
extern const BITMAP bmp_uv2_on;

#endif
