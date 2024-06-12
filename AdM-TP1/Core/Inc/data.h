/*
 * data.h
 *
 *  Created on: Jun 6, 2024
 *      Author: recastrobeltran
 */

#ifndef __DATA_H__
#define __DATA_H__

#include <stdint.h>

#define c_potencia_POTENCIA_LEN     ((uint32_t)4)
#define c_medDif_longitud    		((uint16_t)8)
#define c_eco_longitud    			((uint32_t)4096)

/* c_potencia data */
int16_t c_potencia_vecIn[] = {

	1,
	2,
	3,
	4,

};

/* c_medDif data */

int8_t c_medDif_e [c_medDif_longitud];
int8_t c_medDif_x [] = {5, 9, 15, 22, 2, 4, 1, 9};
int8_t c_medDif_y[] = {1, 3, 5, 10, 1, 8, 3, 1};

int8_t asm_medDif_e [c_medDif_longitud];
int8_t asm_DSP_medDif_e [c_medDif_longitud];

/* c_eco data */

int16_t c_eco_signalIn[c_eco_longitud];
int16_t c_eco_signalOut [c_eco_longitud];
int16_t asm_eco_signalOut [c_eco_longitud];
int16_t asm_DSP_eco_signalOut [c_eco_longitud];

#endif /* __DATA_H__ */
