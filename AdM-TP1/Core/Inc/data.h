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
#define c_medDif_longitud    		((uint16_t)5)
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
int8_t c_medDif_x [] = {1, 2, 3, 4, 5};
int8_t c_medDif_y[] = {6, 7, 8, 9, 10};

int8_t asm_medDif_e [c_medDif_longitud];
int8_t asm_DSP_medDif_e [c_medDif_longitud];

/* c_medDif data */



#endif /* __DATA_H__ */
