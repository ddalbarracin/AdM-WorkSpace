/*
 * asm_func.h
 *
 *  Created on: May 21, 2024
 *      Author: ddalbarracin
 */

#ifndef INC_ASM_FUNC_H_
#define INC_ASM_FUNC_H_
/* Clase 3 */
uint32_t asm_sum(uint32_t primerOperando, uint32_t segundoOperando);
void asm_svc(void);

/* Clase 4*/
void asm_stack(uint32_t uno, uint32_t dos, uint32_t tres, uint32_t cuatro, uint32_t cinco, uint32_t seis);
void asm_zeros(uint32_t *vec, uint32_t longitud);
uint32_t asm_bitfield_clear(uint32_t dato, uint32_t ancho, uint32_t inicio);

/* Clase 4 - Trabajo Practico*/
void asm_mul_esc_32(uint32_t *vectorIn, uint32_t *vectorOut, uint32_t longitud, uint16_t escalar);
void asm_mul_esc_16(uint16_t *vectorIn, uint16_t *vectorOut, uint32_t longitud, uint16_t escalar);
void asm_mul_esc_16_sat(uint16_t *vectorIn, uint16_t *vectorOut, uint32_t longitud, uint16_t escalar);
uint32_t asm_bitfield_toggle(uint32_t *dato, uint32_t ancho, uint32_t inicio);
void asm_pack32to16(int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
uint32_t asm_max(int32_t * vectorIn, uint32_t longitud);
void asm_downSample (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
void asm_invertir(uint16_t * vector, uint32_t longitud);

/* Clase 7 - Trabajo Practico 3 */
uint32_t asm_potencia(int16_t * vecIn, uint32_t longitud);
uint32_t asm_potencia_DSP(int16_t * vecIn, uint32_t longitud);
void asm_medDif(int8_t *e, int8_t *x, int8_t *y, uint16_t longitud);
void asm_medDif_DSP(int8_t *e, int8_t *x, int8_t *y, uint16_t longitud);
void asm_eco(int16_t * signal, int16_t *eco, uint32_t longitud);
void asm_eco_DSP(int16_t * signal, int16_t *eco, uint32_t longitud);


#endif /* INC_ASM_FUNC_H_ */
