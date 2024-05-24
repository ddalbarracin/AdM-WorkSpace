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

#endif /* INC_ASM_FUNC_H_ */
