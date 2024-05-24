/*
 * asm_clase4_tp.S
 *
 *  Created on: May 22, 2024
 *      Author: ddalbarracin
 */


.text				@indica inicio de codigo ejecutable. va a flash
.syntax unified 	@estare utilizando UAL
.thumb				@estare utilizando modo thumb y thumb-2

.global asm_mul_esc_32			@Exercise #2
.global asm_mul_esc_16			@Exercise #3
.global asm_mul_esc_16_sat		@Exercise #4
.global asm_bitfield_toggle		@Exercise #5b



/*
 **************************************************************
 Prototype:
	void asm_mul_esc_32(uint32_t *vectorIn, uint32_t *vectorOut, uint32_t longitud, uint16_t escalar)

 Parameters:
 	r0: puntero a vector de entrada de 32 bits
 	r1: puntero a vector de salida de 32 bits
 	r2: longitud de ambos vectores
 	r3: factor escalar

 Returns: None
 **************************************************************
*/

.thumb_func
	asm_mul_esc_32:


		asm_mul_esc_32_exit:
			bx lr










/*
 **************************************************************
 Prototype:
	void asm_mul_esc_16(uint16_t *vectorIn, uint16_t *vectorOut, uint32_t longitud, uint16_t escalar)

 Parameters:
 	r0: puntero a vector de entrada de 16 bits
 	r1: puntero a vectord de salida de 16 bits
 	r2: longitud de ambos vectores
 	r3: factor escalar

 Returns: None
 **************************************************************
*/

.thumb_func
	asm_mul_esc_16:


		asm_mul_esc_16_exit:
			bx lr





/*
 **************************************************************
 Prototype:
	void asm_mul_esc_16_sat(uint16_t *vectorIn, uint16_t *vectorOut, uint32_t longitud, uint16_t escalar)

 Parameters:
 	r0: puntero a vector de entrada de 16 bits
 	r1: puntero a vectord de salida de 16 bits
 	r2: longitud de ambos vectores
 	r3: factor escalar

 Returns: None
 **************************************************************
*/

.thumb_func
	asm_mul_esc_16_sat:


		asm_mul_esc_16_sat_exit:
			bx lr




/*
 **************************************************************
 Prototype:
 	uint32_t asm_bitfield_toggle(uint32_t *dato, uint32_t ancho, uint32_t inicio)

 Parameters:
 	r0: dato a togglear
 	r1: ancho del campo de bits a togglear
 	r2: inicio del campo de bits a togglear

 Returns: r0: dato con campo de bits toggleado
 **************************************************************
*/

.thumb_func
	asm_bitfield_toggle:


		asm_bitfield_toggle_exit:
			bx lr


