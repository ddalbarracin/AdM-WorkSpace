/*
 * asm_clase3.S
 *
 *  Created on: May 21, 2024
 *      Author: ddalbarracin
 */

.text				@indica inicio de codigo ejecutable. va a flash
.syntax unified 	@estare utilizando UAL
.thumb				@estare utilizando modo thumb y thumb-2

.global asm_sum		@hace global la etiqueta
.global asm_svc


/* desarrollo de funcion suma en assembly
 prototype: uint32_t asm_sum(uint32_t primerOperando, uint32_t segundoOperando)
 Parametros pasados a funcion:
 	r0: uint32_t primerOperando
 	r1: uint32_t segundoOperando

 Retorno:
 	r0: retorna uint32_t
 */
.thumb_func
	asm_sum:
		add r0, r1		@r0 = r0 + r1
		bx lr			@branch with exchange to link register


.thumb_func
	asm_svc:
		svc 0			@call to supervisor call exception
		bx lr







