/*
 * asm_clase4.S
 *
 *  Created on: May 22, 2024
 *      Author: ddalbarracin
 */

.text				@indica inicio de codigo ejecutable. va a flash
.syntax unified 	@estare utilizando UAL
.thumb				@estare utilizando modo thumb y thumb-2

.global asm_stack		@hace global la etiqueta
.global asm_zeros
.global asm_bitfield_clear



/*
 **************************************************************
 Prototype:

	void asm_stack(uint32_t uno, uint32_t dos, uint32_t tres, uint32_t cuatro, uint32_t cinco, uint32_t seis)

 Parameters: r0 a r5
 Returns: None
 **************************************************************
*/
.thumb_func
	asm_stack:
		/*
		/**************************************************************
		 ANTES del push, el Stack se organiza de la siguiente manera:
		 	Dato            	| Dirección
         --------------------------------------------
         r0 = parametro uno		| 		SP + 20
         r1 = parametro dos   	|		SP + 16
         r2 = parametro tres	|		SP + 12
         r3 = parametro cuatro	|		SP + 8
         6° parámetro    		| 		SP + 4
         5° parámetro    		|	<-- SP
        ***************************************************************
        */

        push {r4 - r8} 		// Guardo en Stack los registros r4 a r8

        /*
        ***************************************************************
         LUEGO del push, el Stack se organiza de la siguiente manera:
		 	Dato            	| Dirección
         --------------------------------------------
         r0 = parametro uno		| 		SP + 40
         r1 = parametro dos   	|		SP + 36
         r2 = parametro tres	|		SP + 32
         r3 = parametro cuatro	|		SP + 28
         6° parámetro    		| 		SP + 24
         5° parámetro    		|		SP + 20
         r4 = info del main		| 		SP + 16
         r5 = info del main		| 		SP + 12
         r6 = info del main		| 		SP + 8
         r7 = info del main		| 		SP + 4
         r8 = info del main		| 	<-- SP
        ***************************************************************
        */


		ldr r4, [SP, 20] 	// Recupero el 5to parametro. r4 = *(SP + 20)
		ldr r5, [SP, 24]	// Recupero el 5to parametro. r4 = *(SP + 24)

		pop {r4 - r8}		// Antes de volver al main hago el stack frame
							// para retornar los valores de r4-r8 que usa el main
		bx lr				// Inconditional branch with exchange to Link Register

/*
 **************************************************************
 Prototype:
	void asm_zeros(uint32_t * vector, uint32_t longitud)

 Parameters:
	r0: dirección del vector
	r1: longitud

 Return: None
 **************************************************************
*/
.thumb_func
	asm_zeros:
			// Guarda de seguridad
			cbz r0, zeros_exit				// Compare and branch if r0 is zero
			mov r2, #0						// set r2 = 0

			// Se recorre el vector del último al primer elemento
		loop:
			sub r1, #1						// r1 = r1 - 1
			str r2, [r0, r1, LSL #2]		// *(r0 + (r1<<2)) = r2
			cbz r1, zeros_exit				// compare and branch if r1 is zero
			b loop							// inconditional branch to loop

		zeros_exit:
			bx lr							// Inconditional branch with exchange to Link Register

/*
 **************************************************************
 Prototype:
 	uint32_t asm_bitfield_clear(uint32_t dato, uint32_t ancho, uint32_t inicio)

 Parameters:
 	r0: dato a procesar
 	r1: ancho del campo de bits
 	r2: posición del bit menos significativo del campo de bits

 Return: r0: dato con campos de bits cleared
 **************************************************************
*/
.thumb_func
	asm_bitfield_clear:
			// Guarda de seguridad
			cbz r0, bitfield_exit			// Compare and branch if r0 is zero
			cbz r1, bitfield_exit			// Compare and branch if r1 is zero

			mov r3, #1						// Se coloca un 1 en el lsb de r3
			lsl r3, r1						// Se desplaza el 1 según el ancho del campo
			sub r3, #1						// Se genera la máscara. Ej: 001000 - 1 = 000111
			lsl r3, r2						// Se desplaza la máscara según inicio. Ej: 011100
			mvn r3, r3						// Se niega la máscara. Ej: 100011
			and r0, r3						// Se aplica and bit a bit. Se devuelve el resultado en r0

		bitfield_exit:
			bx lr							// Inconditional branch with exchange to Link Register
