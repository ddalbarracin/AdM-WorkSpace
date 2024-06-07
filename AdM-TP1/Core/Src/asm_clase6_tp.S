/*
 * asm_clase6_tp.S
 *
 *  Created on: Jun 6, 2024
 *      Author: ddalbarracin
 */


.text				@indica inicio de codigo ejecutable. va a flash
.syntax unified 	@estare utilizando UAL
.thumb				@estare utilizando modo thumb y thumb-2

.global asm_potencia				@AdM-TP2-Ej1-asm
.global asm_potencia_DSP			@AdM-TP2-Ej1-asmDSP
.global asm_medDif					@AdM-TP2-Ej2-asm
.global asm_medDif_DSP				@AdM-TP2-Ej2-asmDSP
.global asm_eco						@AdM-TP2-Ej3-asm
.global asm_eco_DSP					@AdM-TP2-Ej3-asmDSP


/*
 **************************************************************
 Prototype:
 	uint32_t asm_potencia(int16_t * vecIn, uint32_t longitud)

 Brief:
 	Implementar una función que calcule la potencia de una señal discreta x[n]
 	con datos signados de 16 bits aplicando la siguiente ecuación:


 Sugerencia:
 	N: longitud de x
 	P es un número sin signo de 32 bits

 Parameters:
 	r0: Input Vector Address
 	r1: Vector's lenght

 Return:
 	r0: Output Vector Potencia

 Created on: Jun 6, 2024
 Author:
 SIU: e22
 **************************************************************
*/
/*
	aqui funcion

*/

.thumb_func

	asm_potencia:
		push {r4 - r8}						@ stack frame
		mov r4, #0							@ default return value
		cbz r0, asm_potencia_exit			@ if pointer is NULL, exit
		cbz r1, asm_potencia_exit			@ if lenght is zero, exit

	loop_potencia:



	asm_potencia_exit:
		mov r0, r4							@ return value
		pop {r4 - r8}						@ stack frame
		bx lr								@ return







/*
 **************************************************************
 Prototype:
 	uint32_t asm_potencia_DSP(int16_t * vecIn, uint32_t longitud)

 Brief:
 	Implementar una función que calcule la potencia de una señal discreta x[n]
 	con datos signados de 16 bits aplicando la siguiente ecuación:


 Sugerencia:
 	N: longitud de x
 	P es un número sin signo de 32 bits

 Parameters:
 	r0: Input Vector Address
 	r1: Vector's lenght

 Return:
 	r0: Output Vector Potencia

 Created on: Jun 6, 2024
 Author:
 SIU: e22
 **************************************************************
*/
/*
	aqui funcion

*/

.thumb_func

	asm_potencia_DSP:
		push {r4 - r8}							@ stack frame
		mov r4, #0								@ default return value
		cbz r0, asm_potencia_DSP_exit			@ if pointer is NULL, exit
		cbz r1, asm_potencia_DSP_exit			@ if lenght is zero, exit

	loop_potencia_DSP:



	asm_potencia_DSP_exit:
		mov r0, r4							@ return value
		pop {r4 - r8}						@ stack frame
		bx lr								@ return






/*
 **************************************************************
 Prototype:
 	void asm_medDif(int8_t *e, int8_t *x, int8_t *y, uint16_t longitud)

 Brief:
 	Implementar una función que calcule el vector de diferencia
 	media entre dos señales discretas x[n] e y[n] con datos
 	signados de 8 bits y de igual cantidad de elementos


 Sugerencia:
 	Donde e[n] es un vector con datos de 8 bits con signo


 Parameters:
 	r0: Output Vector e
 	r1: Input Vector x
 	r2: Input Vector y
 	r3: Vector's lenght

 Return:
 	r0: None

 Created on: Jun 6, 2024
 Author:
 SIU: e22
 **************************************************************
*/
/*
	aqui funcion en C

*/

.thumb_func

	asm_medDif:
		push {r4 - r8}					@ stack frame
		cbz r0, asm_medDif_exit			@ if pointer is NULL, exit
		cbz r1, asm_medDif_exit			@ if pointer is NULL, exit
		cbz r2, asm_medDif_exit			@ if pointer is NULL, exit
		cbz r3, asm_medDif_exit			@ if lenght is zero, exit



	loop_asm_medDif:



	asm_medDif_exit:
		pop {r4 - r8}					@ stack frame
		bx lr							@ return






/*
 **************************************************************
 Prototype:
 	void asm_medDif_DSP(int8_t * e, int8_t *x, int8_t *y, uint16_t longitud)

 Brief:
 	Implementar una función que calcule el vector de diferencia
 	media entre dos señales discretas x[n] e y[n] con datos
 	signados de 8 bits y de igual cantidad de elementos


 Sugerencia:
 	Donde e[n] es un vector con datos de 8 bits con signo


 Parameters:
 	r0: Output Vector e
 	r1: Input Vector x
 	r2: Input Vector y
 	r3: Vector's lenght

 Return:
 	r0: None

 Created on: Jun 6, 2024
 Author:
 SIU: e22
 **************************************************************
*/
/*
	aqui funcion en C

*/

.thumb_func

	asm_medDif_DSP:
		push {r4 - r8}						@ stack frame
		cbz r0, asm_medDif_DSP_exit			@ if pointer is NULL, exit
		cbz r1, asm_medDif_DSP_exit			@ if pointer is NULL, exit
		cbz r2, asm_medDif_DSP_exit			@ if pointer is NULL, exit
		cbz r3, asm_medDif_DSP_exit			@ if lenght is zero, exit



	loop_asm_medDif_DSP:



	asm_medDif_DSP_exit:
		pop {r4 - r8}						@ stack frame
		bx lr								@ return






/*
 **************************************************************
 Prototype:
 	void asm_eco(int16_t * signal, int16_t *eco, uint32_t longitud)

 Brief:
 	Realizar una función que recibe un vector de 4096 valores de
 	16 bits signados, que corresponden a muestras de audio tomadas
 	a una tasa de muestreo de 44.100 muestras/s. La función debe
 	introducir un “eco” que consiste en adicionar a la señal original,
 	la propia señal original dividida por dos y atrasada en 20 mseg

 Sugerencia:
 	Puede utilizar la función rand() para generar los valores del
 	vector de muestra

 Parameters:
 	r0: Input Vector Signal
 	r1: Input Vector Eco
 	r2: Vector's lenght

 Return:
 	r0: None

 Created on: Jun 6, 2024
 Author:
 SIU: e22
 **************************************************************
*/
/*
	aqui funcion en C

*/

.thumb_func

	asm_eco:
		push {r4 - r8}					@ stack frame
		cbz r0, asm_eco_exit			@ if pointer is NULL, exit
		cbz r1, asm_eco_exit			@ if pointer is NULL, exit
		cbz r2, asm_eco_exit			@ if lenght is zero, exit



	loop_asm_eco:



	asm_eco_exit:
		pop {r4 - r8}					@ stack frame
		bx lr							@ return




/*
 **************************************************************
 Prototype:
 	void asm_eco_DSP(int16_t * signal, int16_t *eco, uint32_t longitud)


 Brief:
 	Realizar una función que recibe un vector de 4096 valores de
 	16 bits signados, que corresponden a muestras de audio tomadas
 	a una tasa de muestreo de 44.100 muestras/s. La función debe
 	introducir un “eco” que consiste en adicionar a la señal original,
 	la propia señal original dividida por dos y atrasada en 20 mseg

 Sugerencia:
 	Puede utilizar la función rand() para generar los valores del
 	vector de muestra

 Parameters:
 	r0: Input Vector Signal
 	r1: Input Vector Eco
 	r2: Vector's lenght

 Return: r0: None

 Created on: Jun 6, 2024
 Author:
 SIU: e22
 **************************************************************
*/
/*
	aqui funcion en C

*/

.thumb_func

	asm_eco_DSP:
		push {r4 - r8}
		cbz r0, asm_eco_DSP_exit			@ if pointer is NULL, exit
		cbz r1, asm_eco_DSP_exit			@ if pointer is NULL, exit
		cbz r2, asm_eco_DSP_exit			@ if lenght is zero, exit



	loop_asm_eco_DSP:



	asm_eco_DSP_exit:
		pop {r4 - r8}
		bx lr						@ return



