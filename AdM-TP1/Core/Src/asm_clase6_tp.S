/*
 * AdM_e2201_e2203_e2207_GE3.S
 *
 *  Created on: Jun 6, 2024
 *      Author: Daniel Albarracin
 *				Roberto Castro Beltran
 *				Edilberto Carvajal
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
 Author: Daniel Albarracin
 *		 Roberto Castro Beltran
 *		 Edilberto Carvajal
 **************************************************************
*/
/*

	uint32_t c_potencia (int16_t * vecIn, uint32_t longitud) {

		uint32_t power = 0;

		if ((vecIn != NULL) & (longitud != 0)){

			for (int32_t i = 0; i < longitud; i++) {

				power += (vecIn [i] * vecIn [i]);

			}

			power /= longitud;
		}

		return power;

	}

*/

.thumb_func

	asm_potencia:
		cbz r0, asm_potencia_exit					@ if pointer is NULL, exit
		cbz r1, asm_potencia_exit					@ if lenght is zero, exit

		push {r4 - r5}								@ stack frame
		mov r5, #0									@ default return value. power = 0
		mov r2, r1									@ r2 = r1, r2 = length

		loop_potencia:
			sub r2, #1								@ r2 = length -1
			ldrsh r3, [r0, r2, lsl #1 ]				@ r3 = *(r0 + r2<<1)
			mov r4, r3								@ r4 = r3
			mla r5, r4, r3, r5						@ r5 = (r3*r4) + r5
			cbz r2,  asm_potencia_before_exit		@if r2 is zero then exit
			b loop_potencia							@then branch to loop

	asm_potencia_before_exit:
		udiv r5, r1									@ r5 = r5 / r1 = addition / Length
	asm_potencia_exit:
		mov r0, r5									@ return value
		pop {r4 - r5}								@ stack frame
		bx lr										@ return

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
 Author: Daniel Albarracin
 *		 Roberto Castro Beltran
 *		 Edilberto Carvajal
 **************************************************************
*/
/*

	uint32_t c_potencia (int16_t * vecIn, uint32_t longitud) {

		uint32_t power = 0;

		if ((vecIn != NULL) & (longitud != 0)){

			for (int32_t i = 0; i < longitud; i++) {

				power += (vecIn [i] * vecIn [i]);

			}

			power /= longitud;
		}

		return power;

	}

*/

.thumb_func

	asm_potencia_DSP:
		cbz r0, asm_potencia_DSP_exit					@ if pointer is NULL, exit
		cbz r1, asm_potencia_DSP_exit					@ if lenght is zero, exit

		push {r4 - r5}									@ stack frame
		mov r5, #0										@ default return value
		mov r2, r1										@ r2 = r1, r2 = length

		loop_potencia_DSP:
			sub r2, #2									@ r2 = length -2
			ldr r3, [r0, r2, lsl #1]					@ r3 = *(r0 + r2<<1)
			mov r4, r3									@ r4 = r3
			smlad r5, r3, r4, r5						@ r5 = x[n]*x[n] + x[n-1]*x[n-1] + r5
			cbz r2,  asm_potencia_DSP_before_exit		@if r2 is zero then exit
			b loop_potencia_DSP							@then branch to loop

	asm_potencia_DSP_before_exit:
		udiv r5, r1										@ r5 = r5 / r1 = addition / Length
	asm_potencia_DSP_exit:
		mov r0, r5										@ return value
		pop {r4 - r5}									@ stack frame
		bx lr											@ return

/*
 **************************************************************
 Prototype:
 	void asm_medDif (int8_t *e, int8_t *x, int8_t *y, uint16_t longitud)

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
 Author: Daniel Albarracin
 *		 Roberto Castro Beltran
 *		 Edilberto Carvajal
 **************************************************************
*/
/*

	void c_medDif(int8_t * e, int8_t *x, int8_t *y, uint16_t longitud) {

		if ((e != NULL) & (x != NULL) & (y != NULL) & (longitud != 0)){

			for (int32_t i = 0; i < longitud; i++) {

				e[i] = (x[i] - y [i]) / 2;

			}
		}

		return;
	}

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
 Author: Daniel Albarracin
 *		 Roberto Castro Beltran
 *		 Edilberto Carvajal
 **************************************************************
*/
/*

	void c_medDif(int8_t * e, int8_t *x, int8_t *y, uint16_t longitud) {

		if ((e != NULL) & (x != NULL) & (y != NULL) & (longitud != 0)){

			for (int32_t i = 0; i < longitud; i++) {

				e[i] = (x[i] - y [i]) / 2;

			}
		}

		return;
	}

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
 Author: Daniel Albarracin
 *		 Roberto Castro Beltran
 *		 Edilberto Carvajal
 **************************************************************
*/
/*

	void c_eco(int16_t * signal, int16_t *eco, uint32_t longitud) {

		// offset[index] = ((offset[ms] * fs[Hz]) / 1000) - 1
		uint32_t offset = ((20 * 44100) / 1000) - 1;	// Initial Eco Sample = 881

		int16_t sample = 0;

		if ((signal != NULL) & (longitud != 0)){

			memset(eco, '\0', (longitud * sizeof(int16_t)));

			for (uint32_t i = 0; i < longitud; i++) {

				sample = signal [i];

				if (i >= offset) {

					sample += (signal[i - offset] / 2);

				}

				eco[i] = sample;

			}

		}

		return;
	}

*/

.thumb_func

	asm_eco:
		cbz r0, asm_eco_exit			@ if pointer is NULL, exit
		cbz r1, asm_eco_exit			@ if pointer is NULL, exit
		cbz r2, asm_eco_exit			@ if lenght is zero, exit

		push {r4 - r8}					@ stack frame
		mov r3, #881					@ Echo start index = ((offset[ms] * fs[Hz]) / 1000) - 1
										@ 				   = ((20ms * 44100Hz) / 1000) - 1 = 881
		mov r4, #0						@ sample = 0
		mov r5, #0						@ n = 0

	loop_asm_eco:
		cmp r5, r2
		beq asm_eco_exit					@ n == lenght

		ldrsh r4, [r0, r5, lsl #1]			@ r4 = *(r0 + r5 << 1)

		cmp r5, r3
		blt no_eco

		sub r6, r5, r3						@ r6 = r5 - r3 (r6 = i - offset)
		ldrsh r7, [r0, r6, lsl #1]			@ r7 = *(r0 + r6 << 1)
		asr r7, r7, #1						@ r7 = r7 / 2
		add r4, r4, r7						@ r4 += (signal[i - offset] / 2)

	no_eco:
		strh r4, [r1, r5, lsl #1]			@ *(r1 + r5 << 1) = r4
		add r5, #1							@ n = n + 1

		b loop_asm_eco

	asm_eco_exit:
		pop {r4 - r8}						@ stack frame
		bx lr								@ return




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
 Author: Daniel Albarracin
 *		 Roberto Castro Beltran
 *		 Edilberto Carvajal
 **************************************************************
*/
/*

	void c_eco(int16_t * signal, int16_t *eco, uint32_t longitud) {

		// offset[index] = ((offset[ms] * fs[Hz]) / 1000) - 1
		uint32_t offset = ((20 * 44100) / 1000) - 1;	// Initial Eco Sample = 81

		int16_t sample = 0;

		if ((signal != NULL) & (longitud != 0)){

			memset(eco, '\0', (longitud * sizeof(int16_t)));

			for (uint32_t i = 0; i < longitud; i++) {

				sample = signal [i];

				if (i >= offset) {

					sample += (signal[i - offset] / 2);

				}

				eco[i] = sample;

			}

		}

		return;
	}

*/

.thumb_func

	asm_eco_DSP:
		cbz r0, asm_eco_DSP_exit			@ if pointer is NULL, exit
		cbz r1, asm_eco_DSP_exit			@ if pointer is NULL, exit
		cbz r2, asm_eco_DSP_exit			@ if lenght is zero, exit

		push {r4 - r8}					@ stack frame
		mov r3, #881					@ Echo start index = ((offset[ms] * fs[Hz]) / 1000) - 1
										@ 				   = ((20ms * 44100Hz) / 1000) - 1 = 881
		mov r4, #0						@ sample = 0
		mov r5, #0						@ n = 0
		mov r8, #0						@ aux = 0
		sub r2, #1						@ n = n - 1

	loop_asm_eco_DSP:
		cmp r5, r2
		beq asm_eco_DSP_exit				@ n == lenght

		cmp r5, r3
		bge apply_eco

		b no_eco_DSP

	apply_eco:
		ldr r4, [r0, r5, lsl #1]			@ r4 = *(r0 + r5 << 1)

		sub r6, r5, r3						@ r6 = r5 - r3 (r6 = i - offset)
		ldr r7, [r0, r6, lsl #1]			@ r7 = *(r0 + r6 << 1)
		shadd16 r7, r7, r8					@ (r7 [31 - 16] + 0) / 2 || (r7 [15 - 0] + 0) / 2
		sadd16 r4, r4, r7					@ r4 [31 - 16] + r7 [31 - 16] || r4 [15 - 0] + r7 [15 - 0]

		str r4, [r1, r5, lsl #1]
		add r5, #2							@ n = n + 1

		b loop_asm_eco_DSP


	no_eco_DSP:
		ldrsh r4, [r0, r5, lsl #1]			@ r4 = *(r0 + r5 << 1)
		strh r4, [r1, r5, lsl #1]			@ *(r1 + r5 << 1) = r4
		add r5, #1							@ n = n + 1

		b loop_asm_eco_DSP


	asm_eco_DSP_exit:
		pop {r4 - r8}				@ stack frame
		bx lr						@ return



