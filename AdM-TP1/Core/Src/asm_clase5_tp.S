/*
 * asm_clase5_tp.S
 *
 *  Created on: May 23, 2024
 *      Author: ddalbarracin
 */

.text				@indica inicio de codigo ejecutable. va a flash
.syntax unified 	@estare utilizando UAL
.thumb				@estare utilizando modo thumb y thumb-2

.global asm_pack32to16		@AdM-TP2-Ej1
.global asm_max				@AdM-TP2-Ej2
.global asm_downSample		@AdM-TP2-Ej3
.global asm_invertir		@AdM-TP2-Ej4


/*
 **************************************************************
 Prototype:
 	void asm_pack32to16(int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud)

 Brief:
 	Realizar una función que reciba un vector de números signados de 32 bits y los
	“empaquete” en otro vector de 16 bits, guardando la parte alta (MSB).

 Sugerencia:
 	utilice números mayores a 100.000 y menores a -100.000 en el vector de
	pruebas para poder ver los resultados.

 Parameters:
 	r0: Input Vector Address
 	r1: Output Vector Address
 	r2: Vector's lenght

 Return: r0: None

 Created on: May 25, 2024
 Author: Dante Mendoza
 SIU: e2206
 **************************************************************
*/

.thumb_func

	asm_pack32to16:
	/*
		cbz r0, asm_pack32to16_exit
		cbz r1, asm_pack32to16_exit

	asm_pack32to16_exit:
		bx lr
	*/
    PUSH {R4, R5, LR}            // Guardar registros de uso general y LR

    CMP R2, #0                   // Comprobar si la longitud es 0
    BEQ end                      // Si es 0, terminar

    loop:
        LDR R3, [R0], #4             // Cargar el siguiente valor de 32 bits y post-incrementar vectorIn
        MOV R4, R3, LSR #16          // Desplazar R3 16 bits a la derecha (obtener los 16 bits más significativos)
        STRH R4, [R1], #2            // Almacenar en vectorOut como un valor de 16 bits y post-incrementar vectorOut

        SUBS R2, R2, #1              // Decrementar la longitud
        BNE loop                     // Repetir hasta que R2 sea 0

    end:
        POP {R4, R5, LR}             // Restaurar registros
        BX LR                        // Retornar de la función


/*
 **************************************************************
 Prototype:
 	uint32_t asm_max(int32_t * vectorIn, uint32_t longitud)

 Brief:
 	Realizar una función que reciba un vector de números signados de 32 bits
 	y devuelva la posición del máximo del vector.

 Parameters:
 	r0: Input Vector Address
 	r1: Vector's lenght

 Return: r0: None

 Created on: May 25, 2024
 Author: Daniel David Albarracin
 SIU: e2207
 **************************************************************
*/
/*
	uint32_t max(int32_t *vectorIn, uint32_t longitud){

		uint32_t index = 0;

		uint32_t posMax = 0;
		int32_t MaxValue;

		MaxValue = vectorIn[longitud-1];

		for (index = longitud-1; index > 0; index --){
			if (vectorIn[index-1] > MaxValue){
				posMax = index-1;
				MaxValue = vectorIn[index-1];
			}
		}

		return(posMax);

	}
*/
.thumb_func
	asm_max:
		push {r4-r5}				@ stack frame
		mov r4, #0					@ default return value
		cbz r0, asm_max_exit		@ if pointer is NULL, exit
		cbz r1, asm_max_exit		@ if lenght is zero, exit

		mov r2, r1					@ r2 = index = longitud
		sub r2, #1					@ r2 = index  = longitud -1
		ldr r3, [r0, r1, lsl #2] 	@ r3 = *(r0 + r1<<2) => MaxValue = vectorIn[longitud-1]
		mov r4, r2					@ r4 = posMax = (longitud -1)
	loop_max:
		sub r2, #1					@ r2 = index - 1
		ldr r5, [r0, r2, lsl #2]	@ r5 = *(ro + r2) = vectorIn[index-1]
		cmp r5, r3					@ r5 - r3 > 0 ?
		itt ge
		movge r3, r5				@ if r5>r3 then r3 = r5 => MaxValue = vectorIn[index-1]
		movge r4, r2				@ if r5>r3 then r4 = r2 => posMax = (index - 1)
		cbz r2, asm_max_exit		@ compare and branch if z flag is set. if r1 is zero branch to exit
		b loop_max					@ inconditional branch to loop_max


	asm_max_exit:
		mov r0, r4					@ return Position
		pop {r4 - r5}				@ stack frame
		bx lr						@ return

/*
 **************************************************************
 Prototype:
 	void asm_downSample (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N)

 Brief:
 	Realizar una función que reciba un vector de muestras signadas de
 	32 bits y descarte una de cada N muestras. Si se impone en la
 	aplicación que longitud siempre es múltiplo de N, ¿cómo determinaría
 	la longitud de vectorOut?

 Parameters:
 	r0: Input Vector Address
 	r1: Output Vector Address
 	r2: Vector's lenght
 	r3: N value

 Return: r0: None

 Created on: May 25, 2024
 Author: Edilberto Carvajal Peñafiel
 SIU: e2203
 **************************************************************
*/
/*
	void downSample(int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N){
    	uint32_t outIndex = 0;
    	for (uint32_t i = 0; i < longitud; i++) {
        	if ((i + 1) %N != 0) {
            	vectorOut[outIndex++] = vectorIn[i];
        	}
    	}
	}
*/

.thumb_func
	asm_downSample:
		push {r4-r8}						@ stack frame
		cbz r0, asm_downSple_exit			@ if vectorIn pointer is NULL, exit
		cbz r1, asm_downSple_exit			@ if vectorOut pointer is NULL, exit
		cbz r2, asm_downSple_exit			@ if length pointer is zero, exit
		cbz r3, asm_downSple_exit			@ if N value is zero, exit

		mov r4, #0                          @ outIndex = 0
   	    mov r5, #0                  		@ i = 0

	loop_smpl:
		cmp r5, r2							@ Compare i with longitud
		bge asm_downSple_exit				@ If i >= longitud, exit the loop

		add r6, r5, #1						@ Calculate (i + 1) / N
		udiv r6, r6, r3

		mls r6, r6, r3, r5					 @ Calculate the remainder of (i + 1) % N

		cmp r6, #0							 @ Compare the remainder with 0

		it ne								 @ If-Then: execute the next instruction if the remainder is not equal to 0
		ldrne r6, [r0, r5, LSL #2]			 @ If the remainder is not 0, load vectorIn[i]

		it ne
		strne r6, [r1, r4, LSL #2]			 @ If the remainder is not 0, store it in vectorOut[outIndex]

		it ne
		addne r4, r4, #1					 @ If the remainder is not 0, increment outIndex

		add r5, r5, #1						 @ Increment i

		b loop_smpl							 @ Jump back to the start of the loop

	asm_downSple_exit:
		pop {r4-r8}							 @ stack frame
		bx lr								 @ return

/*
 **************************************************************
 Prototype:
 	void asm_invertir(uint16_t * vector, uint32_t longitud)

 Brief:
 	Realizar una función que reciba un vector de muestras no
 	signadas de 16 bits e invierta su orden.

 Parameters:
 	r0: Input Vector Address
 	r1: Vector's lenght

 Return: r0: None

 Created on: May 25, 2024
 Author: Daniel David Albarracin
 SIU: e2207
 **************************************************************
*/
/*
	void invertir(uint16_t *vector, uint32_t longitud) {

		uint16_t firstValue, lastValue;
		uint16_t index;

		for (index = 0; index < (longitud / 2); index++) {
			firstValue = vector[index];
			lastValue = vector[(longitud - 1) - index];
			vector[index] = lastValue;
			vector[(longitud - 1) - index] = firstValue;
		}

		return;

	}
*/

.thumb_func
	asm_invertir:
		push {r4 - r7}					@ stack frame
		cbz r0, asm_invrtr_exit			@ if pointer is NULL, exit
		cbz r1, asm_invrtr_exit			@ if lenght is zero, exit

		mov r2, #0						@ r2 = index = 0
		mov r6, r1						@ r6 = longitud
		lsr r3, r1, #1					@ r3 = r1/2 = longitud/2
		sub r6, #1						@ r6 = (r1 - 1) => r6 = (longitud - 1)

	loop_invrtr:

		lsl r7, r2, #1					@ r7 = (r2*2)
		ldrh r4, [r0, r7] 				@ r4 = *(r0 + r2*2) => firstValue = vector[index]
		ldrh r5, [r0, r6, lsl #1]		@ r5 = *(r0 + r6<<1) => lastValue = vector[(longitud - 1)]
		strh r5, [r0, r7]				@ *(r0 + r7) = r5 => rvector[index] = lastValue;
		strh r4, [r0, r6, lsl #1]		@ *(r0 + r6<<1) = r4 => vector[(longitud - 1) - index] = firstValue
		add r2, #1						@ r2 = r2 + 1 = indice + 1
		sub r6, #1						@ r6 = r6 -1 => indice + 1
		cmp r2, r3						@ compare r2 with r3.
		beq asm_invrtr_exit				@ if (r2 - r3 == 0) then branch to exit
		b loop_invrtr					@ else branch to loop

	asm_invrtr_exit:
		pop {r4 - r7}					@ stack frame
		bx lr							@ return
