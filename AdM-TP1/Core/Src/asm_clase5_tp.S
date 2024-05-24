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
 **************************************************************
*/

.thumb_func
	asm_pack32to16:




	asm_pack32to16_exit:
		bx lr



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
 **************************************************************
*/

.thumb_func
	asm_max:




	asm_max_exit:
		bx lr


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
 **************************************************************
*/

.thumb_func
	asm_downSample:




	asm_downSample_exit:
		bx lr





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
 **************************************************************
*/

.thumb_func
	asm_invertir:




	asm_invertir_exit:
		bx lr





