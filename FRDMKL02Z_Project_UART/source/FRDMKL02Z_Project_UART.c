/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    FRDMKL02Z_Project_UART.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL02Z4.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h" //resiente

#include "sdk_hal_uart0.h"

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
/*
 * @brief   Application entry point.
 */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

int dance(unsigned char valor);

//nuevo desde aca
void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 800000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}//hasta aca





int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();



    /* Define the init structure for the output LED pin*/
        gpio_pin_config_t led_config = {
            kGPIO_DigitalOutput, 0,
        };

        /* Board pin, clock, debug console init */
        //BOARD_InitPins();
       // BOARD_BootClockRUN();
        BOARD_InitDebugConsole();

        /* Print a note to terminal. */
        PRINTF("\r\n GPIO Driver example\r\n");
        PRINTF("Encender leds con las letras del teclado\r\n "
        		"V: encender luz verde\r\n"
        		"v: apagar luz verde\r\n"
        		"R: encender luz roja\r\n"
        		"r: apagar luz roja\r\n"
        		"A: encender luz azul\r\n"
        		"a: apagar luz azul\r\n"
        		"M: encender luz morada\r\n"
        		"m: apagar luz morada\r\n"
        		"los numeros del 1-9 tienen diferentes configuraciones de luces parpadeante\r\n");

        /* Init output LED GPIO. */ // led rojo PTB6
        GPIO_PinInit(GPIOB, 6U, &led_config);
        //GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_RED_GPIO_PIN, &led_config);

        /* Init output LED GPIO. */ // led verde PTB7
        //GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GREEN_GPIO_PIN, &led_config);
        GPIO_PinInit(GPIOB, 7U, &led_config);


        /* Init output LED GPIO. */ // led azul PTB10
        //GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);
        GPIO_PinInit(GPIOB, 10U, &led_config);



#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    (void)uart0Inicializar(115200);
    GPIO_PortSet(GPIOB, 1u << 6U);
    GPIO_PortSet(GPIOB, 1u << 7U);
    GPIO_PortSet(GPIOB, 1u << 10U);

    while(1) {
    	status_t status;
    	uint8_t nuevo_byte;

    	if(uart0NuevosDatosEnBuffer()>0){
    		status=uart0LeerByteDesdeBufferCircular(&nuevo_byte);
    		if(status==kStatus_Success){
    			printf("dato:%c\r\n",nuevo_byte);
    		}else{
    			printf("error\r\n");
    		}
    	}

    	switch (nuevo_byte)

    	{
    	case 'R':
    		GPIO_PortSet(GPIOB, 1u << 7U);
    		GPIO_PortSet(GPIOB, 1u << 10U);
    		GPIO_PortClear(GPIOB, 1u << 6U);

    		printf("Luz roja encendida\r\n");

    		nuevo_byte = 0;

    	break;
    	case 'r':
    		GPIO_PortSet(GPIOB, 1u << 6U);
    		printf("Luz roja apagada\r\n");

    		nuevo_byte = 0;

    	break;
    	case 'V':
    		GPIO_PortSet(GPIOB, 1u << 6U);
    		GPIO_PortSet(GPIOB, 1u << 10U);
    		GPIO_PortClear(GPIOB, 1u << 7U);

    		printf("Luz verde encendida\r\n");

    		nuevo_byte = 0;

    	break;
    	case 'v':
    		GPIO_PortSet(GPIOB, 1u << 7U);
    		printf("Luz verde apagada\r\n");


    		nuevo_byte = 0;
    	break;
    	case 'A':
    		GPIO_PortSet(GPIOB, 1u << 6U);
    		GPIO_PortSet(GPIOB, 1u << 7U);
    		GPIO_PortClear(GPIOB, 1u << 10U);

    		printf("Luz azul encendida\r\n");

    		nuevo_byte = 0;

    	break;
    	case 'a':
    		GPIO_PortSet(GPIOB, 1u << 10U);
    		printf("Luz azul apagada\r\n");

    		nuevo_byte = 0;
    		break;
    	case 'M':

    		GPIO_PortSet(GPIOB, 1u << 7U);
    		GPIO_PortClear(GPIOB, 1u << 6U);
    		GPIO_PortClear(GPIOB, 1u << 10U);

    		printf("Luz morada encendida\r\n");

    		nuevo_byte = 0;

    		break;
    	case 'm':
    		GPIO_PortSet(GPIOB, 1u << 6U);
    		GPIO_PortSet(GPIOB, 1u << 10U);
    		printf("Luz morada apagada\r\n");

    		nuevo_byte = 0;
    		break;

    	default:
    		if (nuevo_byte > 48 && nuevo_byte < 58){
    			dance(nuevo_byte);
    		}
    		nuevo_byte = 0;
    	}


    }
    return 0 ;
}

int dance(unsigned char valor){
	unsigned char a = 0;
	switch (valor){
	case '1':
		GPIO_PortSet(GPIOB, 1u << 6U);
		GPIO_PortSet(GPIOB, 1u << 10U);
		printf("Luz verde parpadeante\r\n");
		for (a=1;a<10;a++){
		GPIO_PortClear(GPIOB, 1u << 7U);
		delay();
		GPIO_PortSet(GPIOB, 1u << 7U);
		delay();
		}

		printf("Luz verde parpadeante terminado\r\n");
	break;
	case '2':
		GPIO_PortSet(GPIOB, 1u << 7U);
		GPIO_PortSet(GPIOB, 1u << 10U);
		printf("Luz roja parpadeante\r\n");
		for (a=1;a<10;a++){
			GPIO_PortClear(GPIOB, 1u << 6U);
			delay();
			GPIO_PortSet(GPIOB, 1u << 6U);
			delay();
		}

		printf("Luz roja parpadeante terminado\r\n");
		break;
	case '3':
		GPIO_PortSet(GPIOB, 1u << 7U);
		GPIO_PortSet(GPIOB, 1u <<6);
		printf("Luz azul parpadeante\r\n");
		for (a=1;a<10;a++){
			GPIO_PortClear(GPIOB, 1u << 10U);
			delay();
			GPIO_PortSet(GPIOB, 1u << 10U);
			delay();
		}

		printf("Luz azul parpadeante terminado\r\n");
		break;
	case '4':
		GPIO_PortSet(GPIOB, 1u << 6U);
		printf("Luz verde y azul parpadeante\r\n");
		for (a=1;a<10;a++){
			GPIO_PortClear(GPIOB, 1u << 7U);
			GPIO_PortClear(GPIOB, 1u << 10U);
			delay();
			GPIO_PortSet(GPIOB, 1u << 7U);
			GPIO_PortSet(GPIOB, 1u << 10U);
			delay();
		}

		printf("Luz verde y azul parpadeante terminado\r\n");
		break;
	case '5':
		GPIO_PortSet(GPIOB, 1u << 7U);
		printf("Luz morada parpadeante\r\n");
		for (a=1;a<10;a++){
			GPIO_PortClear(GPIOB, 1u << 6U);
			GPIO_PortClear(GPIOB, 1u << 10U);
			delay();
			GPIO_PortSet(GPIOB, 1u << 6U);
			GPIO_PortSet(GPIOB, 1u << 10U);
			delay();
		}

		printf("Luz morada parpadeante terminado\r\n");

		break;
	case '6':
		GPIO_PortSet(GPIOB, 1u << 7U);
		printf("Luz morada y roja parpadeante\r\n");
		for (a=1;a<10;a++){
			GPIO_PortClear(GPIOB, 1u << 6U);
			GPIO_PortClear(GPIOB, 1u << 10U);
			delay();
			GPIO_PortSet(GPIOB, 1u << 6U);
			GPIO_PortSet(GPIOB, 1u << 10U);
			delay();

			GPIO_PortClear(GPIOB, 1u << 6U);
			delay();
			GPIO_PortSet(GPIOB, 1u << 6U);
			delay();

		}

		printf("Luz morada y roja parpadeante terminado\r\n");
		break;
	case '7':
		GPIO_PortSet(GPIOB, 1u << 6U);
		GPIO_PortSet(GPIOB, 1u << 7U);
		GPIO_PortSet(GPIOB, 1u << 10U);
		printf("Luz RGB parpadeante\r\n");
		for (a=1;a<6;a++){
			GPIO_PortClear(GPIOB, 1u << 6U);
			delay();
			GPIO_PortSet(GPIOB, 1u << 6U);
			delay();

			GPIO_PortClear(GPIOB, 1u << 7U);
			delay();
			GPIO_PortSet(GPIOB, 1u << 7U);
			delay();

			GPIO_PortClear(GPIOB, 1u << 10U);
			delay();
			GPIO_PortSet(GPIOB, 1u << 10U);
			delay();

		}

		printf("Luz RGB parpadeante terminado\r\n");

		break;
	case '8':
		GPIO_PortSet(GPIOB, 1u << 6U);
		GPIO_PortSet(GPIOB, 1u << 7U);
		GPIO_PortSet(GPIOB, 1u << 10U);
		printf("Luz blanca y roja parpadeante\r\n");
		for (a=1;a<10;a++){
			GPIO_PortClear(GPIOB, 1u << 6U);
			GPIO_PortClear(GPIOB, 1u << 7U);
			GPIO_PortClear(GPIOB, 1u << 10U);
			delay();
			GPIO_PortSet(GPIOB, 1u << 6U);
			GPIO_PortSet(GPIOB, 1u << 7U);
			GPIO_PortSet(GPIOB, 1u << 10U);
			delay();

			GPIO_PortClear(GPIOB, 1u << 6U);
			delay();
			GPIO_PortSet(GPIOB, 1u << 6U);
			delay();

		}

		printf("Luz blanca y roja parpadeante terminado\r\n");
		break;
	case '9':
		GPIO_PortSet(GPIOB, 1u << 6U);
				GPIO_PortSet(GPIOB, 1u << 7U);
				GPIO_PortSet(GPIOB, 1u << 10U);
				printf("Luz blanca parpadeante\r\n");
				for (a=1;a<10;a++){
					GPIO_PortClear(GPIOB, 1u << 6U);
					GPIO_PortClear(GPIOB, 1u << 7U);
					GPIO_PortClear(GPIOB, 1u << 10U);
					delay();
					GPIO_PortSet(GPIOB, 1u << 6U);
					GPIO_PortSet(GPIOB, 1u << 7U);
					GPIO_PortSet(GPIOB, 1u << 10U);
					delay();


				}

				printf("Luz blanca parpadeante terminado\r\n");
		break;

	}
}

