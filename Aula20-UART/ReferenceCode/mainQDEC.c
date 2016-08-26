/**
 * \file
 *
 * \brief lcd controller ili9325 example.
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage lcd controller ili9325 Example
 *
 * \section Purpose
 *
 * This example demonstrates how to configure lcd controller ili9325
 * to control the LCD on the board.
 *
 * \section Requirements
 *
 * This package can be used with SAM3S, SAM3S-EK2 and SAM4S evaluation kits.
 *
 * \section Description
 *
 * This example first configure ili9325 for access the LCD controller,
 * then initialize the LCD, finally draw some text, image, basic shapes (line,
 * rectangle, circle) on LCD.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board.
 * -# Some text, image and basic shapes should be displayed on the LCD.
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "smc.h"
#include "pmc.h"
#include "pio.h"

#define TC_CMR_WAVSEL(value) ((TC_CMR_WAVSEL_Msk & ((value) << TC_CMR_WAVSEL_Pos)))
#define TC_CMR_ACPC(value) ((TC_CMR_ACPC_Msk & ((value) << TC_CMR_ACPC_Pos)))
#define TC_CMR_LDRA(value) ((TC_CMR_LDRA_Msk & ((value) << TC_CMR_LDRA_Pos)))
#define TC_CMR_LDRB(value) ((TC_CMR_LDRB_Msk & ((value) << TC_CMR_LDRB_Pos)))


/* =========  PROTÓTIPO DE FUNÇÕES    =====================*/

static void configure_console(void);
static void configure_encoder(void);
static void configure_leds(void);
static void configure_lcd(void);
static void configure_tc(void);
void desenhaCabecalho(void);
void clearLine(uint8_t posX, uint8_t posY);


/* ============== DEFINES =============================*/

/* Chip select number to be set */
#define ILI9325_LCD_CS      1

//Blue LED
#define PIN_LED_BLUE		19
#define PIN_LED_BLUE_PIO	PIOA
#define PIN_LED_BLUE_ID		ID_PIOA
#define PIN_LED_BLUE_MASK	(0x80000)	//19 in Hexadecimal
//#define PIN_LED_BLUE_MASK	(1u << PIN_LED_BLUE)  //u: unsigned


/** IRQ priority for PIO (The lower the value, the greater the priority) */
#define IRQ_PRIOR_PIO    0

#define Freq_Init_Blink 4	//Hz

/** Defines para configuração do Quadrature Decoder  **/


#define STRING_EOL    "\r"
#define STRING_HEADER "--QUADRATURE DECODER--\r\n" \
"-- "BOARD_NAME " --\r\n" \
"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

//#define PIN_TC_CAPTURE_PHA PIO_PA0_IDX
//#define PIN_TC_CAPTURE_PHB PIO_PA1_IDX
//#define PIN_TC_CAPTURE_INDX PIO_PA16_IDX
//#define TC_CAPTURE_TIMER_SELECTION TC_CMR_TCCLKS_XC0
	

#define CHANNEL0 0
#define CHANNEL1 1
#define CHANNEL2 2



/* =========		VARIÁVEIS	   =====================*/

struct ili9325_opt_t g_ili9325_display_opt;

/** Capture status*/
//static uint32_t gs_ul_captured_pulses;
static uint32_t prev_cv0=0;


/* =========		 FUNÇÂO MAIN	 =====================*/

/**
 * \brief Application entry point for smc_lcd example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void) {
		
	char charVector[50];
	 
	int teste = 0;
	uint32_t velocidade = 0;
	
	//Variáveis utilizadas pelo encoder
	uint8_t key;
	TcChannel *tc_channel;
	int cv0 = 0, cv1 = 0;
		
	
	sysclk_init();
	board_init();
	
	/* Initialize debug console */
	configure_console();
	
	/* Configura o Encoder de Quadratura */
	configure_encoder();
	
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/** Configura o timer */
	configure_tc();

	/* Configura Leds */
	configure_leds();

	/* Enable peripheral clock */
	pmc_enable_periph_clk(ID_SMC);

	/* Configuração LCD */
	configure_lcd();

		
	desenhaCabecalho();

	while (1) {
		teste++;
		
		clearLine(5, 100);
		sprintf(charVector, "teste=%d", teste);
		printf(charVector);
		ili9325_set_foreground_color(COLOR_DARKVIOLET);
		ili9325_draw_string(5, 100, (uint8_t *)charVector);
		
		cv0  = TC0->TC_CHANNEL[CHANNEL0].TC_CV;        // latch PhaseA/B counter value
		cv1  = TC0->TC_CHANNEL[CHANNEL1].TC_CV;        // latch PhaseA/B counter value
		
//		gpio_toggle_pin(LED0_GPIO);              //blink rate not visible but just in case required..
		if(prev_cv0 != cv0)	{
			printf("Phase A-B Count = %d, Index pulse = %d \n\r", cv0,cv1);
		}
		prev_cv0 = cv0;
		
		clearLine(5, 160);
		sprintf(charVector, "cv0 = %d", cv0);
		printf(charVector);
		ili9325_set_foreground_color(COLOR_BLUEVIOLET);		
		ili9325_draw_string(5, 160, (uint8_t *)charVector);
		
		clearLine(5, 180);
		sprintf(charVector, "cv1 = %d", cv1);
		printf(charVector);
		ili9325_set_foreground_color(COLOR_BLUEVIOLET);		
		ili9325_draw_string(5, 180, (uint8_t *)charVector);
		
		//velocidade = tc_read_ra(TC0, CHANNEL0);
		velocidade = TC0->TC_CHANNEL[0].TC_RA;

		clearLine(5, 240);
		sprintf(charVector, "velocidade = %d", velocidade);
		printf(charVector);
		ili9325_set_foreground_color(COLOR_BLUEVIOLET);		
		ili9325_draw_string(5, 240, (uint8_t *)charVector);		
		
		//delay_ms(50);		
	}
}


/* =========		FUNÇÕES SECUNDÁRIAS		   =====================*/



/**
 *  Configure UART console.
 */
static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};
	
	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}



static void configure_encoder(void) {
		
		
		ioport_set_pin_mode(PIO_PA0_IDX, IOPORT_MODE_MUX_B);	//Connect the pin to the TC0 TIOA0 (A)
		ioport_disable_pin(PIO_PA0_IDX);

		ioport_set_pin_mode(PIO_PA1_IDX, IOPORT_MODE_MUX_B);	//Connect the pin to the TC0 TIOB0 (B)
		ioport_disable_pin(PIO_PA1_IDX);
		
		ioport_set_pin_mode(PIO_PA16_IDX, IOPORT_MODE_MUX_B);	//Connect the pin to the TC0 TIOB1 (Index)
		ioport_disable_pin(PIO_PA16_IDX);


		sysclk_enable_peripheral_clock(ID_TC0);		//Enable the TC0 clock
		sysclk_enable_peripheral_clock(ID_TC1);		//Enable the TC1 clock
		sysclk_enable_peripheral_clock(ID_TC3);		//Enable the TC clock
		
		
		//tc_set_block_mode(TC0, TC_BMR_QDEN | TC_BMR_SPEEDEN | TC_BMR_EDGPHA | (0<< TC_BMR_MAXFILT_Pos));	//Enable the quadrature		//Enable position mode	//Detect quadrature on both PHA and PHB (4X decoding) //Set the filtering
		// enabled
		TC0->TC_BMR |= (1<<TC_BMR_QDEN) | (1<<TC_BMR_SPEEDEN) | (1<<TC_BMR_EDGPHA);
		TC0->TC_CHANNEL[2].TC_RC = 65536;
		
		TC0->TC_CHANNEL[0].TC_CMR |= (0<<TC_CMR_WAVE)		| 
									 //TC_CMR_WAVSEL(0x10)	|
									 //TC_CMR_ACPC(0x11)	|
									 (1<<TC_CMR_ABETRG)		|
									 TC_CMR_LDRA(0x01)		|
									 TC_CMR_LDRB(0x01);

		TC0->TC_CHANNEL[2].TC_CMR |= (1<<TC_CMR_WAVE)		|
									 TC_CMR_WAVSEL(0x10)	|
									 TC_CMR_ACPC(0x11);							 
		
		TC0->TC_CHANNEL[0].TC_CCR |= (1<<TC_CCR_CLKEN);
		

		//tc_init(TC0, CHANNEL0, TC_CMR_TCCLKS_XC0);
		//tc_init(TC0, CHANNEL1, TC_CMR_TCCLKS_XC0);		//Set the clock XC1 (required by the quadrature counter)	//The rotation counter does not increment or decrement on the first index pulse but it does reset the position counter
		
		tc_start(TC0, CHANNEL0);
		tc_start(TC0, CHANNEL1);
		tc_start(TC0, CHANNEL2);
}



/**
 *  Configure Timer Counter 0 to generate an interrupt every 250ms.
 */
// [main_tc_configure]
static void configure_tc(void) {
	/*
	* Aqui atualizamos o clock da cpu que foi configurado em sysclk init
	*
	* O valor atual est'a em : 120_000_000 Hz (120Mhz)
	*/
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	
	/****************************************************************
	* Ativa o clock do periférico TC 0
	*****************************************************************
	* 
    * Parametros : 
    *  1 - ID do periferico
    * 
	*
	*****************************************************************/
	pmc_enable_periph_clk(ID_TC3);

	/*****************************************************************
	* Configura TC para operar no modo de comparação e trigger RC
	*****************************************************************
    *
	* Configura TC para operar no modo de comparação e trigger RC
	* devemos nos preocupar com o clock em que o TC irá operar !
	*
	* Cada TC possui 3 canais, escolher um para utilizar.
	*
    * No nosso caso :
    * 
	*	MCK		= 120_000_000
	*	SLCK	= 32_768		(rtc)
	*
	* Uma opção para achar o valor do divisor é utilizar a funcao, como ela
    * funciona ?
	* tc_find_mck_divisor()
	*
    *
    * Parametros
    *   1 - TC a ser configurado (TC0,TC1, ...)
    *   2 - Canal a ser configurado (0,1,2)
    *   3 - Configurações do TC :
    *
    *   Configurações de modo de operação :
	*	    TC_CMR_ABETRG  : TIOA or TIOB External Trigger Selection 
	*	    TC_CMR_CPCTRG  : RC Compare Trigger Enable 
	*	    TC_CMR_WAVE    : Waveform Mode 
	*
	*     Configurações de clock :
	*	    TC_CMR_TCCLKS_TIMER_CLOCK1 : Clock selected: internal MCK/2 clock signal 
	*	    TC_CMR_TCCLKS_TIMER_CLOCK2 : Clock selected: internal MCK/8 clock signal 
	*	    TC_CMR_TCCLKS_TIMER_CLOCK3 : Clock selected: internal MCK/32 clock signal 
	*	    TC_CMR_TCCLKS_TIMER_CLOCK4 : Clock selected: internal MCK/128 clock signal
	*	    TC_CMR_TCCLKS_TIMER_CLOCK5 : Clock selected: internal SLCK clock signal 
	*
	*****************************************************************/
	tc_init(TC1, CHANNEL0, TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK1);
	    
    /*****************************************************************
	* Configura valor trigger RC
    *****************************************************************
	*
	* Aqui devemos configurar o valor do RC que vai trigar o reinicio da contagem
	* devemos levar em conta a frequência que queremos que o TC gere as interrupções
	* e tambem a frequencia com que o TC está operando.
	*
	* Devemos configurar o RC para o mesmo canal escolhido anteriormente.
	*	
	*   ^ 
	*	|	Contador (incrementado na frequencia escolhida do clock)
	*   |
	*	|	 	Interrupcao	
	*	|------#----------- RC
	*	|	  /
	*	|   /
	*	| /
	*	|-----------------> t
	*
    * Parametros :
    *   1 - TC a ser configurado (TC0,TC1, ...)
    *   2 - Canal a ser configurado (0,1,2)
    *   3 - Valor para trigger do contador (RC)
    *****************************************************************/
    tc_write_rc(TC1, CHANNEL0, 65536);
	
	/*****************************************************************
	* Configura interrupção no TC
    *****************************************************************
    * Parametros :
    *   1 - TC a ser configurado
    *   2 - Canal
    *   3 - Configurações das interrupções 
	* 
	*        Essas configurações estão definidas no head : tc.h 
	*
	*	        TC_IER_COVFS : 	Counter Overflow 
	*	        TC_IER_LOVRS : 	Load Overrun 
	*	        TC_IER_CPAS  : 	RA Compare 
	*	        TC_IER_CPBS  : 	RB Compare 
	*	        TC_IER_CPCS  : 	RC Compare 
	*	        TC_IER_LDRAS : 	RA Loading 
	*	        TC_IER_LDRBS : 	RB Loading 
	*	        TC_IER_ETRGS : 	External Trigger 
	*****************************************************************/
	tc_enable_interrupt(TC1, CHANNEL0, TC_IER_CPCS);
    
    /*****************************************************************
	* Ativar interrupção no NVIC
    *****************************************************************
    *
    * Devemos configurar o NVIC para receber interrupções do TC 
    *
    * Parametros :
    *   1 - ID do periférico
	*****************************************************************/
	NVIC_EnableIRQ(ID_TC3);

    
    /*****************************************************************
	* Inicializa o timer
    *****************************************************************
    *
    * Parametros :
    *   1 - TC
    *   2 - Canal
	*****************************************************************/
    tc_start(TC1, CHANNEL0); //TRAva a placa!!!!!
}


/**
 *  \brief Configure the LEDs
 *
 */
static void configure_leds(void) {
	pmc_enable_periph_clk(PIN_LED_BLUE_ID);	
	pio_set_output(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK, 1, 0, 0);
}




/**
 *  Interrupt handler for TC0 interrupt. 
 */
void TC3_Handler(void) {

	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC1, CHANNEL0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);


	/** Muda o estado do LED */
	if(PIN_LED_BLUE_PIO->PIO_ODSR & PIN_LED_BLUE_MASK){
		pio_clear(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK);
	} else{
		pio_set(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK);
	}
}


/**
 *  \brief Configure the LCD
 *
 */
static void configure_lcd(void) {

	/* Configure SMC interface for Lcd */
	smc_set_setup_timing(SMC,ILI9325_LCD_CS,SMC_SETUP_NWE_SETUP(2)
	| SMC_SETUP_NCS_WR_SETUP(2)
	| SMC_SETUP_NRD_SETUP(2)
	| SMC_SETUP_NCS_RD_SETUP(2));
	smc_set_pulse_timing(SMC, ILI9325_LCD_CS , SMC_PULSE_NWE_PULSE(4)
	| SMC_PULSE_NCS_WR_PULSE(4)
	| SMC_PULSE_NRD_PULSE(10)
	| SMC_PULSE_NCS_RD_PULSE(10));
	smc_set_cycle_timing(SMC, ILI9325_LCD_CS, SMC_CYCLE_NWE_CYCLE(10)
	| SMC_CYCLE_NRD_CYCLE(22));
	#if !defined(SAM4S)
	smc_set_mode(SMC, ILI9325_LCD_CS, SMC_MODE_READ_MODE
	| SMC_MODE_WRITE_MODE
	| SMC_MODE_DBW_8_BIT);
	#else
	smc_set_mode(SMC, ILI9325_LCD_CS, SMC_MODE_READ_MODE
	| SMC_MODE_WRITE_MODE);
	#endif
	/* Initialize display parameter */
	g_ili9325_display_opt.ul_width = ILI9325_LCD_WIDTH;
	g_ili9325_display_opt.ul_height = ILI9325_LCD_HEIGHT;
	g_ili9325_display_opt.foreground_color = COLOR_BLACK;
	g_ili9325_display_opt.background_color = COLOR_WHITE;

	/* Switch off backlight */
	//aat31xx_disable_backlight();

	/* Initialize LCD */
	ili9325_init(&g_ili9325_display_opt);

	/* Set backlight level */
	aat31xx_set_backlight(AAT31XX_MAX_BACKLIGHT_LEVEL);

	ili9325_set_foreground_color(COLOR_WHITE);
	ili9325_draw_filled_rectangle(0, 0, ILI9325_LCD_WIDTH, ILI9325_LCD_HEIGHT);

	/* Turn on LCD */
	ili9325_display_on();
}



/**
 *  Desenha o cabeçalho. 
 */
void desenhaCabecalho(void){
	char charVector[50];
	//printf("\x0C\n\r-- QDec Example --\n\r");
	//sprintf(charVector, "\x0C\n\r-- QDec Example --\n\r");
	//ili9325_set_foreground_color(COLOR_BLACK);
	//ili9325_draw_string(5, 100, (uint8_t *)charVector);

	/* Draw text, image and basic shapes on the LCD */
	ili9325_set_foreground_color(COLOR_BLACK);
	ili9325_draw_string(10, 5, (uint8_t *)"QUADRATURE DECODER");
	
	ili9325_set_foreground_color(COLOR_BLACK);
	ili9325_draw_string(10, 25, (uint8_t *)"Henrique Rosa v1.0");

	//printf("-- Compiled: %s %s --\n\r\n\r", __DATE__, __TIME__);
	sprintf(charVector, "%s %s", __DATE__, __TIME__);
	ili9325_set_foreground_color(COLOR_BLACK);
	ili9325_draw_string(0, 45, (uint8_t *)charVector);
	
	ili9325_draw_filled_rectangle(10, 65, ILI9325_LCD_WIDTH - 10, 68);
	ili9325_draw_filled_rectangle(10, 71, ILI9325_LCD_WIDTH - 10, 74);
}


/**
 *  Apaga uma linha o cabeçalho. 
 */
void clearLine(uint8_t posX, uint8_t posY) {

	/*
	char charVector [30];
	char espacoEmBranco [] = {"                         "}; //Vetor = 20 x ' ', criado para apagar linha.
	
	sprintf(charVector, "%s", espacoEmBranco);
	printf(charVector);
	ili9325_set_foreground_color(color);
	ili9325_draw_string(posX, posY, (uint8_t *)charVector);
	*/
	
	ili9325_set_foreground_color(COLOR_WHITE);
	ili9325_draw_filled_rectangle(posX, posY, ILI9325_LCD_WIDTH, posY + 15);
}