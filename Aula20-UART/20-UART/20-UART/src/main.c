/**
 *	20-UART 
 * Prof. Rafael Corsi
 *
 *    (e.g., HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 */

#include "asf.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "stdio_serial.h"
#include "smc.h"
#include "pmc.h"
#include "pio.h"

/* =========  PROTÓTIPO DE FUNÇÕES    =====================*/

static void configure_leds(void);
static void configure_lcd(void);
static void configure_tc(void);
static void	config_uart(void);
void desenhaCabecalho(void);
void clearLine(uint8_t posX, uint8_t posY);
static void display_menu(void);


/************************************************************************/
/* Configurações                                                        */
/************************************************************************/

/* Chip select number to be set */
#define ILI9325_LCD_CS      1


#define STRING_EOL    "\r"
#define STRING_VERSAO "-- "BOARD_NAME" --\r\n" \
					  "-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

#define CONF_UART_BAUDRATE 115200		
#define CONF_UART          CONSOLE_UART

/** 
 * LEDs
 */ 
#define PIN_LED_BLUE	19
#define PIN_LED_BLUE_PIO	PIOA
#define PIN_LED_BLUE_ID		ID_PIOA
#define PIN_LED_BLUE_MASK	(1u << PIN_LED_BLUE)

#define CHANNEL0 0
#define CHANNEL1 1
#define CHANNEL2 2


/* =========		VARIÁVEIS	   =====================*/

struct ili9325_opt_t g_ili9325_display_opt;

/** Capture status*/
//static uint32_t gs_ul_captured_pulses;
static uint32_t prev_cv0=0;

int flagLED = 0;




/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{
	uint8_t uc_key;

	/* Initialize the system */
	sysclk_init();
	board_init();

	/* Configure LED 1 */
	pmc_enable_periph_clk(PIN_LED_BLUE_ID);
	pio_set_output(PIN_LED_BLUE_PIO , PIN_LED_BLUE_MASK, 1, 0, 0);

	/* Initialize debug console */
	config_uart();
	
	/* frase de boas vindas */
	puts(" ---------------------------- \n\r"
	 	 " Bem vindo terraquio !		\n\r"
		 " ---------------------------- \n\r");
	
	/* Enable peripheral clock */
	pmc_enable_periph_clk(ID_SMC);

	/** Configura o LEDs */
	configure_leds();

	/** Configura o timer */
	configure_tc();


	/* Configuração LCD */
	configure_lcd();
	
	desenhaCabecalho();
	
	/* display main menu */
	display_menu();

	while (1) {
		
		
		usart_serial_getchar((Usart *)CONSOLE_UART, &uc_key);	
		switch (uc_key) {
			case '1':
				display_menu();
				break;
			case '2':
				flagLED = 0;
				pio_clear(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK);
				puts("Led ON \n\r");
				break;
			case '3' :
				flagLED = 1;
				pio_set(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK);
				puts("Led OFF \n\r");
				break;
			case '4' :
				flagLED = 2;
				pio_set(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK);
				puts("Led OFF \n\r");
				break;
			case '5' :
				flagLED = 3;
				pio_set(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK);
				puts("Defina a o valor da Frequência (0-65356) \n\r");
				usart_serial_getchar((Usart *)CONSOLE_UART, &uc_key);
				//tc_write_rc(uc_key);
				break;
				
			default:
				printf("Opcao nao definida: %d \n\r", uc_key);
		}	
	}
}




/************************************************************************/
/* Configura UART                                                       */
/************************************************************************/
void config_uart(void){
	
	/* configura pinos */
	gpio_configure_group(PINS_UART0_PIO, PINS_UART0, PINS_UART0_FLAGS);
	
	/* ativa clock */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	
	/* Configuração UART */
	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.paritytype = UART_MR_PAR_NO,
		.stopbits   = 0
	};
	
	stdio_serial_init((Usart *)CONF_UART, &uart_serial_options);
	
	uart_enable_interrupt(UART0, &uart_serial_options);
}

/************************************************************************/
/* Display Menu                                                         */
/************************************************************************/
static void display_menu(void)
{
	puts(" 1 : exibe novamente esse menu \n\r"
	" 2 : Ativa o LED  \n\r"
	" 3 : Desliga o LED \n\r "
	" 4 : Pisca o LED \n\r "
	" 5 : Definir frequência do LED \n\r ");
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
	tc_init(TC1, CHANNEL0, TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK4);
	    
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
*  Interrupt handler for UART0. 
*/
void UART0_Handler( void ){
	pio_set(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK);
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
	
	
	switch (flagLED){
		
		case 0:
			pio_clear(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK);
			break;
			
		case 1:
			pio_set(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK);
			break;
			
		case 2:
			if(PIN_LED_BLUE_PIO->PIO_ODSR & PIN_LED_BLUE_MASK){
				pio_clear(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK);
				} else{
				pio_set(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK);
			}
			break;
				
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

	/* Draw text, image and basic shapes on the LCD */
	ili9325_set_foreground_color(COLOR_BLACK);
	ili9325_draw_string(10, 5, (uint8_t *)"AULA20-UART");
	
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