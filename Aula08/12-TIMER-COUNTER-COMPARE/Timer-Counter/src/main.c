/************************************************************************
* Timer Counter
*
* Exemplo de utilização do modo RC Compare
* Nesse exemplo o uC entra em modo de stand-by e aguarda por três diferentes
* interrupcoes :
*
* 1 - Timer
*	Muda o estado do led em uma frequencia pré definida (inicial 4Hz)
*
* 2 - Botao 1
*	Altera a frequencia do led em 10% para cima
*	
* 3 - Botao 2
*	Altera a frequencia do led em 10% para baixo
*
************************************************************************/

//#include <delay.h>
#include "asf.h"
#include "Driver/pio_maua.h"
#include "Driver/pmc_maua.h"



//Button 01
#define PIN_PUSHBUTTON_1_MASK	PIO_PB3
#define PIN_PUSHBUTTON_1_PIO	PIOB
#define PIN_PUSHBUTTON_1_ID		ID_PIOB
#define PIN_PUSHBUTTON_1_TYPE	PIO_INPUT
#define PIN_PUSHBUTTON_1_ATTR	PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE

//Button 02
#define PIN_PUSHBUTTON_2_MASK	PIO_PC12
#define PIN_PUSHBUTTON_2_PIO	PIOC
#define PIN_PUSHBUTTON_2_ID		ID_PIOC
#define PIN_PUSHBUTTON_2_TYPE	PIO_INPUT
#define PIN_PUSHBUTTON_2_ATTR	PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_FALL_EDGE

//Blue LED
#define PIN_LED_BLUE		19
#define PIN_LED_BLUE_PIO	PIOA
#define PIN_LED_BLUE_ID		ID_PIOA
#define PIN_LED_BLUE_MASK	(1u << PIN_LED_BLUE)  //u: unsigned



/** IRQ priority for PIO (The lower the value, the greater the priority) */
#define IRQ_PRIOR_PIO    0

#define Freq_Init_Blink 4	//Hz

/**
 *  Handle Interrupcao botao 1
 */
static void Button1_Handler(uint32_t id, uint32_t mask)
{
	tc_write_rc(TC0, 0, tc_read_rc(TC0, 0)*1.1);
}

/**
 *  Handle Interrupcao botao 2.
 */
static void Button2_Handler(uint32_t id, uint32_t mask)
{
	tc_write_rc(TC0, 0, tc_read_rc(TC0, 0)*0.9 );
}

/**
 *  Interrupt handler for TC0 interrupt. 
 */
void TC0_Handler(void)
{
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

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
 *  \brief Configure the Pushbuttons
 *
 *  Configure the PIO as inputs and generate corresponding interrupt when
 *  pressed or released.
 */
static void configure_buttons(void)
{
	//Habilitação dos Clocks dos periféricos
	pmc_enable_periph_clk(PIN_PUSHBUTTON_1_ID);
	pmc_enable_periph_clk(PIN_PUSHBUTTON_2_ID);
	
	//Seleção dos pinos com entrada
	pio_set_input(PIN_PUSHBUTTON_1_ID, PIN_PUSHBUTTON_1_MASK, PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE);
	pio_set_debounce_filter(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK, 100); //Configura divisor do clock para debounce
	pio_handler_set(PIN_PUSHBUTTON_1_PIO, 
					PIN_PUSHBUTTON_1_ID,
					PIN_PUSHBUTTON_1_MASK,  
					PIO_IT_FALL_EDGE ,
					Button1_Handler); //Configura interrupção para acontecer em borda de descida.		
	pio_enable_interrupt(PIN_PUSHBUTTON_1_PIO, PIN_PUSHBUTTON_1_MASK); //Ativa interrupção no periférico B porta do botão	
	NVIC_SetPriority(PIN_PUSHBUTTON_1_ID, 1); //Configura a prioridade da interrupção no PORTB
	NVIC_EnableIRQ(PIN_PUSHBUTTON_1_ID); //Ativa interrupção no port B
	
	
	pio_set_input(PIN_PUSHBUTTON_2_ID, PIN_PUSHBUTTON_2_MASK, PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_FALL_EDGE);
	pio_set_debounce_filter(PIN_PUSHBUTTON_2_PIO, PIN_PUSHBUTTON_2_MASK, 100); //Configura divisor do clock para debounce
	pio_handler_set(PIN_PUSHBUTTON_2_PIO, 
					PIN_PUSHBUTTON_2_ID,
					PIN_PUSHBUTTON_2_MASK,  
					PIO_IT_FALL_EDGE ,
					Button2_Handler); //Configura interrupção para acontecer em borda de descida.		
	pio_enable_interrupt(PIN_PUSHBUTTON_2_PIO, PIN_PUSHBUTTON_2_MASK); //Ativa interrupção no periférico B porta do botão	
	NVIC_SetPriority(PIN_PUSHBUTTON_2_ID, 1); //Configura a prioridade da interrupção no PORTB
	NVIC_EnableIRQ(PIN_PUSHBUTTON_2_ID); //Ativa interrupção no port C		
}

/**
 *  \brief Configure the LEDs
 *
 */
static void configure_leds(void)
{
	pmc_enable_periph_clk(PIN_LED_BLUE_ID);	
	pio_set_output(PIN_LED_BLUE_PIO, PIN_LED_BLUE_MASK, 1, 0, 0);
}




/**
 *  Configure Timer Counter 0 to generate an interrupt every 250ms.
 */
// [main_tc_configure]
static void configure_tc(void)
{
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
	pmc_enable_periph_clk(ID_TC0);

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
	tc_init(TC0, 0, TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK5);
	    
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
    tc_write_rc(TC0, 0, 65535);
	
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
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
    
    /*****************************************************************
	* Ativar interrupção no NVIC
    *****************************************************************
    *
    * Devemos configurar o NVIC para receber interrupções do TC 
    *
    * Parametros :
    *   1 - ID do periférico
	*****************************************************************/
	NVIC_EnableIRQ(ID_TC0);

    
    /*****************************************************************
	* Inicializa o timer
    *****************************************************************
    *
    * Parametros :
    *   1 - TC
    *   2 - Canal
	*****************************************************************/
    tc_start(TC0, 0);
}


/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/** Configura o timer */
	configure_tc();

    /* Configura Leds */
    configure_leds();
	
	/* Configura os botões */
	configure_buttons();
	   
	while (1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
