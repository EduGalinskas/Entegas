/**
 * IMT - Rafael Corsi
 * 
 * Interrupção
*/

#include <asf.h>
#include "Driver/pio_maua.h"
#include "Driver/pmc_maua.h"

/*
 * Prototypes
 */

static void push_button_handle(uint32_t id, uint32_t mask);


/** 
 * Definição dos pinos
 * Pinos do uC referente aos LEDS/ Botão
 *
 * O número referente ao pino (PIOAxx), refere-se ao
 * bit que deve ser configurado no registrador para alterar
 * o estado desse bit específico.
 *
 * exe : O pino PIOA_19 é configurado nos registradores pelo bit
 * 19. O registrador PIO_SODR configura se os pinos serão nível alto.
 * Nesse caso o bit 19 desse registrador é referente ao pino PIOA_19
 *
 * ----------------------------------
 * | BIT 19  | BIT 18  | ... |BIT 0 |
 * ----------------------------------
 * | PIOA_19 | PIOA_18 | ... |PIOA_0|
 * ----------------------------------
 */
#define PIN_LED_BLUE	19
#define PIN_LED_RED		20
#define PIN_LED_GREEN	20
#define PIN_BUTTON		3
#define time			100

/** 
 * Definição dos ports
 * Ports referentes a cada pino
 */
#define PORT_LED_BLUE	PIOA
#define PORT_LED_GREEN	PIOA
#define PORT_LED_RED	PIOC
#define PORT_BUT_2		PIOB

/**
 * Define os IDs dos periféricos associados aos pinos
 */
#define ID_LED_BLUE		ID_PIOA
#define ID_LED_GREEN	ID_PIOA
#define ID_LED_RED		ID_PIOC
#define ID_BUT_2		ID_PIOB

/**
 *	Define as masks utilizadas
 */
#define MASK_LED_BLUE	(1u << PIN_LED_BLUE)
#define MASK_LED_GREEN	(1u << PIN_LED_GREEN)
#define MASK_LED_RED	(1u << PIN_LED_RED)
#define MASK_BUT_2		(1u << PIN_BUTTON)

/************************************************************************/
/*  INTERRUPÇÃO PORTB													*/
/************************************************************************/

static void push_button_handle(uint32_t id, uint32_t mask){
	pio_clear(PIOA, (1 << PIN_LED_GREEN));
	delay_ms(500);
	pio_set(PIOA, (1 << PIN_LED_GREEN));
		
}


/**
 * Main function
 * 1. configura o clock do sistema
 * 2. desabilita wathdog
 * 3. Configura os LEDs como saída
 * 4. Configura o botão como entrada
 * 5. Configura a interrupção no botão
 * 6. Configura a interrupção no NVIC/CORE
 * 7. Ativa interrupção
 * 8. while(1)
 * 
 * 9. Quando o botão for apertado, a função push_button_handle é chamada via interrupção
 */

int main (void)
{

	/**
	* Inicializando o clock do uP
	*/
	sysclk_init();
	
	/** 
	*  Desabilitando o WathDog do uP
	*/
	WDT->WDT_MR = WDT_MR_WDDIS;
		
	/**
	* Ativa clock nos periféricos
	*/
	_pmc_enable_clock_periferico(ID_LED_BLUE);
	_pmc_enable_clock_periferico(ID_LED_GREEN);		// Redundante mas não tem problema !
	_pmc_enable_clock_periferico(ID_LED_RED);
	_pmc_enable_clock_periferico(ID_BUT_2);
				
	/**
	*	Configura saída
	*/
	pio_set_output(PORT_LED_BLUE  , MASK_LED_BLUE	,1,0,0);
	pio_set_output(PORT_LED_GREEN , MASK_LED_GREEN  ,1,0,0);
	pio_set_output(PORT_LED_RED	  , MASK_LED_RED	,1,0,0);
	
	/**
	* Configura entrada
	*/ 
	pio_set_input(PORT_BUT_2, MASK_BUT_2, PIO_PULLUP | PIO_DEBOUNCE);
	
	/*
	 * Configura divisor do clock para debounce
	 */
	pio_set_debounce_filter(PORT_BUT_2, MASK_BUT_2, 100);	
	/* 
	*	Configura interrupção para acontecer em borda de descida.
	*/
	pio_handler_set(PIOB, 
					ID_PIOB,
					MASK_BUT_2,  
					PIO_IT_FALL_EDGE ,
					push_button_handle);
				
	/*
	*	Ativa interrupção no periférico B porta do botão
	*/	
	pio_enable_interrupt(PIOB, MASK_BUT_2 );
	
	/*
	*	Configura a prioridade da interrupção no PORTB
	*/
	NVIC_SetPriority(ID_PIOB, 1);
	
	/*
	*	Ativa interrupção no port B
	*/
	NVIC_EnableIRQ(ID_PIOB);
	
	/**
	*	Loop infinito
	*/
	while(1){
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		
		pio_set(PIOA, (1 << PIN_LED_BLUE));
		delay_ms(500);
		pio_clear(PIOA, (1 << PIN_LED_BLUE));
		delay_ms(500);		pio_set(PIOA, (1 << PIN_LED_BLUE));
		delay_ms(500);
		pio_clear(PIOA, (1 << PIN_LED_BLUE));
		delay_ms(500);	
	}
}

