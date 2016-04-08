/**
 * IMT - Rafael Corsi
 * 
 * PIO - 07
 *  Configura o PIO do SAM4S (Banco A, pino 19) para operar em
 *  modo de output. Esse pino está conectado a um LED, que em 
 *  lógica alta apaga e lógica baixa acende.
*/

#include <asf.h>
#include "Driver/pio_maua.h" //Inclui Biblioteca do PIO.

/*
 * Prototypes
 */

/** 
 * Definição dos pinos
 * Pinos do uC referente aos LEDS.
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
#define PIN_LED_BLUE 19
#define PIN_LED_GREEN 20
#define PIN_LED_RED 20
#define BTN2 3

#define set(REG, x)	REG |= 1 << x; // definição padrão de setar bits

/**
 * Main function
 * 1. configura o clock do sistema
 * 2. desabilita wathdog
 * 3. ativa o clock para o PIOA
 * 4. ativa o controle do pino ao PIO
 * 5. desabilita a proteção contra gravações do registradores
 * 6. ativa a o pino como modo output
 * 7. coloca o HIGH no pino
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
		
	// 29.17.4 PMC Peripheral Clock Enable Register 0
	// 1: Enables the corresponding peripheral clock.
	// ID_PIOA = 11 - TAB 11-1
	PMC->PMC_PCER0 |= 1 << ID_PIOA;
	PMC->PMC_PCER0 |= 1 << ID_PIOC;
	PMC->PMC_PCER0 |= 1 << ID_PIOB;
	
	_pio_set_output(PIOA, ((1 << PIN_LED_BLUE ) | (1 << PIN_LED_GREEN )), 0, 0);
	_pio_set_output(PIOB,(1 << PIN_LED_RED ), 0, 0);

	/*
	 //31.6.1 PIO Enable Register
	// 1: Enables the PIO to control the corresponding pin (disables peripheral control of the pin).	
	PIOA->PIO_PER |= (1 << PIN_LED_BLUE ) | (1 << PIN_LED_GREEN );
	PIOC->PIO_PER |= (1 << PIN_LED_RED );
	// 31.6.46 PIO Write Protection Mode Register
	// 0: Disables the write protection if WPKEY corresponds to 0x50494F (PIO in ASCII).
	PIOA->PIO_WPMR = 0;
	PIOC->PIO_WPMR = 0;
	// 31.6.4 PIO Output Enable Register
	// value =
	//	 	1 : Enables the output on the I/O line.
	//	 	0 : do nothing
	PIOA->PIO_OER =  (1 << PIN_LED_BLUE ) | (1 << PIN_LED_GREEN );
	PIOC->PIO_OER =  (1 << PIN_LED_RED );
	// 31.6.10 PIO Set Output Data Register
	// value = 
	// 		1 : Sets the data to be driven on the I/O line.
	// 		0 : do nothing
	PIOA->PIO_SODR = (1 << PIN_LED_BLUE ) | (1 << PIN_LED_GREEN );
	PIOC->PIO_SODR = (1 << PIN_LED_RED );
*/
	//Configuração do PB3 (BTN2) como entrada	
	PIOB->PIO_PER |= (1 << BTN2); //Habilita o MUX para desabilitar o pino PB3 como saída
	PIOB->PIO_ODR |= (1 << BTN2); // Desabilita o buffer do pino PB3
	
	//Configurando Resistor de PulluP
	//PIOB->PIO_PUER = (1 << BTN2);  //Habilita PullUp
	_pio_pull_up(PIOB, BTN2, BTN2);
	PIOB->PIO_PPDDR = (1 << BTN2);	//Desabilita PullDown
	
	//Configuração de periférico de DEBOUNCING
	PIOB->PIO_IFSCDR = (1 << BTN2); //Seta MUX do Peripheral Clock para o Debouncing Filter
	PIOB->PIO_IFER = (1 << BTN2); //Seta MUX do Sinal de entrada para o Debouncin
	
	/**
	*	Loop infinito
	*/
		while(1){

            /*
             * Utilize a função delay_ms para fazer o led piscar na frequência
             * escolhida por você.
             */
			
			if (((PIOB->PIO_PDSR >> BTN2) & 1)){
				PIOA->PIO_SODR =  (1 << PIN_LED_BLUE ); //led azul apaga
			} else {
				PIOA->PIO_CODR =  (1 << PIN_LED_BLUE ); //led azul acende
			}
			 
			
			/*
			PIOC->PIO_SODR |=  (1 << PIN_LED_RED ); //led red acende
			delay_ms(1000);
			PIOC->PIO_CODR |=  (1 << PIN_LED_RED ); //led red apaga
			PIOA->PIO_CODR |=  (1 << PIN_LED_BLUE ); //led azul acende
			delay_ms(1000);
			PIOA->PIO_SODR |=  (1 << PIN_LED_BLUE ); //led azul apaga
			PIOA->PIO_CODR |=  (1 << PIN_LED_GREEN ); //led verde acende
			delay_ms(1000);
			PIOA->PIO_SODR |=  (1 << PIN_LED_GREEN ); //led verde apaga   
			*/   
			
		
	}
}
