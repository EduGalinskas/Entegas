
#include "pio_maua.h"

void _pio_set_output( Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_default_level, const uint32_t ul_pull_up_enable){
	//31.6.1 PIO Enable Register
	// 1: Enables the PIO to control the corresponding pin (disables peripheral control of the pin).
	p_pio->PIO_PER |= ul_mask;
	
	// 31.6.46 PIO Write Protection Mode Register
	// 0: Disables the write protection if WPKEY corresponds to 0x50494F (PIO in ASCII).
	p_pio->PIO_WPMR = 0;
	
	if (ul_pull_up_enable)
		_pio_pull_up(p_pio,ul_mask, ul_pull_up_enable);
	
	if (ul_default_level)
		_pio_set(p_pio, ul_mask);
	else
		_pio_clear(p_pio, ul_mask);
	
	// 31.6.4 PIO Output Enable Register
	// 1: Enables the output on the I/O line.
	p_pio->PIO_OER |= ul_mask;

}

void _pio_set_input( Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_attribute) {
	
	_pio_pull_up(p_pio,ul_mask, ul_attribute & PIO_PULLUP);
	
	if( ((PIO_DEGLITCH) & (ul_attribute)) > 0 )
	{
		p_pio->PIO_IFER = ul_mask;
		p_pio->PIO_IFSCDR = ul_mask;
	}
	
	if( ((PIO_DEBOUNCE) & (ul_attribute)) > 0 )
	{
		p_pio->PIO_IFER = ul_mask;
		p_pio->PIO_IFSCER = ul_mask;
	}
	p_pio->PIO_PER = ul_mask;
	p_pio->PIO_ODR = ul_mask;
			
		

}

void _pio_pull_up(	Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable) {
	
	if (ul_pull_up_enable == 1)
	{
		p_pio->PIO_PUER = ul_mask;
	}
}
void _pio_pull_down( Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_down_enable){
	if (ul_pull_down_enable == 1)
	{
		p_pio->PIO_PPDER = ul_mask;
	}
}
void _pio_set( Pio *p_pio, const uint32_t ul_mask) {
	p_pio->PIO_SODR = ul_mask;
}
void _pio_clear( Pio *p_pio, const uint32_t ul_mask) {
	p_pio->PIO_CODR = ul_mask;
}

uint32_t _pio_get_output_data_status(const Pio *p_pio, const uint32_t ul_mask) {
	return (p_pio->PIO_PDSR >> ul_mask) & 1;
}
void config_pin_input(Pio *PIO, int pinos){
	
	//31.6.1 PIO Enable Register
	// 1: Enables the PIO to control the corresponding pin (disables peripheral control of the pin).
	PIO->PIO_PER = pinos;
	
	// 31.6.46 PIO Write Protection Mode Register
	// 0: Disables the write protection if WPKEY corresponds to 0x50494F (PIO in ASCII).
	PIO->PIO_WPMR = 0;
	
	// 31.6.4 PIO Output Disable Register
	// 1: Disable the output on the I/O line.
	PIO->PIO_ODR = pinos;
	
	// Enable pull-up
	PIO->PIO_PUER = pinos;
	
	// Ativando o Debouncing
	PIO->PIO_IFSCER = pinos;
}