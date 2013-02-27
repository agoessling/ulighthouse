/*
 * PSK_LED.c
 *
 * Created: 8/26/2012 3:49:26 PM
 *  Author: agoessling
 */ 

/* TLC5940 Pin Mapping
 * 
 * PB0 => SCLK
 * PD0 <= SOUT
 * PD1 => SIN
 *
 * PD4 => XLAT
 * PD5 => BLANK
 * PD6 => GSCLK
 *
 * PA0 => DCPRG
 * PA1 => VPRG
 * PB2 <= XERR
 */

/* Board IO
 *
 * PA5 <= Master Sense
 *
 */

// Defines
#define F_CPU			8000000UL	//8MHz
#define LED_SPI_BAUD	400000UL	//400KHz
#define F_LED_GSCK		1000000UL	//1MHz

#define LED_NO_ERROR	0x00
#define LED_LOD_ERROR	0x01
#define LED_TEF_ERROR	0x02

#define PC_ERR_NONE		0x00
#define PC_ERR_LEN		0x03

#define LED_STATUS_WAIT			0x00
#define LED_STATUS_LOD_ERROR	0x01
#define LED_STATUS_TEF_ERROR	0x02

#define PC_UART_BAUD	250000

#define PC_SFLAG	0x7E
#define PC_ESCP		0x7D
#define PC_ESFLAG	0x5E
#define PC_EESCP	0x5D

#define PC_CMD_POS	0x01
#define PC_LEN_POS	0x02
#define PC_DATA_POS	0x03

#define PC_CMD_CONNECT		0x01
#define PC_CMD_DISCONNECT	0x02
#define PC_CMD_HELLO		0x03
#define PC_CMD_ACK			0x04
#define PC_CMD_EN_LED		0x05
#define PC_CMD_DIS_LED		0x06
#define PC_CMD_SEND_DATA	0x07
#define PC_CMD_LAT_DATA		0x08
#define PC_CMD_SET_DC		0x09

// Macros
#define led_dcprg_set()		PORTA|=(1<<PA0)
#define led_dcprg_clr()		PORTA&=~(1<<PA0)
#define led_dcprg_init()	DDRA|=(1<<PA0)
#define led_vprg_set()		PORTA|=(1<<PA1)
#define led_vprg_clr()		PORTA&=~(1<<PA1)
#define led_vprg_init()		DDRA|=(1<<PA1)

#define led_blank_start()	TCCR1A|=0xF0;TCCR1B|=(1<<CS10)|(1<<CS11)
#define led_blank_stop()	TCCR1A&=0x0F;TCCR1B&=(~((1<<CS10)|(1<<CS11)))
#define led_blank_set()		PORTD|=(1<<PD5);
#define led_blank_clr()		PORTD&=~(1<<PD5);

#define led_chk_blank()		TCCR1B&0x07

#define led_xlat_set()		PORTD|=(1<<PD4)
#define led_xlat_clr()		PORTD&=~(1<<PD4)

#define led_xerr_enable_int()	EIMSK|=(1<<INT2)
#define led_xerr_disable_int()	EIMSK&=~(1<<INT2)

// Includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Functions
void	gpio_init(void);
void	led_spi_init(uint16_t);
uint8_t led_txrx_byte(uint8_t);
void	led_gsck_init(uint8_t);
void	led_blank_init(uint8_t);
uint8_t led_send_data(uint8_t*);
void	led_latch_data(void);
void	led_pack_data(uint8_t*, uint8_t, uint16_t);
void	led_process_LOD(void);
void	led_process_TEF(void);
void	led_xerr_interupt_init(void);
void	led_set_dc(uint8_t, uint8_t);
void	led_run_ramp(void);

void	pc_uart_init(uint16_t);
void	pc_uart_tx_byte(uint8_t);
void	pc_uart_tx_data(uint8_t*, uint8_t);
void	pc_send_cmd(uint8_t, uint8_t, uint8_t*);

uint8_t find_num_brds(void);

// Data Structures

// Global Variables
volatile	uint8_t		led_wait_for_XLAT = 0;			
volatile	uint8_t		led_status = LED_STATUS_WAIT;
			uint8_t		led_ignore_LOD = 1;
			
            uint8_t		pc_rx_buf[200];
volatile	uint8_t		pc_cur_pkt_len = 0;
volatile	uint8_t		pc_pkt_ready = 0;
volatile	uint8_t		pc_escape_found = 1;

int main(void){
	uint8_t error = 0;
	uint8_t resp[10];
    uint8_t num_brds = 1;
    
    // Decide If Master
    if(!(PINA & (1<<PA5))){
        //DDRB |= (1<<PB5);
        while(1){
            // Possible Sleep Here?
            //PORTB |= (1<<PB5);
            //PORTB &= ~(1<<PB5);
        }
    }
	
	// Set Initial DC and GS Before Anything Else
	led_spi_init(F_CPU/(2*LED_SPI_BAUD)-1);

    // Find Length Of Daisy Chain
    num_brds = find_num_brds();

    if(num_brds == 0){
        // Unhandled Error
        while(1);
    }

	// VPRG Output
	led_vprg_init();
	// DCPRG Output and Select DC Internal Registers	
	led_dcprg_init();
	led_dcprg_set();
	// SET DC to Initial 0
	led_set_dc(0, num_brds);

	pc_uart_init(F_CPU/(16*PC_UART_BAUD)-1);
	
	led_gsck_init(F_CPU/F_LED_GSCK);
	led_blank_init(F_CPU/F_LED_GSCK);

	_delay_ms(10);
	if(!led_ignore_LOD)
		led_xerr_interupt_init();
	sei();
	
	while(1){
		while(!pc_pkt_ready);
		
		switch(pc_rx_buf[PC_CMD_POS]){
			case PC_CMD_HELLO:
				pc_send_cmd(PC_CMD_ACK, 1, (pc_rx_buf+PC_CMD_POS));
				break;
				
			case PC_CMD_EN_LED:
				led_blank_start();
				pc_send_cmd(PC_CMD_ACK, 1, (pc_rx_buf+PC_CMD_POS));
				break;
				
			case PC_CMD_DIS_LED:
				led_blank_stop();
				pc_send_cmd(PC_CMD_ACK, 1, (pc_rx_buf+PC_CMD_POS));
				break;
				
			case PC_CMD_SEND_DATA:
				if(pc_rx_buf[PC_LEN_POS] == 72){
					for(uint8_t i=0; i<3; i++){
						error = led_send_data(pc_rx_buf+PC_DATA_POS+24*i);
						if(error == LED_LOD_ERROR){
							if(!led_ignore_LOD){
								led_process_LOD();
								break;
							}								
						}
						else if(error == LED_TEF_ERROR){
							led_process_TEF();
							break;
						}
					}
					resp[0] = pc_rx_buf[PC_CMD_POS];
					resp[1] = error;
					pc_send_cmd(PC_CMD_ACK, 2, resp);				
				}
				else{
					resp[0] = pc_rx_buf[PC_CMD_POS];
					resp[1] = PC_ERR_LEN;
					pc_send_cmd(PC_CMD_ACK, 2, resp);
				}
				break;
				
			case PC_CMD_LAT_DATA:
				led_latch_data();
				if(led_chk_blank()){
					while(led_wait_for_XLAT);
				}
				pc_send_cmd(PC_CMD_ACK, 1, (pc_rx_buf+PC_CMD_POS));
				break;
				
			case PC_CMD_SET_DC:
				if(pc_rx_buf[PC_LEN_POS] == 2){
					led_set_dc(pc_rx_buf[PC_DATA_POS], pc_rx_buf[PC_DATA_POS+1]);
					resp[0] = pc_rx_buf[PC_CMD_POS];
					resp[1] = PC_ERR_NONE;
					pc_send_cmd(PC_CMD_ACK, 2, resp);
				}
				else{
					resp[0] = pc_rx_buf[PC_CMD_POS];
					resp[1] = PC_ERR_LEN;
					pc_send_cmd(PC_CMD_ACK, 2, resp);
				}					
				break;
				
			default:
				break;
		}
		
		pc_pkt_ready = 0;
	}				
}

uint8_t find_num_brds(void){
    uint16_t byte_count = 0;
    uint8_t byte = 0; 

    byte = led_txrx_byte(0xAA);

    // Allow ~100 Board Limit (72*100)
    while(1){
        byte = led_txrx_byte(0x00);
        byte_count++;

        if(byte==0xAA){
            byte = led_txrx_byte(0x00);
            byte_count++;

            if(byte == 0x00){
                byte_count--;
                return byte_count/72;
            }
        }
    }

    // Error
    return 0;
}

void gpio_init(void){
	DDRA = (1<<PA0)|(1<<PA1);
}

void led_spi_init(uint16_t baud_code){
	/* This function sets up USART0 to be used as SPI Master
	   to TLC5940 LED controller. */
	
	UBRR0 = 0;
	/* Setting the XCKn port pin as output, enables master mode. */
	DDRB |= (1<<PINB0);
	/* Set MSPI mode of operation and SPI data mode 3. */
	UCSR0C = (1<<UMSEL01)|(1<<UMSEL00)|(0<<UCPHA0)|(0<<UCPOL0);
	/* Enable receiver and transmitter. */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set baud rate. */
	/* IMPORTANT: The Baud Rate must be set after the transmitter is enabled
	*/
	UBRR0 = baud_code;
}

uint8_t led_txrx_byte(uint8_t data){
	/* This function writes (and reads) a byte to TLC590 via SPI on USART0 */
	
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = data;
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Get and return received data from buffer */
	return UDR0;
}

void led_gsck_init(uint8_t clk_div){
	/* This function initializes Timer2 to F_CPU/clk_div to be used as gsck for TLC5940 
		NOTE: clk_div must be at least 2 */
	
	/* OC2B set to output */
	DDRD |= (1<<PIND6);
	/* Fast PWM: Top=OCRA Non-Inverting OC2B*/
	TCCR2A = (1<<COM2B1)|(1<<WGM21)|(1<<WGM20);
	TCCR2B = (1<<WGM22);
	/* Set frequency to F_CPU/clk_div and duty to 50% */
	OCR2A = clk_div-1;
	OCR2B = ((clk_div-1) >> 1);	//div by 2
	/* Start PWM */
	TCCR2B |= (1<<CS20);
}

void led_blank_init(uint8_t clk_div){
	/* This function initializes Timer1
	 * clk_div is the same here as in led_gsck_init()
	*/
	
	/* OC1A (BLANK) and OC1B (XLAT) outputs */
	DDRD |= (1<<PD4)|(1<<PD5);
	/* Fast PWM: Top=ICR1 */
	TCCR1A = (0<<WGM10)|(1<<WGM11);
	TCCR1B = (1<<WGM12)|(1<<WGM13);
	/* Set frequency to F_CPU/clk_div/4096 assuming Timer1 operates at F_CPU/64 */
	ICR1 = ((uint16_t)clk_div << 6); // mult by 64 sub 1
	OCR1A = ((uint16_t)clk_div << 6) - 1;
	OCR1B = 0xFFFF;
	
	/* Interrupt on OCR1B (XLAT) match */
	TIMSK1 = (1<<OCIE1B);
}

uint8_t led_send_data(uint8_t *data){
	/* This function sends a block of GS data corresponding to 16 outputs of 12 bits = 24 bytes */
	
	uint8_t error=LED_NO_ERROR;
	uint8_t resp = 0;
	
	for(uint8_t i=0; i<24; i++){	
		resp = led_txrx_byte(*data++);
		
		if(i==0){							// SPI output is one bit behind
			if(resp & 0x7F)
				error = LED_LOD_ERROR;
		}	
		else if(i==1){
			if(resp)
				error = LED_LOD_ERROR;
		}			
		else if(i==2){
			if(resp & 0x40)					// TEF Error takes precedent
				error = LED_TEF_ERROR;
			else if(resp & 0x80)
				error = LED_LOD_ERROR;
		}							
	}
	
	return error;
}

void led_latch_data(void){
	cli();
	OCR1B = ICR1-1;
	led_wait_for_XLAT = 1;
	sei();
}

void led_pack_data(uint8_t *pkt, uint8_t output, uint16_t val){
	uint8_t upper_half_off = (3*(15-output)>>1);
	uint8_t lower_half_off = upper_half_off + 1;
	
	if(output%2){
		*(pkt+upper_half_off) = (val>>4);
		*(pkt+lower_half_off) &= 0x0F;
		*(pkt+lower_half_off) |= (val<<4);		
	}
	else {
		*(pkt+upper_half_off) &= 0xF0;
		*(pkt+upper_half_off) |= (val>>8)&(0x0F);
		*(pkt+lower_half_off) = val;
	}
}

void led_process_LOD(void){
	led_status = LED_STATUS_LOD_ERROR;
	led_blank_stop();		
}

void led_process_TEF(void){
	led_status = LED_STATUS_TEF_ERROR;
	led_blank_stop();
}

void led_xerr_interupt_init(void){
	// A Low Level Interrupts
	EICRA = 0x00;
	
	// Enable INT2
	led_xerr_enable_int();
}

void led_set_dc(uint8_t level, uint8_t num_brds){
	// Valid Level: 0-63
	if(level > 63){
		level = 63;
	}
			
	uint8_t blank_on = led_chk_blank();
	
	// Turn Off Outputs	If On
	if(blank_on){
		while(led_wait_for_XLAT);
		led_blank_stop();
	}		
		
	led_vprg_set();
	
	for(uint8_t j=0; j<num_brds; j++){
		// 6 bits for 16 outputs for 3 chips = 288 bits => 36 bytes
		// Send 3 bytes at a time (4x 6bit DC values) => 12 loops
		for(uint8_t i=0; i<12*num_brds; i++){
			led_txrx_byte((level << 2) | (level >> 4));
			led_txrx_byte((level << 4) | (level >> 2));
			led_txrx_byte((level << 6) | level);
		}
	}
	
	// Latch DC Data
	led_xlat_set();
	_delay_us(1);
	led_xlat_clr();
	
	led_vprg_clr();
	
	// Turn On Outputs If Previously On
	if(blank_on){
		led_blank_start();
	}					
}

void pc_uart_init(uint16_t baudcode){
	/* This Function Initializes USART1 for use with FT232R USB to Serial */
	UBRR1 = 0x00;
	
	UCSR1A = 0x00;
	UCSR1B = (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);
	UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);
	
	UBRR1 = baudcode;
}

void pc_uart_tx_byte(uint8_t data){
	/* Wait for empty transmit buffer */
	while ( !( UCSR1A & (1<<UDRE1)) );
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

void pc_uart_tx_data(uint8_t *data, uint8_t len){
	for(uint8_t i=0; i<len; i++){
		pc_uart_tx_byte(*data++);
	}
}

void pc_send_cmd(uint8_t cmd, uint8_t len, uint8_t *data){
	pc_uart_tx_byte(PC_SFLAG);
	
	if(cmd == PC_SFLAG){
		pc_uart_tx_byte(PC_ESCP);
		pc_uart_tx_byte(PC_ESFLAG);
	}
	else if(cmd == PC_ESCP){
		pc_uart_tx_byte(PC_ESCP);
		pc_uart_tx_byte(PC_EESCP);
	}
	else{
		pc_uart_tx_byte(cmd);
	}
	
	if(len == PC_SFLAG){
		pc_uart_tx_byte(PC_ESCP);
		pc_uart_tx_byte(PC_ESFLAG);
	}
	else if(len == PC_ESCP){
		pc_uart_tx_byte(PC_ESCP);
		pc_uart_tx_byte(PC_EESCP);
	}
	else{
		pc_uart_tx_byte(len);
	}
	
	for(uint8_t i=0; i<len; i++){
		if(*data == PC_SFLAG){
			pc_uart_tx_byte(PC_ESCP);
			pc_uart_tx_byte(PC_ESFLAG);
		}
		else if(*data == PC_ESCP){
			pc_uart_tx_byte(PC_ESCP);
			pc_uart_tx_byte(PC_EESCP);
		}
		else{
			pc_uart_tx_byte(*data);
		}
		
		data++;
	}
	
	pc_uart_tx_byte(PC_SFLAG);
}

// Demo Functions
void led_run_ramp(void){
	led_spi_init(F_CPU/(2*LED_SPI_BAUD)-1);
	led_gsck_init(F_CPU/F_LED_GSCK);
	led_blank_init(F_CPU/F_LED_GSCK);
	_delay_ms(10);
	if(!led_ignore_LOD)
		led_xerr_interupt_init();
	
	uint8_t bytes[24]={0}, error=0;
	int16_t i=0,j=2700,k=2700, incr=4;
	uint8_t red_up=1, green_up=0, blue_up=1;
	
	while(1){
		if(red_up)
		i += incr;
		else
		i -= incr;
		
		if(i>=4095){
			i=4095;
			red_up = 0;
		}
		if(i<=0){
			i=0;
			red_up = 1;
		}
		
		led_pack_data(bytes, 0, i);
		
		if(green_up)
		j += incr;
		else
		j -= incr;
		
		if(j>=4095){
			j=4095;
			green_up = 0;
		}
		if(j<=0){
			j=0;
			green_up = 1;
		}
		
		led_pack_data(bytes, 1, j);

		if(blue_up)
		k += incr;
		else
		k -= incr;
		
		if(k>=4095){
			k=4095;
			blue_up = 0;
		}
		if(k<=0){
			k=0;
			blue_up = 1;
		}
		
		led_pack_data(bytes, 2, k);
		
		
		error = led_send_data(bytes);
		if(error == LED_LOD_ERROR){
			if(!led_ignore_LOD)
			led_process_LOD();
		}
		else if(error == LED_TEF_ERROR){
			led_process_TEF();
		}
		led_latch_data();
		while(led_wait_for_XLAT);

	}
}

// ISRs

ISR(TIMER1_COMPB_vect){
	// Remove OCR1B (set to 0xFFFF)
	OCR1B = 0xFFFF;
	led_wait_for_XLAT = 0;
}

ISR(INT2_vect){
	led_blank_stop();
	led_blank_set();
	
	_delay_us(1);
	
	if(PINB&(1<<PB2))		// Read XERR
		led_process_LOD();
	else
		led_process_TEF();
		
	led_blank_clr();
	led_xerr_disable_int();
}

ISR(USART1_RX_vect){
	/* Asynchronously recieve bytes.
	 * Byte Structure: SFLAG | CMD | LEN | D0 | D1 | D2 | ... | SFLAG
	 *            Pos:   0      1     2    3    4    5    ...   LAST
	 */
	uint8_t byte = UDR1;
	
	// Don't Overflow Buffer
	if(pc_cur_pkt_len < sizeof(pc_rx_buf)){		
		if(byte == PC_SFLAG){
			// Long enough and right length to be ending SFLAG
			if((pc_cur_pkt_len >= 3)&&(pc_cur_pkt_len == 3+pc_rx_buf[PC_LEN_POS])){
				pc_rx_buf[pc_cur_pkt_len] = byte;
				pc_cur_pkt_len = 0;
				pc_pkt_ready = 1;
			}
			// Beginning SFLAG
			else{
				pc_rx_buf[0] = byte;
				pc_cur_pkt_len = 1;
			}
			
			pc_escape_found = 0;
		}
		// Regular data after an SFLAG
		else if(pc_cur_pkt_len != 0){
			// Find Escape Character
			if(byte == PC_ESCP){
				pc_escape_found = 1;
			}
			else{
				// Do Byte Unstuffing
				if(pc_escape_found){
					pc_escape_found = 0;
					
					if(byte == PC_EESCP){
						pc_rx_buf[pc_cur_pkt_len] = PC_ESCP;
						pc_cur_pkt_len++;
					}
					else if(byte == PC_ESFLAG){
						pc_rx_buf[pc_cur_pkt_len] = PC_SFLAG;
						pc_cur_pkt_len++;
					}
					// Something Went Wrong, Throw Out Packet
					else{
						pc_cur_pkt_len = 0;
					}
				}
				// Simple Received Byte
				else{
					pc_rx_buf[pc_cur_pkt_len] = byte;
					pc_cur_pkt_len++;
				}
			}				
		}
	}
	else{
		pc_cur_pkt_len = 0;
	}		
}

ISR(BADISR_vect)
{
	while(1);
}
