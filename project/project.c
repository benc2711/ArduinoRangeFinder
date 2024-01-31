/********************************************
 *
 *  Name:Ben Crotty
 *  Email: bcrotty@usc.edu
 *  Section: W 330
 *  Assignment: Project
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


#include "lcd.h"

void variable_delay_us(int16_t);
void debounce(uint8_t);
void debounceC(uint8_t);
void timer1_init();
void timer2_init();
void timer0_init();
void play_note();



//set interupt registers


volatile uint8_t changed = 0;  // Flag for ISR
volatile uint8_t check = 0;  // Flag for ISR
volatile uint8_t over = 0;

volatile uint8_t dataflowStart = 0;
volatile uint8_t dataflowEnd = 0;
volatile uint8_t bufferpos = 0;
volatile unsigned char recBuffer[5];

volatile uint16_t isrcount;
volatile uint16_t isreqcount;


volatile int32_t pulse_count = 0;		// Count to display
volatile uint8_t adj_state = 0;

	volatile uint8_t new_state;
	volatile uint8_t old_state;
    volatile uint8_t change = 0;  // Flag for state change
    volatile uint8_t a, b;
	volatile int16_t count = 0;		

	

int main(void) {
	sei();
	PCICR |= ((1 << PCINT1) | (1 << PCINT2));
	PCMSK1 |= (1 << PCINT10); 
	PCMSK2 |= ((1 << PCINT18) | (1 << PCINT19));

	//Receiver Interupts Part 3
	UCSR0B |= (1 << RXCIE0);

	#define FOSC 16000000
	#define BAUD 9600
	#define MYUBRR (FOSC / 16 / BAUD -1)
	UBRR0 = MYUBRR;
	
	timer2_init();
	timer1_init();
	//Buzzer timer init part 3
	timer0_init();
   

    // Initialize DDR and PORT registers and LCD
	lcd_init();
	
	//Rotary initialization
	PORTD |= (1 << PD2) | (1 << PD3);
	//Aquire button initialization pullup resistor
	PORTB |= (1 << PB4);

	DDRB |= (1 << DDB3);

	DDRC |= ((1 << DDC4) | (1 << DDC5));

	//Serial Initialization part 3
	UCSR0B |= ((1 << TXEN0) | (1 << RXEN0));

	UCSR0C |= (3 << UCSZ00);

	//Set Pc3 as output and put zero in 
	DDRC |= (1 << DDC3);
	PORTC &= ~(1 << PC3);

	//Buzzer Initial
	DDRC |= (1 << DDC5);




	


    // Write a spash screen to the LCD

	lcd_writecommand(1);
    lcd_moveto(0,0);
    lcd_stringout("EE109 Project");
    lcd_moveto(1,0);
    lcd_stringout("Ben Crotty");
    _delay_ms(1000);
    lcd_writecommand(1);

	uint8_t x = PIND;
	a = x & (1 << PD2);
	a = (a >> 2);
	
	b = x & (1 << PD3);
	b = (b >> 3);

    if (!b && !a)
	old_state = 0;
    else if (!b && a)
	old_state = 1;
    else if (b && !a)
	old_state = 2;
    else
	old_state = 3;

    new_state = old_state;
	

	// Reading from memory
	int16_t remt;
	int16_t rem;
	remt = eeprom_read_word((void *) 100);
	if(remt != NULL && remt < 400 && remt > 0 ){
		rem = remt;
	}
	else{
		rem =1;
	}

	int16_t loct;
	int16_t loc;
	loct = eeprom_read_word((void *) 200);
	if(loct != NULL && loct < 400 && loct > 0 ){
		loc = loct;
	}
	else{
		PORTC |= (1 << PC4);
		PORTB &= ~(1 << PB5);
		loc = 1;
	}
				
	
    

    while (1) {                 // Loop forever

		

		uint8_t pressedb1 = (PINB & (1 << PB4));
		uint8_t pressedadj = (PINC & (1 << PC0));

		if(!pressedb1){
			debounce(PB4);
			PORTC |= (1 << PC1);
			_delay_ms(10);
			PORTC &= ~(1 << PC1);
		}
		if(!pressedadj){
			debounceC(PC0);
			
			if(adj_state == 0){
				adj_state = 1;
				lcd_moveto(0,0);
				lcd_stringout("       ");
				lcd_moveto(0,0);
    			lcd_stringout("Remote");
				lcd_moveto(1, 0);
				
				count = rem;
				// _delay_ms(500);
				
				
				char buff[7];
				lcd_moveto(1,0);
				lcd_stringout("        ");
				lcd_moveto(1,0);
				snprintf(buff, 9, "Min=%3d", rem);
				lcd_moveto(1,0);
				lcd_stringout(buff);
				
				
			}
			else{
				adj_state = 0;
				lcd_moveto(0,0);
				lcd_stringout("       ");
				lcd_moveto(0,0);
    			lcd_stringout("Local");
				count = loc;

				
				char buff[7];
				lcd_moveto(1,0);
				lcd_stringout("        ");
				lcd_moveto(1,0);
				snprintf(buff, 9, "Min=%3d", loc);
				lcd_moveto(1,0);
				lcd_stringout(buff);
			}

		}
		

		if (check) { // Did the range finding complete?

		// Output count to LCD
			uint32_t pcount = pulse_count / 2; //Amount of microsceconds
			uint32_t distance_mm = (pcount * 10 ) / 58;
			int16_t distance = distance_mm / 10;
			uint8_t decimal_part = distance_mm % 10;
			char buff[6];
			snprintf(buff, 6, "%03d.%d", distance, decimal_part);
			lcd_moveto(0, 11);
			lcd_stringout(buff);

			// Change the servo motor
			OCR2A = distance * (-23)/400  + 35;


			//Send Data Part 3



			char sentDist[6];
			sentDist[0] = '<';
			


			sentDist[1] = (distance / 100) + '0';
			sentDist[2] = ((distance / 10) % 10) + '0';
			sentDist[3] = (distance % 10) + '0';
			sentDist[4] = (decimal_part) + '0';
			sentDist[5] = '>';
			// lcd_moveto(1,1);
			// lcd_stringout(sentDist);
			// _delay_ms(10000);
			char next_character;
			int i = 1;
			while ((UCSR0A & (1 << UDRE0)) == 0) { }
			UDR0 = sentDist[0];

			while (sentDist[i] == '0' && i < 5) {
				i++;
			}

			while (i < 6) {
				next_character = sentDist[i];
				while ((UCSR0A & (1 << UDRE0)) == 0) { }
				UDR0 = next_character;
				i++;
			}
			// lcd_moveto(1,1);
			// lcd_stringout("Sent");
			 //END SENT DATA 

			// OCR2A = 20;

			if(distance >= loc){
				//Green LED
				//pc4 0 pc5 1
				// lcd_moveto(1, 10);
				// lcd_stringout("GREATER");
				PORTB |= (1 << PB5);
				PORTC &= ~(1 << PC4);

			}
			else if(distance < loc){
				// RED LED
				//00
				// lcd_moveto(1, 10);
				// lcd_stringout("lESS");
				PORTC &= ~(1 << PC4);
				PORTB &= ~(1 << PB5);
				

				
			}
			
			check = 0;


		
		}
		if(over){
			lcd_writecommand(1);
			lcd_stringout("Over range");
			_delay_ms(300);
			lcd_writecommand(1);
			over = 0;
			check = 0;
			//Change light to blue 
			PORTC |= (1 << PC4);
			PORTB &= ~(1 << PB5);
		}
		if(change){
			 // Output count to LCD
			char buff[4];
			snprintf(buff, 4, "%3d", count);
			lcd_moveto(1,4);
			lcd_stringout(buff);
			if(adj_state){
				//Store for remote adress;
				
				
				eeprom_update_word((void *) 100, count);
				rem = count;
			}
			else{
				

				//Store for local adress;
				eeprom_update_word((void *) 200, count);
				loc = count;
			}
			change = 0;
		}
		//Show received Data Part 3
		if(dataflowEnd){
			// lcd_moveto(1,1);
			int num1;
			// lcd_stringout(recBuffer);
			sscanf(recBuffer, "%d", &num1);
			int num2 = num1 % 10;
			int numf = num1/10;
			char buff[6];
			snprintf(buff, 6, "%d.%d", numf, num2);
			lcd_moveto(1, 12);
			lcd_stringout(buff);
			if(numf < rem){
				//Sound the buzzer
				// lcd_stringout("less");
				play_note();
			}
			dataflowEnd =0;
		}
		
    }
}
//Need to send zeros and more properly get decimal 





/*
    variable_delay_us - Delay a variable number of microseconds
*/
void variable_delay_us(int delay)
{
    int i = (delay + 5) / 10;

    while (i--)
        _delay_us(10);
}
void debounce(uint8_t bit)
{
    // Add code to debounce input "bit" of PINB
    // assuming we have sensed the start of a press.
    
        _delay_ms(5);
        while ((PINB & (1 << bit)) ==0){}
        _delay_ms(5);   


}
void debounceC(uint8_t bit)
{
    // Add code to debounce input "bit" of PINB
    // assuming we have sensed the start of a press.
    
        _delay_ms(5);
        while ((PINC & (1 << bit)) ==0){}
        _delay_ms(5);   


}

ISR(PCINT1_vect)
{
	// lcd_writecommand(1);
	// lcd_stringout("called");
	
    if(changed ==0){
		TCNT1 = 0;
   		
   		TCCR1B |= (1 << CS11);
		//Load correct prescalar bits
		changed = 1;
	}
	else{
		//At second call
		
		pulse_count = TCNT1;
		//Clear prescalar bits
		TCCR1B &= ~(1 << CS11);
   		TCCR1B &= ~(1 << CS12);
   		TCCR1B &= ~(1 << CS10);
		changed = 0;
		check = 1;
	}
}
ISR(PCINT2_vect){
	
	
		uint8_t x = PIND;
		a = x & (1 << PD2);
		a = (a >> 2);
		
		b = x & (1 << PD3);
		b = (b >> 3);
		
	if (old_state == 0) {

	    // Handle A and B inputs for state 0	
		
    	if (a){
		new_state = 1;
		count++;
		}
    	else if(b){
		new_state = 2;
		count--;
		}


	}
	else if (old_state == 1) {

	    // Handle A and B inputs for state 1
		if (!a){
		new_state = 0;
		count--;
		}
    	else if (b){
		new_state = 3;
		count++;
		}
    	

	}
	else if (old_state == 2) {

	    // Handle A and B inputs for state 2
		if (!b){
		new_state = 0;
		count++;
		}
    	else if (a){
		new_state = 3;
		count--;
		}
    	

	}
	else {   // old_state = 3

	    // Handle A and B inputs for state 3
		
    	if (!b){
		new_state = 1;
		count--;
		}
    	else if (!a){
		new_state = 2;
		count++;
		}
    	

	}

	// If state changed, update the value of old_state,
	// and set a flag that the state has changed.
	if (new_state != old_state) {
	    change = 1;
	    old_state = new_state;
	}
	if(count == 401){
		count = 0;
	}
	if(count ==-1){
		count = 400;
	}
}


void timer2_init()
{
   	TCCR2A |= (0b11 << WGM20);  // Fast PWM mode, modulus = 256
    TCCR2A |= (0b10 << COM2A0); // Turn D11 on at 0x00 and off at OCR2A
    OCR2A = 22;                // Initial pulse duty cycle of 50%
    TCCR2B |= ((1 << CS22) | (1 << CS21) | (1 << CS20));

}
void timer1_init()
{
   TCCR1B |= (1 << WGM12);
   TIMSK1 |= (1 << OCIE1A);
   OCR1A = 46400;
   TCCR1B &= ~(1 << CS11);
   TCCR1B &= ~(1 << CS12);
   TCCR1B &= ~(1 << CS10);

}
ISR(TIMER1_COMPA_vect)
{
	
	TCCR1B &= ~(1 << CS11);
	TCCR1B &= ~(1 << CS12);
	TCCR1B &= ~(1 << CS10);
	over = 1;
	

}


//Receive interupt Part 3
ISR(USART_RX_vect){

	while(!(UCSR0A & (1 << RXC0))){}
	
	unsigned char c = UDR0;
	// char buff[8];
	// snprintf(buff, 8, "%3d", c);
	// lcd_moveto(1,4);
	// lcd_stringout(buff);
	// _delay_ms(1000);

	
	if(c == '<'){
		
		dataflowStart = 1;
		dataflowEnd = 0;
		bufferpos = 0;
	}
	else if(c == '>' && dataflowStart){
		//If buffer count is greater than zero 
		if(bufferpos > 0){
			dataflowEnd = 1;
			recBuffer[bufferpos] ='\0';
		}
		dataflowStart =0;

	}
	else if(dataflowStart){
		if(!(c >= '0' && c<= '9')){
			dataflowStart = 0;
		}
		else if(bufferpos > 5){
			dataflowStart = 0;
		}
		else if(c!= '<' && c!= '>'){
			recBuffer[bufferpos] = c;
			bufferpos++;
			// PORTC ^= (1<< PC5);
		}
		
	}

}

//Buzzer Timer Part 3

void timer0_init(){
	TCCR0A |= (1 << WGM01);
    TIMSK0 |= (1 << OCIE0A);

}
//Buzzer Interupt Part 3
ISR(TIMER0_COMPA_vect){
	if(isrcount == isreqcount){
		TCCR0B &= ~(1 << CS02);
		isrcount = 0;
		PORTC &= ~(1 << PC5);
	}
	else{
		PORTC ^= (1<<PC5);
		isrcount++;
	}
	
}

void play_note(){
	OCR0A = 16000000 / (2*392);
	TCCR0B |= (1 << CS02);
	isreqcount = 2*392;
	isrcount =0;


}
