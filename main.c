/*
 * Timer.c
 *
 * Created: 8/29/2017 2:36:33 PM
 * Author : Esam Abdulsatar
 */ 


/*
 Clock speed 1000000Hz
 8 Bit timer used timer
 clock prescaler = 64
 overflow time =(2^8)/(1000000/64) = 0.016384s
 to make 1s it requires (1/0.016384)=61 overflows 
*/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define LED 0b00000100		// bit location of LED in PORTB
#define P_B1 0b00100000		// bit location of Push Button 1 in PORTC
#define P_B0 0b00010000		// bit location of Push Button 0 in PORTC
#define DIP 0b00001111		// bit location of DIP switches in PORTD
#define RELAY0 0b00000001	// bit location of RELAY0 in PORTB
#define RELAY1 0b00000010	// bit location of RELAY1 in PORTB

#define P_B1_ON (PINC&P_B1)	//Push Button 1 status (masked)
#define P_B0_ON (PINC&P_B0)	//Push Button 0 status (masked)
#define DIP_VAL (PIND&DIP)	//Dip switches value (maksed)

long timer_cc;				// timer counter increment of 0.016384s
long timer_s;				// timer counter increment of 1s
int timer_m;				// timer counter increment of 1m
int cc;						// counter used to flash LED
int dip;					// buffer memory to hold dip switches values
int max_time;


int complete_flag;

void setup(void)			// setup function to initialize 
{
	DDRB=LED|RELAY0|RELAY1;	// set LED, Relay 0 and Relay 1 as output in portB
	TIMSK=0b00000001;		// enable timer 0 overflow interrupt	TCCR0=0b00000011;		// prescaler 64	SFIOR=0b00000001;		// reset the overflow interrupt	TIFR=0b00000001;		// clear overflow interrupt flag		sei();					// enable global interrupts	timer_cc=0;				// reset timer_cc	timer_s=0;				// reset timer_s
	timer_m=99;				// reset timer_m
	complete_flag=0xff;		// set complete flag to complete status
	cc=0;					// reset cc
	
}


ISR(TIMER0_OVF_vect){		// ISR vector
	cc++;
	dip=DIP_VAL;	
	max_time=(dip<<1)+(dip>>1);
		
	TCNT0=0;
	
	//-----------increment timers-----------//
	timer_cc=(timer_cc+1)%61;
	if(timer_cc==60){
		timer_s=(timer_s+1)%61;
	}
	if(timer_s==60 && timer_cc== 0){
		if(timer_m<99){
			timer_m++;
			timer_s=0;
		}
	}
	//--------------------------------------//
	
	
		
	//----------------LED flash-------------//
	if(timer_m>=max_time){
			if(cc&0x40){PORTB|=LED;}else{PORTB&=~LED;}
		}else{
			if(cc&0x08){PORTB|=LED;}else{PORTB&=~LED;}
		}
	//------------------------------------//
	
	//------------relay control-----------//
	if( (timer_m<dip) || ((timer_m>=(dip+(dip>>1))) && timer_m <max_time) ) PORTB|=RELAY0;
	else  PORTB&=~RELAY0;
	//------------------------------------//
    
	//---------Push Button Actions--------//	
		if(P_B1_ON)timer_m=99;
		
		if(P_B0_ON){
			timer_m=0;
			timer_s=0;
			timer_cc=0;
		}
	//------------------------------------//
	
}

int main(void)
{
	setup();
    while (1); 
    
}

