/*
 * 	main.c
 *  MarcDuino Slave (HoloProjector & Lights Control)
 *  Created on: July 4th, 2012
 *  Author: Marc Verdiell
 *  Version 1.2r: original release 03/26/2013
 *	Version 1.3r: adapting for new JEDI, McWhlr Magic Panel
 *	Version 1.4: consolidate private and release code, HP lights from MarcDuino, PassThrough command
 *	Version 1.5: new buffered serial
 *	Version 1.7: MarcDuino v2 support
 *  Version 1.8: I2C command parsing support, revised serial.c
 *
 */

/***********************
 *  Version 1.8:
 *  I2C command parsing support
 *  Updated serial.c and serial.h libraries
 *  	Changed serial_puts so it waits for available output buffer by default
 *  		If not characters were just dropped when sending data too fast.
 *  	Renamed previous version serial_puts_nowait
 *  Updated the comments in i2c.h
 *
 ***********************/

/**********
 * v1.7
 * MarcDuinoV2 support
 * 	Bumped to version 1.7 to match the Master release
 * 	New suart.c library with stuart2 on PC1 output instead of PC4
 * 	Exchange suart usage to macth MarcDuino v2 board (and Master)
 * 		- Light Output to suart2/PC1 (instead of suart)
 * 		- Slave output on suart/PC0 (instead of suart2)
 * 	I2C library support
 * 	REON holoprojector support
 */

/**********
 * v1.52
 * Added HP test movement command
 * Did not create new project
 */

/**********
 * v1.51
 * Corrected non centered HP servo, add extended movement option
 * (all done in the header files)
 * Did not create new project
 * Corrected debug message version
 */

/** v1.5
 *
 * Up'ed the version number to match the corresponding Dome Panel Control version
 * Switch to Interrupt Driven / Buffered serial implementation
 * Reduce delays after JEDI commands now that suart is more reliable
 */

/** v1.4
 * Add control of HP through our own board
 * 	ON and OF commands will turn on HPs located on srv7-9 headers
 * 	New HP lights commands
 * 	*H1xx, *H2xx, *H3xx, and *H0xx
 * 		Will turn on HP1, 2, 3, and all for xx seconds
 * 	*F1xx, *F2xx, *F3xx, and *F0xx
 * 		Will flicker HP1, 2, 3, and all for xx seconds
 * New library files:
 * 	RS232 with pgm memory for strings
 * 	Refactor sor serial calls
 * 	New realtime library with registered timers
 * 	Remove sequencer files
 *  Added print.c and print.h (convenience, adds 2% of code) (? do I need this)
 * Much smaller DRAM footprint for further expansino
 *  Change most strings constant to program memory strings
 * Support for Alt1 (!) and Alt2 (%) expansion commands
 * 	! commands output on suart (with strip)
 * 	% commands output on suart2 (with strip)
 * Disabled interrupts while transmitting in suart.h -> improved reliability
 */

/** v1.3
 *
 * Created compile switch for 9600 baud rate for new JEDI compatibility
 * Created compile switch for inverted on Magic Panel Output
 * 	(to works with my new Holo Lights and Michael Wheeler's magic panel)
 *
 */


/** Commands implemented in v1.2
 *
 * 	Hardware Setup:
 * 	- 6 servo outputs
 * 		Holo 1 V on pins 1 (front, holo 1)
 * 		Holo 1 H on pins 2
 * 		Holo 2 V on pins 3 (rear, holo 2)
 * 		Holo 2 H on pins 4
 * 		Holo 3 V on pins 5 (top, holo 3)
 * 		Holo 3 H on pins 6
 * 	- 4 digital outputs
 * 		HP1 (front, holo 1) on PORT B Pin 2 (pins labeled SRV7)
 * 		HP2 (rear, holo 2)  on PORT B Pin 3 (pins labeled SRV8)
 * 		HP3 (top, holo 3)   on PORT B Pin 4 (pins labeled SRV9)
 * 		Magic Panel on Port B Pin 5 (pins labeled SRV10)
 * 	- two suarts (software uart) output
 * 		- suart connected to the JEDI Display light control system at 2400 bauds
 * 		- suart2 initialized at 9600 bauds, for future use (daisy-chain expansion?)
 * 	- one RC input
 * 	- one main hardware UART input
 *		- connected to the master panel controller suart output at 9600 bauds, to receive commands
 *		- output sends commands acknowledgments, (not physically connected to anything at present)
 *
 *  Valid start characters recognized in main()
 *  ':' panel command, ignored (see parse_panel_command). This should not be received by this slaved module anyhow
 *  '$' sound command, ignored (see parse_sound_command). This should not be received by this slaved module anyhow
 *  '@' display command, forwarded to JEDI controller on suart1 after stripping the '@' control character
 *  '*' hp command, acted upon here, see below
 *  '!' Alt1 alternate display command, passed to suart after stripping
 *  '%' Alt2 expansion command, passed to suart2 after stripping
 *		The master HP board will forward these to us
 *
 *  Commands recognized (see parse_hp_command)
 *  *RDxx Random Holo movement (xx=01 to 03). xx=00 and >3 all random.
 *  *ONxx Turns Holo Light on (xx=01 to 03). xx=00 or >3 all lights on
 *  *OFxx Turns Holo Lights off (xx=01 to 03). xx=00 turns all lights off
 *  *RCxx Holo vertical movement under RC control, horizontal centered (xx=01-03).
 *  	00 or >3 all RC
 *  *TExx Holo movement test (xx=01-03). Goes through a loop of holo movements
 *  	to assist in adjusting holo servos mechanical setup
 *  	00 or >3 all HPs to test
 *  *STxx stop/reset Holos random movement, turns lights off, and RC off. 00=all off
 *  *HDxx hold: stop holo, do not change light level. 00=all stopped
 *  *MOxx magic panel on. xx=01 to 98, on from 1 to 98 seconds. 00=off, 99=on permanent
 *  *MFxx magic panel flicker xx=1 to 99 flicker for 1 to 99 seconds. 00= off.
 *  *H1xx, *H2xx, *H3xx, and *H0xx
 * 		Will turn on-board HP1, 2, 3, and all (HP0xx) for xx seconds
 * 		99 is on permanently.
 * 		0 is off.
 * 	*F1xx, *F2xx, *F3xx, and *F0xx
 * 		Will flicker on-board HP1, 2, 3, and all (F0xx) for xx seconds
 * 		0 is off.
 * 		(99 is not permanently, but 99 seconds)
 */

#include "main.h"

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>  // needed for the UART interrupt
#include <avr/pgmspace.h>	// for the sequencer data arrays defined with PROGMEM

#include <stdlib.h>			// for itoa(), integer to string conversion
#include <errno.h>			// for errors on atoi (check errno)
#include <stdio.h>			// for sprintf(), don't use if you are short on memory
#include <string.h>			// for strlen()


#include "toolbox.h"		// map, digitalWrite, digitalMode
#include "servo.h"			// servo drivers
#include "realtime.h"		// real time interrupt services
#include "serial.h"			// hardware serial
#include "suart.h"			// software serial (write only)
#include "wmath.h"			// random functions
//#include "sequencer.h"		//  no servo sequencer
//#include "panel_sequences.h"	//  no panel sequences

#ifdef _MARCDUINOV2_
#include "i2c.h"			// include I2C Master libraries for MarcDuino v2
#endif

#define HP_NUM 3			// controlling 3 HPs for RC

// command globals
char command_buffer[CMD_MAX_LENGTH];	// command string buffer

 // Controls verbosity level - affecting i2c for now
 uint8_t errormessageon=_ERROR_MSG_;
 uint8_t feedbackmessageon=_FEEDBACK_MSG_;

// flags for the HP control mode (0=nothing, 1=random, 2=RC)
static uint8_t hp1_control=0;
static uint8_t hp2_control=0;
static uint8_t hp3_control=0;
// Magic panel and holo effect depends on the value of the following global flags:
// if it's 0, it's off
// if it's 1, it's on
// if it's 2, it's on as long as rt_timeout_magic_panel is non-zero
// if it's 3, random flicker as long as rt_timeout_magic_panel is non-zero
static uint8_t magic_control=0;
static uint8_t hp1_light_control=0;
static uint8_t hp2_light_control=0;
static uint8_t hp3_light_control=0;


// MarcDuino V2, holo states for I2C
// set them to 1 initially or they won't turn off during init...
uint8_t holo1state=1;
uint8_t holo2state=1;
uint8_t holo3state=1;

// timers

rt_timer move_timer_HP1;		// HP1 random movement timer
rt_timer move_timer_HP2;		// HP2 random movement timer
rt_timer move_timer_HP3;		// HP3 random movement timer

rt_timer on_timer_magic;		// magic on-time timer
rt_timer flicker_timer_magic;	// magic panel flicker timer

rt_timer on_timer_HP1;			// HP1 on-time timer
rt_timer on_timer_HP2;			// HP2 on-time timer
rt_timer on_timer_HP3;			// HP3 on-time timer
rt_timer flicker_timer_HP1; 	// HP1 flicker timer
rt_timer flicker_timer_HP2; 	// HP2 flicker timer
rt_timer flicker_timer_HP3; 	// HP3 flicker timer

rt_timer test_timer;			// For HP movement test

// string constants are in program memory to save DRAM
const char strOK[] PROGMEM="OK\n\r";
const char strWelcome[] PROGMEM="\n\rMarcDuino HP Control v1.52 \n\r";
const char strEnterPrompt[] PROGMEM="Enter HP or Display command starting with \'*\' or \'@\'\n\r";
const char strSuart1OK[] PROGMEM="\n\rsuart1 Communication OK \n\r";
const char strSuart2OK[] PROGMEM="\n\rsuart2 Communication OK \n\r";
const char strStartCharErr[] PROGMEM="**Unrecognized Command Start Character\r\n";

// utility to echo characters back cleanly
void echo(char ch)
{
	// echo return and line feeds nicely on a terminal
	if(ch=='\r' || ch=='\n' || ch == 0x0D )
	{
		serial_putc('\n');
		serial_putc('\r');
	}
	else serial_putc(ch);
}

int main(void) {

	// start hardware and software UARTs, send check string
	serial_init_9600b8N1();	// 9600 bauds, 8 bits, 1 stop, no parity, use for a regular terminal console
	serial_puts_p(strWelcome);
	serial_puts_p(strEnterPrompt);


// initialize suart used to communicate with the JEDI Display at 2400 or 9600 bauds
	uint16_t baudrate;
#ifdef _9600BAUDSJEDI_
	baudrate=9600;
#else
	baudrate=2400;
#endif
	lightsuart_init(baudrate);


	//suart_puts_p(strSuart1OK);
	//_delay_ms(200);

// suart used for slave out initialized at 9600 bauds, used for Alt2 commands
	slavesuart_init(9600);
#ifdef _MARCDUINOV2_
	slavesuart_puts_p(strSuart1OK); // on suart1 for MarcDuino v2
#else
	slavesuart_puts_p(strSuart2OK);	// on suart2 for MarcDuino v1
#endif


	// abort test routine, and optionally do extra Jedi programmatic setup
	_delay_ms(3000);		// wait that the display system is up
	init_jedi();

	// Received command characters from uart0 are processed asynchronously via interrupt handler
	// further below. They will be used to fill the 'command_buffer' string.
	// The 'command_buffer_full' flag will be raised when the full command is available.
	// The buffer is then read and processed in the main loop.
	serial_enable_rx_interrupt();

	// initialize servo and realtime units
	servo_init();
	realtime_init();

#ifdef _MARCDUINOV2_
	// initialize I2C hardware on MarcDuino v2's with 10k pull-up resistors on.
	i2c_init(TRUE);
#endif

	// register all our timers and timeouts

	rt_add_timer(&move_timer_HP1);
	rt_add_timer(&move_timer_HP2);
	rt_add_timer(&move_timer_HP3);
	rt_add_timer(&flicker_timer_magic);
	rt_add_timer(&on_timer_magic);
	rt_add_timer(&on_timer_HP1);
	rt_add_timer(&on_timer_HP2);
	rt_add_timer(&on_timer_HP3);
	rt_add_timer(&flicker_timer_HP1);
	rt_add_timer(&flicker_timer_HP2);
	rt_add_timer(&flicker_timer_HP3);
	rt_add_timer(&test_timer);

	// initialize magic panel and holo lights as output
	digitalMode(MAGIC_PORT, MAGIC_PIN, OUTPUT);
	digitalMode(HP1_LIGHT_PORT, HP1_LIGHT_PIN, OUTPUT);
	digitalMode(HP2_LIGHT_PORT, HP2_LIGHT_PIN, OUTPUT);
	digitalMode(HP3_LIGHT_PORT, HP3_LIGHT_PIN, OUTPUT);

	// turn off all lights
	holo1OFF();
	holo2OFF();
	holo3OFF();
	MAGIC_OFF;

  while (1)
  {
	/////////////////////////////////////////
	// Serial Command Input
	////////////////////////////////////////
	char command_str[CMD_MAX_LENGTH];
	bool command_available;

	// check for command line input
	if(serial_available())
	{
		char ch;
		ch=serial_getc();										// get input
		echo(ch);												// echo back
		command_available=build_command(ch, command_str);		// build command line
		if (command_available) dispatch_command(command_str);	// send command line to dispatcher
	}

	///////////////////////////////////////////
	// RC and random pattern of HP servos
	//////////////////////////////////////////

	 // How the servo is control depends on hpx_control flags
	 // if it's 0, nothing happens
	 // if it's 1, random pattern
	 // if it's 2, RC control of vertical servo

	 // *********  HP RC movement  ************************

	 if(hp1_control==2) {servo_set(FRONT_HP_SERVO_H, 1500); servo_set(FRONT_HP_SERVO_V, servo_RCread());}
	 if(hp2_control==2) {servo_set(REAR_HP_SERVO_H, 1500); servo_set(REAR_HP_SERVO_V, servo_RCread());}
	 if(hp3_control==2) {servo_set(TOP_HP_SERVO_H, 1500); servo_set(TOP_HP_SERVO_V, servo_RCread());}

	 // *********  HP random movement  ************************
	 if(hp1_control==1)
	 {
		 if(move_timer_HP1==0)
		 {
			 //move_timer_HP1 = rand_howsmall_howbig(50, 500);
			 move_timer_HP1 = rand_howsmall_howbig(20, 200);
			 servo_set(FRONT_HP_SERVO_H, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
			 servo_set(FRONT_HP_SERVO_V, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
		 }
	 }
	 if(hp2_control==1)
	 {
		 if(move_timer_HP2==0)
		 {
			 //move_timer_HP2 = rand_howsmall_howbig(50, 500);
			 move_timer_HP2 = rand_howsmall_howbig(20, 200);
			 servo_set(REAR_HP_SERVO_H, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
			 servo_set(REAR_HP_SERVO_V, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
		 }
	 }
	 if(hp3_control==1)
	 {
		 if(move_timer_HP3==0)
		 {
			 //move_timer_HP3 = rand_howsmall_howbig(50, 500);
			 move_timer_HP3 = rand_howsmall_howbig(20, 200);
			 servo_set(TOP_HP_SERVO_H, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
			 servo_set(TOP_HP_SERVO_V, rand_howsmall_howbig(SERVO_HP_MIN,SERVO_HP_MAX));
		 }
	 }

	 // *********  HP test movement  ************************
	 // move servos to all extreme positions to adjust HP
	 if(hp1_control==3 || hp2_control==3 || hp3_control==3)
	 {
		 if(test_timer==0)
		 {
			 static uint8_t step;
			 static uint16_t a,b;
			 if (step>=17)step=0;
			 step ++;
			 test_timer = 200;
			 switch (step)
			 {
			 	 case 1: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 2: // up
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b= SERVO_HP_MAX;
			 		break;
			 	 case 3: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 4: // down
				 	a= (SERVO_HP_MIN+SERVO_HP_MAX)/2;
				 	b= SERVO_HP_MIN;
			 		break;
			 	 case 5: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 6: // right
			 		a= SERVO_HP_MAX;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 7: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 8: // left
			 		a= SERVO_HP_MIN;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 9: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 10: // up/right
			 		a= SERVO_HP_MAX;
			 		b= SERVO_HP_MAX;
			 		break;
			 	 case 11: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 12: // down/right
			 		a= SERVO_HP_MAX;
			 		b= SERVO_HP_MIN;
			 		break;
			 	 case 13: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 14: // down/left
			 		a= SERVO_HP_MIN;
			 		b= SERVO_HP_MIN;
			 		break;
			 	 case 15: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 case 16: // up/left
			 		a= SERVO_HP_MIN;
			 		b= SERVO_HP_MAX;
			 		break;
			 	 case 17: // center
			 		a=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		b=(SERVO_HP_MIN+SERVO_HP_MAX)/2;
			 		break;
			 	 default:
			 		 break;
			 }
			 if(hp1_control==3)
			 	 {servo_set(FRONT_HP_SERVO_H, a); servo_set(FRONT_HP_SERVO_V, b);}
			 if(hp2_control==3)
			 	 {servo_set(REAR_HP_SERVO_H, a); servo_set(REAR_HP_SERVO_V, b);}
			 if(hp3_control==3)
			 	 {servo_set(TOP_HP_SERVO_H, a); servo_set(TOP_HP_SERVO_V, b);}
		 }
	 }

	 ///////////////////////////////////////////////
	 // Magic panel control
	 //////////////////////////////////////////////

	 // Magic panel effect depends on the 'magic_control' flag:
	 // if it's 0, it's off
	 // if it's 1, it's on
	 // if it's 2, it's on as long as rt_timeout_magic_panel is non-zero
	 // if it's 3, random flicker as long as rt_timeout_magic_panel is non-zero

	 switch(magic_control)
	 {
	 	 case 0:
	 		 MAGIC_OFF;
	 		 break;
	 	 case 1:
	 		 MAGIC_ON;
	 		 break;
	 	 case 2:
	 		 if(on_timer_magic) MAGIC_ON;
	 		 else MAGIC_OFF;
	 		 break;
	 	 case 3:
	 		 if(!on_timer_magic) MAGIC_OFF;
	 		 else
	 		 {
	 			 if(!flicker_timer_magic)	// end of random flicker period
	 			 {
	 				// Michael Wheeler magic panel has reversed polarity
					#ifdef _REVERSEMAGICPANEL_
	 				if(!digitalRead(MAGIC_PORT,MAGIC_PIN)) // panel was on
					#else
	 				if(digitalRead(MAGIC_PORT,MAGIC_PIN)) // panel was on
					#endif
	 				{
	 					flicker_timer_magic=rand_howsmall_howbig(3,10);
	 					MAGIC_OFF;
	 				}
	 				else
	 				{
	 					flicker_timer_magic=rand_howsmall_howbig(5,20);
	 					MAGIC_ON;
	 				}
	 			 }
	 		 }
	 		 break;
	 	 default:
	 		 break;
	 }

	 ///////////////////////////////////////////////
	 // HP Light control
	 //////////////////////////////////////////////

	 // HP light mode depends on the 'hpx_light_control' flag:
	 // if it's 0, it's off
	 // if it's 1, it's on
	 // if it's 2, it's on as long as on_timer_HPx is non-zero
	 // if it's 3, random flicker as long as on_timer_HPx is non-zero


	 switch(hp1_light_control)
	 {
		 case 0:
			 holo1OFF();
			 break;
		 case 1:
			 holo1ON();
			 break;
		 case 2:
			 if(on_timer_HP1) holo1ON();
			 else holo1OFF();
			 break;
		 case 3:
			 if(!on_timer_HP1) holo1OFF();
			 else
			 {
				 if(!flicker_timer_HP1)	// end of random flicker period
				 {
					// check if panel was on
					#ifdef	_REVERSEHOLOS_
					if(!digitalRead(HP1_LIGHT_PORT,HP1_LIGHT_PIN))
					#else
					if(digitalRead(HP1_LIGHT_PORT,HP1_LIGHT_PIN))
					#endif
					{
						flicker_timer_HP1=rand_howsmall_howbig(3,10);
						holo1OFF();
					}
					else
					{
						flicker_timer_HP1=rand_howsmall_howbig(5,20);
						holo1WHITE();
					}
				 }
			 }
			 break;
		 default:
			 break;
	 }

	 switch(hp2_light_control)
	 {
		 case 0:
			 holo2OFF();
			 break;
		 case 1:
			 holo2ON();
			 break;
		 case 2:
			 if(on_timer_HP2) holo2ON();
			 else holo2OFF();
			 break;
		 case 3:
			 if(!on_timer_HP2) holo2OFF();
			 else
			 {
				 if(!flicker_timer_HP2)	// end of random flicker period
				 {
					// check if panel was on
					#ifdef	_REVERSEHOLOS_
					if(!digitalRead(HP2_LIGHT_PORT,HP2_LIGHT_PIN))
					#else
					if(digitalRead(HP2_LIGHT_PORT,HP2_LIGHT_PIN))
					#endif
					{
						flicker_timer_HP2=rand_howsmall_howbig(3,10);
						holo2OFF();
					}
					else
					{
						flicker_timer_HP2=rand_howsmall_howbig(5,20);
						holo2WHITE();
					}
				 }
			 }
			 break;
		 default:
			 break;
	 }

	 switch(hp3_light_control)
	 {
		 case 0:
			 holo3OFF();
			 break;
		 case 1:
			 holo3ON();
			 break;
		 case 2:
			 if(on_timer_HP3) holo3ON();
			 else holo3OFF();
			 break;
		 case 3:
			 if(!on_timer_HP3) holo3OFF();
			 else
			 {
				 if(!flicker_timer_HP3)	// end of random flicker period
				 {
					// check if panel was on
					#ifdef	_REVERSEHOLOS_
					if(!digitalRead(HP3_LIGHT_PORT,HP3_LIGHT_PIN))
					#else
					if(digitalRead(HP3_LIGHT_PORT,HP3_LIGHT_PIN))
					#endif
					{
						flicker_timer_HP3=rand_howsmall_howbig(3,10);
						holo3OFF();
					}
					else
					{
						flicker_timer_HP3=rand_howsmall_howbig(5,20);
						holo3WHITE();
					}
				 }
			 }
			 break;
		 default:
			 break;
	 }

	/***** simple debug RC input test: loopback input to servo */
	//servo_set(2, servo_RCread());

	/***** simple input RC diagnostic test: prints out the RC input value
	char string[17]; 				// put outside while loop?
	sprintf(string, "RC-in: %04d \r\n", servo_RCread());
	serial_puts(string);
	*/

  }
return 0;
}

// builds the command string from the character input
uint8_t build_command(char ch, char* output_str)
{
	static uint8_t pos=0;

	switch(ch)
	{
		case CMD_END_CHAR:								// end character recognized
			command_buffer[pos]='\0';					// append the end of string character
			pos=0;										// reset buffer pointer
			strcpy(output_str, (char*)command_buffer);	// copy result
			return TRUE;								// return and signal command ready
			break;

		default:										// regular character
			command_buffer[pos]=ch;						// append the  character to the command string
			if(pos<=CMD_MAX_LENGTH-1)pos++;				// too many characters, discard them.
			break;
	}
	return FALSE;
}

// dispatches further command processing depending on start character
void dispatch_command(char* command_str)
{
	char start_char=command_str[0];
	uint8_t length=strlen(command_str);

	// prompt on empty command to show life at console
	if(length==0)
	{
		serial_puts_p(strOK);
		return;
	}

	// dispatch the command to the appropriate parser depending on start character
	switch(start_char)
	{
	 case PANEL_START_CHAR:
		 parse_panel_command(command_str, length);
		 break;
	 case HP_START_CHAR:
		 parse_hp_command(command_str, length);
		 break;
	 case DISPLAY_START_CHAR:
		 parse_display_command(command_str,length);
		 break;
	 case SOUND_START_CHAR:
		parse_sound_command(command_str,length);
		break;
	 case ALT1_START_CHAR:
		parse_alt1_command(command_str,length);
		break;
	 case ALT2_START_CHAR:
		parse_alt2_command(command_str,length);
		break;
	 case I2C_START_CHAR:
		 parse_i2c_command(command_str,length);
		 break;
	 default:
		 if(errormessageon) serial_puts_p(strStartCharErr);
		 break;
	}
}


const char strPanelCommandIgnored[] PROGMEM="Panel command - ignored\r\n";
void parse_panel_command(char* command, uint8_t length)
{
	// we should not receive any of this, ignore
	if(feedbackmessageon) serial_puts_p(strPanelCommandIgnored);
}

const char strDisplayCommandForwarded[] PROGMEM="Display command - forwarded to display\r\n";
void parse_display_command(char* command, uint8_t length)
{
	// forward the command minus the @ character to the JEDI display
	if(feedbackmessageon) serial_puts_p(strDisplayCommandForwarded);
	if(length>=3)	// command must have at least a start, a character and an end
	{
		lightsuart_puts(command+1); // discard the start character, JEDI display doesn't use any
		lightsuart_putc('\r');		// add the termination character
	}
}

const char strSoundCommandIgnored[] PROGMEM="Sound command - ignored\r\n";
void parse_sound_command(char* command,uint8_t length)
{
	// we should not receive any of this, ignore
	if(feedbackmessageon) serial_puts_p(strSoundCommandIgnored);
}

// for custom/future expansion,
// Forward command to Lights with start character dropped
// However in normal use the Master MarcDuino catches these, so we don't see them
const char strAlt1Command[] PROGMEM="Alt1 command, output to lights\r\n";
void parse_alt1_command(char* command, uint8_t length)
{
	if(feedbackmessageon) serial_puts_p(strAlt1Command);

	lightsuart_puts(command+1); // discard the start character
	lightsuart_putc('\r');		// add the termination character
}

// for custom/future expansion
// Forward command to next slave after stripping
// These were already forwarded to us in the same manner by the master MarcDuino
const char strAlt2Command[] PROGMEM="Alt2 command, output to slave out\r\n";
void parse_alt2_command(char* command, uint8_t length)
{
	if(feedbackmessageon) serial_puts_p(strAlt1Command);

	slavesuart_puts(command+1);  // discard the start character
	slavesuart_putc('\r');		// add the termination character
}


const char strHPCmdErr[] PROGMEM="**Invalid Panel Command\r\n";
void parse_hp_command(char* command_string, uint8_t length)
{
	/************************************
	 * Command vocabulary
	 * * as start command sign
	 * ST as STop holo
	 * 	argument: 01 02 03 holo number, 00= all holos
	 * RD as RanDom sequence:
	 *  01 02 03 holo number, 00= all holos
	 * ON as turn HP ON
	 *  01 02 03 holo number, 00= all holos
	 * OF as turn HP OFF
	 * RC as control HP (vertical) via RC
	 * 	01 02 03 holo number,
	 * 	00= no RC control
	 *  >03 control all HPs
	 * TE as test HP
	 * 	01 02 03 holo number,
	 *  00= no test
	 */

	char cmd[3];
	char arg[3];

	// a properly constructed command should have 5 chars
	if (length!=5)
	{
		if(errormessageon) serial_puts_p(strHPCmdErr);
		return;
	}

	// character '*' begins command, should have been already checked if this is called
	if(command_string[0]!=HP_START_CHAR)
	{
		if(errormessageon) serial_puts_p(strHPCmdErr);
		return;
	}

	cmd[0]=command_string[1];
	cmd[1]=command_string[2];
	cmd[2]='\0';
	arg[0]=command_string[3];
	arg[1]=command_string[4];
	arg[2]='\0';

	process_command(cmd, arg);
}

const char strI2CCmdErr[] PROGMEM="**Invalid I2C Command\r\n";
void parse_i2c_command(char* cmd, uint8_t length)
{
	/*****************************************
	 * I2C Command structure:
	 *
	 * &device_address(in decimal),<arg>,<arg>,<arg>\r
	 *
	 * Where <arg> is to be sent on i2c as a byte according to
	 * one of four converions:
	 * 	decimal number: 210 (no prefix, max 255, min -128)
	 * 	hex number: xA7 (x prefix, max FF)
	 * 	single char: 'c' (' prefix, optional ' at the end)
	 * 	ASCII string:"hello world (" prefix, no other " at the end)
	 *
	 * Note that numbers are one byte max. To send larger numbers
	 * you have to break them in hex bytes and send them one byte
	 * at a time
	 *
	 * To debug it is useful to set #define _FEEDBACK_MSG_ 1
	 * in main.h, it shows how it interpreted the command payload
	 * on the serial console output
	 *
	 */

	uint8_t i2caddress=0;
	uint8_t payload[256];
	uint8_t payloadIndex=0;
	uint8_t success=0;
	const char delim[]=",";
	char* token;
	//### for debug output
	char str[65];

	// a properly constructed command should have at least 2 chars
	if (length<2)
	{
		if(errormessageon) serial_puts_p(strI2CCmdErr);
#if _FEEDBACK_MSG_ == 1
		serial_puts("Err 1\n\r");
#endif
		return;
	}

	// check first character '&' begins command
	if(cmd[0]!=I2C_START_CHAR)
	{
		if(errormessageon) serial_puts_p(strI2CCmdErr);
#if _FEEDBACK_MSG_ == 1
		serial_puts("Err 2\n\r");
#endif
		return;
	}

	// good enough to send on to the next slave
	// so all slaves execute the same I2C command
	suart2_puts(cmd);
	suart2_putc('\r');	// add the termination character

	// get the address field. Need to tokenize on the next "," or "\0"
	token = strtok(cmd+1, delim);
	if(token == NULL )
	{
		if(errormessageon) serial_puts_p(strI2CCmdErr);
#if _FEEDBACK_MSG_ == 1
		serial_puts("Err 3\n\r");
#endif
		return;
	}

	// convert and check the address
	unsigned int temp;
	success=sscanf(token, "%u", &temp);
	//i2caddress=atoi(token);
	// make sure I can do the conversion to uint8_t
	if(temp<255) i2caddress=(uint8_t)temp;
	else success=0;

	//### confirm first address token is read correctly
	if (success) sprintf(str, "Token: %s, recognized address: %u \r\n", token, i2caddress);
	else sprintf(str, "Token: %s, unrecognized address\r\n", token);
	serial_puts(str);

	if(i2caddress > 127 || !success)
	{
		if(errormessageon) serial_puts_p(strI2CCmdErr);
#if _FEEDBACK_MSG_ == 1
		serial_puts("Err 4\n\r");
#endif
		return;
	}

	// get all arguments separated by commas
	while(token!=NULL)
	{
		token=strtok(NULL, delim); 	// get next token
		if(token==NULL) break;		// exit immediately if token no good

#if _FEEDBACK_MSG_ == 1	// verify token
		serial_puts("Token: "); serial_puts(token); serial_puts("\r\n");
#endif

		// try to convert and append to payload
		success=append_token(payload, &payloadIndex, token);

#if _FEEDBACK_MSG_ == 1 // verify payload
		if(success) serial_puts("Data Good - ");
		else serial_puts("Data Error - ");
		sprintf(str, "Index = %u \r\n", payloadIndex);
		serial_puts(str);
#endif

		//break immediately on payload error
		if(!success) break;
	}

	// send the I2C command if good payload
	if(success && payloadIndex!=0)	sendI2C(i2caddress, payload, payloadIndex);
	else
	{
		if(errormessageon) serial_puts_p(strI2CCmdErr);
#if _FEEDBACK_MSG_ == 1
		serial_puts("Err 5\n\r");
#endif
	}
}

uint8_t append_token(uint8_t* payload, uint8_t* index, char* token)
{
	uint8_t result=0;
	unsigned int unum;
	int num;
	char ch;
	uint8_t i;
	switch(token[0])
	{
		case 'x':	// hex character
			result=sscanf(token+1, "%x", &unum); // skip the x and read the hex number
			if(result)
			{
				if(unum>255) return 0; // limited to 8 bit hex values
				payload[*index]=(uint8_t)unum;
				(*index)++;
				if (*index==0) return 0; // force error on max payload overrun
			}
			break;
		case '"':	// string
			i=1; 	// start after the "
			while(i<255)
			{
				ch=token[i];				// read all the characters in the token
				if(ch=='\0') break;			// end of string
				payload[*index]=ch;			// put in the payload
				(*index)++;					// advance payload index
				if (*index==0) return 0; 	// index wrapped around, exit with error
				i++;						// advance string index
			}
			result=1;
			break;
		case '\'':	// single character
			result=sscanf(token+1, "%c", &ch);
			if(result)
			{
				payload[*index]=ch;
				(*index)++;
				if (*index==0) return 0;
			}
			break;
		default:
			// I have problem here if I get a 16 bit int and it doesn't fit in an int8_t or uint8_t.
			// So I am reducing the allowed range to -128 / +255
			result=sscanf(token, "%d", &num);
			if(result)
			{
				if(num>255 || num<-128) return 0; 				// limited to 8 bit signed or unsigned arguments
				if(num<=127) payload[*index]=(int8_t)num;	// allow signed from -128 to 127
				else payload[*index]=(uint8_t)num;			// but allow unsigned numbers up to 255
				(*index)++;
				if (*index==0) return 0; // force error on max payload overrun
			}
			break;
	}
	return result;
 }

void sendI2C(uint8_t address, uint8_t* payload, uint8_t payload_length)
{

#if _FEEDBACK_MSG_ == 1 // ifdef it to save memory when not using
	serial_puts("### RESULT IS ####\r\n");
	char str[65];
	sprintf(str, "I2C address = %u \r\n", address);
	serial_puts(str);
	sprintf(str, "Payload length= %u \r\n", payload_length);
	serial_puts(str);
	serial_puts("Payload = \r\n");
	for(uint8_t i=0; i<payload_length; i++)
	{
		sprintf(str, "Byte %u = %u \r\n", i, payload[i]);
		serial_puts(str);
	}
	serial_puts("\r\n");
#endif
	// send the data via i2c
	i2c_send_data(address,payload, payload_length, TRUE);
}

void process_command(char* thecommand, char* theargument)
{
	// hex conversion example
	// result=(int)strtol(str,(char **)NULL,16);
	// for now use base10 value conversion
	uint8_t value;
	// char string[35];
	value=atoi(theargument);

	if(strcmp(thecommand,CMD_HOLD)==0)
	{
		serial_puts_p(strOK);
		hold_command(value);
		return;
	}

	if(strcmp(thecommand,CMD_STOP)==0)
	{
		serial_puts_p(strOK);
		stop_command(value);
		return;
	}

	if(strcmp(thecommand,CMD_RANDOM )==0)
	{
		serial_puts_p(strOK);
		random_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_ON)==0)
	{
		serial_puts_p(strOK);
		on_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_OFF)==0)
	{
		serial_puts_p(strOK);
		off_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_RC )==0)
	{
		serial_puts_p(strOK);
		rc_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_TEST )==0)
	{
		serial_puts_p(strOK);
		test_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_MAGIC_ON )==0)
	{
		serial_puts_p(strOK);
		magic_on_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_MAGIC_FLICKER )==0)
	{
		serial_puts_p(strOK);
		magic_flicker_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO1_FLICKER )==0)
	{
		serial_puts_p(strOK);
		holo1_flicker_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO2_FLICKER )==0)
	{
		serial_puts_p(strOK);
		holo2_flicker_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO3_FLICKER )==0)
	{
		serial_puts_p(strOK);
		holo3_flicker_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_ALL_HOLO_FLICKER )==0)
	{
		serial_puts_p(strOK);
		all_holo_flicker_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO1_FLASH )==0)
	{
		serial_puts_p(strOK);
		holo1_flash_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO2_FLASH )==0)
	{
		serial_puts_p(strOK);
		holo2_flash_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_HOLO3_FLASH )==0)
	{
		serial_puts_p(strOK);
		holo3_flash_command(value);
		return;
	}
	if(strcmp(thecommand,CMD_ALL_HOLO_FLASH )==0)
	{
		serial_puts_p(strOK);
		all_holo_flash_command(value);
		return;
	}
	if(errormessageon) serial_puts_p(strHPCmdErr);
}

const char strHP1Stop[] PROGMEM="(HP1 stop) \r\n";
const char strHP2Stop[] PROGMEM="(HP2 stop) \r\n";
const char strHP3Stop[] PROGMEM="(HP3 stop) \r\n";
const char strHPAllStop[] PROGMEM="(All HPs  stop) \r\n";

void stop_command(uint8_t value)
{
	uint8_t i;

	switch(value)
	{
		case 1:
			serial_puts_p(strHP1Stop);
			hp1_control=0;
			off_command(1);
			servo_set(FRONT_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(FRONT_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 2:
			serial_puts_p(strHP2Stop);
			hp2_control=0;
			off_command(2);
			servo_set(REAR_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(REAR_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 3:
			serial_puts_p(strHP3Stop);
			hp3_control=0;
			off_command(3);
			servo_set(TOP_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(TOP_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 0:
		default:
			serial_puts_p(strHPAllStop);
			hp1_control=0;
			hp2_control=0;
			hp3_control=0;
			off_command(0);
			// stop the 3 servos
			for(i=1; i<=HP_NUM*2; i++)
			{
				servo_set(i, SERVO_NO_PULSE);
			}
			break;
	}
}

const char strHP1Hold[] PROGMEM="(HP1 hold) \r\n";
const char strHP2Hold[] PROGMEM="(HP2 hold) \r\n";
const char strHP3Hold[] PROGMEM="(HP3 hold) \r\n";
const char strHPAllDHold[] PROGMEM="(All HPs  hold) \r\n";
void hold_command(uint8_t value)
{
	// like stop/reset, except just stops movement, does not turn off the lights

	uint8_t i;

	switch(value)
	{
		case 1:
			if(feedbackmessageon) serial_puts_p(strHP1Hold);
			hp1_control=0;
			servo_set(FRONT_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(FRONT_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 2:
			if(feedbackmessageon) serial_puts_p(strHP2Hold);
			hp2_control=0;
			servo_set(REAR_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(REAR_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 3:
			if(feedbackmessageon) serial_puts_p(strHP3Hold);
			hp3_control=0;
			servo_set(TOP_HP_SERVO_H,SERVO_NO_PULSE);
			servo_set(TOP_HP_SERVO_V,SERVO_NO_PULSE);
			break;
		case 0:
		default:
			if(feedbackmessageon) serial_puts_p(strHPAllDHold);
			hp1_control=0;
			hp2_control=0;
			hp3_control=0;
			// stop the 3 servos
			for(i=1; i<=HP_NUM*2; i++)
			{
				servo_set(i, SERVO_NO_PULSE);
			}
			break;
	}
}

const char strHP1Rand[] PROGMEM="(HP1 random) \r\n";
const char strHP2Rand[] PROGMEM="(HP2 random) \r\n";
const char strHP3Rand[] PROGMEM="(HP3 random) \r\n";
const char strHPAllDRand[] PROGMEM="(All HPs  random) \r\n";
void random_command(uint8_t value)
{
	switch(value)
	{

		case 1:
			if(feedbackmessageon) serial_puts_p(strHP1Rand);
			hp1_control=1;
			break;
		case 2:
			if(feedbackmessageon) serial_puts_p(strHP2Rand);
			hp2_control=1;
			break;
		case 3:
			if(feedbackmessageon) serial_puts_p(strHP3Rand);
			hp3_control=1;
			break;
		case 0:	// all holos into random movement mode
		default:
			if(feedbackmessageon) serial_puts_p(strHPAllDRand);
			hp1_control=hp2_control=hp3_control=1;
			break;
	}
}

const char strHP1On[] PROGMEM="(HP1 on) \r\n";
const char strHP2On[] PROGMEM="(HP2 on) \r\n";
const char strHP3On[] PROGMEM="(HP3 on) \r\n";
const char strHPAllDOn[] PROGMEM="(All HPs  on) \r\n";
void on_command(uint8_t value)
{
	switch(value)
	{
		case 1:
			if(feedbackmessageon) serial_puts_p(strHP1On);
			hp1_light_control=1;
			lightsuart_puts("6T1\r"); // turn the light on (front)
			break;
		case 2:
			if(feedbackmessageon) serial_puts_p(strHP2On);
			hp2_light_control=1;
			lightsuart_puts("7T1\r"); // turn the light on (rear)
			break;
		case 3:
			if(feedbackmessageon) serial_puts_p(strHP3On);
			hp3_light_control=1;
			lightsuart_puts("8T1\r"); // turn the light on (top)
			break;
		case 0:
		default:
			if(feedbackmessageon) serial_puts_p(strHPAllDOn);
			hp1_light_control=1;
			hp2_light_control=1;
			hp3_light_control=1;
			lightsuart_puts("6T1\r"); // turn the light on (front)
			_delay_ms(100);
			lightsuart_puts("7T1\r"); // turn the light on (rear)
			_delay_ms(100);
			lightsuart_puts("8T1\r"); // turn the light on (top)
			break;
	}
}

const char strHP1Off[] PROGMEM="(HP1 off) \r\n";
const char strHP2Off[] PROGMEM="(HP2 off) \r\n";
const char strHP3Off[] PROGMEM="(HP3 off) \r\n";
const char strHPAllDOff[] PROGMEM="(All HPs  off) \r\n";
void off_command(uint8_t value)
{
	switch(value)
	{
		case 1:
			if(feedbackmessageon) serial_puts_p(strHP1Off);
			hp1_light_control=0;
			lightsuart_puts("6D\r"); // turn the light off (front)
			break;
		case 2:
			if(feedbackmessageon) serial_puts_p(strHP2Off);
			hp2_light_control=0;
			lightsuart_puts("7D\r"); // turn the light off (rear)
			break;
		case 3:
			if(feedbackmessageon) serial_puts_p(strHP3Off);
			hp3_light_control=0;
			lightsuart_puts("8D\r"); // turn the light off (top)
			break;
		case 0:
		default:
			if(feedbackmessageon) serial_puts_p(strHPAllDOff);
			hp1_light_control=0;
			hp2_light_control=0;
			hp3_light_control=0;
			lightsuart_puts("6D\r"); // turn the light on (front)
			_delay_ms(100);
			lightsuart_puts("7D\r"); // turn the light on (rear)
			_delay_ms(100);
			lightsuart_puts("8D\r"); // turn the light on (top)
			break;
	}
}

const char strHP1RC[] PROGMEM="(HP1 RC) \r\n";
const char strHP2RC[] PROGMEM="(HP2 RC) \r\n";
const char strHP3RC[] PROGMEM="(HP3 RC) \r\n";
const char strHPAllRC[] PROGMEM="(All HPs RC) \r\n";
void rc_command(uint8_t value)
{

	switch(value)
	{
		// release control of all the HP that were under RC (control value=2)
		// turn servos and light off.

		// add the hp in question to RC control, turn the light on.
		case 1:
			hp1_control=2;		// servo flag to RC control
			servo_set(FRONT_HP_SERVO_H,1500);	// horizontal servo to middle
			if(feedbackmessageon) serial_puts_p(strHP1RC);
			break;
		case 2:
			hp2_control=2;
			servo_set(REAR_HP_SERVO_H,1500);
			if(feedbackmessageon) serial_puts_p(strHP2RC);
			break;
		case 3:
			hp3_control=2;
			servo_set(TOP_HP_SERVO_H,1500);
			if(feedbackmessageon) serial_puts_p(strHP3RC);
			break;
		case 0: // all holos under RC
		default:
			hp1_control=hp2_control=hp3_control=2;
			servo_set(FRONT_HP_SERVO_H,1500);
			servo_set(REAR_HP_SERVO_H,1500);
			servo_set(TOP_HP_SERVO_H,1500);
			if(feedbackmessageon) serial_puts_p(strHPAllRC);
			break;
	}
}

const char strHP1TE[] PROGMEM="(HP1 Test) \r\n";
const char strHP2TE[] PROGMEM="(HP2 Test) \r\n";
const char strHP3TE[] PROGMEM="(HP3 Test) \r\n";
const char strHPAllTE[] PROGMEM="(All HPs Test) \r\n";
// test mode
void test_command(uint8_t value)
{
	switch(value)
	{

		case 1:
			if(feedbackmessageon) serial_puts_p(strHP1TE);
			hp1_control=3;
			break;
		case 2:
			if(feedbackmessageon) serial_puts_p(strHP2TE);
			hp2_control=3;
			break;
		case 3:
			if(feedbackmessageon) serial_puts_p(strHP3TE);
			hp3_control=3;
			break;
		case 0:	// all holos into random movement mode
		default:
			if(feedbackmessageon) serial_puts_p(strHPAllTE);
			hp1_control=hp2_control=hp3_control=3;
			break;
	}
}

// Magic panel on command
// 0= off, 99= on, 1 to 98 : on for 1 to 98 seconds
const char strMagicOn[] PROGMEM="(Magic On) \r\n";
void magic_on_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strMagicOn);
	switch(value)
	{
		case 0:
			magic_control=0;
			on_timer_magic=0;
			break;
		case 99:
			magic_control=1;
			on_timer_magic=0;
			break;
		default:
			magic_control=2;
			on_timer_magic=value*100;
			break;
	}
}

// Magic panel flicker command
// 0= flicker off, 1 to 99 : on for 1 to 99 seconds
const char strMagicFlicker[] PROGMEM="(Magic Flicker) \r\n";
void magic_flicker_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strMagicFlicker);
	switch(value)
	{
		case 0:
			magic_control=0;
			on_timer_magic=0;
			break;
		default:
			magic_control=3;
			on_timer_magic=value*100;
			break;
	}
}

const char strHP1Flicker[] PROGMEM="(HP1 Flicker) \r\n";
void holo1_flicker_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP1Flicker);
	switch(value)
	{
		case 0:
			hp1_light_control=0;
			on_timer_HP1=0;
			break;
		default:
			hp1_light_control=3;
			on_timer_HP1=value*100;
			break;
	}
}

const char strHP2Flicker[] PROGMEM="(HP2 Flicker) \r\n";
void holo2_flicker_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP2Flicker);
	switch(value)
	{
		case 0:
			hp2_light_control=0;
			on_timer_HP2=0;
			break;
		default:
			hp2_light_control=3;
			on_timer_HP2=value*100;
			break;
	}
}

const char strHP3Flicker[] PROGMEM="(HP3 Flicker) \r\n";
void holo3_flicker_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP3Flicker);
	switch(value)
	{
		case 0:
			hp3_light_control=0;
			on_timer_HP3=0;
			break;
		default:
			hp3_light_control=3;
			on_timer_HP3=value*100;
			break;
	}
}

const char strAllHPFlicker[] PROGMEM="(All HP Flicker) \r\n";
void all_holo_flicker_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strAllHPFlicker);
	switch(value)
	{
		case 0:
			hp1_light_control=hp2_light_control=hp3_light_control=0;
			on_timer_HP1=on_timer_HP2=on_timer_HP3=0;
			break;
		default:
			hp1_light_control=hp2_light_control=hp3_light_control=3;
			on_timer_HP1=on_timer_HP2=on_timer_HP3=value*100;
			break;
	}
}

const char strHP1Flash[] PROGMEM="(HP1 Flash) \r\n";
void holo1_flash_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP1Flash);
	switch(value)
	{
		case 0:
			hp1_light_control=0;
			on_timer_HP1=0;
			break;
		case 99:
			hp1_light_control=1;
			on_timer_HP1=0;
			break;
		default:
			hp1_light_control=2;
			on_timer_HP1=value*100;
			break;
	}
}

const char strHP2Flash[] PROGMEM="(HP2 Flash) \r\n";
void holo2_flash_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP2Flash);
	switch(value)
	{
		case 0:
			hp2_light_control=0;
			on_timer_HP2=0;
			break;
		case 99:
			hp2_light_control=1;
			on_timer_HP2=0;
			break;
		default:
			hp2_light_control=2;
			on_timer_HP2=value*100;
			break;
	}
}

const char strHP3Flash[] PROGMEM="(HP3 Flash) \r\n";
void holo3_flash_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strHP3Flash);
	switch(value)
	{
		case 0:
			hp3_light_control=0;
			on_timer_HP3=0;
			break;
		case 99:
			hp3_light_control=1;
			on_timer_HP3=0;
			break;
		default:
			hp3_light_control=2;
			on_timer_HP3=value*100;
			break;
	}
}

const char strAllHPFlash[] PROGMEM="(All HP Flash) \r\n";
void all_holo_flash_command(uint8_t value)
{
	if(feedbackmessageon) serial_puts_p(strAllHPFlash);
	switch(value)
	{
		case 0:
			hp1_light_control=hp2_light_control=hp3_light_control=0;
			on_timer_HP1=on_timer_HP2=on_timer_HP3=0;
			break;
		case 99:
			hp1_light_control=hp2_light_control=hp3_light_control=1;
			on_timer_HP1=on_timer_HP2=on_timer_HP3=0;
			break;
		default:
			hp1_light_control=hp2_light_control=hp3_light_control=2;
			on_timer_HP1=on_timer_HP2=on_timer_HP3=value*100;
			break;
	}
}


void init_jedi()
{
	lightsuart_puts("0T1\r");	// abort test routine, reset all to normal
	_delay_ms(20);
#ifdef _DIGITALJEDI_
	/**** initialize JEDI display for digital output on HPs and PSI ******/
	// I connected Mike Velchecks rear PSI to the JEDI, which requires output to be turned to digital
	// My holo lights are the older version and also require HPs to be set to digital
	lightsuart_puts("6P91\r");	// change front holo (6) parameter 9 (P9) to digital (1)
	_delay_ms(20);
	lightsuart_puts("5P91\r");   // change rear PSI (5) parameter 9 (P9) to digital (1)
	_delay_ms(20);
#endif
}

/*****************************************
 *  MarcDuino v2 REON holo support
 */

void holo1ON()
{
	if(holo1state!=1)
	{
		holo1state=1;
		HP1_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_ON;
			i2c_send_data(REON1, &command, 1, TRUE);
		#endif
	}
}

void holo2ON()
{
	if(holo2state!=1)
	{
		holo2state=1;
		HP2_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_ON;
			i2c_send_data(REON2, &command, 1, TRUE);
		#endif
	}
}

void holo3ON()
{
	if(holo3state!=1)
	{
		holo3state=1;
		HP3_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_ON;
			i2c_send_data(REON3, &command, 1, TRUE);
		#endif
	}
}


void holo1WHITE()
{
	if(holo1state!=1)
	{
		holo1state=1;
		HP1_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_WHITE;
			i2c_send_data(REON1, &command, 1, TRUE);
		#endif
	}
}

void holo2WHITE()
{
	if(holo2state!=1)
	{
		holo2state=1;
		HP2_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_WHITE;
			i2c_send_data(REON2, &command, 1, TRUE);
		#endif
	}
}

void holo3WHITE()
{
	if(holo3state!=1)
	{
		holo3state=1;
		HP3_ON;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_WHITE;
			i2c_send_data(REON3, &command, 1, TRUE);
		#endif
	}
}

void holo1OFF()
{
	if(holo1state!=0)
	{
		holo1state=0;
		HP1_OFF;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_OFF;
			i2c_send_data(REON1, &command, 1, TRUE);
		#endif
	}
}

void holo2OFF()
{
	if(holo2state!=0)
	{
		holo2state=0;
		HP2_OFF;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_OFF;
			i2c_send_data(REON2, &command, 1, TRUE);
		#endif
	}
}

void holo3OFF()
{
	if(holo3state!=0)
	{
		holo3state=0;
		HP3_OFF;
		#ifdef _MARCDUINOV2_
			uint8_t command=REON_OFF;
			i2c_send_data(REON3, &command, 1, TRUE);
		#endif
	}
}



/*****************************************
 * MarcDuino v1 versus v2 suart and suart2 change
 * In the slave, suarts were swapped compared to Master (in error)
 * This is corrected in v2:
 *
 * Lights output is
 * 	suart which is PC0 on v1
 * 	suart2 which is PC1 on v2
 * Next slave outpur is
 * 	suart2 which is PC4 on v1
 * 	suart which is PC0 on v2
 * The suart.c library handles the PC4/PC1 pin change
 * These functions handle the suart/suart2 inversion
 *******************************************/

void slavesuart_init(uint16_t baudrate)
{
#ifdef _MARCDUINOV2_
	suart_init(baudrate); 		// suart (PC0) for MarcDuino v2
#else
	suart2_init(baudrate);		// suart2 (PC4) for MarcDuino v1
#endif
}
void slavesuart_putc(unsigned char c)
{
#ifdef _MARCDUINOV2_
	suart_putc(c); 		// suart (PC0) for MarcDuino v2
#else
	suart2_putc(c);		// suart2 (PC4) for MarcDuino v1
#endif
}

void slavesuart_puts(char* s)
{
#ifdef _MARCDUINOV2_
	suart_puts(s); 		// suart (PC0) for MarcDuino v2
#else
	suart2_puts(s);		// suart2 (PC4) for MarcDuino v1
#endif
}

void slavesuart_puts_p(const char *progmem_s )
{
#ifdef _MARCDUINOV2_
	suart_puts_p(progmem_s); 		// suart (PC0) for MarcDuino v2
#else
	suart2_puts_p(progmem_s);		// suart2 (PC4) for MarcDuino v1
#endif
}

void lightsuart_init(uint16_t baudrate)
{
#ifdef _MARCDUINOV2_
	suart2_init(baudrate); 		// suart2 (PC1) for MarcDuino v2
#else
	suart_init(baudrate);		// suart (PC0) for MarcDuino v1
#endif
}

void lightsuart_putc(unsigned char c)
{
#ifdef _MARCDUINOV2_
	suart2_putc(c); 		// suart2 (PC1) for MarcDuino v2
#else
	suart_putc(c);		// suart (PC0) for MarcDuino v1
#endif
}

void lightsuart_puts(char* s)
{
#ifdef _MARCDUINOV2_
	suart2_puts(s); 		// suart2 (PC1) for MarcDuino v2
#else
	suart_puts(s);		// suart (PC0) for MarcDuino v1
#endif
}
void lightsuart_puts_p(const char *progmem_s )
{
#ifdef _MARCDUINOV2_
	suart2_puts_p(progmem_s); 		// suart2 (PC1) for MarcDuino v2
#else
	suart_puts_p(progmem_s);		// suart (PC0) for MarcDuino v1
#endif
}
