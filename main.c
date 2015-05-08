/*
    example BiblioPixel protocol WS281x controller using PIC16F1454 microcontroller

    Copyright (C) 2015 Peter Lawrence

    based on top of M-Stack USB driver stack by Alan Ott, Signal 11 Software

    Permission is hereby granted, free of charge, to any person obtaining a 
    copy of this software and associated documentation files (the "Software"), 
    to deal in the Software without restriction, including without limitation 
    the rights to use, copy, modify, merge, publish, distribute, sublicense, 
    and/or sell copies of the Software, and to permit persons to whom the 
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in 
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
    DEALINGS IN THE SOFTWARE.
*/

#include "usb.h"
#include <xc.h>
#include <string.h>
#include "usb_config.h"
#include "usb_ch9.h"
#include "usb_cdc.h"

#define LED_COUNT 128

struct ws_led_struct
{
	uint8_t g, r, b; /* the order is critical: the WS281x expects green, red, then blue */
};

/* array storing state of LEDs */
static struct ws_led_struct leds[LED_COUNT];

/* pointer used by ISR to incrementally read leds[] array */
static uint8_t *isr_ptr;

int main(void)
{
	uint8_t *in_buf;
	const uint8_t *out_buf;
	uint8_t fromPC_count, toPC_count, index;
	uint8_t command, color_index, current_byte, previous_byte, led_index;
	uint16_t data_remaining;
	uint8_t *wptr;

	/* SPI (WS281x) init */
	SSP1STAT = 0x40;
	SSP1CON1 = 0x20;
	ANSELCbits.ANSC2 = 0;
	TRISCbits.TRISC2 = 0;

	/* enable everything but global interrupts in preparation for SPI interrupt */
	PIR1bits.SSP1IF = 0;
	PIE1bits.SSP1IE = 1;
	INTCONbits.PEIE = 1;

	usb_init();

	in_buf = usb_get_in_buffer(2);

	/* there is nothing yet to send to the PC */
	toPC_count = 0;
	/* we are not in the middle of a message */
	data_remaining = 0;

	for (;;)
	{
		usb_service();

		/* if USB isn't configured, there is no point in proceeding further */
		if (!usb_is_configured())
			continue;

		/* proceed further only if the PC can accept more data */
		if (usb_in_endpoint_halted(2) || usb_in_endpoint_busy(2))
			continue;

		/*
		if we've reached here, the USB stack can accept more; 
		if we have data to send to the PC, we hand it over
		*/
		if (toPC_count > 0)
		{
			usb_send_in_buffer(2, toPC_count);
			toPC_count = 0;
		}

		/* if we pass this test, we are committed to make the usb_arm_out_endpoint() call */
		if (!usb_out_endpoint_has_data(2))
			continue;

		/* ask USB stack for more data from the PC */
		fromPC_count = usb_get_out_buffer(2, &out_buf);

		for (index = 0; index <fromPC_count; index++)
		{
			previous_byte = current_byte;
			current_byte = out_buf[index];

			if (data_remaining)
			{
				/* we are one byte closer to reaching the end of the message */
				data_remaining--;

				if (2 == command)
				{
					if (wptr)
					{
						/*
						BiblioPixel writes R,G,B ... but we need G,R,B
						*/
						switch (color_index)
						{
							case 0: /* red */
								*(wptr + 1) = current_byte;
								break;
							case 1: /* green */
								*(wptr + 0) = current_byte;
								break;
							case 2: /* blue */
								*(wptr + 2) = current_byte;
								wptr += 3;
								break;
						}
					}

					if (0 == data_remaining)
					{
						/*
						we've now received an entire array of data; to send to the WS281x,
						we just fire and forget using the interrupt service routine
						*/
						isr_ptr = (uint8_t *)leds;
						INTCONbits.GIE = 1;
						SSP1BUF = 0x00;
					}

					/* bookkeep which color we are expecting (0 = R, 1 = G, 2 = B) */
					color_index++;
					if (3 == color_index)
					{
						color_index = 0;

						/* idiot-proof ourselves in case the user tries to write beyond the array */
						led_index++;
						if (LED_COUNT == led_index)
							wptr = NULL;
					}
				}

				/* hooray!  we've reached the end of the packet; report back success */
				if (0 == data_remaining)
					in_buf[toPC_count++] = 0xFF;
			}
			else
			{
				switch (index)
				{
					case 0:
						/* rewind to the beginning */
						command = current_byte;
						color_index = 0; led_index = 0;
						wptr = (uint8_t *)leds;
						break;
					case 1:
						/*
						we can't write to data_remaining here, as the above logic 
						depends on data_remaining being zero during a header
						*/
						break;
					case 2:
						/* now we know the length of the message */
						data_remaining = (uint16_t)current_byte << 8;
						data_remaining |= previous_byte;
						break;
				}
			}
		}

		/* indicate to USB stack that we can receive more data */
		usb_arm_out_endpoint(2);
	}
}

void interrupt isr()
{
	static uint8_t bit_position;
	static uint16_t byte_count;
	static uint8_t current_byte;

	/* check if SSP1IF interrupt has fired... */
	if (PIR1bits.SSP1IF)
	{
		/* ... and acknowledge it by clearing SSP1IF flag */
		PIR1bits.SSP1IF = 0;

		if (0 == bit_position)
		{
			/* 
			if bit_position is zero, we've exhausted all the bits in the previous byte 
			and need to load current_byte with the next byte
			*/
			if ((LED_COUNT * sizeof(struct ws_led_struct)) == byte_count)
			{
				/*
				we've reached the end of the LED data, and 
				the interrupt routine's work is done;
				so, we disable the interrupt and bail
				*/
				INTCONbits.GIE = 0;
				byte_count = 0;
				return;
			}

			/* load next byte into current_byte */
			current_byte = *isr_ptr++;
			byte_count++;
		}

		/* WS281X expects long pulse for '1' and short pulse for '0' */
		SSP1BUF = (current_byte & 0x80) ? 0xFF : 0xF0;

		/* preemptively shift next bit into position and update bit_position */
		current_byte <<= 1;
		bit_position = (bit_position + 1) & 0x7;		
	}
}
