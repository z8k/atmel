/**
 * A simple 2-channel min-max thermometer demonstrating use of:
 * 
 * - The LCD display
 * - PWM control (LCD backlight brightness)
 * - A/D converter (an LM35 temperature sensor)
 * - Dallas/Maxim 1-Wire bus (a DS1820 temperature sensor)
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>

#include "liblcd/lcd.h"
#include "1wire/polled/OWIPolled.h"
#include "1wire/polled/OWIHighLevelFunctions.h"
#include "1wire/polled/OWIBitFunctions.h"
#include "1wire/common_files/OWIcrc.h"


/**
 * User-defined characters for the LCD.
 * The first user-defined char is left blank because we can't
 * embed a NULL \000 in a string. The second is a degrees symbol.
 */
static const PROGMEM unsigned char degreesChar[] =
{
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x02, 0x05, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00
};



// 1-wire stuff. This is copied from the example in ../lib/1wire/polled.
#define DS1820_FAMILY_ID			0x10
#define DS1820_START_CONVERSION	0x44
#define DS1820_READ_SCRATCHPAD	0xbe
#define DS1820_ERROR				-1000   // Return code. Outside temperature range.

#define SEARCH_SUCCESSFUL		0x00
#define SEARCH_CRC_ERROR			0x01

#define FALSE	0
#define TRUE		1

#define MAX_DEVICES	8       //!< Max number of devices to search for.
#define BUSES		(OWI_PIN_3) //!< Buses to search.


/**
 *  The OWI_device data type holds information about what bus each device
 *  is connected to, and its 64 bit identifier.
 */
typedef struct OWI_device {
	unsigned char bus;      //!< A bitmask of the bus the device is connected to.
	unsigned char id[8];    //!< The 64 bit identifier.
} OWI_device;


unsigned char SearchBuses(OWI_device * devices, unsigned char len, unsigned char buses);
OWI_device * FindFamily(unsigned char familyID, OWI_device * devices, unsigned char size);
signed int DS1820_ReadTemperature(unsigned char bus, unsigned char * id);

OWI_device devices[MAX_DEVICES];

/**
 * Finds all devices (upper bounded by MAX_DEVICES) on the buses defined by BUSES.
 * Returns the first DS1820 temperature sensor it finds.
 */
OWI_device * findDS1820(void) {
	OWI_Init(BUSES);
	
	// Do the bus search until all ids are read without crc error.
	while(SearchBuses(devices, MAX_DEVICES, BUSES) != SEARCH_SUCCESSFUL)
		;
	
	return FindFamily(DS1820_FAMILY_ID, devices, MAX_DEVICES);
}

/*! \brief  Perform a 1-Wire search
 * 
 *  This function shows how the OWI_SearchRom function can be used to
 *  discover all slaves on the bus. It will also CRC check the 64 bit
 *  identifiers.
 * 
 *  \param  devices Pointer to an array of type OWI_device. The discovered
 *                  devices will be placed from the beginning of this array.
 * 
 *  \param  len     The length of the device array. (Max. number of elements).
 * 
 *  \param  buses   Bitmask of the buses to perform search on.
 * 
 *  \retval SEARCH_SUCCESSFUL   Search completed successfully.
 *  \retval SEARCH_CRC_ERROR    A CRC error occured. Probably because of noise
 *                              during transmission.
 */
unsigned char SearchBuses(OWI_device * devices, unsigned char len, unsigned char buses) {
	unsigned char i, j;
	unsigned char presence;
	unsigned char * newID;
	unsigned char * currentID;
	unsigned char currentBus;
	unsigned char lastDeviation;
	unsigned char numDevices;
	
	// Initialize all addresses as zero, on bus 0 (does not exist).
	// Do a search on the bus to discover all addresses.
	for (i = 0; i < len; i++) {
		devices[i].bus = 0x00;
		for (j = 0; j < 8; j++) {
			devices[i].id[j] = 0x00;
		}
	}
	
	// Find the buses with slave devices.
	presence = OWI_DetectPresence(BUSES);
	
	numDevices = 0;
	newID = devices[0].id;
	
	// Go through all buses with slave devices.
	for (currentBus = 0x01; currentBus; currentBus <<= 1) {
		lastDeviation = 0;
		currentID = newID;
		if (currentBus & presence) { // Devices available on this bus.
			// Do slave search on each bus, and place identifiers and corresponding
			// bus "addresses" in the array.
			do {
				memcpy(newID, currentID, 8);
				OWI_DetectPresence(currentBus);
				lastDeviation = OWI_SearchRom(newID, lastDeviation, currentBus);
				currentID = newID;
				devices[numDevices].bus = currentBus;
				numDevices++;
				newID=devices[numDevices].id;
			}  while (lastDeviation != OWI_ROM_SEARCH_FINISHED);
		}
	}
	
	// Go through all the devices and do CRC check.
	for (i = 0; i < numDevices; i++) {
		// If any id has a crc error, return error.
		if(OWI_CheckRomCRC(devices[i].id) != OWI_CRC_OK) {
			return SEARCH_CRC_ERROR;
		}
	}
	// Else, return Successful.
	return SEARCH_SUCCESSFUL;
}

/*! \brief  Find the first device of a family based on the family id
 * 
 *  This function returns a pointer to a device in the device array
 *  that matches the specified family.
 * 
 *  \param  familyID    The 8 bit family ID to search for.
 * 
 *  \param  devices     An array of devices to search through.
 * 
 *  \param  size        The size of the array 'devices'
 * 
 *  \return A pointer to a device of the family.
 *  \retval NULL    if no device of the family was found.
 */
OWI_device * FindFamily(unsigned char familyID, OWI_device * devices, unsigned char size) {
	unsigned char i = 0;
	
	// Search through the array.
	while (i < size) {
		// Return the pointer if there is a family id match.
		if ((*devices).id[0] == familyID) {
			return devices;
		}
		devices++;
		i++;
	}
	// Else, return NULL.
	return NULL;
}


/*! \brief  Read the temperature from a DS1820 temperature sensor.
 * 
 *  This function will start a conversion and read back the temperature
 *  from a DS1820 temperature sensor.
 * 
 *  \param  bus A bitmask of the bus where the DS1820 is located.
 * 
 *  \param  id  The 64 bit identifier of the DS1820.
 * 
 *  \return The 16 bit signed temperature read from the DS1820.
 * The DS1820 must have Vdd pin connected for this code to work.
 */
signed int DS1820_ReadTemperature(unsigned char bus, unsigned char * id) {
	signed int temperature;
	
	// Reset, presence.
	if (!OWI_DetectPresence(bus)) {
		return DS1820_ERROR; // Error
	}
	// Match the id found earlier.
	OWI_MatchRom(id, bus);
	// Send start conversion command.
	OWI_SendByte(DS1820_START_CONVERSION, bus);
	// Wait until conversion is finished.
	// Bus line is held low until conversion is finished.
	while (!OWI_ReadBit(bus))
		;
	
	// Reset, presence.
	if (!OWI_DetectPresence(bus)) {
		return -1000; // Error
	}
	// Match id again.
	OWI_MatchRom(id, bus);
	// Send READ SCRATCHPAD command.
	OWI_SendByte(DS1820_READ_SCRATCHPAD, bus);
	// Read only two first bytes (temperature low, temperature high)
	// and place them in the 16 bit temperature variable.
	temperature = OWI_ReceiveByte(bus);
	temperature |= (OWI_ReceiveByte(bus) << 8);
	
	return temperature;
}


/**
 * Sets the pwm output for the LCD backlight on pin PB1 (OC1A).
 */
void setpwm(uint8_t pwm)
{
	OCR1A = pwm;
	
	// set non-inverting mode
	TCCR1A |= (1 << COM1A1);
	
	// set fast PWM Mode
	TCCR1A |= (1 << WGM10);
	
	// set prescaler to 8 and start PWM
	TCCR1B |= (1 << CS11);
}

/** Slowly increases the pwm output from minimum to maximum. */
void fadeUp(void) {
	uint8_t brightness;
	DDRB |= (1<<PB1); // Pin becomes an output
	for (brightness = 0; brightness < 255; ++brightness) {
		setpwm(brightness);
		_delay_ms(5);
	}
}

/** Slowly decreases the pwm output from maximum to minimum. */
void fadeDown(void) {
	uint8_t brightness;
	for (brightness = 255; brightness > 0; --brightness) {
		setpwm(brightness);
		_delay_ms(2);
	}
	DDRB &=~ (1 << PB1); // Pin becomes an input, so the backlight switches right off
}

/** Initialises the ADC. */
void initADC(void) {
	// Internal 1.1V voltage reference, external capacitor on AREF pin.
	ADMUX |= (1<<REFS1)|(1<<REFS0);
	// Set prescaler to 128 and enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}

/** Read from ADC. */
uint16_t readADC(uint8_t ADCchannel) {
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
	//single conversion mode
	ADCSRA |= (1<<ADSC);
	// wait until ADC conversion is complete
	while(ADCSRA & (1<<ADSC))
		;
	return ADC;
}

/** The main loop. Sets up the 1-wire bus, then loops forever reading and displaying temperatures. */
int main(void) {
	char buffer[32];
	uint16_t adc;
	int8_t onTime, hiloTime;
	bool backlight, lo;
	double adcv, tempI, tempO, minI, maxI, minO, maxO;
	unsigned char i;
	OWI_device * ds1820;
	
	backlight = false;
	lo = true;
	onTime = -1;
	hiloTime = 0;
	minI = minO = 100.0;
	maxI = maxO = tempI = tempO = 0.0;
	initADC();
	
	DDRB &=~ (1 << PB2);	/* Pin PB2 input: Light the display backlight when the button is pressed. */
	PORTB |= (1 << PB2);	/* Pin PB2 pull-up enabled    */
	
	/* initialize display, cursor off */
	lcd_init(LCD_DISP_ON);
	
	lcd_command(_BV(LCD_CGRAM));  /* set CG RAM start address 0 */
	for(i = 0; i < 16; i++) {
		lcd_data(pgm_read_byte_near(&degreesChar[i]));
	}
	
	ds1820 = findDS1820();
	//OWImain();
	lcd_clrscr();
	for (;;) {
		adc = readADC(0);
		adcv = 1.1 / 1023 * adc;
		
		tempI = adcv * 100.0;
		if (tempI < minI) {
			minI = tempI;
		}
		if (tempI > maxI) {
			maxI = tempI;
		}
		if (ds1820 != NULL) {
			tempO = (double) DS1820_ReadTemperature((*ds1820).bus, (*ds1820).id) / 2.0;
		}
		if (tempO < minO) {
			minO = tempO;
		}
		if (tempO > maxO) {
			maxO = tempO;
		}
		lcd_gotoxy(0, 0);
		if (lo) {
			sprintf(buffer, "I:%4.1f\001C lo:%4.1f\n", tempI, minI);
			lcd_puts(buffer);
			sprintf(buffer, "O:%4.1f\001C lo:%4.1f ", tempO, minO);
			lcd_puts(buffer);
		} else {
			sprintf(buffer, "I:%4.1f\001C hi:%4.1f\n", tempI, maxI);
			lcd_puts(buffer);
			sprintf(buffer, "O:%4.1f\001C hi:%4.1f ", tempO, maxO);
			lcd_puts(buffer);
		}
		
		if (onTime == -1) {
			// Just started running. Fade the backlight up.
			backlight = true;
			onTime = 0;
			fadeUp();
		} else if ((PINB & (1<<PINB2)) == 0) {
			// Button PB2 pressed. Switch the backlight on, or prevent it turning off.
			onTime = 0;
			if (!backlight) {
				backlight = true;
				fadeUp();
			}
		} else if (backlight && ++onTime > 12) {
			// Button not press, backlight on time elapsed. Switch it off.
			onTime = 0;
			backlight = false;
			fadeDown();
		}
		
		// Alternate display between stored highest and lowest temps.
		if (++hiloTime > 8) {
			hiloTime = 0;
			lo = !lo;
		}
		_delay_ms(500);
	}
}
