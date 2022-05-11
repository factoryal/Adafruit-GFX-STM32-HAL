/*********************************************************************
This is a library for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

These displays use SPI to communicate, 4 or 5 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

/*
 *  Modified by Neal Horman 7/14/2012 for use in mbed
 */

#ifndef _ADAFRUIT_SSD1306_H_
#define _ADAFRUIT_SSD1306_H_

// #include "mbed.h"
#include "main.h"
#include "Adafruit_GFX.h"

#include <vector>
#include <algorithm>

// A DigitalOut sub-class that provides a constructed default state
// class DigitalOut2 : public DigitalOut
// {
// public:
// 	DigitalOut2(PinName pin, bool active = false) : DigitalOut(pin) { write(active); };
// 	DigitalOut2& operator= (int value) { write(value); return *this; };
// 	DigitalOut2& operator= (DigitalOut2& rhs) { write(rhs.read()); return *this; };
// 	operator int() { return read(); };
// };

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

/** The pure base class for the SSD1306 display driver.
 *
 * You should derive from this for a new transport interface type,
 * such as the SPI and I2C drivers.
 */
class Adafruit_SSD1306 : public Adafruit_GFX
{
public:
	// Adafruit_SSD1306(PinName RST, uint8_t rawHeight = 32, uint8_t rawWidth = 128)
	// 	: Adafruit_GFX(rawWidth,rawHeight)
	// 	, rst(RST,false)
	// {
	// 	buffer.resize(rawHeight * rawWidth / 8);
	// };
	Adafruit_SSD1306(GPIO_Typedef* RST_port, uint16_t RST_pin, uint8_t rawHeight = 32, uint8_t rawWidth = 128)
		: Adafruit_GFX(rawWidth,rawHeight)
	{
		rst_port = RST_port;
		rst_pin = RST_pin;
		HAL_GPIO_WritePin(rst_port, rst_pin, GPIO_PIN_RESET);
		buffer.resize(rawHeight * rawWidth / 8);
	};

	void begin(uint8_t switchvcc = SSD1306_SWITCHCAPVCC);
	
	// These must be implemented in the derived transport driver
	virtual void command(uint8_t c) = 0;
	virtual void data(uint8_t c) = 0;
	virtual void drawPixel(int16_t x, int16_t y, uint16_t color);

	/// Clear the display buffer    
	void clearDisplay(void);
	virtual void invertDisplay(bool i);

	/// Cause the display to be updated with the buffer content.
	void display();
	/// Fill the buffer with the AdaFruit splash screen.
	virtual void splash();
    
protected:
	virtual void sendDisplayBuffer() = 0;
	// DigitalOut2 rst;
	GPIO_Typedef* rst_port;
	uint16_t rst_pin;

	// the memory buffer for the LCD
	std::vector<uint8_t> buffer;
};


/** This is the SPI SSD1306 display driver transport class
 *
 */
class Adafruit_SSD1306_Spi : public Adafruit_SSD1306
{
public:
	/** Create a SSD1306 SPI transport display driver instance with the specified DC, RST, and CS pins, as well as the display dimentions
	 *
	 * Required parameters
	 * @param spi - a reference to an initialized SPI object
	 * @param DC (Data/Command) pin name
	 * @param RST (Reset) pin name
	 * @param CS (Chip Select) pin name
	 *
	 * Optional parameters
	 * @param rawHeight - the vertical number of pixels for the display, defaults to 32
	 * @param rawWidth - the horizonal number of pixels for the display, defaults to 128
	 */
	// Adafruit_SSD1306_Spi(SPI &spi, PinName DC, PinName RST, PinName CS, uint8_t rawHieght = 32, uint8_t rawWidth = 128)
	//     : Adafruit_SSD1306(RST, rawHieght, rawWidth)
	//     , cs(CS,true)
	//     , dc(DC,false)
	//     , mspi(spi)
	//     {
	// 	    begin();
	// 	    splash();
	// 	    display();
	//     };
	Adafruit_SSD1306_Spi(SPI_HandleTypeDef* hspi, 
		GPIO_TypeDef* DC_port, uint16_t DC_pin, 
		GPIO_TypeDef* RST_port, uint16_t RST_pin, 
		GPIO_TypeDef* CS_port, uint16_t CS_pin, 
		uint8_t rawHieght = 32, uint8_t rawWidth = 128)
	    : Adafruit_SSD1306(RST_port, RST_pin, rawHieght, rawWidth)
	    , cs(CS,true)
	    , dc(DC,false)
	    , mspi(spi)
	    {
			_hspi = hspi;
			dc_port = DC_port;
			dc_pin = DC_pin;
			rst_port = RST_port;
			rst_pin = RST_pin;
			cs_port = CS_port;
			cs_pin = CS_pin;

			HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(dc_port, dc_pin, GPIO_PIN_RESET);

		    begin();
		    splash();
		    display();
	    };

	virtual void command(uint8_t c)
	{
	    // cs = 1;
	    // dc = 0;
	    // cs = 0;
	    // mspi.write(c);
	    // cs = 1;
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dc_port, dc_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(_hspi, &c, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
	};

	virtual void data(uint8_t c)
	{
	    // cs = 1;
	    // dc = 1;
	    // cs = 0;
	    // mspi.write(c);
	    // cs = 1;
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dc_port, dc_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(_hspi, &c, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
	};

protected:
	virtual void sendDisplayBuffer()
	{
		// cs = 1;
		// dc = 1;
		// cs = 0;
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dc_port, dc_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

		// for(uint16_t i=0, q=buffer.size(); i<q; i++)
		// 	mspi.write(buffer[i]);

		// if(height() == 32)
		// {
		// 	for(uint16_t i=0, q=buffer.size(); i<q; i++)
		// 		mspi.write(0);
		// }
		HAL_SPI_Transmit(_hspi, buffer, buffer.size(), HAL_MAX_DELAY);
		if(height() == 32) {
			uint8_t* zeros = new uint8_t[buffer.size()];
			HAL_SPI_Transmit(_hspi, zeros, buffer.size(), HAL_MAX_DELAY);
		}

		// cs = 1;
		HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
	};

	// DigitalOut2 cs, dc;
	// SPI &mspi;
	GPIO_TypeDef *cs_port, *dc_port;
	uint16_t cs_pin, dc_pin;
	SPI_HandleTypeDef* _hspi;
};

/** This is the I2C SSD1306 display driver transport class
 *
 */
class Adafruit_SSD1306_I2c : public Adafruit_SSD1306
{
public:
	#define SSD_I2C_ADDRESS     0x78
	/** Create a SSD1306 I2C transport display driver instance with the specified RST pin name, the I2C address, as well as the display dimensions
	 *
	 * Required parameters
	 * @param i2c - A reference to an initialized I2C object
	 * @param RST - The Reset pin name
	 *
	 * Optional parameters
	 * @param i2cAddress - The i2c address of the display
	 * @param rawHeight - The vertical number of pixels for the display, defaults to 32
	 * @param rawWidth - The horizonal number of pixels for the display, defaults to 128
	 */
	// Adafruit_SSD1306_I2c(I2C &i2c, PinName RST, uint8_t i2cAddress = SSD_I2C_ADDRESS, uint8_t rawHeight = 32, uint8_t rawWidth = 128)
	//     : Adafruit_SSD1306(RST, rawHeight, rawWidth)
	//     , mi2c(i2c)
	//     , mi2cAddress(i2cAddress)
	//     {
	// 	    begin();
	// 	    splash();
	// 	    display();
	//     };
	Adafruit_SSD1306_I2c(I2C_HandleTypeDef* hi2c, GPIO_TypeDef* RST_port, uint16_t RST_pin, uint8_t i2cAddress = SSD_I2C_ADDRESS, uint8_t rawHeight = 32, uint8_t rawWidth = 128)
	    : Adafruit_SSD1306(RST_port, RST_pin, rawHeight, rawWidth)
	    , mi2cAddress(i2cAddress)
	    {
			_hi2c = hi2c;
		    begin();
		    splash();
		    display();
	    };

	virtual void command(uint8_t c)
	{
		char buff[2];
		buff[0] = 0; // Command Mode
		buff[1] = c;
		// mi2c.write(mi2cAddress, buff, sizeof(buff));
		HAL_I2C_Master_Transmit(_hi2c, mi2cAddress << 1, buff, sizeof(buff), HAL_MAX_DELAY);
	}

	virtual void data(uint8_t c)
	{
		char buff[2];
		buff[0] = 0x40; // Data Mode
		buff[1] = c;
		// mi2c.write(mi2cAddress, buff, sizeof(buff));
		HAL_I2C_Master_Transmit(_hi2c, mi2cAddress << 1, buff, sizeof(buff), HAL_MAX_DELAY);
	};

protected:
	virtual void sendDisplayBuffer()
	{
		char buff[17];
		buff[0] = 0x40; // Data Mode

		// send display buffer in 16 byte chunks
		for(uint16_t i=0, q=buffer.size(); i<q; i+=16 ) 
		{	uint8_t x ;

			// TODO - this will segfault if buffer.size() % 16 != 0
			for(x=1; x<sizeof(buff); x++) 
				buff[x] = buffer[i+x-1];
			// mi2c.write(mi2cAddress, buff, sizeof(buff));
			HAL_I2C_Master_Transmit(_hi2c, mi2cAddress << 1, buff, sizeof(buff), HAL_MAX_DELAY);
		}
	};

	// I2C &mi2c;
	// uint8_t mi2cAddress;

	I2C_HandleTypeDef* _hi2c;
};

#endif