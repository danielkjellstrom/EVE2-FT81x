#include "EVE2.h"

/*
  -Implement eve2_read() and eve2_write() with the specific MCU's SPI interface. This example is made with Arduino based code.
  -When initializing the EVE2 controller, the SPI clock should not be greater than 11MHz.
  -The data shall be exchanged with the most-significant bit first and the SPI should be configured in mode 0 (CPOL 0, CPHA 0)
*/
void spi_init_slow_mode(void)
{
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
}

/*
  Implement eve2_read() and eve2_write() with the specific MCU's SPI interface. This example is made with Arduino based code.
  Once the EVE2 controller has been initialized, the SPI clock shall be increase to no more than 30MHz.
*/
void spi_init_fast_mode(void)
{
  SPI.setClockDivider(SPI_CLOCK_DIV2);
}

uint8_t eve2_init(void)
{ 
  //Wake-up FT81x
  // 1) Lower PD#
  // 2) Hold for 20ms
  // 3) Raise PD#
  // 4) Hold for 20ms
  
  eve2_send_cmd(ACTIVE, 0);
  delay(5);
  
  long timeOut = millis(); //Arduino based API. Replace with appropiated API
  
  while((uint8_t)REG_ID_RST != eve2_rd8(REG_ID))
  {
    if((millis() - timeOut) > TIMEOUT_EVE2_DETECT)
    return 0;
    
    delay(1);
  }

  //Config eve2 controller with the display's timming parameters
  eve2_config();

  return 1;
}

uint8_t eve2_config(void)
{
  /* Disable display's clocking and backlight */
  eve2_wr8(REG_PCLK, 0); // Set PCLK to zero - don't clock the LCD until later
  eve2_set_backlight_intensity(0); // Turn off backlight 

  /* Configure timing registers */
  eve2_wr16(REG_HCYCLE, DISP_HCYCLE);
  eve2_wr16(REG_HOFFSET, DISP_HOFFSET);
  eve2_wr16(REG_HSYNC0, DISP_HSYNC0);
  eve2_wr16(REG_HSYNC1, DISP_HSYNC1);
  eve2_wr16(REG_VCYCLE, DISP_VCYCLE);
  eve2_wr16(REG_VOFFSET, DISP_VOFFSET);
  eve2_wr16(REG_VSYNC0, DISP_VSYNC0);
  eve2_wr16(REG_VSYNC1, DISP_VSYNC1);
  eve2_wr8(REG_SWIZZLE, DISP_SWIZZLE); 
  eve2_wr8(REG_PCLK_POL, DISP_PCLK_POL);
  eve2_wr8(REG_CSPREAD, DISP_CSPREAD);
  eve2_wr16(REG_HSIZE, DISP_HSIZE); 
  eve2_wr16(REG_VSIZE, DISP_VSIZE);

  /* Disable audio engine */
  eve2_wr8(REG_VOL_PB, 0); //Turn recorded audio volume down
  eve2_wr8(REG_VOL_SOUND, 0); //Turn synthesizer volume down
  eve2_wr16(REG_SOUND, 0x6000); //Set synthesizer to mute

  /* Populate first Display List. Clear display */
  eve2_wr32(RAM_DL+0,CLEAR_COLOR_RGB(0,0,0)); //Default color -> black
  eve2_wr32(RAM_DL+4,CLEAR(1,1,1)); 
  eve2_wr32(RAM_DL+8,D_DISPLAY());
  eve2_wr8(REG_DLSWAP, DLSWAP_FRAME); //Swap display list

  eve2_wr8(REG_GPIO_DIR, 0x83 | eve2_rd8(REG_GPIO_DIR)); 
  eve2_wr8(REG_GPIO, 0x80 | eve2_rd8(REG_GPIO)); //Enable display bit (DISP pin)
  
  eve2_wr8(REG_PCLK, 8); //Clock the display. Display is now active 
  
  for(int duty = 0; duty <= 128; duty++)
  {
    eve2_wr8(REG_PWM_DUTY, duty);    //Turn on backlight - ramp up slowly to full brightness
    delay(10);
  }

  delay(1);

  //Change to SPI fast mode
  spi_init_fast_mode();
}

uint8_t eve2_display_ftdi_logo_example(void)
{
  eve2_wr32(RAM_DL + 0, CLEAR(1, 1, 1)); //Clear screen to default settings (Color determined by CLEAR_COLOR_RGB())

  eve2_wr32(RAM_DL + 4, BEGIN(BITMAPS)); //Start drawing bitmaps 
  eve2_wr32(RAM_DL + 8, VERTEX2II(220, 110, 31, 'F')); //Ascii F in font 31 
  eve2_wr32(RAM_DL + 12, VERTEX2II(244, 110, 31, 'T')); //Ascii T 
  eve2_wr32(RAM_DL + 16, VERTEX2II(270, 110, 31, 'D')); //Ascii D 
  eve2_wr32(RAM_DL + 20, VERTEX2II(299, 110, 31, 'I')); //Ascii I 
  eve2_wr32(RAM_DL + 24, END()); //Let the controller know we are done drawing 

  eve2_wr32(RAM_DL + 28, COLOR_RGB(160, 22, 22)); //Change colour to red 
  eve2_wr32(RAM_DL + 32, POINT_SIZE(320)); //Set point size to 20 pixels in radius 
  eve2_wr32(RAM_DL + 36, BEGIN(POINTS)); //Start drawing a point
  eve2_wr32(RAM_DL + 40, VERTEX2II(192, 133, 0, 0)); //Red point 
  eve2_wr32(RAM_DL + 44, END()); //Let the controller know we are done drawing 

  eve2_wr32(RAM_DL + 48, D_DISPLAY()); //Display the image
  eve2_wr8(REG_DLSWAP,DLSWAP_FRAME); //Swap display list
}

void eve2_set_driving(uint8_t reg_drive)
{
  //Set driving of desired output
  eve2_wr8(REG_GPIO, reg_drive | eve2_rd8(REG_GPIO));
}

void eve2_set_backlight_intensity(uint8_t dutyCycle)
{
  uint8_t calculatedDutyCycle = 128; //Init to max duty cycle

  if(dutyCycle < 100)
  calculatedDutyCycle = round((128 * dutyCycle) / 100); //Calculate from %

  //Write duty cycle to display
  eve2_wr8(REG_PWM_DUTY, calculatedDutyCycle);
}

void eve2_set_backlight_frequency(uint16_t blFreq)
{
  if(blFreq < EVE2_BL_MIN_FREQ)
  blFreq = EVE2_BL_MIN_FREQ;

  else if(blFreq > EVE2_BL_MAX_FREQ)
  blFreq = EVE2_BL_MAX_FREQ;

  //Write frequency to display
  eve2_wr16(REG_PWM_HZ, blFreq);
}

void eve2_send_cmd(uint8_t cmd, uint8_t parameter)
{
  uint8_t cmdData[3];

  cmdData[0] = cmd;
  cmdData[1] = parameter;
  cmdData[2] = 0x00; //Fixed

  eve2_write(NULL, cmdData, 0, 3);
}

//---------------- Write functions ------------------//

void eve2_wr8(uint32_t addr, uint8_t dataByte)
{
  uint8_t addrToPass[3];

  addrToPass[0] = (addr >> 16) | EVE2_WRITE_MASK;
  addrToPass[1] = addr >> 8;
  addrToPass[2] = addr;

  eve2_write(addrToPass, &dataByte, 3, 1);
}

void eve2_wr16(uint32_t addr, uint16_t dataBytes)
{
  uint8_t addrToPass[3], dataToPass[2];

  addrToPass[0] = (addr >> 16) | EVE2_WRITE_MASK;
  addrToPass[1] = addr >> 8;
  addrToPass[2] = addr;
  
  dataToPass[0] = dataBytes >> 8;
  dataToPass[1] = dataBytes;

  eve2_write(addrToPass, dataToPass, 3, 2);
}

void eve2_wr32(uint32_t addr, uint32_t dataBytes)
{
  uint8_t addrToPass[3], dataToPass[4];

  addrToPass[0] = (addr >> 16) | EVE2_WRITE_MASK;
  addrToPass[1] = addr >> 8;
  addrToPass[2] = addr;
  
  dataToPass[0] = dataBytes >> 24;
  dataToPass[1] = dataBytes >> 16;
  dataToPass[2] = dataBytes >> 8;
  dataToPass[3] = dataBytes;

  eve2_write(addrToPass, dataToPass, 3, 4);
}

//-----------------------------------------------------//

//----------------- Read functions -------------------//

uint8_t eve2_rd8(uint32_t addr)
{  
  uint8_t addrToPass[3];

  addrToPass[0] = addr >> 16;
  addrToPass[1] = addr >> 8;
  addrToPass[2] = addr;

  static uint8_t* dataReceived = eve2_read(addrToPass, 3, 1); //Address, Address length, Data length

  return (uint8_t)*dataReceived;
}

uint16_t eve2_rd16(uint32_t addr)
{
  uint8_t addrToPass[3];

  addrToPass[0] = addr >> 16;
  addrToPass[1] = addr >> 8;
  addrToPass[2] = addr;

  static uint8_t* dataReceived = eve2_read(addrToPass, 3, 2); //Address, Address length, Data length

  return (uint16_t)dataReceived[0] | (uint16_t)dataReceived[1];
}

uint32_t eve2_rd32(uint32_t addr)
{
  uint8_t addrToPass[3];

  addrToPass[0] = addr >> 16;
  addrToPass[1] = addr >> 8;
  addrToPass[2] = addr;

  static uint8_t* dataReceived = eve2_read(addrToPass, 3, 3); //Address, Address length, Data length

  return (uint32_t)dataReceived[0] | (uint32_t)dataReceived[1] | (uint32_t)dataReceived[2];
}

//-----------------------------------------------------//

//------------ Hardware Specific Functions -------------//

/*
  Implement eve2_read() and eve2_write() with the specific MCU's SPI interface. This example is made with Arduino based code
*/

static uint8_t* eve2_read(uint8_t *addr, uint8_t addrLenght, uint8_t dataLenght) //dataLength in bytes
{
  uint8_t bytesRead[dataLenght];
  uint8_t i;

  digitalWrite(PIN_NUM_CS, LOW);
  
  //Send address
  for(i = 0; i < addrLenght; i++)
  SPI.transfer(addr[i]);

  //Dummy byte
  SPI.transfer(0);

  //Transfer 1 byte, read 1 byte
  for(i = 0; i < dataLenght; i++)
  bytesRead[i] = SPI.transfer(0); 

  //Cast to static type variable. Otherwise the compiler frees the allocated memory after return (This issue has been only encountered on ESP32)
  static uint8_t* returnData = bytesRead;

  digitalWrite(PIN_NUM_CS, HIGH);

  return  returnData;
}

void eve2_write(uint8_t* addr, uint8_t* dataBytes, uint8_t addrLenght, uint8_t dataLenght)
{
  uint8_t i;

  digitalWrite(PIN_NUM_CS, LOW);

  //Send address
  for(i = 0; i < addrLenght; i++)
  SPI.transfer(addr[i]);
  
  //Write data
  for(i = 0; i < dataLenght; i++)
  SPI.transfer(dataBytes[i]); 

  digitalWrite(PIN_NUM_CS, HIGH);
}

//-----------------------------------------------------//
