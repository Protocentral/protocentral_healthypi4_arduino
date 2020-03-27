//////////////////////////////////////////////////////////////////////////////////////////
//
//   Arduino Library for ADS1292R Shield/Breakout
//
//   Copyright (c) 2017 ProtoCentral
//
//   This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   Requires g4p_control graphing library for processing.  Built on V4.1
//   Downloaded from Processing IDE Sketch->Import Library->Add Library->G4P Install
//
/////////////////////////////////////////////////////////////////////////////////////////
#include "Arduino.h"
#include "Protocentral_ADS1292r.h"
#include <SPI.h>

volatile byte SPI_RX_Buff[15];
volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile bool ads1292dataReceived = false;
volatile bool ads1292r_intr_flag = false;

const int pwdn_pin = 27;
const int start_pin = 14;
const int chip_select = 17;

unsigned long uecgtemp = 0;
unsigned long resultTemp = 0;

signed long secgtemp = 0;

long status_byte=0;

uint8_t LeadStatus=0;

int j,i;

void IRAM_ATTR ads1292r_interrupt_handler(void)
{
 ads1292r_intr_flag = true;
}

boolean ads1292r::getAds1292r_Data_if_Available(const int data_ready,const int chip_select,ads1292r_data *data_struct)
{
  
   if (ads1292r_intr_flag)      // Sampling rate is set to 125SPS ,DRDY ticks for every 8ms
   {
     ads1292r_intr_flag = false;
     SPI_RX_Buff_Ptr = ads1292_Read_Data(chip_select); // Read the data,point the data to a pointer
     ads1292dataReceived = true;
     uecgtemp = (unsigned long) (  ((unsigned long)SPI_RX_Buff_Ptr[3] << 16) | ( (unsigned long) SPI_RX_Buff_Ptr[4] << 8) |  (unsigned long) SPI_RX_Buff_Ptr[5]);
     uecgtemp = (unsigned long) (uecgtemp << 8);
     secgtemp = (signed long) (uecgtemp);
     data_struct->raw_resp = secgtemp;
     uecgtemp = (unsigned long) (  ((unsigned long)SPI_RX_Buff_Ptr[6] << 16) | ( (unsigned long) SPI_RX_Buff_Ptr[7] << 8) |  (unsigned long) SPI_RX_Buff_Ptr[8]);
     uecgtemp = (unsigned long) (uecgtemp << 8);
     secgtemp = (signed long) (uecgtemp);
     secgtemp = (signed long) (secgtemp >> 8);
     data_struct->raw_ecg = secgtemp;
     status_byte = (long)((long)SPI_RX_Buff_Ptr[2] | ((long) SPI_RX_Buff_Ptr[1]) <<8 | ((long) SPI_RX_Buff_Ptr[0])<<16); // First 3 bytes represents the status
     status_byte  = (status_byte & 0x0f8000) >> 15;  // bit15 gives the lead status
     LeadStatus = (unsigned char ) status_byte ;
     data_struct->status_reg = LeadStatus;
     ads1292dataReceived = false;
     SPI_RX_Buff_Count = 0;
     return true;

    }
    else
    {
    return false;
    }

}

char* ads1292r::ads1292_Read_Data(const int chip_select)
{
  static char SPI_Dummy_Buff[10];
  digitalWrite(chip_select, LOW);

  for (int i = 0; i < 9; ++i)
  {
    SPI_Dummy_Buff[i] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
  }

  digitalWrite(chip_select, HIGH);
  return SPI_Dummy_Buff;
}

char* ads1292r::ads1292_Read_Data()
{
  static char SPI_Dummy_Buff[10];
  digitalWrite(chip_select, LOW);

  for (int i = 0; i < 9; ++i)
  {
    SPI_Dummy_Buff[i] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
  }

  digitalWrite(chip_select, HIGH);
  return SPI_Dummy_Buff;
}

void ads1292r::ads1292_Init(const int chip_select,const int pwdn_pin,const int start_pin)
{
  // start the SPI library:
  ads1292_Reset(pwdn_pin);
  delay(100);
  ads1292_Disable_Start(start_pin);
  ads1292_Enable_Start(start_pin);
  ads1292_Hard_Stop(start_pin);
  ads1292_Start_Data_Conv_Command(chip_select);
  ads1292_Soft_Stop(chip_select);
  delay(50);
  ads1292_Stop_Read_Data_Continuous(chip_select);					// SDATAC command
  delay(300);
  ads1292_Reg_Write(ADS1292_REG_CONFIG1, 0x00,chip_select); 		//Set sampling rate to 125 SPS
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CONFIG2, 0b10100000,chip_select);	//Lead-off comp off, test signal disabled
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_LOFF, 0b00010000,chip_select);		//Lead-off defaults
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CH1SET, 0b01000000,chip_select);	//Ch 1 enabled, gain 6, connected to electrode in
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CH2SET, 0b01100000,chip_select);	//Ch 2 enabled, gain 6, connected to electrode in
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_RLDSENS, 0b00101100,chip_select);	//RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_LOFFSENS, 0x00,chip_select);		//LOFF settings: all disabled
  delay(10);													//Skip register 8, LOFF Settings default
  ads1292_Reg_Write(ADS1292_REG_RESP1, 0b11110010,chip_select);		//Respiration: MOD/DEMOD turned only, phase 0
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_RESP2, 0b00000011,chip_select);		//Respiration: Calib OFF, respiration freq defaults
  delay(10);
  ads1292_Start_Read_Data_Continuous(chip_select);
  delay(10);
  ads1292_Enable_Start(start_pin); 
}

void ads1292r::ads1292_Init()
{
  // start the SPI library:
  ads1292_Reset();
  delay(100);
  ads1292_Disable_Start();
  ads1292_Enable_Start();
  ads1292_Hard_Stop();
  ads1292_Start_Data_Conv_Command();
  ads1292_Soft_Stop();
  delay(50);
  ads1292_Stop_Read_Data_Continuous();         // SDATAC command
  delay(300);
  ads1292_Reg_Write(ADS1292_REG_CONFIG1, 0x00);     //Set sampling rate to 125 SPS
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CONFIG2, 0b10100000); //Lead-off comp off, test signal disabled
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_LOFF, 0b00010000);    //Lead-off defaults
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CH1SET, 0b01000000);  //Ch 1 enabled, gain 6, connected to electrode in
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_CH2SET, 0b01100000);  //Ch 2 enabled, gain 6, connected to electrode in
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_RLDSENS, 0b00101100); //RLD settings: fmod/16, RLD enabled, RLD inputs from Ch2 only
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_LOFFSENS, 0x00);    //LOFF settings: all disabled
  delay(10);                          //Skip register 8, LOFF Settings default
  ads1292_Reg_Write(ADS1292_REG_RESP1, 0b11110010);   //Respiration: MOD/DEMOD turned only, phase 0
  delay(10);
  ads1292_Reg_Write(ADS1292_REG_RESP2, 0b00000011);   //Respiration: Calib OFF, respiration freq defaults
  delay(10);
  ads1292_Start_Read_Data_Continuous();
  delay(10);
  ads1292_Enable_Start(); 
}

void ads1292r::ads1292_Reset(const int pwdn_pin)
{
  digitalWrite(pwdn_pin, HIGH);
  delay(100);					// Wait 100 mSec
  digitalWrite(pwdn_pin, LOW);
  delay(100);
  digitalWrite(pwdn_pin, HIGH);
  delay(100);
}

void ads1292r::ads1292_Reset()
{
  digitalWrite(pwdn_pin, HIGH);
  delay(100);         // Wait 100 mSec
  digitalWrite(pwdn_pin, LOW);
  delay(100);
  digitalWrite(pwdn_pin, HIGH);
  delay(100);
}

void ads1292r::ads1292_Disable_Start(const int start_pin)
{
  digitalWrite(start_pin, LOW);
  delay(20);
}

void ads1292r::ads1292_Disable_Start()
{
  digitalWrite(start_pin, LOW);
  delay(20);
}

void ads1292r::ads1292_Enable_Start(const int start_pin)
{
  digitalWrite(start_pin, HIGH);
  delay(20);
}

void ads1292r::ads1292_Enable_Start()
{
  digitalWrite(start_pin, HIGH);
  delay(20);
}

void ads1292r::ads1292_Hard_Stop (const int start_pin)
{
  digitalWrite(start_pin, LOW);
  delay(100);
}

void ads1292r::ads1292_Hard_Stop ()
{
  digitalWrite(start_pin, LOW);
  delay(100);
}

void ads1292r::ads1292_Start_Data_Conv_Command (const int chip_select)
{
  ads1292_SPI_Command_Data(START,chip_select);					// Send 0x08 to the ADS1x9x
}

void ads1292r::ads1292_Start_Data_Conv_Command ()
{
  ads1292_SPI_Command_Data(START,chip_select);          // Send 0x08 to the ADS1x9x
}

void ads1292r::ads1292_Soft_Stop (const int chip_select)
{
  ads1292_SPI_Command_Data(STOP,chip_select);                   // Send 0x0A to the ADS1x9x
}

void ads1292r::ads1292_Soft_Stop ()
{
  ads1292_SPI_Command_Data(STOP,chip_select);                   // Send 0x0A to the ADS1x9x
}

void ads1292r::ads1292_Start_Read_Data_Continuous (const int chip_select)
{
  ads1292_SPI_Command_Data(RDATAC,chip_select);					// Send 0x10 to the ADS1x9x
}

void ads1292r::ads1292_Start_Read_Data_Continuous ()
{
  ads1292_SPI_Command_Data(RDATAC,chip_select);         // Send 0x10 to the ADS1x9x
}

void ads1292r::ads1292_Stop_Read_Data_Continuous (const int chip_select)
{
  ads1292_SPI_Command_Data(SDATAC,chip_select);					// Send 0x11 to the ADS1x9x
}

void ads1292r::ads1292_Stop_Read_Data_Continuous ()
{
  ads1292_SPI_Command_Data(SDATAC,chip_select);         // Send 0x11 to the ADS1x9x
}


void ads1292r::ads1292_SPI_Command_Data(unsigned char data_in,const int chip_select)
{
  byte data[1];
  digitalWrite(chip_select, LOW);
  delay(2);
  digitalWrite(chip_select, HIGH);
  delay(2);
  digitalWrite(chip_select, LOW);
  delay(2);
  SPI.transfer(data_in);
  delay(2);
  digitalWrite(chip_select, HIGH);
}

void ads1292r::ads1292_SPI_Command_Data(unsigned char data_in)
{
  byte data[1];
  digitalWrite(chip_select, LOW);
  delay(2);
  digitalWrite(chip_select, HIGH);
  delay(2);
  digitalWrite(chip_select, LOW);
  delay(2);
  SPI.transfer(data_in);
  delay(2);
  digitalWrite(chip_select, HIGH);
}

void ads1292r::ads1292_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA,const int chip_select)
{

  switch (READ_WRITE_ADDRESS)
  {
    case 1:
            DATA = DATA & 0x87;
	          break;
    case 2:
            DATA = DATA & 0xFB;
	          DATA |= 0x80;
	          break;
    case 3:
      	    DATA = DATA & 0xFD;
      	    DATA |= 0x10;
      	    break;
    case 7:
      	    DATA = DATA & 0x3F;
      	    break;
    case 8:
    	      DATA = DATA & 0x5F;
	          break;
    case 9:
      	    DATA |= 0x02;
      	    break;
    case 10:
      	    DATA = DATA & 0x87;
      	    DATA |= 0x01;
      	    break;
    case 11:
      	    DATA = DATA & 0x0F;
      	    break;
    default:
            break;
  }
  // now combine the register address and the command into one byte:
  byte dataToSend = READ_WRITE_ADDRESS | WREG;
  digitalWrite(chip_select, LOW);
  delay(2);
  digitalWrite(chip_select, HIGH);
  delay(2);
  // take the chip select low to select the device:
  digitalWrite(chip_select, LOW);
  delay(2);
  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(0x00);		//number of register to wr
  SPI.transfer(DATA);		//Send value to record into register
  delay(2);
  // take the chip select high to de-select:
  digitalWrite(chip_select, HIGH);
}

void ads1292r::ads1292_Reg_Write (unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
{

  switch (READ_WRITE_ADDRESS)
  {
    case 1:
            DATA = DATA & 0x87;
            break;
    case 2:
            DATA = DATA & 0xFB;
            DATA |= 0x80;
            break;
    case 3:
            DATA = DATA & 0xFD;
            DATA |= 0x10;
            break;
    case 7:
            DATA = DATA & 0x3F;
            break;
    case 8:
            DATA = DATA & 0x5F;
            break;
    case 9:
            DATA |= 0x02;
            break;
    case 10:
            DATA = DATA & 0x87;
            DATA |= 0x01;
            break;
    case 11:
            DATA = DATA & 0x0F;
            break;
    default:
            break;
  }
  // now combine the register address and the command into one byte:
  byte dataToSend = READ_WRITE_ADDRESS | WREG;
  digitalWrite(chip_select, LOW);
  delay(2);
  digitalWrite(chip_select, HIGH);
  delay(2);
  // take the chip select low to select the device:
  digitalWrite(chip_select, LOW);
  delay(2);
  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(0x00);   //number of register to wr
  SPI.transfer(DATA);   //Send value to record into register
  delay(2);
  // take the chip select high to de-select:
  digitalWrite(chip_select, HIGH);
}
