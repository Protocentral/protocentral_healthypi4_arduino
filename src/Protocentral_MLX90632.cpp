//////////////////////////////////////////////////////////////////////////////////////////
//
// If you have bought the breakout the connection with the Arduino board is as follows:
//
//|MLX pin label| Arduino Connection   |Pin Function      |
//|----------------- |:--------------------:|-----------------:|
//| SDA              |    A4               |  Serial Data      |
//| SCL              | A5                  |  Serial Clock     |
//| Vin              | 5V                  |  Power            |
//| GND              | Gnd                 |  Gnd              |
//
//
// Based on the original MLX90632 librry by Nathan Seidle, Sparkfun Electronics
//
// This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU General Public License for more details.
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
/////////////////////////////////////////////////////////////////////////////////////////


#include "Protocentral_MLX90632.h"

double P_R;
double P_G;
double P_T;
double P_O;
double Ea;
double Eb;
double Fa;
double Fb;
double Ga;
double Gb;
double Ka;
double Ha;
double Hb;

double TOdut = 25.0; //Assume 25C for first iteration
double TO0 = 25.0; //object temp from previous calculation
double TA0 = 25.0; //ambient temp from previous calculation
double sensorTemp; //Internal temp of the MLX sensor

boolean Protocentral_MLX90632::begin()
{
  uint8_t deviceAddress = MLX90632_ADDRESS;
  TwoWire &wirePort = Wire;
  Protocentral_MLX90632::status returnError;
  if (begin(deviceAddress, wirePort, returnError) == true)
    return (true);
  return (false);
}

boolean Protocentral_MLX90632::begin(uint8_t deviceAddress, TwoWire &wirePort, status &returnError)
{
  returnError = SENSOR_SUCCESS;
  _deviceAddress = deviceAddress; //Get the I2C address from user
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  uint16_t thisAddress;
  returnError = readRegister16(EE_I2C_ADDRESS, thisAddress);
  if (thisAddress != _deviceAddress >> 1)
  {
    if (_printDebug)
    {
      _debugPort->print(F("Error: Communication failure. Check wiring. Expected device address: 0x"));
      _debugPort->print(_deviceAddress, HEX);
      _debugPort->print(F(", instead read: 0x"));
      _debugPort->print(thisAddress << 1, HEX);
      _debugPort->println();
    }
    returnError = SENSOR_ID_ERROR;
    return (false); //Error
  }

  //Load all the static calibration factors
  int16_t tempValue16;
  int32_t tempValue32;
  readRegister32(EE_P_R, (uint32_t&)tempValue32);
  P_R = (double)tempValue32 * pow(2, -8);
  readRegister32(EE_P_G, (uint32_t&)tempValue32);
  P_G = (double)tempValue32 * pow(2, -20);
  readRegister32(EE_P_T, (uint32_t&)tempValue32);
  P_T = (double)tempValue32 * pow(2, -44);
  readRegister32(EE_P_O, (uint32_t&)tempValue32);
  P_O = (double)tempValue32 * pow(2, -8);
  readRegister32(EE_Ea, (uint32_t&)tempValue32);
  Ea = (double)tempValue32 * pow(2, -16);
  readRegister32(EE_Eb, (uint32_t&)tempValue32);
  Eb = (double)tempValue32 * pow(2, -8);
  readRegister32(EE_Fa, (uint32_t&)tempValue32);
  Fa = (double)tempValue32 * pow(2, -46);
  readRegister32(EE_Fb, (uint32_t&)tempValue32);
  Fb = (double)tempValue32 * pow(2, -36);
  readRegister32(EE_Ga, (uint32_t&)tempValue32);
  Ga = (double)tempValue32 * pow(2, -36);

  readRegister16(EE_Gb, (uint16_t&)tempValue16);
  Gb = (double)tempValue16 * pow(2, -10);
  readRegister16(EE_Ka, (uint16_t&)tempValue16);
  Ka = (double)tempValue16 * pow(2, -10);
  readRegister16(EE_Ha, (uint16_t&)tempValue16);
  Ha = (double)tempValue16 * pow(2, -14); //Ha!
  readRegister16(EE_Hb, (uint16_t&)tempValue16);
  Hb = (double)tempValue16 * pow(2, -14);

  return (true);
}

float Protocentral_MLX90632::getObjectTemp()
{
  Protocentral_MLX90632::status returnError;
  return (getObjectTemp(returnError));
}

float Protocentral_MLX90632::getObjectTemp(status& returnError)
{
  returnError = SENSOR_SUCCESS;

  if(getMode() != MODE_CONTINUOUS) setSOC();

  clearNewData();

  uint16_t counter = 0;
  while (dataAvailable() == false)
  {
    delay(1);
    counter++;
    if (counter == MAX_WAIT)
    {
      if (_printDebug) _debugPort->println(F("Data available timeout"));
      returnError = SENSOR_TIMEOUT_ERROR;
      return (0.0);
    }
  }

  gatherSensorTemp(returnError);
  if (returnError != SENSOR_SUCCESS)
  {
    if (_printDebug)
    {
      _debugPort->println(F("Sensor temperature not found"));
      if(returnError == SENSOR_TIMEOUT_ERROR) _debugPort->println(F("Timeout"));
    }
    return (0.0); //Error
  }

 int16_t lowerRAM = 0;
  int16_t upperRAM = 0;

  int16_t sixRAM;
  readRegister16(RAM_6, (uint16_t&)sixRAM);
  int16_t nineRAM;
  readRegister16(RAM_9, (uint16_t&)nineRAM);

  //Object temp requires 3 iterations
  for (uint8_t i = 0 ; i < 3 ; i++)
  {
    double VRta = nineRAM + Gb * (sixRAM / 12.0);

    double AMB = (sixRAM / 12.0) / VRta * pow(2, 19);

    double sensorTemp = P_O + (AMB - P_R) / P_G + P_T * pow((AMB - P_R), 2);

    float S = (float)(lowerRAM + upperRAM) / 2.0;
    double VRto = nineRAM + Ka * (sixRAM / 12.0);
    double Sto = (S / 12.0) / VRto * (double)pow(2, 19);

    double TAdut = (AMB - Eb) / Ea + 25.0;

    double ambientTempK = TAdut + 273.15;

    double bigFraction = Sto / (1 * Fa * Ha * (1 + Ga * (TOdut - TO0) + Fb * (TAdut - TA0)));

    double objectTemp = bigFraction + pow(ambientTempK, 4);
    objectTemp = pow(objectTemp, 0.25); //Take 4th root
    objectTemp = objectTemp - 273.15 - Hb;
    TO0 = objectTemp;
  }

  return (TO0);
}

float Protocentral_MLX90632::getObjectTempF()
{
  float tempC = getObjectTemp();
  float tempF = tempC * 9.0/5.0 + 32.0;
  return(tempF);
}

float Protocentral_MLX90632::getSensorTemp()
{
  Protocentral_MLX90632::status tempFlag;
  return (getSensorTemp(tempFlag));
}

float Protocentral_MLX90632::getSensorTemp(status &returnError)
{
  returnError = SENSOR_SUCCESS;

  if(getMode() != MODE_CONTINUOUS) setSOC();

  clearNewData();

  uint16_t counter = 0;
  while (dataAvailable() == false)
  {
    delay(1);
    counter++;
    if (counter == MAX_WAIT)
    {
      returnError = SENSOR_TIMEOUT_ERROR;
      return (0.0);
    }
  }

  return (gatherSensorTemp(returnError));
}

float Protocentral_MLX90632::gatherSensorTemp(status &returnError)
{
  returnError = SENSOR_SUCCESS;

  int16_t sixRAM;
  readRegister16(RAM_6, (uint16_t&)sixRAM);
  int16_t nineRAM;
  readRegister16(RAM_9, (uint16_t&)nineRAM);

  double VRta = nineRAM + Gb * (sixRAM / 12.0);

  double AMB = (sixRAM / 12.0) / VRta * pow(2, 19);

  double sensorTemp = P_O + (AMB - P_R) / P_G + P_T * pow((AMB - P_R), 2);

  return(sensorTemp);
}

bool Protocentral_MLX90632::dataAvailable()
{
  if (getStatus() & ((uint16_t)1 << BIT_NEW_DATA)) return (true);
  return (false);
}
void Protocentral_MLX90632::clearNewData()
{
  uint16_t reg = getStatus(); //Get current bits
  reg &= ~(1 << BIT_NEW_DATA); //Clear the bit
  writeRegister16(REG_STATUS, reg); //Set the mode bits
}

uint16_t Protocentral_MLX90632::getStatus()
{
  Protocentral_MLX90632::status returnError;
  return (getStatus(returnError));
}

uint16_t Protocentral_MLX90632::getStatus(status& returnError)
{
  uint16_t deviceStatus;
  returnError = readRegister16(REG_STATUS, deviceStatus);
  return (deviceStatus);
}

void Protocentral_MLX90632::stepMode()
{
  setMode(MODE_STEP);
}

void Protocentral_MLX90632::continuousMode()
{
  setMode(MODE_CONTINUOUS);
}

Protocentral_MLX90632::status Protocentral_MLX90632::setSOC()
{
  uint16_t reg;
  Protocentral_MLX90632::status returnError = readRegister16(REG_CONTROL, reg); //Get current bits
  reg |= (1 << 3); //Set the bit
  writeRegister16(REG_CONTROL, reg); //Set the bit
  return (returnError);
}

Protocentral_MLX90632::status Protocentral_MLX90632::setMode(uint8_t mode)
{
  uint16_t reg;
  Protocentral_MLX90632::status returnError = readRegister16(REG_CONTROL, reg); //Get current bits
  reg &= ~(0x0003 << 1); //Clear the mode bits
  reg |= (mode << 1); //Set the bits
  writeRegister16(REG_CONTROL, reg); //Set the mode bits
  return (returnError);
}

uint8_t Protocentral_MLX90632::getMode()
{
  Protocentral_MLX90632::status returnError;
  return (getMode(returnError));
}

uint8_t Protocentral_MLX90632::getMode(status &returnError)
{
  uint16_t mode;
  returnError = readRegister16(REG_CONTROL, mode);
  mode = (mode >> 1) & 0x0003;
  return (mode);
}

Protocentral_MLX90632::status Protocentral_MLX90632::readRegister16(uint16_t addr, uint16_t &outputPointer)
{
  Protocentral_MLX90632::status returnError = SENSOR_SUCCESS;

  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(addr >> 8);
  _i2cPort->write(addr & 0xFF);
  //_i2cPort->endTransmission(false);
  if (_i2cPort->endTransmission(false) != 0)
  {

    if (_printDebug) _debugPort->println(F("I2C Error: End transmission"));
    returnError = SENSOR_I2C_ERROR;
  }

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)2);
  if (_i2cPort->available())
  {
    uint8_t msb = _i2cPort->read();
    uint8_t lsb = _i2cPort->read();

    outputPointer = (uint16_t)msb << 8 | lsb;
  }
  else
  {
    if (_printDebug) _debugPort->println(F("I2C Error: No read response"));
    returnError = SENSOR_I2C_ERROR;
  }

  return (returnError);
}

Protocentral_MLX90632::status Protocentral_MLX90632::readRegister32(uint16_t addr, uint32_t &outputPointer)
{
  Protocentral_MLX90632::status returnError = SENSOR_SUCCESS;

  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(addr >> 8);
  _i2cPort->write(addr & 0xFF);
  if (_i2cPort->endTransmission(false) != 0)
  {
    if (_printDebug) _debugPort->println(F("I2C Error: End transmission"));
    returnError = SENSOR_I2C_ERROR;
  }
  _i2cPort->requestFrom(_deviceAddress, (uint8_t)4);
  if (_i2cPort->available())
  {
    uint8_t msb0 = _i2cPort->read();
    uint8_t lsb0 = _i2cPort->read();
    uint8_t msb1 = _i2cPort->read();
    uint8_t lsb1 = _i2cPort->read();

    uint16_t lower = (uint16_t)msb0 << 8 | lsb0;
    uint16_t upper = (uint16_t)msb1 << 8 | lsb1;

    outputPointer = (uint32_t)upper << 16 | lower;
  }
  return (returnError);
}

Protocentral_MLX90632::status Protocentral_MLX90632::writeRegister16(uint16_t addr, uint16_t val)
{
  Protocentral_MLX90632::status returnError = SENSOR_SUCCESS;

  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(addr >> 8); //MSB
  _i2cPort->write(addr & 0xFF); //LSB
  _i2cPort->write(val >> 8); //MSB
  _i2cPort->write(val & 0xFF); //LSB
  if (_i2cPort->endTransmission() != 0)
  {
    //Sensor did not ACK
    if (_printDebug) _debugPort->println(F("I2C Error: End transmission"));
    returnError = SENSOR_I2C_ERROR;
  }
  return (returnError);
}
