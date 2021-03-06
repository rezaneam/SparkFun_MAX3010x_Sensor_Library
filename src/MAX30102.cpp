/***************************************************
  This is a library written for the Maxim MAX30102 

  These sensors use I2C to communicate, as well as a single (optional)
  interrupt line that is not currently supported in this driver.

  Written by Peter Jansen and Nathan Seidle (SparkFun)
  BSD license, all text above must be included in any redistribution.
 *****************************************************/

#include "MAX30102.h"

//The MAX30102 stores up to 32 samples on the IC
//This is additional local storage to the microcontroller
const int STORAGE_SIZE = 4; //Each long is 4 bytes so limit this to fit on your micro
struct Record
{
  uint32_t red[STORAGE_SIZE];
  uint32_t IR[STORAGE_SIZE];
  uint32_t green[STORAGE_SIZE];
  byte head;
  byte tail;
} sense; //This is our circular buffer of readings from the sensor

MAX30102::MAX30102()
{
  // Constructor
}

int MAX30102::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2caddr)
{

  _i2cPort = &wirePort; //Grab which port the user wants us to use

  _i2cPort->begin();
  _i2cPort->setClock(i2cSpeed);

  _i2caddr = i2caddr;

  // Step 1: Initial Communication and Verification
  // Check that a MAX30102 is connected
  if (readPartID() != MAX30102_EXPECTEDPARTID)
  {
    // Error -- Part ID read from MAX30102 does not match expected part ID.
    // This may mean there is a physical connectivity problem (broken wire, unpowered, etc).
    return -1;
  }

  // Populate revision ID
  readRevisionID();

  return revisionID;
}

//
// Configuration
//

//Begin Interrupt configuration
uint8_t MAX30102::getINT1(void)
{
  return (readRegister8(_i2caddr, MAX30102_INTSTAT1));
}
uint8_t MAX30102::getINT2(void)
{
  return (readRegister8(_i2caddr, MAX30102_INTSTAT2));
}

void MAX30102::enableAFULL(void)
{
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_ENABLE);
}
void MAX30102::disableAFULL(void)
{
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_DISABLE);
}

void MAX30102::enableDATARDY(void)
{
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_ENABLE);
}
void MAX30102::disableDATARDY(void)
{
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_DISABLE);
}

void MAX30102::enableALCOVF(void)
{
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_ENABLE);
}
void MAX30102::disableALCOVF(void)
{
  bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_DISABLE);
}

void MAX30102::enableDIETEMPRDY(void)
{
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX30102::disableDIETEMPRDY(void)
{
  bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_DISABLE);
}

//End Interrupt configuration

void MAX30102::softReset(void)
{
  bitMask(MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    uint8_t response = readRegister8(_i2caddr, MAX30102_MODECONFIG);
    if ((response & MAX30102_RESET) == 0)
      break;  //We're done!
    delay(1); //Let's not over burden the I2C bus
  }
}

void MAX30102::shutDown(void)
{
  // Put IC into low power mode (datasheet pg. 19)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

void MAX30102::wakeUp(void)
{
  // Pull IC out of low power mode (datasheet pg. 19)
  bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

void MAX30102::setLEDMode(uint8_t mode)
{
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode);
}

void MAX30102::setADCRange(uint8_t adcRange)
{
  // adcRange: one of MAX30102_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange);
}

void MAX30102::setSampleRate(uint8_t sampleRate)
{
  // sampleRate: one of MAX30102_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}

void MAX30102::setPulseWidth(uint8_t pulseWidth)
{
  // pulseWidth: one of MAX30102_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void MAX30102::setPulseAmplitudeRed(uint8_t amplitude)
{
  writeRegister8(_i2caddr, MAX30102_LED1_PULSEAMP, amplitude);
}

void MAX30102::setPulseAmplitudeIR(uint8_t amplitude)
{
  writeRegister8(_i2caddr, MAX30102_LED2_PULSEAMP, amplitude);
}

//Given a slot number assign a thing to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void MAX30102::enableSlot(uint8_t slotNumber, uint8_t device)
{

  switch (slotNumber)
  {
  case (1):
    bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, device);
    break;
  case (2):
    bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT2_MASK, device << 4);
    break;
  case (3):
    bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT3_MASK, device);
    break;
  case (4):
    bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT4_MASK, device << 4);
    break;
  default:
    //Shouldn't be here!
    break;
  }
}

//Clears all slot assignments
void MAX30102::disableSlots(void)
{
  writeRegister8(_i2caddr, MAX30102_MULTILEDCONFIG1, 0);
  writeRegister8(_i2caddr, MAX30102_MULTILEDCONFIG2, 0);
}

//
// FIFO Configuration
//

//Set sample average (Table 3, Page 18)
void MAX30102::setFIFOAverage(uint8_t numberOfSamples)
{
  bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void MAX30102::clearFIFO(void)
{
  writeRegister8(_i2caddr, MAX30102_FIFOWRITEPTR, 0);
  writeRegister8(_i2caddr, MAX30102_FIFOOVERFLOW, 0);
  writeRegister8(_i2caddr, MAX30102_FIFOREADPTR, 0);
}

//Enable roll over if FIFO over flows
void MAX30102::enableFIFORollover(void)
{
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

//Disable roll over if FIFO over flows
void MAX30102::disableFIFORollover(void)
{
  bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void MAX30102::setFIFOAlmostFull(uint8_t numberOfSamples)
{
  bitMask(MAX30102_FIFOCONFIG, MAX30102_A_FULL_MASK, numberOfSamples);
}

//Read the FIFO Write Pointer
uint8_t MAX30102::getWritePointer(void)
{
  return (readRegister8(_i2caddr, MAX30102_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
uint8_t MAX30102::getReadPointer(void)
{
  return (readRegister8(_i2caddr, MAX30102_FIFOREADPTR));
}

// Die Temperature
// Returns temp in C
float MAX30102::readTemperature()
{
  // Step 1: Config die temperature register to take 1 temperature sample
  writeRegister8(_i2caddr, MAX30102_DIETEMPCONFIG, 0x01);

  // Poll for bit to clear, reading is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    uint8_t response = readRegister8(_i2caddr, MAX30102_DIETEMPCONFIG);
    if ((response & 0x01) == 0)
      break;  //We're done!
    delay(1); //Let's not over burden the I2C bus
  }
  //TODO How do we want to fail? With what type of error?
  //? if(millis() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)
  int8_t tempInt = readRegister8(_i2caddr, MAX30102_DIETEMPINT);
  uint8_t tempFrac = readRegister8(_i2caddr, MAX30102_DIETEMPFRAC);

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}

// Returns die temp in F
float MAX30102::readTemperatureF()
{
  float temp = readTemperature();

  if (temp != -999.0)
    temp = temp * 1.8 + 32.0;

  return (temp);
}

//
// Device ID and Revision
//
uint8_t MAX30102::readPartID()
{
  return readRegister8(_i2caddr, MAX30102_PARTID);
}

void MAX30102::readRevisionID()
{
  revisionID = readRegister8(_i2caddr, MAX30102_REVISIONID);
}

uint8_t MAX30102::getRevisionID()
{
  return revisionID;
}

//Setup the sensor
//The MAX30102 has many settings. By default we select:
// Sample Average = 4
// Mode = MultiLED
// ADC Range = 16384 (62.5pA per LSB)
// Sample rate = 50
//Use the default setup if you are just getting started with the MAX30102 sensor
void MAX30102::setup(byte powerLevel,
                     MAX30102_SAMPLEAVG sampleAverage,
                     MAX30102_MODE ledMode,
                     MAX30102_SAMPLERATE sampleRate, MAX30102_PULSEWIDTH pulseWidth,
                     MAX30102_ADCRANGE adcRange)
{
  softReset(); //Reset all configuration, threshold, and data registers to POR values

  //FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //The chip will average multiple samples of same type together if you wish
  setFIFOAverage(sampleAverage);

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  enableFIFORollover(); //Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  setLEDMode(ledMode);

  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  setADCRange(adcRange);

  setSampleRate(sampleRate);

  //The longer the pulse width the longer range of detection you'll have
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  setPulseWidth(pulseWidth);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeIR(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Multi-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  enableSlot(1, SLOT_RED_LED);
  if (ledMode > 1)
    enableSlot(2, SLOT_IR_LED);
  //enableSlot(1, SLOT_RED_PILOT);
  //enableSlot(2, SLOT_IR_PILOT);
  //enableSlot(3, SLOT_GREEN_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  clearFIFO(); //Reset the FIFO before we begin checking the sensor
}

void MAX30102::updatePowerLevel(byte powerLevel)
{
  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeIR(powerLevel);
}

//
// Data Collection
//

//Tell caller how many samples are available
uint8_t MAX30102::available(void)
{
  int8_t numberOfSamples = sense.head - sense.tail;
  if (numberOfSamples < 0)
    numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}

//Report the most recent red value
uint32_t MAX30102::getRed(void)
{
  //Check the sensor for new data for 250ms
  if (safeCheck(250))
    return (sense.red[sense.head]);
  else
    return (0); //Sensor failed to find new data
}

//Report the most recent IR value
uint32_t MAX30102::getIR(void)
{
  //Check the sensor for new data for 250ms
  if (safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return (0); //Sensor failed to find new data
}

bool MAX30102::getRaw(uint32_t &red, uint32_t &ir)
{
  if (safeCheck(250))
  {
    ir = sense.IR[sense.head];
    red = sense.red[sense.head];
    return true;
  }
  return false;
}

//Report the next Red value in the FIFO
uint32_t MAX30102::getFIFORed(void)
{
  return (sense.red[sense.tail]);
}

//Report the next IR value in the FIFO
uint32_t MAX30102::getFIFOIR(void)
{
  return (sense.IR[sense.tail]);
}

//Report the next Green value in the FIFO
uint32_t MAX30102::getFIFOGreen(void)
{
  return (sense.green[sense.tail]);
}

//Advance the tail
void MAX30102::nextSample(void)
{
  if (available()) //Only advance the tail if new data is available
  {
    sense.tail++;
    sense.tail %= STORAGE_SIZE; //Wrap condition
  }
}

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained
uint16_t MAX30102::check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  byte readPointer = getReadPointer();
  byte writePointer = getWritePointer();

  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0)
      numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
    _i2cPort->beginTransmission(MAX30102_ADDRESS);
    _i2cPort->write(MAX30102_FIFODATA);
    _i2cPort->endTransmission();

    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor
      _i2cPort->requestFrom(MAX30102_ADDRESS, toGet);

      while (toGet > 0)
      {
        sense.head++;               //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        byte temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;

        //Burst read three bytes - RED
        temp[3] = 0;
        temp[2] = _i2cPort->read();
        temp[1] = _i2cPort->read();
        temp[0] = _i2cPort->read();

        //Convert array to long
        memcpy(&tempLong, temp, sizeof(tempLong));

        tempLong &= 0x3FFFF; //Zero out all but 18 bits

        sense.red[sense.head] = tempLong; //Store this reading into the sense array

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - IR
          temp[3] = 0;
          temp[2] = _i2cPort->read();
          temp[1] = _i2cPort->read();
          temp[0] = _i2cPort->read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

          tempLong &= 0x3FFFF; //Zero out all but 18 bits

          sense.IR[sense.head] = tempLong;
        }

        if (activeLEDs > 2)
        {
          //Burst read three more bytes - Green
          temp[3] = 0;
          temp[2] = _i2cPort->read();
          temp[1] = _i2cPort->read();
          temp[0] = _i2cPort->read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

          tempLong &= 0x3FFFF; //Zero out all but 18 bits

          sense.green[sense.head] = tempLong;
        }

        toGet -= activeLEDs * 3;
      }

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found
}

//Check for new data but give up after a certain amount of time
//Returns true if new data was found
//Returns false if new data was not found
bool MAX30102::safeCheck(uint8_t maxTimeToCheck)
{
  uint32_t markTime = millis();

  while (1)
  {
    if (millis() - markTime > maxTimeToCheck)
      return (false);

    if (check() == true) //We found new data!
      return (true);

    delay(1);
  }
}

//Given a register, read it, mask it, and then set the thing
void MAX30102::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(_i2caddr, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister8(_i2caddr, reg, originalContents | thing);
}

//
// Low-level I2C Communication
//
uint8_t MAX30102::readRegister8(uint8_t address, uint8_t reg)
{
  _i2cPort->beginTransmission(address);
  _i2cPort->write(reg);
  _i2cPort->endTransmission(false);

  _i2cPort->requestFrom((uint8_t)address, (uint8_t)1); // Request 1 byte
  if (_i2cPort->available())
  {
    return (_i2cPort->read());
  }

  return (0); //Fail
}

void MAX30102::writeRegister8(uint8_t address, uint8_t reg, uint8_t value)
{
  _i2cPort->beginTransmission(address);
  _i2cPort->write(reg);
  _i2cPort->write(value);
  _i2cPort->endTransmission();
}
