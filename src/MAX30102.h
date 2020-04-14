/*************************************************** 
 This is a library written for the Maxim MAX30102 Optical Smoke Detector
 It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.

 These sensors use I2C to communicate, as well as a single (optional)
 interrupt line that is not currently supported in this driver.
 
 Written by Peter Jansen and Nathan Seidle (SparkFun)
 BSD license, all text above must be included in any redistribution.
 *****************************************************/

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#define MAX30102_ADDRESS 0x57 //7-bit I2C Address
//Note that MAX30102 has the same I2C address and Part ID

#define I2C_SPEED_STANDARD 100000
#define I2C_SPEED_FAST 400000

// Register Map Address
// Status Registers
#define MAX30102_INTSTAT1 0x00   // Interrupt Status 1
#define MAX30102_INTSTAT2 0x01   // Interrupt Status 2
#define MAX30102_INTENABLE1 0x02 // Interrupt Enable 1
#define MAX30102_INTENABLE2 0x03 // Interrupt Enable 2

// FIFO Registers
#define MAX30102_FIFOWRITEPTR 0x04 // FIFO Write Pointer
#define MAX30102_FIFOOVERFLOW 0x05 // Overflow Counter
#define MAX30102_FIFOREADPTR 0x06  // FIFO Read Pointer
#define MAX30102_FIFODATA 0x07     // FIFO Data Register

// Configuration Registers
#define MAX30102_FIFOCONFIG 0x08     // FIFO Configuration
#define MAX30102_MODECONFIG 0x09     // MODE Configuration
#define MAX30102_PARTICLECONFIG 0x0A // SpO2 Configuration
#define MAX30102_LED1_PULSEAMP 0x0C  // LED Pulse Amplitude
#define MAX30102_LED2_PULSEAMP 0x0D  // LED Pulse Amplitude

#define MAX30102_MULTILEDCONFIG1 0x11 // Multi-LED Mode Control Registers SLOT2 & SLOT1
#define MAX30102_MULTILEDCONFIG2 0x12 // Multi-LED Mode Control Registers SLOT4 & SLOT3

// Die Temperature Registers
#define MAX30102_DIETEMPINT 0x1F    // Die Temp Integer
#define MAX30102_DIETEMPFRAC 0x20   // Die Temp Fraction
#define MAX30102_DIETEMPCONFIG 0x21 // Die Temperature Config

// Part ID Registers
#define MAX30102_REVISIONID 0xFE // Revision ID
#define MAX30102_PARTID 0xFF     //  PART ID Should always be 0x15. Identical to MAX30102.

// MAX30102 Commands
// Interrupt configuration (pg 12, 13)
#define MAX30102_INT_A_FULL_MASK 0x80 // A_FULL: FIFO Almost Full Flag
#define MAX30102_INT_A_FULL_ENABLE 0x80
#define MAX30102_INT_A_FULL_DISABLE 0x00

#define MAX30102_INT_DATA_RDY_MASK 0x40 // PPG_RDY: New FIFO Data Read
#define MAX30102_INT_DATA_RDY_ENABLE 0x40
#define MAX30102_INT_DATA_RDY_DISABLE 0x00

#define MAX30102_INT_ALC_OVF_MASK 0x20 // ALC_OVF: Ambient Light Cancellation Overflow
#define MAX30102_INT_ALC_OVF_ENABLE 0x20
#define MAX30102_INT_ALC_OVF_DISABLE 0x00

#define MAX30102_INT_PWR_RDY_INT_MASK 0x01 // PWR_RDY: Power Ready Flag

#define MAX30102_INT_DIE_TEMP_RDY_MASK 0x02 // DIE_TEMP_RDY: Internal Temperature Ready Flag
#define MAX30102_INT_DIE_TEMP_RDY_ENABLE 0x02
#define MAX30102_INT_DIE_TEMP_RDY_DISABLE 0x00

// FIFO Configuration (0x08) Page 17
#define MAX30102_SAMPLEAVG_MASK 0xE0 // Bits 7:5: Sample Averaging (SMP_AVE)

enum MAX30102_SAMPLEAVG
{
  MAX30102_SAMPLEAVG_1 = 0x00,
  MAX30102_SAMPLEAVG_2 = 0x20,
  MAX30102_SAMPLEAVG_4 = 0x40,
  MAX30102_SAMPLEAVG_8 = 0x60,
  MAX30102_SAMPLEAVG_16 = 0x80,
  MAX30102_SAMPLEAVG_32 = 0xA0
};

#define MAX30102_ROLLOVER_MASK 0xEF // Bit 4: FIFO Rolls on Full (FIFO_ROLLOVER_EN)
#define MAX30102_ROLLOVER_ENABLE 0x10
#define MAX30102_ROLLOVER_DISABLE 0x00

#define MAX30102_A_FULL_MASK 0xF0 // Bits 3:0: FIFO Almost Full Value (FIFO_A_FULL)

// Mode Configuration (0x09) commands (page 18)
#define MAX30102_SHUTDOWN_MASK 0x7F // Bit 7: Shutdown Control (SHDN)
#define MAX30102_SHUTDOWN 0x80
#define MAX30102_WAKEUP 0x00

#define MAX30102_RESET_MASK 0xBF // Bit 6: Reset Control (RESET)
#define MAX30102_RESET 0x40

#define MAX30102_MODE_MASK 0xF8 // Bits 2:0: Mode Control
enum MAX30102_MODE
{
  MAX30102_MODE_HEAR_RATE = 0x02, // Heart Rate mode, Red only
  MAX30102_MODE_SPO2 = 0x03,      // SpO2 mode, Red and IR
  MAX30102_MODE_MULTILED = 0x07   // Multi-LED mode, Red and IR
};

// SpO2 Configuration (0x0A) configuration commands (pgs 18, 19)
#define MAX30102_ADCRANGE_MASK 0x9F // Bits 6:5: SpO2 ADC Range Control
enum MAX30102_ADCRANGE
{
  MAX30102_ADCRANGE_2048 = 0x00,
  MAX30102_ADCRANGE_4096 = 0x20,
  MAX30102_ADCRANGE_8192 = 0x40,
  MAX30102_ADCRANGE_16384 = 0x60
};

#define MAX30102_SAMPLERATE_MASK 0xE3 // Bits 4:2: SpO2 Sample Rate Control
enum MAX30102_SAMPLERATE
{
  MAX30102_SAMPLERATE_50 = 0x00,
  MAX30102_SAMPLERATE_100 = 0x04,
  MAX30102_SAMPLERATE_200 = 0x08,
  MAX30102_SAMPLERATE_400 = 0x0C,
  MAX30102_SAMPLERATE_800 = 0x10,
  MAX30102_SAMPLERATE_1000 = 0x14,
  MAX30102_SAMPLERATE_1600 = 0x18,
  MAX30102_SAMPLERATE_3200 = 0x1C
};

#define MAX30102_PULSEWIDTH_MASK 0xFC // Bits 1:0: LED Pulse Width Control and ADC Resolution
enum MAX30102_PULSEWIDTH
{
  MAX30102_PULSEWIDTH_69 = 0x00,  // 69 (68.95)us
  MAX30102_PULSEWIDTH_118 = 0x01, // 118 (117.78)us
  MAX30102_PULSEWIDTH_215 = 0x02, // 215 (215.44)us
  MAX30102_PULSEWIDTH_411 = 0x03  // 411 (410.75)
};

//Multi-LED Mode configuration (pg 22)
#define MAX30102_SLOT1_MASK 0xF8
#define MAX30102_SLOT2_MASK 0x8F
#define MAX30102_SLOT3_MASK 0xF8
#define MAX30102_SLOT4_MASK 0x8F

#define SLOT_NONE 0x00
#define SLOT_RED_LED 0x01
#define SLOT_IR_LED 0x02

#define MAX30102_EXPECTEDPARTID 0x15

//Define the size of the I2C buffer based on the platform the user has
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

//I2C_BUFFER_LENGTH is defined in Wire.H
#define I2C_BUFFER_LENGTH BUFFER_LENGTH

#elif defined(__SAMD21G18A__)

//SAMD21 uses RingBuffer.h
#define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE

#else

//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

#endif

class MAX30102
{
public:
  MAX30102(void);

  int begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = MAX30102_ADDRESS);

  uint32_t getRed(void);                  //Returns immediate red value
  uint32_t getIR(void);                   //Returns immediate IR value
  bool safeCheck(uint8_t maxTimeToCheck); //Given a max amount of time, check for new data

  bool getRaw(uint32_t &red, uint32_t &ir);
  // Configuration
  void softReset();
  void shutDown();
  void wakeUp();

  void setLEDMode(uint8_t mode);

  void setADCRange(uint8_t adcRange);
  void setSampleRate(uint8_t sampleRate);
  void setPulseWidth(uint8_t pulseWidth);

  void setPulseAmplitudeRed(uint8_t value);
  void setPulseAmplitudeIR(uint8_t value);

  //Multi-led configuration mode (page 22)
  void enableSlot(uint8_t slotNumber, uint8_t device); //Given slot number, assign a device to slot
  void disableSlots(void);

  // Data Collection

  //Interrupts (page 13, 14)
  uint8_t getINT1(void);  //Returns the main interrupt group
  uint8_t getINT2(void);  //Returns the temp ready interrupt
  void enableAFULL(void); //Enable/disable individual interrupts
  void disableAFULL(void);
  void enableDATARDY(void);
  void disableDATARDY(void);
  void enableALCOVF(void);
  void disableALCOVF(void);
  void enableDIETEMPRDY(void);
  void disableDIETEMPRDY(void);

  //FIFO Configuration (page 18)
  void setFIFOAverage(uint8_t samples);
  void enableFIFORollover();
  void disableFIFORollover();
  void setFIFOAlmostFull(uint8_t samples);

  //FIFO Reading
  uint16_t check(void);        //Checks for new data and fills FIFO
  uint8_t available(void);     //Tells caller how many new samples are available (head - tail)
  void nextSample(void);       //Advances the tail of the sense array
  uint32_t getFIFORed(void);   //Returns the FIFO sample pointed to by tail
  uint32_t getFIFOIR(void);    //Returns the FIFO sample pointed to by tail
  uint32_t getFIFOGreen(void); //Returns the FIFO sample pointed to by tail

  uint8_t getWritePointer(void);
  uint8_t getReadPointer(void);
  void clearFIFO(void); //Sets the read/write pointers to zero

  // Die Temperature
  float readTemperature();
  float readTemperatureF();

  // Detecting ID/Revision
  uint8_t getRevisionID();
  uint8_t readPartID();

  // Setup the IC with user selectable settings
  void setup(byte powerLevel = 0x1F,
             MAX30102_SAMPLEAVG sampleAverage = MAX30102_SAMPLEAVG_2,
             MAX30102_MODE ledMode = MAX30102_MODE_SPO2,
             MAX30102_SAMPLERATE sampleRate = MAX30102_SAMPLERATE_400,
             MAX30102_PULSEWIDTH pulseWidth = MAX30102_PULSEWIDTH_411,
             MAX30102_ADCRANGE adcRange = MAX30102_ADCRANGE_4096);

  void updatePowerLevel(byte powerLevel);
  // Low-level I2C communication
  uint8_t readRegister8(uint8_t address, uint8_t reg);
  void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);

private:
  TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
  uint8_t _i2caddr;

  //activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
  byte activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO

  uint8_t revisionID;

  void readRevisionID();

  void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
};
