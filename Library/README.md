Library that can be used to implement sensors and other applications on the Ambiq Apollo 3. 

Already supports the AS7341 and SHTC3 sensors, DAC63002 Digital-to-Analog-Converter, MB85RC64TA FRAM memory and the AM1805 external RTC.
Goal of this library is that a solid foundation of functionalities like timekeeping memory management is given to other people working with the EcoSense Microcontroller Platform based on the Ambiq Apollo 3
<br />
<br />
### Detailed list of provided functionalities:
* Provide drivers for SPI and I2C communication
* Provide drivers for RTC: timekeeping and sleeping (set in seconds)
* Provide drivers for the DAC63002
* Provide drivers for external FRAM
* Provide driver & measurement protocol for PAR using the AS7341 sensor build on included I2C Library
* Provide driver & measurement protocol for temperature/humidity using the SHTC3 sensor build on included I2C Library
* Provide a pin assignment file with all GPIO just named by their connected board
* Added source/header for a task handler where sensor implementations are supposed to be added. The implementation for the AS7341 can be used as an example on how the use of the task handler is intended 

<br />
<br />
<br />


# More specific description of the libraries:

### ADC:
* two configurations implemented: one for single shot measurements for e.g. the super cap measurement
* two initialization functions: one for each config, the PAM one can be configured for DMA operation using a #define
* function to deconfig/disable the ADC
* function for DMA configuration
* IRQ routine for either DMA or direct data aquisition
* function to measure the temperature using the ADC
* function to get a ADC sample
* timer control function to set the sampling frequency
* function to conduct the PAM sampling

### I2C 

* Configuration of the I2C module can be set here (100 kHz vs 400 kHz vs 1 MHz)
* Initialization of the I2C module
* De-Initialization of the I2C module
* Read & Write Function for the I2C module

### SPI

* Configuration of the SPI module can be set here
* Initialization of the SPI module
* De-Initialization of the SPI module
* Read & Write Function for SPI module

### GPIO

* Providing many preconfigurations for I2C, SPI, IRQ, Input, Output
* Function that initializes all GPIOs
* Function to read input GPIO
* Function to write output GPIO
* Functions to handle IRQs
* IRQ routine
* Function to enable printf commands

### Timing

* Initialization for stimer with different intervals
* Delay function where the Apollo with in sleep mode for x ms
* De-Initialization of the stimer
* IRQ handler for sleep & application timer
* Set frequency and duty-cycle for PWM generation using DAC

## Device Control

### (reduced) device.c

* Turn on/off external components using switches
* Initialize DAC for PWM operation
* Boardtest after manufacturing
* go to sleep
* Prepare platform for sensing
* Measure temperature using internal adc

### Taskhandler

* Function to execute the PAR measurement
* Function to initialize the PAR measurement
* Function to execute the Temperature measurement
* Function to calculate the PAR
* Function to execute all measurements in one

## (external) Peripherals

### AM1805:
* Function to read the ID to verify communication
* Function to initialize the RTC: configured to work as powerswitch device with time keeping
* Function to retrieve the current time of the ext RTC
* Function to set the time on the ext RTC
* Function to set the alarm time for wakeup
* Function to go to sleep and wakeup in x seconds
* Function to read the status register
* Function to read a register

### AS7341:
* Function to turn on the AS
* Function to read the status register
* Function to read a given register
* Function to clear the AS interrupts
* Function to configure the SMUX
* Function to start the measurement
* Function to set the wait time on the AS
* Function to configure the interrupts on the AS
* Function to configure the integration time for the measurement
* Function to configure the AStep and Atime
* Function to read the results for regular operation --deprecated
* Function to read the results for regular operation -- new one
* Function to read the results in IRQ controlled operation
* Function to set the gain for the internal ADCs
* Function to stop the measurement
* Function to test the communication with the module
* Function to configure the AS with given values
* IRQ handler for Apollo

### DAC63002

* Function to read the ID to verify communication
* Function to set the gain for the DAC voltage output
* Function to disable the comparator on channel 1
* Function to set the output range of the iOut mode (uA)
* Function to set the DAC1 comparator hysteresis
* Function to set the DAC scale
* Function to configure the DAC1 broadcast
* Function to set the DAC1 broadcast
* Function to set the DAC1 linear slew rate
* Function to set the DAC1 logarithmic slew rate
* Function to set the DAC1 output
* Function to set the DAC1 common config register
* Function to configure the DAC common trigger register
* Function to set the DAC trigger register
* Function to read the general status of the DAC
* Function to read the status of the comparator status register
* Function to set the device mode config register
* Function to read the DAC1 output register
* Function to read generic registers
* Function to write generic registers
* Function to take care of endianess
* Function to set DAC voltage in V





### INA232

* Function to test the communication
* Function to initialize the INA232
* Function to read the Shunt Voltage
* Function to read the Bus Voltage
* Function to read the (calculated) Power --deprecated because of low precision
* Function to read the current
* Function to read generic register

### MB85RC64TA (FRAM)

* Function to read ID/test communication
* Function to write blocking/non-blocking
* Function to read blocking/non-blocking

### SHTC-3

* Function to read data & conduct CRC
* Function that implements CRC
* Function to calculate temperature & humidity
* Function that return results of temperature & humidity measurement
* Function to read ID/test communication
* Functions to set sleep/wakeup







