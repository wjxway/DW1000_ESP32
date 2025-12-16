=============================================================================
README.txt - Decawave's DW1000 Application Programming Interface (API)
=============================================================================

The DW1000 API package is composed of the following folders:

    - decadriver:
    
      Driver for DW1000 UWB transceiver IC.. Details about each function can
      be found in DW1000 API Guide.

    - examples:
    
      A set of individual (simple) examples showing how to achieve different
      functionalities, e.g. sending a frame, receiving a frame, putting the
      DW1000 to sleep, two-way ranging.  The emphasis of theses examples is
      to be readable with explanatory notes to explain and show how the main
      features of the DW1000 work and can be used.

    - Libraries, Linkers:
    
      Hardware abstraction layer (system start-up code and peripheral
      drivers) for ARM Cortex-M and ST STM32 F1 processors. Provided by ST
      Microelectronics.
      
    - platform:

      Platform dependant implementation of low-level features (IT management,
      mutex, sleep, etc).

Please refer to DW1000 API Guide accompanying this package for more details
about provided API and examples.

NOTE: The DW1000 API/driver code included in this package is an unbundled
      version of the DW1000 API/driver. This version may be different to
      (generally by being newer than) those bundled with Decawave's other
      products.

=============================================================================
=                                                                           =
=                               Release Notes                               =
=                                                                           =
=============================================================================

Summary:
-Update the dwt_intialise function to improve start up time 
-New Crystal Trim read function 
-New GPIO functions 
-New configuration function for improved performance when 64 preamble length is used 
-Improved OTP writing/reading APIs 
-Improved TX power/BW adjustment functions, removed use of floating point math

New Examples:
10a - GPIO example to show the use of DW1000 GPIOs to control LEDS 
1e  - TX with CCA to show how preamble detection may be used as a CCA method
2f  - RX with XTAL trim to show how local clock can be adjusted to match the 
      one of the remote transmitter

Updated Examples 
9a -  TX Bandwidth and Power Reference Measurements 
9b -  TX Bandwidth and Power Compensation 
4b -  Continuous frame mode 

=============================================================================
Package v2.14 / Driver v5.1.0  (20th December 2018)
=============================================================================

a) updated the API version number to 5.1.0 

b) updated dwt_intialise function config parameter to add extra configuration options
 which specify which OTP values, if any, to read from DW IC. Also a POWER_ON/WAKE_UP
 option has been added which limits the number of SPI read/writes when calling 
 this after DW IC wakeup (in case micro-processor was also in sleep state and 
 local structure was not preserved in memory)
 
c) removed dwt_getinitxtaltrim API, added dwt_getxtaltrim API, the latter 
allows the user to read curent XTAL trim value. If the user wishes to read 
OTP XTAL trim value, they can use dwt_otpread API to read address 0x1E(XTRIM_ADDRESS)
or they can read the current trim code immediately after the dwt_initialise 
call on power up.
 
d) added dwt_configurefor64plen API which configures the DW IC for 64 preamble
length operation when 64 length OPS table cannot be used due to large xtal offset

e) modified dwt_calcpowertempadj API to use raw temperature delta, and integer
numbers. Updated examples 9a and 9b to use raw temperature as read from the IC.
f) added new APIs to convert raw temperature to degrees and vice-versa, 
and also to convert raw volatge to volts and vice-versa.  
 
g) updated preamble coring threshold for PRF16/PAC8 to 0x3C

h) Updated the OTP writing APIs - removed unnecessary code, and modified code 
to achieve improved yield in DW IC production (e.g. charge pump test is not necessary)

i) Added temperature and voltage APIs for conversions from the raw values:  
dwt_convertrawtemperature and dwt_convertrawvoltage 

j) reduced the sleep() to 1 ms in dwt_calcbandwidthtempadj and dwt_calcpgcount
   the sleep() can be set as low as 10 us
   
k) correct couple of definitions to match the DW1000 User Manual 
   (e.g. AGC_CTRL_LEN set to 33, correct PANADR_PAN_ID_MASK)

 