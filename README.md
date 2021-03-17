# Basic-TTN-Balloon-Tracker
No Frills TTN Cayenne GPS tracker using LMIC

Pin mappings for ATmega328P

Lora:
Atmega     |        Lora
-----------+-----------------
VCC         >      3.3V
GND         >      GND
D4          >      DIO0
D5          >      DIO1
D6          >      DIO5
D7          >      DIO2
D10         >      NSS
MOSI        >      MOSI
MISO        >      MISO 
CLK         >      SCK

GPS:
Atmega     |          GPS
-----------+-----------------
Pin 8      >          TX Pin GPS
Pin 9      >          RX pin  GPS
VCC        >          VCC GPS (3.3)
GND        >          GND GPS
