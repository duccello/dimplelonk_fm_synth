This is a little Polyphonic FM-like synth we made with a friend with a ESP32-WROVER-KIT.
The focus of this project was to use and re-use some spare electronics we had laying around so they can have an instrument to connect to their midi keyboard, and to learn something in the meanwhile.

GPIO connections

34 -> MIDI via 6N138 (I followed this schematic for the 5 pin conection )

14 -> I2S data (DIN)	14	DAC DIN

26 -> I2S bit clock (BCK)	26	DAC BCK

27 -> I2S word select (LRCK)	27	DAC LCK / LRCK

35 -> Encoder 1 - A 

36 -> Encoder 1 - B

39 -> Encoder 2 - A

25 -> Encoder 2 - B	

13 -> Encoder 3 - A

15 -> Encoder 3 - B

GPIO 35, 36 and 39 need pull-up resistors (10k to 3.3 V) - check if your encoders have them built in
DAC (PCM5100/5102): SCK to GND, plus 3.3 V and GND.
The LCD was already wired on the kit we had

The UI with the screen and encoders is intuitive enough for anyone to select and tweak sounds easily.
