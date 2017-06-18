# Steps for trigger the bug


build & transfer the project

	make all
	make download

Use a serial port terminal ask gtkterm or hyperterminal, config the serial port as 115200 8N1 and send a huge file wirh no interruptions.

The blinking will stop.

# Considerations

See the code for understanding the LED1, LED2 and LED3  indication.

The test project is made for edu-ciaa-nxp, but can with ciaa-nxp with output limitations, since digital outputs 0-3 are relays.

The files  `projects/blinky_uart_waitevent/inc/ciaaUART.h` `projects/blinky_uart_waitevent/inc/ciaaIO.h` 
`projects/blinky_uart_waitevent/src/ciaaIO.c` `projects/blinky_uart_waitevent/src/ciaaUART.c` were taken from
firmware v2 https://github.com/ciaa/firmware_v2/ and adapted to this enviroment.


