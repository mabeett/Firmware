# Steps for triggering the bug


build & transfer the project

	make all
	make download

Use a serial port terminal ask gtkterm or hyperterminal, config the serial port as 115200 8N1 and send a huge file with no interruptions.

The blinking will stop.

If wout apply Gustavo Muro's suggested patch ther will be problems, but seems being another cause:

* apply de patch

	`bash parchar_rtos.sh`

* Set your target in `Makefile.mine`, if you use an `edu-ciaa-nxp` you will need an UART adapter (rs-232 shifter or a FTDI one) since you 
will need transfer ata via `LPC_USART3` UART. In `ciaa-nxp` target this uart is the RS232 DB9 port.
* make all && make download
* From your computer transfer a huge file via FTDI UART and transfer a huge file via RS232 UART.
* Wait a - very large - period of time until leds `LED2` and `LEDB` stop blinking. This is the error.
* If you stop and start again the data transfer via `LPC_USART2` uart the ISR led blinks but not the task related via WaitEvent.

# Considerations

See the code for understanding the LED1, LED2 and LED3  indication.

The test project is made for edu-ciaa-nxp, but can with ciaa-nxp with output limitations, since digital outputs 0-3 are relays.

The files  `projects/blinky_uart_waitevent/inc/ciaaUART.h` `projects/blinky_uart_waitevent/inc/ciaaIO.h` 
`projects/blinky_uart_waitevent/src/ciaaIO.c` `projects/blinky_uart_waitevent/src/ciaaUART.c` were taken from
firmware v2 https://github.com/ciaa/firmware_v2/ and adapted to this enviroment.


