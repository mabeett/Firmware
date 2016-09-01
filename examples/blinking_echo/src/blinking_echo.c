/* Copyright 2014, Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking_echo example source file
 **
 ** This is a mini example of the CIAA Firmware to test the periodical
 ** task excecution and serial port funcionality.
 ** To run this sample in x86 plataform you must enable the funcionality of
 ** uart device setting a value of une or more of folowing macros defined
 ** in header file modules/plataforms/x86/inc/ciaaDriverUart_Internal.h
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Blinking Blinking_echo example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * MaCe         Mariano Cerdeiro
 * PR           Pablo Ridolfi
 * JuCe         Juan Cecconi
 * GMuro        Gustavo Muro
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20141019 v0.0.2   JuCe add printf in each task,
 *                        remove trailing spaces
 * 20140731 v0.0.1   PR   first functional version
 */

/*==================[inclusions]=============================================*/
#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "blinking_echo.h"         /* <= own header */
// #include "ciaaUART.h"

#ifndef CIAAUART_H_
#define CIAAUART_H_

#warning "FIXME: este parche deberia ser reemplazado por algo aceptable"
#ifdef FALSE
#undef FALSE
#endif

#ifdef TRUE
#undef TRUE
#endif

#include "chip.h"
#include "string.h"


typedef enum _ciaaUarts_e
{
        CIAA_UART_485 = 0,
        CIAA_UART_USB = 1,
        CIAA_UART_232 = 2
}ciaaUART_e;


#define rs485Print(x) uartSend(0, (uint8_t *)(x), strlen(x))
#define dbgPrint(x)   uartSend(1, (uint8_t *)(x), strlen(x))
#define rs232Print(x) uartSend(2, (uint8_t *)(x), strlen(x))

#define UART_BUF_SIZE   512
#define UART_RX_FIFO_SIZE 16

typedef struct _uartData
{
        LPC_USART_T * uart;
        RINGBUFF_T * rrb;
        RINGBUFF_T * trb;
}uartData_t;


void ciaaUARTInit(void);
int uartSend(ciaaUART_e nUART, void * data, int datalen);
int uartRecv(ciaaUART_e nUART, void * data, int datalen);

#endif /* CIAAUART_H_ */

uint8_t rxbuf[3][UART_BUF_SIZE];
uint8_t txbuf[3][UART_BUF_SIZE];

RINGBUFF_T rrb[3];
RINGBUFF_T trb[3];


uartData_t uarts[3] =
{
        { LPC_USART0, &(rrb[0]), &(trb[0]) },
        { LPC_USART2, &(rrb[1]), &(trb[1]) },
        { LPC_USART3, &(rrb[2]), &(trb[2]) },
};

// El kernel hace algunas de estas tareas drivers?
#define CIAAKSTART 1

// poner en 1 si se usa el posixopen para usb-uart
#define OPEN_FTDI232 0

// poner en 1 si se usa el posixopen para rs232
#define OPEN_RS232 0

// si el poil declara la interrupción ajustar en 1
#define OIL_EN_IRQ_USART3   1

// si el poil declara la interrupción ajustar en 1
#define OIL_EN_IRQ_USART2   1

void ciaaUARTInit(void)
{
    /* UART2 (USB-UART) */
#define ENABLEUSB 1
#if ENABLEUSB
/*  desde ciaak_start();
    luego ciaaDriverUart_init();
    luego ciaaDriverUart_hwInit(); */

#ifndef CIAAKSTART
    /* UART2 (USB-UART) */
    /* Chip_UART_Init(LPC_USART2); */
    /* Chip_UART_SetBaud(LPC_USART2, 115200); */

    /* esto lo agrega openOSEK-ciaa */
    /* Chip_UART_SetupFIFOS(LPC_USART2, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0); */

    /* Chip_UART_TXEnable(LPC_USART2); */

    /* Chip_SCU_PinMux(7, 1, MD_PDN, FUNC6);              */ /* P7_1: UART2_TXD */
    /* Chip_SCU_PinMux(7, 2, MD_PLN|MD_EZI|MD_ZI, FUNC6); */ /* P7_2: UART2_RXD */

    /* UART2 (USB-UART) */
    Chip_UART_Init(LPC_USART2);
    Chip_UART_SetBaud(LPC_USART2, 115200);

    Chip_UART_TXEnable(LPC_USART2);

    Chip_SCU_PinMux(7, 1, MD_PDN, FUNC6);              /* P7_1: UART2_TXD */
    Chip_SCU_PinMux(7, 2, MD_PLN|MD_EZI|MD_ZI, FUNC6); /* P7_2: UART2_RXD */
#endif /* CIAAKSTART */

/*  desde ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR);
    luego ciaaPOSIX_open(char const * path, uint8_t oflag);
    luego ciaaPOSIX_assert(serialDevice->device->open(path, (ciaaDevices_deviceType *)device->loLayer, oflag) == device->loLayer);
    luego ciaaDevices_deviceType * ciaaDriverUart_open(char const * path, ciaaDevices_deviceType * device, uint8_t const oflag) */
#if OPEN_FTDI232
#else
    /* Restart FIFOS: set Enable, Reset content, set trigger level */
    /* Chip_UART_SetupFIFOS((LPC_USART_T *)device->loLayer, UART_FCR_FIFO_EN | UART_FCR_TX_RS | UART_FCR_RX_RS | UART_FCR_TRG_LEV0); */
    /* dummy read */
    /* Chip_UART_ReadByte((LPC_USART_T *)device->loLayer); */
    /* enable rx interrupt */
    /* Chip_UART_IntEnable((LPC_USART_T *)device->loLayer, UART_IER_RBRINT); */

    /* Restart FIFOS: set Enable, Reset content, set trigger level */
    Chip_UART_SetupFIFOS(LPC_USART2, UART_FCR_FIFO_EN | UART_FCR_TX_RS | UART_FCR_RX_RS | UART_FCR_TRG_LEV0);
    /* dummy read */
    Chip_UART_ReadByte(LPC_USART2);
    /* enable rx interrupt */
    Chip_UART_IntEnable(LPC_USART2, UART_IER_RBRINT);
#endif /* OPEN_FTDI232 */

/*  desde  StartOS(AppMode1);
    luego StartOs_Arch_Cpu();
    luego Enable_User_ISRs(); */
#ifndef OIL_EN_IRQ_USART2
    NVIC_EnableIRQ(USART2_IRQn);
#endif /* OIL_EN_IRQ_USART2 */

    RingBuffer_Init(uarts[CIAA_UART_USB].rrb, rxbuf[CIAA_UART_USB], 1, UART_BUF_SIZE);
    RingBuffer_Init(uarts[CIAA_UART_USB].trb, txbuf[CIAA_UART_USB], 1, UART_BUF_SIZE);
#endif /* ENABLEUSB */

    /* UART3 (RS232) */
#define ENABLE232 0
#if ENABLE232
/*  desde ciaak_start();
    luego ciaaDriverUart_init();
    luego ciaaDriverUart_hwInit(); */
#ifndef CIAAKSTART
    /* UART3 (RS232) */
    /* Chip_UART_Init(LPC_USART3); */
    /* Chip_UART_SetBaud(LPC_USART3, 115200); */

    /* Chip_UART_SetupFIFOS(LPC_USART3, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0);

    /* Chip_UART_TXEnable(LPC_USART3); */

    /* Chip_SCU_PinMux(2, 3, MD_PDN, FUNC2);              */ /* P2_3: UART3_TXD */
    /* Chip_SCU_PinMux(2, 4, MD_PLN|MD_EZI|MD_ZI, FUNC2); */ /* P2_4: UART3_RXD */

    /* UART3 (RS232) */
    Chip_UART_Init(LPC_USART3);
    Chip_UART_SetBaud(LPC_USART3, 115200);

    Chip_UART_SetupFIFOS(LPC_USART3, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0);

    Chip_UART_TXEnable(LPC_USART3);

    Chip_SCU_PinMux(2, 3, MD_PDN, FUNC2);              /* P2_3: UART3_TXD */
    Chip_SCU_PinMux(2, 4, MD_PLN|MD_EZI|MD_ZI, FUNC2); /* P2_4: UART3_RXD */
#endif /* CIAAKSTART */

/*  desde ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR);
    luego ciaaPOSIX_open(char const * path, uint8_t oflag);
    luego ciaaPOSIX_assert(serialDevice->device->open(path, (ciaaDevices_deviceType *)device->loLayer, oflag) == device->loLayer);
    luego ciaaDevices_deviceType * ciaaDriverUart_open(char const * path, ciaaDevices_deviceType * device, uint8_t const oflag) */
#if OPEN_RS232
#else
    /* Restart FIFOS: set Enable, Reset content, set trigger level */
    /* Chip_UART_SetupFIFOS((LPC_USART_T *)device->loLayer, UART_FCR_FIFO_EN | UART_FCR_TX_RS | UART_FCR_RX_RS | UART_FCR_TRG_LEV0);*/
    /* dummy read */
    /* Chip_UART_ReadByte((LPC_USART_T *)device->loLayer);*/
    /* enable rx interrupt */
    /* Chip_UART_IntEnable((LPC_USART_T *)device->loLayer, UART_IER_RBRINT);*/

    /* Restart FIFOS: set Enable, Reset content, set trigger level */
    Chip_UART_SetupFIFOS(LPC_USART3, UART_FCR_FIFO_EN | UART_FCR_TX_RS | UART_FCR_RX_RS | UART_FCR_TRG_LEV0);
    /* dummy read */
    Chip_UART_ReadByte(LPC_USART3);
    /* enable rx interrupt */
    Chip_UART_IntEnable(LPC_USART3, UART_IER_RBRINT);
#endif /* OPEN_RS232 */
/*  desde  StartOS(AppMode1);
    luego StartOs_Arch_Cpu();
    luego Enable_User_ISRs(); */

#ifndef OIL_EN_IRQ_USART3
    NVIC_EnableIRQ(USART3_IRQn);
#endif /* OIL_EN_IRQ_USART2 */

    RingBuffer_Init(uarts[CIAA_UART_232].rrb, rxbuf[CIAA_UART_232], 1, UART_BUF_SIZE);
    RingBuffer_Init(uarts[CIAA_UART_232].trb, txbuf[CIAA_UART_232], 1, UART_BUF_SIZE);
#endif /* ENABLE232 */
}

void uart_irq(ciaaUART_e n)
{
    uartData_t * u = &(uarts[n]);

    Chip_UART_IRQRBHandler(u->uart, u->rrb, u->trb);
}

void UART0_IRQHandler(void)
{
    uart_irq(CIAA_UART_485);
}

void UART2_IRQHandler(void)
{
    uart_irq(CIAA_UART_USB);
}

void UART3_IRQHandler(void)
{
    uart_irq(CIAA_UART_232);
}

int uartSend(ciaaUART_e nUART, void * data, int datalen)
{
    uartData_t * u = &(uarts[nUART]);

    return Chip_UART_SendRB(u->uart, u->trb, data, datalen);
}

int uartRecv(ciaaUART_e nUART, void * data, int datalen)
{
    uartData_t * u = &(uarts[nUART]);

    return Chip_UART_ReadRB(u->uart, u->rrb, data, datalen);
}


/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
/** \brief File descriptor for digital input ports
 *
 * Device path /dev/dio/in/0
 */
static int32_t fd_in;

/** \brief File descriptor for digital output ports
 *
 * Device path /dev/dio/out/0
 */
static int32_t fd_out;


/** \brief Periodic Task Counter
 *
 */
static uint32_t Periodic_Task_Counter;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
   /* Starts the operating system in the Application Mode 1 */
   /* This example has only one Application Mode */
   StartOS(AppMode1);

   /* StartOs shall never returns, but to avoid compiler warnings or errors
    * 0 is returned */
   return 0;
}

/** \brief Error Hook function
 *
 * This fucntion is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
   ciaaPOSIX_printf("ErrorHook was called\n");
   ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{
   /* init CIAA kernel and devices */
   ciaak_start();

   // viene de ridolfi
   Chip_UART_SetupFIFOS(LPC_USART2, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV3);

   ciaaPOSIX_printf("Init Task...\n");
   /* open CIAA digital inputs */
   fd_in = ciaaPOSIX_open("/dev/dio/in/0", ciaaPOSIX_O_RDONLY);

   /* open CIAA digital outputs */
   fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);

   ciaaUARTInit();

   /* activate example tasks */
   Periodic_Task_Counter = 0;
   SetRelAlarm(ActivatePeriodicTask, 200, 200);

   /* Activates the SerialEchoTask task */
   ActivateTask(SerialEchoTask);

   /* end InitTask */
   TerminateTask();
}

/** \brief Serial Echo Task
 *
 * This tasks waits for input data from fd_uart1 and writes the received data
 * to fd_uart1 and fd_uart2. This taks alos blinkgs the output 5.
 *
 */
TASK(SerialEchoTask)
{
   int8_t buf[20];   /* buffer for uart operation              */
   uint8_t outputs;  /* to store outputs status                */
   int32_t ret;      /* return value variable for posix calls  */

   ciaaPOSIX_printf("SerialEchoTask...\n");
   /* send a message to the world :) */
   char message[] = "Hi! :)\nSerialEchoTask: Waiting for characters...\n";
   uartSend(CIAA_UART_USB, message, 49);

   if(1)
   {
      /* wait for any character ... */
      ret = uartRecv(CIAA_UART_USB, buf, 20);

      if(ret > 0)
      {
         /* ... and write them to the same device */
         uartSend(CIAA_UART_USB, buf, ret);

#if ENABLE232
         /* also write them to the other device */
         uartSend(CIAA_UART_232, buf, ret);
#endif
      }

      /* blink output 5 with each loop */
      ciaaPOSIX_read(fd_out, &outputs, 1);
      outputs ^= 0x20;
      ciaaPOSIX_write(fd_out, &outputs, 1);
   }
}

/** \brief Periodic Task
 *
 * This task is activated by the Alarm ActivatePeriodicTask.
 * This task copies the status of the inputs bits 0..3 to the output bits 0..3.
 * This task also blinks the output 4
 */
TASK(PeriodicTask)
{
   /*
    * Example:
    *    Read inputs 0..3, update outputs 0..3.
    *    Blink output 4
    */

   /* variables to store input/output status */
   uint8_t inputs = 0, outputs = 0;

   /* read inputs */
   ciaaPOSIX_read(fd_in, &inputs, 1);

   /* read outputs */
   ciaaPOSIX_read(fd_out, &outputs, 1);

   /* update outputs with inputs */
   outputs &= 0xF0;
   outputs |= inputs & 0x0F;

   /* blink */
   outputs ^= 0x10;

   /* write */
   ciaaPOSIX_write(fd_out, &outputs, 1);

   /* Print Task info */
   Periodic_Task_Counter++;
   ciaaPOSIX_printf("Periodic Task: %d\n", Periodic_Task_Counter);

   /* end PeriodicTask */
   TerminateTask();
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

