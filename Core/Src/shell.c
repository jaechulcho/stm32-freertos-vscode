#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_intsup.h>
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "task.h"
#include "queue.h"
#include "FreeRTOS_CLI.h"
#include "shell.h"
#include "myerror.h"
#include "stm32l4xx_hal.h"
#include "uart_api.h"

#define SHELLTASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 128U)

static TaskHandle_t ShellTaskHandle;
static StackType_t  ShellTaskBuffer[SHELLTASK_STACK_SIZE];
static StaticTask_t xShellTaskBuffer;

#define SHELL_QUE_LENGTH    (256)
#define SHELL_QUE_ITEM_SIZE (sizeof(uint8_t))
static QueueHandle_t xShellUartRxQueue;
static StaticQueue_t xShellQueue;
static uint8_t       ucShellQueStorageArea[SHELL_QUE_LENGTH * SHELL_QUE_ITEM_SIZE];

#define CMD_IN_LEN  (SHELL_QUE_LENGTH)
#define CMD_OUT_LEN (SHELL_QUE_LENGTH * 2)
static const char* prompt = "> ";
static char        uinbuf[CMD_IN_LEN];
static char        uprevinbuf[CMD_IN_LEN];
static char        uoutbuf[CMD_OUT_LEN];

static int  xGetChar(uint8_t* prxChar_, size_t len_);
static void ShellTask(void* argument);
static int  vConsoleWrite(const char* buff);

/* shell command functions */
static BaseType_t prvCmdVersion(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString);

/* shell command structure list */
static const CLI_Command_Definition_t cmdlist[] = {
  { "gver", "gver: Get FW version\r\n", prvCmdVersion, 0 },
  { NULL, NULL, NULL, 0 }
};

int ShellTaskStart(void)
{
  int                             retval = ERR_NONE;
  const CLI_Command_Definition_t* pcmd;

  for (pcmd = cmdlist; pcmd->pcCommand != NULL; pcmd++) {
    (void)FreeRTOS_CLIRegisterCommand(pcmd);
  }

  if (retval == 0) {
    xShellUartRxQueue = xQueueCreateStatic(
        SHELL_QUE_LENGTH,
        SHELL_QUE_ITEM_SIZE,
        ucShellQueStorageArea,
        &xShellQueue);
    if (NULL == xShellUartRxQueue) {
      retval = ERR_QUEUECREATESTATIC;
    }
  }
  if (retval == 0) {
    retval = register_rxqueue(xShellUartRxQueue);
  }
  if (retval == 0) {
    ShellTaskHandle = xTaskCreateStatic(
        ShellTask,
        "ShellTask",
        SHELLTASK_STACK_SIZE,
        NULL,
        configMAX_PRIORITIES - 5,
        ShellTaskBuffer,
        &xShellTaskBuffer);
    if (NULL == ShellTaskHandle) {
      retval = ERR_TASKCREATESTATIC_FAIL;
    }
  }

  return retval;
}

static void ShellTask(void* argument)
{
  (void)argument;
  uint8_t    recvch   = '\0';
  uint16_t   uinindex = 0;
  BaseType_t remained;

  (void)uart_rx_it_enable();

  while (pdTRUE) {
    xGetChar(&recvch, sizeof(recvch));
    switch (recvch) {
    case '\r':
    case '\n':
      if (uinindex != 0U) {
        vConsoleWrite("\r\n\r\n");
        strncpy(uprevinbuf, uinbuf, sizeof(uprevinbuf));
        do {
          remained = FreeRTOS_CLIProcessCommand(
              uinbuf,
              uoutbuf,
              sizeof(uoutbuf));
          vConsoleWrite(uoutbuf);
        } while (remained == pdTRUE);
      }
      uinindex = 0;
      memset(uinbuf, 0, sizeof(uinbuf));
      memset(uoutbuf, 0, sizeof(uoutbuf));
      vConsoleWrite("\r\n");
      vConsoleWrite(prompt);
      break;
    case '\f':
      vConsoleWrite("\x1b[2J\x1b[0;0H");
      vConsoleWrite("\r\n");
      vConsoleWrite(prompt);
      break;
    case 3: /* Ctrl+C*/
      uinindex = 0;
      memset(uinbuf, 0, sizeof(uinbuf));
      vConsoleWrite("\r\n");
      vConsoleWrite(prompt);
      break;
    case 127:
    case 21:
    case '\b':
      if (uinindex > 0) {
        uinindex--;
        uinbuf[uinindex] = '\0';
        vConsoleWrite("\b \b");
      }
      break;
    case '\t':
      while (uinindex) {
        uinindex--;
        vConsoleWrite("\b \b");
      }
      strncpy(uinbuf, uprevinbuf, sizeof(uinbuf));
      uinindex = (uint16_t)strlen(uinbuf);
      vConsoleWrite(uinbuf);
      break;
    default:
      if ((uinindex < (sizeof(uinbuf) - 1)) && ((recvch >= 32) && (recvch <= 126))) {
        uinbuf[uinindex] = recvch;
        vConsoleWrite(uinbuf + uinindex);
        uinindex++;
      }
      break;
    }
  }
}

static int xGetChar(uint8_t* prxChar_, size_t len_)
{
  (void)len_;
  int        retval = ERR_NONE;
  BaseType_t xRet;

  if (xShellUartRxQueue == NULL || prxChar_ == NULL) {
    retval = ERR_INVALID_ARGUMENT;
  }
  if (retval == ERR_NONE) {
    xRet = xQueueReceive(xShellUartRxQueue, prxChar_, portMAX_DELAY);
    if (pdFALSE == xRet) {
      retval = ERR_QUEUERECV;
    }
  }

  return retval;
}

static BaseType_t prvCmdVersion(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString)
{
  BaseType_t retval = pdFALSE;
  (void)pcCommandString;

  (void)snprintf(pcWriteBuffer, xWriteBufferLen, "%s\r\n", "Hello. Wrorld");

  return retval;
}

static int vConsoleWrite(const char* buff)
{
  int retval;
  retval = printf("%s", buff);
  fflush(0);
  return retval;
}
