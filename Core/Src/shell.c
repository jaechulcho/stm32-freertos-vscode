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

#define EMBEDDED_CLI_IMPL
#include "embedded_cli.h"

// for task
#define SHELLTASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 128U)

static TaskHandle_t ShellTaskHandle;
static StackType_t  ShellTaskBuffer[SHELLTASK_STACK_SIZE];
static StaticTask_t xShellTaskBuffer;

#define SHELL_QUE_LENGTH    (256)
#define SHELL_QUE_ITEM_SIZE (sizeof(uint8_t))
static QueueHandle_t xShellUartRxQueue;
static StaticQueue_t xShellQueue;
static uint8_t       ucShellQueStorageArea[SHELL_QUE_LENGTH * SHELL_QUE_ITEM_SIZE];

// for embedded_cli
// CLI buffer
static EmbeddedCli* cli;
static CLI_UINT     cliBuffer[BYTES_TO_CLI_UINTS(CLI_BUFFER_SIZE)];
// Bool to disable the interrupts, if CLI is not yet ready.
static bool         cliIsReady = false;

static void setupCli(void);
static void writeCharToCli(EmbeddedCli* embeddedCli, char c);

// private functions
static int  xGetChar(void);
static void ShellTask(void* argument);
static void initCliBinding(void);
static void Cmd_clearCLI(EmbeddedCli* cli, char* args, void* context);

/* shell command functions */

/* shell command structure list */

/* ShellTaskStart */
int ShellTaskStart(void)
{
  int retval = ERR_NONE;

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

/* ShellTask Impt */
static void ShellTask(void* argument)
{
  (void)argument;

  (void)setupCli();

  embeddedCliProcess(cli);
  while (pdTRUE) {
    (void)xGetChar();
    embeddedCliProcess(cli);
  }
}

static int xGetChar(void)
{
  int        retval = ERR_NONE;
  BaseType_t xRet;
  char       recvch;

  if (xShellUartRxQueue == NULL) {
    retval = ERR_INVALID_ARGUMENT;
  }
  if (retval == ERR_NONE) {
    xRet = xQueueReceive(xShellUartRxQueue, &recvch, portMAX_DELAY);
    if (pdFALSE == xRet) {
      retval = ERR_QUEUERECV;
    } else {
      embeddedCliReceiveChar(cli, recvch);
    }
  }

  return retval;
}

static void setupCli(void)
{
  // UART interrupt
  (void)uart_rx_it_enable();
  // Initialize the CLI configuration settings
  EmbeddedCliConfig* config  = embeddedCliDefaultConfig();
  config->cliBuffer          = cliBuffer;
  config->cliBufferSize      = CLI_BUFFER_SIZE;
  config->rxBufferSize       = CLI_RX_BUFFER_SIZE;
  config->cmdBufferSize      = CLI_CMD_BUFFER_SIZE;
  config->historyBufferSize  = CLI_HISTORY_SIZE;
  config->maxBindingCount    = CLI_MAX_BINDING_COUNT;
  config->enableAutoComplete = false;
  // Create new CLI instance
  cli                        = embeddedCliNew(config);
  // Assign character write function
  cli->writeChar             = writeCharToCli;

  // Add all the inital command bindings
  initCliBinding();
  // Init the CLI with blank screen
  // Cmd_clearCLI(cli, NULL, NULL);
  // CLI has now bean initialzed, set bool to true to enable interrupt.
  cliIsReady = true;
}

static void writeCharToCli(EmbeddedCli* embeddedCli, char c)
{
  (void)embeddedCli;

  (void)uart_send((char*)&c, 1);
}

static void initCliBinding(void)
{
  CliCommandBinding clear_binding = {
    .name         = "clear",
    .help         = "Clears the console",
    .tokenizeArgs = true,
    .context      = NULL,
    .binding      = Cmd_clearCLI
  };

  embeddedCliAddBinding(cli, clear_binding);
}

static void Cmd_clearCLI(EmbeddedCli* cli, char* args, void* context)
{
  (void)cli;
  (void)args;
  (void)context;

  printf("\33[2J");
  fflush(NULL);
}
