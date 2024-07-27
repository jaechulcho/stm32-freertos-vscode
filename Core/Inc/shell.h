#ifndef __SHELL_H
#define __SHELL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "embedded_cli.h"

// for embedded_cli
// Definitions for CLI sizes
#define CLI_BUFFER_SIZE       (2048)
#define CLI_RX_BUFFER_SIZE    (16)
#define CLI_CMD_BUFFER_SIZE   (32)
#define CLI_HISTORY_SIZE      (32)
#define CLI_MAX_BINDING_COUNT (32)

#define CLI_PRINT_BUFFER_SIZE (512)

#define UART_RX_BUFF_SIZE (1U)


// exported functions
extern int ShellTaskStart(void);

#ifdef __cplusplus
}
#endif

#endif
