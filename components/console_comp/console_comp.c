/*
 * console.c
 *
 *  Created on: Apr 30, 2021
 *      Author: strngr
 */
#include "console_comp.h"
#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "../SD_card/sd_card.h"
#include "esp_spi_flash.h"
#include "../gpio_ctrl/gpio_ctrl.h"

static const char *TAG = "console";

#define PROMPT_STR   "med_dev"
#define HISTORY_PATH MOUNT_POINT "/history.txt"

/* Prompt to be printed before each line.
 * This can be customized, made dynamic, etc.
*/
static const char *prompt = LOG_COLOR_I PROMPT_STR "> " LOG_RESET_COLOR;

//****** "set_gpio_pin" command ******////
static struct {
    struct arg_int *pin_num;
    struct arg_int *pin_mode;
    struct arg_int *pull_mode;
    struct arg_int *level;
    struct arg_end *end;
} gpio_pin_cfg_args;

static int gpio_pin_cfg(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&gpio_pin_cfg_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, gpio_pin_cfg_args.end, argv[0]);
        return 1;
    }
    return gpio_pin_configure(gpio_pin_cfg_args.pin_num->ival[0],
                              gpio_pin_cfg_args.pin_mode->ival[0],
                              gpio_pin_cfg_args.pull_mode->ival[0],
                              gpio_pin_cfg_args.level->ival[0]);
}

static void register_gpio_pin_cfg(void)
{
    gpio_pin_cfg_args.pin_num = arg_intn(
            "n",
            "pin_num",
            "<pin number>",
            0,
            GPIO_NUM_MAX,
            "The number of pin to set. For example GPIO0 has number 0, GPIO11 - 11");
    gpio_pin_cfg_args.pin_mode =
            arg_intn("m",
                     "pin_mode",
                     "<pin mode>",
                     0,
                     2,
                     "GPIO mode. 0 - DISABLE, 1 - INPUT, 2 - OUTPUT");
    gpio_pin_cfg_args.pull_mode = arg_intn(
            "p",
            "pull_mode",
            "<pull mode>",
            0,
            3,
            "Pull mode  for GPIO. 0 - PULL_UP, 1 - PULL_DOWN, 2 - PULLUP_PULLDOWN, 3 - FLOATING");
    gpio_pin_cfg_args.level =
            arg_intn("l",
                     "level",
                     "<output level>",
                     0,
                     1,
                     "GPIO voltage logic level. 0 - LOW, 1 - HIGH");
    gpio_pin_cfg_args.end       = arg_end(4);
    const esp_console_cmd_t cmd = {
        .command  = "gpio_set",
        .help     = "Set gpio mode and level",
        .hint     = NULL,
        .func     = &gpio_pin_cfg,
        .argtable = &gpio_pin_cfg_args,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
/* 'version' command */
static int get_version(int argc, char **argv)
{
    esp_chip_info_t info;
    esp_chip_info(&info);
    printf("IDF Version:%s\r\n", esp_get_idf_version());
    printf("Chip info:\r\n");
    printf("\tmodel:%s\r\n", info.model == CHIP_ESP32 ? "ESP32" : "Unknow");
    printf("\tcores:%d\r\n", info.cores);
    printf("\tfeature:%s%s%s%s%d%s\r\n",
           info.features & CHIP_FEATURE_WIFI_BGN ? "/802.11bgn" : "",
           info.features & CHIP_FEATURE_BLE ? "/BLE" : "",
           info.features & CHIP_FEATURE_BT ? "/BT" : "",
           info.features & CHIP_FEATURE_EMB_FLASH ? "/Embedded-Flash:" :
                                                    "/External-Flash:",
           spi_flash_get_chip_size() / (1024 * 1024),
           " MB");
    printf("\trevision number:%d\r\n", info.revision);
    return 0;
}

static void register_version(void)
{
    const esp_console_cmd_t cmd = {
        .command = "version",
        .help    = "Get version of chip and SDK",
        .hint    = NULL,
        .func    = &get_version,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_commands(void)
{
    ESP_ERROR_CHECK(esp_console_register_help_command());
    register_version();
    register_gpio_pin_cfg();
    /* Another register functions */
}
static void init_console(void)
{
    /* Drain stdout before reconfiguring it */
    fflush(stdout);
    fsync(fileno(stdout));

    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                              ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM,
                                              ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    const uart_config_t uart_config = {
        .baud_rate  = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .source_clk = UART_SCLK_REF_TICK,
    };
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK(uart_driver_install(
            CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(
            uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    /* Initialize the console */
    esp_console_config_t console_config = { .max_cmdline_args   = 8,
                                            .max_cmdline_length = 256,
                                            .hint_color =
                                                    atoi(LOG_COLOR_CYAN) };
    ESP_ERROR_CHECK(esp_console_init(&console_config));

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback *)&esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);

    /* Don't return empty lines */
    linenoiseAllowEmpty(false);

    /* Load command history from filesystem */
    linenoiseHistoryLoad(HISTORY_PATH);
    register_commands();
}

void console_task(void *pvParameters)
{
    // initializing console
    init_console();

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    printf("\n"
           "Type 'help' to get the list of commands.\n"
           "Use UP/DOWN arrows to navigate through command history.\n"
           "Press TAB when typing command name to auto-complete.\n"
           "Press Enter or Ctrl+C will terminate the console environment.\n");

    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = PROMPT_STR "> ";
    }

    for (;;) {
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        char *line = linenoise(prompt);
        if (line == NULL) { /* Break on EOF or error */
            break;
        }
        /* Add the command to the history if not empty*/
        if (strlen(line) > 0) {
            linenoiseHistoryAdd(line);
            /* Save command history to filesystem */
            linenoiseHistorySave(HISTORY_PATH);
        }
        /* Try to run the command */
        int       ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n",
                   ret,
                   esp_err_to_name(ret));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }

    ESP_LOGE(TAG, "Error or end-of-input, terminating console");
    esp_console_deinit();

    vTaskDelete(NULL);
}
