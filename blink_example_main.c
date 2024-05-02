#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define RELAY_GPIO_PIN  GPIO_NUM_2 // Change this to the GPIO pin connected to the relay
#define UART_TX_PIN     GPIO_NUM_1 // UART TX pin
#define UART_RX_PIN     GPIO_NUM_3 // UART RX pin
#define BUF_SIZE        (1024)

void relay_control_task(void *pvParameters) {
    char rx_buffer[BUF_SIZE];
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure the GPIO pin connected to the relay
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << RELAY_GPIO_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Initially set the GPIO pin low
    gpio_set_level(RELAY_GPIO_PIN, 0);

    while (1) {
        int len = uart_read_bytes(UART_NUM_1, (uint8_t *)rx_buffer, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            rx_buffer[len] = 0; // Null-terminate the received string
            if (strcmp(rx_buffer, "on") == 0) {
                gpio_set_level(RELAY_GPIO_PIN, 1); // Turn on the relay
                printf("Relay turned ON\n");
            } else if (strcmp(rx_buffer, "off") == 0) {
                gpio_set_level(RELAY_GPIO_PIN, 0); // Turn off the relay
                printf("Relay turned OFF\n");
            } else {
                printf("Invalid command\n");
            }
        }
    }
}

void app_main() {
    xTaskCreate(&relay_control_task, "relay_control_task", 4096, NULL, 5, NULL);
}