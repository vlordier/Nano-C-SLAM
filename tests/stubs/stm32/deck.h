#ifndef TEST_DECK_H
#define TEST_DECK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DECK_GPIO_IO1 1
#define DECK_GPIO_IO2 2
#define SPI_BAUDRATE_2MHZ 2000000

int digitalRead(uint32_t pin);
void digitalWrite(uint32_t pin, int value);
void spiBeginTransaction(uint32_t baudrate);
void spiEndTransaction(void);
void spiExchange(uint16_t length, const uint8_t *tx, uint8_t *rx);
bool spiBeginTransactionTimeout(uint32_t baudrate, uint32_t timeout_ticks);
bool spiExchangeTimeout(size_t length, const uint8_t *tx, uint8_t *rx,
                        uint32_t timeout_ticks);

#endif
