#ifndef BOUNDED_BATCH_H
#define BOUNDED_BATCH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef bool (*bounded_batch_send_fn)(const void *item, void *context);

/*
 * Send at most budget items from the head of a packed buffer. Successfully
 * sent items are removed; the first failed item and all following items stay
 * in their original order.
 */
bool bounded_batch_flush(void *items, size_t item_size, uint16_t *count,
                         size_t capacity, size_t budget,
                         bounded_batch_send_fn send_item, void *context);

#endif
