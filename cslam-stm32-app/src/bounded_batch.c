#include "bounded_batch.h"

#include <string.h>

bool bounded_batch_flush(void *items, size_t item_size, uint16_t *count,
                         size_t capacity, size_t budget,
                         bounded_batch_send_fn send_item, void *context) {
    if (items == NULL || item_size == 0 || count == NULL || *count > capacity ||
        send_item == NULL) {
        return false;
    }

    size_t sent = 0;
    while (*count > 0 && sent < budget) {
        if (!send_item(items, context))
            return false;

        (*count)--;
        sent++;
        if (*count > 0) {
            memmove(items, (unsigned char *)items + item_size,
                    *count * item_size);
        }
    }
    return true;
}
