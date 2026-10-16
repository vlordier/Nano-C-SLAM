#include <stdlib.h>

#include "../cslam-gap9-app/slam/rcm/circular-queue.h"

#define CHECK(condition) \
    do { \
        if (!(condition)) \
            abort(); \
    } while (0)

int main(void) {
    circular_queue_t queue;
    node value = {.index = 12, .neighbors = 2, .visited = 0};
    node result = {0};

    CHECK(!init_queue(NULL));
    CHECK(!is_full(NULL));
    CHECK(is_empty(NULL));
    CHECK(!push_to_queue(NULL, value));
    CHECK(!pop_from_queue(NULL, &result));
    CHECK(!pop_from_queue(&queue, NULL));

    CHECK(init_queue(&queue));
    CHECK(is_empty(&queue));
    CHECK(!pop_from_queue(&queue, &result));
    CHECK(push_to_queue(&queue, value));
    CHECK(!is_empty(&queue));
    CHECK(pop_from_queue(&queue, &result));
    CHECK(result.index == value.index && result.neighbors == value.neighbors);
    CHECK(is_empty(&queue));

    queue.tail = MAX_SIZE;
    CHECK(!push_to_queue(&queue, value));
    queue.tail = 0;
    queue.size = MAX_SIZE + 1;
    CHECK(!push_to_queue(&queue, value));
    CHECK(!pop_from_queue(&queue, &result));
    return 0;
}
