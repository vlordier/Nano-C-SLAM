# UWB token tenure bound

The receive-side token lease is 55 seconds. This bound depends on the
per-tenure quotas in `cslam-stm32-app/src/config_params.h` and the current
`NR_OF_NODES == 2`; changing either the quotas, timeouts, or number of peers
requires recalculating it.

Each deck SPI transaction is bounded by a 100 ms mutex wait and a 100 ms
full-duplex DMA wait. A pose send performs two transactions (dummy clocks and
payload), so its conservative upper bound is 400 ms. A pose read performs four
transactions plus two 100 ms GAP9-ready/completion waits, so it is bounded by
1 second. The existing GAP9 busy-low deadlines for SCAN and OPTIMIZE are 10
seconds each.

The largest combined owner tenure includes one of each bounded activity:

| Work | Conservative bound |
| --- | ---: |
| One internal augmented pose | 0.400 s |
| One external augmented pose and one external pose update | 0.800 s |
| One scan: 20 UWB pose sends (3 attempts each) and GAP9 SCAN handshake | 11.010 s |
| Optimization: initial/correction reads, 20 updated pose reads, OPTIMIZE handshake, and three UWB update packets | 32.750 s |
| One-peer four-way ranging retries and token handoff retries | 0.100 s |
| Scheduler and explicit per-item delays | 0.100 s |
| **Calculated total** | **45.160 s** |

The 55-second lease leaves 9.84 seconds beyond this code-derived bound. Queue
and external-pose backlog is retained in FIFO order and is drained over
subsequent token tenures. The optimized-pose batch remains capped by the
20-entry `scan_poses` array. These are software timeout calculations, not
physical radio or flight measurements. Hardware timing can exceed these
assumptions and still requires propeller-off validation.

After a successful outgoing token handoff, the STM32 clears its one-entry UWB
message queue before enabling the responder again. This removes already
queued ACK/retry messages that could otherwise be mistaken for a fresh token
after the next tenure. A failed handoff lands and does not clear-and-continue.
The current TOKEN payload has no epoch, so a radio packet that arrives after
the queue purge still cannot be distinguished from a fresh token in software;
that case remains a propeller-off radio validation requirement.
