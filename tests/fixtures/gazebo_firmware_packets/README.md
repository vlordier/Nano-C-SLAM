# Gazebo-to-firmware SPI golden frames

These 97-byte hexadecimal frames were captured from the owned STM32 builders
in `cslam-stm32-app/src/spi_comm/spi_comm.c` by `tests/spi_comm_test.c`'s host
stub for the first sample in the accepted Gazebo run
`15a06f08-657e-4012-922e-4ca35f7dda22`.

The POSE fixture uses ID 0, timestamp 66100 ms, the sample's float32
odometry, and its four cardinal range readings duplicated into symmetric columns 3 and 4 of
`aug_pose_t.tof[4][8]`; all other ToF cells use -1. The projected bin centers
are +2.8125 and -2.8125 degrees around each cardinal direction. The SCAN fixture refers to pose ID 0. The C host
test calls the actual `spi_send_aug_pose` and `spi_send_scan_info` functions
and compares their captured frames with these files. The Python packetizer
also compares its output with the same vectors.
