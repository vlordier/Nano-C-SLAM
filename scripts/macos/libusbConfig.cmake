# Homebrew's libusb formula ships pkg-config metadata, but no CMake package
# config. The Crazyswarm2 C++ driver expects these variables from find_package.
set(LIBUSB_INCLUDE_DIR "/opt/homebrew/opt/libusb/include/libusb-1.0")
set(LIBUSB_LIBRARY "/opt/homebrew/opt/libusb/lib/libusb-1.0.dylib")
set(libusb_FOUND TRUE)
