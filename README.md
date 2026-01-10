# chromium-nb

Sources to build chromium on NetBSD:

1. HID support (services/device/hid): based on OpenBSD’s *fido* and the FreeBSD backends, and uses [libudev-bsd](https://github.com/kikadf/libudev-bsd).

## Usage

1. Extract the files into Chromium's source tree
2. For HID support add `use_udev=true` to your `GN_ARGS`

