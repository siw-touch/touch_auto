# touch
SiW Touch Driver for Automotive & Large type

This driver supports Silicon-works Touch Device.

Test platform : Odroid-xu4 (Exynos5422, Android 4.4.4, Kernel v3.10.9)

Author:

kimhh@siliconworks.co.kr

# Recommended base folder

: {kernel top}/input/touchscreen/siw


List of supported devices

: SW1828, SW42103, SW17700

: SW42101


# for Built-in

{kernel top}/input/touchscreen/siw $ mv Kconfig_builtin Kconfig

{kernel top}/input/touchscreen/siw $ mv Makefile_builtin Makefile


Add this 1 line into {kernel top}/input/touchscreen/Kconfig

: source "drivers/input/touchscreen/siw/Kconfig"

Add this 1 line into {kernel top}/input/touchscreen/Makefile

: obj-$(CONFIG_TOUCHSCREEN_SIW) += siw/


# for Module test

{kernel top}/input/touchscreen/siw $ mv Makefile_module Makefile

{kernel top}/input/touchscreen/siw $ make


You can choose the device type in this Makefile(Makefile_module)

ENTRY_NAME=sw1770   //SW17700 selected

...

# for DTS

See '_reference / device_tree'


# git

[clone]

$ git clone https://github.com/siw-touch/touch_auto.git .

[push (id/pw required)]

$ git push origin master

