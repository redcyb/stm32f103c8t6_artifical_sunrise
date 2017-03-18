# stm32f103c8t6 Artifical Sunrise #

## This is a small simple project for stm32f103c8t6 MCU. ##



It uses alarm to start the artifical sunrise.

Also user can set light on/off via bluetooth.

Time can be set via bluetooth.


#### To set time use next 9-symbol pattern:

**sHH:MM:SS**

where 

HH - hours    00-24

MM - minutes  00-59

HH - seconds  00-59


#### To set light to **on** use next 9-symbol pattern:

**nPPxxxxxx**

where 

PP - percents from max power 00-99

xxxxxx - any 6 characters


#### To set light to **off** use next 9-symbol pattern:

**mxxxxxxxx**

where 

xxxxxxxx - any 8 characters
