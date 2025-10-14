Application for resetting USB devices on macOS. Borrowed from USB Prober app. 

# Usage

`reenumerate [OPTIONS] [vendor_id,product_id [vendor_id,product_id] [locationID [location ID]]...`

# Options

The available options are as follows.  If no option is specified the values after the options are assumed to be a
sequence of vendorID,productID pairs (in hex).  If the `-l` option is specified, the values after the options are assumed
to be locationID's (in hex).  If no action option is specitied, a reenumerate command will be sent to the device(s):
```
--locationID, -l
	 The values after the options are locationIDs, instead of vendorID,productID.
--configuration, -c
	 Set the USB configuration to the value specified.
--resume, -r
	 Send a USB Resume to the device.
--suspend, -s
	 Send a USB Suspend to the device.
--reset, -R
	 Send a USB ResetDevice to the device.
--verbose, -v
	 Verbose mode.
--version, -V
	 Print version.
--help, -h, -?
	 Show this help.
```

One can determine vendor, product, and location IDs with

```
system_profiler SPUSBDataType
```

# Examples
Reset device at vid: 0x0488, pid: 0x5740

`reenumerate 0x0488,0x5740`

Set the configuration of the device at vid: 0x05ac, pid: 0x1126 to 1:
```
$ reenumerate -v -c 1 0x05ac,0x1126

reenumerate the devices at locationIDs 0xfa144300 and 0xfd141310

$ reenumerate -v -l 0xfa144300 0xfd141310
```

# Compiling

Either use CMake or `make`

# Installing

`make install` from CMake build directory
