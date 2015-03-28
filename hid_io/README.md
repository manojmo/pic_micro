# General purpose IO routines for a USB HID device
This project was started with the idea of turning a USB device into a kind of IO Swiss-knife,
so that many things could be done with a single program. The use-case was to implement a remote-control,
first to read the durations, and next to write them back. I had success in the read, the writes require
more accurate timings. It is based on Microchips *Custom HID* MLA project.
Not all files are provided, only the main ones that were changed. The app_device_hid_custom.* files 
replaced with app_device_hid_io.*. Actually, i wanted to abstract the io routines into pin_io module,
which was independent of protocol, i.e USB/Serial/Other. And to create it as a stanalone project with as few files
as possible. Some such items are TODO, i may not get time to finish them. i have used the 18F4550 for this project.

## Firing the IO commands :
This is being done using the pyusb framework. The hid_io_test.py file has the code to send the commands
and receive the results. Just as a POC, the results can be saved to a .JS file, and viewed as a chart in mychart.html,
using the Chart.js framework. That needs to be downloaded separately. The script also has a facility to adjust 
results, based on expected timing inaccuracies. For the sampling commands, num_samples are specified, and the
script will calculate the number of packets and loop to receive them.

## Common features
The framework is generic, and the commands accept input like which pin to use, what delay intervals, etc.
(As a result, accurate timing is an issue. ). There is a debug feature that can return you the actual time taken
for an interval. (not very exact) routines are provided to measure and execute 2^24 cycle delays. 
Timings can be specified in uS or mS. There are flags for each command, to provide additional control.
All sampling commands also specify an IDLE state/value, and samplin does to start until the input changes from
the IDLE state/value. This is useful for manually-triggered sampling, like reading from a remote.

The commands are
## WP : Write Pin :
This allows us to write a pattern to the `<pin_num>`. A pattern is an array of 2-byte durations, 
starting from HI. Currently, upto ~ 60 durations are supported. They end with a zero duration marker. 
There is a facility to make the HIGHs pulsed at a carrier-freq, as needed for IR codes. 
A reset flag, ms/uS flag, and a repeat flag are also provided.

## RP : Read Pin :
This allows us to read `<num_samples>` of HIGH/LOW  states from `<pin_num>` at `<sampleInterval>` intervals. 
Not well tested. TODO : store the result in bits instead of bytes. A debug flag and mS/uS flag are also provided.

## RP : Read Pin durations :
This allows us to read `<num_samples>` durations of HIGH/LOWs by sampling `<pin_num>` at intervals of `<sampleInterval>`.
durations upto FFFF cycles are curently supported. A debug flag, idle-state flag and mS/uS flag are also provided.
This was used to read durations of IR-code from tata-sky remote, but i needed to adjust the values for code-exec
delays. i have blogged about it here : http://chaukasalshi.blogspot.in/2015/03/pic-micro-remote-for-tata-sky.html

## RA : Read ADC :
This allows us to read `<num_samples>` samples from ADC at intervals of `<sampleInterval>`. 
A debug flag, idle-value and mS/uS flag are also provided

