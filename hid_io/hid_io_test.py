import usb.core
import usb.util
import sys, math
import json

# find our device
dev = usb.core.find(idVendor=0x04d8, idProduct=0x003f)

# was it found?
if dev is None:
    raise ValueError('Device not found')

interface = 0

if dev.is_kernel_driver_active(interface) is True:
            print "but we need to detach kernel driver"
            dev.detach_kernel_driver(interface)

# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.set_configuration()

# get an endpoint instance
cfg = dev.get_active_configuration()
#intf = cfg[(0,0)]

for cfg in dev:
    sys.stdout.write("ConfigurationValue:" + str(cfg.bConfigurationValue) + '\n')
    for intf in cfg:
        #print vars(intf)
        sys.stdout.write('\t' + \
                         "InterfaceNumber:" + str(intf.bInterfaceNumber) + \
                         ',' + \
                         "AlternateSetting:" + str(intf.bAlternateSetting) + \
                         '\n')
        for ep in intf:
            #print vars(ep)
            sys.stdout.write('\t\t' + \
                             "EndpointAddress:" + str(ep.bEndpointAddress) + \
                             '\n')

assert ep is not None

cmd  = sys.argv[1]
scalingFactor = float(1) # the reading may need to be scaled. e.g. /3 if it is number of instruction cycles

if cmd == 'WP' : 
# Write Pin
    command = 0x11 # COMMAND_IO_WRITE_PIN
    pinNum = 0xd4 # Naming convention like port+pin, e.g 0xd0=RD0, 0xa1= RA1 etc
    carrierInterval = 26 # The carrier pulsing during high, e.g. for infrared remote etc
    carrier_duty_cycle_on = 4; # part of carrierInterval in hi state
    carrier_duty_cycle_off = 4; # part of carrierInterval in lo state. We can keep hi + lo < total to compensate for other code exec
    durationsData = ""
    # we can send data > 64 bytes, internally, it will be split into packets of 64 bytes
    arr_durations = [2666,889,444,444,444,444,444,889,444,889,889,444,444,444,444,444,444,444,444,444,444,444,444,444,444,
        444,444,444,444,444,444,444,444,444,444,444,444,444,889,889,444,444,444,444,444,444,444,444,444, 65000]

    #for loop in range(1) :
    #    arr_durations.extend( (100, 200))
    print arr_durations

    print "sum(arr_durations):", sum(arr_durations) , "len:", len( arr_durations)
    for duration in arr_durations :
        durationsData = durationsData + chr( duration >>8) + chr( duration & 0xFF)
    DURATION_START_INDEX = 7
    # the extended flag tells the usb app that the command is not yet complete, more data is to follow.
    flag_extended = 1 if len( arr_durations) * 2 > (64 - DURATION_START_INDEX) else 0 #1 : Extended( > 64 bytes < 128 bytes), 0 : <64 bytes
    flag_repeat = 0 # 2 is ON, pattern is repeated num_repeats times.
    num_repeats = 2 # a value of 0 means infinite
    flag_is_millis = 0 # 4 : durations in milliseconds, 0 : in microseconds
    flag_reset = 8 # 8 : reset to 0 after done
    flags = flag_extended | flag_repeat | flag_is_millis | flag_reset
    print "flags:" , flags
    # send write pattern. 
    # interface, command, timeout
    dev.write( 1, chr(command) + chr(pinNum) + chr(flags) 
        + chr(carrierInterval) + chr(carrier_duty_cycle_on) + chr(carrier_duty_cycle_off) 
        + chr(num_repeats)
        + durationsData
        + chr(0) + chr(0), # end of command data
        10)

elif cmd == 'RP' or cmd == 'RPD' or cmd == 'RA':
    allresults = []
    # read the ADC. COMMAND_IO_READ_ADC
    # send command on EP_OUT
    if cmd == 'RA' :
        command = 0x14
        numSamples = 500
        sampleInterval = 0 #uS
        minStartingValue = 0 # value that should be reached before we start sampling. Useful for manual triggering like pressing a remote key
        chnls = 0 # TODO : bitmap of ADC channels to use
        flag_is_debug = 0 # 2 is ON, intervals are in ms
        flags = flag_is_debug
        numBytesInPacket = 64
        numBytesInSample = 2
        reservedBytesInPacket = 2
        samplesPerPacket = (numBytesInPacket - reservedBytesInPacket)/numBytesInSample
        packetsToRead = int( math.ceil( float(numSamples)/samplesPerPacket))
        # Min time between samples taken by uC is 11*TAD+TACQ = 21us
        # From comparing a wave of 100 ON/200 OFF, it seems around 8-10 uS is spent on code exec. This might be added as well
        codeExecTime = 9 #uS
        minSamplingTime = 21 #uS
        effSamplingInterval = sampleInterval + minSamplingTime + codeExecTime
        print "numSamples:%d, sampleInterval:%d, samplesPerPacket:%d, packetsToRead:%d" % (numSamples, sampleInterval, samplesPerPacket, packetsToRead)
        dev.write( 1, chr(command) + chr(chnls) + chr(flags) + chr(numSamples >>8) + chr(numSamples & 0x00FF) + chr(sampleInterval >>8) + chr(sampleInterval & 0x00FF)
            + chr(minStartingValue >>8) + chr(minStartingValue & 0x00FF), 
            100)

    elif cmd == 'RP' :
        # Read PIN
        command = 0x12
        pinNum = 0xd2
        flag_idleState = 1 # Hi/LO. We wait for state to change from this. Useful for manual input actions like pressing a remote.
        flag_is_millis = 0 # 4 is ON, intervals are in ms
        flag_is_debug = 2 # 2 is ON, intervals are in ms
        flags = flag_idleState | flag_is_debug | flag_is_millis 
        numSamples = 1000
        sampleInterval = 100 #uS
        numBytesInPacket = 64
        numBytesInSample = 1
        reservedBytesInPacket = 2
        samplesPerPacket = (numBytesInPacket - reservedBytesInPacket)/numBytesInSample
        packetsToRead = int( math.ceil( float(numSamples)/samplesPerPacket))
        # No ADC here, so no acquisition time
        codeExecTime = 0 #uS
        minSamplingTime = 0 #uS
        effSamplingInterval = sampleInterval + minSamplingTime + codeExecTime
        print "numSamples:%d, sampleInterval:%d, samplesPerPacket:%d, packetsToRead:%d" % ( numSamples, sampleInterval, samplesPerPacket, packetsToRead)
        dev.write( 1, chr(command) + chr(pinNum) + chr(flags) + chr(numSamples >>8) + chr(numSamples & 0x00FF) + chr(sampleInterval >>8) 
            + chr(sampleInterval & 0x00FF), 10)

    elif cmd == 'RPD' :
        # Read PIN DURATIONS
        command = 0x13
        pinNum = 0xd2
        flag_idleState = 1 # Hi/LO. We wait for state to change from this. Useful for manual input actions like pressing a remote.
        flag_is_millis = 0 # 4 is ON, intervals are in ms
        flag_is_debug = 0 # 2 is ON, intervals are in ms
        flags = flag_idleState | flag_is_debug | flag_is_millis 
        numSamples = 200
        sampleInterval = 5 #uS
        numBytesInPacket = 64
        numBytesInSample = 2
        reservedBytesInPacket = 2
        samplesPerPacket = (numBytesInPacket - reservedBytesInPacket)/numBytesInSample
        packetsToRead = int( math.ceil( float(numSamples)/samplesPerPacket))
        # No ADC here, so no acquisition time
        codeExecTime = 9 #uS
        minSamplingTime = 1 #uS
        effSamplingInterval = sampleInterval + minSamplingTime + codeExecTime
        scalingFactor = float(sampleInterval + codeExecTime)/sampleInterval
        print "numSamples:%d, sampleInterval:%d, samplesPerPacket:%d, packetsToRead:%d" % ( numSamples,sampleInterval, samplesPerPacket, packetsToRead)
        dev.write( 1, chr(command) + chr(pinNum) + chr(flags) + chr(numSamples >>8) + chr(numSamples & 0x00FF) + chr(sampleInterval >>8) 
            + chr(sampleInterval & 0x00FF), 10)

    # read on EP_IN
    # seems that the data-size we request should be in multiples of actual data size sent.
    # e.g. requesting to read 100, when size is 64 can cause an overflow error.
    # but 128 might work, in that case result of 2 requests will be clubbed together.
    # Seems to be connected to timeout as well. 64 + 10ms works as expected
    # Note that timeout is also directly related with sampling interval.
    # Min time between samples taken by uC is 11*TAD+TACQ = 21us
    try :
        for loop in range( packetsToRead ) :
            ret = dev.read( 129, 64, 10000) # ep, num-bytes, timeout
            numValidBytes = int(ret[1])
            print "Received cmd:%d, packetNum:%d, numValidBytes:%d, data[0]:%d, scalingFactor:%f" % (ret[0], loop, ret[1], (ret[2] << 8 | ret[3]), scalingFactor )
            
	    if numBytesInSample == 2 :
		for i in xrange( reservedBytesInPacket, numValidBytes -1, 2):
		    allresults.append( int(round( (ret[i] << 8 | ret[i+1]) * scalingFactor )) )
            elif numBytesInSample == 1 :
            	for i in xrange( reservedBytesInPacket, numValidBytes, 1):
            	    allresults.append( int(round(ret[i] * scalingFactor)) )
            		
            allresults.append( -1) # End of packet indicator. There will be sampling discontinuity at this point, since time is spent in sending the packet.
            # for binary file use : allresults.extend( ret[reservedBytesInPacket : numValidBytes-1] )
    except Exception as error :
        # trap error so that earlier results can be processed after a timeout. This may be useful in cases where we have kept numSamples
        # on the higher side, and since we don't get those many in reality, a timeout may occur.
        print "Error:" , error, sys.exc_info()

    print "allresults:", allresults
    # save to file
    # for binary
    # newFile = open ("results.bi", "wb")
    # newFile.write( bytearray(allresults))


    with open( "../hid_io_results_" + cmd + ".js", 'wb') as outfile:
        outfile.write( "var sampleInterval = " + str(effSamplingInterval) + ";")
        outfile.write( "var hid_io_data = ")
        json.dump( allresults, outfile)
