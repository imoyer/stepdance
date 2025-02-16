# Universal Personality Programmer
# Version 3.0
# Modified 7/30/19 for the UPP Console
#
# pyGestalt Virtual Node

from pygestalt import nodes, core, packets, utilities
from pygestalt.utilities import notice
import math

#--- MCU LIBRARY ---
global globalMCUList 
globalMCUList = [] #a globally accessible list that contains known microcontrollers

def registerNewMCU(microcontrollerObject):
    """Adds a microcontroller to the global microcontroller list."""
    global globalMCUList
    globalMCUList += [microcontrollerObject]
    return

#--- GENERAL CLASSES ---
class programmingSocket(object):
    """A definition for the programming socket thru which the microcontroller interfaces to the UPP.
    
    The purpose of the programming socket is to interface internal pins on the UPP to pins on the target microcontroller.
    """
    def __init__(self, name = "", pinCount = None, customMapping = None, signalPinMap = {}):
        """Initializes a new programming socket.
        
        name -- the name of the programming socket, to be used in reporting to the user.
        pinCount -- the number of pins on the socket
        customMapping -- an ordered list whose indices are pins on the socket, and the values at those indices are internal pins on the UPP
        signalPinMap -- a dictionary containing {signalName:internalPinNumber} pairs, used to hardwire signals to pins on the UPP (e.g. thru a 6-pin standard header):
            signalName -- the string name of a signal e.g. vcc, mosi, etc...
            internalPinNumber -- the internal pin number on the UPP. The first 20 pins map to pin numbers on the onboard 20-pin ZIF socket, additional numbers assigned to shield headers.
        """
        self.name = name
        self.pinCount = pinCount
        self.customMapping = customMapping
        if customMapping != None and len(customMapping) != pinCount:   #make sure enough pins have been defined if a custom mapping is provided
            print("ERROR: The provided custom mapping is incomplete. Only " + str(len(customMapping)) + "/" + str(pinCount) + " pins have been defined!")
            raise Exception
        self.signalPinMap = signalPinMap
    
    def internalPinNumber(self, socketPinNumber):
        """Returns the UPP internal pin number for a provided socket pin number.
        
        If no custom mapping was provided on instantiation of the programmingSocket, this will simply pass thru the number. Otherwise the mapping will be applied.
        """
        if self.customMapping == None: #no custom mapping, pass thru the pin number
            return socketPinNumber
        else:
            return self.customMapping[socketPinNumber]
    
    
    def socketPosition(self, chip, pinNumber):
        """Returns the position of a chip pin transposed onto the socket.
        
        chip -- the target microcontroller object
        pinNumber -- the desired pin number of the chip
        
        This follows the standard convention where Pin 1 is in the lower left of both the socket and the chip.
        IT IS ASSUMED THAT THE CHIP IS LEFT-JUSTIFIED ON THE SOCKET.
        """
        chipPinCount = chip.pinCount    #the number of pins on the chip
        #returns the socket position for a given pin
        chipMidPoint = (chipPinCount+1) / 2.0 #no zero pin
        
        if pinNumber < chipMidPoint:
            return self.internalPinNumber(pinNumber)
        else:
            return self.internalPinNumber(self.pinCount - (chipPinCount - pinNumber))    
    
    
    def getSignalPin(self, signal, chip = None):
        """Returns the internal pin on the UPP that is connected to the requested signal.

        signal -- the string signal name        
        chip -- the target microcontroller object. If not provided, only mapped signals can be looked up
        
        If the signal is hardwired - as indicated by a presence in self.signalPinMap - the internal pin number stored in this dictionary will be used.
        Otherwise, the signal will be referenced into the target microcontroller to obtain a chip pin number, which will then be transposed by the socket.
        """
        if signal in self.signalPinMap:
            return self.signalPinMap[signal]
        else:
            if chip == None:
                print("Error: signal is not mapped on interface. Please provide a chip in order to reference MCU signal names.")
                return False
            if self.pinCount == None:
                print("Error: this socket interface (" + str(self.name) + ") does not support MCU pin name mapping.")
                return False
            if hasattr(chip, signal):
                return self.socketPosition(chip, getattr(chip, signal))
            elif signal in chip.IOPins:
                return self.socketPosition(chip, chip.IOPins[signal])
            else:
                print("Error: the provided microcontroller (" + chip.name + ") does not have the requested signal '" + str(signal) + "'")
                return False
        
    
    
    def getSignalPinMap(self, chip, signals = ['vcc', 'gnd', 'mosi', 'miso', 'sck', 'reset']):
        """Returns a dictionary containing a mapping between signals on the target microncroller and internal pins on the UPP.
        
        chip -- the target microcontroller object
        signals -- a list of signals to return in the mapping. A standard set is provided as default.
        """
        return {signal:self.getSignalPin(signal, chip) for signal in signals}
        

class microcontroller(object):
    """A microncontroller definition containing all of the information needed to identify and program a particular microcontroller."""
    def __init__(self, name = None, signature = None, pageSize = None, autoRegister = True, vcc = None, gnd = None, mosi = None, miso = None, sck = None, reset = None, pinCount = None, **IOPins):
        """Initializes a new target chip.
        
        name -- a string name by which to identify the chip
        signature -- the three-byte signature provided by atmel that uniquely identifies the chip type and version (e.g. attiny44 vs attiny84 vs ...)
        pageSize -- the memory programming page size in bytes. Pages get sequentiall filled and the committed to flash memory.
        autoRegister -- if False, the microcontroller won't automatically be registered with the global list
        vcc -- the package pin number of the +V pin. 'Package pin number' means the pin number relative to the chip's package, not the programmer socket
        gnd -- the package pin number of the ground pin
        mosi -- the package pin number of the SPI MOSI pin
        miso -- the package pin number of the SPI MISO pin
        sck -- the package pin number of the SPI SCK pin
        reset -- the package pin number of reset pin
        pinCount -- the number of pins in the package
        IOPins -- any additional pins to be mapped
        """
        self.name = name
        self.signature = signature
        self.pageSize = pageSize    #page size in bytes
        self.vcc = vcc
        self.gnd = gnd
        self.mosi = mosi
        self.miso = miso
        self.sck = sck
        self.reset = reset
        self.pinCount = pinCount
        self.IOPins = IOPins
    
        if autoRegister: registerNewMCU(self) #registers the MCU
        
    def __str__(self):
        return self.name 


#standard ATTiny configurations
eightPinATTinyConfig = {'pinCount':8, 'gnd':4, 'vcc':8, 'sck':7, 'miso':6, 'mosi':5, 'reset':1, 'PB5':1, 'PB3':2, 'PB4':3, 'PB0':5, 'PB1':6, 'PB2':7}
fourteenPinATTinyConfig = {'pinCount':14, 'gnd': 14, 'vcc': 1, 'sck': 9, 'miso':8, 'mosi':7, 'reset':4, 'PB0':2, 'PB1':3, 'PB3':4, 'PB2':5, 'PA7':6, 'PA6':7, 'PA5':8, 'PA4':9, 'PA3':10, 'PA2':11, 'PA1':12, 'PA0':13}
twentyPinATTinyConfig = {'pinCount':20, 'gnd': 10, 'vcc': 20, 'sck': 19, 'miso': 18, 'mosi': 17, 'reset': 1, 'PA2':1, 'PD0':2, 'PD1':3, 'PA1':4, 'PA0':5, 'PD2':6, 'PD3':7, 'PD4':8, 'PD5':9, 'PD6':11, 'PB0':12, 'PB1':13, 'PB2':14, 'PB3':15, 'PB4':16, 'PB5':17, 'PB6':18, 'PB7':19}

#generic attiny chips
attiny_8pin = microcontroller(name = "attiny_8pin", autoRegister = False, **eightPinATTinyConfig)
attiny_14pin = microcontroller(name = "attiny_14pin", autoRegister = False, **fourteenPinATTinyConfig)
attiny_20pin = microcontroller(name = "attiny_20pin", autoRegister = False, **twentyPinATTinyConfig)

attiny13 = microcontroller(**eightPinATTinyConfig)
attiny15 = microcontroller(**eightPinATTinyConfig)
attiny25 = microcontroller(name = "ATTiny25", signature = 0x08911E, pageSize = 32, **eightPinATTinyConfig)
attiny45 = microcontroller(name = "ATTiny45", signature = 0x06921E, pageSize = 64, **eightPinATTinyConfig)
attiny85 = microcontroller(name = "ATTiny85", signature = 0x0B931E, pageSize = 64, **eightPinATTinyConfig)
attiny24 = microcontroller(name = "ATTiny24", signature = 0x0B911E, pageSize = 32, **fourteenPinATTinyConfig)
attiny44 = microcontroller(name = "ATTiny44", signature = 0x07921E, pageSize = 64, **fourteenPinATTinyConfig)
attiny84 = microcontroller(name = "ATTiny84", signature = 0x0C931E, pageSize = 64, **fourteenPinATTinyConfig)
attiny2313 = microcontroller(name = "ATTiny2313", signature = 0x0A911E, **twentyPinATTinyConfig)
attiny4313 = microcontroller(name = "ATTiny4313", signature = 0x0D921E, **twentyPinATTinyConfig)
atmega328 = microcontroller(name = "ATMega328", signature = 0x0F951E, pageSize = 128)


class virtualNode(nodes.gestaltVirtualNode):
    """The Universal Personality Programmer virtual node."""

    def init(self):
        """Initialization routine for the virtual node."""
        
        #define sockets
        self.zifSocket = programmingSocket(name = "ZIF SOCKET", pinCount = 20, signalPinMap = {'doppelscl':19, 'doppelsda':18}) #onboard 20-pin ZIF socket
        self.externalProgCable = programmingSocket(name = "EXTERNAL ISP CABLE", signalPinMap = {'gnd': 1, 'vcc':20, 'sck':23, 'miso': 22, 'mosi':21, 'reset':28, 'doppelscl':25, 'doppelsda':24}) #these are mapped to the shield pins. gnd and vcc are dummies as are hardwired on the external prog. interface
        
        self.socketInterfaces = [self.zifSocket, self.externalProgCable] #list of all interfaces to be iterated thru during chip discovery process
        self.activeSocketInterface = self.zifSocket #this is the currently used interface
        
        #generic attiny chips
        self.prototypeChips = [attiny_8pin, attiny_14pin, attiny_20pin]    #used for searching pin configuration, references module-level definitions (at top of file)
        
    
    
    def initPackets(self):
        """Define all packets."""
        self.configureChipRequestPacket = packets.template('configureChipRequest',
                                                            packets.unsignedInt('gnd', 1),
                                                            packets.unsignedInt('vcc', 1),
                                                            packets.unsignedInt('sck',1),
                                                            packets.unsignedInt('miso',1),
                                                            packets.unsignedInt('mosi',1),
                                                            packets.unsignedInt('reset',1))

        self.progInitResponsePacket = packets.template('programmerInitializationResponse',
                                                        packets.unsignedInt('responseCode', 1))
        
        self.readSignatureResponsePacket = packets.template('readSignatureResponse',
                                                            packets.unsignedInt('signatureBytes', 4))
        
        self.readFusesResponsePacket = packets.template('readFusesResponse',
                                                        packets.unsignedInt('lfuse', 1),
                                                        packets.unsignedInt('hfuse', 1),
                                                        packets.unsignedInt('efuse', 1))
            
        self.writeFusesRequestPacket = packets.template('writeFusesRequest',
                                                        packets.unsignedInt('lfuse', 1),
                                                        packets.unsignedInt('hfuse', 1),
                                                        packets.unsignedInt('efuse', 1))
        
        self.loadDataRequestPacket = packets.template('loadDataRequest',
                                                        packets.unsignedInt('dataSize',1),
                                                        packets.unsignedInt('byteBaseAddress', 2),
                                                        packets.pList('data'))        
            
        self.writePageRequestPacket = packets.template('writePageRequest',
                                                        packets.unsignedInt('bytePageAddress', 2))

        self.readDataRequestPacket = packets.template('readDataRequest',
                                                        packets.unsignedInt('dataSize',1),
                                                        packets.unsignedInt('byteBaseAddress', 2))
        
        self.readDataResponsePacket = packets.template('readDataResponse',
                                                        packets.pList('data'))
        
        self.writeEEPROMRequestPacket = packets.template('writeEEPROMRequest',
                                                            packets.unsignedInt('address',1),
                                                            packets.unsignedInt('data',1))
        
        self.readEEPROMRequestPacket = packets.template('readEEPROMRequest',
                                                            packets.unsignedInt('address',1))
        
        self.readEEPROMResponsePacket = packets.template('readEEPROMResponse',
                                                            packets.unsignedInt('data',1))
        
        self.lastKnownSignatureResponsePacket = packets.template('lastKnownSignatureResponse',
                                                                    packets.unsignedInt('signatureBytes', 4))
        
        self.readInternalPinRequestPacket = packets.template('readInternalPinRequest',
                                                            packets.unsignedInt('pin',1))
        
        self.readInternalPinResponsePacket = packets.template('readInternalPinResponse',
                                                            packets.unsignedInt('value',1))
        
        self.configureDoppelRequestPacket = packets.template('configureDoppelRequest',
                                                                packets.unsignedInt('sclPin',1),
                                                                packets.unsignedInt('sdaPin',1))
        
        self.sendDoppelMessageRequestPacket = packets.template('sendDoppelMessageRequest',
                                                                packets.unsignedInt('packetLength',1),
                                                                packets.unsignedInt('address', 1),
                                                                packets.pList('message'))
        
        self.sendDoppelMessageResponsePacket = packets.template('sendDoppelMessageResponse',
                                                                packets.unsignedInt('isConfigured',1))
       
        self.enableDoppelReceiverResponsePacket = packets.template('enableDoppelReceiverResponse',
                                                                    packets.unsignedInt('isConfigured',1))
        
        self.readDoppelMessagesResponsePacket = packets.template('readDoppelMessageResponse',
                                                                    packets.pList('message'))
                           
    def initPorts(self):
        """Bind ports to funtions and packet templates."""
        self.bindPort(port = 10, outboundFunction = self.configureChipRequest, outboundTemplate = self.configureChipRequestPacket)
        
        #programming
        self.bindPort(port = 11, outboundFunction = self.progInitRequest, inboundTemplate = self.progInitResponsePacket)
        self.bindPort(port = 12, outboundFunction = self.readSignatureRequest, inboundTemplate = self.readSignatureResponsePacket)
        self.bindPort(port = 13, outboundFunction = self.readFusesRequest, inboundTemplate = self.readFusesResponsePacket)
        self.bindPort(port = 14, outboundFunction = self.writeFusesRequest, outboundTemplate = self.writeFusesRequestPacket)
        self.bindPort(port = 15, outboundFunction = self.eraseChipRequest)
        self.bindPort(port = 16, outboundFunction = self.loadDataRequest, outboundTemplate = self.loadDataRequestPacket)
        self.bindPort(port = 17, outboundFunction = self.writePageRequest, outboundTemplate = self.writePageRequestPacket)
        self.bindPort(port = 18, outboundFunction = self.readDataRequest, outboundTemplate = self.readDataRequestPacket, inboundTemplate = self.readDataResponsePacket)
        self.bindPort(port = 19, outboundFunction = self.writeEEPROMRequest, outboundTemplate = self.writeEEPROMRequestPacket)
        self.bindPort(port = 20, outboundFunction = self.readEEPROMRequest, outboundTemplate = self.readEEPROMRequestPacket, inboundTemplate = self.readEEPROMResponsePacket)
        self.bindPort(port = 21, outboundFunction = self.runChipRequest)
        self.bindPort(port = 22, outboundFunction = self.lastKnownSignatureRequest, inboundTemplate = self.lastKnownSignatureResponsePacket)
        self.bindPort(port = 23, outboundFunction = self.readInternalPinRequest, outboundTemplate = self.readInternalPinRequestPacket, inboundTemplate = self.readInternalPinResponsePacket)
        
        #doppel interface
        self.bindPort(port = 30, outboundFunction = self.configureDoppelRequest, outboundTemplate = self.configureDoppelRequestPacket)
        self.bindPort(port = 31, outboundFunction = self.enableDoppelReceiverRequest, inboundTemplate = self.enableDoppelReceiverResponsePacket)
        self.bindPort(port = 32, outboundFunction = self.sendDoppelMessageRequest, outboundTemplate = self.sendDoppelMessageRequestPacket, inboundTemplate = self.sendDoppelMessageResponsePacket)
        self.bindPort(port = 33, outboundFunction = self.readDoppelMessagesRequest, inboundTemplate = self.readDoppelMessagesResponsePacket)
        self.bindPort(port = 34, outboundFunction = self.resetDoppelMessageBufferRequest)
        self.bindPort(port = 35, outboundFunction = self.enterFreeRunningModeRequest)

#--UTILITY FUNCTIONS-----
    
    def readChipPin(self, pin):
        """Reads the state of a pin on the target microcontroller.
        
        pin -- either a numerical pin number on the target MCU, or a named signal. If a named signal is used, it must be defined by the chip or the socket interface.
        
        Returns the logical value of the pin, either 0 or 1.
        """
        chip = self.identifyChipFromSignature(self.lastKnownSignatureRequest())
        if type(pin) == str:
            internalPinNumber = self.activeSocketInterface.getSignalPin(pin, chip) #pin is provided as a signal name
        else:
            internalPinNumber = self.activeSocketInterface.socketPosition(chip, pin) #pin is assumed to be an integer pin number on the chip
        
        socketPinState = self.readInternalPinRequest(internalPinNumber)
        return socketPinState

    def identifyChipFromSignature(self, signature):
        """Returns the microcontroller object corresponding to the provided signature, or False if none is found.
        
        The module-level global MCU list is used as the corpus for the lookup.
        """
        global globalMCUList
        for chip in globalMCUList:
            if chip.signature == signature:
                return chip
        print("NO CHIP WITH SIGNATURE " + str(signature) + " IN DATABASE")
        return False

    def identifyChipConfiguration(self):
        """Identifies the active socket interface and chip configuration.
        
        This function varies the socket interface and prototype chip configuration until communication is established with the target MCU.
        
        Sets the virtual node's active socket interface to the last successfully used interface.
        Returns the prototype microcontroller object.
        """
        
        for socketInterface in self.socketInterfaces: #iterate over socket interfaces
            self.activeSocketInterface = socketInterface
            for chip in self.prototypeChips:    #iterate over known chip prototypes
                self.configureChipRequest(chip)     #configure programmer for chip
                if self.progInitRequest(): return chip
        return False
    
    def identifyChip(self):
        """Automatically determines the target microcontroller using the module-level global MCU list as the lookup corpus.
        
        Returns the target microcontroller object.
        """
        global globalMCUList
        chipPrototype = self.identifyChipConfiguration()    #try to figure out chip type
        if chipPrototype:   #was able to communicate
            chipSignature = self.readSignatureRequest()
            for chip in globalMCUList:  #search for signature in known chip list
                if chipSignature == chip.signature:
                    notice(self, chip.name + " IS CURRENTLY IN "+ self.activeSocketInterface.name + ".")
                    if self.progInitRequest():
                        notice(self, chip.name + " IS READY FOR PROGRAMMING")
                    else:
                        notice(self, chip.name + " COULD NOT BE INITIALIZED FOR PROGRAMMING")
                    return chip
            print("UNABLE TO IDENTIFY CHIP IN " + self.activeSocketInterface.name + " WITH SIGNATURE " + str(chipSignature))
            return False
        else:
            print("COULD NOT COMMUNICATE WITH CHIP IN SOCKET")
            print("MAKE SURE CHIP IS INSTALLED CORRECTLY (LOWER LEFT CHIP AND SOCKET PINS CONNECTED, WITH PIN 1 MARK ON LOWER LEFT)")
            return False
    
    def setFuses(self, lfuse = None, hfuse = None, efuse = None):
        """Sets all fuse bytes on the target microcontroller.
        
        lfuse -- the lfuse byte.
        hfuse -- the hfuse byte.
        efuse -- the efuse byte.
        Any fuse byte inputs not provided will remain unchanged.
        
        Returns the result of teh writeFusesRequest call, which is False or True depending on success of the operation.
        """
        currentFuses = self.readFusesRequest()
        if lfuse == None: lfuse = currentFuses['lfuse']
        if hfuse == None: hfuse = currentFuses['hfuse']
        if efuse == None: efuse = currentFuses['efuse']
        return self.writeFusesRequest(lfuse, hfuse, efuse)
        
    def program(self, filename = None):
        """Loads a program onto the target microcontroller.
        
        filename -- a path to the target program, which should be in intel hex format.
        """
        
        #identify chip
        chip = self.identifyChip()
        if not chip:    #could not identify chip
            return False
        pageSize = chip.pageSize #get chip page size
        
        #initialize programming mode
        if not self.progInitRequest():
            notice(self, "UNABLE TO INITIALIZE PROGRAMMING MODE")
            return False
        
        #erase program memory
        if not self.eraseChipRequest():
            notice(self, "UNABLE TO ERASE CHIP")
            return False
        
        #parse input file
        parser = utilities.intelHexParser() #Intel Hex Format Parser Object
        parser.openHexFile(filename)
        parser.loadHexFile()
        pages = parser.returnPages(pageSize)    #get program data, split into pages according to device page size.

        #write pages
        for page in pages:
            pageNumber = page[0][0] #address of first byte
            pageData = [addressBytePair[1] for addressBytePair in page]
            self.loadDataRequest(pageNumber, pageData)
            self.writePageRequest(pageNumber)
            notice(self, "WROTE PAGE "+ str(pageNumber))
            
        #verify pages
        for page in pages:
            pageNumber = page[0][0] #address of first byte
            pageData = [addressBytePair[1] for addressBytePair in page]
            verifyData = self.readDataRequest(pageNumber, pageSize)
            if pageData == verifyData:
                notice(self, "VERIFIED PAGE "+ str(pageNumber) + "!")
            else:
                notice(self, "VERIFY ERROR IN PAGE " + str(pageNumber))
                print(pageData)
                print(verifyData)
                return False
        
        return True                     

        
#---UTILITY CLASSES-----
    class fuseDict(dict):
        """Displays fuses using hex notation."""
        def __str__(self):
            return str(dict((key, hex(self[key])) for key in self))


#---SERVICE ROUTINES-----
    class resetDoppelMessageBufferRequest(core.actionObject):
        """Resets the doppel message buffer."""
        def init(self):
            if self.transmitUntilResponse():
                return True
            else:
                notice(self.virtualNode, 'got no response to doppel message buffer reset request')
                return False

    class readDoppelMessagesRequest(core.actionObject):
        """Returns all doppel messages stored in the UPP doppel relay buffer.
        
        Messages are returned as a message list: [[message1....], [message2...], [messageN....]], where each message is a list of received bytes.
        """
        def init(self):
            if self.transmitUntilResponse():
                rawMessages = self.getPacket()['message']
                messages = []
                while True:
                    newMessage, rawMessages = self.extractNextDoppelMessage(rawMessages)
                    if newMessage:
                        messages += [newMessage]
                    else:
                        break
                return messages
            else:
                notice(self.virtualNode, 'got no response to doppel read message request')
                return False

        def extractNextDoppelMessage(self, packet):
            """Returns the next doppel message in an input packet, or False if none are avaliable.
             
             packet -- a list in the format [message1Length, message1Byte0, message1Byte1..., message2Length, message2Byte0, message2Byte1,...]
             
             Doppel messages are captured in a buffer on the UPP, and then asynchronously relayed on request to the virtual node. These messages are
             stored in the buffer in the format shown above. This function will extract the first message in the packet by reading the message length from the
             first byte, and then slicing the packet to extract the message. Only the message bytes will be returned; the length byte will be discarded.
             
             returns message, remainder
                 message -- the message bytes of the first message
                 remainder -- the remainder of the packet trailing the extracted message
            """
            try:
                messageLength = packet[0] #first byte is the length of the message
                message = packet[1:messageLength+1]
                if len(message) != messageLength: return False, False
                remainder = packet[messageLength+1::]
                return message, remainder
            except:
                return False, False

    class sendDoppelMessageRequest(core.actionObject):
        def init(self, address, message = []):
            self.setPacket(packetLength = len(message)+1, address = address, message = message)
            if self.transmitUntilResponse():
                if self.getPacket()['isConfigured']:
                    return True
                else:
                    notice(self.virtualNode, 'unable to transmit because doppel network port not configured')
                    return False
            else:
                notice(self.virtualNode, 'got no response to doppel transmit request.')
                return False
                
    class enableDoppelReceiverRequest(core.actionObject):
        def init(self):
            if self.transmitUntilResponse():
                if self.getPacket()['isConfigured']:
                    return True
                else:
                    notice(self.virtualNode, 'unable to enable receiver because doppel network port not configured')
                    return False
            else:
                notice(self.virtualNode, 'got no response to doppel receiver enable request.')
                return False

    class configureDoppelRequest(core.actionObject):
        """Configures the UPP to communicate with a chip using the doppel protocol."""
        def init(self, sclPin = None, sdaPin = None):
            """Initialization for configureDoppelRequest.
            
            sclPin -- the internal UPP pin number of the doppel scl signal, or the signal name on the target mcu.
            sdaPin -- the internal UPP pin number of the doppel sda signal, or the signal name on the target mcu.
            
            Note that the internal UPP pin number is the same as the onboard ZIF socket number for pins 1-20. Other pins map to the shield headers.
            If scl or sda pins are not provided, the default values provided by the active socket interface will be used.
            """
            chip = self.virtualNode.identifyChipFromSignature(self.virtualNode.lastKnownSignatureRequest())
            if sclPin == None or sdaPin == None: #incomplete pin information provided, so use defaults built into the socket signal map.
                sclSocketPin = self.virtualNode.activeSocketInterface.getSignalPin('doppelscl')
                sdaSocketPin = self.virtualNode.activeSocketInterface.getSignalPin('doppelsda')
                sclPin = sclSocketPin #reassign so that reports corectly
                sdaPin = sdaSocketPin
            else: #pin info was provided
                if type(sclPin) == int: #provided scl pin is an integer - assumed to be an UPP internal pin number
                    sclSocketPin = sclPin
                elif type(sclPin) == str: #provided scl pin is a string -- assumed to be a signal name on the chip
                    sclSocketPin = self.virtualNode.activeSocketInterface.getSignalPin(sclPin, chip)
                else:
                    print("Doppel SCL pin must be provided either as an internal pin on the UPP, or a pin name on the target MCU.")
                    return False
                if type(sdaPin) == int: #provided sda pin is an integer -- assumed to be an UPP internal pin number
                    sdaSocketPin = sdaPin
                elif type(sdaPin) == str: #provided sda pin is a string -- assumed to be a signal name on the chip
                    sdaSocketPin = self.virtualNode.activeSocketInterface.getSignalPin(sdaPin, chip)
                else:
                    print("Doppel SDA pin must be provided as either an internal pin on the UPP, or a pin name on the target MCU.")
                    return False

            self.setPacket(sclPin = sclSocketPin, sdaPin = sdaSocketPin)
            if self.transmitUntilResponse():
                print("Current Doppel Network Internal UPP Pin Assignments:")
                print("SCL: " + str(sclPin))
                print("SDA: " + str(sdaPin))
                return True
            else:
                notice(self.virtualNode, "got no response to doppel configuration request.")
                return False

    class configureChipRequest(core.actionObject):
        def init(self, chipObject):
            signalMap = self.virtualNode.activeSocketInterface.getSignalPinMap(chipObject)
            self.setPacket(**signalMap)
            if self.transmitUntilResponse():
                return True
            else: 
                notice(self.virtualNode, "got no response to status request")
                return False
                
    class progInitRequest(core.actionObject):
        #initializes programming mode
        def init(self):
            if self.transmitUntilResponse():
                responseCode = self.getPacket()['responseCode']
                return (responseCode == 83)
            else:
                return False
                
    class readSignatureRequest(core.actionObject):
        #reads atmel device signature
        def init(self):
            if self.transmitUntilResponse():
                return self.getPacket()['signatureBytes'] & 0xFFFFFF    #return only last three bytes
            else:
                return False
 
    class lastKnownSignatureRequest(core.actionObject):
        #returns the signature last read from the chip
        def init(self):
            if self.transmitUntilResponse():
                return self.getPacket()['signatureBytes'] & 0xFFFFFF    #return only last three bytes
            else:
                return False
                             
    class readFusesRequest(core.actionObject):
        #reads fuse settings
        def init(self):
            if self.transmitUntilResponse():
                return self.virtualNode.fuseDict(self.getPacket())    #return fuses
            else:
                return False
                 
    class writeFusesRequest(core.actionObject):
        #writes fuse settings
        def init(self, lfuse, hfuse, efuse):
            self.setPacket(lfuse = lfuse, hfuse = hfuse, efuse = efuse)
            if self.transmitUntilResponse():
                return True
            else:
                return False

    class eraseChipRequest(core.actionObject):
        #erases program memory
        def init(self):
            if self.transmitUntilResponse():
                return True
            else:
                return False
                
    class loadDataRequest(core.actionObject):
        #loads data into page buffer
        def init(self, address, data):
            self.setPacket(dataSize = len(data), byteBaseAddress = address, data = data)
            if self.transmitUntilResponse():
                return True
            else:
                return False

    class writePageRequest(core.actionObject):
        #write page to program memory
        def init(self, address):
            self.setPacket(bytePageAddress = address)
            if self.transmitUntilResponse():
                return True
            else:
                return False
                
    class readDataRequest(core.actionObject):
        #reads data from page
        def init(self, address, numBytes):
            self.setPacket(byteBaseAddress = address, dataSize = numBytes)
            if self.transmitUntilResponse():
                return self.getPacket()['data']
            else:
                return False

    class readEEPROMRequest(core.actionObject):
        #reads data from page
        def init(self, address):
            self.setPacket(address = address)
            if self.transmitUntilResponse():
                return self.getPacket()['data']
            else:
                return False

    class writeEEPROMRequest(core.actionObject):
        #reads data from page
        def init(self, address, data):
            self.setPacket(address = address, data = data)
            if self.transmitUntilResponse():
                verifyValue = self.virtualNode.readEEPROMRequest(address)
                if data == verifyValue:
                    notice(self.virtualNode, "WROTE " + str(data) + " TO EEPROM ADDRESS " + str(address))
                    return True
                else:
                    notice(self.virtualNode, "ATTEMPTED TO WRITE " + str(data) + " TO ADDRESS " + str(address) + ", BUT READ BACK " + str(verifyValue))
                    return False
            else:
                return False
    
    class readInternalPinRequest(core.actionObject):
        #reads an internal pin on the UPP.
        def init(self, pin):
            #pin -- corresponds to the socket pin for pins on the onboard ZIF socket
            self.setPacket(pin = pin)
            if self.transmitUntilResponse():
                return self.getPacket()['value']
            else:
                notice(self.virtualNode, "UNABLE TO READ SOCKET PIN "+ str(pin))
                return False
    
    class runChipRequest(core.actionObject):
        #brings the chip out of reset and sets all socket pins except VCC/GND to inputs
        def init(self):
            if self.transmitUntilResponse():
                notice(self.virtualNode, "TARGET CHIP APPLICATION NOW RUNNING")
                return True
            else:
                notice(self.virtualNode, "UNABLE TO START TARGET CHIP APPLICATION")
                return False
            
    class enterFreeRunningModeRequest(core.actionObject):
        #returns the UPP to free-running mode
        def init(self):
            if self.transmitUntilResponse():
                notice(self.virtualNode, "UPP IS NOW IN FREE-RUNNING MODE")
                return True
            else:
                notice(self.virtualNode, "UNABLE TO BRING UPP INTO FREE-RUNNING MODE")
                return False