# Remote Procedure Call Interface
# Stepdance
# A creative motion control platform
# 
# (C) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost, Emilie Yu

import json
import serial

class rpc(object):
    def __init__(self, port_name):
        self.serial_port = serial.Serial(port_name, 4000000, timeout=0.1)
        self.index = {}
        self.load_index()
        print(self.index)
    
    def load_index(self):
        '''Loads an index of attributes from the remote system.
        
        First requests an index from a remote stepdance system, and then processes the index by identifying parent objects
        and classifying them. This helps with our __getattr__ lookup system, to know when we need to request and return
        (or set) a value from the remote system.
        '''
        raw_index = self.remote_call("__index__")['return']
        self.index = {}
        for key in raw_index:
            self.index.update({key:raw_index[key]}) # transfer the full attribute name to the index
            elements = key.split('.')
            path = ''
            for element in elements[:-1]:
                path += element
                self.index.update({path : "object"})
                path += '.'

    def remote_call(self, name, parameter_list = []):
        message = json.dumps({"name":name, "args":parameter_list})
        self.serial_port.write(message.encode('utf-8') + b'\n')
        return self.read_response()
    
    def read_response(self):
        return json.loads(self.serial_port.readline())
    
    def __getattr__(self, name):
        trace = attribute_trace(self)
        return trace.__getattr__(name)



class attribute_trace(object):
    def __init__(self, rpc_object:rpc):
        # causes infinite recursion if we use normal assignment
        object.__setattr__(self, "trace_list", [])
        object.__setattr__(self, "attribute_index", rpc_object.index)
        object.__setattr__(self, "rpc_object", rpc_object)

    def get_attribute_name(self):
        '''Returns an attribute name based on the tracelist'''
        return '.'.join(self.trace_list)
    
    def find_attribute(self):
        attribute_name = self.get_attribute_name()
        if attribute_name in self.attribute_index:
            return self.attribute_index[attribute_name]
        else:
            return None

    def __call__(self, *args):
        print("CALLED!")

    def __getattr__(self, name):
        self.trace_list += [name]
        name = self.get_attribute_name()
        attribute_type = self.find_attribute()
        if attribute_type == "parameter":
            result = self.rpc_object.remote_call(name)
            return float(result['return'])
        elif attribute_type == "object":
            return self #return this attribute_trace object, which will be called again
        elif attribute_type == "function":
            return self #return attribute_trace object, so __call__ can be called.
        else:
            print("RPC ERROR: OBJECT " + name + " IS UNKNOWN")
            return None

    def __call__(self, *args, **kwds):
        name = self.get_attribute_name()
        attribute_type = self.find_attribute()
        if attribute_type == "parameter":
            print("RPC ERROR: OBJECT " + name + " IS A PARAMETER AND NON-CALLABLE.")
            return None
        elif attribute_type == "object":
            print("RPC ERROR: OBJECT " + name + " IS NON-CALLABLE.")
            return None
        elif attribute_type == "function":
            result = self.rpc_object.remote_call(name, args)
            if 'return' in result:
                return float(result['return'])
            else:
                return True
        else:
            print("RPC ERROR: OBJECT " + name + " IS UNKNOWN")
            return None
        
    def __setattr__(self, name, value):
        if name in self.__dict__:
            object.__setattr__(self, name, value) #use base __setattr__ function.
        else:
            name = self.get_attribute_name()
            attribute_type = self.find_attribute()
            if attribute_type == "parameter":
                self.rpc_object.remote_call(name, [value])
                return True
            elif attribute_type == "object":
                print("RPC ERROR: OBJECT " + name + " IS NOT SETTABLE.")
                return False
            elif attribute_type == "function":
                print("RPC ERROR: OBJECT " + name + " IS A FUNCTION AND NON-SETTABLE")
                return False
            else:
                print("RPC ERROR: OBJECT " + name + " IS UNKNOWN")
                return False

myrpc = rpc("/dev/tty.usbmodem160437701")
# myrpc.testValue = 123.4
import time

print("PRE-MOVE POSITIONS")
print("X: " + str(myrpc.kinematics.input_x.read()))
print("Y: " + str(myrpc.kinematics.input_y.read()))
print("A: " + str(myrpc.channel_a.input_target_position.read()))
print("B: " + str(myrpc.channel_b.input_target_position.read()))
myrpc.tbi.add_move(1, 10, 10)
time.sleep(2)
print("POST-MOVE POSITIONS")
print("X: " + str(myrpc.kinematics.input_x.read()))
print("Y: " + str(myrpc.kinematics.input_y.read()))
print("A: " + str(myrpc.channel_a.input_target_position.read()))
print("B: " + str(myrpc.channel_b.input_target_position.read()))
myrpc.kinematics.input_x.write_deep(20)
print("POST-DEEP-PUSH POSITIONS")
print("X: " + str(myrpc.kinematics.input_x.read()))
print("Y: " + str(myrpc.kinematics.input_y.read()))
print("A: " + str(myrpc.channel_a.input_target_position.read()))
print("B: " + str(myrpc.channel_b.input_target_position.read()))
print("PRE DEEP-PULL POSITIONS")
myrpc.channel_a.input_target_position.write_deep(-10)
myrpc.channel_b.input_target_position.write_deep(-50)
print("X: " + str(myrpc.kinematics.input_x.read()))
print("Y: " + str(myrpc.kinematics.input_y.read()))
print("A: " + str(myrpc.channel_a.input_target_position.read()))
print("B: " + str(myrpc.channel_b.input_target_position.read()))
print("POST DEEP-PULL POSITIONS")
print(myrpc.kinematics.input_x.read_deep())
print(myrpc.kinematics.input_y.read_deep())
print("X: " + str(myrpc.kinematics.input_x.read()))
print("Y: " + str(myrpc.kinematics.input_y.read()))
print("A: " + str(myrpc.channel_a.input_target_position.read()))
print("B: " + str(myrpc.channel_b.input_target_position.read()))