#include "Arduino.h"
#include "usb_serial.h"
#include "core_pins.h"
#include <ArduinoJson.h>
#include "HardwareSerial.h"
#include "Stream.h"
#include "rpc.hpp"

RPC::RPC(){};

void RPC::begin(){
  begin(&Serial);
}

void RPC::begin(Stream *target_stream){
  this->rpc_stream = target_stream; //sets the RPC interface
  register_plugin(PLUGIN_LOOP); //this runs in the main program loop
}

void RPC::begin(usb_serial_class *target_usb_serial){
  target_usb_serial -> begin(115200); //baud rate is unused
  this->rpc_stream = target_usb_serial; //sets the RPC interface
  register_plugin(PLUGIN_LOOP); //this runs in the main program loop
};

void RPC::begin(HardwareSerialIMXRT *target_serial, uint32_t baud, uint16_t format){
  target_serial->begin(baud, format);
  this->rpc_stream = target_serial; //sets the RPC interface
  register_plugin(PLUGIN_LOOP); //this runs in the main program loop
}

void RPC::reset_inbound_state(){
  inbound_json_doc.clear();
  inbound_string = "";
}

void RPC::reset_outbound_state(){
  outbound_json_doc.clear();
}

void RPC::loop(){
  while(rpc_stream->available()){
    char c = rpc_stream->read();
    if(c == '\n'){ //end of JSON stream
      DeserializationError error = deserializeJson(inbound_json_doc, inbound_string);
      if(!error){
        rpc_call(inbound_json_doc["name"], inbound_json_doc["args"]); //make RPC call
      }else{
        // return deserialization error
      }
      reset_inbound_state();
    }else{ //still in JSON stream
      inbound_string += c;
    }
  }
}

void RPC::rpc_call(const String& name, JsonArray args){
  if(name == "__index__"){ //calling internal index function
    send_index();
  }else{
    auto result = rpc_registry.find(name); //find function in registry
    if(result != rpc_registry.end()){
      result->second(args); //calls the function mapped to name (e.g. the second value of the map)
    }else{
      reset_outbound_state();
      outbound_json_doc["error"] = "Unknown Function or Parameter";
      serializeJson(outbound_json_doc, *rpc_stream);
      rpc_stream->println(); //send newline character
    }
  }
}

void RPC::send_index(){ //returns an index of all functions and parameters that are currently mapped.
      reset_outbound_state();
      outbound_json_doc["result"] = "ok";
      JsonObject index = outbound_json_doc["return"].to<JsonObject>();
      for (auto const &key_value : rpc_index){
        index[key_value.first] = key_value.second;
      }
      serializeJson(outbound_json_doc, *rpc_stream);
      rpc_stream->println();
}
