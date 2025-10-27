#include <type_traits>
#include <utility>
#include "WString.h"
#include <sys/_stdint.h>
#include "HardwareSerial.h"
#include "Arduino.h"
#include "usb_serial.h"
#include <functional>
#include <map>
#include <ArduinoJson.h>

/*
Remote Procedure Call (RPC) Module of the StepDance Control System

This module contains facilities for accessing internal stepdance functions and parameters over serial via remote procedure calls.

[More Details to be Added]

A part of the Mixing Metaphors Project
(c) 2025 Ilan Moyer, Jennifer Jacobs, Devon Frost

*/
#include "Stream.h"
#include "core.hpp"

#ifndef rpc_h //prevent importing twice
#define rpc_h

class RPC : public Plugin{
  public:
    RPC();
    void begin(); //defaults to Serial as the input stream
    void begin(Stream *target_stream);
    void begin(usb_serial_class *target_usb_serial);
    void begin(HardwareSerialIMXRT *target_serial, uint32_t baud, uint16_t format = 0); //hardware serial

    // --- RPC Registration ---
    template<typename Ret, typename... Args> //function registration
    void enroll(const String& name, Ret(*func)(Args...)){ //registers an RPC function with any signature, as handled by above templating
      add_to_registry(name, [func, this](JsonArray args){ //creates a lambda function with access to func, that accepts a JsonArray of arguments (i.e. matching the registry map definition)
        this->call_and_respond(func, args, std::index_sequence_for<Args...>{}); //the lambda function will then call a call_and_respond
      });
      rpc_index[name] = "function";
    }

    template<typename Obj, typename Ret, typename... Args> //bound method registration. This is intended to be called from within the enroll() method of a plugin
    void enroll(const String& instance_name, const String& name, Obj& instance, Ret(Obj::*method)(Args...)){
      add_to_registry(instance_name + "." + name, [&instance, method, this](JsonArray args){
        this->call_and_respond(instance, method, args, std::index_sequence_for<Args...>{});
      });
      rpc_index[instance_name + "." + name] = "function";
    }

    void enroll(const String& name, Plugin& instance){ //enrolls a plugin instance
      instance.enroll(this, name);
    }

    template<typename T, typename = std::enable_if_t<!std::is_base_of_v<Plugin, T>>> //parameter registration
    void enroll(const String& name, T& parameter){
      add_to_registry(name, [&parameter, this](JsonArray args){
        if(!args.isNull() && args.size() > 0){ //we're setting the value of the parameter
          parameter = args[0].as<T>();
          reset_outbound_state();
          outbound_json_doc["result"] = "ok";
          serializeJson(outbound_json_doc, *rpc_stream);
          rpc_stream->println();
        }else{ //getting the value
          reset_outbound_state();
          outbound_json_doc["result"] = "ok";
          outbound_json_doc["return"] = parameter;
          serializeJson(outbound_json_doc, *rpc_stream);
          rpc_stream->println();
        }
      });
      rpc_index[name] = "parameter";
    }

    // --- RPC Dispatch ---
    template<typename... Args, size_t... I>  // function with no return value
    void call_and_respond(void(*func)(Args...), JsonArray args, std::index_sequence<I...>){
      func(args[I].as<Args>()...); //calls function with args
      reset_outbound_state();
      outbound_json_doc["result"] = "ok";
      serializeJson(outbound_json_doc, *rpc_stream);
      rpc_stream->println();
    }

    template<typename Obj, typename... Args, size_t... I>  // bound method with no return value
    void call_and_respond(Obj& instance, void(Obj::*method)(Args...), JsonArray args, std::index_sequence<I...>){
      (instance.*method)(args[I].as<Args>()...); //calls function with args
      reset_outbound_state();
      outbound_json_doc["result"] = "ok";
      serializeJson(outbound_json_doc, *rpc_stream);
      rpc_stream->println();
    }

    template<typename Ret, typename... Args, size_t... I>
    void call_and_respond(Ret(*func)(Args...), JsonArray args, std::index_sequence<I...>){
      Ret ret = func(args[I].as<Args>()...); //calls function with args and returns type Ret
      reset_outbound_state();
      outbound_json_doc["result"] = "ok";
      outbound_json_doc["return"] = ret;
      serializeJson(outbound_json_doc, *rpc_stream);
      rpc_stream->println();
    }

    template<typename Obj, typename Ret, typename... Args, size_t... I>  // bound method with no return value
    void call_and_respond(Obj& instance, Ret(Obj::*method)(Args...), JsonArray args, std::index_sequence<I...>){
      Ret ret = (instance.*method)(args[I].as<Args>()...); //calls function with args
      reset_outbound_state();
      outbound_json_doc["result"] = "ok";
      outbound_json_doc["return"] = ret;
      serializeJson(outbound_json_doc, *rpc_stream);
      rpc_stream->println();
    }

  private:
    void reset_inbound_state();
    void reset_outbound_state();

    Stream *rpc_stream; //pointer to an I/O stream for the remote call
    String inbound_string;
    JsonDocument inbound_json_doc; //stores JSON doc based on inbound stream
    JsonDocument outbound_json_doc;

    using RPCFunction = std::function<void(JsonArray)>; //function format as it goes into the registry
    std::map<String, RPCFunction> rpc_registry; //registry for storing rpc functions. Note that these are lambda functions wrapping the actual function (or parameter) to be called/returned.
    std::map<String, String> rpc_index; //index of all rpc strings and their type. This can be reported back to the remote system as a convenience.

    inline void add_to_registry(const String& name, RPCFunction rpc_function){
      rpc_registry[name] = rpc_function;
    };

    void rpc_call(const String& name, JsonArray args); //makes an RPC call, and handles returning values etc.
    void send_index(); //returns an index of all the functions registered in the RPC.

  protected:
    void loop(); // should be run inside loop
};


#endif //rpc_h