// Teensy 4.1 pin numbers

#ifdef module_driver
  #define IO_A1     24
  #define IO_A2     26
  #define IO_A3     27
  #define IO_A4     25
  #define IO_D1     28
  #define IO_D2     12
  #define MOTOR_A_VREF    15
  #define MOTOR_B_VREF    41
  #define MOTOR_C_VREF    40
  #define MOTOR_D_VREF    38
#endif

#ifdef module_legacy
  #define IO_A  16
  #define IO_B  15
  #define IO_C  14
  #define IO_D  41
  #define IO_E  40
#endif

#ifdef module_basic
  #define IO_A1     14
  #define IO_A2     15
  #define IO_A3     20
  #define IO_A4     21
  #define IO_D1     13
  #define IO_D2     12
#endif
