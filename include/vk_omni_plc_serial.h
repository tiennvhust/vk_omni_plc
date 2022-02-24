#ifndef VK_OMNI_PLC_SERIAL_H
#define VK_OMNI_PLC_SERIAL_H

#define ADDRESS 156
#define CONTROL 103
#define ACKNOWLEDGE 104

#define TIME_OUT 30

#define DEFAULT_BAUDRATE BaudRate::B_115200
#define DEFAULT_DEVICE "/dev/ttyUSB0"

/*acknowledge codes*/
#define RES_CODE 8
#define INI_CODE 9
#define EMG_CODE 10
#define PRT_CODE 11
#define NOM_CODE 12
#define ERR_CODE 13

/*command codes*/
#define RDY_CODE 21
#define RST_CODE 22
#define STT_CODE 23

#endif //PLC_SERIAL_H