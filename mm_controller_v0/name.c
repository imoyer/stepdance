
#include <usb_names.h>
#define EIBOTBOARD //changes USB descriptor to mimic EIBOTBOARD

#ifdef EIBOTBOARD

#define MANUFACTURER_NAME  \
	{                        \
		'C', 'W', 'P', '+', 'E', 'C', 'L'     \
	}
#define MANUFACTURER_NAME_LEN 7

#define PRODUCT_NAME                                                 \
	{                                                                  \
		'E', 'i', 'B', 'o', 't', 'B', 'o', 'a', 'r', 'd'                 \
	}
#define PRODUCT_NAME_LEN 10

#define SERIAL_NUMBER       \
	{                         \
		'E', 'i', 'B', 'o', 't' \
	}
#define SERIAL_NUMBER_LEN 5

#else

#define MANUFACTURER_NAME  \
	{                        \
		'C', 'W', 'P', '+', 'E', 'C', 'L'     \
	}
#define MANUFACTURER_NAME_LEN 7

#define PRODUCT_NAME                                \
	{                                                 \
		'S', 't', 'e', 'p', 'D', 'a', 'n', 'c', 'e'     \
	}
#define PRODUCT_NAME_LEN 9

#define SERIAL_NUMBER                                \
	{                                                  \
		'S', 't', 'e', 'p', 'D', 'a', 'n', 'c', 'e'      \
	}
#define SERIAL_NUMBER_LEN 9

#endif

struct usb_string_descriptor_struct usb_string_manufacturer_name = {
	2 + MANUFACTURER_NAME_LEN * 2,
	3,
	MANUFACTURER_NAME};

struct usb_string_descriptor_struct usb_string_product_name = {
	2 + PRODUCT_NAME_LEN * 2,
	3,
	PRODUCT_NAME};

struct usb_string_descriptor_struct usb_string_serial_number = {
	2 + SERIAL_NUMBER_LEN * 2,
	3,
	SERIAL_NUMBER};