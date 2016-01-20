#ifndef qsiolib
#define qsiolib

//#pragma hdrstop

//#include "libusb\win\usb.h"
#include <usb.h>

#define MAX_BUFFER_SZ			16384*8

#define QS1R_GUID				L"{1491A52C-73EC-4596-8638-C458CFB91A17}"
#define QS1R_VID				0xfffe
#define QS1R_PID				0x8
#define QS1R_MISSING_EEPROM_VID 0x4b4
#define QS1R_MISSING_EEPROM_PID 0x8613

#define MAX_EP0_PACKET_SIZE		64
#define MAX_EP4_PACKET_SIZE		1024

#define QS1R_DAC_EP				0x02
#define QS1R_CH0_EP				0x86
#define QS1R_CH1_EP				0x88

#define FX2_RAM_RESET			0xE600
#define FX2_WRITE_RAM_REQ		0xA0 

/* Vendor Request Types */
#define VRT_VENDOR_IN			0xC0
#define VRT_VENDOR_OUT			0x40

/* Vendor In Commands */
#define	VRQ_I2C_READ			0x81	// wValueL: i2c address; length: how much to read
#define	VRQ_SPI_READ			0x82	// wValue: optional header bytes
// wIndexH:	enables
// wIndexL:	format
// len: how much to read

#define VRQ_SN_READ     		0x83

#define VRQ_EEPROM_TYPE_READ	0x84
#define VRQ_I2C_SPEED_READ		0x85
#define VRQ_MULTI_READ			0x86
#define VRQ_DEBUG_READ			0x87

/* Vendor Out Commands */
#define VRQ_FPGA_LOAD			0x02
#define FL_BEGIN				0
#define FL_XFER					1
#define FL_END					2

#define VRQ_FPGA_SET_RESET		0x04	// wValueL: {0,1}
#define VRQ_MULTI_WRITE			0x05
#define VRQ_REQ_I2C_WRITE  		0x08	// wValueL: i2c address; data: data
#define VRQ_REQ_SPI_WRITE 		0x09	// wValue: optional header bytes
// wIndexH:	enables
// wIndexL:	format
// len: how much to write

#define VRQ_I2C_SPEED_SET  		0x0B  	// wValueL: {0,1}
#define VRQ_CPU_SPEED_SET		0x0C 	// wValueL: {0, 1, 2}
#define VRQ_EP_RESET			0x0D

#define DEFAULT_VID 0xfffe
#define DEFAULT_PID 0x8
#define QS1R_EEPROM_ADDR		0x51

#define MB_FREQRX0_REG			0
#define MB_FREQRX1_REG			1
#define MB_CONTRL0				2  
#define MB_CONTRL1				3
#define MB_SAMPLERATE			4

#define MB_CONTRL0_BIT0			0x1
#define MB_CONTRL0_BIT1			0x2
#define MB_CONTRL0_BIT2			0x4
#define MB_CONTRL0_BIT3			0x8
#define MB_CONTRL0_BIT4			0x10
#define MB_CONTRL0_BIT5			0x20
#define MB_CONTRL0_BIT6			0x40
#define MB_CONTRL0_BIT7			0x80
#define MB_CONTRL0_BIT31		0x80000000

#define MASTER_RESET			MB_CONTRL0_BIT31

#define MB_CONTRL1_BIT0			0x1
#define MB_CONTRL1_BIT1			0x2
#define MB_CONTRL1_BIT2			0x4

#define PGA						MB_CONTRL1_BIT0
#define RANDOM					MB_CONTRL1_BIT1
#define DITHER					MB_CONTRL1_BIT2

#define	MB_RXN_REG				1
#define MB_SPS_REG				2
#define MB_ID_REG				3

#define ID_2RX					0x20000000
#define ID_1RXWR 				0x04112011
#define ID_FWWR					3032011

#define EP2						2
#define EP4						4
#define EP6						6
#define EP8						8

#define USB_TIMEOUT_CONTROL		500
#define USB_TIMEOUT_BULK		1000

void qs1rio_init( );
void qs1rio_close( );
const char * get_last_error( );
int findQsDevice( );
int deviceCount( );
int epCount( );
int loadFirmware( const char * filename );
int loadFpga( const char * filename );
int readFwSn( );
int read( int ep, unsigned char * buffer, int length );
int write( int ep, unsigned char * buffer, int length );
int writeEP2( unsigned char * buffer, int length );
int writeEP4( unsigned char * buffer, int length );
int readEP6( unsigned char * buffer, int length );
int readEP8( unsigned char * buffer, int length );
int readEEPROM( int address, int offset, unsigned char * buffer, int length );
int writeEEPROM( int address, int offset, unsigned char * buffer, int length );
int readI2C( int address, unsigned char * buffer, int length );
int writeI2C( int address, unsigned char * buffer, int length );
int readMultibusInt( int index );
int readMultibusBuf( int index, unsigned char * buffer, int length );
int writeMultibusInt( int index, int value );
int writeMultibusBuf( int index, unsigned char * buffer, int length );
void resetDevice( );
void clearHalt( int ep );
int write_cpu_ram( int startaddr, char * buffer, int length );

extern struct usb_dev_handle * hdev;
extern char * device_path;
extern char * last_error;
extern int dev_was_found;
extern int dev_count;
extern int ep_count;
extern int qs_EP2;
extern int qs_EP4;
extern int qs_EP6;
extern int qs_EP8;

#endif

