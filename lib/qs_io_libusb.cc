
#include "qs_io_libusb.h"
#include <stdio.h>
#include <string.h>

struct usb_dev_handle * hdev;
char * device_path;
char * last_error;
int dev_was_found;
int dev_count;
int ep_count;
int qs_EP2;
int qs_EP4;
int qs_EP6;
int qs_EP8;

void qs1rio_init( )
{
    hdev = NULL;

	last_error = 0;
	device_path = 0;

	dev_was_found = 0;
	dev_count = 0;
	ep_count = 0;
	qs_EP2 = 0x02;
	qs_EP4 = 0x04;
	qs_EP6 = 0x86;
	qs_EP8 = 0x88;

	last_error = (char *)malloc( 256 );
	device_path = (char *)malloc( 256 );

	strcpy( last_error, "None" );
	strcpy( device_path, "" );

	usb_init( );
}

void clearHalt( int ep )
{
    if ( hdev != NULL ) usb_clear_halt( hdev, ep );
}

void qs1rio_close( )
{
    if ( hdev )
    {
        usb_clear_halt( hdev, qs_EP2 );
        usb_clear_halt( hdev, qs_EP4 );
        usb_clear_halt( hdev, qs_EP6 );
        usb_clear_halt( hdev, qs_EP8 );
        usb_release_interface( hdev, 0 );
        usb_close( hdev );
    }
	hdev = NULL;

	if ( last_error ) {
		free (last_error);
	}
	if ( device_path ) {
		free (device_path);
	}
}

const char * get_last_error( )
{
	return last_error;
}
int findQsDevice( )
{
	struct usb_bus * bus;
	struct usb_device * dev;

	dev_count = 0;
	dev_was_found = 0;

    usb_find_busses( );
	usb_find_devices( );

    for (bus = usb_get_busses(); bus; bus = bus->next)
    {
        for (dev = bus->devices; dev; dev = dev->next)
        {
            if ( (	dev->descriptor.idVendor == QS1R_VID 
                        && dev->descriptor.idProduct == QS1R_PID )
                || ( dev->descriptor.idVendor == QS1R_MISSING_EEPROM_VID
                     && dev->descriptor.idProduct == QS1R_MISSING_EEPROM_PID ) )
                {
				strcpy( device_path, dev->filename );
				hdev = usb_open(dev);
                dev_count++;
				dev_was_found = 1;
				goto found;
            }
        }
    }
	strcpy( last_error, "Could not find device." );
    return -1;

	found:

    if ( hdev )
    {
        if ( usb_set_configuration( hdev, 1 ) != 0 )
        {
			strcpy( last_error, "Could not set configuration 1." );
			return -1;
        }
        if ( usb_claim_interface( hdev, 0 ) != 0 )
        {
			strcpy( last_error, "Could not claim interface 0." );
            return -1;
        }
        if ( usb_set_altinterface( hdev, 0 ) != 0 )
		{
			strcpy( last_error, "Could not set alt interface 0." );
            return -1;
        }

        if ( usb_clear_halt( hdev, qs_EP2 ) != 0 )
		{
			strcpy( last_error, "Could not clear halt on EP2" );
        }
        if ( usb_clear_halt( hdev, qs_EP4 ) != 0 )
        {
			strcpy( last_error, "Could not clear halt on EP4" );
        }
        if ( usb_clear_halt( hdev, qs_EP6 ) != 0 )
        {
			strcpy( last_error, "Could not clear halt on EP6" );
        }
        if ( usb_clear_halt( hdev, qs_EP8 ) != 0 )
        {
			strcpy( last_error, "Could not clear halt on EP8" );
        }
    }
    return 0;
}

int deviceCount( )
{
    return dev_count;
}

int epCount( )
{
    return ep_count;
}

int loadFirmware( const char * filename )
{
	char value = 1;
	FILE *f = 0;
	char s[1024];
	int flength;
	int faddr;
	int type;
	unsigned char data[256];
	unsigned char checksum, a;
	unsigned int b;
	int i;

    if (!dev_was_found)
	{
		strcpy( last_error, "Need to call findDevice first." );
		return -1;
    }

    // PUT CPU IN RESET


	if ( !write_cpu_ram( FX2_RAM_RESET, &value, 1 ) )
	{
		strcpy( last_error, "Could not put CPU in reset." );
		return -1;
	}

	if ( ( f = fopen( filename, "r" ) ) == 0 )
	{
		strcpy( last_error, "loadFirmware: filename does not exist." );
		return -1;
	}

    while (!feof(f) && fgets(s, 1024, f)) {
        if (s[0] != ':') {
			fclose(f);
			strcpy( last_error, "QsDevice::loadFirmware: error, firmware file appears to be corrupted" );
			return -1;
        }
        sscanf(s+1, "%02x", &flength);
		sscanf(s+3, "%04x", &faddr);
		sscanf(s+7, "%02x", &type);

		if (type == 0) {
            a = flength + (faddr & 0xff) + (faddr>>8) + type;
            for (i=0; i<flength; i++) {
                sscanf(s+9+i*2, "%02x", &b);
                data[i] = b;
                a = a + data[i];
            }

            sscanf(s+9+flength*2, "%02x", &b);
            checksum = b;
			if (((a + checksum) & 0xff) != 0x00) {
				strcpy( last_error, "loadFirmware: checksum failed." );
                fclose(f);
				return -1;
			}
			if (!write_cpu_ram(faddr, (char *)data, flength)) {
				strcpy( last_error, "loadFirmware: write failed." );
				fclose(f);
				return -1;
            }
        } else if (type == 0x01) {
            break;
        } else if (type == 0x02) {
			strcpy( last_error, "loadFirmware: extended address not supported." );
			fclose(f);
			return -1;
        }
    }

    fclose(f);
	// TAKE CPU OUT OF RESET

	value = 0;
	if ( !write_cpu_ram( FX2_RAM_RESET, &value, 1 ) )
	{
		strcpy( last_error, "Could not take CPU out of reset." );
		return -1;
	}
	return 0;
}

int loadFpga( const char * filename )
{
	FILE *f = 0;
	unsigned long count = 0;
	unsigned char buffer[ MAX_EP4_PACKET_SIZE ];
	int n = 0;

    if (!dev_was_found)
    {
		strcpy( last_error, "Need to call findDevice first." );
		return -1;
	}

	if ( ( f = fopen( filename, "rb") ) == 0)
	{
		strcpy( last_error, "loadFpga: filename does not exist." );
		return -1;
	}

	count = usb_control_msg(	hdev,
								VRT_VENDOR_OUT,
                                VRQ_FPGA_LOAD,
                                0,
                                FL_BEGIN,
                                0,
                                0,
                                USB_TIMEOUT_CONTROL );

    if ( count != 0 )
	{
		fclose(f);
		strcpy( last_error, "loadFpga: failed in FL_BEGIN load stage" );
		return -1;
	}

    while ( ( n = fread( buffer, 1, sizeof( buffer ), f ) ) > 0 )

    {   
		int len = writeEP4( buffer, n );
		if ( len != n )
		{
			fclose(f);
			strcpy( last_error, "loadFpga: failed in FL_XFER load stage" );
			break;
        }
	}

    count = usb_control_msg(	hdev,
                                VRT_VENDOR_OUT,
                                VRQ_FPGA_LOAD,
                                0,
                                FL_END,
                                0,
                                0,
                                USB_TIMEOUT_CONTROL );

    if ( count != 0 )
    {
		fclose(f);
		strcpy( last_error, "loadFpga: failed in FL_END load stage" );
		return -1;
    }

    fclose(f);

    return 0;
}

int readFwSn( )
{
	unsigned long count = 0;
	unsigned long nsize = 4;
	unsigned int i;
	char buf[4];

	if (!dev_was_found)
	{
		strcpy( last_error, "Need to call findDevice first." );
		return -1;
	}

	for ( i = 0; i < nsize; i++ )
    {
        buf[i] = 0;
    }

    count = usb_control_msg(	hdev, 
                                VRT_VENDOR_IN,
                                VRQ_SN_READ,
                                0,
                                0,
                                buf,
                                nsize,
                                USB_TIMEOUT_CONTROL );

    if ( count != nsize )
	{
		strcpy( last_error, "readFwSn: control transfer failed." );
        return -1;
    }
    return (int)((unsigned char)buf[0] +
                 ((unsigned char)buf[1] << 8) +
                 ((unsigned char)buf[2] << 16) +
                 ((unsigned char)buf[3] << 24));
}

int read( int ep, unsigned char * buffer, int length )
{
	int chanreg = 0;
	unsigned long count = 0;

	if (!dev_was_found)
	{
		strcpy( last_error, "Need to call findDevice first." );
		return -1;
	}

    if (ep == 6)
        chanreg = qs_EP6;
    else if (ep == 8)
        chanreg = qs_EP8;
    else
	{
		strcpy( last_error, "read: ep is invalid... (6/8 is valid)" );
        return -1;
    }

    count = usb_bulk_read( hdev, chanreg, (char *)buffer, length, USB_TIMEOUT_BULK );

    if ( count != (unsigned int)length )
	{
		strcpy( last_error, "read: could not read pipe." );
		return -1;
    }

    return count;
}

int write( int ep, unsigned char * buffer, int length )
{
	int chanreg = 0;
	unsigned long count = 0;

    if (!dev_was_found)
	{
		strcpy( last_error, "Need to call findDevice first." );
        return -1;
    }

	if (ep == 2)
        chanreg = qs_EP2;
    else if (ep == 4)
        chanreg = qs_EP4;
    else
	{
		strcpy( last_error, "write: ep is invalid... (2/4 is valid)" );
        return -1;
    }

    count = usb_bulk_write( hdev, chanreg, (char *)buffer, length, USB_TIMEOUT_BULK );

    if ( count != (unsigned int)length )
	{
		strcpy( last_error, "write: could not write pipe" );
		return -1;
    }

    return count;
}

int writeEP2( unsigned char * buffer, int length )
{
    if ( usb_bulk_write( hdev, qs_EP2, (char *)buffer, length, USB_TIMEOUT_BULK ) != length )
	{
		strcpy( last_error, "write: could not write pipe" );
		if ( usb_clear_halt( hdev, EP2 ) != 0 )
		{
			strcpy( last_error, "Could not clear halt on EP2" );
        }
        return -1;
    }
    return length;
}

int writeEP4( unsigned char * buffer, int length )
{
    if ( usb_bulk_write( hdev, qs_EP4, (char *)buffer, length, USB_TIMEOUT_BULK ) != length )
	{
		strcpy( last_error, "write: could not write pipe" );
		if ( usb_clear_halt( hdev, EP4 ) != 0 )
        {
				strcpy( last_error, "Could not clear halt on EP4" );
        }
        return -1;
    }
    return length;
}

int readEP6( unsigned char * buffer, int length )
{
    if ( usb_bulk_read( hdev, qs_EP6, (char *)buffer, length, USB_TIMEOUT_BULK ) != length )
	{
		strcpy( last_error, "read: could not write pipe" );
		if ( usb_clear_halt( hdev, EP6 ) != 0 )
        {
				strcpy( last_error, "Could not clear halt on EP6" );
		}
        return -1;
    }
    return length;
}

int readEP8( unsigned char * buffer, int length )
{
    if ( usb_bulk_read( hdev, qs_EP8, (char *)buffer, length, USB_TIMEOUT_BULK ) != length )
    {
        strcpy( last_error, "read: could not write pipe" );
        if ( usb_clear_halt( hdev, EP8 ) != 0 )
        {
				strcpy( last_error, "Could not clear halt on EP8" );
        }
        return -1;
    }
    return length;
}

int readEEPROM( int address, int offset, unsigned char * buffer, int length )
{
	unsigned char cmd[2];

    if (!dev_was_found)
	{
		strcpy( last_error, "Need to call findDevice first." );
		return -1;
    }

    if (length < 1) return -1;
    if (buffer == 0) return -1;

    cmd[0] = (char)((0xFF00 & offset) >> 8); // high byte address
    cmd[1] = (char)(0xFF & offset); // low byte address

    // set address pointer in EEPROM
    if (writeI2C(address, cmd, 2) != 2)
	{
		strcpy( last_error, "readEEPROM: Could not set EEPROM address" );
        return -1;
    }

    // now read from the address
    if (readI2C(address, buffer, length) != length)
	{
		strcpy( last_error, "readEEPROM: Could not read EEPROM device" );
		return -1;
    }

    return length;
}

int writeEEPROM( int address, int offset, unsigned char * buffer, int length )
{
	unsigned char cmd[3];
	int i;

    if (!dev_was_found)
	{
		strcpy( last_error, "Need to call findDevice first." );
        return -1;
	}

    if (length < 1) return -1;
    if (buffer == 0) return -1;

	for ( i=0; i < length; i++ ) {
        cmd[0] = (char)((0xFF00 & offset) >> 8); // high byte address
        cmd[1] = (char)(0xFF & offset); // low byte address
        cmd[2] = (char)buffer[i]; // value to write
        // set address pointer in EEPROM
        if (writeI2C(address, cmd, 3) != 3)
        {
			strcpy( last_error, "writeEEPROM: Could not write EEPROM" );
			return -1;
        }
        offset++;
        usleep( 10000 );
	}
	return length;
}

int readI2C( int address, unsigned char * buffer, int length )
{
	unsigned long count = 0;

    if (!dev_was_found)
	{
		strcpy( last_error, "Need to call findDevice first." );
		return -1;
    }

    count = usb_control_msg(	hdev,
                                VRT_VENDOR_IN,
                                VRQ_I2C_READ,
                                address,
                                0,
                                (char *)buffer,
                                length,
                                USB_TIMEOUT_CONTROL );

    if ( count != (unsigned int)length )
    {
		strcpy( last_error, "readI2C: control transfer failed." );
		return -1;
    }

    return count;
}

int writeI2C( int address, unsigned char * buffer, int length )
{
	unsigned long count = 0;

    if (!dev_was_found)
	{
		strcpy( last_error, "Need to call findDevice first." );
        return -1;
    }

	count = usb_control_msg(	hdev,
                                VRT_VENDOR_OUT,
                                VRQ_REQ_I2C_WRITE,
                                address,
                                0,
                                (char *)buffer,
                                length,
                                USB_TIMEOUT_CONTROL );

    if ( count != (unsigned int)length )
	{
		strcpy( last_error, "writeI2C: control transfer failed." );
		return -1;
    }

    return count;
}

int readMultibusInt( int index )
{
	unsigned long count = 0;
	unsigned char buf[4];
	unsigned long nsize = 4;

    if (!dev_was_found)
    {
		strcpy( last_error, "Need to call findDevice first." );
        return -1;
    }

    count = usb_control_msg(	hdev,
                                VRT_VENDOR_IN,
                                VRQ_MULTI_READ,
                                index,
                                0,
                                (char *)buf,
                                nsize,
                                USB_TIMEOUT_CONTROL );

    if ( count != nsize)
	{
		strcpy( last_error, "readMultibus: control transfer failed." );
		return -1;
    }

    return buf[0] + (buf[1] << 8) + (buf[2] << 16) + (buf[3] << 24);
}

int readMultibusBuf( int index, unsigned char * buffer, int length )
{
	unsigned long count = 0;
	unsigned long nsize = 4;

    if (!dev_was_found)
    {
		strcpy( last_error, "Need to call findDevice first." );
		return -1;
    }

    if (length != 4)
    {
		strcpy( last_error, "readMultibus: buffer must be length 4." );
        return -1;
    }

    count = usb_control_msg(	hdev,
                                VRT_VENDOR_IN,
                                VRQ_MULTI_READ,
                                index,
                                0,
                                (char *)buffer,
                                nsize,
                                USB_TIMEOUT_CONTROL );

    if ( count != nsize )
    {
		strcpy( last_error, "readMultibus: control transfer failed." );
        return -1;
    }

    return nsize;
}

int writeMultibusInt( int index, int value )
{
	unsigned long count = 0;
	unsigned char buf[4];
	unsigned long nsize = 4;

	if (!dev_was_found)
	{
		strcpy( last_error, "Need to call findDevice first." );
        return -1;
    }

    buf[0] = (value >>  0) & 0xff;
    buf[1] = (value >>  8) & 0xff;
    buf[2] = (value >> 16) & 0xff;
    buf[3] = (value >> 24) & 0xff;

    count = usb_control_msg(	hdev,
                                VRT_VENDOR_OUT,
                                VRQ_MULTI_WRITE,
                                index,
                                0,
                                (char *)buf,
                                nsize,
                                USB_TIMEOUT_CONTROL );

    if ( count != nsize )
	{
		strcpy( last_error, "writeMultibus: control transfer failed." );
		return -1;
    }

    return count;
}

int writeMultibusBuf( int index, unsigned char * buffer, int length )
{
	unsigned long count = 0;
	unsigned long nsize = 4;

    if (!dev_was_found)
	{
		strcpy( last_error, "Need to call findDevice first." );
        return -1;
    }

    if (length != 4)
	{
		strcpy( last_error, "writeMultibus: buffer must be length 4." );
		return -1;
    }

    count = usb_control_msg(	hdev,
                                VRT_VENDOR_OUT,
                                VRQ_MULTI_WRITE,
                                index,
                                0,
                                (char *)buffer,
                                nsize,
                                USB_TIMEOUT_CONTROL );

    if ( count != nsize )
	{
		strcpy( last_error, "writeMultibus: control transfer failed." );
		return -1;
    }

    return nsize;
}

void resetDevice( )
{
    if ( hdev == 0 ) return;
    usb_reset( hdev );
}

int write_cpu_ram(int startaddr, char * buffer, int length)
{
    int pkt_size = MAX_EP0_PACKET_SIZE;
    int addr = 0;
    int nsize = 0;
    unsigned long count = 0;

    for (addr = startaddr; addr < startaddr + length; addr += pkt_size)
    {
        nsize = length + startaddr - addr;
        if (nsize > pkt_size) nsize = pkt_size;

        count = usb_control_msg(hdev,
                                VRT_VENDOR_OUT,
                                FX2_WRITE_RAM_REQ,
                                addr,
                                0,
                                (char *)(buffer + (addr - startaddr)),
                                nsize,
                                USB_TIMEOUT_CONTROL );


        if ( count != (unsigned int)nsize )
        {
			strcpy( last_error, "write_cpu_ram failed" );
            return -1;
        }
    }

    return (int)count;
}
