/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <unistd.h>

#include <gnuradio/io_signature.h>
#include "qs1r_src_impl.h"
#include "qs_io_libusb.h"

#define QS1R_FIRMWARE_VER "qs1r_firmware_03032011.hex"
#define QS1R_FPGA_VER "QS1R_WINRAD_04112011.rbf"
#define BLOCKSZ 4096


static const char *std_paths[] = {
  ".",
  "/usr/share/libgnuradio-qs1r",
  "/usr/local/share/libgnuradio-qs1r",
  "/lib/firmware/qs1r",
  0
};


static char *
find_file (const char *filename)
{
  const char **sp = std_paths;
  static char path[1000];

  while (*sp){
      //snprintf (path, sizeof (path), "%s/rev%d/%s", *sp, hw_rev, filename); //don't need rev support now
      snprintf (path, sizeof (path), "%s/%s", *sp, filename);
    if (access (path, R_OK) == 0)
      return path;
    sp++;
  }
  return 0;
}


static bool first = true;


////////////////////
#define HiBYTE(w) (uint8_t)((w >> 8) & 0x00ff)
#define LoBYTE(w) (uint8_t)((w >> 0) & 0x00ff)
#define CONST__GAINoff 0b10001000
#define CONST__GAINon  0b10001001 // Weak Signal Booster ON
#define CONST__LoBAND  0b00000001
#define CONST__MiBAND  0b00000010
#define CONST__HiBAND  0b00001100

#define RCVPCH1 37300

int KSH14xPLL(bool TunerGain, uint32_t FrcvWork)

{
  uint32_t fPCH1;
  uint8_t buffer[4];

  buffer[2] = (TunerGain)? CONST__GAINon:CONST__GAINoff;
  buffer[3] = CONST__LoBAND;

  if(FrcvWork > 155000L) buffer[3] = CONST__MiBAND;
  if(FrcvWork > 440000L) buffer[3] = CONST__HiBAND;

  fPCH1 = (uint16_t)((FrcvWork + RCVPCH1)/50);
  buffer[0] = HiBYTE(fPCH1);
  buffer[1] = LoBYTE(fPCH1);

//  int count = writeI2C(0b11000000, buffer, 4);
  int count = writeI2C(0b1100000, buffer, 4);
  fprintf(stderr, "i2c write count: %d\n", count);
//  I2CStart();
//  I2COutByte(0b11000000);   // Tuner Address
//  I2COutByte(HiBYTE(fPCH1));
//  I2COutByte(LoBYTE(fPCH1));
//  I2COutByte((TunerGain)? CONST__GAINon:CONST__GAINoff);
//  I2COutByte(Band);
//  I2CStop();
    return (count == 4) ? 0 : -1 ;
}



namespace gr {
  namespace qs1r {

    qs1r_src::sptr
    qs1r_src::make(unsigned long frequency, unsigned int samplerate, bool pga_flag, bool rand_flag, bool dith_flag, int ppm)
    {
      return gnuradio::get_initial_sptr
        (new qs1r_src_impl(frequency, samplerate, pga_flag, rand_flag, dith_flag, ppm));
    }

    /*
     * The private constructor
     */
    qs1r_src_impl::qs1r_src_impl(unsigned long frequency, unsigned int samplerate, bool pga_flag, bool rand_flag, bool dith_flag, int ppm)
      : gr::sync_block("qs1r_src",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(int)))
    {
        set_output_multiple (BLOCKSZ * sizeof(int) * 2);

        if (first) {

            qs1rio_init( );

            if (findQsDevice( ) == -1) // try to initialize board
            {
                throw std::runtime_error("Could not find QS1R! Make sure board is connected and powered up.\n");
            }
            fprintf(stderr, "Found QS1R, checking firmware...\n");
            usleep(50000);

            if ( readFwSn() != ID_FWWR ) {
                fprintf(stderr, "Found QS1R, loading firmware...\n");
                char *fw_path = find_file(QS1R_FIRMWARE_VER);
                if(!fw_path) {
                    throw std::runtime_error("Could not load QS1R firmware!\n");
                }

                if ( loadFirmware( fw_path ) == -1) {
                    fprintf(stderr, "QS1R error: %s\n", get_last_error());
                    throw std::runtime_error("Could not load QS1R firmware!\n");
                }
                fprintf(stderr, "Firmware loaded... reinit...\n");
                usleep(4000000);
                if ( findQsDevice( ) == -1 ) // try to initialize board again
                {
                    throw std::runtime_error("Could not find QS1R!\n");
                }
                fprintf(stderr, "Found QS1R...\n");
                usleep(100000);
            }


            if ( readMultibusInt( MB_ID_REG ) != ID_1RXWR ) // is FPGA loaded with proper file?
            {
                fprintf(stderr, "Loading QS1R FPGA... Please wait...\n");
                char *fw_path = find_file(QS1R_FPGA_VER);
                if(!fw_path) {
                    throw std::runtime_error("Could not load QS1R firmware!\n");
                }
                if ( loadFpga( fw_path ) == -1 ) {
                    fprintf(stderr, "QS1R error: %s\n", get_last_error());
                    throw std::runtime_error("Could not load QS1R Fpga!\n");
                }
            }

            fprintf(stderr, "QS1R FPGA is loaded...\n");
            usleep(500000);
            first = false;
        }
#if 1
        writeMultibusInt( MB_CONTRL0, 0x80000000 ); //master reset
        usleep( 10000 );
        writeMultibusInt( MB_CONTRL0, 0x00000003 ); // dac bypass, mute external dac
        usleep( 10000 );

        uint32_t ctrl_reg = 0x00000000;
        if(pga_flag) ctrl_reg |= 0x01;
        if(rand_flag) ctrl_reg |= 0x02;
        if(dith_flag) ctrl_reg |= 0x04;
        writeMultibusInt( MB_CONTRL1, ctrl_reg);
        usleep( 10000 );
#else
        writeMultibusInt( MB_CONTRL1, 0x80000000 ); //master reset
        usleep( 10000 );
        writeMultibusInt( MB_CONTRL1, 0x00000003 ); //master reset
        usleep( 10000 );
#endif
        writeMultibusInt(MB_SAMPLERATE, samplerate);

        if(frequency > 60000000) {
            //use KSH-148 tuner...
            if(KSH14xPLL(false, frequency/1000) == -1) {
                throw std::runtime_error("Could not set VHF/UHF tuner frequency!\n");
            }

            //tune receiver to 37.300MHz (center)
            double LOfreq = RCVPCH1 * 1000.0f + (frequency % 1000);
            int frequency_word = (int)((LOfreq) / ( 125e6 + (double)ppm ) * 4294967296.0);
            writeMultibusInt(MB_FREQRX0_REG, frequency_word);
        } else {
            double LOfreq = (double)frequency;
            int frequency_word = (int)((LOfreq) / ( 125e6 + (double)ppm ) * 4294967296.0);
            writeMultibusInt(MB_FREQRX0_REG, frequency_word);
        }
    }

    /*
     * Our virtual destructor.
     */
    qs1r_src_impl::~qs1r_src_impl()
    {
        writeMultibusInt( MB_CONTRL1, 0x80000000 );
        qs1rio_close( );
    }

    int
    qs1r_src_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        int *out = (int *) output_items[0];

        // Do <+signal processing+>
        int length = BLOCKSZ * sizeof(int) * 2;
//        unsigned char * buffer_c = new unsigned char[length];


        if ( readEP6( (unsigned char *)out, length ) == length ) {
            return BLOCKSZ*2;
        } else {
            throw std::runtime_error("QS1R read error!\n");
        }
        // Tell runtime system how many output items we produced.
        return -1;
    }

  } /* namespace qs1r */
} /* namespace gr */

