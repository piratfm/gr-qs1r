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


#ifndef INCLUDED_QS1R_QS1R_SRC_H
#define INCLUDED_QS1R_QS1R_SRC_H

#include <qs1r/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace qs1r {

    /*!
     * \brief <+description of block+>
     * \ingroup qs1r
     *
     */
    class QS1R_API qs1r_src : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<qs1r_src> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of qs1r::qs1r_src.
       *
       * To avoid accidental use of raw pointers, qs1r::qs1r_src's
       * constructor is in a private implementation
       * class. qs1r::qs1r_src::make is the public interface for
       * creating new instances.
       */
      static sptr make(unsigned long frequency, unsigned int samplerate, bool pga_flag, bool rand_flag, bool dith_flag, int ppm);
    };

  } // namespace qs1r
} // namespace gr

#endif /* INCLUDED_QS1R_QS1R_SRC_H */

