/* -*- c++ -*- */

#define QS1R_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "qs1r_swig_doc.i"

%{
#include "qs1r/qs1r_src.h"
%}

%include "qs1r/qs1r_src.h"
GR_SWIG_BLOCK_MAGIC2(qs1r, qs1r_src);
