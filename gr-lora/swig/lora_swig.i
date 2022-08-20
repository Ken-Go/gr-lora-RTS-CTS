/* -*- c++ -*- */

#define LORA_API

%include "gnuradio.i"           // the common stuff

//load generated python docstrings
%include "lora_swig_doc.i"

%{
#include "lora/demod.h"
#include "lora/mod.h"
#include "lora/pyramid_demod.h"
#include "lora/decode.h"
#include "lora/encode.h"
#include "lora/detect.h"
#include "lora/cad.h"
#include "lora/RTS_Sender.h"
#include "lora/CTS_Sender.h"
%}

%include "lora/demod.h"
GR_SWIG_BLOCK_MAGIC2(lora, demod);
%include "lora/mod.h"
GR_SWIG_BLOCK_MAGIC2(lora, mod);
%include "lora/pyramid_demod.h"
GR_SWIG_BLOCK_MAGIC2(lora, pyramid_demod);
%include "lora/decode.h"
GR_SWIG_BLOCK_MAGIC2(lora, decode);
%include "lora/encode.h"
GR_SWIG_BLOCK_MAGIC2(lora, encode);
%include "lora/detect.h"
GR_SWIG_BLOCK_MAGIC2(lora, detect);
%include "lora/cad.h"
GR_SWIG_BLOCK_MAGIC2(lora, cad);
%include "lora/RTS_Sender.h"
GR_SWIG_BLOCK_MAGIC2(lora, RTS_Sender);
%include "lora/CTS_Sender.h"
GR_SWIG_BLOCK_MAGIC2(lora, CTS_Sender);
