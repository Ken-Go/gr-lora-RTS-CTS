/* -*- c++ -*- */
/*
 * Copyright 2022 zjhao.
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

#ifndef INCLUDED_LORA_CAD_IMPL_H
#define INCLUDED_LORA_CAD_IMPL_H

#include <lora/cad.h>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <queue>
#include <complex>
#include <fstream>
#include <gnuradio/fft/fft.h>
#include <gnuradio/fft/window.h>
#include <volk/volk.h>
#include "utilities.h"

namespace gr {
  namespace lora {

    class cad_impl : public cad
    {
     private:
      // Nothing to declare in this block.
      pmt::pmt_t d_cad_port;
      
      cad_state_t   d_state;
      uint8_t d_sf;
      unsigned short  d_num_symbols;
      unsigned short  d_fft_size_factor;
      unsigned int    d_fft_size;

      unsigned short  d_p;
      unsigned int    d_num_samples;
      unsigned int    d_bin_len;
      uint8_t d_count;
      fft::fft_complex *d_fft;
      std::vector<float> d_window;
      float d_beta;
      bool  d_CAD_result;

      std::vector<gr_complex> d_upchirp;
      std::vector<gr_complex> d_downchirp;

      std::vector<unsigned int>  d_argmax_history;
     public:
      cad_impl(uint8_t spreading_factor,
              uint16_t fft_factor,
              float    fs_bw_ratio);
      ~cad_impl();

      float search_Magnitude(const lv_32fc_t *fft_result,
                             float *buffer1, float *buffer2);

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

    };

  } // namespace lora
} // namespace gr

#endif /* INCLUDED_LORA_CAD_IMPL_H */

