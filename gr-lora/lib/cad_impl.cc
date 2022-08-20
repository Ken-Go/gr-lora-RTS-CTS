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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "cad_impl.h"

namespace gr {
  namespace lora {

    cad::sptr
    cad::make(uint8_t spreading_factor,
              uint16_t fft_factor,
              float    fs_bw_ratio)
    {
      return gnuradio::get_initial_sptr
        (new cad_impl(spreading_factor,fft_factor,fs_bw_ratio));
    }


    /*
     * The private constructor
     */
    cad_impl::cad_impl(uint8_t spreading_factor,
                       uint16_t fft_factor,
                       float    fs_bw_ratio)
      : gr::block("cad",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0)),
        d_sf(spreading_factor),
        d_fft_size_factor(fft_factor)
    {
      d_p = (int) fs_bw_ratio;        
      d_cad_port = pmt::mp("CAD");
      message_port_register_out(d_cad_port);
      d_count = 0;
      d_state = S_CAD_RESET;
      d_CAD_result = false;
      d_num_symbols = (1 << d_sf);
      d_num_samples = d_p * d_num_symbols;
      d_bin_len = d_fft_size_factor * d_num_symbols;
      d_fft_size = d_fft_size_factor *d_num_samples;
      d_fft = new fft::fft_complex(d_fft_size,true,1);
      
      d_window = fft::window::build(fft::window::WIN_KAISER, d_num_samples, d_beta);

      for (int i = 0; i < d_num_samples; i++) {
        double phase = M_PI/d_p*(i-i*i/(float)d_num_samples);
        d_downchirp.push_back(gr_complex(std::polar(1.0, phase)));
        d_upchirp.push_back(gr_complex(std::polar(1.0, -phase)));
      }
    }

    /*
     * Our virtual destructor.
     */
    cad_impl::~cad_impl()
    {
      delete d_fft;
    }

    float
    cad_impl::search_Magnitude(const lv_32fc_t *fft_result,
                                float *buffer1, float *buffer2)
    {
      // size of buffer1:   d_fft_size (float)
      // size of buffer2:   d_bin_len  (float)
   
      // fft result magnitude summation
      volk_32fc_magnitude_32f(buffer1, fft_result, d_fft_size);
      volk_32f_x2_add_32f(buffer2, buffer1, &buffer1[d_fft_size-d_bin_len], d_bin_len);
      
      float meanval = 0;
      volk_32f_accumulator_s32f(&meanval,buffer2,d_bin_len);
      meanval /= d_bin_len;
      // if(  meanval  > 0.01){
      //   std::cout<<"buffer mag is :"<<meanval<<std::endl;
      // }
      if(  meanval  > 0.01){
        return meanval;
      }else{
        return -1;
      }
    }

    void
    cad_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
      /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
      ninput_items_required[0] = noutput_items * (1 << d_sf) * 2;
    }

    int
    cad_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      const gr_complex *in = (const gr_complex *) input_items[0];
      unsigned int  *out    = (unsigned int   *) output_items[0];
      float CAD_Search_num = 0;
      unsigned int num_consumed   = d_num_samples;
      bool CAD_found = false;
      gr_complex *buffer     = (gr_complex *)volk_malloc(d_fft_size*sizeof(gr_complex), volk_get_alignment());
      gr_complex *up_block   = (gr_complex *)volk_malloc(d_fft_size*sizeof(gr_complex), volk_get_alignment());
      float *fft_res_mag = (float*)volk_malloc(d_fft_size*sizeof(float), volk_get_alignment());
      float *fft_res_add = (float*)volk_malloc(d_bin_len*sizeof(float), volk_get_alignment());
      
      // Dechirp the incoming signal
      volk_32fc_x2_multiply_32fc(up_block, in, &d_downchirp[0], d_num_samples);
      
      memset(d_fft->get_inbuf(),  0,d_fft_size*sizeof(gr_complex));
      memcpy(d_fft->get_inbuf(), &up_block[0], d_num_samples*sizeof(gr_complex));
      d_fft->execute();
      CAD_Search_num = search_Magnitude(d_fft->get_outbuf(),fft_res_mag,fft_res_add);
      d_argmax_history.push_back(CAD_Search_num);
      switch(d_state){
      case S_CAD_RESET:{
        d_argmax_history.clear();
        d_state = S_CAD_PREFILL;
          break;
      }
          
       
      case S_CAD_PREFILL:
      {
          float positive_negative_ratio = 0;
          int positiveNum = 0;
          if(d_argmax_history.size() == 8){
            for(int i = 0; i < d_argmax_history.size();i++){
              if(d_argmax_history[i] > 0.01){
                  positiveNum++;
              }
            }
            positive_negative_ratio = positiveNum / d_argmax_history.size();
            if(positive_negative_ratio > 0.5){
              d_CAD_result = true;
            }else{
              d_CAD_result = false;
            }
            d_state = S_CAD_OUT;
          }
        break;
      }
      case S_CAD_OUT:
      {
          pmt::pmt_t dict = pmt::make_dict();
          if(d_CAD_result){
            dict = pmt::dict_add(dict, pmt::intern("result"), pmt::PMT_T);
          }else{
            dict = pmt::dict_add(dict, pmt::intern("result"), pmt::PMT_F);
          }
          
          pmt::pmt_t msg_pair = pmt::cons(dict, pmt::make_u8vector(10, 0));
          message_port_pub(d_cad_port, msg_pair);
          
          d_state = S_CAD_RESET;
        break;
      }

      default :
        break;
      }

      // Do <+signal processing+>
      // Tell runtime system how many input items we consumed on
      // each input stream.
      consume_each (num_consumed);

      // Tell runtime system how many output items we produced.
      return noutput_items;
    }

  } /* namespace lora */
} /* namespace gr */

