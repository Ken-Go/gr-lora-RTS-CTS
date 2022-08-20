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

#ifndef INCLUDED_LORA_RTS_SENDER_IMPL_H
#define INCLUDED_LORA_RTS_SENDER_IMPL_H

#include <lora/RTS_Sender.h>
#include <string>
#include <lora/demod.h>
#include <lora/detect.h>
#include <lora/decode.h>
#include <volk/volk.h>
#include <fstream>
#include <cmath>
#include <queue>
#include <gnuradio/flowgraph.h>
#include <vector>
#include "utilities.h"
#include <gnuradio/fft/fft.h>

#define DETECT_CAD_FRAME_NUM 8
#define DETECR_CAD_MAG  0.01
namespace gr {
  namespace lora {
        
    class RTS_Sender_impl : public RTS_Sender
    {
     private:
      // Port parameter
      pmt::pmt_t d_in_port;
      pmt::pmt_t d_out_port;
      pmt::pmt_t d_CTS_port;
      //************public parameter****************

      //Node ID
      uint8_t d_Node_ID;
      uint8_t d_duration;  

      //different state
      RTS_Sender_State d_state;
      
      //**************state S_WAIT_DATA parameter********************* 
      pmt::pmt_t d_new_data;
      std::queue<std::string> d_buffer_strs;
      


      //*************state S_RTS_CAD parameter************************
      uint8_t d_sf;      
      unsigned short d_num_symbols;
      unsigned short d_num_samples;
      unsigned short d_fs_bw_ratio;
      unsigned short d_fft_size_factor;
      unsigned int   d_fft_size;
      unsigned int   d_bin_len;
      
      std::vector<gr_complex> d_upchirp;
      std::vector<gr_complex> d_downchirp;

      fft::fft_complex * d_fft;
      std::vector<unsigned int> d_argmax_history;
      //*************state S_RTS_WAIT_CTS parameter************************
      
      //*************state S_RTS_CAD parameter************************



    public:
      RTS_Sender_impl(uint8_t NodeId,uint8_t Duration);      
      ~RTS_Sender_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);
      //search this frame mean mag
      float search_fft_mean_mag(const lv_32fc_t * fft_result,float * buffer1,float * buffer2);
      //detect cad
      float CAD_Detect(const lv_32fc_t * fft_result,float * buffer1,float * buffer2);


      int general_work(int noutput_items,
           gr_vector_int &ninput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

      void Get_Read(const pmt::pmt_t msg);
      void HandleCTS(const pmt::pmt_t msg);
    };

  } // namespace lora
} // namespace gr

#endif /* INCLUDED_LORA_RTS_SENDER_IMPL_H */

