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
#include "RTS_Sender_impl.h"

#define RTS_DEBUG_OFF 0
#define RTS_DEBUG_INFO 1
#define RTS_DEBUG RTS_DEBUG_OFF
// CTS put in packet or symbol
#define USE_CTS_PACKET 0
#define USE_CTS_SYMBOL 1
#define USE_CTS_COND USE_CTS_PACKET

namespace gr
{
    namespace lora
    {

        RTS_Sender::sptr
        RTS_Sender::make(uint8_t NodeId, uint8_t Duration)
        {
                return gnuradio::get_initial_sptr(new RTS_Sender_impl(NodeId, Duration));
        }

        /*
            * The private constructor
            */
        RTS_Sender_impl::RTS_Sender_impl(uint8_t NodeId, uint8_t Duration)
            : gr::block("RTS_Sender",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex))),
                d_Node_ID(NodeId),
                d_duration(Duration)
        {
            d_in_port = pmt::mp("inmsg");
            d_out_port = pmt::mp("outmsg");
            d_CTS_port = pmt::mp("inCTS");
            message_port_register_in(d_in_port);
            message_port_register_in(d_CTS_port);
            message_port_register_out(d_out_port);

            set_msg_handler(d_in_port, boost::bind(&RTS_Sender_impl::Get_Read, this, _1));
            set_msg_handler(d_CTS_port, boost::bind(&RTS_Sender_impl::HandleCTS, this, _1));
            d_state = S_RTS_WAIT_DATA;

            d_fs_bw_ratio = 2;
            d_sf = 7;
            d_fft_size_factor = 2;
            d_num_symbols = (1 << d_sf);
            d_num_samples = d_fs_bw_ratio * d_num_symbols;
            d_bin_len = d_fft_size_factor * d_num_symbols;
            d_fft_size = d_fft_size_factor * d_num_samples;
            d_fft = new fft::fft_complex(d_fft_size, true, 1);

            // Create local chirp tables.  Each table is 2 chirps long to allow memcpying from arbitrary offsets.
            for (int i = 0; i < d_num_samples; i++)
            {
                    double phase = M_PI / d_fs_bw_ratio * (i - i * i / (float)d_num_samples);
                    d_downchirp.push_back(gr_complex(std::polar(1.0, phase)));
                    d_upchirp.push_back(gr_complex(std::polar(1.0, -phase)));
            }
        }

        /*
            * Our virtual destructor.
            */
        RTS_Sender_impl::~RTS_Sender_impl()
        {
            delete d_fft;
            d_downchirp.clear();
            d_upchirp.clear();
        }

        void
        RTS_Sender_impl::forecast(int noutput_items, gr_vector_int &ninput_items_required)
        {
            /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
            ninput_items_required[0] = noutput_items * (1 << d_sf) * 2;
        }
        float
        RTS_Sender_impl::search_fft_mean_mag(const lv_32fc_t *fft_result, float *buffer1, float *buffer2)
        {
            /*
                *  fft_result: d_fft_size
                *  buffer1: d_fft_size (float)
                *  buffer2: d_bin_len (float)
                */

            volk_32fc_magnitude_32f(buffer1, fft_result, d_fft_size);
            volk_32f_x2_add_32f(buffer2, buffer1, &buffer1[d_fft_size - d_bin_len], d_bin_len);
            float mean_val = 0;
            volk_32f_accumulator_s32f(&mean_val, buffer2, d_bin_len);
            mean_val /= d_bin_len;
            if (mean_val < 0.01)
            {
                    mean_val = 0;
            }
            return mean_val;
        }
        float
        RTS_Sender_impl::CAD_Detect(const lv_32fc_t *fft_result, float *buffer1, float *buffer2)
        {
            float mean_mag = search_fft_mean_mag(fft_result, buffer1, buffer2);
            #if RTS_DEBUG >= RTS_DEBUG_INFO
                std::cout << "mean_mag::  " << mean_mag << std::endl;
            #endif
            return mean_mag;
        }
        void
        RTS_Sender_impl::HandleCTS(const pmt::pmt_t msg)
        {
            if(d_state == S_RTS_WAIT_CTS){
                pmt::pmt_t msg_output(pmt::cdr(msg));
                std::vector<uint8_t> tmp_vector;
                size_t len;
                const uint8_t *fp = pmt::u8vector_elements(msg_output, len);
                for(size_t i = 0;i < len;i++){
                    tmp_vector.push_back(fp[i]);
                }
                std::string CTSmsg = u8vectorToString(tmp_vector,tmp_vector.size());
                /*   CTS msg is = "#CTS#ID#DURATION#";
                *
                */
                #if RTS_DEBUG >= RTS_DEBUG_INFO
                    std::cout<<"CTS MSG is ::"<<CTSmsg<<std::endl;
                #endif
                if(hasSubstring(CTSmsg,"CTS")){
                    std::vector<std::string> strs = splitstring(CTSmsg);
                    if(std::stoi(strs[0]) == d_Node_ID){
                        d_state = S_RTS_SEND_DATA;
                    }else{
                        d_duration = std::stoi(strs[1]);
                        d_state = S_RTS_SLEEP;
                    }
                }
            }
        }
        /*
            * Read and store the user print infomation;
            * Warning：
            *   if  [state] is [S_RTS_WAIT_DATA]
            *   switch [state] to [S_RTS_CAD]
            *   else [state] is not [S_RTS_WAIT_DATA]
            *   just [store] msg to queue;
            */
        void
        RTS_Sender_impl::Get_Read(const pmt::pmt_t msg)
        {
                pmt::pmt_t data(pmt::cdr(msg));
                // std::string s = pmt::symbol_to_string(data);
                d_new_data = data;
                // buffer_strs.push(s);
                #if RTS_DEBUG == RTS_DEBUG_INFO
                    std::cout << "***** MESSAGE DEBUG PRINT ********\n";
                    pmt::print(d_new_data);
                    std::cout << "**********************************\n";
                #endif
                // switch RTS state
                if (d_state == S_RTS_WAIT_DATA)
                {
                    d_state = S_RTS_SEND_RTS;
                }
        }

        int
        RTS_Sender_impl::general_work(int noutput_items,
                                        gr_vector_int &ninput_items,
                                        gr_vector_const_void_star &input_items,
                                        gr_vector_void_star &output_items)
        {
            const gr_complex *in = (const gr_complex *)input_items[0];
            gr_complex *out = (gr_complex *)output_items[0];

            unsigned int num_consumed = d_num_samples;
            gr_complex *up_block = (gr_complex *)volk_malloc(d_fft_size * sizeof(gr_complex), volk_get_alignment());
            float *fft_res_mag = (float *)volk_malloc(d_fft_size * sizeof(float), volk_get_alignment());
            float *fft_res_add = (float *)volk_malloc(d_bin_len * sizeof(float), volk_get_alignment());

            if (up_block == NULL || fft_res_mag == NULL || fft_res_add == NULL)
            {
                std::cerr << "Unable to allocate processing buffer!" << std::endl;
            }

            // Dechirp the incoming signal
            volk_32fc_x2_multiply_32fc(up_block, in, &d_downchirp[0], d_num_samples);

            // Preamble and Data FFT
            // If d_fft_size_factor is greater than 1, the rest of the sample buffer will be zeroed out and blend into the window
            memset(d_fft->get_inbuf(), 0, d_fft_size * sizeof(gr_complex));
            memcpy(d_fft->get_inbuf(), &up_block[0], d_num_samples * sizeof(gr_complex));
            d_fft->execute();

            switch (d_state)
            {
                case S_RTS_SLEEP:
                {
                    break;
                }
                case S_RTS_RESET:
                {
                    break;
                }
                case S_RTS_WAIT_DATA:
                { // callback  GET_Read function to handle input message

                    break;
                }

                case S_RTS_CAD:
                {
                    float cad_detect_mag = CAD_Detect(d_fft->get_inbuf(), fft_res_mag, fft_res_add);
                    // std::cout<<"Cad_detect num:"<<cad_detect<<std::endl;
                    d_argmax_history.push_back(cad_detect_mag);
                    if (d_argmax_history.size() == DETECT_CAD_FRAME_NUM)
                    {
                        double ratio = 0;
                        int positiveCount = 0;
                        for (int i = 0; i < d_argmax_history.size(); i++)
                        {
                            if (positiveCount > DETECR_CAD_MAG)
                            {
                                positiveCount++;
                            }
                        }
                        ratio = 1.0 * positiveCount / d_argmax_history.size();
                        #if RTS_DEBUG >= RTS_DEBUG_INFO
                            std::cout << "CAD ratio::  " << ratio << std::endl;
                        #endif
                        if (ratio  <   0.5)
                        {
                            d_state = S_RTS_SEND_RTS;
                        }
                        else
                        {
                            d_state = S_RTS_SLEEP;
                        }
                    }
                    break;
                }

                case S_RTS_SEND_RTS:
                {
                    // payload(std::string) convert to payloadVector(u8vector);
                    std::string payload;
                    payload.append(std::to_string(d_Node_ID));
                    payload.push_back('#');
                    payload.append(std::to_string(d_duration));
                    pmt::pmt_t message = pmt::string_to_symbol(payload);
                    // size_t pkt_len(0);
                    // const uint8_t * plvector = pmt::u8vector_elements(message,pkt_len);

                    std::vector<uint8_t> myVector(payload.begin(), payload.end());
                    uint8_t *plvector = &myVector[0];

                    // create dict pairs as metadata for this pdu
                    pmt::pmt_t dict = pmt::make_dict();
                    pmt::dict_add(dict, pmt::string_to_symbol("payload"), message);

                    std::vector<uint8_t> symbols;
                    for (uint8_t i = 0; i < payload.size(); i++)
                    {
                        symbols.push_back(plvector[i]);
                    }
                    pmt::pmt_t output = pmt::init_u8vector(symbols.size(), symbols);

                    pmt::pmt_t pdu = pmt::cons(dict, output);

                    message_port_pub(d_out_port, pdu);
                    d_state = S_RTS_SEND_DATA;
                    break;
                }

                case S_RTS_WAIT_CTS:
                {
                    #if USE_CTS_COND == USE_CTS_PACKET
                        std::cout << "COND is packet" << std::endl;
                    #elif USE_CTS_COND == USE_CTS_SYMBOL
                        std::cout << "COND is symbol" << std::endl;
                    #endif
                    break;
                }
                case S_RTS_SEND_DATA:
                {
                    pmt::pmt_t pdu = pmt::cons(pmt::make_dict(), d_new_data);
                    message_port_pub(d_out_port, pdu);
                    d_state = S_RTS_WAIT_DATA;
                    break;
                }
            }

            // Do <+signal processing+>
            // Tell runtime system how many input items we consumed on
            // each input stream.
            consume_each(noutput_items);

            volk_free(up_block);
            volk_free(fft_res_mag);
            volk_free(fft_res_add);
            // Tell runtime system how many output items we produced.
            return noutput_items;
        }
    } /* namespace lora */
} /* namespace gr */
