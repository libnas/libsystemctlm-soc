/* Modified by libnas.  */

/*
 * TLM-2.0 model of the Xilinx XDMA.
 *
 * Currently only supports PCIe-AXI brigde mode in tandem with QEMU.
 *
 * Copyright (c) 2020 Xilinx Inc.
 * Written by Edgar E. Iglesias.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef PCI_XILINX_XDMA_DEF_H__
#define PCI_XILINX_XDMA_DEF_H__
#define SC_INCLUDE_DYNAMIC_PROCESSES

#include <stdint.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include "systemc.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "tlm_utils/tlm_quantumkeeper.h"

#include "tlm-modules/pcie-controller.h"
#include "soc/pci/core/pcie-root-port.h"
#include "memory.h"

#include "tlm-bridges/amba.h"
#include "tlm-extensions/genattr.h"
#include "tlm-bridges/tlm2axis-bridge.h"
#include "tlm-bridges/axis2tlm-bridge.h"

using namespace sc_core;
using namespace sc_dt;
using namespace std;

#include "trace.h"
#include "iconnect.h"
#include "debugdev.h"

#include "remote-port-tlm.h"
#include "remote-port-tlm-pci-ep.h"

#define NR_MMIO_BAR 6
#define NR_IRQ 15
#define XDMA_H2C_OFS 0x0000UL
#define XDMA_C2H_OFS 0x1000UL
#define XDMA_CFG_OFS 0x3000UL
#define XDMA_MAX_SIZE 0x8000UL
#define AXIS_DATA_WIDTH 64
#define AXIS_STOP 1 << 0
#define AXIS_COMPLETED 1 << 1
#define AXIS_EOP 1 << 4
#define MAX_LEN_PER_TRANSFER 8
#define D(x)
uint32_t regs[XDMA_MAX_SIZE >> 2];
uint32_t axi_regs[0xA8];
const int config_bar_pos = 0;
const int bypass_bar_pos = 1;

void init_write(uint32_t *regs, uint32_t tmp, uint32_t addr) {
    memcpy(regs + addr, &tmp, 4);
}

void init_regs(uint32_t *regs) {
    memset(regs, 0, XDMA_MAX_SIZE);
    init_write(regs, 0x1FC00003, 0x0000);
    init_write(regs, 0x00010120, 0x004C);
    init_write(regs, 0x1FC00003, 0x1000);
    init_write(regs, 0x00010120, 0x104C);
    init_write(regs, 0x1FC00003, 0x3000);
    init_write(regs, 0x0000FF01, 0x3010);
    init_write(regs, 0x00000001, 0x301C);
    init_write(regs, 0x00000055, 0x3040);
    init_write(regs, 0x00000055, 0x3040);
    init_write(regs, 0x00000055, 0x3044);
    init_write(regs, 0x00004006, 0x304C);
}

class xdma : public pci_device_base
{
public:
	SC_HAS_PROCESS(xdma);
    sc_in<bool> resetn;
    sc_in<bool> clk;

	/* Interface to toward PCIE.  */
	tlm_utils::simple_target_socket<xdma> config_bar;
	tlm_utils::simple_target_socket<xdma> user_bar;
	tlm_utils::simple_initiator_socket<xdma> dma;

    /* H2C0 and C2H0 channels. */
    // tlm_utils::simple_target_socket<xdma> h2c0_channel;
    // tlm_utils::simple_initiator_socket<xdma> c2h0_channel;

    /* H2C signals.  */
    sc_in<bool> m_axis_h2c_tready;
    sc_out<bool> m_axis_h2c_tlast;
    sc_out<sc_bv<AXIS_DATA_WIDTH> > m_axis_h2c_tdata;
    sc_out<bool> m_axis_h2c_tvalid;
    sc_out<sc_bv<AXIS_DATA_WIDTH/8> > m_axis_h2c_tuser;
    sc_out<sc_bv<AXIS_DATA_WIDTH/8> > m_axis_h2c_tkeep;

    /* C2H signals.  */
    sc_out<bool> s_axis_c2h_tready;
    sc_in<bool> s_axis_c2h_tlast;
    sc_in<sc_bv<AXIS_DATA_WIDTH> > s_axis_c2h_tdata;
    sc_in<bool> s_axis_c2h_tvalid;
    sc_in<sc_bv<AXIS_DATA_WIDTH/8> > s_axis_c2h_tuser;
    sc_in<sc_bv<AXIS_DATA_WIDTH/8> > s_axis_c2h_tkeep;

    /* H2C descriptor bypass ports.  */
    sc_out<bool> h2c_dsc_byp_ready;
    sc_in<bool> h2c_dsc_byp_load;
    sc_in<sc_bv<64> > h2c_dsc_byp_src_addr;
    sc_in<sc_bv<64> > h2c_dsc_byp_dst_addr;
    sc_in<sc_bv<28> > h2c_dsc_byp_len;
    sc_in<sc_bv<16> > h2c_dsc_byp_ctl;

    /* C2H descriptor bypass ports.  */
    sc_out<bool> c2h_dsc_byp_ready;
    sc_in<bool> c2h_dsc_byp_load;
    sc_in<sc_bv<64> > c2h_dsc_byp_src_addr;
    sc_in<sc_bv<64> > c2h_dsc_byp_dst_addr;
    sc_in<sc_bv<28> > c2h_dsc_byp_len;
    sc_in<sc_bv<16> > c2h_dsc_byp_ctl;

    /* Global Vars to store sth.  */
    bool nxt_clk_h2c = false;
    bool nxt_clk_c2h = false;
    bool h2c_idle = true;
    bool c2h_idle = true;
    sc_mutex m_mutex;
    uint64_t tmp_h2c_dsc_byp_src_addr = 0;
    uint64_t tmp_h2c_dsc_byp_dst_addr = 0;
    uint32_t tmp_h2c_dsc_byp_len = 0;
    uint16_t tmp_h2c_dsc_byp_ctl = 0;
    uint64_t tmp_c2h_dsc_byp_src_addr = 0;
    uint64_t tmp_c2h_dsc_byp_dst_addr = 0;
    uint32_t tmp_c2h_dsc_byp_len = 0;
    uint16_t tmp_c2h_dsc_byp_ctl = 0;
    sc_time delay = SC_ZERO_TIME;

    // execute in order
    sc_event e_before, e_after_h2c, e_after_c2h;

    xdma(sc_core::sc_module_name name) :
        pci_device_base(name, NR_MMIO_BAR, NR_IRQ),
		resetn("resetn"),
        clk("clk"),
		config_bar("config_bar"), // bar_num = 1
		user_bar("user_bar"), // bar_num = 0
		dma("dma"),
        // h2c0_channel("h2c0_channel"),
        // c2h0_channel("c2h0_channel"),

        m_axis_h2c_tready("m_axis_h2c_tready"),
        m_axis_h2c_tlast("m_axis_h2c_tlast"),
        m_axis_h2c_tdata("m_axis_h2c_tdata"),
        m_axis_h2c_tvalid("m_axis_h2c_tvalid"),
        m_axis_h2c_tuser("m_axis_h2c_tuser"),
        m_axis_h2c_tkeep("m_axis_h2c_tkeep"),

        s_axis_c2h_tready("s_axis_c2h_tready"),
        s_axis_c2h_tlast("s_axis_c2h_tlast"),
        s_axis_c2h_tdata("s_axis_c2h_tdata"),
        s_axis_c2h_tvalid("s_axis_c2h_tvalid"),
        s_axis_c2h_tuser("s_axis_c2h_tuser"),
        s_axis_c2h_tkeep("s_axis_c2h_tkeep"),

        h2c_dsc_byp_ready("h2c_dsc_byp_ready"),
        h2c_dsc_byp_load("h2c_dsc_byp_load"),
        h2c_dsc_byp_src_addr("h2c_dsc_byp_src_addr"),
        h2c_dsc_byp_dst_addr("h2c_dsc_byp_dst_addr"),
        h2c_dsc_byp_len("h2c_dsc_byp_len"),
        h2c_dsc_byp_ctl("h2c_dsc_byp_ctl"),

        c2h_dsc_byp_ready("c2h_dsc_byp_ready"),
        c2h_dsc_byp_load("c2h_dsc_byp_load"),
        c2h_dsc_byp_src_addr("c2h_dsc_byp_src_addr"),
        c2h_dsc_byp_dst_addr("c2h_dsc_byp_dst_addr"),
        c2h_dsc_byp_len("c2h_dsc_byp_len"),
        c2h_dsc_byp_ctl("c2h_dsc_byp_ctl")
    {
        init_regs(&regs[0]);
        // init vars
        h2c_dsc_byp_ready.write(true);
        c2h_dsc_byp_ready.write(true);
        
        // In tg-tlm.h, to generate a transfer and send it to the socket.
        // h2c0_channel.register_b_transport(this, &xdma::h2c_b_transport);
        config_bar.register_b_transport(this, &xdma::config_bar_b_transport);
        user_bar.register_b_transport(this, &xdma::user_bar_b_transport);

        SC_THREAD(before_clk);
        SC_THREAD(after_clk_h2c);
        SC_THREAD(after_clk_c2h);
    }

    void before_clk() {
        wait(SC_ZERO_TIME);
        bool first = true;
        while(true) {
            wait(clk.posedge_event() | resetn.negedge_event());
            if(!first) {
                wait(e_after_h2c & e_after_c2h);
                first = false;
            }

            /* h2c_recv_dsc_byp()  */
            if(nxt_clk_h2c) {
                h2c_dsc_byp_ready.write(false);
                nxt_clk_h2c = false;
            }

            /* c2h_recv_dsc_byp()  */
            if(nxt_clk_c2h) {
                c2h_dsc_byp_ready.write(false);
                nxt_clk_c2h = false;
            }

            /* h2c_send()  */
            if(h2c_idle) {
                h2c_dsc_byp_ready.write(true);
            }

            /* c2h_recv()  */
            if(c2h_idle) {
                c2h_dsc_byp_ready.write(true);
            }

            e_before.notify();

        }
    }

    void after_clk_h2c() {
        while(true) {
            wait(clk.posedge_event() | resetn.negedge_event());
            wait(e_before);

            /* h2c_recv_dsc_byp()  */
            if(h2c_dsc_byp_ready.read() && h2c_dsc_byp_load.read()) {
                nxt_clk_h2c = true;

                tmp_h2c_dsc_byp_src_addr = h2c_dsc_byp_src_addr.read().to_uint64();
                tmp_h2c_dsc_byp_dst_addr = h2c_dsc_byp_dst_addr.read().to_uint64();
                tmp_h2c_dsc_byp_len = h2c_dsc_byp_len.read().to_uint64();
                tmp_h2c_dsc_byp_ctl = h2c_dsc_byp_ctl.read().to_uint64();
            }

            /* h2c_send()  */
            // only need to use the h2c_src_addr ans size
            /* Using trans.get_extension(genattr); and genattr->set_eop(true) to
                set up the tlast signal  */
            // Set up 
            if(!h2c_idle) {
                if(tmp_h2c_dsc_byp_len <= MAX_LEN_PER_TRANSFER) {
                    m_axis_h2c_tlast.write(true);
                    m_axis_h2c_tvalid.write(true);
                    // Using tlm2axis bridge b_transport()
                    /* Do transfer Write(tmp_h2c_dsc_byp_src_addr, tmp_h2c_dsc_byp_dst_addr, 
                        tmp_h2c_dsc_byp_len)  */

                    char data[15] = {0};
                    tlm::tlm_generic_payload trans[2];

                    trans[0].set_command(tlm::TLM_READ_COMMAND);
                    trans[0].set_data_ptr((unsigned char *)&data[0]);
                    trans[0].set_streaming_width(tmp_h2c_dsc_byp_len);
                    trans[0].set_data_length(tmp_h2c_dsc_byp_len);
                    trans[0].set_address(tmp_h2c_dsc_byp_src_addr);

                    // this.dma to forward the request
                    // TBD
                    dma->b_transport(trans[0],delay);

                    trans[1].set_command(tlm::TLM_WRITE_COMMAND);
                    trans[1].set_data_ptr((unsigned char *)&data[0]);
                    trans[1].set_streaming_width(tmp_h2c_dsc_byp_len);
                    trans[1].set_data_length(tmp_h2c_dsc_byp_len);
                    trans[1].set_address(0);
                    genattr_extension *genattr = new genattr_extension();
                    genattr->set_eop(false);
                    trans[1].set_extension(genattr);
                    h2c_tlm2axis(trans[1], delay);

                    tmp_h2c_dsc_byp_src_addr = 0;
                    tmp_h2c_dsc_byp_dst_addr = 0;
                    tmp_h2c_dsc_byp_len = 0;
                    tmp_h2c_dsc_byp_ctl = 0;

                    h2c_idle = true;
                    m_axis_h2c_tlast.write(false);
                    m_axis_h2c_tvalid.write(false);
                }
                else {
                    m_axis_h2c_tvalid.write(true);
                    
                    char data[15] = {0};
                    tlm::tlm_generic_payload trans[2];

                    trans[0].set_command(tlm::TLM_READ_COMMAND);
                    trans[0].set_data_ptr((unsigned char *)&data[0]);
                    trans[0].set_streaming_width(8);
                    trans[0].set_data_length(8);
                    trans[0].set_address(tmp_h2c_dsc_byp_src_addr);

                    // this.dma to forward the request
                    // TBD
                    dma->b_transport(trans[0],delay);

                    trans[1].set_command(tlm::TLM_WRITE_COMMAND);
                    trans[1].set_data_ptr((unsigned char *)&data[0]);
                    trans[1].set_streaming_width(8);
                    trans[1].set_data_length(8);
                    trans[1].set_address(0);
                    genattr_extension *genattr = new genattr_extension();
                    genattr->set_eop(false);
                    trans[1].set_extension(genattr);
                    h2c_tlm2axis(trans[1], delay);

                    tmp_h2c_dsc_byp_src_addr += MAX_LEN_PER_TRANSFER;
                    // tmp_h2c_dsc_byp_dst_addr += MAX_LEN_PER_TRANSFER;
                    tmp_h2c_dsc_byp_len -= MAX_LEN_PER_TRANSFER;

                }
            }

            e_after_h2c.notify();

        }
    }

    void after_clk_c2h() {
        while(true) {
            wait(clk.posedge_event() | resetn.negedge_event());
            wait(e_before);

            /* c2h_recv_dsc_byp()  */
            if(c2h_dsc_byp_ready.read() && c2h_dsc_byp_load.read()) {
                nxt_clk_c2h = true;
                
                tmp_c2h_dsc_byp_src_addr = c2h_dsc_byp_src_addr.read().to_uint64();
                tmp_c2h_dsc_byp_dst_addr = c2h_dsc_byp_dst_addr.read().to_uint64();
                tmp_c2h_dsc_byp_len = c2h_dsc_byp_len.read().to_uint64();
                tmp_c2h_dsc_byp_ctl = c2h_dsc_byp_ctl.read().to_uint64();
            }

            // SC_THREAD for axis-slave
            // only need to use the c2h_dst_addr
            /* c2h_recv()  */
            // Using b_transport
            if(!c2h_idle) {
                if (tmp_c2h_dsc_byp_len <= MAX_LEN_PER_TRANSFER) {
                    s_axis_c2h_tready.write(true);
                    
                    char data[80];
                    tlm::tlm_generic_payload gp;
                    unsigned int pos = 0;
                    unsigned int bus_width = AXIS_DATA_WIDTH / 8;
                    genattr_extension *genattr = new genattr_extension();

                    gp.set_command(tlm::TLM_WRITE_COMMAND);
                    gp.set_address(tmp_c2h_dsc_byp_dst_addr);
                    gp.set_data_ptr(reinterpret_cast<unsigned char*>(data[0]));
                    gp.set_byte_enable_ptr(NULL);
                    gp.set_byte_enable_length(0);
                    gp.set_extension(genattr);

                    if (s_axis_c2h_tvalid.read()) {
                        unsigned int last_byte = get_last_byte();
                        unsigned int i;

                        for (i = 0; i < bus_width; i++) {
                            if (s_axis_c2h_tkeep.read().bit(i)) {
                                unsigned int firstbit = i * 8;
                                unsigned int lastbit = firstbit + 8 - 1;

                                data[pos++] =
                                    s_axis_c2h_tdata.read().range(lastbit, firstbit).to_uint();

                                if (pos == AXIS_DATA_WIDTH) {
                                    if (s_axis_c2h_tlast.read() && i == last_byte) {
                                        genattr->set_eop();
                                    }

                                    run_tlm(gp, pos);
                                    // tlm packets toward 
                                    genattr->set_eop(false);



                                }
                            }
                        }

                        if (m_axis_h2c_tlast.read() && pos > 0) {
                            genattr->set_eop();

                            run_tlm(gp, pos);

                            genattr->set_eop(false);
                        }
                        s_axis_c2h_tready.write(false);
                    }

                }
                else {
                    // TBD
                }
                
            }

            e_after_c2h.notify();
        }
    }


    void h2c_tlm2axis(tlm::tlm_generic_payload& trans, sc_time& delay) {
        unsigned int bus_width = AXIS_DATA_WIDTH / 8;
		uint8_t *data = trans.get_data_ptr();
		unsigned int len = trans.get_data_length();
		unsigned int pos = 0;
		genattr_extension *genattr;
		bool eop = true;

		// Since we're going to do waits in order to wiggle the
		// AXI signals, we need to eliminate the accumulated
		// TLM delay.
		wait(delay, clk.posedge_event() | resetn.negedge_event());
		delay = SC_ZERO_TIME;

		m_mutex.lock();
		// Get end of packet
		trans.get_extension(genattr);
		if (genattr) {
			eop = genattr->get_eop();
		}

		do {
			sc_bv<AXIS_DATA_WIDTH> tmp;
			sc_bv<AXIS_DATA_WIDTH/8> keep;

			for (unsigned int i = 0;
				i < bus_width && pos < len; i++) {
				int firstbit = i*8;
				int lastbit = firstbit + 8-1;

				tmp.range(lastbit, firstbit) = data[pos++];
				keep[i] = true;
			}
			m_axis_h2c_tdata.write(tmp);
			m_axis_h2c_tkeep.write(keep);

			if (pos == len && eop) {
				m_axis_h2c_tlast.write(true);
			}

			m_axis_h2c_tvalid.write(true);

            do {
			    sc_core::wait(clk.posedge_event() | resetn.negedge_event());
		    } while (m_axis_h2c_tready.read() == false && resetn.read() == true);

			/* Abort transaction if reset is asserted. */
			if (resetn.read() == false) {
				trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
				m_mutex.unlock();
				return;
			}

		} while (pos < len);

		m_axis_h2c_tvalid.write(false);
		m_axis_h2c_tlast.write(false);

		trans.set_response_status(tlm::TLM_OK_RESPONSE);

		m_mutex.unlock();
    }

    // Do transaction in uint32_t array regs.
    void config_bar_b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
        tlm::tlm_command cmd = trans.get_command();
		sc_dt::uint64 addr = trans.get_address();
		unsigned char *data = trans.get_data_ptr();
		unsigned int len = trans.get_data_length();
		unsigned char *byte_en = trans.get_byte_enable_ptr();
		unsigned int s_width = trans.get_streaming_width();
		uint32_t v = 0;
        if (byte_en || len > 4 || s_width < len) {
			goto err;
		}
		if (cmd == tlm::TLM_READ_COMMAND) {
			v = regs[addr >> 2];
            memcpy(data, &v, len);
		}
        if (cmd == tlm::TLM_WRITE_COMMAND) {
            memcpy(&v, data, len);
            regs[addr >> 2] = v;
        }
        trans.set_response_status(tlm::TLM_OK_RESPONSE);
		return;
    err:
		SC_REPORT_WARNING("xdma",
				"unsupported read / write on the config bar");
		trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
		return;
	}

    void user_bar_b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
        tlm::tlm_command cmd = trans.get_command();
		sc_dt::uint64 addr = trans.get_address();
		unsigned char *data = trans.get_data_ptr();
		unsigned int len = trans.get_data_length();
		unsigned char *byte_en = trans.get_byte_enable_ptr();
		unsigned int s_width = trans.get_streaming_width();
		uint32_t v = 0;
        if (byte_en || len > 4 || s_width < len) {
			goto err;
		}
		if (cmd == tlm::TLM_READ_COMMAND) {
			v = axi_regs[addr >> 2];
            memcpy(data, &v, len);
		}
        if (cmd == tlm::TLM_WRITE_COMMAND) {
            memcpy(&v, data, len);
            axi_regs[addr >> 2] = v;
        }
        trans.set_response_status(tlm::TLM_OK_RESPONSE);
		return;
    err:
		SC_REPORT_WARNING("xdma",
				"unsupported read / write on the config bar");
		trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
		return;
    }

    unsigned int get_last_byte()
	{
		unsigned int bus_width = AXIS_DATA_WIDTH / 8;
		unsigned int last_byte = 0;
		unsigned int i;

		for (i = 0; i < bus_width; i++) {
			if (s_axis_c2h_tkeep.read().bit(i)) {
				last_byte = i;
			}
		}

		return last_byte;
	}

    void run_tlm(tlm::tlm_generic_payload& gp, unsigned int &pos)
	{

		gp.set_data_length(pos);
		gp.set_streaming_width(pos);
		gp.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

		dma->b_transport(gp, delay);

		s_axis_c2h_tready.write(false);
		wait(delay);
		s_axis_c2h_tready.write(true);

		pos = 0;
	}



};
#endif
