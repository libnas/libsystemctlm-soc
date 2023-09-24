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
#ifndef PCI_XILINX_XDMA_DUT_H__
#define PCI_XILINX_XDMA_DUT_H__
#define SC_INCLUDE_DYNAMIC_PROCESSES

#include "soc/pci/xilinx/xdma-def.h"
#include <queue>
using namespace sc_core;
using namespace sc_dt;
using namespace std;

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

struct desc_struct {
    uint64_t dsc_byp_src_addr;
    uint64_t dsc_byp_dst_addr;
    uint32_t dsc_byp_len;
    uint16_t dsc_byp_ctl;
};

class xdma_dut : public pci_device_base
{
public:
	SC_HAS_PROCESS(xdma_dut);
    sc_in<bool> resetn;
    sc_in<bool> clk;

    queue<desc_struct> desc_queue_h2c;
    queue<desc_struct> desc_queue_c2h;

    char buffer[4500]; // 4096 bytes buffer
    // Add memory
    char mem[35000]; 

    /* H2C signals.  */
    sc_out<bool> m_axis_h2c_tready;
    sc_in<bool> m_axis_h2c_tlast;
    sc_in<sc_bv<AXIS_DATA_WIDTH> > m_axis_h2c_tdata;
    sc_in<bool> m_axis_h2c_tvalid;
    sc_in<sc_bv<AXIS_DATA_WIDTH/8> > m_axis_h2c_tuser;
    sc_in<sc_bv<AXIS_DATA_WIDTH/8> > m_axis_h2c_tkeep;

    /* C2H signals.  */
    sc_in<bool> s_axis_c2h_tready;
    sc_out<bool> s_axis_c2h_tlast;
    sc_out<sc_bv<AXIS_DATA_WIDTH> > s_axis_c2h_tdata;
    sc_out<bool> s_axis_c2h_tvalid;
    sc_out<sc_bv<AXIS_DATA_WIDTH/8> > s_axis_c2h_tuser;
    sc_out<sc_bv<AXIS_DATA_WIDTH/8> > s_axis_c2h_tkeep;

    /* H2C descriptor bypass ports.  */
    sc_in<bool> h2c_dsc_byp_ready;
    sc_out<bool> h2c_dsc_byp_load;
    sc_out<sc_bv<64> > h2c_dsc_byp_src_addr;
    sc_out<sc_bv<64> > h2c_dsc_byp_dst_addr;
    sc_out<sc_bv<28> > h2c_dsc_byp_len;
    sc_out<sc_bv<16> > h2c_dsc_byp_ctl;

    /* C2H descriptor bypass ports.  */
    sc_in<bool> c2h_dsc_byp_ready;
    sc_out<bool> c2h_dsc_byp_load;
    sc_out<sc_bv<64> > c2h_dsc_byp_src_addr;
    sc_out<sc_bv<64> > c2h_dsc_byp_dst_addr;
    sc_out<sc_bv<28> > c2h_dsc_byp_len;
    sc_out<sc_bv<16> > c2h_dsc_byp_ctl;

    /* Global Vars to store sth.  */
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

    xdma_dut(sc_core::sc_module_name name) :
        pci_device_base(name, NR_MMIO_BAR, NR_IRQ),
		resetn("resetn"),
        clk("clk"),
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
        add_desc(0x2,0x20,32,0x0,0); 
        add_desc(0x20,0x0,32,0x0,1);
        add_desc(0x40,0x50,64,0x0,0);
        add_desc(0x50,0x0,64,0x0,1);

        SC_THREAD(after_clk_h2c);
        SC_THREAD(after_clk_c2h);

        // init vars
        h2c_dsc_byp_load.write(true);
        c2h_dsc_byp_load.write(true);
    }

    void after_clk_h2c() {
        char data[80] = {0};
        uint32_t offset = 0;
        while(true) {
            wait(clk.posedge_event() | resetn.negedge_event());

            /* h2c_recv_dsc_byp()  */
            if(h2c_dsc_byp_ready.read() && h2c_dsc_byp_load.read() && (!desc_queue_h2c.empty())) {
                get_desc(true);
                h2c_dsc_byp_src_addr.write(tmp_h2c_dsc_byp_src_addr);
                h2c_dsc_byp_dst_addr.write(tmp_h2c_dsc_byp_dst_addr);
                h2c_dsc_byp_len.write(tmp_h2c_dsc_byp_len);
                h2c_dsc_byp_ctl.write(tmp_h2c_dsc_byp_ctl);
            }

            /* h2c_send()  */
            // only need to use the h2c_src_addr ans size
            /* Using trans.get_extension(genattr); and genattr->set_eop(true) to
                set up the tlast signal  */
            // Set up 
            if(m_axis_h2c_tready.read() && m_axis_h2c_tvalid.read()) {
                unsigned int pos = 0;
                unsigned int bus_width = AXIS_DATA_WIDTH / 8;
                unsigned int last_byte = get_last_byte();
                unsigned int i;

                for (i = 0; i < bus_width; i++) {
                    if (s_axis_c2h_tkeep.read().bit(i)) {
                        unsigned int firstbit = i * 8;
                        unsigned int lastbit = firstbit + 8 - 1;

                        data[pos++] =
                            s_axis_c2h_tdata.read().range(lastbit, firstbit).to_uint();
                    }
                }

                memcpy((&buffer[0])+offset,&data[0],pos);

                if (tmp_h2c_dsc_byp_len <= MAX_LEN_PER_TRANSFER && tmp_h2c_dsc_byp_len > 0) {
                    // Using tlm2axis bridge b_transport()
                    /* Do transfer Write(tmp_h2c_dsc_byp_src_addr, tmp_h2c_dsc_byp_dst_addr, 
                        tmp_h2c_dsc_byp_len)  */

                    tmp_h2c_dsc_byp_src_addr = 0;
                    tmp_h2c_dsc_byp_dst_addr = 0;
                    tmp_h2c_dsc_byp_len = 0;
                    tmp_h2c_dsc_byp_ctl = 0;
                    offset = 0;

                }
                if (tmp_h2c_dsc_byp_len > MAX_LEN_PER_TRANSFER) {

                    tmp_h2c_dsc_byp_src_addr += MAX_LEN_PER_TRANSFER;
                    // tmp_h2c_dsc_byp_dst_addr += MAX_LEN_PER_TRANSFER;
                    tmp_h2c_dsc_byp_len -= MAX_LEN_PER_TRANSFER;
                    offset += 8;

                }
            }

        }
    }

    void after_clk_c2h() {
        uint64_t data = 0;
        while(true) {
            wait(clk.posedge_event() | resetn.negedge_event());

            /* c2h_recv_dsc_byp()  */
            if(c2h_dsc_byp_ready.read() && c2h_dsc_byp_load.read() && (!desc_queue_c2h.empty())) {
                get_desc(false);
                c2h_dsc_byp_src_addr.write(tmp_c2h_dsc_byp_src_addr);
                c2h_dsc_byp_dst_addr.write(tmp_c2h_dsc_byp_dst_addr);
                c2h_dsc_byp_len.write(tmp_c2h_dsc_byp_len);
                c2h_dsc_byp_ctl.write(tmp_c2h_dsc_byp_ctl);
            }

            // SC_THREAD for axis-slave
            // only need to use the c2h_dst_addr
            /* c2h_recv()  */
            // Using b_transport
            if(s_axis_c2h_tready.read() && s_axis_c2h_tvalid.read()) {
                data = 0;
                if (tmp_c2h_dsc_byp_len <= MAX_LEN_PER_TRANSFER
                    && tmp_c2h_dsc_byp_len > 0) {
                    memcpy(&data,(&mem[0])+tmp_c2h_dsc_byp_src_addr,tmp_c2h_dsc_byp_len);
                    s_axis_c2h_tdata.write(data);
                    s_axis_c2h_tlast.write(true);
                    sc_bv<AXIS_DATA_WIDTH/8> keep;
                    for(int i=0;i<tmp_c2h_dsc_byp_len;i++) keep[i] = true;
                    s_axis_c2h_tkeep.write(keep);
                    wait(delay);
                }
                if (tmp_c2h_dsc_byp_len > MAX_LEN_PER_TRANSFER) {
                    memcpy(&data,(&mem[0])+tmp_c2h_dsc_byp_src_addr,tmp_c2h_dsc_byp_len);
                    s_axis_c2h_tdata.write(data);
                    s_axis_c2h_tlast.write(false);
                    sc_bv<AXIS_DATA_WIDTH/8> keep;
                    for(int i=0;i<8;i++) keep[i] = true;
                    s_axis_c2h_tkeep.write(keep);
                    wait(delay);
                }
                
            }

        }
    }

    unsigned int get_last_byte()
	{
		unsigned int bus_width = AXIS_DATA_WIDTH / 8;
		unsigned int last_byte = 0;
		unsigned int i;

		for (i = 0; i < bus_width; i++) {
			if (m_axis_h2c_tkeep.read().bit(i)) {
				last_byte = i;
			}
		}

		return last_byte;
	}


    void add_desc(uint64_t src_addr, uint64_t dst_addr,
        uint32_t len, uint16_t ctl, bool is_h2c) {
        desc_struct tmp_desc;
        tmp_desc.dsc_byp_src_addr = src_addr;
        tmp_desc.dsc_byp_dst_addr = dst_addr;
        tmp_desc.dsc_byp_len = len;
        tmp_desc.dsc_byp_ctl = ctl;
        if (is_h2c) {
            desc_queue_h2c.push(tmp_desc);
        }
        else {
            desc_queue_c2h.push(tmp_desc);
        }
    }

    void get_desc(bool is_h2c) {
        desc_struct tmp_desc;
        if (is_h2c) {
            tmp_desc = desc_queue_h2c.front();
            desc_queue_h2c.pop();
            tmp_h2c_dsc_byp_ctl = tmp_desc.dsc_byp_ctl;
            tmp_h2c_dsc_byp_src_addr = tmp_desc.dsc_byp_src_addr;
            tmp_h2c_dsc_byp_dst_addr = tmp_desc.dsc_byp_dst_addr;
            tmp_h2c_dsc_byp_len = tmp_desc.dsc_byp_len;
        }
        else {
            tmp_desc = desc_queue_c2h.front();
            desc_queue_c2h.pop();
            tmp_c2h_dsc_byp_ctl = tmp_desc.dsc_byp_ctl;
            tmp_c2h_dsc_byp_src_addr = tmp_desc.dsc_byp_src_addr;
            tmp_c2h_dsc_byp_dst_addr = tmp_desc.dsc_byp_dst_addr;
            tmp_c2h_dsc_byp_len = tmp_desc.dsc_byp_len;
        }
    }


};
#endif
