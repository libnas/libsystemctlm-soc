/*
 * Copyright (c) 2018 Xilinx Inc.
 * Written by Francisco Iglesias.
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

#ifndef SIGNALS_XDMA_H__
#define SIGNALS_XDMA_H__

#include "tlm-bridges/amba.h"

using namespace sc_core;
using namespace sc_dt;
using namespace std;

class XDMASignals : public sc_core::sc_module
{
public:
    /* H2C signals.  */
    sc_signal<bool> m_axis_h2c_tready;
    sc_signal<bool> m_axis_h2c_tlast;
    sc_signal<sc_bv<64> > m_axis_h2c_tdata;
    sc_signal<bool> m_axis_h2c_tvalid;
    sc_signal<sc_bv<64/8> > m_axis_h2c_tuser;
    sc_signal<sc_bv<64/8> > m_axis_h2c_tkeep;

    /* C2H signals.  */
    sc_signal<bool> s_axis_c2h_tready;
    sc_signal<bool> s_axis_c2h_tlast;
    sc_signal<sc_bv<64> > s_axis_c2h_tdata;
    sc_signal<bool> s_axis_c2h_tvalid;
    sc_signal<sc_bv<64/8> > s_axis_c2h_tuser;
    sc_signal<sc_bv<64/8> > s_axis_c2h_tkeep;

    /* H2C descriptor bypass ports.  */
    sc_signal<bool> h2c_dsc_byp_ready;
    sc_signal<bool> h2c_dsc_byp_load;
    sc_signal<sc_bv<64> > h2c_dsc_byp_src_addr;
    sc_signal<sc_bv<64> > h2c_dsc_byp_dst_addr;
    sc_signal<sc_bv<28> > h2c_dsc_byp_len;
    sc_signal<sc_bv<16> > h2c_dsc_byp_ctl;

    /* C2H descriptor bypass ports.  */
    sc_signal<bool> c2h_dsc_byp_ready;
    sc_signal<bool> c2h_dsc_byp_load;
    sc_signal<sc_bv<64> > c2h_dsc_byp_src_addr;
    sc_signal<sc_bv<64> > c2h_dsc_byp_dst_addr;
    sc_signal<sc_bv<28> > c2h_dsc_byp_len;
    sc_signal<sc_bv<16> > c2h_dsc_byp_ctl;

	template<typename T>
	void connect(T *dev)
	{
        dev->m_axis_h2c_tready(m_axis_h2c_tready);
        dev->m_axis_h2c_tlast(m_axis_h2c_tlast);
        dev->m_axis_h2c_tdata(m_axis_h2c_tdata);
        dev->m_axis_h2c_tvalid(m_axis_h2c_tvalid);
        dev->m_axis_h2c_tuser(m_axis_h2c_tuser);
        dev->m_axis_h2c_tkeep(m_axis_h2c_tkeep);

        dev->s_axis_c2h_tready(s_axis_c2h_tready);
        dev->s_axis_c2h_tlast(s_axis_c2h_tlast);
        dev->s_axis_c2h_tdata(s_axis_c2h_tdata);
        dev->s_axis_c2h_tvalid(s_axis_c2h_tvalid);
        dev->s_axis_c2h_tuser(s_axis_c2h_tuser);
        dev->s_axis_c2h_tkeep(s_axis_c2h_tkeep);

        dev->h2c_dsc_byp_ready(h2c_dsc_byp_ready);
        dev->h2c_dsc_byp_load(h2c_dsc_byp_load);
        dev->h2c_dsc_byp_src_addr(h2c_dsc_byp_src_addr);
        dev->h2c_dsc_byp_dst_addr(h2c_dsc_byp_dst_addr);
        dev->h2c_dsc_byp_len(h2c_dsc_byp_len);
        dev->h2c_dsc_byp_ctl(h2c_dsc_byp_ctl);

        dev->c2h_dsc_byp_ready(c2h_dsc_byp_ready);
        dev->c2h_dsc_byp_load(c2h_dsc_byp_load);
        dev->c2h_dsc_byp_src_addr(c2h_dsc_byp_src_addr);
        dev->c2h_dsc_byp_dst_addr(c2h_dsc_byp_dst_addr);
        dev->c2h_dsc_byp_len(c2h_dsc_byp_len);
        dev->c2h_dsc_byp_ctl(c2h_dsc_byp_ctl);
	}

	void Trace(sc_trace_file *f)
	{
		sc_trace(f, m_axis_h2c_tready, m_axis_h2c_tready.name());
        sc_trace(f, m_axis_h2c_tlast, m_axis_h2c_tlast.name());
        sc_trace(f, m_axis_h2c_tdata, m_axis_h2c_tdata.name());
        sc_trace(f, m_axis_h2c_tvalid, m_axis_h2c_tvalid.name());
        sc_trace(f, m_axis_h2c_tuser, m_axis_h2c_tuser.name());
        sc_trace(f, m_axis_h2c_tkeep, m_axis_h2c_tkeep.name());

        sc_trace(f, s_axis_c2h_tready, s_axis_c2h_tready.name());
        sc_trace(f, s_axis_c2h_tlast, s_axis_c2h_tlast.name());
        sc_trace(f, s_axis_c2h_tdata, s_axis_c2h_tdata.name());
        sc_trace(f, s_axis_c2h_tvalid, s_axis_c2h_tvalid.name());
        sc_trace(f, s_axis_c2h_tuser, s_axis_c2h_tuser.name());
        sc_trace(f, s_axis_c2h_tkeep, s_axis_c2h_tkeep.name());

        sc_trace(f, h2c_dsc_byp_ready, h2c_dsc_byp_ready.name());
        sc_trace(f, h2c_dsc_byp_load, h2c_dsc_byp_load.name());
        sc_trace(f, h2c_dsc_byp_src_addr, h2c_dsc_byp_src_addr.name());
        sc_trace(f, h2c_dsc_byp_dst_addr, h2c_dsc_byp_dst_addr.name());
        sc_trace(f, h2c_dsc_byp_len, h2c_dsc_byp_len.name());
        sc_trace(f, h2c_dsc_byp_ctl, h2c_dsc_byp_ctl.name());
        
        sc_trace(f, c2h_dsc_byp_ready, c2h_dsc_byp_ready.name());
        sc_trace(f, c2h_dsc_byp_load, c2h_dsc_byp_load.name());
        sc_trace(f, c2h_dsc_byp_src_addr, c2h_dsc_byp_src_addr.name());
        sc_trace(f, c2h_dsc_byp_dst_addr, c2h_dsc_byp_dst_addr.name());
        sc_trace(f, c2h_dsc_byp_len, c2h_dsc_byp_len.name());
        sc_trace(f, c2h_dsc_byp_ctl, c2h_dsc_byp_ctl.name());
	}

	template<typename T>
	void connect(T& dev)
	{
		connect(&dev);
	}

	XDMASignals(sc_core::sc_module_name name) :
		sc_module(name),

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
	{}

private:
};
#endif
