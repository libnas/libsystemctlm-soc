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

using namespace sc_core;
using namespace sc_dt;
using namespace std;

#include "trace.h"
#include "iconnect.h"
#include "debugdev.h"

#include "remote-port-tlm.h"
#include "remote-port-tlm-pci-ep.h"

#define XDMA_H2C_OFS 0x0000UL
#define XDMA_C2H_OFS 0x1000UL
#define XDMA_CFG_OFS 0x3000UL
#define XDMA_MAX_SIZE 0x8000UL
#define AXIS_DATA_WIDTH 64
#define AXIS_STOP 1 << 0
#define AXIS_COMPLETED 1 << 1
#define AXIS_EOP 1 << 4
#define MAX_LEN_PER_TRANSFER 1024
#define D(x)
u32 regs[XDMA_MAX_SIZE >> 2];
const int config_bar_pos = 0;
const int bypass_bar_pos = 1;

void init_write(u32 *regs, u32 tmp, u32 addr) {
    memcpy(regs + addr, &tmp, 4);
}

void init_regs(u32 *regs) {
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

template <int NUM_USR_IRQ>
class xdma : public pci_device_base
{
public:
	SC_HAS_PROCESS(xdma);
    sc_in<bool> rst;
    sc_in<bool> clk;
	tlm_utils::simple_initiator_socket<xdma> card_bus;

	/* Interface to toward PCIE.  */
	tlm_utils::simple_target_socket<xdma> config_bar;
	tlm_utils::simple_target_socket<xdma> user_bar;
	tlm_utils::simple_initiator_socket<xdma> dma;
	sc_vector<sc_out<bool> > irq;

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

    xdma(sc_core::sc_module_name name) :
        pci_device_base(name, NR_MMIO_BAR, NR_IRQ),
		rst("rst"),
        clk("clk"),
		card_bus("card_initiator_socket"),
		config_bar("config_bar"),
		user_bar("user_bar"),
		dma("dma"),
		irq("irq", NR_QDMA_IRQ),

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
        init_regs(&regs);
        SC_THREAD(fetch_then_exec);
    }

    void fetch_then_exec() {
        // init vars
        h2c_dsc_byp_ready.write(true);
        bool nxt_clk_h2c = false;
        c2h_dsc_byp_ready.write(true);
        bool nxt_clk_c2h = false;
        bool h2c_idle = true;
        bool c2h_idle = true;

        u64 tmp_h2c_dsc_byp_src_addr = 0;
        u64 tmp_h2c_dsc_byp_dst_addr = 0;
        u32 tmp_h2c_dsc_byp_len = 0;
        u16 tmp_h2c_dsc_byp_ctl = 0;

        u64 tmp_c2h_dsc_byp_src_addr = 0;
        u64 tmp_c2h_dsc_byp_dst_addr = 0;
        u32 tmp_c2h_dsc_byp_len = 0;
        u16 tmp_c2h_dsc_byp_ctl = 0;

        while(true) {
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

            wait(clk.posedge_event() | rst.negedge_event());

            /* h2c_recv_dsc_byp()  */
            if(h2c_dsc_byp_ready.read() && h2c_dsc_byp_load.read()) {
                /*
                tmp_h2c_dsc_byp_src_addr = h2c_dsc_byp_src_addr;
                tmp_h2c_dsc_byp_dst_addr = h2c_dsc_byp_dst_addr;
                tmp_h2c_dsc_byp_len = h2c_dsc_byp_len;
                tmp_h2c_dsc_byp_ctl = h2c_dsc_byp_ctl;
                */
                // delay
                nxt_clk_h2c = true;
            }

            /* c2h_recv_dsc_byp()  */
            if(c2h_dsc_byp_ready.read() && c2h_dsc_byp_load.read()) {
                /*
                tmp_c2h_dsc_byp_src_addr = c2h_dsc_byp_src_addr;
                tmp_c2h_dsc_byp_dst_addr = c2h_dsc_byp_dst_addr;
                tmp_c2h_dsc_byp_len = c2h_dsc_byp_len;
                tmp_c2h_dsc_byp_ctl = c2h_dsc_byp_ctl;
                */
                // delay
                nxt_clk_c2h = true;
            }

            /* h2c_send()  */
            if(!h2c_idle) {
                if(tmp_h2c_dsc_byp_len <= MAX_LEN_PER_TRANSFER) {
                    m_axis_h2c_tlast.write(true);
                    m_axis_h2c_tvalid.write(true);
                    // delay
                    /* Do transfer Write(tmp_h2c_dsc_byp_src_addr, tmp_h2c_dsc_byp_dst_addr, 
                        tmp_h2c_dsc_byp_len)  */
                    h2c_idle = true;
                    m_axis_h2c_tlast.write(false);
                    m_axis_h2c_tvalid.write(false);
                }
                else {
                    m_axis_h2c_tvalid.write(true);
                    // delay
                    /* Do transfer Write(tmp_h2c_dsc_byp_src_addr, tmp_h2c_dsc_byp_dst_addr, 
                        MAX_LEN_PER_TRANSFER)  */
                    tmp_h2c_dsc_byp_src_addr += MAX_LEN_PER_TRANSFER;
                    tmp_h2c_dsc_byp_dst_addr += MAX_LEN_PER_TRANSFER;
                    tmp_h2c_dsc_byp_len -= MAX_LEN_PER_TRANSFER;
                }
            }

            /* c2h_recv()  */
            if(!c2h_idle) {

            }
        }


    }

};
