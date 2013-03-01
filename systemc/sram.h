#ifndef _SRAM_H
#define _SRAM_H

#include "systemc"
using namespace sc_core;
using namespace sc_dt;

template<int width, int depthLog2, int ports>
class SRAM : public sc_module {
	typedef SRAM SC_CURRENT_USER_MODULE;

	static const int delay = (depthLog2-8>0?depthLog2-8:1)*(ports>1?(ports>2?(ports>3?10:2):1.4):1);

	typedef sc_bv<width> Row;
	Row data[1<<depthLog2];
	typedef sc_uint<depthLog2> Addr;

	typedef sc_bv<width> pipeline_t[delay];
	pipeline_t pipeline[ports];
	int pipe_ptr;

	void work() {
		for(int p=0; p<ports; ++p) {
			if (!ena[p]) continue;
			if (we[p]) data[addr[p].read()] = din[p].read();
			(pipeline[p])[pipe_ptr] = data[addr[p].read()];
		}
		pipe_ptr = (pipe_ptr+1) % delay;
		for(int p=0; p<ports; ++p) {
			dout[p].write((pipeline[p])[pipe_ptr]);
		}
	}
public:
	static const int Ports = ports;
	static const int Width = width;
	static const int DepthLog2 = depthLog2;
	static const int Delay = delay;

	sc_in<bool> clk;
	sc_in<bool> ena[ports];
	sc_in<Addr> addr[ports];
	sc_in<Row>  din[ports];
	sc_in<bool> we[ports];
	sc_out<Row> dout[ports];

	SRAM(const sc_module_name &name)
	: sc_module(name)
	{
		SC_METHOD(work); sensitive << clk.pos();
	}
};

#endif
