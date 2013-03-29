#ifndef _TOP_H
#define _TOP_H

#include "systemc"
#include "cache.h"
#include "sram.h"

using namespace sc_core;
using namespace sc_dt;

class Top : public sc_module
{
	typedef Top SC_CURRENT_USER_MODULE;
	
	CacheProject cache;

	void work() {
		for(int p=0; p<Cache::ports; ++p) {
//		    ena[p] = true;
//		    ena[p] = false;
		    cout << " --------TOP ------------------" << endl;
		    cout << " IS_INSERT " << is_insert[p] << endl;
		    cout << " HAS_DATA " << has_data[p] << endl;
		    cout << " NEEDS_DATA " << needs_data[p] << endl;
		    cout << " ADDR " << addr[p] << endl;
//		    cout << " DATA " << data[p] << endl;
		    cout << " UPDATE-STATE " << update_state[p] << endl;
		    cout << " NEW-STATE " << new_state[p] << endl;
		    cout << " REPLY READY " << reply_ready[p] << endl;
		    cout << " REPLY  ADDR " << reply_addr[p] << endl;
                    cout << " REPLY TOKEN : " << reply_token[p].read();
		    cout << " -------------------------" << endl;
		    cout << endl ;
                }
		if (reset) {
			return;
		}
	}
public:
	sc_in<bool>             clk;
	sc_in<bool>             reset;

	sc_out<bool>            port_available[Cache::ports];

	sc_signal<bool>         ena[Cache::ports];
	sc_signal<bool>         is_insert[Cache::ports];
	sc_signal<bool>         has_data[Cache::ports];
	sc_signal<bool>         needs_data[Cache::ports];
	sc_signal<sc_uint<64> > addr[Cache::ports];
	//sc_signal<sc_uint<64> > data[Cache::ports];
	sc_signal<sc_bv<8 * Cache::block_size> > data[Cache::ports];
	sc_signal<bool>         update_state[Cache::ports];
	sc_signal<sc_uint<8> >  new_state[Cache::ports];
	sc_signal<bool>         reply_ready[Cache::ports];
	sc_signal<sc_uint<64> > reply_addr[Cache::ports];
	sc_signal<sc_bv<8*Cache::block_size> > reply_data[Cache::ports];
	sc_signal<sc_uint<64> > reply_token[Cache::ports];
	sc_signal<sc_uint<8> >  reply_state[Cache::ports];

	Top(const sc_module_name &name)
	:	sc_module(name)
	,	cache("cache")
	{
		cache.clk(clk);
		cache.reset(reset);
		for(int p=0; p<Cache::ports; ++p) {
			port_available[p](cache.port_available[p]);
			cache.in_ena[p](ena[p]);
			cache.in_is_insert[p](is_insert[p]);
			cache.in_has_data[p](has_data[p]);
			cache.in_needs_data[p](needs_data[p]);
			cache.in_addr[p](addr[p]);
			cache.in_data[p](data[p]);
			cache.in_update_state[p](update_state[p]);
			cache.in_new_state[p](new_state[p]);
			cache.out_ready[p](reply_ready[p]);
			cache.out_addr[p](reply_addr[p]);
			cache.out_data[p](reply_data[p]);
			cache.out_token[p](reply_token[p]);
			cache.out_state[p](reply_state[p]);
		}

		SC_METHOD(work); sensitive << clk.pos();
	}
};
extern Top top;

#endif
