#ifndef _CACHE_H
#define _CACHE_H

#include "systemc"
#include "sram.h"

#include <iostream>
using namespace std;

extern ofstream mylogfile;
#define sysc_logfile if (0) mylogfile

using namespace sc_core;
using namespace sc_dt;

#define ENABLE_SYSTEMC_CACHE

class Cache : public sc_module
{
	typedef Cache SC_CURRENT_USER_MODULE;

	virtual void work() = 0;
public:
	static const int block_size = 64;
	static const int ports = 1;//2;

	static const int depthLog2 = 7;
	static const int idx_mask = (1<<depthLog2)-1;

	sc_in<bool>          clk;
	sc_in<bool>          reset;

	sc_signal<bool>      port_available[ports];

	sc_in<bool>          in_ena[ports];
	sc_in<bool>          in_is_insert[ports];
	sc_in<bool>          in_has_data[ports];
	sc_in<bool>          in_needs_data[ports];
	sc_in<sc_uint<64> >  in_addr[ports];
	sc_in<sc_uint<64> >  in_data[ports];
	sc_in<bool>          in_update_state[ports];
	sc_in<sc_uint<8> >   in_new_state[ports];
	sc_out<bool>         out_ready[ports];
	sc_out<sc_uint<64> > out_addr[ports];
	sc_out<sc_bv<8*block_size> >   out_data[ports];
	sc_out<sc_uint<64> > out_token[ports];
	sc_out<sc_uint<8> >  out_state[ports];

	Cache(const sc_module_name &name)
	:	sc_module(name)
	{
		SC_METHOD(work); sensitive << clk.pos();
		for(int p=0; p<ports; ++p)
			port_available[p] = false;
	}
};

class CacheProject: public Cache
{

	SRAM<64,8,1> data;
	SRAM<50,8,1> tag;
	bool probe,insert;
	
	void work(){
	
	if(in_needs_data[0].read() && in_ena[0].read() && port_available[0]){
		port_available[0] = false;
		probe = true;
		insert = false;
		tag.ena[0].write(true);
		tag.addr[0].write(in_addr[0].read());
	}
	
	if(port_available[0] == false) && (probe == true)      {

			/* Clock Cycle  3*/
			/* TAG SRAM EN*/
		if(tag.ena[0].read())        {
		   
			 if(tag.dout[0].read()) {
 
				if(tag_matched(tag.dout[0].read())) {

				   tag.ena[0].write(false);
				   data.ena[0].write(true);
				   data.addr[0].write(in_addr[0].read());
		        	} 
		        	else{
			
				    out_token[0].write(in_addr[0].read());
                        	    out_addr[0].write(in_addr[0].read());
                            	    out_ready[0].write(0xff);
                            	    port_available[0] = true;
			         }
		         }		
		  }
	     
			/* Clock Cycle 5 */
			/* DATA SRAM EN*/
		  if(data.ena[0].read()) {

		           if (data.dout[0].read()) {
			    out_data[0].write(data.dout[0].read());
			    out_token[0].write(in_addr[0].read());
			    out_addr[0].write(in_addr[0].read());
			    out_ready[0].write(true);
                            port_available[0] = true;
		           }
	
		  }	
	
                   
	      }





	if(in_needs_data[0].read() && in_ena[0].read() && tag.dout[0].read() && !port_available[0] && tag_matched(tag.dout[0].read()) && data.ena[0]){
				out_data[0].write(data.dout[0].read());in_needs_data[0].read() && in_ena[0].read() && tag.dout[0].read() && !port_available[0] && tag_matched(tag.dout[0].read())
				out_token[0].write(in_addr[0].read());
				out_addr[0].write(in_addr[0].read());
				out_ready[0].write(true);
				port_available[0] = true;
		}
		else{
				out_token[0].write(in_addr[0].read());
                                out_addr[0].write(in_addr[0].read());
                                out_ready[0].write(0xff);
                                port_available[0] = true;
			}
	}
	}
public:
	CacheProject(const char *name): Cache(name), data("data"), tag("tag"){

		data.clk(clk);
		tag.clk(clk);
		for(int p=0;p<Cache::ports;++p){
			data.ena[p](in_enable[p]);
			tag.ena[p](in_enable[p]);

			data.addr[p](in_addr[p]);
			tag.addr[p](in_addr[p]);

			data.din[p](in_data[p]);
			tag.din[p](in_data[p]);

			data.we[p](in_has_data[p]);
			tag.we[p](in_has_data[p]);

			data.dout[p](out_data[p]);
			//tag.dout[p](out_data[p]);
			//data.we[p](out_ready[p]);
		}


	}
	void 




};

#endif
