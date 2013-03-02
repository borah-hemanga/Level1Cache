#ifndef _CACHE_H
#define _CACHE_H

#include "systemc"
#include "sram.h"

#include <iostream>
#include <string.h>
using namespace std;

extern ofstream mylogfile;
#define sysc_logfile if (0) mylogfile

using namespace sc_core;
using namespace sc_dt;

#define ENABLE_SYSTEMC_CACHE

#define GET_TAG(x)   (x >> 14)
#define GET_INDEX(x) ((x >> 6) && (0xff))


class Cache : public sc_module
{
	typedef Cache SC_CURRENT_USER_MODULE;

	virtual void work() = 0;
public:
	static const int block_size = 64;
	static const int ports = 1;

	static const int depthLog2 = 8;
	static const int idx_mask = (1<<depthLog2)-1;

	sc_in<bool>          clk;
	sc_in<bool>          reset;

	sc_signal<bool>      port_available[ports];

	sc_in<bool>          in_ena[ports];
	sc_in<bool>          in_is_insert[ports];
	sc_in<bool>          in_has_data[ports];
	sc_in<bool>          in_needs_data[ports];
	sc_in<sc_uint<64> >  in_addr[ports];
	//sc_in<sc_uint<64> >  in_data[ports];
	sc_in<sc_bv<8*block_size> > in_data[ports];
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
		for(int p=0; p<ports; ++p) {
			port_available[p] = true;
                }
	}
};

class CacheProject: public Cache
{

	enum STATE {
			INIT,
			TAG_AND_STATE_WAIT,
			TAG_AND_STATE_READY,
			SRAM_DATA_WAIT,
			SRAM_DATA_READY
		    };

	struct state_parameter {
		unsigned long int prm_addr[ports];
		unsigned long int prm_index[ports];
		unsigned long int prm_tag[ports];
		unsigned long int prm_state[ports];
		bool prm_isstate[ports];
		bool is_read[ports];
		bool is_write[ports];
		bool is_insert[ports];
		int  next_state[ports];
	}state;

	
	static const int block_offset_bits = 6;
	static const int block_size_bits   = 8 * block_size;
	static const int tag_size_bits     = 64 - depthLog2 - block_offset_bits;
	static const int state_size_bits   = 8;


			/* SRAMs */
        SRAM<block_size_bits,depthLog2,ports> data;
        SRAM<tag_size_bits,depthLog2,ports>   tag;
        SRAM<state_bits,depthLog2,ports>      state;


		/* cache controller signals */
	sc_signal<bool> tag_ena[ports];
	sc_signal<sc_bv<block_size_bits> > tag_data[ports];
	sc_signal<sc_uint<depthLog2> > tag_addr[ports];
	sc_signal<sc_uint<depthLog2> > tag_addr[ports];
	sc_signal<bool> tag_we[ports];

	sc_signal<bool> data_ena[ports];
	sc_signal<sc_bv<tag_size_bits> > data_data[ports];
	sc_signal<sc_uint<depthLog2> > data_addr[ports];
	sc_signal<bool> data_we[ports];
	
	sc_signal<bool> state_ena[ports];
	sc_signal<sc_bv<state_size_bits > > state_data[ports];
	sc_signal<sc_uint<depthLog2 > > state_addr[ports];
	sc_signal<bool > state_we[ports];

       unsigned short delay_cycle[ports] = 0;

       void process(int portno) {

	if (state.next_state[portno] == INIT) {		// common to read, write, insert

			state.prm_addr[portno]    = in_addr[portno].read();
			state.prm_index[portno]   = GET_INDEX(state.prm_addr[portno]);
			state.prm_tag[portno]     = GET_TAG(state.prm_addr[portno]);
			state.prm_isstate[portno] = in_update_state[portno].read();			

			if (state.prm_isstate[portno])
			    state.prm_state[portno]   = in_new_state[portno].read();
			else
			    state.prm_state[portno]   = 0;

				/* Set TAG_SRAM */
			tag_addr[portno].write(state.prm_index[portno]);
			tag_ena[portno]            = true;

				/* Set STATE_SRAM */
			state_addr[portno]        = state.prm_index[portno];
			state_ena[portno]          = true;

				/* Set STATE_SRAM */
			delay_cycle[portno]      = 1; 
			state.next_state[portno] = TAG_AND_STATE_WAIT;

			return ;	
				
	 }
	
	if (state.next_state[portno] == TAG_AND_STATE_WAIT) {

			delay_cycle[portno] = 1;
			state.next_state[portno] = TAG_AND_STATE_READY;
			return;
	 }

	if (state.next_state[portno] == TAG_AND_STATE_READY) {

            	    if (state.prm_tag[portno] == tag_data[portno] ) { // TAG MATCH
					// Cache Hit

			if(state.isread[portno]) {

				state.prm_state[portno] = state.din[portno];

				/*disable SRAMS in the nextcycle*/

				data_addr[portno] = state.prm_index[portno];
				data_ena[portno]  = true;

		        	delay_cycle[portno] = 1;
				state.next_state[portno] = DATA_SRAM_WAIT;
				return;
			}
					/* Dirty Write */
		        else if(state.iswrite[portno]) {
				state.prm_state[portno] = state.din[portno];

			        if (state.prm_isstate[portno])     {
			            state.din[portno] = state.prm_state[portno];
				    state_addr[portno] = state.prm_index[portno];
				    state_we[portno]   = true;

				}

				data_addr[portno] = state.prm_index[portno];
				data.din[portno] = in_data[portno];

				data_we[portno]   = true;
				data_ena[portno]  = true;

		        	delay_cycle[portno] = 1;
				state.next_state[portno] = SRAM_DATA_WAIT;
				return;
			}

			else if(state.isinsert[portno]){
				state.prm_state[portno] = state.din[portno];

			        if (state.prm_isstate[portno])     {
			            state.din[portno] = state.prm_state[portno];
				    state_addr[portno] = state.prm_index[portno];
				    state_we[portno]   = true;
			        }
		
				data_addr[portno] = state.prm_index[portno];
				data.din[portno] = in_data[portno];

				data_we[portno]   = true;
				data_ena[portno]  = true;

		        	delay_cycle[portno] = 1;
				state.next_state[portno] = DATA_SRAM_WAIT;
				return;
                        }		
		
   	    }	
			// Cache Miss 
            else {
				/* Read Miss */
			if(state[portno].isread) {

				tag_ena[portno] = false;
				state_ena[portno] = false;
				out_addr[portno].write(tag_data[portno]);
		        	out_token[portno].write(state.prm_addr[portno]);
				out_state[portno].write(0xff);
				out_ready[portno].write(true);
				port_available[portno] = true;
				state[portno].isread   = false;
		        	delay_cycle[portno] = 0;
				state.next_state[portno] = INIT;
				return;

			}     /* Write Miss */
			else if (state[portno].iswrite) {

				tag_ena[portno] = false;
				state_ena[portno] = false;
		        	out_token[portno].write(state.prm_addr[portno]);
				out_state[portno].write(0xff);
				out_ready[portno].write(true);
				port_available[portno] = true;
				state[portno].isread   = false;
		        	delay_cycle[portno] = 0;
				state.next_state[portno] = INIT;
				return;

			}	/* Insert */
			else if (state[portno].isinsert) {

			        tag_data[portno] = state.prm_tag[portno];
				tag_addr[portno] = state.prm_index[portno];
				tag_we[portno]   = true;
				     /* IS_UPDATE_STATE ALWAYS TRUW FOR IS INSERT */
			        state.din[portno] = state.prm_state[portno];
				state_addr[portno] = state.prm_index[portno];
				state_we[portno]   = true;
			
				out_state[portno].write(state.din[portno]);
		        	delay_cycle[portno] = 1;
				state.next_state[portno] = SRAM_DATA_WAIT
				return;

                         }		

		} // end of Cache Miss


         }

	 if (state.next_state[portno] == SRAM_DATA_WAIT) {

     	     tag_we[portno]     = false;
	     tag_ena[portno]    = false;
  	     state_we[portno]   = false;
  	     state_ena[portno]  = false;

             delay_cycle[portno] = 1;
	     state.next_state[portno] = SRAM_DATA_READY;
	     return;
	 }

	 if(state.next_state[portno] == SRAM_DATA_READY) {

	         if(state.isread[portno])  {
					/*Write In Blocks*/
                		out_data[portno].write(data.din[portno]);
				data_en[portno] = false;

				out_addr[portno].write(state.prm_tag[portno]);
			        out_token[portno].write(state.prm_addr[portno]);
				out_state[portno].write(state.prm_state[portno]);
				out_ready[portno].write(true);

				port_available[portno] = true;

			        delay_cycle[portno] = 0;
				state.next_state[portno] = INIT;
				return;
	         }			
				
	         if((state.iswrite[portno]) || (state.isinsert[portno])) {

                		out_data[portno].write(0);
				data_we[portno] = false;
				data_en[portno] = false;

				out_addr[portno].write(state.prm_tag[portno]);
			        out_token[portno].write(state.prm_addr[portno]);
				out_state[portno].write(state.prm_state[portno]);
				out_ready[portno].write(true);

				port_available[portno] = true;
			        delay_cycle[portno] = 0;
				state.next_state[portno] = INIT;
				return;
	         }			


	     }		
	     cout << " DBG :: UNEXPECTED STATE " << endl;

         }


	void work(){ 

        int i = 0;
        for (i = 0; i < ports; i++) { 

             if(in_ena[i].read() && in_needs_data[i].read())    {
		 state.is_read[i]   = true;
		 state.is_write[i]  = false;
		 state.is_insert[i] = false;
		 port_available[i] = false;
		 delay_cycle[i] = 0;
	         out_ready[i].write(false); 	 
		 state.next_state[portno] = INIT;
	     }
	     else if (in_ena[i].read() && in_has_data[i].read())    {	
		 state.is_read[i]   = false;
		 state.is_write[i]  = true;
		 state.is_insert[i] = false;
		 port_available[i] = false;
		 delay_cycle[i] = 0;
	         out_ready[i].write(false); 	 
		 state.next_state[i] = INIT;
	     }
	     else if (in_ena[i].read() && in_is_insert[i].read())    {	
		 state.is_read[i]   = false;
		 state.is_write[i]  = false;
		 state.is_insert[i] = true;
		 port_available[i] = false;
		 delay_cycle[i] = 0;
	         out_ready[i].write(false); 	 
		 state.next_state[i] = INIT;
             }

	     if (delay_cycle[i] > 0)
		 --delay_cycle[i];	

	 	
	     if (delay_cycle[i] == 0)		  	
                 process(i);
	     else
		 cout << " DBG :: Delay Remaining " << delay_cycle[i] << endl << endl;	     
	  }
        }
public:
	CacheProject(const char *name): Cache(name),data("data"),tag("tag"),state("state"){

	int i = 0;
	data.clk(clk);
	tag.clk(clk);
	state.clk(clk);

		/* Signal Binding */
	for ( i = 0; i < ports; i++ ) {
	      tag.ena[i](tag_ena[i]);
	      tag.data[i](tag_data[i]);
	      tag.addr[i](tag_addr[i]);
	      tag.we[i](tag_we[i]);

	      data.ena[i](data_ena[i]);
	      data.din[i](data_data[i]);
	      data.addr[i](data_addr[i]);
	      data.we[i](data_we[i]);

	      state.ena[i](state_ena[i]);
	      state.din[i](state_datain[i]);
	      state.addr[i](state_addr[i]);
	      state.we[i](state_we[i]);
	}
   }
};


#endif
