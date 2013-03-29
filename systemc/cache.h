
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
#define GET_INDEX(x) ((x >> 6) & (0xff))


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

	unsigned long long int time_c;

	Cache(const sc_module_name &name)
	:	sc_module(name)
	{
		SC_METHOD(work); sensitive << clk.pos();
		for(int p=0; p<ports; ++p) {
			port_available[p] = false;
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
			SRAM_DATA_READY,
			END,
			WAIT_FOR_REQ
		    };

	static const int block_offset_bits = 6;
	static const int block_size_bits   = 8 * block_size;
	static const int tag_size_bits     = 64 - depthLog2 - block_offset_bits;
	static const int state_size_bits   = 8;

	struct state_parameter {
		sc_uint<block_size>            prm_addr[ports];
		sc_bv<state_size_bits>         prm_state[ports];
		sc_bv<depthLog2>               prm_index[ports];
		sc_bv<tag_size_bits>           prm_tag[ports];
		bool 		               prm_isstate[ports];
		bool                           is_read[ports];
		bool                           is_write[ports];
		bool                           is_insert[ports];
		int                            next_state[ports];
	}state_var;

	
			/* SRAMs */
        SRAM<block_size_bits,depthLog2,ports> data;
        SRAM<tag_size_bits,depthLog2,ports>   tag;
        SRAM<state_size_bits,depthLog2,ports>      state;


		/* cache controller signals */
	sc_signal<bool>                    tag_ena[ports];
	sc_signal<sc_bv<tag_size_bits> >   tag_din[ports];
	sc_signal<sc_bv<tag_size_bits> >   tag_dout[ports];
	sc_signal<sc_uint<depthLog2> >     tag_addr[ports];
	sc_signal<bool>                    tag_we[ports];

	sc_signal<bool>                    data_ena[ports];
	sc_signal<sc_bv<block_size_bits> > data_din[ports];
	sc_signal<sc_bv<block_size_bits> > data_dout[ports];
	sc_signal<sc_uint<depthLog2> >     data_addr[ports];
	sc_signal<bool>                    data_we[ports];
	
	sc_signal<bool>                     state_ena[ports];
	sc_signal<sc_bv<state_size_bits > > state_din[ports];
	sc_signal<sc_bv<state_size_bits > > state_dout[ports];
	sc_signal<sc_uint<depthLog2 > >     state_addr[ports];
	sc_signal<bool >                    state_we[ports];

       unsigned short delay_cycle[ports];
      #if DEBUG 
         sc_trace_file *wf; 
      #endif

		    /* Cycle Processing Function Based on Current State of Request*/

       void process(int portno) {
				/* Start State */

	if (state_var.next_state[portno] == INIT) {		// common to read, write, insert

			state_var.prm_addr[portno]    = in_addr[portno].read();
			state_var.prm_index[portno]   = GET_INDEX(state_var.prm_addr[portno]);
			state_var.prm_tag[portno]     = GET_TAG(state_var.prm_addr[portno]);
			state_var.prm_isstate[portno] = in_update_state[portno].read();			

			if (state_var.prm_isstate[portno])
			    state_var.prm_state[portno]   = in_new_state[portno].read();
			else
		        	/*throw away the received out state*/
			    state_var.prm_state[portno]   = 0;

				/* Set TAG_SRAM */
			tag_addr[portno].write(state_var.prm_index[portno]);
			tag_ena[portno].write(true);

				/* Set STATE_SRAM */
			state_addr[portno].write(state_var.prm_index[portno]);
			state_ena[portno].write(true);

				/* Set STATE_SRAM */
			delay_cycle[portno]      = 1; 
			state_var.next_state[portno] = TAG_AND_STATE_WAIT;
			#if DEBUG
			cout << " [cache] STATE : INI " << endl << endl;
			#endif
			return ;	
				
	 }
	
	if (state_var.next_state[portno] == TAG_AND_STATE_WAIT) {

			delay_cycle[portno] = 1;
			state_var.next_state[portno] = TAG_AND_STATE_READY;
			#if DEBUG
			cout << " [cache] STATE : TAG_AND_STATE_WAIT " << endl << endl;
			#endif
			return;
	 }

	if (state_var.next_state[portno] == TAG_AND_STATE_READY) {

            	    if (state_var.prm_tag[portno] == tag_dout[portno].read() ) { // TAG MATCH
					// Cache Hit
			#if DEBUG
			    cout << " [cache] STATE : HIT " << endl << endl;
			#endif
			if(state_var.is_read[portno]) {

				state_var.prm_state[portno] = state_dout[portno].read();

				/*disable SRAMS in the nextcycle*/

	     		        tag_ena[portno].write(false);
  	     			state_ena[portno].write(false);

				data_addr[portno].write(state_var.prm_index[portno]);
				data_ena[portno].write(true);

		        	delay_cycle[portno] = 1;
				state_var.next_state[portno] = SRAM_DATA_WAIT;
				#if DEBUG
			           cout << " [cache] STATE : READ HIT " << endl << endl;
				#endif
				return;
			}
					/* Dirty Write */

		        else if(state_var.is_write[portno]) {

				state_var.prm_state[portno] = state_dout[portno].read();

			        if (state_var.prm_isstate[portno])     {
			            state_din[portno].write(state_var.prm_state[portno]);
				    state_addr[portno].write(state_var.prm_index[portno]);
				    state_we[portno].write(true);
				/* We can go ahead with the SRAM Write*/
				}

				data_din[portno].write(in_data[portno].read());
				data_addr[portno].write(state_var.prm_index[portno]);

				data_we[portno].write(true);
				data_ena[portno].write(true);

		        	delay_cycle[portno] = 1;
				state_var.next_state[portno] = SRAM_DATA_WAIT;
				#if DEBUG
			            cout << " [cache] STATE : WRITE HIT " << endl << endl;
				#endif
				return;
			}

			else if(state_var.is_insert[portno]){
					/* Read the evicted block enrty state */

			        if (state_var.prm_isstate[portno])     {
			            state_din[portno].write(state_var.prm_state[portno]);
				    state_addr[portno].write(state_var.prm_index[portno]);
				    state_we[portno].write(true);
			        }
					/*prev tag */
				state_var.prm_state[portno] = state_dout[portno].read();

				data_addr[portno].write(state_var.prm_index[portno]);
				data_din[portno].write(in_data[portno].read());

				data_we[portno].write(true);
				data_ena[portno].write(true);

		        	delay_cycle[portno] = 1;
				state_var.next_state[portno] = SRAM_DATA_WAIT;
				#if DEBUG
			           cout << " [cache] STATE : ISINSERT HIT " << endl << endl;
				#endif
				return;
                        }		
		
   	    }	
			// Cache Miss 
            else {
				/* Read Miss */
			if(state_var.is_read[portno]) {

					/*Disable Bothm SRAM's */
				out_ready[portno].write(true);
				out_data[portno].write(0);
				out_addr[portno].write(tag_dout[portno].read());
		        	out_token[portno].write(state_var.prm_addr[portno]);
				out_state[portno].write(0xff);

				tag_ena[portno].write(false);
				state_ena[portno].write(false);

				state_var.is_read[portno]   = false;
					/*?*/
			        port_available[portno] = true;

		        	delay_cycle[portno] = 1;
				state_var.next_state[portno] = END;
				#if DEBUG
			        cout << " [cache] STATE : READ MISS " << endl << endl;
				#endif
				return;

			}     /* Write Miss */
			else if (state_var.is_write[portno]) {

				/* Out Addr should contain the accessed tag */
				out_data[portno].write(0);
				out_addr[portno].write(tag_dout[portno].read());
		        	out_token[portno].write(state_var.prm_addr[portno]);
				out_state[portno].write(0xff);
				out_ready[portno].write(true);

				tag_ena[portno].write(false);
				state_ena[portno].write(false);

				state_var.is_write[portno]   = false;

		        	delay_cycle[portno] = 1;
				state_var.next_state[portno] = END;
				#if DEBUG
			         cout << " [cache] STATE : WRITE MISS " << endl << endl;
				#endif
				return;

			}	/* Insert */
			else if (state_var.is_insert[portno]) {

					/*evict and write the tag */
				#if DEBUG
				cout << " ***DBG ::TAG and STATE INSERT CHECK***" << endl << endl;
				cout << " Address "    << state_var.prm_addr[portno] << endl
			             << " TAG_ENTRY "  << state_var.prm_tag[portno]   << endl
				     << " Index  "     << state_var.prm_index[portno] << endl
				     << " State  "     << state_var.prm_state[portno] << endl 
				     << " Data  "      << in_data[portno].read() << endl 
				     << endl;
				#endif

			        tag_din[portno].write(state_var.prm_tag[portno]);
				tag_addr[portno].write(state_var.prm_index[portno]);
				tag_we[portno].write(true);
				     /* IS_UPDATE_STATE ALWAYS TRUW FOR IS INSERT */
			        state_din[portno].write(state_var.prm_state[portno]);
				state_addr[portno].write(state_var.prm_index[portno]);
				state_we[portno].write(true);

					/*old tag */
				state_var.prm_tag[portno] = tag_dout[portno].read();
					/*evict the old state */
				state_var.prm_state[portno] = state.dout[portno].read();

                		data_din[portno].write(in_data[portno].read());
                		data_addr[portno].write(state_var.prm_index[portno]);
				data_we[portno].write(true);

				data_ena[portno].write(true);

		        	delay_cycle[portno] = 1;
				state_var.next_state[portno] = SRAM_DATA_WAIT;
				#if DEBUG
			            cout << " [cache] STATE : IS INSERT MISS " << endl << endl;
				#endif
				return;

                         }		

		} // end of Cache Miss


         }

	 if (state_var.next_state[portno] == SRAM_DATA_WAIT) {

             delay_cycle[portno] = 1;
	     state_var.next_state[portno] = SRAM_DATA_READY;
	     #if DEBUG
	         cout << " [cache] STATE : SRAM DATA WAIT " << endl << endl;
	     #endif
	     return;
	 }

	 if(state_var.next_state[portno] == SRAM_DATA_READY) {
        
		/* all the SRAM access time identical */

     	     	 tag_we[portno].write(false);
	     	 tag_ena[portno].write(false);

  	         state_we[portno].write(false);
  	         state_ena[portno].write(false);

		/* Check The TAG AND INDEX ENTRY */

	     #if DEBUG
		cout << " **DBG ::TAG and STATE INSERT VALIDATE**" << endl << endl;
		cout << "Address" << in_addr[portno].read() << endl
		     << "TAG_ENTRY " << tag_dout[portno].read() << endl
		     << "Index  " << tag_addr[portno].read() << endl
		     << "State  " << state_dout[portno].read() << endl
		     << endl
		     << endl;
	     #endif

	         if(state_var.is_read[portno])  {
					/*Write In Blocks to the Cache Ouput*/
		               #if DEBUG
	     			cout << " [cache] STATE : READ SRAM DATA  " << endl << endl;
			        cout << "DBG : DATA_READ " << data_dout[portno].read() << endl
				     << "DBG : INDEX     " << data_addr[portno].read() << endl 
				     << "DBG : EN        " << data_ena[portno].read() << endl
				     << "DBG : TAG       " << state_var.prm_tag[portno] << endl;
				#endif
                		out_data[portno].write(data_dout[portno].read());
				data_ena[portno].write(false);

				out_addr[portno].write(state_var.prm_tag[portno]);
			        out_token[portno].write(state_var.prm_addr[portno]);
				out_state[portno].write(state_var.prm_state[portno]);
				out_ready[portno].write(true);

			        delay_cycle[portno] = 1;
				state_var.next_state[portno] = END;

				return;
	         }			
				
	         if((state_var.is_write[portno]) || (state_var.is_insert[portno])) {

				/* Ouput is Empty */
				#if DEBUG
				cout << " DBG :DATA IN "  << in_data[portno].read() << endl
				     << " ADDRESS       " << in_addr[portno].read() << endl	
				     << " DATA WE       " << data_we[portno].read() << endl
				     << " DATA EN       " << data_ena[portno].read() << endl;	
				#endif
                		out_data[portno].write(0);
				data_we[portno].write(false);
				data_ena[portno].write(false);

				out_addr[portno].write(state_var.prm_tag[portno]);
			        out_token[portno].write(state_var.prm_addr[portno]);
				out_state[portno].write(state_var.prm_state[portno]);
				out_ready[portno].write(true);

			        delay_cycle[portno] = 1;
				state_var.next_state[portno] = END;
				#if DEBUG
	     			cout << " [cache] STATE : WRITE | INSERT SRAM DATA  " << endl << endl;
				#endif
				return;
	         }			


	     }	

	     if (state_var.next_state[portno] == END)   {

			       state_var.is_write[portno]    = false;
			       state_var.is_read[portno]     = false;
			       state_var.is_insert[portno]   = false;
			       out_ready[portno].write(false);
			       port_available[portno] = true;
			       delay_cycle[portno] = 0;	 
			       state_var.next_state[portno] = WAIT_FOR_REQ;
			       return;

	     }

 
           	//SRAM_ACCESS	
	     cout << " DBG :: UNEXPECTED STATE " << endl;

         }


	void work(){ 
        #if DEBUG
 	time_c++;
	cout << " ****************************START OF CACHE CLOCK CYCLE*********************************** " 
	     << time_c << "P Avail "
	     << port_available[0] 
	     << endl << endl;
	cout << "P_IN_ENABLE "  << in_ena[0] << "P_IN_addr " << in_addr[0] << endl << endl;
	cout << "P_READ REQ "   << in_needs_data[0] 
	     << "P_WRITE REQ "  << in_has_data[0] 
	     << "P_INSERT REQ " << in_is_insert[0] 
	     << "P_OUTPUT_READY " << out_ready[0]
	     << endl ;

	#endif

        int i = 0;
        for (i = 0; i < ports; i++) { 

             if(in_ena[i].read() && in_needs_data[i].read())    {
		 state_var.is_read[i]   = true;
		 state_var.is_write[i]  = false;
		 state_var.is_insert[i] = false;
		 port_available[i] = false;
		 delay_cycle[i] = 0;
	         out_ready[i].write(false); 	 
		 state_var.next_state[i] = INIT;
	     }
	     else if (in_ena[i].read() && in_has_data[i].read() && !in_is_insert[i].read())    {	
		 state_var.is_read[i]   = false;
		 state_var.is_write[i]  = true;
		 state_var.is_insert[i] = false;
		 port_available[i] = false;
		 delay_cycle[i] = 0;
	         out_ready[i].write(false); 	 
		 state_var.next_state[i] = INIT;
	     }
	     else if (in_ena[i].read() && in_has_data[i].read() && in_is_insert[i].read())    {	
		 state_var.is_read[i]   = false;
		 state_var.is_write[i]  = false;
		 state_var.is_insert[i] = true;
		 port_available[i] = false;
		 delay_cycle[i] = 0;
	         out_ready[i].write(false); 	 
		 state_var.next_state[i] = INIT;
             }

	     if (delay_cycle[i] > 0)
		 --delay_cycle[i];	

	 	
	     if ((delay_cycle[i] == 0) && (state_var.next_state[i] != WAIT_FOR_REQ))		  	
                 process(i);
	     #if DEBUG	
	     else
		 cout << " DBG :: Delay Remaining " << delay_cycle[i] << "PORT AVAILABLE " 
		      << port_available[i] << endl << endl;	     
	      sc_trace(wf, clk, "clock");
	      sc_trace(wf, out_state[i], "outready");
	     #endif	 
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
	      port_available[i] = true;	
	      tag.ena[i](tag_ena[i]);
	      tag.din[i](tag_din[i]);
	      tag.dout[i](tag_dout[i]);
	      tag.addr[i](tag_addr[i]);
	      tag.we[i](tag_we[i]);

	      data.ena[i](data_ena[i]);
	      data.din[i](data_din[i]);
	      data.dout[i](data_dout[i]);
	      data.addr[i](data_addr[i]);
	      data.we[i](data_we[i]);

	      state.ena[i](state_ena[i]);
	      state.din[i](state_din[i]);
	      state.dout[i](state_dout[i]);
	      state.addr[i](state_addr[i]);
	      state.we[i](state_we[i]);

	      state_var.next_state[i] = WAIT_FOR_REQ;	
	}
	  #if DEBUG
	  wf = sc_create_vcd_trace_file("counter");
	  #endif	
   }
   ~CacheProject() {
          #if DEBUG
	         sc_close_vcd_trace_file(wf);
	  #endif
   }
};


#endif
