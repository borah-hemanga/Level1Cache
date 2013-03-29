#include "systemc"
#include "wrapper.h"
#include "top.h"

ofstream mylogfile("mylog.log");
#define sysc_logfile if (0) mylogfile

using namespace sc_core;

sc_time clockcycle(500, SC_PS);

sc_signal<bool> reset;
sc_clock clk("clk", clockcycle);
Top top("top");

int systemc_base_port;
systemc_request_map systemc_requests;
bool systemc_port_available[Cache::ports];
std::map<long long, long long> systemc_cache_data;
void systemc_clock() {
	sc_start(clockcycle);
#ifdef ENABLE_SYSTEMC_CACHE
	systemc_base_port = 0;
	for(int p=0; p<Cache::ports; ++p) {
		top.ena[p] = false;
		systemc_port_available[p] = top.port_available[p].read();
		if (top.reply_ready[p].read()) {
			long long physAddress = top.reply_addr[p].read();
			long long replyToken = top.reply_token[p].read();
			sysc_logfile << "port:" << p << " reply on " << std::hex << replyToken << " physAddress=" << physAddress << endl;
			sysc_logfile << "systemc_requests_waiting(" << dec << systemc_requests.size() << "):";
			for(systemc_request_map::iterator e=systemc_requests.begin(); e!=systemc_requests.end(); ++e) {
				sysc_logfile << " " << std::hex << e->first;
			}
			sysc_logfile << endl;
			systemc_request_map::iterator req = systemc_requests.find(replyToken);
			assert(req != systemc_requests.end());
			req->second.reply_ready = true;
			req->second.reply_state = top.reply_state[p].read();
			if (req->second.reply_state == 0xff) {
				physAddress = 0;
			} else {
				long long physData = top.reply_data[p].read().to_uint64();
				if (req->second.needs_data && systemc_cache_data[replyToken/Cache::block_size] && physData != systemc_cache_data[replyToken/Cache::block_size]) {
					std::cerr << endl << "Address " << std::hex << replyToken << " was supposed to have " << systemc_cache_data[replyToken/Cache::block_size] << ", but the cache returned " << physData << endl;
					assert(0);
				}
			}
			req->second.reply_tag = physAddress;
		}
	}
#endif
}

void systemc_reset() {
	reset = true;
	sc_start(clockcycle/2);
	reset = false;
	sc_start(clockcycle/2);
}

struct cache_ent { long long tag; unsigned char state; };
std::map<long long, cache_ent> cache_contents;

#define rot(x,k) (((x)<<(k)) | ((x)>>(32-(k))))
long long mix_input = 0xfeeddeadbeeff00d;
long long  hashme(long long a) {
	long long b = ~mix_input;
	long long c = ++mix_input;
	a -= c;  a ^= rot(c, 4);  c += b;
	b -= a;  b ^= rot(a, 6);  a += c;
	c -= b;  c ^= rot(b, 8);  b += a;
	a -= c;  a ^= rot(c,16);  c += b;
	b -= a;  b ^= rot(a,19);  a += c;
	c -= b;  c ^= rot(b, 4);  b += a;
	return a;
}

void systemc_request_submit(long long request) {
#ifdef ENABLE_SYSTEMC_CACHE
	systemc_request& req(systemc_requests[request]);

	int p;
	for(p=systemc_base_port; p<Cache::ports; ++p) {
		if (systemc_port_available[p]) break;
	}
	assert(p < Cache::ports);
	if (req.physAddress == 0) {
		req.reply_ready = true;
		req.reply_tag = 0;
		req.reply_state = 0;
		return;
	}
	sysc_logfile << "port:" << p << " trying " << std::hex << req.physAddress << " type=" << req.type << flush << endl;
	top.ena[p] = true;
	top.is_insert[p] = (req.type == systemc_request::SYSTEMC_CACHE_INSERT);
	top.has_data[p] = req.has_data;
	if (req.has_data) {
		assert(!req.needs_data);
		long long value = hashme(req.physAddress);
		systemc_cache_data[req.physAddress/Cache::block_size] = value;
		top.data[p] = value;
		sysc_logfile << "write:" << p << " trying " << std::hex << req.physAddress << " value=" << value << " type=" << req.type << endl;
	} else {
		sysc_logfile << "read:" << p << " trying " << std::hex << req.physAddress << " type=" << req.type << endl;
	}
	top.needs_data[p] = req.needs_data;
	top.addr[p] = req.physAddress;
	top.update_state[p] = req.update_state;
	top.new_state[p] = req.new_state;
	systemc_base_port = p+1;
#endif
}

int sc_main(int argc, char **argv) {
	mylogfile.rdbuf()->pubsetbuf(0,0);

	top.clk(clk);
	top.reset(reset);

	sc_start(SC_ZERO_TIME);
	reset = true;
	
	systemc_main(argc, argv, NULL);
	return 0;
}
