#ifdef __cplusplus
extern "C" {
#endif
#define main systemc_main
int main(int argc, char **argv, char **envp);
void systemc_clock(void);
void systemc_reset(void);
#ifdef __cplusplus
}

#include <map>
void systemc_request_submit(long long request);
struct systemc_request {
	enum { SYSTEMC_CACHE_PROBE, SYSTEMC_CACHE_INSERT } type;
	bool has_data;
	bool needs_data;
	long long physAddress;
	bool update_state;
	unsigned char new_state;
	bool sent;
	bool reply_ready;
	unsigned char reply_state;
	long long reply_tag;
	systemc_request(): sent(false), reply_ready(false) { }
};
typedef std::map<long long, systemc_request> systemc_request_map;
extern systemc_request_map systemc_requests;
extern int systemc_base_port;
extern bool systemc_port_available[];
#endif
