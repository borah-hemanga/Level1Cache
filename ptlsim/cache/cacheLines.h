
/*
 * MARSSx86 : A Full System Computer-Architecture Simulator
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
 *
 * Copyright 2009 Avadh Patel <apatel@cs.binghamton.edu>
 * Copyright 2009 Furat Afram <fafram@cs.binghamton.edu>
 *
 */

#ifndef CACHE_LINES_H
#define CACHE_LINES_H

#include <logic.h>
#include "systemc/wrapper.h"

extern ofstream mylogfile;
#define cacheline_logfile if (0) mylogfile

namespace Memory {

    struct CacheLine
    {
        W64 tag;
        /* This is a generic variable used by all caches to represent its
         * coherence state */
        W8 state;

        CacheLine(): tag(-1), state(0) { }
        CacheLine(W64 _tag, W8 _state): tag(_tag), state(_state) { }

        void init(W64 tag_t) {
            tag = tag_t;
            if (tag == (W64)-1) state = 0;
        }

        void reset() {
            tag = -1;
            state = 0;
        }

        void invalidate() { reset(); }

        void print(ostream& os) const {
            os << "Cacheline: tag[", (void*)tag, "] ";
            os << "state[", state, "] ";
        }
    };

    static inline ostream& operator <<(ostream& os, const CacheLine& line)
    {
        line.print(os);
        return os;
    }

    static inline ostream& operator ,(ostream& os, const CacheLine& line)
    {
        line.print(os);
        return os;
    }

    // A base struct to provide a pointer to CacheLines without any need
    // of a template
    struct CacheLinesBase
    {
        public:
            virtual void init()=0;
            virtual W64 tagOf(W64 address)=0;
            virtual int latency() const =0;
            virtual CacheLine* probe(MemoryRequest *request, bool update_state, W8 new_state, bool& ready)=0;
            virtual CacheLine* insert(MemoryRequest *request, W64& oldTag, W8 new_state, W64 new_tag, bool& ready)=0;
            virtual int invalidate(MemoryRequest *request)=0;
            virtual bool has_fast_path()=0;
            virtual bool get_port(MemoryRequest *request)=0;
            virtual void print(ostream& os) const =0;
            virtual int get_line_bits() const=0;
            virtual int get_access_latency() const=0;
            virtual int get_size() const=0;
            virtual int get_set_count() const=0;
            virtual int get_way_count() const=0;
            virtual int get_line_size() const=0;
    };

    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        class CacheLines : public CacheLinesBase,
        public AssociativeArray<W64, CacheLine, SET_COUNT,
        WAY_COUNT, LINE_SIZE>
    {
        private:
            int readPortUsed_;
            int writePortUsed_;
            int readPorts_;
            int writePorts_;
            bool systemc_proxy;
            W64 lastAccessCycle_;
            CacheLine templine;

        public:
            typedef AssociativeArray<W64, CacheLine, SET_COUNT,
                    WAY_COUNT, LINE_SIZE> base_t;
            typedef FullyAssociativeArray<W64, CacheLine, WAY_COUNT,
                    NullAssociativeArrayStatisticsCollector<W64,
                    CacheLine> > Set;

            CacheLines(int readPorts, int writePorts);
            void init();
            W64 tagOf(W64 address);
            int latency() const { return LATENCY; };
            CacheLine* probe(MemoryRequest *request, bool update_state, W8 new_state, bool& ready);
            CacheLine* insert(MemoryRequest *request, W64& oldTag, W8 new_state, W64 new_tag, bool& ready);
            int invalidate(MemoryRequest *request);
            bool has_fast_path();
            bool get_port(MemoryRequest *request);
            void print(ostream& os) const;

            /**
             * @brief Get Cache Size
             *
             * @return Size of Cache in bytes
             */
            int get_size() const {
              return (SET_COUNT * WAY_COUNT * LINE_SIZE);
            }

            /**
             * @brief Get Number of Sets in Cache
             *
             * @return Sets in Cache
             */
            int get_set_count() const {
              return SET_COUNT;
            }

            /**
             * @brief Get Cache Lines per Set (Number of Ways)
             *
             * @return Number of Cache Lines in one Set
             */
            int get_way_count() const {
              return WAY_COUNT;
            }

            /**
             * @brief Get number of bytes in a cache line
             *
             * @return Number of bytes in Cache Line
             */
            int get_line_size() const {
              return LINE_SIZE;
            }

            int get_line_bits() const {
                return log2(LINE_SIZE);
            }

            int get_access_latency() const {
                return LATENCY;
            }
    };

    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        static inline ostream& operator <<(ostream& os, const
                CacheLines<SET_COUNT, WAY_COUNT, LINE_SIZE, LATENCY>&
                cacheLines)
        {
            cacheLines.print(os);
            return os;
        }

    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        static inline ostream& operator ,(ostream& os, const
                CacheLines<SET_COUNT, WAY_COUNT, LINE_SIZE, LATENCY>&
                cacheLines)
        {
            cacheLines.print(os);
            return os;
        }

    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        CacheLines<SET_COUNT, WAY_COUNT, LINE_SIZE, LATENCY>::CacheLines(int readPorts, int writePorts) :
            readPorts_(readPorts)
            , writePorts_(writePorts)
            , systemc_proxy(false)
    {
        lastAccessCycle_ = 0;
        readPortUsed_ = 0;
        writePortUsed_ = 0;
        if (writePorts < 0) {
          writePorts_ = -writePorts_;
          systemc_proxy = true;
        }
    }

    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        void CacheLines<SET_COUNT, WAY_COUNT, LINE_SIZE, LATENCY>::init()
        {
            foreach(i, SET_COUNT) {
                Set &set = base_t::sets[i];
                foreach(j, WAY_COUNT) {
                    set.data[j].init(-1);
                }
            }
        }

    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        W64 CacheLines<SET_COUNT, WAY_COUNT, LINE_SIZE, LATENCY>::tagOf(W64 address)
        {
            return floor(address, LINE_SIZE);
        }


    // Return true if valid line is found, else return false
    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        CacheLine* CacheLines<SET_COUNT, WAY_COUNT, LINE_SIZE, LATENCY>::probe(MemoryRequest *request, bool update_state, W8 new_state, bool& ready)
        {
            W64 physAddress = request->get_physical_address();
            if (systemc_proxy) {
              cacheline_logfile << "probe: " << *request << " update_state: " << update_state << " new_state: " << (int)new_state << " physAddress=" << physAddress << endl;
              systemc_request& req(systemc_requests[physAddress]);
              if (!req.sent) {
                  cacheline_logfile << "sending probe..." << endl;
                  req.type = systemc_request::SYSTEMC_CACHE_PROBE;
                  OP_TYPE type = request->get_type();
                  req.has_data = ((type == MEMORY_OP_WRITE) || (type == MEMORY_OP_UPDATE)) ;
                  req.needs_data = (type == MEMORY_OP_READ);
                  req.physAddress = physAddress;
                  req.update_state = update_state;
                  req.new_state = new_state;
                  systemc_request_submit(physAddress);
                  req.sent = true;
                  ready = false;
                  return NULL;
              }
              ready = req.reply_ready;
              cacheline_logfile << "sent, ready? " << ready << " reply_tag: " << std::hex << req.reply_tag << endl;
              templine.tag = req.reply_tag;
              templine.state = req.reply_state;
              if (ready) {
                req.sent = false;
                systemc_requests.erase(physAddress);
                cacheline_logfile << "systemc_requests.size() == " << systemc_requests.size() << endl;
              }
              if (templine.tag == 0) return NULL;
              if (ready) cacheline_logfile << "... hit" << endl;
              return &templine;
            }
            CacheLine *line = base_t::probe(physAddress);

            ready = true;
            return line;
        }

    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        CacheLine* CacheLines<SET_COUNT, WAY_COUNT, LINE_SIZE, LATENCY>::insert(MemoryRequest *request, W64& oldTag, W8 new_state, W64 new_tag, bool& ready)
        {
            W64 physAddress = request->get_physical_address();
            if (systemc_proxy) {
              cacheline_logfile << "insert: " << *request << " new_state: " << (int)new_state << endl;
              systemc_request& req(systemc_requests[physAddress]);
              if (!req.sent) {
                  cacheline_logfile << "sending insert..." << endl;
                  req.type = systemc_request::SYSTEMC_CACHE_INSERT;
                  req.has_data = true;
                  req.needs_data = false;
                  req.physAddress = physAddress;
                  req.update_state = true;
                  req.new_state = new_state;
                  systemc_request_submit(physAddress);
                  req.sent = true;
                  ready = false;
                  return NULL;
              }
              ready = req.reply_ready;
              cacheline_logfile << "sent, ready? " << ready << " reply_tag: " << std::hex << req.reply_tag << endl;
              templine.tag = oldTag = req.reply_tag;
              templine.state = req.reply_state;
              if (ready) {
                req.sent = false;
                systemc_requests.erase(physAddress);
                assert(systemc_requests.size() < 1000);
                cacheline_logfile << "systemc_requests.size() == " << systemc_requests.size() << endl;
              }
              return &templine;
            }
            CacheLine *line = base_t::select(physAddress, oldTag);

            ready = true;
            return line;
        }

    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        int CacheLines<SET_COUNT, WAY_COUNT, LINE_SIZE, LATENCY>::invalidate(MemoryRequest *request)
        {
            assert(!systemc_proxy);
            return base_t::invalidate(request->get_physical_address());
        }

    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        bool CacheLines<SET_COUNT, WAY_COUNT, LINE_SIZE, LATENCY>::has_fast_path()
        {
            return !systemc_proxy;
        }

    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        bool CacheLines<SET_COUNT, WAY_COUNT, LINE_SIZE, LATENCY>::get_port(MemoryRequest *request)
        {
            bool rc = false;

            if (systemc_proxy) {
                for(int p=systemc_base_port; p<(readPorts_+writePorts_); ++p) {
                    if (systemc_port_available[p])  {
cacheline_logfile << "getport: " << *request << " available on " << p << endl;
												return true;
										}
								}
cacheline_logfile << "getport: " << *request << " all busy" << endl;
                return false;
            }

            if(lastAccessCycle_ < sim_cycle) {
                lastAccessCycle_ = sim_cycle;
                writePortUsed_ = 0;
                readPortUsed_ = 0;
            }

            switch(request->get_type()) {
                case MEMORY_OP_READ:
                    rc = (readPortUsed_ < readPorts_) ? ++readPortUsed_ : 0;
                    if (!rc) {
                        rc = (writePortUsed_ < writePorts_) ? ++writePortUsed_ : 0;
                    }
                    break;
                case MEMORY_OP_WRITE:
                case MEMORY_OP_UPDATE:
                case MEMORY_OP_EVICT:
                    rc = (writePortUsed_ < writePorts_) ? ++writePortUsed_ : 0;
                    break;
                default:
                    memdebug("Unknown type of memory request: " <<
                            request->get_type() << endl);
                    assert(0);
            };

            return rc;
        }

    template <int SET_COUNT, int WAY_COUNT, int LINE_SIZE, int LATENCY>
        void CacheLines<SET_COUNT, WAY_COUNT, LINE_SIZE, LATENCY>::print(ostream& os) const
        {
            foreach(i, SET_COUNT) {
                const Set &set = base_t::sets[i];
                foreach(j, WAY_COUNT) {
                    os << set.data[j];
                }
            }
        }

};

#endif // CACHE_LINES_H
