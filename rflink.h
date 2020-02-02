// vim:ts=4:sw=4:tw=80:et
/*
  rflink.h

  Header of rflink.cpp
*/

/*
  Copyright 2020 Sébastien Millet

  rflink is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  rflink is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program. If not, see
  <https://www.gnu.org/licenses>.
*/

#ifndef _RFLINK_H
#define _RFLINK_H

//#define ERR_STRINGS

// If DEBUG is activated (said differently: if you uncomment RFLINK_DEBUG), then
// you got to include the debug library (debug.h, debug.cpp) in the compilation.
//#define RFLINK_DEBUG
//#define RFLINK_DEBUG_EVENTTIMER
//#define RFLINK_DEBUG_EVENTTIMER_ONLY

// Don't uncomment the below unless you know what you are doing...
//#define DEBUG_KEEP_SENDING_EVEN_AFTER_RECEIVING_ACK

#include <Arduino.h>

#define ENFORCE_MAX_TASK_COUNT_AT_COMPILE_TIME
#define DEFAULT_MAX_TASK_COUNT                15
#define DEFAULT_PRE_ALLOCATE                   0
#define PKTID_CACHE_SIZE                      10

// Delays below are in milliseconds
#define DEFAULT_RECEIVE_DATA_AVAIL_DELAY     900
#define DEFAULT_RECEIVE_PURGE_DELAY         1000
#define DEFAULT_RECEIVE_TIMEOUT_DELAY          0
#define DEFAULT_SEND_PURGE_DELAY            1000
// The below value makes 49 hours.
#define CACHE_PKTID_DISCARD_DELAY      176400000

#define MIN_DEVICE_RESET_DELAY              1000

#define POST_DEVICE_RESET_DELAY                1

#define ERR_OK                                 0
#define ERR_DEVICE_NOT_REGISTERED              1
#define ERR_SEND_FUNC_NOT_REGISTERED           2
#define ERR_RECEIVE_FUNC_NOT_REGISTERED        3
#define ERR_SEND_DATA_LEN_ABOVE_LIMIT          4
#define ERR_SEND_IO                            5
#define ERR_SEND_BAD_ARGUMENTS                 6
#define ERR_SEND_NO_ACK_RCVD                   7
#define ERR_TASK_CREATED_OK                    8
#define ERR_UNABLE_TO_CREATE_TASK              9
#define ERR_UNKNOWN_TASKID                    10
#define ERR_UNDEFINED                         11
#define ERR_TASK_UNDERWAY                     12
#define ERR_TIMEOUT                           13

// NOTE
// rflink.cpp assumes an address is 1-byte.
// If not, rflink.cpp code MUST be reviewed.
typedef uint8_t address_t;
static_assert(sizeof(address_t) == 1,
  "don't know how to handle not 1-byte addresses");
#define ASSUME_DEVICE_ADDRESS_IS_ONE_BYTE // rflink.cpp will not compile if this
                                          // define does not exist
#define ADDR_BROADCAST                      0xFF

typedef uint16_t pktid_t;

typedef uint16_t taskid_t;

// "m" like milliseconds
typedef long unsigned int mtime_t;

struct Header {
    /*
     *  WARNING
     *
     *  On CC1101, the first byte MUST be the destination address.
     */
    address_t dst;
    address_t src;
    uint8_t flags;
    pktid_t pktid;
    uint8_t len;
};

#define FLAG_NONE 0
#define FLAG_SIN  (1 << 0)
#define FLAG_ACK  (1 << 1)

struct Packet {
    Header header;
    // 'data' member is never used neither "malloc'ed".
    // Used to extract address of first byte of data AFTER packet header.
    //
    // This position can be referred to as:
    //   &pkt->header + 1
    // (therefore, without the need for the below definition)
    //
    // However the writer of this code finds it clearer to use instead:
    //   &pkt->data
    uint8_t data;
};

class RFLink;

class PktKeeper {
    private:
        Packet *pkt;

    public:
        PktKeeper();
        PktKeeper(byte pkt_len);
        ~PktKeeper();

        void release_data();

        void copy_packet(const PktKeeper* pktkeeper);
        bool check_rcvd_pkt_is_ok(const RFLink *link, byte nb_bytes);

        void prepare_for_sending(const RFLink *link, Header* header,
                                 const void *data);

        const Header* get_header_ptr() const;
        byte get_flags();
        void set_flags(byte arg_flags);

        const Packet* get_pkt_ptr_ro() const;
        // Yes, getting rid of the below would be cleaner, but to the expense of
        // some dynamic memory (used as intermediate buffer), or, more
        // complexity (ask PktKeeper class to manage reception).
        Packet* notrecommended_get_pkt_ptr();

        byte get_pkt_len() const;
        const void* get_data_ptr() const;
        byte get_data_len() const;

        void copy_data(void *buf, byte buf_len, byte* rec_len) const;
        void reduce_packet_to_its_header();
};

typedef enum {
    OPT_ADDRESS = 0,
    OPT_SNIF_MODE,
    OPT_EMISSION_POWER
} opt_t;

typedef struct {
    bool used;
    address_t src;
    mtime_t mtime;
    pktid_t last_pktid_seen;
} cache_pktid_t;

enum {
    ST_NOTHING = 0,
    ST_SEND,
    ST_SEND_DONE,
    ST_RECEIVE,
    ST_RECEIVE_DATA_AVAILABLE,
    ST_RECEIVE_DATA_RETRIEVED,
    ST_RECEIVE_TIMEDOUT,
    ST_FINISHED,
    ST_LAST
};

#define T_NONE      0
#define T_EVWAKEUP  (1 << 0)
#define T_EVPKTRCVD (1 << 1)

class Task {
    friend class RFLink;

    private:
        Task* next;

        taskid_t taskid;
        byte status;

        PktKeeper pktkeeper;

        mtime_t mtime_ref;

        mtime_t mtime_wakeup;
        byte last_retcode;

        byte nb_send_schedules;
        const mtime_t* send_schedule_ptr;
        byte send_schedule_pos;

        unsigned char evtsub_wakeup    :1;
        unsigned char evtsub_pktrcvd   :1;

        unsigned char is_an_ack        :1;
        unsigned char need_ack         :1;
        unsigned char has_received_ack :1;
        unsigned char unattended       :1;

        unsigned char rcv_from_1sender :1;

        address_t addr_rcv_from_1sender;

        byte nbsend;
};

struct RXConfig {
    unsigned char def_sender     :1;
    unsigned char def_timeout    :1;
    unsigned char def_rxcallback :1;
    address_t sender;
    mtime_t timeout;
    void (*rxcallback)(byte res, void* buf, byte buf_len, byte* rec_len);
};

struct RFLinkFunctions {
    void (*deviceInit)(byte *max_data_len, bool reset_only);
    byte (*deviceSend)(const void* data, byte len);
    byte (*deviceReceive)(void* buf, byte buf_len);
    void (*deviceSetOpt)(opt_t opt, void* data, byte len);

    void (*setInterrupt)(void (*func)());
    void (*resetInterrupt)();

    RFLinkFunctions();
};

class RFLink {
    private:

// Variables

        byte max_payload_len;

        // The below is NOT about Arduino' instructions noInterrupts() and
        // interrupts().
        // It is all about the RF device' interrupt being armed, or not.

        // Are interrupts armed?
        //   true:  interrupts are armed
        //          = execution of attachInterrupt()
        //   false: interrupts are not armed
        //          = execution of detachInterrupt()
        //          = also default situation (if none of attachInterrupt() and
        //            detachInterrupt() got called)
        unsigned char interrupt_is_attached :1;

        unsigned char device_addr_has_been_defined :1;
        unsigned char pre_allocate :1;

        unsigned char auto_sleep :1;

        address_t device_addr;

        pktid_t last_pktid;
        taskid_t last_taskid;

        mtime_t receive_data_avail_delay;
        mtime_t receive_purge_delay;
        mtime_t send_purge_delay;

        mtime_t last_device_reset;

        PktKeeper *recpkt;

        byte task_count;
        byte max_task_count;

        // Will gracefully manage packet ids (that is, discard a given packet if
        // id already seen for a given source), up to as many different sources.
        cache_pktid_t cache_pktids[PKTID_CACHE_SIZE];

#ifndef ENFORCE_MAX_TASK_COUNT_AT_COMPILE_TIME
        Task* tskhead;
#else
        Task tskhead[DEFAULT_MAX_TASK_COUNT];
#endif

        RFLinkFunctions funcs;

// Member-functions

        // "Arm" device interruptions
        void interrupts_on();
        // "Unarm" device interruptions
        void interrupts_off();

        void task_destroy(Task* tsk);
        void task_initialize(Task* tsk);
        Task* task_create(byte status);

        bool check_pktid_already_seen(address_t src, pktid_t pktid);

        Task* get_task_by_taskid(taskid_t taskid);

        void initialize_recpkt_if_necessary();

    public:

        RFLink(byte maxtask = DEFAULT_MAX_TASK_COUNT,
               unsigned char prealloc = DEFAULT_PRE_ALLOCATE);
        ~RFLink();

        void register_funcs(const RFLinkFunctions* arg_funcs);
        static byte get_header_len();

        byte get_max_payload_len() const;
        byte get_pkt_max_size() const;

        const char* get_err_string(byte errcode) const;

        void set_opt(opt_t opt, void* data, byte len);
        void set_opt_byte(opt_t opt, byte value);

        void set_auto_sleep(bool v);

        void do_events();

        byte send_noblock(taskid_t* taskid, address_t dst,
                          const void* data, byte len, bool ack);
        byte send_ack_noblock(taskid_t* taskid, Header* h);
        byte send_get_final_status(taskid_t taskid, byte *nbsend = nullptr);
        void send_ack(Task* tsk);
        byte send(address_t dst, const void* data, byte len, bool ack,
                  byte *nbsend = nullptr);

        byte tev_wakeup(Task* tsk);
        byte tev_received(Task* tsk, PktKeeper* pk, bool pktid_already_seen,
                          bool* pkt_consumed);

        byte receive_noblock(taskid_t* taskid, RXConfig* cfg = nullptr);
        byte data_retrieve(Task* tsk, void* buf, byte buf_len, byte* rec_len,
                           address_t* sender);
        byte receive(void* buf, byte buf_len, byte* rec_len,
                     address_t* sender = nullptr, RXConfig* cfg = nullptr);

        void data_retrieved_post(Task* tsk);
        byte task_get_status(taskid_t taskid);

        void delay_ms(long int d);

#ifdef RFLINK_DEBUG
        void dbg_print_status(bool is_eligible_for_sleep);
#endif

};

#endif // _RFLINK_H

