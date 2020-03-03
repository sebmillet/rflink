// vim:ts=4:sw=4:tw=80:et
/*
  rflink.cpp

  Manages communication with radio frequence device.
  Made initially for CC1101, although CC1101-specific code should be found
  inside cc1101wrapper only.

  This communication management consists in:
    - Providing a way to guarantee data reception with request of ACK
    - Repeats sending (until ACK received or timeout)
    - Avoid data duplication on receiver side by using unique packet ids
    - Provides "blocking" and "non blocking" API
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

#include <Arduino.h>
#include <avr/sleep.h>

#include "rflink.h"

// NOTE
// The timings below are NOT cumulative: they all are defined as the delay (in
// milliseconds) since the sending started.

// Wrapper' sending schedule without expecting an ACK
const mtime_t snd_sched[] = {
//    0, 300, 600
//    0, 400, 900, 2000, 2800, 3900
    0, 200, 550, 900
};
const byte snd_sched_len = (sizeof(snd_sched) / sizeof(*snd_sched));

// Wrapper' sending schedule, expecting an ack
// *IMPORTANT NOTE*
// At all timings EXCEPT THE LAST ONE, does a sending occur.
// Because we need a delay to wait for a response ACK, this last timing is used
// for this delay.
const mtime_t snd_expack_sched[] = {
//    0, 280, 750, 850
//    0, 100, 300, 600, 1000, 3000, 3100, 3800, 3900
    0, 100, 450, 800, 900
};
const byte snd_expack_sched_len =
                    (sizeof(snd_expack_sched) / sizeof(*snd_expack_sched));

// Wrapper' ACK sending schedule
// You may (but I didn't see an interest for it) use the array below as a way
// to:
//   - Shift initial ACK send, in case a device would need a small delay between
//     tx and rx (seems not needed by CC1101).
//   - Repeat ACK sendings to increase the likelihood of ACK reception by the
//     original packet sender.
//
//     For example you can set the array to:
//       { 0, 100, 200 }
//     in order to send three ACK with 100 milliseconds of delay between each.
//
//     Or (2nd example), you can set it to:
//       { 50, 250 }
//     to wait an intial delay of 50 milliseconds before first ACK sending, then
//     send a second ACK 200 milliseconds after the first ACK got sent.
//
//   - For now the array { 0 } means, just send one ACK as soon as a packet is
//     received.
const mtime_t snd_repack_sched[] = { 0 };
const byte snd_repack_sched_len =
        (sizeof(snd_repack_sched) / sizeof(*snd_repack_sched));

#ifndef RFLINK_DEBUG

#include <assert.h>

#define dbg(a)
#define dbgf(...)
#define dbgbin(a, b, c)

#define ET_REG(...)
#define ET_STRINGS(a, b)
#define ET_PRTPERIOD(a)

#else

#include "debug.h"

#endif

#if !defined(RFLINK_DEBUG) || !defined(RFLINK_DEBUG_EVENTTIMER)

#define ET_REG(...)
#define ET_STRINGS(a, b)
#define ET_PRTPERIOD(a)

#else

#ifdef RFLINK_DEBUG_EVENTTIMER_ONLY

#undef dbg
#define dbg(a)
#undef dbgf
#define dbgf(...)
#undef dbgbin
#define dbgbin(a, b, c)

#endif // RFLINK_DEBUG_EVENTTIMER_ONLY

static EventTimer ET;
#define ET_REG(...)      ET.ev_reg(__VA_ARGS__)
#define ET_STRINGS(a, b) ET.ev_set_all_strings(a, b)
#define ET_PRTPERIOD(a)  ET.ev_print_by_period(a)

// Don't use zero for a real event (it is used to mark array entries as being
// unused).
enum {
    EV_NONE = 0,                     // Never use it
    EV_SEND_CALL,
    EV_SENT_OK,
    EV_SENT_NOTOK,
    EV_RECEIVE_CALL,
    EV_RECEIVED_OK,
    EV_RECEIVED_NOTOK,
    EV_RECEIVED_0_BYTE_RCVD
};

// *IMPORTANT*
// NEVER LONGER THAN EV_STRING_MAX_LENGTH
const char ev00[] PROGMEM = "";                       // EV_NONE
const char ev01[] PROGMEM = "[>>-- send call";        // EV_SEND_CALL
const char ev02[] PROGMEM = "-->>] sent ok";          // EV_SENT_OK
const char ev03[] PROGMEM = "!!!>] ** SENT NOTOK **"; // EV_SENT_NOTOK
const char ev04[] PROGMEM = "[<<-- rcvd call";        // EV_RECEIVE_CALL
const char ev05[] PROGMEM = "--<<] rcvd ok";          // EV_RECEIVED_OK
const char ev06[] PROGMEM = "!!!<] ** RCVD NOTOK **"; // EV_RECEIVED_NOTOK
const char ev07[] PROGMEM = "----] rcvd empty";       // EV_RECEIVED_0_BYTE_RCVD

const char *const ev_string_table[] PROGMEM = {
    ev00, ev01, ev02, ev03, ev04, ev05, ev06, ev07
};

#endif // !defined(RFLINK_DEBUG) || !defined(RFLINK_DEBUG_EVENTTIMER);

static bool interrupted = false;
static void interrupt_func() {
    interrupted = true;
}

#ifdef ERR_STRINGS

#define ERR_STRING_MAX_LENGTH 50

const char erUN[] PROGMEM = "unknown error code";
// ERR_OK
const char er00[] PROGMEM = "no error";
// ERR_DEVICE_NOT_REGISTERED
const char er01[] PROGMEM = "device not registered";
// ERR_SEND_FUNC_NOT_REGISTERED
const char er02[] PROGMEM = "send funct. not registered";
// ERR_RECEIVE_FUNC_NOT_REGISTERED
const char er03[] PROGMEM = "receive funct. not registered";
// ERR_SEND_DATA_LEN_ABOVE_LIMIT
const char er04[] PROGMEM = "send data length above limit";
// ERR_SEND_IO
const char er05[] PROGMEM = "send I/O error";
// ERR_SEND_BAD_ARGUMENTS
const char er06[] PROGMEM = "bad send arguments";
// ERR_SEND_NO_ACK_RCVD
const char er07[] PROGMEM = "no ack received";
// ERR_TASK_CREATED_OK
const char er08[] PROGMEM = "task created ok";
// ERR_UNABLE_TO_CREATE_TASK
const char er09[] PROGMEM = "unable to create task";
// ERR_UNKNOWN_TASKID
const char er10[] PROGMEM = "unknown taskid";
// ERR_UNDEFINED
const char er11[] PROGMEM = "undefined (no return code available)";
// ERR_TASK_UNDERWAY
const char er12[] PROGMEM = "task is underway";
// ERR_TIMEOUT
const char er13[] PROGMEM = "timeout";

const char *const err_string_table[] PROGMEM = {
    er00, er01, er02, er03, er04, er05, er06, er07, er08, er09, er10, er11,
    er12, er13
};

#define ERR_STRING_TABLE_LEN \
  (sizeof(err_string_table) / sizeof(*err_string_table))

char err_string_buffer[ERR_STRING_MAX_LENGTH + 1];

#else

const char err_string_buffer[] = "error strings not available";

#endif // ERR_STRINGS

static inline mtime_t get_current_time() {
    return millis();
}

static uint8_t to_flags(byte seq, byte opt) {
    return ((seq & 0x0F) << 4) | (opt & 0x0F);
}

static void from_flags(byte flags, byte* seq, byte* opt) {
    *opt = (flags & 0x0F);
    *seq = flags >> 4;
}


//
// Tasks
//

void RFLink::task_destroy(Task* tsk_to_destroy) {
    if (!pre_allocate) {
#ifndef ENFORCE_MAX_TASK_COUNT_AT_COMPILE_TIME
        if (tsk_to_destroy == tskhead) {
            tskhead = tsk_to_destroy->next;
        } else {
            Task* tsk = tskhead;
            Task* previous_tsk = tsk;
            while (tsk != tsk_to_destroy && tsk != nullptr) {
                previous_tsk = tsk;
                tsk = tsk->next;
            }
            assert(tsk);
            previous_tsk->next = tsk_to_destroy->next;
        }

        delete tsk_to_destroy;
#endif
    } else {
        tsk_to_destroy->pktkeeper.release_data();
        task_initialize(tsk_to_destroy);
    }

    --task_count;
}

void RFLink::task_initialize(Task* tsk) {
    tsk->taskid = 0;
    tsk->status = ST_NOTHING;
    tsk->evtsub_wakeup = 0;
    tsk->evtsub_pktrcvd = 0;
    tsk->last_retcode = ERR_UNDEFINED;
}

Task* RFLink::task_create(byte status) {
    if (task_count >= max_task_count)
        return nullptr;

    Task* tsk;

    if (!pre_allocate) {

#ifndef ENFORCE_MAX_TASK_COUNT_AT_COMPILE_TIME
        tsk = new Task;
        if (!tsk)
            return nullptr;

        if (!tskhead) {
            tskhead = tsk;
        } else {
            Task* tskwalker = tskhead;
            Task* previous_tskwalker = tskwalker;
            while (tskwalker != nullptr) {
                previous_tskwalker = tskwalker;
                tskwalker = tskwalker->next;
            }
            assert(!previous_tskwalker->next);
            previous_tskwalker->next = tsk;
        }
        tsk->next = nullptr;
#endif

    } else {

        tsk = tskhead;
        while (tsk != nullptr && tsk->status != ST_NOTHING)
            tsk = tsk->next;
        if (!tsk)
            return nullptr;

    }

    task_initialize(tsk);

    tsk->taskid = ++last_taskid;
    tsk->status = status;
    tsk->mtime_ref = get_current_time();

    tsk->is_an_ack = 0;
    tsk->need_ack = 0;
    tsk->has_received_ack = 0;
    tsk->unattended = 0;

    tsk->nbsend = 0;

    ++task_count;

    return tsk;
}

//
// RFLink
//

RFLinkFunctions::RFLinkFunctions():
    deviceInit(nullptr),
    deviceSend(nullptr),
    deviceReceive(nullptr),
    deviceSetOpt(nullptr),
    setInterrupt(nullptr),
    resetInterrupt(nullptr) {

}

RFLink::RFLink(byte maxtask, unsigned char prealloc):
      max_payload_len(0),
      interrupt_is_attached(0),
      device_addr_has_been_defined(0),
      pre_allocate(prealloc),
      auto_sleep(0),
      device_addr(0x00),
      last_pktid(0),
      last_taskid(0),
      receive_data_avail_delay(DEFAULT_RECEIVE_DATA_AVAIL_DELAY),
      receive_purge_delay(DEFAULT_RECEIVE_PURGE_DELAY),
      send_purge_delay(DEFAULT_SEND_PURGE_DELAY),
      last_device_reset(0),
      recpkt(nullptr),
      task_count(0),
      max_task_count(maxtask)
#ifndef ENFORCE_MAX_TASK_COUNT_AT_COMPILE_TIME
      ,tskhead(nullptr)
#endif
{

    for (byte i = 0; i < PKTID_CACHE_SIZE; ++i) {
        cache_pktids[i].used = false;
    }

#if defined(RFLINK_DEBUG) && defined(RFLINK_DEBUG_EVENTTIMER)
    ET_STRINGS(ev_string_table,
      sizeof(ev_string_table) / sizeof(*ev_string_table));
#endif

#ifdef ENFORCE_MAX_TASK_COUNT_AT_COMPILE_TIME
    maxtask = DEFAULT_MAX_TASK_COUNT;
    pre_allocate = 1;
#else
    if (!maxtask)
        maxtask = DEFAULT_MAX_TASK_COUNT;
#endif

    if (pre_allocate) {

        dbg("PREALLOCATE = 1");

#ifndef ENFORCE_MAX_TASK_COUNT_AT_COMPILE_TIME
        // Defensive programming
        assert(tskhead == nullptr);
        // Defensive programming
        assert(max_task_count >= 1);

        tskhead = new Task[max_task_count];
#endif

        for (byte i = 0; i < max_task_count; ++i) {
            tskhead[i].next =
              (i < max_task_count - 1) ? &tskhead[i + 1] : nullptr;
            task_initialize(&tskhead[i]);
        }
    } else {
        dbg("preallocate = 0");
    }
}

RFLink::~RFLink() {
    if (recpkt)
        delete recpkt;

    if (!pre_allocate) {
        while (tskhead)
            task_destroy(tskhead);
    } else {
#ifndef ENFORCE_MAX_TASK_COUNT_AT_COMPILE_TIME
        for (Task* tsk = tskhead; tsk != nullptr; tsk = tsk->next) {
            task_destroy(tsk);
        }
        delete []tskhead;
#endif
    }
}

void RFLink::register_funcs(const RFLinkFunctions* arg_funcs) {
    funcs = *arg_funcs;

    if (!funcs.deviceInit)
        return;

    byte max_data_len;
    (*funcs.deviceInit)(&max_data_len, false);
    max_payload_len = max_data_len - sizeof(Header);

    if (pre_allocate)
        initialize_recpkt_if_necessary();
}

byte RFLink::get_header_len() {
    return sizeof(Header);
}

byte RFLink::get_pkt_max_size() const {
    return sizeof(Header) + max_payload_len;
}

byte RFLink::get_max_payload_len() const {
    return max_payload_len;
}

const char* RFLink::get_err_string(byte errcode) const {

#ifdef ERR_STRINGS
    if (errcode < ERR_STRING_TABLE_LEN) {
        strcpy_P(err_string_buffer,
          (char*)pgm_read_word(&(err_string_table[errcode])));
    } else {
        strcpy_P(err_string_buffer, (char*)erUN);
    }
#endif // ERR_STRINGS

    return err_string_buffer;
}

void RFLink::initialize_recpkt_if_necessary() {
    if (!recpkt) {
        dbg("********** INITIALIZED RECPKT");
        recpkt = new PktKeeper(get_pkt_max_size());
    }
}

byte RFLink::tev_received(Task* tsk, PktKeeper* pk,
                          bool pktid_already_seen, bool* pkt_consumed) {
    assert(!*pkt_consumed);

    Header hbackup = (*pk->get_header_ptr());
    byte ret = tsk->status;

    byte seq;
    byte opt;
    from_flags(pk->get_flags(), &seq, &opt);

    if (opt & FLAG_ACK) {
        if ((tsk->status == ST_SEND || tsk->status == ST_SEND_DONE)) {
            if (tsk->need_ack && !tsk->has_received_ack) {
                if (tsk->pktkeeper.get_header_ptr()->pktid
                      == pk->get_header_ptr()->pktid) {

#ifndef DEBUG_KEEP_SENDING_EVEN_AFTER_RECEIVING_ACK
                    tsk->has_received_ack = 1;

                    if (tsk->status == ST_SEND) {
                        tsk->mtime_wakeup =
                          get_current_time() + send_purge_delay;
                        ret = ST_SEND_DONE;
                    }

                    // We received ACK: we therefore don't need to keep whole
                    // packet any longer.
                    tsk->pktkeeper.reduce_packet_to_its_header();

                    *pkt_consumed = true;
#endif

                }
            }
        }
        return ret;
    }

    if (tsk->status == ST_RECEIVE && !pktid_already_seen) {

        tsk->pktkeeper.copy_packet(pk);
        tsk->last_retcode = ERR_OK;
        *pkt_consumed = true;
        ret = ST_RECEIVE_DATA_AVAILABLE;
        tsk->evtsub_wakeup = 1;
        tsk->mtime_ref = get_current_time();
        tsk->mtime_wakeup = tsk->mtime_ref + receive_data_avail_delay;

    } else if (tsk->status == ST_RECEIVE_DATA_AVAILABLE
               || tsk->status == ST_RECEIVE_DATA_RETRIEVED) {

        if (tsk->pktkeeper.get_header_ptr()->pktid == hbackup.pktid
            && tsk->pktkeeper.get_header_ptr()->src == hbackup.src) {
            *pkt_consumed = true;

            if (tsk->status == ST_RECEIVE_DATA_RETRIEVED) {
                send_ack(tsk);
            }

        }
    }

    return ret;
}

byte RFLink::tev_wakeup(Task* tsk) {

    if (tsk->status == ST_SEND) {
        if (!tsk->need_ack
             || tsk->send_schedule_pos < tsk->nb_send_schedules - 1) {
            tsk->nbsend++;
            ET_REG(EV_SEND_CALL);
            byte r = (*funcs.deviceSend)(
                       tsk->pktkeeper.get_pkt_ptr_ro(),
                       tsk->pktkeeper.get_pkt_len()
                     );

            tsk->last_retcode = r;

#ifdef RFLINK_DEBUG

#ifndef RFLINK_DEBUG_EVENTTIMER_ONLY
            const Header* h = tsk->pktkeeper.get_header_ptr();
#endif
            if (r) {
                ET_REG(EV_SENT_NOTOK);

#ifndef RFLINK_DEBUG_EVENTTIMER_ONLY
                dbgf("send err: taskid=%u, s=0x%02x, d=0x%02x, fl=0x%02x"
                     ", pktid=0x%04x, len=%i, err=%i: %s",
                     tsk->taskid, h->src, h->dst, h->flags, h->pktid, h->len,
                     r, get_err_string(r));
#endif

            } else {
                ET_REG(EV_SENT_OK);

#ifndef RFLINK_DEBUG_EVENTTIMER_ONLY
                dbgf("send ok:  taskid=%u, s=0x%02x, d=0x%02x, fl=0x%02x"
                     ", pktid=0x%04x, len=%i", tsk->taskid,
                     h->src, h->dst, h->flags, h->pktid, h->len);
#endif

            }
#endif // RFLINK_DEBUG

            byte seq;
            byte opts;
            from_flags(tsk->pktkeeper.get_flags(), &seq, &opts);

            if (!tsk->is_an_ack)
                ++seq;

            tsk->pktkeeper.set_flags(to_flags(seq, tsk->pktkeeper.get_flags()));
        }

        tsk->send_schedule_pos++;

        if (tsk->send_schedule_pos < tsk->nb_send_schedules) {
            tsk->mtime_wakeup =
              tsk->mtime_ref + tsk->send_schedule_ptr[tsk->send_schedule_pos];
        } else {

            if (tsk->unattended)
                tsk->mtime_wakeup = get_current_time();
            else
                tsk->mtime_wakeup = get_current_time() + send_purge_delay;

            return ST_SEND_DONE;

        }

        return tsk->status;
    } else if (tsk->status == ST_SEND_DONE) {
        return ST_FINISHED;
    } else if (tsk->status == ST_RECEIVE_DATA_RETRIEVED
                 || tsk->status == ST_RECEIVE_TIMEDOUT) {
        return ST_FINISHED;
    } else if (tsk->status == ST_RECEIVE_DATA_AVAILABLE) {
        data_retrieved_post(tsk);
        return ST_RECEIVE_TIMEDOUT;
    } else if (tsk->status == ST_RECEIVE) {
        tsk->evtsub_wakeup = 1;
        tsk->mtime_wakeup = tsk->mtime_ref + DEFAULT_RECEIVE_TIMEOUT_DELAY;
        return ST_RECEIVE_TIMEDOUT;
    } else {
        // Execution shall never arrive here
        assert(false);
    }

    // Never executed
    return ST_NOTHING;

}

void RFLink::interrupts_on() {
    if (!interrupt_is_attached) {
        interrupt_is_attached = 1;
        (*funcs.setInterrupt)(interrupt_func);
//        dbg("enabled interrupts");
    }
}

void RFLink::interrupts_off() {
    if (interrupt_is_attached) {
        interrupt_is_attached = 0;
        (*funcs.resetInterrupt)();
//        dbg("disabled interrupts");
    }
}

// FIXME
//   Timing management won't work with auto_sleep() enabled, during periods
//   where CPU sleeps.
//   Not a very big issue though...
bool RFLink::check_pktid_already_seen(address_t src, pktid_t pktid) {

    bool src_found = false;

    int unused_entry_idx = -1;
    int oldest_entry_idx = -1;
    mtime_t biggest_elapsed_found = 0;
    mtime_t tref = get_current_time();

    bool ret = false;

    for (int i = 0; (unsigned)i < PKTID_CACHE_SIZE; ++i) {

        cache_pktid_t* current = &cache_pktids[i];

        mtime_t elapsed = tref - current->mtime;

        if (current->used && elapsed >= CACHE_PKTID_DISCARD_DELAY) {
//            dbgf("IDrec: discard tooold=#%i, s=0x%02x", i, current->src);
            current->used = false;
        }

        if (!current->used) {
            if (unused_entry_idx < 0)
                unused_entry_idx = i;
            continue;
        }

        if (current->src == src) {

            // ret being true should never happen: one entry per source
            assert(!ret);

            src_found = true;

//            dbgf("IDrec: match #%i, s=0x%02x", i, src);
            current->mtime = tref;
            if (current->last_pktid_seen == pktid) {
                ret = true;
            } else {
                current->last_pktid_seen = pktid;
            }

        } else if (elapsed > biggest_elapsed_found) {

            biggest_elapsed_found = elapsed;
            oldest_entry_idx = i;

        }

    }

    if (!src_found) {
        int idx = unused_entry_idx;
        if (idx < 0) {
            idx = oldest_entry_idx;
//            dbgf("IDrec: erase oldest=#%i, s=0x%02x",
//                 idx, cache_pktids[idx].src);
        } else {
//            dbgf("IDrec: will use empty slot #%i", idx);
        }

        cache_pktids[idx].used = true;
        cache_pktids[idx].src = src;
        cache_pktids[idx].mtime = tref;
        cache_pktids[idx].last_pktid_seen = pktid;
    }

    return ret;
}

void RFLink::do_events() {

    if (!funcs.deviceInit)
        return;

    bool i_want_to_receive = false;
    for (Task* tsk = tskhead; tsk != nullptr; tsk = tsk->next) {
        if (tsk->evtsub_pktrcvd) {
            i_want_to_receive = true;
            break;
        }
    }
    if (!funcs.deviceReceive)
        i_want_to_receive = false;

    if (i_want_to_receive)
        interrupts_on();

    // If true, pkt contains a packet that we want to hand over to a task
    bool got_a_pkt = false;

    unsigned long int zzz000 = micros();
    (void)zzz000;

    if (interrupted) {
        interrupts_off();

#if defined(RFLINK_DEBUG) && defined(RFLINK_DEBUG_EVENTTIMER_ONLY)
        mtime_t t0 = get_current_time();
#endif

        byte nb_bytes_rcvd = 0;
        if (i_want_to_receive) {
            initialize_recpkt_if_necessary();

            // FIXME
            // Writing directly into PktKeeper' packet is not good practice.
            // Doing it in a clean way will be a bit overkill (imho).
            nb_bytes_rcvd =
              (*funcs.deviceReceive)(
                 recpkt->notrecommended_get_pkt_ptr(), get_pkt_max_size()
              );

            got_a_pkt = recpkt->check_rcvd_pkt_is_ok(this, nb_bytes_rcvd);
        }

#ifdef RFLINK_DEBUG
#ifndef RFLINK_DEBUG_EVENTTIMER_ONLY
        const Header* h = recpkt->get_header_ptr();
#endif
        if (got_a_pkt) {
            ET_REG(EV_RECEIVE_CALL, t0);
            ET_REG(EV_RECEIVED_OK);
            dbgf("incoming pkt:       s=0x%02x, d=0x%02x, fl=0x%02x"
                   ", pktid=0x%04x, len=%i",
                   h->src, h->dst, h->flags, h->pktid, h->len);
        } else if (nb_bytes_rcvd >= sizeof(Header)) {
            ET_REG(EV_RECEIVE_CALL, t0);
            ET_REG(EV_RECEIVED_NOTOK);
            dbgf("incoming pkt: packet of incorrect size"
                   ", len=%i, header.len=%i",
                   nb_bytes_rcvd, recpkt->get_data_len());
        } else if (nb_bytes_rcvd >= 1) {
            ET_REG(EV_RECEIVE_CALL, t0);
            ET_REG(EV_RECEIVED_NOTOK);
            dbgf("incoming pkt: packet of incorrect size, len=%i",
                   nb_bytes_rcvd);
        } else {
            ET_REG(EV_RECEIVE_CALL, t0);
            ET_REG(EV_RECEIVED_0_BYTE_RCVD);
            dbg("incoming pkt: empty packet (no reception)");
        }
#endif // RFLINK_DEBUG

        interrupted = false;
        interrupts_on();
    }

    mtime_t tref = get_current_time();

    bool pktid_already_seen = false;
    if (got_a_pkt) {
        const Header* h = recpkt->get_header_ptr();
        pktid_already_seen = check_pktid_already_seen(h->src, h->pktid);
    }

    bool device_needs_reset = false;

    Task* tsknext;
    for (Task* tsk = tskhead; tsk != nullptr; tsk = tsknext) {

        // We must record what the next task is, in case our current tsk gets
        // destroyed.
        tsknext = tsk->next;

        byte new_status = tsk->status;

        if (tsk->evtsub_pktrcvd && got_a_pkt) {
            bool pkt_consumed = false;
            new_status = tev_received(tsk, recpkt, pktid_already_seen,
              &pkt_consumed);
            if (pkt_consumed) {
                dbgf("incoming pkt: pkt consumed by taskid=%u, st=%i",
                       tsk->taskid, tsk->status);
                got_a_pkt = false;
            }
        }

        if (tsk->evtsub_wakeup && new_status == tsk->status) {
            // NOTE
            // Yes, casting to "signed" works if the difference does not go
            // beyond the type capacity (here: around 24 days).
            long int elapsed = (long int)(tref - tsk->mtime_wakeup);
            if (elapsed >= 0) {
                new_status = tev_wakeup(tsk);
            }
        }

        if (new_status != ST_RECEIVE
              && new_status != ST_NOTHING
              && new_status != ST_FINISHED) {
            if (!tsk->evtsub_wakeup) {
                dbgf("taskid:%i", tsk->taskid);
                assert(false);
            }
        }

        if (new_status == ST_FINISHED) {
            if (tsk->status == ST_SEND_DONE
                  && tsk->need_ack && !tsk->has_received_ack) {
                device_needs_reset = true;
            }
            task_destroy(tsk);
        } else {
            tsk->status = new_status;
        }
    }

    if (got_a_pkt) {
        dbg("incoming pkt: packet not consumed");
    }

    if (device_needs_reset) {
        mtime_t now = get_current_time();
        if ((now - last_device_reset) >= MIN_DEVICE_RESET_DELAY) {
            last_device_reset = now;
            (*funcs.deviceInit)(nullptr, true);
            delay(POST_DEVICE_RESET_DELAY);
            dbg("did reset device");
        }
    }

    // MANAGE "GO TO SLEEP"

    //   First thing is, to work out whether or not, we are in a status that
    //   allows to go to sleep.
    //   The condition is: we are waiting for a packet and that's it (no other
    //   pending task, no wake-up scheduled)

    byte count_task_evtsub_pktrcvd = 0;
    byte count_task_evtsub_wakeup = 0;
    byte count_task_non_nothing = 0;
    for (Task* tsk = tskhead; tsk != nullptr; tsk = tsk->next) {
        if (tsk->evtsub_pktrcvd)
            count_task_evtsub_pktrcvd++;
        if (tsk->evtsub_wakeup)
            count_task_evtsub_wakeup++;
        else if (tsk->status != ST_NOTHING)
            count_task_non_nothing++;
    }
    static bool last_is_eligible_for_sleep = false;
    bool is_eligible_for_sleep =
      (count_task_evtsub_pktrcvd == 1
       && count_task_evtsub_wakeup == 0
       && count_task_non_nothing == 1);

    if (is_eligible_for_sleep && auto_sleep) {
        sleep_enable();
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        dbg("Going to sleep...");
#ifdef RFLINK_DEBUG
        // Needed to have data sent over the serial line, before going to sleep
        // really.
        delay(20);
#endif

//        noInterrupts();
        (*funcs.deviceInit)(nullptr, true);
//        interrupts();
        sleep_cpu();

        dbg("WAKE UP!!!");
    } else if (is_eligible_for_sleep) {
        if (!last_is_eligible_for_sleep) {
            dbg("Could go to sleep, but auto_sleep is not activated");
        }
    }
    last_is_eligible_for_sleep = is_eligible_for_sleep;

#ifdef RFLINK_DEBUG
    dbg_print_status(is_eligible_for_sleep);
#endif

    ET_PRTPERIOD(10000);
}

#ifdef RFLINK_DEBUG
void RFLink::dbg_print_status(bool is_eligible_for_sleep) {
    static long unsigned print_status_last_t = get_current_time();
    byte n = 0, a = 0, f = 0, r = 0;
    for (Task* tsk = tskhead; tsk != nullptr; tsk = tsk->next) {
        byte st = tsk->status;
        if (st == ST_NOTHING)
            ++n;
        else if (st == ST_FINISHED)
            ++f;
        else
            ++a;
        if (st == ST_RECEIVE)
            ++r;
    }
    long unsigned t = get_current_time();
    if ((t - print_status_last_t) >= 500) {
        print_status_last_t = t;
        dbgf("do_events: N=%2i A=%2i F=%2i (R=%2i) tskc=%2i"
               " (mpl=%i) (soT=%i) (fm=%u) (S=%i)",
               n, a, f, r, task_count, max_payload_len, sizeof(Task),
               freeMemory(), is_eligible_for_sleep);
    }
}
#endif // RFLINK_DEBUG

byte RFLink::send_ack_noblock(taskid_t* taskid, Header* h) {

    assert(!h->len);

    if (!funcs.deviceInit)
        return ERR_DEVICE_NOT_REGISTERED;
    else if (!funcs.deviceSend)
        return ERR_SEND_FUNC_NOT_REGISTERED;

    Task* tsk = task_create(ST_SEND);
    if (!tsk) {
        return ERR_UNABLE_TO_CREATE_TASK;
    }

    *taskid = tsk->taskid;

    tsk->evtsub_wakeup = 1;
    tsk->send_schedule_ptr = snd_repack_sched;
    tsk->nb_send_schedules = snd_repack_sched_len;
    tsk->send_schedule_pos = 0;
    tsk->mtime_wakeup = tsk->mtime_ref
                        + tsk->send_schedule_ptr[tsk->send_schedule_pos];

    tsk->is_an_ack = 1;
    tsk->unattended = 1;

    tsk->pktkeeper.prepare_for_sending(this, h, nullptr);

//    dbgf("send_ack_noblock: taskid=%u, s=0x%02x, d=0x%02x, fl=0x%02x"
//           ", pktid=0x%04u, len=%i",
//           tsk->taskid, h->src, h->dst, h->flags, h->pktid, h->len);

    return ERR_TASK_CREATED_OK;

}

byte RFLink::send_noblock(taskid_t* taskid, address_t dst,
                          const void* data, byte len, bool ack) {
    if (!funcs.deviceInit)
        return ERR_DEVICE_NOT_REGISTERED;
    else if (!funcs.deviceSend)
        return ERR_SEND_FUNC_NOT_REGISTERED;

    if (len > max_payload_len)
        return ERR_SEND_DATA_LEN_ABOVE_LIMIT;

    // NOTE
    // We don't test the other way round (len != 0 while data being null),
    // this'd be not bad practice, but a pure bug. And this condition is tested
    // by prepare_for_sending().
    if (len == 0 && data != nullptr)
        return ERR_SEND_BAD_ARGUMENTS;

    Task* tsk = task_create(ST_SEND);
    if (!tsk) {
        return ERR_UNABLE_TO_CREATE_TASK;
    }

    *taskid = tsk->taskid;

    tsk->evtsub_wakeup = 1;
    tsk->nb_send_schedules = (ack ? snd_expack_sched_len : snd_sched_len);
    tsk->send_schedule_ptr = (ack ? snd_expack_sched : snd_sched);
    tsk->send_schedule_pos = 0;
    tsk->mtime_wakeup = tsk->mtime_ref
                        + tsk->send_schedule_ptr[tsk->send_schedule_pos];

    if (ack) {
        tsk->need_ack = 1;
        tsk->evtsub_pktrcvd = 1;
    }

    Header h;
    h.src = device_addr;
    h.dst = dst;
    h.flags = to_flags(0, (ack ? FLAG_SIN : FLAG_NONE));
    h.pktid = ++last_pktid;
    h.len = len;

    tsk->pktkeeper.prepare_for_sending(this, &h, data);

//    dbgf("send_noblock: taskid=%u, s=0x%02x, d=0x%02x, fl=0x%02x"
//           ", pktid=0x%04u, len=%i",
//           tsk->taskid, h.src, h.dst, h.flags, h.pktid, h.len);

    return ERR_TASK_CREATED_OK;
}

Task* RFLink::get_task_by_taskid(taskid_t taskid) {
    for (Task* tsk = tskhead; tsk != nullptr; tsk = tsk->next) {
        if (tsk->taskid == taskid) {
            return tsk;
        }
    }
    return nullptr;
}

byte RFLink::task_get_status(taskid_t taskid) {
    Task* tsk = get_task_by_taskid(taskid);

    if (!tsk)
        return ST_NOTHING;

    return tsk->status;
}

byte RFLink::send_get_final_status(taskid_t taskid, byte* nbsend) {
    Task* tsk = get_task_by_taskid(taskid);
    if (!tsk)
        return ERR_UNKNOWN_TASKID;

    if (tsk->status != ST_SEND_DONE)
        return ERR_TASK_UNDERWAY;

    byte ret = ERR_UNDEFINED;

    if (tsk->need_ack && tsk->has_received_ack) {
        ret = ERR_OK;
    } else if (tsk->need_ack) {
        ret = ERR_SEND_NO_ACK_RCVD;
    } else {
        ret = tsk->last_retcode;
    }
    if (nbsend) {
        *nbsend = tsk->nbsend;
    }

    dbgf("taskid=%u: terminating immediately", tsk->taskid);

    tsk->evtsub_wakeup = 1;
    tsk->mtime_wakeup = get_current_time();

    return ret;
}

byte RFLink::send(address_t dst, const void* data, byte len, bool ack,
                  byte *nbsend) {
    taskid_t taskid;
    if (!len)
        data = nullptr;
    byte r = send_noblock(&taskid, dst, data, len, ack);

    if (r != ERR_TASK_CREATED_OK) {
        dbgf("send: no task created, error #%i: %s", r, get_err_string(r));
        return r;
    }

    while (task_get_status(taskid) == ST_SEND) {
        do_events();
    }

    return send_get_final_status(taskid, nbsend);
}

byte RFLink::receive_noblock(taskid_t* taskid, RXConfig* cfg) {
    if (!funcs.deviceInit)
        return ERR_DEVICE_NOT_REGISTERED;
    else if (!funcs.deviceReceive)
        return ERR_RECEIVE_FUNC_NOT_REGISTERED;

    Task* tsk = task_create(ST_RECEIVE);
    if (!tsk) {
        return ERR_UNABLE_TO_CREATE_TASK;
    }

    *taskid = tsk->taskid;
    tsk->evtsub_pktrcvd = 1;
    if (cfg) {
        if (cfg->def_timeout) {
            tsk->evtsub_wakeup = 1;
            tsk->mtime_wakeup = tsk->mtime_ref + cfg->timeout;
        }
    }

//    dbgf("receive_noblock: taskid=%u", tsk->taskid);

    return ERR_TASK_CREATED_OK;
}

void RFLink::send_ack(Task* tsk) {
    byte seq;
    byte opt;
    from_flags(tsk->pktkeeper.get_flags(), &seq, &opt);
    if (opt & FLAG_SIN) {

        const Header* h = tsk->pktkeeper.get_header_ptr();
        Header ack_h;
        ack_h.dst = h->src;
        ack_h.src = device_addr;
        ack_h.flags = to_flags(seq, FLAG_ACK);
        ack_h.pktid = h->pktid;
        ack_h.len = 0;

        dbgf("sending back ACK for s=0x%02x, d=0x%02x, pktid=0x%04x",
               ack_h.src, ack_h.dst, ack_h.pktid);

        taskid_t taskid;
        send_ack_noblock(&taskid, &ack_h);
    }
}

void RFLink::data_retrieved_post(Task* tsk) {
    tsk->pktkeeper.reduce_packet_to_its_header();
    tsk->evtsub_wakeup = 1;
    tsk->mtime_wakeup = tsk->mtime_ref + receive_purge_delay;
}

byte RFLink::data_retrieve(Task* tsk, void* buf, byte buf_len, byte* rec_len,
                           address_t* sender) {
    if (!tsk)
        return ST_NOTHING;

    if (tsk->status != ST_RECEIVE_DATA_AVAILABLE)
        return tsk->status;

    tsk->pktkeeper.copy_data(buf, buf_len, rec_len);
    if (sender)
        *sender = tsk->pktkeeper.get_header_ptr()->src;

    data_retrieved_post(tsk);
    tsk->status = ST_RECEIVE_DATA_RETRIEVED;

    send_ack(tsk);

    return tsk->status;
}

byte RFLink::receive(void* buf, byte buf_len, byte* rec_len,
                     address_t* sender, RXConfig* cfg) {
    taskid_t taskid;
    byte r = receive_noblock(&taskid, cfg);

    do_events();

//    dbgf("receive started, taskid = %u", taskid);

    if (r != ERR_TASK_CREATED_OK) {
//        dbgf("receive: no task created, error #%i: %s", r, get_err_string(r));
        return r;
    }

    while (task_get_status(taskid) == ST_RECEIVE) {
        do_events();
    }

    Task* tsk = get_task_by_taskid(taskid);
    r = data_retrieve(tsk, buf, buf_len, rec_len, sender);

    do_events();

    if (r == ST_RECEIVE_DATA_AVAILABLE || r == ST_RECEIVE) {
        assert(false);
    } else if (r == ST_NOTHING) {
        return ERR_TIMEOUT;
    } else if (r == ST_RECEIVE_DATA_RETRIEVED) {
        return ERR_OK;
    } else if (r == ST_RECEIVE_TIMEDOUT) {
        return ERR_TIMEOUT;
    }

    assert(false);

    // Never executed
    return ERR_UNDEFINED;
}

void RFLink::delay_ms(long int d) {
    if (d <= 0)
        return;

    unsigned long int t0 = get_current_time();
    while ((signed)(get_current_time() - t0) < d) {
        do_events();
    }
}

void RFLink::set_opt(opt_t opt, void* data, byte len) {
    if (!funcs.deviceSetOpt)
        return;

    (*funcs.deviceSetOpt)(opt, data, len);

#ifdef ASSUME_DEVICE_ADDRESS_IS_ONE_BYTE
    if (opt == OPT_ADDRESS) {
        device_addr_has_been_defined = 1;
        device_addr = *((byte*)data);
    }
#else
#error "PLEASE REVIEW THIS CODE HERE: NEED TO HANDLE NOT-1-BYTE-LIKE ADDRESSES"
#endif
}

void RFLink::set_opt_byte(opt_t opt, byte value) {
    set_opt(opt, &value, sizeof(value));
}

void RFLink::set_auto_sleep(bool v) {
    auto_sleep = v;
}


//
// PktKeeper
//

PktKeeper::PktKeeper():pkt(nullptr) {

}

PktKeeper::PktKeeper(byte pkt_len) {
    pkt = (Packet*)malloc(pkt_len);
}

PktKeeper::~PktKeeper() {
    if (pkt) {
        free(pkt);
        dbg("freeing pkt (destructor)");
    }
}

void PktKeeper::copy_packet(const PktKeeper* pktkeeper) {
    assert(!pkt);

    byte len = pktkeeper->get_pkt_len();
    pkt = (Packet*)malloc(len);
    memcpy(pkt, pktkeeper->get_pkt_ptr_ro(), len);
}

bool PktKeeper::check_rcvd_pkt_is_ok(const RFLink* link, byte nb_bytes) {
    if (!pkt)
        return false;

    if (pkt->header.len > link->get_max_payload_len())
        return false;

    return (get_pkt_len() == nb_bytes);
}

void PktKeeper::release_data() {
    if (pkt) {
        free(pkt);
        dbg("freeing pkt (release_data)");
        pkt = nullptr;
    }
}

void PktKeeper::prepare_for_sending(const RFLink* link, Header* header,
                                    const void *data) {
    assert(pkt == nullptr);

    assert(   (header->len == 0 && data == nullptr)
           || (header->len >= 1 && data != nullptr));

    pkt = (Packet*)malloc(sizeof(Header) + header->len);

    pkt->header = *header;

    if (pkt->header.len > link->get_max_payload_len()) {
        pkt->header.len = link->get_max_payload_len();
    }

    if (pkt->header.len)
        memcpy(&pkt->data, data, pkt->header.len);
}

const Header* PktKeeper::get_header_ptr() const {
    if (!pkt)
        return nullptr;

    return &pkt->header;
}

byte PktKeeper::get_flags() {
    if (!pkt)
        return 0xFF;

    return pkt->header.flags;
}

void PktKeeper::set_flags(byte arg_flags) {
    if (!pkt)
        return;

    pkt->header.flags = arg_flags;
}

Packet* PktKeeper::notrecommended_get_pkt_ptr() {
    return pkt;
}

const Packet* PktKeeper::get_pkt_ptr_ro() const {
    return pkt;
}

byte PktKeeper::get_pkt_len() const {
    if (!pkt)
        return 0;

    return sizeof(Header) + pkt->header.len;
}

const void* PktKeeper::get_data_ptr() const {
    if (!pkt)
        return nullptr;

    return &pkt->data;
}

byte PktKeeper::get_data_len() const {
    if (!pkt)
        return 0xFF;

    return pkt->header.len;
}

void PktKeeper::reduce_packet_to_its_header() {
    assert(pkt);

    Packet* new_pkt = (Packet*)malloc(sizeof(Header));
    new_pkt->header = pkt->header;
    new_pkt->header.len = 0;
    free(pkt);
    pkt = new_pkt;

    dbg("** PACKET REDUCED **");
}

void PktKeeper::copy_data(void *buf, byte buf_len, byte* rec_len) const {
    if (!pkt)
        return;

    *rec_len = pkt->header.len;
    if (*rec_len > buf_len)
        *rec_len = buf_len;

    if (*rec_len)
        memcpy(buf, &pkt->data, *rec_len);
}

