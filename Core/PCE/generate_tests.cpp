//
// Created by . on 6/26/25.
//
#include "PceCpu.h"
#include "PceMemoryManager.h"
#undef printf

#include <pwd.h>
#include <unistd.h>

#include <printf.h>
#include <malloc/malloc.h>

#include "generate_tests.h"
#include "gt_private.h"

#define MAX_TRANSACTIONS 800
#define MAX_RAM_PAIRS 800
#define NUMTESTS 2500
#define ISTART 1
#define IEND 2

#define ALLOC_BUF_SIZE (2 * 1024 * 1024)

#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wc++11-narrowing"

static u64 cR8(void *ptr, u32 addr) {
    return ((u8 *)ptr)[addr];
}

static u64 cR16(void *ptr, u32 addr) {
    return *(u16*)(((u8*)ptr)+addr);
}

static u64 cR32(void *ptr, u32 addr) {
    return *(u32*)(((u8*)ptr)+addr);
}

static u64 cR64(void *ptr, u32 addr) {
    return *(u64*)(((u8*)ptr)+addr);
}

static void cW8(void *ptr, u32 addr, u64 val) {
    *(((u8*)ptr)+addr) = val;
}

static void cW16(void *ptr, u32 addr, u64 val) {
    *(u16 *)(((u8*)ptr)+addr) = val;
}

static void cW32(void *ptr, u32 addr, u64 val) {
    *(u32 *)(((u8*)ptr)+addr) = val;
}

static void cW64(void *ptr, u32 addr, u64 val) {
    *(u64 *)(((u8*)ptr)+addr) = val;
}

static u64 (*cR[9])(void *, u32) = {
        NULL,
        &cR8,
        &cR16,
        NULL,
        &cR32,
        NULL,
        NULL,
        NULL,
        &cR64
};

static void (*cW[9])(void *, u32, u64) = {
        NULL,
        &cW8,
        &cW16,
        NULL,
        &cW32,
        NULL,
        NULL,
        NULL,
        &cW64
};

struct cycle_pins {
    u32 RD, WR, Addr, D;
};

struct RAM_pair {
    u32 addr;
    u16 val;
};

struct huc6280_test_state {
    u32 A, X, Y, S, PC, P;
    u32 MPR[8];
    u32 num_RAM;
    struct RAM_pair RAM_pairs[MAX_RAM_PAIRS];
};

#define MAX_CYCLES 5000

struct huc6280_test {
    char name[100];
    u32 opcode;
    struct huc6280_test_state initial;
    struct huc6280_test_state final;

    u32 num_cycles;
    struct cycle_pins cycles[MAX_CYCLES];
};


struct huc6280_test_struct {
    struct sfc32_state rstate;
    struct huc6280_test tests[NUMTESTS];
    struct huc6280_test *cur;

    u32 log_transactions;
    u32 cur_cycle;

    PceCpu *cpu;
    u8 *buf, *ptr;
};

struct huc6280_test_struct ts{};


static u32 rint(sfc32_state &rs, u32 min, u32 max)
{
    double a = (sfc32(rs) / 4294967295.0) * (double)max;
    return (u32)a + min;
}

static void add_read_cycle(u32 addr, u32 val)
{
    struct cycle_pins *c = &ts.cur->cycles[ts.cur_cycle++];
    c->RD = 1;
    c->WR = 0;
    c->Addr = addr;
    c->D = val;
}

static void add_write_cycle(u32 addr, u32 val)
{
    struct cycle_pins *c = &ts.cur->cycles[ts.cur_cycle++];
    c->WR = 1;
    c->RD = 0;
    c->Addr = addr;
    c->D = val;
}

static void add_idle_cycle()
{
    struct cycle_pins *c = &ts.cur->cycles[ts.cur_cycle++];
    c->RD = c->WR = 0;
    c->Addr = -1;
    c->D = -1;
}


static void initial_random_state(struct huc6280_test_state *s, u32 opcode)
{
    s->X = sfc32(ts.rstate) & 0xFF;
    s->Y = sfc32(ts.rstate) & 0xFF;
    s->A = sfc32(ts.rstate) & 0xFF;
    s->PC = sfc32(ts.rstate) & 0xFFFF;
    s->S = sfc32(ts.rstate) & 0xFF;
    s->P = sfc32(ts.rstate) & 0xEF;
    for (u32 i = 0; i < 8; i++) {
        s->MPR[i] = sfc32(ts.rstate) & 0xFF;
    }
    s->num_RAM = 0;
}

static void copy_state_from_cpu(struct huc6280_test_state &state)
{
    state.X = ts.cpu->_state.X;
    state.Y = ts.cpu->_state.Y;
    state.A = ts.cpu->_state.A;
    state.S = ts.cpu->_state.SP;
    state.P = ts.cpu->_state.PS;
    state.PC = ts.cpu->_state.PC;
    for (u32 i = 0; i < 8; i++) {
        state.MPR[i] = ts.cpu->_memoryManager->_state.Mpr[i];
    }
}

static void copy_state_to_cpu(struct huc6280_test_state &state)
{
    ts.cpu->_state.X = state.X;
    ts.cpu->_state.Y = state.Y;
    ts.cpu->_state.A = state.A;
    ts.cpu->_state.SP = state.S;
    ts.cpu->_state.PS = state.P;
    ts.cpu->_state.PC = state.PC;
    for (u32 i = 0; i < 8; i++) {
        ts.cpu->_memoryManager->_state.Mpr[i] = state.MPR[i];
    }
}

static u32 get_long_addr(u32 addr)
{
    u32 mpr = addr >> 13;
    u32 addr_lo = addr & 0x1FFF;
    u32 upper13 = ts.cpu->_memoryManager->_state.Mpr[mpr];
    addr = (upper13 << 13) | addr_lo;
    return addr;
}

void test_do_idle()
{
    add_idle_cycle();
}

void test_do_write(uint16_t maddr, uint8_t value, MemoryOperationType type)
{
    u32 found = 0;
    u32 addr = get_long_addr(maddr);
    for (u32 i = 0; i < ts.cur->final.num_RAM; i++) {
        struct RAM_pair *rp = &ts.cur->final.RAM_pairs[i];
        if (rp->addr == addr) {
            found = 1;
            rp->val = value;
        }
    }
    if (!found) {
        struct RAM_pair *rp = &ts.cur->final.RAM_pairs[ts.cur->final.num_RAM++];
        rp->addr = addr;
        rp->val = value;
    }
    add_write_cycle(addr, value);
}

uint8_t test_do_read(uint16_t maddr, MemoryOperationType type)
{
    static int ignore_reset = 2;
    u32 addr = maddr;
    if (ignore_reset > 0) {
        ignore_reset--;
        return 0;
    }
    u32 found = 0;
    u32 v;

    addr = get_long_addr(addr);
    // TODO: also search initial RAM
    for (u32 i = 0; i < ts.cur->final.num_RAM; i++) {
        // search final RAM for a previous write
        struct RAM_pair *rp = &ts.cur->final.RAM_pairs[i];
        if (rp->addr == addr) {
            found = 1;
            v = rp->val;
            break;
        }
    }
    if (!found) {
        // Search initial RAM for an initial value
        for (u32 i = 0; i < ts.cur->initial.num_RAM; i++) {
            struct RAM_pair *rp = &ts.cur->initial.RAM_pairs[i];
            if (rp->addr == addr) {
                found = 1;
                v = rp->val;
                break;
            }
        }
        if (!found) {
            printf("\nNOT FOUND %06x", addr);
            struct RAM_pair *rp = &ts.cur->initial.RAM_pairs[ts.cur->initial.num_RAM++];
            rp->addr = addr;
            rp->val = sfc32(ts.rstate) & 0xFF;
            v = rp->val;
        }
    }

    printf("\nDO READ ADDR:%06x: %02x", addr, v);

    // a read should be added to initial RAMpairs if not exist

    // a write should overwrite anything same addr in final RAMpairs
    add_read_cycle(addr, v);
    // TODO: find idles
    return v;
}

static void construct_path(char *out, u32 ins)
{
    char test_path[500];
    const char *homeDir = getenv("HOME");

    if (!homeDir) {
        struct passwd* pwd = getpwuid(getuid());
        if (pwd)
            homeDir = pwd->pw_dir;
    }

    char *tp = out;
    tp += sprintf(tp, "%s", homeDir);
    tp += sprintf(tp, "/dev/huc6280/v1");

    tp += sprintf(tp, "%s/", test_path);
    tp += sprintf(tp, "%02x.json.bin", ins);
}

static const char *pads = "                                                  ";

#define W8(x) { cW8(ts.ptr, 0, x); ts.ptr += 1; }
#define W16(x) { cW16(ts.ptr, 0, x); ts.ptr += 2; }
#define W32(x) { cW32(ts.ptr, 0, x); ts.ptr += 4; }

static void write_state(struct huc6280_test_state &st)
{
    u8 *bufbegin = ts.ptr;
    ts.ptr += 4;

    W8(st.A)
    W8(st.X)
    W8(st.Y)
    W8(st.S)
    W8(st.P)
    W16(st.PC)
    for (u32 i = 0; i < 8; i++) {
        W8(st.MPR[i])
    }
    W32(st.num_RAM);
    for (u32 i = 0; i < st.num_RAM; i++) {
        struct RAM_pair *rp = &st.RAM_pairs[i];
        W16(rp->addr);
        W8(rp->val);
    }
    size_t len = ts.ptr - bufbegin;
    cW32(bufbegin, 0, (u32)len);
}

static void write_test_to_disk(FILE *fo, u32 num)
{
    // write test params:
    // name (max 50), opcode,
    // initial and final state, num_cycles and cycles

    // Length of test...
    ts.ptr = ts.buf;
    ts.ptr += 4;

    // name
    char name[51];
    u32 sl = snprintf((char *)ts.ptr, 50, "%02x #%d", ts.cur->opcode, num);
    u32 slen = strlen((char *)ts.ptr);
    ts.ptr += sl;
    if (sl < 50) {
        memcpy(ts.ptr, pads, 50 - sl);
        ts.ptr += 50 - sl;
    }
    W32(ts.cur->opcode)
    write_state(ts.cur->initial);
    write_state(ts.cur->final);
    W32(ts.cur->num_cycles)
    for (u32 i = 0; i < ts.cur->num_cycles; i++) {
        struct cycle_pins *c = &ts.cur->cycles[i];
        u32 o = c->RD | (c->WR << 1) | (c->Addr << 8) | (c->D << 24);
        W32(o)
    }
    size_t len = ts.ptr - ts.buf;
    cW32(ts.buf, 0, (u32)len);
    fwrite(ts.buf, 1, len, fo);
}

static void generate_test(u32 opc)
{
    printf("\nGenerate test for %02x", opc);
    char name[51];
    char fpath[500];
    construct_path(fpath, opc);
    snprintf(name, sizeof(name), "%02x.json", opc);
    sfc32_seed(name, ts.rstate);

    // TODO: generate filename
    // TODO: open file

    for (u32 j = 0; j < 20; j++) sfc32(ts.rstate);

    FILE *fout = fopen(fpath, "w");
    u32 nt = NUMTESTS;
    fwrite(&nt, sizeof(nt), 1, fout);

    for (u32 i = 0; i < NUMTESTS; i++) {
        ts.cur = &ts.tests[i];
        ts.log_transactions = 0;
        ts.cur_cycle = 0;
        ts.cur->num_cycles = 0;

        initial_random_state(&ts.cur->initial, opc);
        copy_state_to_cpu(ts.cur->initial);

        u32 iaddr = get_long_addr(ts.cur->initial.PC);
        ts.cur->initial.num_RAM = 1;
        ts.cur->initial.RAM_pairs[0].addr = iaddr;
        ts.cur->initial.RAM_pairs[0].val = opc;
        ts.cur->final.num_RAM = 1;
        ts.cur->final.RAM_pairs[0].addr = iaddr;
        ts.cur->final.RAM_pairs[0].val = opc;
        ts.cur->opcode = opc;

        ts.cpu->Exec();

        ts.cur->num_cycles = ts.cur_cycle;
        copy_state_from_cpu(ts.cur->final);

        write_test_to_disk(fout, i);
    }
    fclose(fout);
}

void generate_tests(PceCpu &cpu)
{
    printf("\nGENERATE TEST!");
    u8 *a = (u8 *)malloc(ALLOC_BUF_SIZE);
    ts.buf = a;
    assert(ts.buf);
    ts.cpu = &cpu;
    for (u32 i = ISTART; i < IEND; i++) {
        generate_test(i);
    }
    throw 4;
}