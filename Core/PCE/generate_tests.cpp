//
// Created by . on 6/26/25.
//
#include "PceCpu.h"
#include "PceMemoryManager.h"
#undef printf


#include <printf.h>
#include <malloc/malloc.h>

#include "generate_tests.h"
#include "gt_private.h"

#define MAX_TRANSACTIONS 800
#define MAX_RAM_PAIRS 800
#define NUMTESTS 2500
#define ISTART 1
#define IEND 2

#define ALLOC_BUF_SIZE (1 * 1024 * 1024)

#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wc++11-narrowing"

enum transaction_kind {
    tk_idle,
    tk_read,
    tk_write,
    tk_regread,
    tk_regwrite
};

struct transaction {
    enum transaction_kind kind;
    u32 len;
    u32 MRQ, RW, OTH;
    u32 addr_bus;
    u32 data_bus;

    u32 visited;
};


struct RAM_pair {
    u32 addr;
    u16 val;
};

struct huc6280_test_transactions {
    u32 num_transactions;
    struct transaction items[MAX_TRANSACTIONS];
};

struct huc6280_test_state {
    u32 A, X, Y, S, PC, P;
    u32 MPR[8];
    u32 num_RAM;
    struct RAM_pair RAM_pairs[MAX_RAM_PAIRS];
};

struct huc6280_test {
    char name[100];
    u32 opcode;
    struct huc6280_test_state initial;
    struct huc6280_test_state final;
    struct huc6280_test_transactions transactions;

    u32 num_cycles;
    u32 current_cycle;
};

struct huc6280_test_struct {
    struct sfc32_state rstate;
    struct huc6280_test tests[NUMTESTS];
    struct huc6280_test *cur;

    struct huc6280_test_transactions transactions;
    u32 log_transactions;

    PceCpu *cpu;
    u8 *buf, *ptr;
};

struct huc6280_test_struct ts{};


static u32 rint(sfc32_state &rs, u32 min, u32 max)
{
    double a = (sfc32(rs) / 4294967295.0) * (double)max;
    return (u32)a + min;
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

void test_do_write(uint16_t addr, uint8_t value, MemoryOperationType type)
{
    printf("\nDO WRITE ADDR:%04x VAL:%02x", addr, value);
}

static u32 get_long_addr(u32 addr)
{
    u32 mpr = addr >> 13;
    u32 addr_lo = addr & 0x1FFF;
    u32 upper13 = ts.cpu->_memoryManager->_state.Mpr[mpr];
    addr = (upper13 << 13) | addr_lo;
    return addr;
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
    u32 v = 0;

    addr = get_long_addr(addr);

    for (u32 i = 0; i < ts.cur->final.num_RAM; i++) {
        struct RAM_pair *rp = &ts.cur->final.RAM_pairs[i];
        printf("\nTESTING ADDR %06x", rp->addr);
        if (rp->addr == addr) {
            found = 1;
            v = rp->val;
            break;
        }
    }
    if (!found) {
        printf("\nNOT FOUND %06x", addr);
    }
    else {
        printf("\nFOUND!");
    }
    printf("\nDO READ ADDR:%06x: %02x", addr, v);


    // TODO: add cycle
    // TODO: change "transaction" to "cycle" and do cycle by cycle
    // TODO: find idles
    return v;
}


static void generate_test(u32 opc)
{
    printf("\nGenerate test for %02x", opc);
    char name[50];
    snprintf(name, sizeof(name), "%02x.json", opc);
    sfc32_seed(name, ts.rstate);
    ts.ptr = ts.buf;

    for (u32 j = 0; j < 20; j++) sfc32(ts.rstate);

    for (u32 i = 0; i < 1; i++) {
        ts.cur = &ts.tests[i];
        ts.log_transactions = 0;

        ts.transactions.num_transactions = 0;

        initial_random_state(&ts.cur->initial, opc);
        copy_state_to_cpu(ts.cur->initial);

        u32 iaddr = get_long_addr(ts.cur->initial.PC);
        printf("\nIADDR: %06x", iaddr);
        ts.cur->initial.num_RAM = 1;
        ts.cur->initial.RAM_pairs[0].addr = iaddr;
        ts.cur->initial.RAM_pairs[0].val = opc;
        ts.cur->final.num_RAM = 1;
        ts.cur->final.RAM_pairs[0].addr = iaddr;
        ts.cur->final.RAM_pairs[0].val = opc;

        ts.cpu->Exec();
    }
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