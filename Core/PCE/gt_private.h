//
// Created by . on 6/27/25.
//

#ifndef MESEN2_GT_PRIVATE_H
#define MESEN2_GT_PRIVATE_H

#include <string.h>
#include <stdint.h>
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

typedef uint64 u64;
typedef int64 s64;
typedef int64 i64;

typedef uint32 u32;
typedef int32 s32;
typedef int32 i32;

typedef uint16 u16;
typedef int16 s16;
typedef int16 i16;

typedef uint8 u8;
typedef int8 s8;
typedef int8 i8;

typedef float f32;
typedef double f64;

struct sfc32_state {
    u64 a, b, c, d;
};

void sfc32_seed(const char *seed, struct sfc32_state *state);
u32 sfc32(struct sfc32_state *state);


struct cyrb128_ret {
    u32 a, b, c, d;
};

// We use seedable, repeatable RNGs to make reproducible tests.
// https://stackoverflow.com/questions/521295/seeding-the-random-number-generator-in-javascript
static struct cyrb128_ret cyrb128(const char *str) {
    u32 h1 = 1779033703, h2 = 3144134277,
            h3 = 1013904242, h4 = 2773480762;
    for (u32 i = 0; i < strlen(str); i++) {
        u32 k = (u32)(u8)str[i];
        h1 = h2 ^ ((h1 ^ k) * 597399067);
        h2 = h3 ^ ((h2 ^ k) * 2869860233);
        h3 = h4 ^ ((h3 ^ k) * 951274213);
        h4 = h1 ^ ((h4 ^ k) * 2716044179);
    }
    h1 = (h3 ^ (h1 >> 18)) * 597399067;
    h2 = (h4 ^ (h2 >> 22)) * 2869860233;
    h3 = (h1 ^ (h3 >> 17)) *  951274213;
    h4 = (h2 ^ (h4 >> 19)) * 2716044179;
    return (struct cyrb128_ret){ .a = h1^h2^h3^h4, .b = h2^h1, .c = h3^h1, .d = h4^h1};
}

void sfc32_seed(const char *seed, struct sfc32_state &state) {
    struct cyrb128_ret r = cyrb128(seed);
    state.a = r.a;
    state.b = r.b;
    state.c = r.c;
    state.d = r.d;
}

u32 sfc32(struct sfc32_state &state) {
    u64 t = state.a + state.b;
    state.a = state.b ^ state.b >> 9;
    state.b = state.c + (state.c << 3);
    state.c = (state.c << 21 | state.c >> 11);
    state.d = state.d + 1;
    t = t + state.d;
    state.c = state.c + t;
    return (u32)(t >> 32);
}


#endif //MESEN2_GT_PRIVATE_H

