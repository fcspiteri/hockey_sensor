#include "hks_sha256.h"

#include <string.h>

static inline uint32_t sha_rotr(uint32_t x, uint32_t n) {
    return (x >> n) | (x << (32u - n));
}

static inline uint32_t sha_ch(uint32_t x, uint32_t y, uint32_t z) {
    return (x & y) ^ ((~x) & z);
}

static inline uint32_t sha_maj(uint32_t x, uint32_t y, uint32_t z) {
    return (x & y) ^ (x & z) ^ (y & z);
}

static inline uint32_t sha_ep0(uint32_t x) {
    return sha_rotr(x, 2u) ^ sha_rotr(x, 13u) ^ sha_rotr(x, 22u);
}

static inline uint32_t sha_ep1(uint32_t x) {
    return sha_rotr(x, 6u) ^ sha_rotr(x, 11u) ^ sha_rotr(x, 25u);
}

static inline uint32_t sha_sig0(uint32_t x) {
    return sha_rotr(x, 7u) ^ sha_rotr(x, 18u) ^ (x >> 3);
}

static inline uint32_t sha_sig1(uint32_t x) {
    return sha_rotr(x, 17u) ^ sha_rotr(x, 19u) ^ (x >> 10);
}

static const uint32_t k_sha256_k[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u
};

static void sha256_transform(hks_sha256_ctx_t *ctx, const uint8_t block[64]) {
    uint32_t m[64];
    for (size_t i = 0u, j = 0u; i < 16u; i++, j += 4u) {
        m[i] = ((uint32_t)block[j] << 24) |
               ((uint32_t)block[j + 1u] << 16) |
               ((uint32_t)block[j + 2u] << 8) |
               (uint32_t)block[j + 3u];
    }
    for (size_t i = 16u; i < 64u; i++) {
        m[i] = sha_sig1(m[i - 2u]) + m[i - 7u] + sha_sig0(m[i - 15u]) + m[i - 16u];
    }

    uint32_t a = ctx->state[0];
    uint32_t b = ctx->state[1];
    uint32_t c = ctx->state[2];
    uint32_t d = ctx->state[3];
    uint32_t e = ctx->state[4];
    uint32_t f = ctx->state[5];
    uint32_t g = ctx->state[6];
    uint32_t h = ctx->state[7];

    for (size_t i = 0u; i < 64u; i++) {
        uint32_t t1 = h + sha_ep1(e) + sha_ch(e, f, g) + k_sha256_k[i] + m[i];
        uint32_t t2 = sha_ep0(a) + sha_maj(a, b, c);
        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }

    ctx->state[0] += a;
    ctx->state[1] += b;
    ctx->state[2] += c;
    ctx->state[3] += d;
    ctx->state[4] += e;
    ctx->state[5] += f;
    ctx->state[6] += g;
    ctx->state[7] += h;
}

void hks_sha256_init(hks_sha256_ctx_t *ctx) {
    if (ctx == NULL) {
        return;
    }
    ctx->datalen = 0u;
    ctx->bitlen = 0u;
    ctx->state[0] = 0x6a09e667u;
    ctx->state[1] = 0xbb67ae85u;
    ctx->state[2] = 0x3c6ef372u;
    ctx->state[3] = 0xa54ff53au;
    ctx->state[4] = 0x510e527fu;
    ctx->state[5] = 0x9b05688cu;
    ctx->state[6] = 0x1f83d9abu;
    ctx->state[7] = 0x5be0cd19u;
}

void hks_sha256_update(hks_sha256_ctx_t *ctx, const void *data, size_t len) {
    if (ctx == NULL || data == NULL || len == 0u) {
        return;
    }
    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0u; i < len; i++) {
        ctx->data[ctx->datalen++] = bytes[i];
        if (ctx->datalen == HKS_SHA256_BLOCK_BYTES) {
            sha256_transform(ctx, ctx->data);
            ctx->bitlen += 512u;
            ctx->datalen = 0u;
        }
    }
}

void hks_sha256_final(hks_sha256_ctx_t *ctx, uint8_t out[HKS_SHA256_DIGEST_BYTES]) {
    if (ctx == NULL || out == NULL) {
        return;
    }
    size_t i = ctx->datalen;

    if (ctx->datalen < 56u) {
        ctx->data[i++] = 0x80u;
        while (i < 56u) {
            ctx->data[i++] = 0x00u;
        }
    } else {
        ctx->data[i++] = 0x80u;
        while (i < 64u) {
            ctx->data[i++] = 0x00u;
        }
        sha256_transform(ctx, ctx->data);
        memset(ctx->data, 0, 56u);
    }

    ctx->bitlen += (uint64_t)ctx->datalen * 8u;
    ctx->data[63] = (uint8_t)(ctx->bitlen & 0xFFu);
    ctx->data[62] = (uint8_t)((ctx->bitlen >> 8) & 0xFFu);
    ctx->data[61] = (uint8_t)((ctx->bitlen >> 16) & 0xFFu);
    ctx->data[60] = (uint8_t)((ctx->bitlen >> 24) & 0xFFu);
    ctx->data[59] = (uint8_t)((ctx->bitlen >> 32) & 0xFFu);
    ctx->data[58] = (uint8_t)((ctx->bitlen >> 40) & 0xFFu);
    ctx->data[57] = (uint8_t)((ctx->bitlen >> 48) & 0xFFu);
    ctx->data[56] = (uint8_t)((ctx->bitlen >> 56) & 0xFFu);
    sha256_transform(ctx, ctx->data);

    for (size_t w = 0u; w < 8u; w++) {
        out[w * 4u + 0u] = (uint8_t)((ctx->state[w] >> 24) & 0xFFu);
        out[w * 4u + 1u] = (uint8_t)((ctx->state[w] >> 16) & 0xFFu);
        out[w * 4u + 2u] = (uint8_t)((ctx->state[w] >> 8) & 0xFFu);
        out[w * 4u + 3u] = (uint8_t)(ctx->state[w] & 0xFFu);
    }
}

void hks_sha256(const void *data, size_t len, uint8_t out[HKS_SHA256_DIGEST_BYTES]) {
    hks_sha256_ctx_t ctx;
    hks_sha256_init(&ctx);
    hks_sha256_update(&ctx, data, len);
    hks_sha256_final(&ctx, out);
}
