#ifndef HKS_SHA256_H
#define HKS_SHA256_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HKS_SHA256_DIGEST_BYTES 32u
#define HKS_SHA256_BLOCK_BYTES  64u

/**
 * Plain POD streaming FIPS 180-4 SHA-256 context (zero dynamic allocation).
 * May be copied by value to checkpoint an intermediate digest (e.g., pre_footer_sha256).
 */
typedef struct {
    uint32_t state[8];
    uint64_t bitlen;
    uint8_t data[HKS_SHA256_BLOCK_BYTES];
    size_t datalen;
} hks_sha256_ctx_t;

void hks_sha256_init(hks_sha256_ctx_t *ctx);
void hks_sha256_update(hks_sha256_ctx_t *ctx, const void *data, size_t len);
void hks_sha256_final(hks_sha256_ctx_t *ctx, uint8_t out[HKS_SHA256_DIGEST_BYTES]);
void hks_sha256(const void *data, size_t len, uint8_t out[HKS_SHA256_DIGEST_BYTES]);

#ifdef __cplusplus
}
#endif

#endif /* HKS_SHA256_H */
