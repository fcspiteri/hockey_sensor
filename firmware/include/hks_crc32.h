#ifndef HKS_CRC32_H
#define HKS_CRC32_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HKS_CRC32_POLY_REFLECTED 0xEDB88320u
#define HKS_CRC32_INIT           0xFFFFFFFFu
#define HKS_CRC32_XOROUT         0xFFFFFFFFu

/**
 * Initialize a streaming IEEE 802.3 CRC32 state.
 * Pass the returned state to hks_crc32_update() and finalize with hks_crc32_final().
 */
uint32_t hks_crc32_init(void);

/**
 * Update a streaming IEEE 802.3 CRC32 state over [data, data + len).
 */
uint32_t hks_crc32_update(uint32_t state, const void *data, size_t len);

/**
 * Finalize a streaming IEEE 802.3 CRC32 state (applies XOROUT = 0xFFFFFFFF).
 */
uint32_t hks_crc32_final(uint32_t state);

/**
 * Compute one-shot IEEE 802.3 CRC32 over [data, data + len).
 * Vector: hks_crc32("123456789", 9) == 0xCBF43926u.
 */
uint32_t hks_crc32(const void *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* HKS_CRC32_H */
