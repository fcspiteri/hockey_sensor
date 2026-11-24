#ifndef HKS_FREESTANDING_STRING_H
#define HKS_FREESTANDING_STRING_H

#include <stddef.h>

static inline void *memcpy(void *dst, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    for (size_t i = 0u; i < n; i++) {
        d[i] = s[i];
    }
    return dst;
}

static inline void *memmove(void *dst, const void *src, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;
    if (d < s) {
        for (size_t i = 0u; i < n; i++) {
            d[i] = s[i];
        }
    } else if (d > s) {
        for (size_t i = n; i > 0u; i--) {
            d[i - 1u] = s[i - 1u];
        }
    }
    return dst;
}

static inline void *memset(void *dst, int c, size_t n) {
    unsigned char *d = (unsigned char *)dst;
    for (size_t i = 0u; i < n; i++) {
        d[i] = (unsigned char)c;
    }
    return dst;
}

static inline int memcmp(const void *a, const void *b, size_t n) {
    const unsigned char *pa = (const unsigned char *)a;
    const unsigned char *pb = (const unsigned char *)b;
    for (size_t i = 0u; i < n; i++) {
        if (pa[i] != pb[i]) {
            return (pa[i] < pb[i]) ? -1 : 1;
        }
    }
    return 0;
}

#endif /* HKS_FREESTANDING_STRING_H */
