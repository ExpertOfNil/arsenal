#ifndef ARENA_H
#define ARENA_H

#include <stdalign.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/mman.h>
#include <unistd.h>

#define ARENA_SIZE_DEFAULT (1024u * 1024u)
#ifdef __cplusplus
#include <cstddef>
#define ALIGNOF(T) alignof(T)
#define ALIGNOF_MAX ALIGNOF(std::max_align_t)
#else
#define ALIGNOF(T) _Alignof(T)
#define ALIGNOF_MAX ALIGNOF(max_align_t)
#endif


static inline size_t next_pow2(size_t v) {
    if (v == 0) return 1;
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
#if SIZE_MAX > 0xffffffffu
    v |= v >> 32;
#endif
    v++;
    return v;
}

static inline int is_power_of_two(size_t x) {
    return x && ((x & (x - 1)) == 0);
}

static inline int add_checked_size(size_t a, size_t b, size_t* out) {
    if (a > SIZE_MAX - b) return -1;
    *out = a + b;
    return 0;
}

static inline int add_checked_ptr(uintptr_t a, uintptr_t b, uintptr_t* out) {
    if (a > UINTPTR_MAX - b) return -1;
    *out = a + b;
    return 0;
}

/* Round up to the nearest power of 2 */
static inline int round_up(size_t n, size_t value, size_t* out) {
    if (!is_power_of_two(value)) return -1;
    size_t mask = value - 1;
    if (n > SIZE_MAX - mask) return -1;
    *out = (n + mask) & ~mask;
    return 0;
}

static inline int align_up(uintptr_t p, size_t align, uintptr_t* out) {
    if (align == 0) return -1;
    uintptr_t mask = (uintptr_t)align - 1;
    if (p > UINTPTR_MAX - mask) return -1;
    *out = (p + mask) & ~mask;
    return 0;
}

static inline size_t clamp_align(size_t align) {
    size_t min = ALIGNOF_MAX;
    if (align < min) align = min;
    if (align > (SIZE_MAX >> 1) + 1) return 0;
    return next_pow2(align);
}

typedef struct Arena {
    uint8_t* base;
    size_t offset;
    size_t capacity;
} Arena;

static inline int Arena_init(Arena* arena, size_t capacity) {
    if (capacity == 0) capacity = ARENA_SIZE_DEFAULT;

    long page_l = sysconf(_SC_PAGESIZE);
    if (page_l <= 0) return -1;
    size_t page = (size_t)page_l;

    size_t mmap_size = 0;
    if (round_up(capacity, page, &mmap_size) != 0) return -1;
    arena->base = (uint8_t*)mmap(
        NULL,
        mmap_size,
        PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS,
        -1,
        0
    );
    if (arena->base == MAP_FAILED) {
        return -1;
    }
    arena->capacity = mmap_size;
    arena->offset = 0;
    return 0;
}

static inline int Arena_init_default(Arena* arena) {
    return Arena_init(arena, ARENA_SIZE_DEFAULT);
}

#define ARENA_ALLOC(arena, T) ((T*)Arena_alloc((arena), sizeof(T), ALIGNOF(T)))
#define ARENA_ALLOC_N(arena, T, n) \
    ((T*)Arena_alloc((arena), (sizeof(T) * (size_t)(n)), ALIGNOF(T)))
static inline void* Arena_alloc(Arena* arena, size_t size, size_t align) {
    if (arena == NULL || arena->base == NULL || arena->base == MAP_FAILED) {
        return NULL;
    }
    uintptr_t p = 0;
    uintptr_t base = (uintptr_t)arena->base;
    if (add_checked_ptr(base, (uintptr_t)arena->offset, &p) != 0) {
        return NULL;
    }
    align = clamp_align(align);
    uintptr_t aligned = 0;
    if (align_up(p, align, &aligned) != 0) return NULL;
    size_t used = (size_t)(aligned - base);
    size_t new_offset = 0;
    if (add_checked_size(used, size, &new_offset) != 0) {
        return NULL;
    }
    if (new_offset > arena->capacity) return NULL;
    arena->offset = new_offset;
    return (void*)aligned;
}

static inline void Arena_reset(Arena* arena) {
    if (arena != NULL) {
        arena->offset = 0;
    }
}

static inline void Arena_free(Arena* arena) {
    if (arena != NULL) {
        if (arena->base != NULL && arena->base != MAP_FAILED) {
            munmap(arena->base, arena->capacity);
        }
    }
    arena->base = NULL;
    arena->offset = 0;
    arena->capacity = 0;
}

#endif /* ARENA_H */
