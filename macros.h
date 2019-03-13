#ifndef __MACROS_H
#define __MACROS_H

#include <stddef.h>
#include <cstdio>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MAKEWORD(a, b)  		(((uint16_t)(((uint8_t)(a) << 8)) | ((uint16_t) ( (uint8_t) (b)))))

/* Get 8 bits of 16 bit value. */
#define LO8(x)                  ((uint8_t) ((x) & 0xFFu))
#define HI8(x)                  ((uint8_t) ((uint16_t)(x) >> 8))

/* Get 16 bits of 32 bit value. */
#define LO16(x)                 ((uint16_t) ((x) & 0xFFFFu))
#define HI16(x)                 ((uint16_t) ((uint32_t)(x) >> 16))

/* Swap the byte ordering of 32 bit value */
#define SWAP_ENDIAN32(x)  \
        ((uint32_t)((((x) >> 24) & 0x000000FFu) | (((x) & 0x00FF0000u) >> 8) | (((x) & 0x0000FF00u) << 8) | ((x) << 24)))

/* Swap the byte ordering of 16 bit value */
#define SWAP_ENDIAN16(x)      	((uint16_t)(((x) << 8) | (((x) >> 8) & 0x00FFu)))


//----------------------------------------------------------------------------------
// Helper functions
//----------------------------------------------------------------------------------
inline void hex_dump(const void *src, size_t length, size_t line_size, const char *prefix)
{
    int i = 0;
    const unsigned char *address = (unsigned char*) src;
    const unsigned char *line = address;
    unsigned char c;

    std::printf("%s | ", prefix);

    while (length-- > 0) {
        std::printf("%02X ", *address++);

        if (!(++i % line_size) || (length == 0 && i % line_size)) {
            if (length == 0) {
                while (i++ % line_size)
                    printf("__ ");
            }
            printf(" |");
            while (line < address) {
                c = *line++;
                printf("%c", (c < 32 || c > 126) ? '.' : c);
            }
            printf("|\n");
            if (length > 0)
                printf("%s | ", prefix);
        }
    }
}

#endif