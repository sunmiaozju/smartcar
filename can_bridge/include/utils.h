/*
 * @Description: the bit read and write function
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-26 11:01:52
 * @LastEditTime: 2019-03-26 11:15:23
 */

#ifndef CANLIB_BITBUFFER_H
#define CANLIB_BITBUFFER_H

#include <cstdint>
#include <stdint-gcc.h>

namespace CanBridge {

#define GRP_OFFSET(x) ((x)&0x07)
#define BYT_OFFSET(x) ((x) >> 3)

inline int readBit(uint8_t* buf, int pos)
{
    int index = BYT_OFFSET(pos);
    int grp = GRP_OFFSET(pos);
    return (buf[index] >> grp) & 1;
}

inline int readAsInt(uint8_t* buf, int offset, int length)
{
    int ret = 0;
    for (int i = length - 1; i >= 0; --i) {
        ret = ret * 2 + readBit(buf, offset + i);
    }
    return ret;
}

static inline void setBit(uint8_t* buf, int pos)
{
    int index = BYT_OFFSET(pos);
    int grp = GRP_OFFSET(pos);
    buf[index] |= (1 << grp);
}

static inline void clearBit(uint8_t* buf, int pos)
{
    int index = BYT_OFFSET(pos);
    int grp = GRP_OFFSET(pos);
    uint8_t v = ~(1u << grp);
    buf[index] &= v;
}

inline void writeBit(uint8_t* buf, int pos, bool b)
{
    if (b)
        setBit(buf, pos);
    else
        clearBit(buf, pos);
}

inline void writeInt(uint8_t* buf, int offset, int length, int v)
{
    for (int i = 0; i < length; ++i) {
        if (v & 1) {
            setBit(buf, offset + i);
        } else {
            clearBit(buf, offset + i);
        }
        v >>= 1;
    }
}
}

#endif // !CANLIB_BITBUFFER_H