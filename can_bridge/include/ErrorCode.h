/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-03-25 20:14:45
 * @LastEditTime: 2019-03-25 20:50:56
 */
#ifndef CANLIB_ERRORCODE_H
#define CANLIB_ERRORCODE_H

enum ErrorCode {
    SUCCESS = 0,
    FAILED = -1,
    NULL_VALUE = -2,
    REPEATED_KEY = -3,
    TIMEOUT = -4,
    INTERRUPTED = -5,
    ALLOCATE_ERROR = -6,
    DEVICE_NOT_FOUND = -7
};

#endif // !CANLIB_ERRORCODE_H