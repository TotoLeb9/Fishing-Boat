/*
 * log.h
 *
 *  Created on: Dec 11, 2025
 *      Author: totoleb
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

#pragma once
#include <stdio.h>

#define LOG_LEVEL_NONE   0
#define LOG_LEVEL_ERROR  1
#define LOG_LEVEL_WARN   2
#define LOG_LEVEL_INFO   3
#define LOG_LEVEL_DEBUG  4

#ifdef DEBUG
    #define LOG_LEVEL LOG_LEVEL_DEBUG
#else
    #define LOG_LEVEL LOG_LEVEL_NONE
#endif

#if LOG_LEVEL >= LOG_LEVEL_ERROR
    #define LOG_ERROR(...)   printf("(E) " __VA_ARGS__)
#else
    #define LOG_ERROR(...)   do {} while(0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
    #define LOG_WARN(...)    printf("(W) " __VA_ARGS__)
#else
    #define LOG_WARN(...)    do {} while(0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
    #define LOG_INFO(...)    printf("(I) " __VA_ARGS__)
#else
    #define LOG_INFO(...)    do {} while(0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
    #define LOG_DEBUG(...)   printf("(D) " __VA_ARGS__)
#else
    #define LOG_DEBUG(...)   do {} while(0)
#endif

#endif /* INC_LOG_H_ */
