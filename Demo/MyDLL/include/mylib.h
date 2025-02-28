//
// Created by hyl on 25-2-28.
//

#ifndef HYLREG_MYDLL_H
#define HYLREG_MYDLL_H

#pragma once

#ifdef _WIN32
#ifdef MYLIB_EXPORTS
        #define MYLIB_API __declspec(dllexport)
    #else
        #define MYLIB_API __declspec(dllimport)
    #endif
#else
#define MYLIB_API __attribute__((visibility("default")))
#endif

extern "C" MYLIB_API int add(int a, int b);
extern "C" MYLIB_API int subtract(int a, int b);


#endif //HYLREG_MYDLL_H
