//
// Created by hyl on 25-2-28.
//

#define MYDLL_EXPORTS // 定义导出标识符

#include "../include/mylib.h"


MYLIB_API int add(int a, int b) {
    return a + b;
}

MYLIB_API int subtract(int a, int b) {
    return a - b;
}
