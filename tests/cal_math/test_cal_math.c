#include "../../src/cal_math.h"
#include <stdio.h>

int main(void)
{
    int fails = cal_math_selftest();
    printf("cal_math_selftest: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
