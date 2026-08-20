#include "../../src/cal_run_math.h"
#include "../../src/cal_math.h"
#include <stdio.h>

int main(void)
{
    int fails = cal_run_math_selftest();

    printf("cal_run_math_selftest: %d failure(s)\n", fails);
    return fails == 0 ? 0 : 1;
}
