#include <stdio.h>
#include <stdlib.h>

int *primos(void)
{
    int *v, i;
    v[0] = 1009;
    v[1] = 1013;
    v[2] = 1019;
    for (i = 0; i < 3; i++)
    printf("%d\n", v[i]);
    return v;

}

int main(void)

{
    *primos();
    return 0;
}