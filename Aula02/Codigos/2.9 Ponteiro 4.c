#include <stdio.h>
#include <stdlib.h>

int print_array (int teste_array[20])
{
    int *p,i;
    p = &teste_array[0];
    for (i = 0; i <20; i++)
        printf("teste_array[%d] = %d\n", i, p[i]);
}


int main(void)

{
    int my_array[20] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    print_array(my_array);
    return 0;
}