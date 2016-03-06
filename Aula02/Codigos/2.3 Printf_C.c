//2.3 Printf_C
#include <stdio.h>

int main()
{
    printf("Vinte primeiros numeros primos \n");
    int i;
    int vetor[] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71};
    
    for (i = 0; i<=19; i++)
    {
        printf("Primo %d = %d\n", i+1, vetor[i]);
    }
    return 0;
}
