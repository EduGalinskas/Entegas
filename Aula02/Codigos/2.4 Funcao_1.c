//2.4 Função 1

#include <stdio.h>

int Calc_MDC (int num1, int num2);


int main()
{
    int num1, num2;
    
    printf("Calcula MDC\n");
    
    printf("Numero 1: \n");
    scanf("%d", &num1);
   
    printf("Numero 2: \n");
    scanf("%d", &num2);
    
    printf("H.C.F of %d and %d is %d \n", num1, num2, Calc_MDC(num1, num2));

    return 0;
}


int Calc_MDC (int num1, int num2){
    int hcf,i;
    for ( i = 1; i <= num1 || i <= num2; ++i)
    {
        if (num1%i == 0 && num2%i == 0) {
            hcf = i;
        }
    }
    return hcf;
}
