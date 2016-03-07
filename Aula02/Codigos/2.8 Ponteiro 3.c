/*
 EXECICIO 2.8 Ponteiro 3
 
*/

#include <stdio.h>


int main()
{
    int vetor[] = {0, 1, 2, 3, 4, 5}; 
    int i;
    
    for(i = 0; i < 5; i++){
        printf("vetor[%d] = %d\n", i, *(vetor + i));
    }
    
}