/*
 EXECICIO 2.9 Ponteiro 4
 
*/

#include <stdio.h>

void printArray(int vetor[]);

int main()
{
    int vetor[] = {0, 1, 2, 3, 4, 5};      
    printArray(vetor);	
    return 0;
}


void printArray(int vetor[]){
	int i;
    
    for(i = 0; i < 5; i++){
        printf("vetor[%d] = %d\n", i, vetor[i]);
    }
}