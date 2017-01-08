#include "instruction.hh"


instruction::instruction(int id){
    printf("created instruction");
    printf("%d", id);
}

void instruction:: printid(){
    printf("id is: ");
    printf("%d", id);
}