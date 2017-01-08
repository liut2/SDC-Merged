#include "request.hh"


request::request(int id){
    printf("created request");
    printf("%d", id);
}

void request:: printid(){
    printf("id is: ");
    printf("%d", id);
}