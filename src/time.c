#include "time.h"

uint32_t board_millis(void);

void delay_Ms(uint32_t delay){
    uint32_t _start = board_millis();
    while(1){
        if((board_millis() - _start) > delay)
            break;
    }
}
