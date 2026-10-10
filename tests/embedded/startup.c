#include <stdint.h>
extern uint32_t _stack_top, _data_start, _data_end, _data_load, _bss_start, _bss_end;
int main(void);
void smoke_exit(int code);
void Reset_Handler(void);
void Fault_Handler(void) { smoke_exit(99); }

__attribute__((section(".vectors"), used))
const uintptr_t vectors[16] = {
    (uintptr_t)&_stack_top, (uintptr_t)Reset_Handler,
    (uintptr_t)Fault_Handler, (uintptr_t)Fault_Handler,
    (uintptr_t)Fault_Handler, (uintptr_t)Fault_Handler,
    (uintptr_t)Fault_Handler, 0,0,0,0,(uintptr_t)Fault_Handler,
    (uintptr_t)Fault_Handler,0,(uintptr_t)Fault_Handler,(uintptr_t)Fault_Handler
};
void Reset_Handler(void)
{
    uint32_t *from=&_data_load;
    for (uint32_t *to=&_data_start; to<&_data_end; ++to) *to=*from++;
    for (uint32_t *to=&_bss_start; to<&_bss_end; ++to) *to=0;
    smoke_exit(main());
}
