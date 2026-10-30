#include "stm32h7xx.h"
extern unsigned long _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);
extern void __libc_init_array(void), SysTick_Handler(void), OTG_HS_IRQHandler(void);
void Default_Handler(void) { while(1) {} }
void Reset_Handler(void) {
    unsigned long *src=&_sidata;
    for(unsigned long *p=&_sdata;p<&_edata;) *p++=*src++;
    for(unsigned long *p=&_sbss;p<&_ebss;) *p++=0;
    SystemInit(); SCB->VTOR=0x08000000u; __DSB(); __ISB(); __libc_init_array(); (void)main(); while(1) {}
}
__attribute__((section(".isr_vector"),used))
void (*const vectors[16+163])(void)={
    [0]=(void (*)(void))(&_estack), [1]=Reset_Handler,
    [2 ... 14]=Default_Handler, [15]=SysTick_Handler,
    [16 ... 16+OTG_HS_IRQn-1]=Default_Handler,
    [16+OTG_HS_IRQn+1 ... 16+162]=Default_Handler,
    [16+OTG_HS_IRQn]=OTG_HS_IRQHandler
};
void _init(void) {}
void _fini(void) {}
