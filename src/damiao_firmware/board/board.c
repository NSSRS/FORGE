#include "board.h"
#include "wire.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>
extern USBD_HandleTypeDef hUsbDeviceHS;
static arm_controller arm;
static arm_wire wire;
static FDCAN_HandleTypeDef can[3];
static uint8_t rx[1024];
static volatile unsigned head,tail;
static volatile bool overflow;
/* Only one outstanding USB response. Buffer remains unchanged until USB completes. */
static uint8_t response[64];
static bool pending, in_flight;

void forge_usb_receive(const uint8_t *data,uint32_t n) {
    for(uint32_t i=0;i<n;i++) {
        unsigned next=(head+1)%sizeof(rx);
        if(next==tail) { overflow=true; return; }
        rx[head]=data[i]; __DMB(); head=next;
    }
}
static bool send_can(void *ctx,uint8_t bus,uint16_t id,const uint8_t *d,uint8_t n) {
    (void)ctx;
    FDCAN_TxHeaderTypeDef h={0};
    h.Identifier=id; h.IdType=FDCAN_STANDARD_ID; h.TxFrameType=FDCAN_DATA_FRAME;
    h.DataLength=(n==4)?FDCAN_DLC_BYTES_4:FDCAN_DLC_BYTES_8;
    h.ErrorStateIndicator=FDCAN_ESI_ACTIVE; h.BitRateSwitch=FDCAN_BRS_OFF;
    h.FDFormat=FDCAN_CLASSIC_CAN; h.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(&can[bus-1],&h,(uint8_t *)d)==HAL_OK;
}
static void init_can(unsigned i) {
    FDCAN_HandleTypeDef *h=&can[i];
    h->Instance=i==0?FDCAN1:i==1?FDCAN2:FDCAN3;
    h->Init.FrameFormat=FDCAN_FRAME_CLASSIC; h->Init.Mode=FDCAN_MODE_NORMAL;
    h->Init.AutoRetransmission=DISABLE; /* do not retain stale torque commands on bus failure */
    /* HSE=24 MHz, PLL M=2,N=40,Q=4 => 120 MHz CAN clock; 120 tq => 1 Mbps. */
    h->Init.NominalPrescaler=1; h->Init.NominalSyncJumpWidth=20;
    h->Init.NominalTimeSeg1=99; h->Init.NominalTimeSeg2=20;
    h->Init.DataPrescaler=1; h->Init.DataSyncJumpWidth=1;
    h->Init.DataTimeSeg1=1; h->Init.DataTimeSeg2=1;
    h->Init.MessageRAMOffset=i*256;
    h->Init.StdFiltersNbr=1; h->Init.RxFifo0ElmtsNbr=24;
    h->Init.RxFifo0ElmtSize=FDCAN_DATA_BYTES_8; h->Init.TxFifoQueueElmtsNbr=16;
    h->Init.TxFifoQueueMode=FDCAN_TX_FIFO_OPERATION; h->Init.TxElmtSize=FDCAN_DATA_BYTES_8;
    if(HAL_FDCAN_Init(h)!=HAL_OK) Error_Handler();
    FDCAN_FilterTypeDef f={.IdType=FDCAN_STANDARD_ID,.FilterIndex=0,.FilterType=FDCAN_FILTER_MASK,
        .FilterConfig=FDCAN_FILTER_TO_RXFIFO0,.FilterID1=0,.FilterID2=0};
    if(HAL_FDCAN_ConfigFilter(h,&f)!=HAL_OK ||
       HAL_FDCAN_ConfigGlobalFilter(h,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE)!=HAL_OK ||
       HAL_FDCAN_Start(h)!=HAL_OK) Error_Handler();
}
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *h) {
    __HAL_RCC_FDCAN_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
    RCC_PeriphCLKInitTypeDef c={0}; c.PeriphClockSelection=RCC_PERIPHCLK_FDCAN; c.FdcanClockSelection=RCC_FDCANCLKSOURCE_PLL;
    if(HAL_RCCEx_PeriphCLKConfig(&c)!=HAL_OK) Error_Handler();
    GPIO_InitTypeDef p={.Mode=GPIO_MODE_AF_PP,.Pull=GPIO_NOPULL,.Speed=GPIO_SPEED_FREQ_VERY_HIGH};
    if(h->Instance==FDCAN1) { p.Pin=GPIO_PIN_0|GPIO_PIN_1; p.Alternate=GPIO_AF9_FDCAN1; HAL_GPIO_Init(GPIOD,&p); }
    else if(h->Instance==FDCAN2) { p.Pin=GPIO_PIN_5|GPIO_PIN_6; p.Alternate=GPIO_AF9_FDCAN2; HAL_GPIO_Init(GPIOB,&p); }
    else { p.Pin=GPIO_PIN_12|GPIO_PIN_13; p.Alternate=GPIO_AF5_FDCAN3; HAL_GPIO_Init(GPIOD,&p); }
}
void forge_init(void) {
    arm_init(&arm,send_can,0); wire_init(&wire,&arm);
    for(unsigned i=0;i<3;i++) init_can(i);
    IWDG1->KR=0xcccc; IWDG1->KR=0x5555; IWDG1->PR=3; IWDG1->RLR=999;
    while(IWDG1->SR) {}
    IWDG1->KR=0xaaaa;
}
void forge_poll(void) {
    uint32_t now=HAL_GetTick();
    if(overflow) { arm.fault=true; arm_stop(&arm); tail=head; wire.used=0; overflow=false; }
    for(unsigned i=0;i<3;i++) {
        FDCAN_ProtocolStatusTypeDef status;
        if(HAL_FDCAN_GetProtocolStatus(&can[i],&status)!=HAL_OK || status.BusOff) { arm.fault=true; arm_stop(&arm); }
        /* Bounded drain so receive flooding cannot starve the command watchdog. */
        for(unsigned n=0;n<24 && HAL_FDCAN_GetRxFifoFillLevel(&can[i],FDCAN_RX_FIFO0);n++) {
            FDCAN_RxHeaderTypeDef h; uint8_t d[8];
            if(HAL_FDCAN_GetRxMessage(&can[i],FDCAN_RX_FIFO0,&h,d)!=HAL_OK) break;
            if(h.IdType==FDCAN_STANDARD_ID && h.RxFrameType==FDCAN_DATA_FRAME && h.DataLength==FDCAN_DLC_BYTES_8)
                arm_receive(&arm,i+1,h.Identifier,d,8,now);
        }
    }
    USBD_CDC_HandleTypeDef *cdc=(USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    bool connected=hUsbDeviceHS.dev_state==USBD_STATE_CONFIGURED && cdc;
    if(!connected) {
        bool running=false; for(unsigned i=0;i<6;i++) running|=arm.joints[i].enabled;
        if(running) { arm.fault=true; arm_stop(&arm); }
        pending=false; in_flight=false; tail=head; wire.used=0; wire.session=false;
    } else {
        if(in_flight && cdc->TxState==0) in_flight=false;
        if(!pending && !in_flight) {
            for(unsigned n=0;n<128 && tail!=head;n++) {
                uint8_t b=rx[tail]; __DMB(); tail=(tail+1)%sizeof(rx);
                if(wire_byte(&wire,b,now,response)) { pending=true; break; }
            }
        }
        if(pending && CDC_Transmit_HS(response,64)==USBD_OK) { pending=false; in_flight=true; }
    }
    arm_tick(&arm,now);
    IWDG1->KR=0xaaaa;
}
