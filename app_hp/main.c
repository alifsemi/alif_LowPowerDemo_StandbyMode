#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include <alif.h>
#include <RTE_Components.h>
#include <app_mem_regions.h>
#include <se_services_port.h>
#include <retarget_config.h>
#include <retarget_init.h>
#include <sys_clocks.h>
#include <drv_bkram.h>
#include <drv_mhu.h>
#include <lptimer.h>
#include <uart.h>
#include <pm.h>

#define MHU_VAL 0x1234
volatile uint32_t mhu_rx_value;

volatile uint32_t ms_ticks;
void SysTick_Handler (void) { ms_ticks++; }
void delay_ms (uint32_t msec) { msec += ms_ticks; while(ms_ticks < msec) __WFI(); }

void MHU_RTSS_S_TX_IRQHandler()
{
    MHU_SENDER_regs *MHU = (MHU_SENDER_regs *) RTSS_TX_MHU0_BASE;
    uint32_t int_st = MHU->INT_ST;
    MHU->INT_CLR = int_st;
}

void MHU_RTSS_S_RX_IRQHandler()
{
    MHU_RECEIVER_regs *MHU = (MHU_RECEIVER_regs *) RTSS_RX_MHU0_BASE;
    uint32_t int_st = MHU->INT_ST;

    uint32_t check_val;
    MHU_RECEIVER_Read(RTSS_RX_MHU0_BASE, 0, &check_val);
    MHU_RECEIVER_Clear(RTSS_RX_MHU0_BASE, 0, check_val);
    mhu_rx_value = check_val;

    MHU->INT_CLR = int_st;

    uint32_t count;
    bk_ram_rd(&count, BKRAM_INDEX_HP_RX_CNT);
    count++;
    bk_ram_wr(&count, BKRAM_INDEX_HP_RX_CNT);
}

static void boot_from_por()
{
    printf("RTSS-HP first boot\r\n\n");
    delay_ms(100);
}

static void boot_from_standby()
{
    uint32_t cycle_cnt;
    bk_ram_rd(&cycle_cnt, BKRAM_INDEX_HP_CYCLES);
    cycle_cnt++;
    bk_ram_wr(&cycle_cnt, BKRAM_INDEX_HP_CYCLES);
    printf("RTSS-HP resume count: %" PRIu32 "\r\n", cycle_cnt);

    NVIC_EnableIRQ(41);
    NVIC_EnableIRQ(42);

    uint32_t count;
    bk_ram_rd(&count, BKRAM_INDEX_HP_RX_CNT);
    printf("MHU interrupt count: %" PRIu32 " (RX)\r\n\n", count);
}

static void enter_standby()
{
    while(1) pm_core_enter_deep_sleep_request_subsys_off();
}

static void execute_while1_rtsshp()
{
    uint32_t active_ms;
    bk_ram_rd(&active_ms, BKRAM_INDEX_WHILE1);

    /* while(1) */
    active_ms += ms_ticks;
    while(ms_ticks < active_ms);

    MHU_SENDER_Set(RTSS_TX_MHU0_BASE, 0, MHU_VAL);
}

static bool PrintPendingIRQ()
{
    uint32_t wic_pending = 0;
    wic_pending |= NVIC->ISPR[0];
    wic_pending |= NVIC->ISPR[1];

    /* nothing to do if IRQs 0-63 are clear */
    if (wic_pending == 0) return false;

    /* Note: IRQ lines are shared in this multicore system,
     * you will see pending IRQs not meant for this core. */
    for (uint32_t i = 0; i < 64; i++) {
        if (NVIC_GetPendingIRQ(i)) {
            printf("IRQ%u is pending\r\n", i);
        }
    }

    /* For example: only MHU0 RX should wake the HP core */
    if (NVIC_GetPendingIRQ(41)) {
        return true;
    }

    return false;
}

static void uart_init()
{
#if defined(RTE_CMSIS_Compiler_STDIN_Custom)
    stdin_init();
#endif
#if defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    stdout_init();
#endif
}

/* call this function after making clock tree changes */
static void uart_update()
{
#if defined(RTE_CMSIS_Compiler_STDIN_Custom) || defined(RTE_CMSIS_Compiler_STDOUT_Custom)
#define _UART_BASE_(n)      UART##n##_BASE
#define UART_BASE(n)        _UART_BASE_(n)
    uart_set_baudrate((UART_Type*)UART_BASE(PRINTF_UART_CONSOLE), SystemAPBClock, PRINTF_UART_CONSOLE_BAUD_RATE);
#endif
}

int main (void)
{
    ms_ticks = 0;
    SystemCoreClock = 76800000;
    SystemAXIClock = 76800000;
    SystemAHBClock = SystemAXIClock >> 1;
    SystemAPBClock = SystemAXIClock >> 2;
    SysTick_Config(SystemCoreClock/1000);
    uart_init();

    bool wake_event = PrintPendingIRQ();
    if (wake_event) {
        boot_from_standby();
        execute_while1_rtsshp();
        enter_standby();
    }
    else {
        boot_from_por();
        enter_standby();
    }
    return 0;
}
