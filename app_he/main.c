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

#define LPT_CH  0
volatile uint32_t lpt_irq;

#define MHU_VAL 0x1234
volatile uint32_t mhu_rx_value;

volatile uint32_t ms_ticks;
void SysTick_Handler (void) { ms_ticks++; }
void delay_ms (uint32_t msec) { msec += ms_ticks; while(ms_ticks < msec) __WFI(); }
extern int32_t get_int_input();

void LPTIMER0_IRQHandler()
{
    LPTIMER_Type *lptimer = (LPTIMER_Type *) LPTIMER_BASE;
    lptimer_clear_interrupt(lptimer, 0);
    lpt_irq = 1;

    uint32_t count;
    bk_ram_rd(&count, BKRAM_INDEX_LPT0_COUNT);
    count++;
    bk_ram_wr(&count, BKRAM_INDEX_LPT0_COUNT);
}

void LPTIMER1_IRQHandler()
{
    LPTIMER_Type *lptimer = (LPTIMER_Type *) LPTIMER_BASE;
    lptimer_clear_interrupt(lptimer, 1);
    lpt_irq = 1;

    uint32_t count;
    bk_ram_rd(&count, BKRAM_INDEX_LPT1_COUNT);
    count++;
    bk_ram_wr(&count, BKRAM_INDEX_LPT1_COUNT);
}

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
    bk_ram_rd(&count, BKRAM_INDEX_HE_RX_CNT);
    count++;
    bk_ram_wr(&count, BKRAM_INDEX_HE_RX_CNT);
}

static void reset_hp()
{
    *(volatile uint32_t*)0x1A010310 = 3U;
    while(*(volatile uint32_t*)0x1A010314 != 4) __WFI();
    *(volatile uint32_t*)0x1A010310 = 1U;
}

static void boot_from_por()
{
    printf("RTSS-HE first boot\r\n\n");

    printf("Wake up period in milliseconds (e.g. 10ms to 10000ms)\r\n");
    printf("> 1000");
    int32_t sleep_ms = 1000;//get_int_input();

    printf("\r\nTime spent running while(1) (e.g. 1ms to 1000ms)\r\n");
    printf("> 100");
    int32_t active_ms = 100;//get_int_input();

    printf("\r\nStarting Power cycle demo\r\n\n");

    /* Initialize the SE services */
    uint32_t ret, response;
    se_services_port_init();

    /* Request the SECENC to power itself down */
    ret = SERVICES_power_se_sleep_req(se_services_s_handle, 0, &response);
    if (ret || response) while(1);

    /* turn off DEBUG and SYSTOP */
    *(volatile uint32_t*)0x1A010400 = 0;

    /* Flex GPIO are typically 1.8V */
    VBAT->GPIO_CTRL = 0x11;

    /* Retain HE-TCM and 4kB Backup SRAM */
    VBAT->RET_CTRL = 0x3FF00;

    /* Retain HE-TCM and 4kB and SRAM0 */
    // VBAT->RET_CTRL = 0x30000;   

    /* Retain HE-TCM and 4kB and SRAM0+1 */
    // VBAT->RET_CTRL = 0x00000;

    /* Backup HE's VTOR value to VBAT */
    *(volatile uint32_t *)(0x1A60A024UL) = SCB->VTOR;

    /* adjust the internal dc-dc output voltage */
    uint32_t reg_data, dcdc_trim;
    reg_data = ANA->DCDC_REG1;
    dcdc_trim = ((reg_data >> 3) & 63U) - 15;
    reg_data &= ~(63U << 3);
    reg_data |= (dcdc_trim << 3);
    ANA->DCDC_REG1 = reg_data;

    /* adjust other low-level settings */
    ANA->DCDC_REG2 |= (1U << 23);       // put DCDC in PFM mode
    ANA->VBAT_ANA_REG1 |= (15U << 8);   // enable RET LDOs
    ANA->VBAT_ANA_REG2 |=  (5U << 19);  // select 600kHz standby mode clock

    /* Clear the Backup RAM */
    uint32_t bk_data = 0;
    for (int i = 0; i < 100; i++) {
        bk_ram_wr(&bk_data, i);
    }

    bk_data = 0xB007ED;
    bk_ram_wr(&bk_data, BKRAM_INDEX_FIRSTBOOT);
    bk_ram_wr(&active_ms, BKRAM_INDEX_WHILE1);

    uint32_t lptimer_count = roundf(sleep_ms * 32.768) - 1;
    LPTIMER_Type *lptimer = (LPTIMER_Type *) LPTIMER_BASE;
    lptimer_load_count(lptimer, LPT_CH, &lptimer_count);
    lptimer_set_mode_userdefined(lptimer, LPT_CH);
    lptimer_enable_counter(lptimer, LPT_CH);
    lptimer_clear_interrupt(lptimer, LPT_CH);

    NVIC_ClearPendingIRQ(60 + LPT_CH);
    NVIC_EnableIRQ(60 + LPT_CH);
}

static void boot_from_standby()
{
    uint32_t cycle_cnt;
    bk_ram_rd(&cycle_cnt, BKRAM_INDEX_HE_CYCLES);
    cycle_cnt++;
    bk_ram_wr(&cycle_cnt, BKRAM_INDEX_HE_CYCLES);
    printf("RTSS-HE resume count: %" PRIu32 "\r\n", cycle_cnt);

    NVIC_EnableIRQ(41);
    NVIC_EnableIRQ(42);
    NVIC_EnableIRQ(60 + LPT_CH);

    uint32_t count1, count2;
    bk_ram_rd(&count1, BKRAM_INDEX_LPT0_COUNT + LPT_CH);
    bk_ram_rd(&count2, BKRAM_INDEX_HE_RX_CNT);
    printf("LPTIMER interrupt count: %" PRIu32 "\r\n", count1);
    printf("MHU interrupt count: %" PRIu32 " (RX)\r\n\n", count2);

    /* code prior to this line must be in TCM */
    ANA->VBAT_ANA_REG2 |= (1U << 5);    // enable MRAM LDO
}

static void enter_standby()
{
    delay_ms(5); /* small delay for UART prints to finish */

    /* code after this line must be in TCM */
    ANA->VBAT_ANA_REG2 &= ~(1U << 5);    // disable MRAM LDO

    while(1) pm_core_enter_deep_sleep_request_subsys_off();
}

static void execute_while1()
{
    uint32_t active_ms;
    bk_ram_rd(&active_ms, BKRAM_INDEX_WHILE1);

    /* while(1) */
    active_ms += ms_ticks;
    while(ms_ticks < active_ms);
}

static void execute_while1_rtsshp()
{
    uint32_t cycle_cnt;
    bk_ram_rd(&cycle_cnt, BKRAM_INDEX_HE_CYCLES);

    /* only proceed during 10th boot cycle */
    if (cycle_cnt % 10) return;
    printf("RTSS-HE sending message to HP\r\n");

    /* put DCDC in PWM mode for stability at high load */
    ANA->DCDC_REG2 &= ~(1U << 23);

    /* turn on SYSTOP for use by the HP core */
    *(volatile uint32_t*)0x1A010400 |= (1U << 5);

    /* interrupt the HP core via MHU */
    MHU_SENDER_Set(RTSS_TX_MHU0_BASE, 0, MHU_VAL);

    /* HE core stays in deep sleep until HP responds */
    lpt_irq = 0;
    mhu_rx_value = 0;
    while ((mhu_rx_value == 0) && (lpt_irq == 0)) {
        pm_core_enter_deep_sleep();
    }

    if (lpt_irq) {
        printf("RTSS-HE continuing without reply\r\n\n");
        reset_hp();
    }
    else {
        printf("RTSS-HE received message from HP\r\n\n");
    }

    /* turn off SYSTOP no longer used by the HP core */
    *(volatile uint32_t*)0x1A010400 &= ~(1U << 5);

    /* put DCDC in PFM mode after SYSTOP and HP are off */
    ANA->DCDC_REG2 |= (1U << 23);
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

    /* For example: only LPTIMER should wake the HE core */
    if (NVIC_GetPendingIRQ(60 + LPT_CH)) {
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

/* call this function after any change to SystemCoreClock */
static void uart_update()
{
#if defined(RTE_CMSIS_Compiler_STDIN_Custom) || defined(RTE_CMSIS_Compiler_STDOUT_Custom)
    uart_set_baudrate((UART_Type*)LPUART_BASE, SystemCoreClock, PRINTF_UART_CONSOLE_BAUD_RATE);
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
        execute_while1();
        execute_while1_rtsshp();
        enter_standby();
    }
    else {
        boot_from_por();
        enter_standby();
    }
    return 0;
}
