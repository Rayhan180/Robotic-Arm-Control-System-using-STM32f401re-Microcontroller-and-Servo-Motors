#include "stm32f4xx.h"

#define CONTROL_MODE_POT   (1U)
#define CONTROL_MODE_I2C   (2U)
#define CONTROL_MODE       (CONTROL_MODE_POT)

#define SERVO_COUNT        (5U)
#define SERVO_PERIOD_US    (20000U)
#define SERVO_MIN_PULSE_US (1000U)
#define SERVO_MAX_PULSE_US (2000U)

#define I2C_SLAVE_ADDR_7B  (0x28U)
#define I2C_RX_BYTES       (SERVO_COUNT)

static volatile uint8_t g_i2c_rx_buf[I2C_RX_BYTES];
static volatile uint8_t g_i2c_rx_index = 0U;
static volatile uint8_t g_i2c_frame_ready = 0U;

/* tiny delay (bring-up use) */
static void delay_cycles(volatile uint32_t cycles)
{
    while (cycles--)
    {
        __NOP();
    }
}

/* 84 MHz SYSCLK from HSI via PLL */
static void clock_init_84mhz_hsi_pll(void)
{
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {;}

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= (RCC_CFGR_HPRE_DIV1 |
                  RCC_CFGR_PPRE1_DIV2 |
                  RCC_CFGR_PPRE2_DIV1);

    RCC->PLLCFGR = 0U;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;
    RCC->PLLCFGR |= (16U  << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (336U << RCC_PLLCFGR_PLLN_Pos);

    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
    RCC->PLLCFGR |= (1U << RCC_PLLCFGR_PLLP_Pos); /* PLLP = 4 => 336/4 = 84MHz */

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {;}

    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {;}
}

/* TIM3(4ch) + TIM4(1ch) @ 1MHz tick, 20ms period */
static void pwm_init_servos(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;

    /* TIM3: PA6/PA7 AF2, PB0/PB1 AF2 */
    GPIOA->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOA->MODER |=  (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
    GPIOA->AFR[0] &= ~((0xFU << (6U * 4U)) | (0xFU << (7U * 4U)));
    GPIOA->AFR[0] |=  ((2U  << (6U * 4U)) | (2U  << (7U * 4U)));

    GPIOB->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
    GPIOB->MODER |=  (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1);
    GPIOB->AFR[0] &= ~((0xFU << (0U * 4U)) | (0xFU << (1U * 4U)));
    GPIOB->AFR[0] |=  ((2U  << (0U * 4U)) | (2U  << (1U * 4U)));

    /* TIM4: PB6 AF2 */
    GPIOB->MODER &= ~(GPIO_MODER_MODE6);
    GPIOB->MODER |=  (GPIO_MODER_MODE6_1);
    GPIOB->AFR[0] &= ~(0xFU << (6U * 4U));
    GPIOB->AFR[0] |=  (2U  << (6U * 4U));

    /* TIM3 base */
    TIM3->PSC = (84U - 1U);
    TIM3->ARR = (SERVO_PERIOD_US - 1U);
    TIM3->CR1 |= TIM_CR1_ARPE;

    /* PWM mode 1 + preload */
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S | TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM3->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_CC4S | TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);

    TIM3->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM3->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM3->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM3->CCMR2 |= (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

    TIM3->CCR1 = SERVO_MIN_PULSE_US;
    TIM3->CCR2 = SERVO_MIN_PULSE_US;
    TIM3->CCR3 = SERVO_MIN_PULSE_US;
    TIM3->CCR4 = SERVO_MIN_PULSE_US;

    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM3->EGR |= TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;

    /* TIM4 base */
    TIM4->PSC = (84U - 1U);
    TIM4->ARR = (SERVO_PERIOD_US - 1U);
    TIM4->CR1 |= TIM_CR1_ARPE;

    TIM4->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1M);
    TIM4->CCMR1 |=  (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

    TIM4->CCR1 = SERVO_MIN_PULSE_US;
    TIM4->CCER |= TIM_CCER_CC1E;

    TIM4->EGR |= TIM_EGR_UG;
    TIM4->CR1 |= TIM_CR1_CEN;
}

/* ADC1: PA0..PA4 (IN0..IN4), single conversion polling */
static void adc_init_pots(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    GPIOA->MODER |= (GPIO_MODER_MODE0 |
                     GPIO_MODER_MODE1 |
                     GPIO_MODER_MODE2 |
                     GPIO_MODER_MODE3 |
                     GPIO_MODER_MODE4);

    /* ADC prescaler / common settings */
    ADC->CCR = 0U;

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;

    /* sample time: ~84 cycles for ch0..ch4 */
    ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4);
    ADC1->SMPR2 |=  (ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP2_2 | ADC_SMPR2_SMP3_2 | ADC_SMPR2_SMP4_2);

    ADC1->CR2 |= ADC_CR2_ADON;
    delay_cycles(10000U);
}

static uint16_t read_adc_channel(uint8_t channel)
{
    ADC1->SQR1 = 0U;                         /* 1 conversion */
    ADC1->SQR3 = (uint32_t)(channel & 0x1FU);

    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while ((ADC1->SR & ADC_SR_EOC) == 0U) {;}
    return (uint16_t)(ADC1->DR & 0xFFFFU);
}

static uint16_t map_adc_to_pulse_us(uint16_t adc)
{
    uint32_t range = (uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    uint32_t pulse = (uint32_t)SERVO_MIN_PULSE_US + ((uint32_t)adc * range) / 4095U;

    if (pulse < SERVO_MIN_PULSE_US) pulse = SERVO_MIN_PULSE_US;
    if (pulse > SERVO_MAX_PULSE_US) pulse = SERVO_MAX_PULSE_US;

    return (uint16_t)pulse;
}

static uint16_t map_angle_to_pulse_us(uint8_t angle)
{
    if (angle > 180U) angle = 180U;

    uint32_t range = (uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    uint32_t pulse = (uint32_t)SERVO_MIN_PULSE_US + ((uint32_t)angle * range) / 180U;

    if (pulse < SERVO_MIN_PULSE_US) pulse = SERVO_MIN_PULSE_US;
    if (pulse > SERVO_MAX_PULSE_US) pulse = SERVO_MAX_PULSE_US;

    return (uint16_t)pulse;
}

static void servo_set_pulse_us(uint8_t idx, uint16_t pulse)
{
    if (pulse < SERVO_MIN_PULSE_US) pulse = SERVO_MIN_PULSE_US;
    if (pulse > SERVO_MAX_PULSE_US) pulse = SERVO_MAX_PULSE_US;

    switch (idx)
    {
        case 0U: TIM3->CCR1 = pulse; break;
        case 1U: TIM3->CCR2 = pulse; break;
        case 2U: TIM3->CCR3 = pulse; break;
        case 3U: TIM3->CCR4 = pulse; break;
        case 4U: TIM4->CCR1 = pulse; break;
        default: break;
    }
}

/* I2C1 slave RX: PB8=SCL, PB9=SDA (AF4) */
static void i2c1_slave_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9);
    GPIOB->MODER |=  (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1);

    GPIOB->OTYPER |= (GPIO_OTYPER_OT8 | GPIO_OTYPER_OT9);
    GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPD8 | GPIO_PUPDR_PUPD9);
    GPIOB->PUPDR  |=  (GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0);

    GPIOB->AFR[1] &= ~((0xFU << ((8U - 8U) * 4U)) | (0xFU << ((9U - 8U) * 4U)));
    GPIOB->AFR[1] |=  ((4U  << ((8U - 8U) * 4U)) | (4U  << ((9U - 8U) * 4U)));

    I2C1->CR1 = 0U;
    I2C1->CR2 = 42U; /* APB1 MHz (42MHz) */

    I2C1->OAR1 = 0U;
    I2C1->OAR1 |= (uint32_t)(I2C_SLAVE_ADDR_7B << 1U);
    I2C1->OAR1 |= I2C_OAR1_OA1EN;

    /* Keep valid timing values (mainly relevant for master mode) */
    I2C1->CCR   = 210U;
    I2C1->TRISE = 43U;

    I2C1->CR1 |= I2C_CR1_ACK;

    I2C1->CR2 |= I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;

    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);

    I2C1->CR1 |= I2C_CR1_PE;

    g_i2c_rx_index = 0U;
    g_i2c_frame_ready = 0U;
}

/* I2C1 event ISR: ADDR/RXNE/STOPF */
void I2C1_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;

    if (sr1 & I2C_SR1_ADDR)
    {
        (void)I2C1->SR2;            /* clear ADDR */
        g_i2c_rx_index = 0U;
        g_i2c_frame_ready = 0U;
        return;
    }

    if (sr1 & I2C_SR1_RXNE)
    {
        uint8_t b = (uint8_t)I2C1->DR;

        if (g_i2c_rx_index < I2C_RX_BYTES)
        {
            g_i2c_rx_buf[g_i2c_rx_index++] = b;

            if (g_i2c_rx_index >= I2C_RX_BYTES)
            {
                g_i2c_frame_ready = 1U;
            }
        }
        return;
    }

    if (sr1 & I2C_SR1_STOPF)
    {
        (void)I2C1->SR1;            /* clear STOPF sequence */
        I2C1->CR1 |= I2C_CR1_PE;    /* write CR1 to clear STOPF */
        return;
    }
}

/* I2C1 error ISR */
void I2C1_ER_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;
    (void)sr1;

    I2C1->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR);
}

int main(void)
{
    clock_init_84mhz_hsi_pll();
    pwm_init_servos();
    adc_init_pots();

#if (CONTROL_MODE == CONTROL_MODE_I2C)
    i2c1_slave_init();
#endif

    while (1)
    {
#if (CONTROL_MODE == CONTROL_MODE_POT)

        uint16_t a0 = read_adc_channel(0U);
        uint16_t a1 = read_adc_channel(1U);
        uint16_t a2 = read_adc_channel(2U);
        uint16_t a3 = read_adc_channel(3U);
        uint16_t a4 = read_adc_channel(4U);

        servo_set_pulse_us(0U, map_adc_to_pulse_us(a0));
        servo_set_pulse_us(1U, map_adc_to_pulse_us(a1));
        servo_set_pulse_us(2U, map_adc_to_pulse_us(a2));
        servo_set_pulse_us(3U, map_adc_to_pulse_us(a3));
        servo_set_pulse_us(4U, map_adc_to_pulse_us(a4));

        delay_cycles(50000U);

#elif (CONTROL_MODE == CONTROL_MODE_I2C)

        if (g_i2c_frame_ready != 0U)
        {
            uint8_t s0 = g_i2c_rx_buf[0U];
            uint8_t s1 = g_i2c_rx_buf[1U];
            uint8_t s2 = g_i2c_rx_buf[2U];
            uint8_t s3 = g_i2c_rx_buf[3U];
            uint8_t s4 = g_i2c_rx_buf[4U];

            servo_set_pulse_us(0U, map_angle_to_pulse_us(s0));
            servo_set_pulse_us(1U, map_angle_to_pulse_us(s1));
            servo_set_pulse_us(2U, map_angle_to_pulse_us(s2));
            servo_set_pulse_us(3U, map_angle_to_pulse_us(s3));
            servo_set_pulse_us(4U, map_angle_to_pulse_us(s4));

            g_i2c_frame_ready = 0U;
        }

#endif
    }
}
