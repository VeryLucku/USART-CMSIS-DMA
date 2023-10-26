#include "cmsis_init.h"

struct USART_buffer *buf;

void usart_init()
{
    // Enable tacting of ports
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);

    // usart2 ports - enable alternate function mode
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE2, 0b10 << GPIO_MODER_MODE2_Pos);
    MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE3, 0b10 << GPIO_MODER_MODE3_Pos);

    // change to usart2 mode
    MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL2, 0b0111 << GPIO_AFRL_AFSEL2_Pos);
    MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFSEL3, 0b0111 << GPIO_AFRL_AFSEL3_Pos);

    // enable tacting on usart2
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);

    uint32_t baud_rate = 115200;
    SystemCoreClockUpdate();
    USART2->BRR = (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]) / baud_rate;

    CLEAR_BIT(USART2->CR1, USART_CR1_M); // 1 start bit, 8 data bits, n stop bits
    CLEAR_BIT(USART2->CR1, USART_CR1_WAKE);
    CLEAR_BIT(USART2->CR1, USART_CR1_PCE);
    CLEAR_BIT(USART2->CR1, USART_CR1_PEIE);
    CLEAR_BIT(USART2->CR1, USART_CR1_TXEIE);
    CLEAR_BIT(USART2->CR1, USART_CR1_TCIE);
    CLEAR_BIT(USART2->CR1, USART_CR1_TXEIE);
    CLEAR_BIT(USART2->CR1, USART_CR1_IDLEIE);
    CLEAR_BIT(USART2->CR1, USART_CR1_RWU);

    SET_BIT(USART2->CR1, USART_CR1_IDLEIE);

    USART2->CR2 = 0;
    CLEAR_BIT(USART2->CR2, USART_CR2_STOP); // stop bits count

    USART2->CR3 = 0;
    SET_BIT(USART2->CR3, USART_CR3_DMAR);
    SET_BIT(USART2->CR3, USART_CR3_DMAT);

    SET_BIT(USART2->CR1, USART_CR1_TE);
    SET_BIT(USART2->CR1, USART_CR1_RE);
    SET_BIT(USART2->CR1, USART_CR1_UE);

    NVIC_EnableIRQ(USART2_IRQn);
}

void DMA_init()
{
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_DMA1EN);

    SET_BIT(RX_DMA_S->CR, RX_DMA_CH << DMA_SxCR_CHSEL_Pos);
    SET_BIT(TX_DMA_S->CR, TX_DMA_CH << DMA_SxCR_CHSEL_Pos);

    CLEAR_BIT(RX_DMA_S->CR, DMA_SxCR_MSIZE);
    CLEAR_BIT(TX_DMA_S->CR, DMA_SxCR_MSIZE);

    CLEAR_BIT(RX_DMA_S->CR, DMA_SxCR_PSIZE);
    CLEAR_BIT(TX_DMA_S->CR, DMA_SxCR_PSIZE);

    SET_BIT(RX_DMA_S->CR, DMA_SxCR_MINC);
    SET_BIT(TX_DMA_S->CR, DMA_SxCR_MINC);

    CLEAR_BIT(RX_DMA_S->CR, DMA_SxCR_DIR);
    SET_BIT(TX_DMA_S->CR, 0b01 << DMA_SxCR_DIR_Pos);

    SET_BIT(RX_DMA_S->PAR, (uint32_t)(&USART2->DR));
    SET_BIT(TX_DMA_S->PAR, (uint32_t)(&USART2->DR));

    SET_BIT(RX_DMA_S->M0AR, (uint32_t)buf->rx_buffer);
    SET_BIT(TX_DMA_S->M0AR, (uint32_t)buf->tx_buffer);

    SET_BIT(RX_DMA_S->NDTR, BUFFER_SIZE);

    SET_BIT(RX_DMA_S->CR, DMA_SxCR_CIRC);
    CLEAR_BIT(TX_DMA_S->CR, DMA_SxCR_CIRC);

    SET_BIT(RX_DMA_S->CR, DMA_SxCR_TCIE);
    SET_BIT(TX_DMA_S->CR, DMA_SxCR_TCIE);

    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    SET_BIT(RX_DMA_S->CR, DMA_SxCR_EN);
}

void set_husart_buffer(struct USART_buffer *buffer) {
    buf = buffer;
}
