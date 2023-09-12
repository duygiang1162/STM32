#include "main.h"

int main(void) {
GPIO_InitTypeDef GPIO_InitStruct;
HAL_Init();
__HAL_RCC_GPIOC_CLK_ENABLE();
/* Configure GPIO pin : PC4 */
GPIO_InitStruct.Pin = GPIO_PIN_4;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
/* Configure GPIO pin : PC8 */
GPIO_InitStruct.Pin = GPIO_PIN_8;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
while(1) {
GPIOC->ODR = 0x110;
GPIOC->ODR = 0;
}
}