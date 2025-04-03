#include "gpio.h"


/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10  ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/


TIM_HandleTypeDef htim2, htim3;


void MX_GPIO_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                            |Audio_RST_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = PDM_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = I2S3_WS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PAPin PAPin PAPin */
    GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = BOOT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = CLK_IN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                            PDPin */
    GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                            |Audio_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PCPin PCPin PCPin */
    GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = VBUS_FS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PAPin PAPin PAPin */
    GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PBPin PBPin */
    GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  //   GPIO_InitTypeDef  gpio_init_structure_led;
  //   gpio_init_structure_led.Pin   = GPIO_PIN_12;
  //   gpio_init_structure_led.Mode  = GPIO_MODE_OUTPUT_PP;
  //   gpio_init_structure_led.Pull  = GPIO_NOPULL;
  //   gpio_init_structure_led.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  //   HAL_GPIO_Init(GPIOD, &gpio_init_structure_led);
  //   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

    // Init for PC7 for RIGHT enc
    GPIO_InitTypeDef  gpio_init_structure_pc7;
    gpio_init_structure_pc7.Pin   = GPIO_PIN_7;
    gpio_init_structure_pc7.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure_pc7.Pull  = GPIO_NOPULL;
    gpio_init_structure_pc7.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio_init_structure_pc7);

    // Init for PA3 for LEFT enc
    GPIO_InitTypeDef  gpio_init_structure_pa3;
    gpio_init_structure_pa3.Pin   = GPIO_PIN_3;
    gpio_init_structure_pa3.Mode  = GPIO_MODE_INPUT;
    gpio_init_structure_pa3.Pull  = GPIO_NOPULL;
    gpio_init_structure_pa3.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init_structure_pa3);

    ////////////////////////////////////////////////////
    ///// Setting PWM pins PA0 and PA1 for LEFT DC /////
    ////////////////////////////////////////////////////
    GPIO_InitTypeDef GPIO_InitStruct_PWM0L, GPIO_InitStruct_PWM1L;
    TIM_OC_InitTypeDef sConfigOC0L, sConfigOC1L;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 8399; // Предделитель для частоты 1 кГц при тактовой частоте 84 МГц
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 99;    // Период для частоты 1 кГц
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
      Error_Handler();
    }

    // Init PA0 (TIM2_CH1)
    GPIO_InitStruct_PWM0L.Pin = GPIO_PIN_0;
    GPIO_InitStruct_PWM0L.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct_PWM0L.Pull = GPIO_NOPULL;
    GPIO_InitStruct_PWM0L.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct_PWM0L.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_PWM0L);

    // Init PA1 (TIM2_CH2)
    GPIO_InitStruct_PWM1L.Pin = GPIO_PIN_1;
    GPIO_InitStruct_PWM1L.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct_PWM1L.Pull = GPIO_NOPULL;
    GPIO_InitStruct_PWM1L.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct_PWM1L.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_PWM1L);

    // Setting TIM2_CH1 (PA0)
    sConfigOC0L.OCMode = TIM_OCMODE_PWM1;
    sConfigOC0L.Pulse = 0; // 50% duty cycle
    sConfigOC0L.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC0L.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC0L, TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
    }
    // Setting TIM2_CH2 (PA1)
    sConfigOC1L.OCMode = TIM_OCMODE_PWM1;
    sConfigOC1L.Pulse = 0; // 50% duty cycle
    sConfigOC1L.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC1L.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC1L, TIM_CHANNEL_2) != HAL_OK) {
      Error_Handler();
    }
    
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
    {
      Error_Handler();
    }

    /////////////////////////////////////////////////////
    ///// Setting PWM pins PB4 and PB5 for RIGHT DC /////
    /////////////////////////////////////////////////////
    GPIO_InitTypeDef GPIO_InitStruct_PWM0R, GPIO_InitStruct_PWM1R;
    TIM_OC_InitTypeDef sConfigOC0R, sConfigOC1R;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 8399; // Предделитель для частоты 1 кГц при тактовой частоте 84 МГц
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 99;    // Период для частоты 1 кГц
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    
    // Init PB4 (TIM3_CH1)
    GPIO_InitStruct_PWM0R.Pin = GPIO_PIN_4;
    GPIO_InitStruct_PWM0R.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct_PWM0R.Pull = GPIO_NOPULL;
    GPIO_InitStruct_PWM0R.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct_PWM0R.Alternate = GPIO_AF2_TIM3; // Альтернативная функция TIM3 для PB4
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_PWM0R);
    
    // Init PB5 (TIM3_CH2)
    GPIO_InitStruct_PWM1R.Pin = GPIO_PIN_5;
    GPIO_InitStruct_PWM1R.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct_PWM1R.Pull = GPIO_NOPULL;
    GPIO_InitStruct_PWM1R.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct_PWM1R.Alternate = GPIO_AF2_TIM3; // Альтернативная функция TIM3 для PB5
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_PWM1R);
    
    // Setting TIM3_CH1 (PB4)
    sConfigOC0R.OCMode = TIM_OCMODE_PWM1;
    sConfigOC0R.Pulse = 0;
    sConfigOC0R.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC0R.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC0R, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    
    // Setting TIM3_CH2 (PB5)
    sConfigOC1R.OCMode = TIM_OCMODE_PWM1;
    sConfigOC1R.Pulse = 0;
    sConfigOC1R.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC1R.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC1R, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    
    // Start PWM on TIM3_CH1
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    
    // Start PWM on TIM3_CH2
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
}


void set_pwm_lf(uint8_t pwm) {
    uint32_t PULSE_VALUE = (uint32_t)(htim2.Init.Period*pwm/256);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PULSE_VALUE);
}


void set_pwm_lb(uint8_t pwm) {
    uint32_t PULSE_VALUE = (uint32_t)(htim2.Init.Period*pwm/256);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PULSE_VALUE);
}


void set_pwm_rf(uint8_t pwm) {
    uint32_t PULSE_VALUE = (uint32_t)(htim3.Init.Period*pwm/256);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PULSE_VALUE);
}


void set_pwm_rb(uint8_t pwm) {
    uint32_t PULSE_VALUE = (uint32_t)(htim3.Init.Period*pwm/256);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PULSE_VALUE);
}
