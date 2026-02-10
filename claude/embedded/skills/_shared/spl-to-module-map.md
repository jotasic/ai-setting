# SPL Function → Module Mapping

## RCC (Reset and Clock Control)
| SPL Source | Functions | When Needed |
|-----------|-----------|-------------|
| `stm32f10x_rcc.c` | `RCC_*PeriphClockCmd`, `RCC_*Config` | Always (clock setup) |

## GPIO
| SPL Source | Functions | When Needed |
|-----------|-----------|-------------|
| `stm32f10x_gpio.c` | `GPIO_Init`, `GPIO_SetBits`, `GPIO_ResetBits`, `GPIO_ReadInputDataBit`, `GPIO_PinRemapConfig` | Always (pin config) |

## USART
| SPL Source | Functions | When Needed |
|-----------|-----------|-------------|
| `stm32f10x_usart.c` | `USART_Init`, `USART_Cmd`, `USART_SendData`, `USART_ReceiveData`, `USART_ITConfig`, `USART_GetFlagStatus` | UART communication |

## SPI
| SPL Source | Functions | When Needed |
|-----------|-----------|-------------|
| `stm32f10x_spi.c` | `SPI_Init`, `SPI_Cmd`, `SPI_I2S_SendData`, `SPI_I2S_ReceiveData`, `SPI_GetFlagStatus` | SPI peripherals |

## I2C
| SPL Source | Functions | When Needed |
|-----------|-----------|-------------|
| `stm32f10x_i2c.c` | `I2C_Init`, `I2C_Cmd`, `I2C_Send7bitAddress`, `I2C_SendData`, `I2C_ReceiveData`, `I2C_CheckEvent` | I2C devices |

## TIM (Timers)
| SPL Source | Functions | When Needed |
|-----------|-----------|-------------|
| `stm32f10x_tim.c` | `TIM_TimeBaseInit`, `TIM_Cmd`, `TIM_ITConfig`, `TIM_OC*Init`, `TIM_SetCompare*`, `TIM_GetITStatus`, `TIM_ClearITPendingBit` | Timers, PWM |

## ADC
| SPL Source | Functions | When Needed |
|-----------|-----------|-------------|
| `stm32f10x_adc.c` | `ADC_Init`, `ADC_Cmd`, `ADC_RegularChannelConfig`, `ADC_SoftwareStartConvCmd`, `ADC_GetConversionValue` | Analog inputs |

## DMA
| SPL Source | Functions | When Needed |
|-----------|-----------|-------------|
| `stm32f10x_dma.c` | `DMA_Init`, `DMA_Cmd`, `DMA_SetCurrDataCounter`, `DMA_GetFlagStatus`, `DMA_ITConfig` | DMA transfers |

## EXTI (External Interrupts)
| SPL Source | Functions | When Needed |
|-----------|-----------|-------------|
| `stm32f10x_exti.c` | `EXTI_Init`, `EXTI_GetITStatus`, `EXTI_ClearITPendingBit` | External pin interrupts |

## FLASH
| SPL Source | Functions | When Needed |
|-----------|-----------|-------------|
| `stm32f10x_flash.c` | `FLASH_Unlock`, `FLASH_ErasePage`, `FLASH_ProgramHalfWord`, `FLASH_SetLatency` | Flash R/W, clock config |

## NVIC & Misc (CMSIS / misc.c)
| SPL Source | Functions | When Needed |
|-----------|-----------|-------------|
| `misc.c` | `NVIC_Init`, `NVIC_PriorityGroupConfig`, `NVIC_SetVectorTable`, `SysTick_CLKSourceConfig` | Interrupts |

## Rarely Needed (Remove if Unused)
| SPL Source | Description |
|-----------|-------------|
| `stm32f10x_bkp.c` | Backup registers |
| `stm32f10x_can.c` | CAN bus |
| `stm32f10x_cec.c` | Consumer Electronics Control |
| `stm32f10x_crc.c` | CRC calculation |
| `stm32f10x_dac.c` | DAC output |
| `stm32f10x_dbgmcu.c` | Debug MCU config |
| `stm32f10x_fsmc.c` | Flexible Static Memory Controller |
| `stm32f10x_iwdg.c` | Independent Watchdog |
| `stm32f10x_pwr.c` | Power control |
| `stm32f10x_rtc.c` | Real-Time Clock |
| `stm32f10x_sdio.c` | SD/MMC interface |
| `stm32f10x_wwdg.c` | Window Watchdog |
