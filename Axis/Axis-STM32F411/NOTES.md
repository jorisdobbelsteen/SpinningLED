Software for Axis Controller board with STM32F411.

<!-- TOC -->
* [Mapping of SPI interfaces](#mapping-of-spi-interfaces)
* [Connection of Timers](#connection-of-timers)
  * [Rotation](#rotation)
* [TODO](#todo)
<!-- TOC -->

# Mapping of SPI interfaces #

| Board | STM32 |
|-------|-------|
| LS#1  | SPI4  |
| LS#2  | SPI1  |
| LS#2  | SPI2  |
| LS#2  | SPI3  |

# Connection of Timers #

The timer channels are connected together in a very specific pattern.

| Timer | Channel | Direction | Pin | Description                                                      |
|-------|---------|-----------|-----|------------------------------------------------------------------|
| TIM1  | 1       | Input     | PA8 | Rotation Detection.<br/>Pull up when slight is detected.         |
| TIM1  | 2       | Output    | PA9 | Rotation LED (yellow)                                            |
| TIM2  | -       | -         | -   | Pixel update trigger                                             |
| TIM3  | 4       | Output    | PC9 | Error LED (red). To generate error pattern. Each pulse is 250ms. |

## Rotation - TIM1 ##

TIM1 is to acquire rotation speed, in addition to blinking the rotation LED.

There are several situations for the timer to generate an event:
* Timer overflow --> No or insufficient rotation
* Timer input capture
  * If no timer overflow happen since previous capture, then we acquired a rotation correctly.
  * If timer overflow did happen, then rotation was too slow

Upon acquiring rotation, the application should:
* Reset pixel generation timer to zero to align. This can be done with hardware.
* Program period of the pixels based on observed rotation.

When a timeout is reached:
* Rotation LEDs should be turned off or put in an error pattern.

The timeout period depends on the clock frequency and the number of pixels. The timeout period for rotation is calculated as `LEDS_X * 65536 / TIM_CLK_FREQ` seconds.

| TIM_CLK_FREQ | LEDS_X | Maximum period (ms) | Minimum rotations / sec |
|--------------|--------|---------------------|-------------------------|
| 80 MHz       | 400    | 327                 | 3.0                     |
| 80 MHz       | 500    | 409                 | 2.4                     |
| 80 MHz       | 600    | 491                 | 2.0                     |
| 100 MHz      | 400    | 262                 | 3.8                     |
| 100 MHz      | 500    | 327                 | 3.0                     |
| 50 MHz       | 400    | 524                 | 1.9                     |
| 50 MHz       | 500    | 655                 | 1.5                     |

To generate a pulse of X ms for the rotation led, use the following formula:
X * TIM_CLK_FREQ / LEDS_X / 1000 ticks.

## DMA trigger Timer -TIM2 ##

TIM2 is used to provide triggers when the pixel update should be transmitted. This happens on every overflow of the timer.

## Error LED timer - TIM3 ##

The error LED generated a blinking pattern on the LED.
This is achieved with PWM output together with circular DMA transfer each period.

## STM32F411 Master-Slave ##

| Slave TIM | ITR0      | ITR1      | ITR2      | ITR3      |
|-----------|-----------|-----------|-----------|-----------|
| TIM1      | TIM5_TRGO | TIM2_TRGO | TIM3_TRGO | TIM4_TRGO |
| TIM2      | TIM1_TRGO | -         | TIM5_TRGO | TIM4_TRGO |
| TIM3      | TIM1_TRGO | TIM2_TRGO | TIM3_TRGO | TIM4_TRGO |
| TIM4      | TIM1_TRGO | TIM2_TRGO | TIM5_TRGO | -         |
| TIM5      | TIM2_TRGO | TIM3_TRGO | TIM4_TRGO | -         |
| TIM9      | TIM2_TRGO | TIM3_TRGO | TIM10_OC  | TIM11_OC  |


# TODO #
* Have RGB565 at all. Use SIMD for RGB565 to RGB888 conversion.
* HSV algorithm with SIMD.
* LED DMA: Use 16-bit or 32-bit transfers to reduce bus load.