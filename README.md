# stm32-sin-generator
STM32 project for produce output repeating voltage shape, generally - sin wave

Microcontroller - stm32f103, on a default plue pill board.
Initialisation made using CubeMx. Pure C project, Keil uVision IDE.

MC produces voltage level using PWM. ADC with DMA is used for the feedback signal acquisition.
For filtration of the ADC siglnal implemented universal discrete filter.
MC is connected to PC via virtual COM-port (USB). Over that protocol ordets from PC and telemetry from MC are transmitted.

![alt text](https://github.com/Nonmant/stm32-sin-generator-and-measuer/blob/master/generated%20signal.jpg?raw=true)
