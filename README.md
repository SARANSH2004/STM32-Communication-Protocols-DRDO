# STM32 Communication Protocols – DRDO Training Project

## 📌 Overview
This repository contains a bare-metal STM32F446 embedded systems project developed during DRDO-certified training. The project implements register-level drivers for GPIO, SPI, I2C, USART, and RCC without using HAL libraries.

## 🛠️ Technologies
- STM32F446 Microcontroller
- Embedded C
- UART, SPI, I2C
- GPIO, EXTI, NVIC
- Register-level programming (No HAL)

## 📂 Project Structure
- MCU: Custom STM32 register definitions
- Drivers: Peripheral drivers (GPIO, SPI, I2C, USART, RCC)
- Applications: Standalone test programs validating each driver

## 🔌 Key Features
- Register-level peripheral control
- Interrupt-driven GPIO handling
- SPI command-based communication
- UART transmission testing
- I2C master implementation

## 🎯 Purpose
Designed to understand low-level microcontroller architecture, memory-mapped registers, interrupt handling, and real-time embedded communication used in defense-grade systems.

## 🚀 Future Scope
- DMA integration
- RTOS support
- Protocol optimization
