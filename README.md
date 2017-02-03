# SSD1306-GameOfLife
Game Of Life As Screensaver

Two color OLED display SSD1306 (128x64) presents Conway's Game Of Life on lower 6 rows. On top 2 rows is drawn RTC clock value. Pseudo random generator is used for initial seeding of the game board. It is based on the internal temperature sensor of STM32F103C6. All these is managed by FreeRTOS 9.0. All buffers are statically allocated.

IDE:      EmBitz 1.11
Compiler: GCC ARM

DATE: 03.Feb.2017
