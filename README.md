# XY-plotter

This is a LPC1549-based firmware to control Makeblock XY-plotter with laser engraver update.


## Requirement
- Hardware
  - [Makeblock XY-plotter](https://www.makeblock.com/project/xy-plotter-robot-kit)
  - [LPC1549 LPCXpresso Board](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/general-purpose-mcus/lpc1500-cortex-m3/lpcxpresso-board-for-lpc1549:OM13056)
  
- Sofware
  - [MCUXpresso IDE](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE) to build the source code
  - [mDraw](https://github.com/Makeblock-official/mDrawBot) is used to control plotter
  
- The plotter ships with an Arduino compatible controller board which may be used as a reference when we evaluate the outcome of the project.
- The plotter consists of frame, two stepper motors and their driver chips, a servo to control pencil movement and a PWM driven laser engraver.
- The firmware receives and interprets G-codes and driver the mechanics accordingly. G-codes are fairly simple instructions that tell for example the plotter to move to a certain point or to draw an arc on the canvas. Some G codesread/write settings of the device.
- [Documentation](https://github.com/arsiarola/XY-plotter/blob/master/XY-plotter/docs/XY-plotter_documentation.pdf)
