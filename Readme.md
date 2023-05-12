Almurut I
=========

![Almurut I in its Emulator](https://i.imgur.com/Dvi3jq5.png)

The aim of this project is to create an emulator of an imaginary 8-bit CPU designed for educational purposes. It is inspired by elements of the x86 and ARM architectures but with a much simpler instruction set. It is called Almurut I. It has a few small registers, can only access 4 KB of memory, and has a minimal instruction set. The emulator is written in C and is incomplete. You have to finish it by implementing the missing instructions.

## Tasks

1. Upload/clone the source files from this repository to our course server `auca.space`.
2. Compile the assembler and the incomplete emulator of the CPU with the `make` command.
3. Translate the assembly of the `pong.s` program into machine code with the `./asa pong.s pong` command.
4. Run the code with the incomplete emulator with the `./asc pong` command. The pong program should not function properly in the incomplete emulator.
5. Run the code with the emulator given to you by the instructor with the `./asc-from-inst pong` command. The pong program should work correctly in the complete emulator showing the ball jumping from one side of the screen to another.
6. Familiarize yourself with the debug output of the emulator.
7. Open the sources of the incomplete emulator in `asc.h` that you must finish. Study the sources.
8. Find the `TODO` comments in `asc.h`, follow them, and make the emulator work correctly by implementing the missing instructions.
9. Test your emulator with the `pong` program. It should now run the program correctly.

## Recommendations

1. Some comments in the source files of the emulator are there to help you understand how the instructions should work. Read them carefully.
2. Use the `pong` program as a test program for your emulator. Single step the program in the complete emulator from the instructor to better understand how a particular instruction should work in your version of the emulator.
3. To complete this project, use the knowledge you have gained about x86 and ARM from the previous lectures and labs. The instructions of x86 and ARM CPUs inspired most instructions in this CPU. Therefore, books and reference documents about x86 and ARM, such as 'Intel x86 JUMP Quick Reference' could be incredibly helpful.

## What to Submit

1. In your private course repository given to you by the instructor during the lecture, create the path `project/`.
2. Put the `asc.h` file into that directory.
3. Commit and push your repository through Git. Submit the last commit URL to Canvas before the deadline.

## Deadline

Check Canvas for information about the deadlines.

## Documentation

    man gcc
    man gdb
    man make

## Links

### C

* [Beej's Guide to C Programming](https://beej.us/guide/bgc)

### x86 ISA

* [X86 Instruction Reference](http://www.felixcloutier.com/x86)
* [Intel x86 JUMP Quick Reference](http://www.unixwiz.net/techtips/x86-jumps.html)

### ARM64 ISA

* [Arm Architecture Reference Manual Armv8, for Armv8-A architecture profile](https://developer.arm.com/documentation/ddi0487/latest)
* [Arm Instruction Set Reference Guide](https://developer.arm.com/documentation/100076/0100/a64-instruction-set-reference)

### Tools

* [GDB Quick Reference](https://users.ece.utexas.edu/~adnan/gdb-refcard.pdf)
* [Pro Git](https://git-scm.com/book/en/v2)

## Books

### C

* C Programming Language, 2nd Edition by Brian W. Kernighan and Dennis M. Ritchie
* C Programming: A Modern Approach, 2nd Edition by K. N. King

### Assembly

* Assembly Language for x86 Processors, 7th Edition by Kip R. Irvine
* ARM 64-Bit Assembly Language by Larry D. Pyeatt and William Ughetta
