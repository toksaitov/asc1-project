#ifndef ASC_H
#define ASC_H

#include <assert.h>
#include <ctype.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

/* Device */

#define CPU_INST      39          // number of CPU instructions
#define CPU_INST_SIZE 3           // fixed size of every instruction in bytes
#define CPU_REGS      17          // number of CPU registers
#define CPU_ISRS      16          // number of interrupt service routines
#define CPU_EXCPT_INVALID_INSTR 0 // exception code for an invalid instruction

#define MEM_SIZE     0x1000       // amount of main addressable system memory
#define STACK_SIZE   0x80         // default hardware stack size
#define IVT_SIZE     0x20         // size of the interrupt vector table (8 exception entries + 8 interrupt entries, each entry is 2 bytes in size (code segment and offset))
#define SECTION_STEP 0x10         // byte difference between two consequent sections (currently equals to 1 << 4)

#define PIC_CMD_IO_PORT  0x20     // interrupt controller's command IO port to configure it
#define PIC_DATA_IO_PORT 0x21     // interrupt controller's data IO port to query its state or configure it

#define PIT_IRQ          8        // interrupt vector to use to trigger timer's interrupt requests in PIC
#define PIT_CMD_IO_PORT  0x43     // timer's command IO port to configure it
#define PIT_DATA_IO_PORT 0x40     // timer's data IO port to query its state or configure it

#define KBDC_IRQ           1      // interrupt vector to use to trigger keyboard controller's interrupt requests in PIC
#define KBDC_DATA_IO_PORT  0x60   // keyboard controller's data port to query the currently pressed key by reading the scancode register

#define DISPC_SCR_WIDTH    128    // horizontal resolution in pixels of a screen connected to the display controller
#define DISPC_SCR_HEIGHT   64     // vertical resolution in pixels of a screen connected to the display controller
#define DISPC_BUFF_SIZE    0x400  // size of the display controller's screen buffer (height in pixels times width in pixels / 8 as every pixel is 1-bit in depth)
#define DISPC_CMD_IO_PORT  0xD4   // display controller's command IO port to configure it
#define DISPC_DATA_IO_PORT 0xDA   // display controller's data port to query its state by reading its status register
#define DISPC_CYCLES_PER_FRAME     1667 // 16.67 ms/frame * 100 cycles/ms of our CPU
#define DISPC_CYCLES_BEFORE_VBLANK 1524 // 45 invisible VBI lines / 525 total lines * 16.67 ms/frame * 100 cycles/ms of our CPU

#define MAX_FIRMWARE_SIZE (MEM_SIZE - DISPC_BUFF_SIZE * 2 - STACK_SIZE - IVT_SIZE) // Maximum size of the firmware

typedef struct cpu
{
    /* General-Purpose Registers */

    uint8_t r0; // 1'st arg. to a function, 1'st return val. from a function, caller-saved
    uint8_t r1; // 2'nd arg. to a function, 2'nd return val. from a function, caller-saved
    uint8_t r2; // 3'rd arg. to a function,                                   caller-saved
    uint8_t r3; // 4'th arg. to a function,                                   caller-saved
    uint8_t r4; // 5'th arg. to a function,                                   caller-saved
    uint8_t r5; // 6'th arg. to a function,                                   caller-saved
    uint8_t r6; //                                                            caller-saved
    uint8_t r7; //                                                            callee-saved

    /* Segment Registers */

    uint8_t cs; // code segment  ([`cs` reg. << 4 + offset in `ip`/`lr` reg.] form an effective address to find a byte in memory to access program's code)
    uint8_t ds; // data segment  ([`ds` reg. << 4 + offset in `r*` reg.] form an effective address to find a byte in memory to access program's data)
    uint8_t ss; // stack segment ([`ss` reg. << 4 + offset in `fp`/`sp` reg.] form an effective address to find a byte in memory to access program's stack)

    /* Special-Purpose Registers */

    uint8_t fp; // frame pointer that points to the base of the stack of a function frame
    uint8_t sp; // stack pointer that points to the top of the stack
    uint8_t ip; // instruction pointer that points to the next instruction to execute
    uint8_t ls; // link segment register with code segment of a function to return to
    uint8_t lr; // link address register with return address from a function

    /*
        Flags' Register Bits:
            XXXXXXXX
        Flags:
            RRRIOSZC

            C (Carry flag): Set on unsigned carry or borrow; cleared otherwise
            Z (Zero flags): Set if result is zero; cleared otherwise
            S (Sign flag): Set if result is a positive number; cleared otherwise
            O (Overflow flag): Set on signed overflow; cleared otherwise
            I (Interrupt flag): Set when maskable interrupts are enabled; cleared otherwise
            R (Reserved): Reserved for future use
    */
    uint8_t flags;

    /* Utility Fields (not part of a real device) */

    size_t cycles; // Number of cycles since the start of the emulation
    uint8_t *regs[CPU_REGS]; // array of pointers to the CPU's registers to simplify their access in the emulator
} cpu_t;

typedef struct mem
{
    uint8_t bytes[MEM_SIZE]; // RAM bytes with indices representing addresses from low to high.
} mem_t;

typedef enum pic_state
{
    PIC_AWAITS_IMR,
    PIC_AWAITS_VEC_OFF
} pic_state_t;

typedef struct pic
{
    uint8_t imr; // mask that controls which interrupt request lines are disabled (1) and which are enabled (0)
    uint8_t vec_off; // offset of the interrupt vector to use to trigger interrupt requests

    /* Utility Fields (may not be a part of a real device) */

    pic_state_t state; // IO state of the PIC
} pic_t;

typedef enum pit_state
{
    PIT_AWAITS_FREQ_DIV_HIGH,
    PIT_AWAITS_FREQ_DIV_LOW
} pit_state_t;

typedef struct pit
{
    uint8_t freq_div_high; // 16-bit timer's frequency divisor that controls how often the timer should trigger an interrupt
    uint8_t freq_div_low;
    uint8_t cnt_high;      // 16-bit timer's current counter value
    uint8_t cnt_low;

    /* Utility Fields (may not be a part of a real device) */

    pit_state_t state; // IO state of the PIT
} pit_t;

typedef struct kbdc
{
    uint8_t scancode; // unique identifier of the last keyboard button pressed
} kbdc_t;

typedef enum dispc_state
{
    DISPC_AWAITS_MMIO_DS,
    DISPC_AWAITS_MMIO_ADDR
} dispc_state_t;

typedef struct dispc
{
    /* Memory-mapped I/O data segment and address of the screen buffer to trace (from the top-left corner + `DISPC_BUFF_SIZE` bytes) */
    uint8_t mmio_ds;
    uint8_t mmio_addr;

    /*
        Status' Register Bits:
            XXXXXXXX
        Status:
            RRRRRRRV

            V (Vertical retrace): Set on vertical retrace; cleared otherwise
            R (Reserved): Reserved for future use
    */
    uint8_t status;

    /* Utility Fields (may not be a part of a real device) */

    size_t cycles; // number of cycles since the last vertical retrace
    dispc_state_t state; // IO state of the display controller
} dispc_t;

typedef struct device
{
    cpu_t   cpu;
    mem_t   mem;
    pic_t   pic;
    pit_t   pit;
    kbdc_t  kbdc;
    dispc_t dispc;
} device_t;

/* Instruction Encoding */

typedef enum cpu_mnem_code
{
    CPU_MNEM_HLT,  // halts the CPU
    CPU_MNEM_LR,   // loads a value from a memory address into a register
    CPU_MNEM_LI,   // loads an immediate value into a register
    CPU_MNEM_MOV,  // moves a value from one register to another
    CPU_MNEM_SR,   // stores a value from a register into a memory address
    CPU_MNEM_PUSH, // pushes a value from a register onto the stack
    CPU_MNEM_POP,  // pops a value from the stack into a register
    CPU_MNEM_ADD,  // adds two values and stores the result in a register updating flags
    CPU_MNEM_ADC,  // adds two values with carry and stores the result in a register updating flags
    CPU_MNEM_SUB,  // subtracts two values and stores the result in a register updating flags
    CPU_MNEM_CMP,  // subtracts two values and updates flags
    CPU_MNEM_SBB,  // subtracts two values with borrow and stores the result in a register updating flags
    CPU_MNEM_NOT,  // inverts a value and stores the result in a register updating flags
    CPU_MNEM_AND,  // bitwise ANDs two values and stores the result in a register updating flags
    CPU_MNEM_TEST, // bitwise ANDs two values and updates flags
    CPU_MNEM_OR,   // bitwise ORs two values and stores the result in a register updating flags
    CPU_MNEM_XOR,  // bitwise XORs two values and stores the result in a register updating flags
    CPU_MNEM_SHL,  // shifts a value left and stores the result in a register updating flags
    CPU_MNEM_SHR,  // shifts a value right and stores the result in a register updating flags
    CPU_MNEM_SAR,  // shifts a value right with sign extension and stores the result in a register updating flags
    CPU_MNEM_JMP,  // jumps to a memory address
    CPU_MNEM_JE,   // jumps to a memory address if the zero flag is set
    CPU_MNEM_JNE,  // jumps to a memory address if the zero flag is cleared
    CPU_MNEM_JG,   // jumps to a memory address if the zero flag is cleared and the sign flag is equal to the overflow flag
    CPU_MNEM_JGE,  // jumps to a memory address if the sign flag is equal to the overflow flag
    CPU_MNEM_JL,   // jumps to a memory address if the sign flag is not equal to the overflow flag
    CPU_MNEM_JLE,  // jumps to a memory address if the zero flag is set or the sign flag is not equal to the overflow flag
    CPU_MNEM_JA,   // jumps to a memory address if the carry flag is cleared and the zero flag is cleared
    CPU_MNEM_JAE,  // jumps to a memory address if the carry flag is cleared
    CPU_MNEM_JB,   // jumps to a memory address if the carry flag is set
    CPU_MNEM_JBE,  // jumps to a memory address if the carry flag is set or the zero flag is set
    CPU_MNEM_CALL, // calls a function at a memory address
    CPU_MNEM_RET,  // returns from a function
    CPU_MNEM_IRET, // returns from an interrupt restoring some state of the CPU before the interrupt
    CPU_MNEM_INB,  // reads a byte from a port into a register
    CPU_MNEM_OUTB, // writes a byte from a register into a port
    CPU_MNEM_CLI,  // clears the interrupt flag
    CPU_MNEM_STI,  // sets the interrupt flag
    CPU_MNEM_NOP   // does nothing
} cpu_mnem_code_t;

typedef enum cpu_reg_code
{
    CPU_REG_R0,
    CPU_REG_R1,
    CPU_REG_R2,
    CPU_REG_R3,
    CPU_REG_R4,
    CPU_REG_R5,
    CPU_REG_R6,
    CPU_REG_R7,
    CPU_REG_CS,
    CPU_REG_DS,
    CPU_REG_SS,
    CPU_REG_FP,
    CPU_REG_SP,
    CPU_REG_IP,
    CPU_REG_LS,
    CPU_REG_LR,
    CPU_REG_FLAGS
} cpu_reg_code_t;

typedef struct mnem_enc
{
    char *mnem;
    cpu_mnem_code_t code;
} mnem_enc_t;

typedef struct reg_enc
{
    char *name;
    cpu_reg_code_t code;
} reg_enc_t;

static const mnem_enc_t MNEM_ENC_TABLE[CPU_INST] = {
    { "hlt",  CPU_MNEM_HLT  },
    { "lr",   CPU_MNEM_LR   },
    { "li",   CPU_MNEM_LI   },
    { "mov",  CPU_MNEM_MOV  },
    { "sr",   CPU_MNEM_SR   },
    { "push", CPU_MNEM_PUSH },
    { "pop",  CPU_MNEM_POP  },
    { "add",  CPU_MNEM_ADD  },
    { "adc",  CPU_MNEM_ADC  },
    { "sub",  CPU_MNEM_SUB  },
    { "cmp",  CPU_MNEM_CMP  },
    { "sbb",  CPU_MNEM_SBB  },
    { "not",  CPU_MNEM_NOT  },
    { "and",  CPU_MNEM_AND  },
    { "test", CPU_MNEM_TEST },
    { "or",   CPU_MNEM_OR   },
    { "xor",  CPU_MNEM_XOR  },
    { "shl",  CPU_MNEM_SHL  },
    { "shr",  CPU_MNEM_SHR  },
    { "sar",  CPU_MNEM_SAR  },
    { "jmp",  CPU_MNEM_JMP  },
    { "je",   CPU_MNEM_JE   },
    { "jne",  CPU_MNEM_JNE  },
    { "jg",   CPU_MNEM_JG   },
    { "jge",  CPU_MNEM_JGE  },
    { "jl",   CPU_MNEM_JL   },
    { "jle",  CPU_MNEM_JLE  },
    { "ja",   CPU_MNEM_JA   },
    { "jae",  CPU_MNEM_JAE  },
    { "jb",   CPU_MNEM_JB   },
    { "jbe",  CPU_MNEM_JBE  },
    { "call", CPU_MNEM_CALL },
    { "ret",  CPU_MNEM_RET  },
    { "iret", CPU_MNEM_IRET },
    { "inb",  CPU_MNEM_INB  },
    { "outb", CPU_MNEM_OUTB },
    { "cli",  CPU_MNEM_CLI  },
    { "sti",  CPU_MNEM_STI  },
    { "nop",  CPU_MNEM_NOP  }
};

static const reg_enc_t REG_ENC_TABLE[CPU_REGS] = {
    { "r0",    CPU_REG_R0 },
    { "r1",    CPU_REG_R1 },
    { "r2",    CPU_REG_R2 },
    { "r3",    CPU_REG_R3 },
    { "r4",    CPU_REG_R4 },
    { "r5",    CPU_REG_R5 },
    { "r6",    CPU_REG_R6 },
    { "r7",    CPU_REG_R7 },
    { "cs",    CPU_REG_CS },
    { "ds",    CPU_REG_DS },
    { "ss",    CPU_REG_SS },
    { "fp",    CPU_REG_FP },
    { "sp",    CPU_REG_SP },
    { "ip",    CPU_REG_IP },
    { "ls",    CPU_REG_LS },
    { "lr",    CPU_REG_LR },
    { "flags", CPU_REG_FLAGS }
};

/* Executable Format */

#define BIN_FMT_MAGIC 0x61736266 /* "asbf" */

typedef struct __attribute__((packed)) bin_fmt_header
{
    uint32_t magic;
    uint16_t data_sec_size;
    uint16_t text_sec_size;
} bin_fmt_header_t;

typedef struct firmware
{
    uint8_t *image;
    size_t image_size;

    uint8_t *data_sec;
    size_t data_sec_size;

    uint8_t *text_sec;
    size_t text_sec_size;
} firmware_t;

/* Helpful Constants */

/* CPU's Flag Bits */

#define CPU_FLAG_C 0b00000001
#define CPU_FLAG_Z 0b00000010
#define CPU_FLAG_S 0b00000100
#define CPU_FLAG_O 0b00001000
#define CPU_FLAG_I 0b00010000

/* Display Controller's Flag Bits */

#define DISPC_FLAG_VBLANK 0b00000001

/* Initialization Functions */

static void cpu_init(cpu_t *cpu)
{
    assert(cpu != NULL);

    cpu->r0 = 0;
    cpu->r1 = 0;
    cpu->r2 = 0;
    cpu->r3 = 0;
    cpu->r4 = 0;
    cpu->r5 = 0;
    cpu->r6 = 0;
    cpu->r7 = 0;

    cpu->cs = IVT_SIZE / SECTION_STEP; // the text section starts at the beginning of the address space skipping the interrupt vector table
    cpu->ds = cpu->cs + 512 / SECTION_STEP; // on boot, the data section starts after the first 512 bytes of the text section
    cpu->ss = (MEM_SIZE - DISPC_BUFF_SIZE * 2 - STACK_SIZE) / SECTION_STEP; // stack starts below the display buffers (front and back) at the end of the address space

    cpu->fp = STACK_SIZE - 1;
    cpu->sp = cpu->fp;  // the stack grows toward low memory addresses
    cpu->ip = 0;
    cpu->ls = cpu->cs;
    cpu->lr = cpu->ip;
    cpu->flags = 0b00000010; // only Z (Zero flag) is set

    cpu->cycles = 0;
    cpu->regs[CPU_REG_R0] = &cpu->r0; // pointers to registers to simplify lookup in the emulator
    cpu->regs[CPU_REG_R1] = &cpu->r1;
    cpu->regs[CPU_REG_R2] = &cpu->r2;
    cpu->regs[CPU_REG_R3] = &cpu->r3;
    cpu->regs[CPU_REG_R4] = &cpu->r4;
    cpu->regs[CPU_REG_R5] = &cpu->r5;
    cpu->regs[CPU_REG_R6] = &cpu->r6;
    cpu->regs[CPU_REG_R7] = &cpu->r7;

    cpu->regs[CPU_REG_CS] = &cpu->cs;
    cpu->regs[CPU_REG_DS] = &cpu->ds;
    cpu->regs[CPU_REG_SS] = &cpu->ss;

    cpu->regs[CPU_REG_FP] = &cpu->fp;
    cpu->regs[CPU_REG_SP] = &cpu->sp;
    cpu->regs[CPU_REG_IP] = &cpu->ip;
    cpu->regs[CPU_REG_LS] = &cpu->ls;
    cpu->regs[CPU_REG_LR] = &cpu->lr;
    cpu->regs[CPU_REG_FLAGS] = &cpu->flags;
}

static void cpu_raise_irq(device_t *dev, uint8_t irq)
{
    assert(dev != NULL);

    if ((dev->cpu.flags & CPU_FLAG_I) && !(dev->pic.imr & (1 << irq))) { // if interrupts are enabled and the interrupt is not masked
        dev->mem.bytes[(dev->cpu.ss << 4) + --dev->cpu.sp] = dev->cpu.flags;
        dev->mem.bytes[(dev->cpu.ss << 4) + --dev->cpu.sp] = dev->cpu.cs;
        dev->mem.bytes[(dev->cpu.ss << 4) + --dev->cpu.sp] = dev->cpu.ip;

        uint8_t cs   = dev->mem.bytes[(dev->pic.vec_off + irq) * 2];
        uint8_t addr = dev->mem.bytes[(dev->pic.vec_off + irq) * 2 + 1];
        dev->cpu.cs = cs;
        dev->cpu.ip = addr;
    }
}

static void cpu_raise_excpt(device_t *dev, uint8_t excpt)
{
    assert(dev != NULL);

    /* Exceptions are not masked, so we don't need to check the IMR. */

    dev->mem.bytes[(dev->cpu.ss << 4) + --dev->cpu.sp] = dev->cpu.flags;
    dev->mem.bytes[(dev->cpu.ss << 4) + --dev->cpu.sp] = dev->cpu.cs;
    dev->mem.bytes[(dev->cpu.ss << 4) + --dev->cpu.sp] = dev->cpu.ip;

    uint8_t cs   = dev->mem.bytes[excpt * 2];
    uint8_t addr = dev->mem.bytes[excpt * 2 + 1];
    dev->cpu.cs = cs;
    dev->cpu.ip = addr;
}

static void mem_init(mem_t *mem)
{
    assert(mem != NULL);

    memset(mem->bytes, 0, MEM_SIZE);

    /* On boot, make the CPU loop executing the `nop` (no operation) instruction. */

    size_t cursor = IVT_SIZE;
    mem->bytes[cursor] = MNEM_ENC_TABLE[CPU_MNEM_NOP].code;
    cursor += CPU_INST_SIZE;
    mem->bytes[cursor] = MNEM_ENC_TABLE[CPU_MNEM_JMP].code;
    mem->bytes[cursor + 1] = REG_ENC_TABLE[CPU_REG_R0].code;
}

static void pic_init(pic_t *pic)
{
    assert(pic != NULL);

    pic->imr = 0b11111111; // all interrupts are masked at boot
    pic->vec_off = 8; // the first 8 vectors are reserved for CPU exceptions
}

static void pit_init(pit_t *pit)
{
    assert(pit != NULL);

    pit->freq_div_high = 0; // 0 is interpreted as 65536 in PIT
    pit->freq_div_low  = 0;
    pit->cnt_high = pit->freq_div_high;
    pit->cnt_low  = pit->freq_div_low;
}

static void kbdc_init(kbdc_t *kbdc)
{
    assert(kbdc != NULL);

    kbdc->scancode = 0;
}

static void dispc_init(dispc_t *dispc)
{
    assert(dispc != NULL);

    dispc->mmio_ds = (MEM_SIZE - DISPC_BUFF_SIZE) / SECTION_STEP; // the screen buffer is memory-mapped to the end of memory space
    dispc->mmio_addr = 0;

    dispc->status = 0b00000000; // not in VBLANK at boot
    dispc->cycles = 0;
}

static void dev_init(device_t *dev, firmware_t *firmware)
{
    assert(dev != NULL);

    assert(firmware != NULL);
    assert(firmware->image != NULL);
    assert(firmware->data_sec != NULL);
    assert(firmware->text_sec != NULL);

    assert(firmware->image_size <= MAX_FIRMWARE_SIZE);
    assert(firmware->image_size == firmware->data_sec_size + firmware->text_sec_size);

    cpu_init(&dev->cpu);
    mem_init(&dev->mem);
    pic_init(&dev->pic);
    pit_init(&dev->pit);
    kbdc_init(&dev->kbdc);
    dispc_init(&dev->dispc);

    memcpy(dev->mem.bytes + IVT_SIZE, firmware->text_sec, firmware->text_sec_size);
    memcpy(dev->mem.bytes + IVT_SIZE + firmware->text_sec_size, firmware->data_sec, firmware->data_sec_size);
    dev->cpu.ds = (IVT_SIZE + firmware->text_sec_size) / SECTION_STEP;
}

/* CPU Instructions */

/* Load and Store Instructions */

static void cpu_hlt(device_t *dev)
{
    assert(dev != NULL);

    /*
        Stops the CPU from progressing by not changing the instruction pointer.
        It is important to ensure that this instruction has a code 0x00 to
        prevent uninitialized code doing harm.
    */
}

static void cpu_lr(device_t *dev, uint8_t reg_with_addr_of_src, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_src >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_li(device_t *dev, uint8_t imm, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_mov(device_t *dev, uint8_t reg_src, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_sr(device_t *dev, uint8_t reg_src, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_push(device_t *dev, uint8_t reg_src)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_pop(device_t *dev, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

/* Arithmetic Instructions */

static void cpu_add(device_t *dev, uint8_t reg_src, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_adc(device_t *dev, uint8_t reg_src, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_sub(device_t *dev, uint8_t reg_src, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_cmp(device_t *dev, uint8_t reg_src, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_sbb(device_t *dev, uint8_t reg_src, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        uint16_t carry = dev->cpu.flags & 0b00000001;
        int res = (int) *dev->cpu.regs[reg_dst] - (int) *dev->cpu.regs[reg_src] - carry;

        if (res & 0b100000000) {
            dev->cpu.flags |= CPU_FLAG_C;
        } else {
            dev->cpu.flags &= ~CPU_FLAG_C;
        }
        if (!res) {
            dev->cpu.flags |= CPU_FLAG_Z;
        } else {
            dev->cpu.flags &= ~CPU_FLAG_Z;
        }
        if (res & 0b10000000) {
            dev->cpu.flags |= CPU_FLAG_S;
        } else {
            dev->cpu.flags &= ~CPU_FLAG_S;
        }
        if ((*dev->cpu.regs[reg_dst] & 0b10000000) == (*dev->cpu.regs[reg_src] & 0b10000000) &&
            (*dev->cpu.regs[reg_dst] & 0b10000000) != (res & 0b10000000)) {
            dev->cpu.flags |= CPU_FLAG_O;
        } else {
            dev->cpu.flags &= ~CPU_FLAG_O;
        }
        *dev->cpu.regs[reg_dst] = *dev->cpu.regs[reg_dst] - *dev->cpu.regs[reg_src] - carry;

        dev->cpu.ip += CPU_INST_SIZE;
    }
}

/* Logical Instructions */

static void cpu_not(device_t *dev, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_and(device_t *dev, uint8_t reg_src, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_test(device_t *dev, uint8_t reg_src, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        uint8_t res = *dev->cpu.regs[reg_dst] & *dev->cpu.regs[reg_src];

        dev->cpu.flags &= ~CPU_FLAG_C;
        if (!res) {
            dev->cpu.flags |= CPU_FLAG_Z;
        } else {
            dev->cpu.flags &= ~CPU_FLAG_Z;
        }
        if (res & 0b10000000) {
            dev->cpu.flags |= CPU_FLAG_S;
        } else {
            dev->cpu.flags &= ~CPU_FLAG_S;
        }
        dev->cpu.flags &= ~CPU_FLAG_O;

        dev->cpu.ip += CPU_INST_SIZE;
    }
}

static void cpu_or(device_t *dev, uint8_t reg_src, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_xor(device_t *dev, uint8_t reg_src, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        uint8_t res = *dev->cpu.regs[reg_dst] ^ *dev->cpu.regs[reg_src];

        dev->cpu.flags &= ~CPU_FLAG_C;
        if (!res) {
            dev->cpu.flags |= CPU_FLAG_Z;
        } else {
            dev->cpu.flags &= ~CPU_FLAG_Z;
        }
        if (res & 0b10000000) {
            dev->cpu.flags |= CPU_FLAG_S;
        } else {
            dev->cpu.flags &= ~CPU_FLAG_S;
        }
        dev->cpu.flags &= ~CPU_FLAG_O;

        *dev->cpu.regs[reg_dst] = res;

        dev->cpu.ip += CPU_INST_SIZE;
    }
}

/* Shift Instructions */

static void cpu_shl(device_t *dev, uint8_t shift_in_reg, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (shift_in_reg >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_shr(device_t *dev, uint8_t shift_in_reg, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (shift_in_reg >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        if (*dev->cpu.regs[shift_in_reg] != 0) {
            uint8_t res = *dev->cpu.regs[reg_dst] >> *dev->cpu.regs[shift_in_reg];

            uint8_t carry = (*dev->cpu.regs[reg_dst] >> (*dev->cpu.regs[shift_in_reg] - 1)) & 1;
            if (carry) {
                dev->cpu.flags |= CPU_FLAG_C;
            } else {
                dev->cpu.flags &= ~CPU_FLAG_C;
            }
            if (!res) {
                dev->cpu.flags |= CPU_FLAG_Z;
            } else {
                dev->cpu.flags &= ~CPU_FLAG_Z;
            }
            if (res & 0b10000000) {
                dev->cpu.flags |= CPU_FLAG_S;
            } else {
                dev->cpu.flags &= ~CPU_FLAG_S;
            }
            if (*dev->cpu.regs[reg_dst] >> 7) {
                dev->cpu.flags |= CPU_FLAG_O;
            } else {
                dev->cpu.flags &= ~CPU_FLAG_O;
            }

            *dev->cpu.regs[reg_dst] = res;
        }

        dev->cpu.ip += CPU_INST_SIZE;
    }
}

static void cpu_sar(device_t *dev, uint8_t shift_in_reg, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (shift_in_reg >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        if (*dev->cpu.regs[shift_in_reg] != 0) {
            int8_t res = (int8_t) ((int) *dev->cpu.regs[reg_dst] >> (int) *dev->cpu.regs[shift_in_reg]);

            uint8_t carry = (*dev->cpu.regs[reg_dst] >> (*dev->cpu.regs[shift_in_reg] - 1)) & 1;
            if (carry) {
                dev->cpu.flags |= CPU_FLAG_C;
            } else {
                dev->cpu.flags &= ~CPU_FLAG_C;
            }
            if (!res) {
                dev->cpu.flags |= CPU_FLAG_Z;
            } else {
                dev->cpu.flags &= ~CPU_FLAG_Z;
            }
            if (res & 0b10000000) {
                dev->cpu.flags |= CPU_FLAG_S;
            } else {
                dev->cpu.flags &= ~CPU_FLAG_S;
            }
            dev->cpu.flags &= ~CPU_FLAG_O;

            *dev->cpu.regs[reg_dst] = res;
        }

        dev->cpu.ip += CPU_INST_SIZE;
    }
}

/* Branch Instructions */

/* Unconditional Branch Instructions */

static void cpu_jmp(device_t *dev, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

/* Conditional Branch Instructions */

static void cpu_je(device_t *dev, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        if (dev->cpu.flags & CPU_FLAG_Z) {
            dev->cpu.ip = *dev->cpu.regs[reg_with_addr_of_dst];
        } else {
            dev->cpu.ip += CPU_INST_SIZE;
        }
    }
}

static void cpu_jne(device_t *dev, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        if (!(dev->cpu.flags & CPU_FLAG_Z)) {
            dev->cpu.ip = *dev->cpu.regs[reg_with_addr_of_dst];
        } else {
            dev->cpu.ip += CPU_INST_SIZE;
        }
    }
}

static void cpu_jg(device_t *dev, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        if (!(dev->cpu.flags & CPU_FLAG_Z) && !!(dev->cpu.flags & CPU_FLAG_S) == !!(dev->cpu.flags & CPU_FLAG_O)) {
            dev->cpu.ip = *dev->cpu.regs[reg_with_addr_of_dst];
        } else {
            dev->cpu.ip += CPU_INST_SIZE;
        }
    }
}

static void cpu_jge(device_t *dev, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_jl(device_t *dev, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_jle(device_t *dev, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        if ((dev->cpu.flags & CPU_FLAG_Z) || !!(dev->cpu.flags & CPU_FLAG_S) != !!(dev->cpu.flags & CPU_FLAG_O)) {
            dev->cpu.ip = *dev->cpu.regs[reg_with_addr_of_dst];
        } else {
            dev->cpu.ip += CPU_INST_SIZE;
        }
    }
}

static void cpu_ja(device_t *dev, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_jae(device_t *dev, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_jb(device_t *dev, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        // TODO: implement this instruction
    }
}

static void cpu_jbe(device_t *dev, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        if ((dev->cpu.flags & CPU_FLAG_C) || (dev->cpu.flags & CPU_FLAG_Z)) {
            dev->cpu.ip = *dev->cpu.regs[reg_with_addr_of_dst];
        } else {
            dev->cpu.ip += CPU_INST_SIZE;
        }
    }
}

/* Subroutine Control Instructions */

static void cpu_call(device_t *dev, uint8_t reg_with_cs_of_dst, uint8_t reg_with_addr_of_dst)
{
    assert(dev != NULL);

    if (reg_with_cs_of_dst >= CPU_REGS || reg_with_addr_of_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        dev->cpu.ls = dev->cpu.cs;
        dev->cpu.lr = dev->cpu.ip + CPU_INST_SIZE;

        dev->cpu.cs = *dev->cpu.regs[reg_with_cs_of_dst];
        dev->cpu.ip = *dev->cpu.regs[reg_with_addr_of_dst];
    }
}

static void cpu_ret(device_t *dev)
{
    assert(dev != NULL);

    // TODO: implement this instruction
}

static void cpu_iret(device_t *dev)
{
    assert(dev != NULL);

    dev->cpu.ip = dev->mem.bytes[(dev->cpu.ss << 4) + dev->cpu.sp++];
    dev->cpu.cs = dev->mem.bytes[(dev->cpu.ss << 4) + dev->cpu.sp++];
    dev->cpu.flags = dev->mem.bytes[(dev->cpu.ss << 4) + dev->cpu.sp++];
}

/* Input and Output Instructions */

static void cpu_inb(device_t *dev, uint8_t src_port_in_reg, uint8_t reg_dst)
{
    assert(dev != NULL);

    if (src_port_in_reg >= CPU_REGS || reg_dst >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        if (*dev->cpu.regs[src_port_in_reg] == PIC_DATA_IO_PORT) {
            *dev->cpu.regs[reg_dst] = dev->pic.imr;
        } else if (*dev->cpu.regs[src_port_in_reg] == KBDC_DATA_IO_PORT) {
            *dev->cpu.regs[reg_dst] = dev->kbdc.scancode;
        } else if (*dev->cpu.regs[src_port_in_reg] == DISPC_DATA_IO_PORT) {
            *dev->cpu.regs[reg_dst] = dev->dispc.status;
        }

        dev->cpu.ip += CPU_INST_SIZE;
    }
}

static void cpu_outb(device_t *dev, uint8_t reg_src, uint8_t dst_port_in_reg)
{
    assert(dev != NULL);

    if (reg_src >= CPU_REGS || dst_port_in_reg >= CPU_REGS) {
        cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
    } else {
        if (*dev->cpu.regs[dst_port_in_reg] == PIC_CMD_IO_PORT) {
            dev->pic.state = (pic_state_t) *dev->cpu.regs[reg_src];
        } else if (*dev->cpu.regs[dst_port_in_reg] == PIT_CMD_IO_PORT) {
            dev->pit.state = (pit_state_t) *dev->cpu.regs[reg_src];
        } else if (*dev->cpu.regs[dst_port_in_reg] == DISPC_CMD_IO_PORT) {
            dev->dispc.state = (dispc_state_t) *dev->cpu.regs[reg_src];
        }

        if (*dev->cpu.regs[dst_port_in_reg] == PIC_DATA_IO_PORT) {
            if (dev->pic.state == PIC_AWAITS_IMR) {
                dev->pic.imr = *dev->cpu.regs[reg_src];
            } else if (dev->pic.state == PIC_AWAITS_VEC_OFF) {
                dev->pic.vec_off = *dev->cpu.regs[reg_src];
            }
        } else if (*dev->cpu.regs[dst_port_in_reg] == PIT_DATA_IO_PORT) {
            if (dev->pit.state == PIT_AWAITS_FREQ_DIV_LOW) {
                dev->pit.freq_div_low = *dev->cpu.regs[reg_src];
            } else if (dev->pit.state == PIT_AWAITS_FREQ_DIV_HIGH) {
                dev->pit.freq_div_high = *dev->cpu.regs[reg_src];
                dev->pit.cnt_low  = dev->pit.freq_div_low;
                dev->pit.cnt_high = dev->pit.freq_div_high;
            }
        } else if (*dev->cpu.regs[dst_port_in_reg] == DISPC_DATA_IO_PORT) {
            if (dev->dispc.state == DISPC_AWAITS_MMIO_DS) {
                dev->dispc.mmio_ds = *dev->cpu.regs[reg_src];
            } else if (dev->dispc.state == DISPC_AWAITS_MMIO_ADDR) {
                dev->dispc.mmio_addr = *dev->cpu.regs[reg_src];
            }
        }

        dev->cpu.ip += CPU_INST_SIZE;
    }
}

/* Special Instructions */

static void cpu_cli(device_t *dev)
{
    assert(dev != NULL);

    dev->cpu.flags &= ~CPU_FLAG_I;
    dev->cpu.ip += CPU_INST_SIZE;
}

static void cpu_sti(device_t *dev)
{
    assert(dev != NULL);

    dev->cpu.flags |= CPU_FLAG_I;
    dev->cpu.ip += CPU_INST_SIZE;
}

static void cpu_nop(device_t *dev)
{
    assert(dev != NULL);

    // Do nothing

    dev->cpu.ip += CPU_INST_SIZE;
}

/* Device Emulation */

static void cpu_step(device_t *dev)
{
    assert(dev != NULL);

    ++dev->cpu.cycles;

    size_t inst_addr = (dev->cpu.cs << 4) + dev->cpu.ip;
    uint8_t inst = dev->mem.bytes[inst_addr];
    switch (inst) {
        case CPU_MNEM_HLT: {
            cpu_hlt(dev);
            break;
        }
        case CPU_MNEM_LR: {
            uint8_t reg_with_addr_of_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_lr(dev, reg_with_addr_of_src, reg_dst);
            break;
        }
        case CPU_MNEM_LI: {
            uint8_t imm = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_li(dev, imm, reg_dst);
            break;
        }
        case CPU_MNEM_MOV: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_mov(dev, reg_src, reg_dst);
            break;
        }
        case CPU_MNEM_SR: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 2];
            cpu_sr(dev, reg_src, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_PUSH: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            cpu_push(dev, reg_src);
            break;
        }
        case CPU_MNEM_POP: {
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 1];
            cpu_pop(dev, reg_dst);
            break;
        }
        case CPU_MNEM_ADD: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_add(dev, reg_src, reg_dst);
            break;
        }
        case CPU_MNEM_ADC: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_adc(dev, reg_src, reg_dst);
            break;
        }
        case CPU_MNEM_SUB: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_sub(dev, reg_src, reg_dst);
            break;
        }
        case CPU_MNEM_CMP: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_cmp(dev, reg_src, reg_dst);
            break;
        }
        case CPU_MNEM_SBB: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_sbb(dev, reg_src, reg_dst);
            break;
        }
        case CPU_MNEM_NOT: {
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 1];
            cpu_not(dev, reg_dst);
            break;
        }
        case CPU_MNEM_AND: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_and(dev, reg_src, reg_dst);
            break;
        }
        case CPU_MNEM_TEST: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_test(dev, reg_src, reg_dst);
            break;
        }
        case CPU_MNEM_OR: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_or(dev, reg_src, reg_dst);
            break;
        }
        case CPU_MNEM_XOR: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_xor(dev, reg_src, reg_dst);
            break;
        }
        case CPU_MNEM_SHL: {
            uint8_t shift_in_reg = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_shl(dev, shift_in_reg, reg_dst);
            break;
        }
        case CPU_MNEM_SHR: {
            uint8_t shift_in_reg = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_shr(dev, shift_in_reg, reg_dst);
            break;
        }
        case CPU_MNEM_SAR: {
            uint8_t shift_in_reg = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_sar(dev, shift_in_reg, reg_dst);
            break;
        }
        case CPU_MNEM_JMP: {
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 1];
            cpu_jmp(dev, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_JE: {
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 1];
            cpu_je(dev, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_JNE: {
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 1];
            cpu_jne(dev, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_JG: {
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 1];
            cpu_jg(dev, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_JGE: {
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 1];
            cpu_jge(dev, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_JL: {
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 1];
            cpu_jl(dev, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_JLE: {
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 1];
            cpu_jle(dev, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_JA: {
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 1];
            cpu_ja(dev, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_JAE: {
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 1];
            cpu_jae(dev, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_JB: {
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 1];
            cpu_jb(dev, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_JBE: {
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 1];
            cpu_jbe(dev, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_CALL: {
            uint8_t reg_with_cs_of_dst = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_with_addr_of_dst = dev->mem.bytes[inst_addr + 2];
            cpu_call(dev, reg_with_cs_of_dst, reg_with_addr_of_dst);
            break;
        }
        case CPU_MNEM_RET: {
            cpu_ret(dev);
            break;
        }
        case CPU_MNEM_IRET: {
            cpu_iret(dev);
            break;
        }
        case CPU_MNEM_INB: {
            uint8_t src_port_in_reg = dev->mem.bytes[inst_addr + 1];
            uint8_t reg_dst = dev->mem.bytes[inst_addr + 2];
            cpu_inb(dev, src_port_in_reg, reg_dst);
            break;
        }
        case CPU_MNEM_OUTB: {
            uint8_t reg_src = dev->mem.bytes[inst_addr + 1];
            uint8_t dst_port_in_reg = dev->mem.bytes[inst_addr + 2];
            cpu_outb(dev, reg_src, dst_port_in_reg);
            break;
        }
        case CPU_MNEM_CLI: {
            cpu_cli(dev);
            break;
        }
        case CPU_MNEM_STI: {
            cpu_sti(dev);
            break;
        }
        case CPU_MNEM_NOP: {
            cpu_nop(dev);
            break;
        }
        default: {
            cpu_raise_excpt(dev, CPU_EXCPT_INVALID_INSTR);
        }
    }
}

static void mem_step(device_t *dev)
{
    assert(dev != NULL);

    // empty for now, as we don't need to refresh DRAM in our emulator :)
}

static void pic_step(device_t *dev)
{
    assert(dev != NULL);

    // empty for now
}

static void pit_step(device_t *dev)
{
    assert(dev != NULL);

    uint16_t cnt = dev->pit.cnt_high << 8 | dev->pit.cnt_low;
    --cnt;
    if (cnt == 0) {
        cpu_raise_irq(dev, PIT_IRQ);
    }
    dev->pit.cnt_low  = (uint8_t) (cnt & 0xff);
    dev->pit.cnt_high = (uint8_t) ((cnt >> 8) & 0xff);
}

static void kbdc_step(device_t *dev, uint8_t external_scancode)
{
    assert(dev != NULL);

    if (external_scancode != 0) {
        dev->kbdc.scancode = external_scancode;

        cpu_raise_irq(dev, KBDC_IRQ);
    }
}

static void dispc_step(device_t *dev)
{
    assert(dev != NULL);

    ++dev->dispc.cycles;
    if (dev->dispc.cycles >= DISPC_CYCLES_PER_FRAME) {
        dev->dispc.cycles = 0;
    }

    if (dev->dispc.cycles >= DISPC_CYCLES_BEFORE_VBLANK) {
        dev->dispc.status |= DISPC_FLAG_VBLANK;
    } else {
        dev->dispc.status &= ~DISPC_FLAG_VBLANK;
    }
}

static void dev_step(device_t *dev, uint8_t external_scancode)
{
    assert(dev != NULL);

    cpu_step(dev);
    mem_step(dev);
    pic_step(dev);
    pit_step(dev);
    kbdc_step(dev, external_scancode);
    dispc_step(dev);
}

/* Helper Functions */

static void cpu_to_string(cpu_t *cpu, mem_t *mem, char *buff, size_t buff_size)
{
    assert(cpu != NULL);
    assert(mem != NULL);

    assert(buff != NULL);
    assert(buff_size > 0);

    size_t pos = 0;

    int res = snprintf(
        buff, buff_size,

        "CPU State:\n"
        " General-Purpose Registers:\n"
        "  r0: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  r1: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  r2: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  r3: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  r4: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  r5: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  r6: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  r7: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        " Segment Registers:\n"
        "  cs: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  ds: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  ss: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        " Special-Purpose Registers:\n"
        "  fp: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  sp: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  ip: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  ls: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  lr: 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "  flags: %u%u%u%u%u%u%u%u, 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "         RRRIOSZC\n"
        " Internal Data:\n"
        "  cycles: %zu",

        cpu->r0, cpu->r0,
        cpu->r1, cpu->r1,
        cpu->r2, cpu->r2,
        cpu->r3, cpu->r3,
        cpu->r4, cpu->r4,
        cpu->r5, cpu->r5,
        cpu->r6, cpu->r6,
        cpu->r7, cpu->r7,

        cpu->cs, cpu->cs,
        cpu->ds, cpu->ds,
        cpu->ss, cpu->ss,

        cpu->fp, cpu->fp,
        cpu->sp, cpu->sp,
        cpu->ip, cpu->ip,
        cpu->ls, cpu->ls,
        cpu->lr, cpu->lr,
        (cpu->flags & 0b10000000) >> 7,
        (cpu->flags & 0b01000000) >> 6,
        (cpu->flags & 0b00100000) >> 5,
        (cpu->flags & 0b00010000) >> 4,
        (cpu->flags & 0b00001000) >> 3,
        (cpu->flags & 0b00000100) >> 2,
        (cpu->flags & 0b00000010) >> 1,
        (cpu->flags & 0b00000001),
        cpu->flags, cpu->flags,

        cpu->cycles
    );
    if (res < 0 || (ssize_t) buff_size - (ssize_t) (pos += res) <= 1) {
        goto out;
    }

    for (size_t i = 0; i < CPU_ISRS; ++i) {
        res = snprintf(
            buff + pos,
            buff_size - pos,
            i == 0 ?
                "\n Interrupt Service Routines:\n"
                "  ISR of IRQ %2zu: 0x%02" PRIx8 ":0x%02" PRIx8 "\n" :
                (i < CPU_ISRS - 1 ?
                    "  ISR of IRQ %2zu: 0x%02" PRIx8 ":0x%02" PRIx8 "\n" :
                    "  ISR of IRQ %2zu: 0x%02" PRIx8 ":0x%02" PRIx8),
            i, mem->bytes[i * 2], mem->bytes[i * 2 + 1]
        );
        if (res < 0 || (ssize_t) buff_size - (ssize_t) (pos += res) <= 1) {
            break;
        }
    }

out:
    if (pos < buff_size) {
        buff[pos] = '\0';
    } else {
        buff[buff_size - 1] = '\0';
    }
}

static void mem_to_string(mem_t *mem, char *buff, size_t buff_size)
{
    assert(mem != NULL);

    assert(buff != NULL);
    assert(buff_size > 0);

    size_t pos = 0;
    for (size_t i = 0; i < MEM_SIZE; ++i) {
        if (i % 8 == 0) {
            int res = snprintf(
                buff + pos,
                buff_size - pos,
                i == 0 ? "Memory State:\n [0x%04zx]:" : "\n [0x%04zx]:",
                i
            );
            if (res < 0 || (ssize_t) buff_size - (ssize_t) (pos += res) <= 1) {
                break;
            }
        }
        int res = snprintf(buff + pos, buff_size - pos, " 0x%02" PRIx8, mem->bytes[i]);
        if (res < 0 || (ssize_t) buff_size - (ssize_t) (pos += res) <= 1) {
            break;
        }
    }

    if (pos < buff_size) {
        buff[pos] = '\0';
    } else {
        buff[buff_size - 1] = '\0';
    }
}

static void pic_to_string(pic_t *pic, char *buff, size_t buff_size)
{
    assert(pic != NULL);

    assert(buff != NULL);
    assert(buff_size > 0);

    snprintf(
        buff, buff_size,

        "Programmable Interrupt Controller's State:\n"
        " imr: %u%u%u%u%u%u%u%u, 0x%02" PRIx8 " (%" PRIu8 ")\n"
        " vec_off: 0x%02" PRIx8,

        (pic->imr & 0b10000000) >> 7,
        (pic->imr & 0b01000000) >> 6,
        (pic->imr & 0b00100000) >> 5,
        (pic->imr & 0b00010000) >> 4,
        (pic->imr & 0b00001000) >> 3,
        (pic->imr & 0b00000100) >> 2,
        (pic->imr & 0b00000010) >> 1,
        (pic->imr & 0b00000001),
        pic->imr, pic->imr,
        pic->vec_off
    );
}

static void pit_to_string(pit_t *pit, char *buff, size_t buff_size)
{
    assert(pit != NULL);

    assert(buff != NULL);
    assert(buff_size > 0);

    snprintf(
        buff, buff_size,

        "Programmable Interval Timer's State:\n"
        " freq_div: 0x%02" PRIx8 ":0x%02" PRIx8 " (%" PRIu16 ")\n"
        " cnt:      0x%02" PRIx8 ":0x%02" PRIx8 " (%" PRIu16 ")",

        pit->freq_div_high, pit->freq_div_low,
        (uint16_t) (pit->freq_div_high << 8 | pit->freq_div_low),

        pit->cnt_high, pit->cnt_low,
        (uint16_t) (pit->cnt_high << 8 | pit->cnt_low)
    );
}

static void kbdc_to_string(kbdc_t *kbdc, char *buff, size_t buff_size)
{
    assert(kbdc != NULL);

    assert(buff != NULL);
    assert(buff_size > 0);

    if (isprint(kbdc->scancode)) {
        snprintf(
            buff, buff_size,
            "Keyboard Controller's State:\n scancode: 0x%02" PRIx8 " ('%c')",
            kbdc->scancode, (char) kbdc->scancode
        );
    } else {
        snprintf(
            buff, buff_size,
            "Keyboard Controller's State:\n scancode: 0x%02" PRIx8,
            kbdc->scancode
        );
    }
}

static void dispc_to_string(dispc_t *dispc, char *buff, size_t buff_size)
{
    assert(dispc != NULL);

    assert(buff != NULL);
    assert(buff_size > 0);

    snprintf(
        buff, buff_size,

        "Display Controller's State:\n"
        " mmio: 0x%02" PRIx8 ":0x%02" PRIx8 "\n"
        " status: %u%u%u%u%u%u%u%u, 0x%02" PRIx8 " (%" PRIu8 ")\n"
        "         RRRRRRRV",

        dispc->mmio_ds, dispc->mmio_addr,
        (dispc->status & 0b10000000) >> 7,
        (dispc->status & 0b01000000) >> 6,
        (dispc->status & 0b00100000) >> 5,
        (dispc->status & 0b00010000) >> 4,
        (dispc->status & 0b00001000) >> 3,
        (dispc->status & 0b00000100) >> 2,
        (dispc->status & 0b00000010) >> 1,
        (dispc->status & 0b00000001),
        dispc->status, dispc->status
    );
}

static void dispc_front_buff_to_string(dispc_t *dispc, mem_t *mem, char *buff, size_t buff_size)
{
    assert(dispc != NULL);
    assert(mem != NULL);

    assert(buff != NULL);
    assert(buff_size > 0);

    size_t pos = 0;

    int res = snprintf(
        buff, buff_size,
        "Display Buffer's State:\n "
    );
    if (res < 0 || (ssize_t) buff_size - (ssize_t) (pos += res) <= 1) {
        goto out;
    }

    for (size_t i = 0, y = 0; y < DISPC_SCR_HEIGHT; ++y) {
        for (size_t x = 0; x < DISPC_SCR_WIDTH; x += 8, ++i) {
            size_t eff_addr = (dispc->mmio_ds << 4) + dispc->mmio_addr;
            uint8_t byte = mem->bytes[eff_addr + i];
            for (size_t j = 0; j < 8; ++j) {
                uint8_t bit = (byte & 0b10000000) >> 7;
                res = snprintf(
                    buff + pos,
                    buff_size - pos,
                    bit == 1 ? "#" : "."
                );
                if (res < 0 || (ssize_t) buff_size - (ssize_t) (pos += res) <= 1) {
                    goto out;
                }
                byte <<= 1;
            }
        }
        if (y < DISPC_SCR_HEIGHT - 1) {
             res = snprintf(
                buff + pos,
                buff_size - pos,
                "\n "
            );
            if (res < 0 || (ssize_t) buff_size - (ssize_t) (pos += res) <= 1) {
                break;
            }
        }
    }

out:
    if (pos < buff_size) {
        buff[pos] = '\0';
    } else {
        buff[buff_size - 1] = '\0';
    }
}

#endif /* ASC_H */
