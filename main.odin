package main 

import "core:c/libc"
import "core:sys/posix"
import "core:fmt"
import "core:os"
import "core:slice"
import "base:runtime"

MEMORY_MAX :: 65535  // 0xFFFF
memory: [65536]u16 // Array of 65535 16-bit unsigned integers

Register :: enum {
    R_R0 = 0,
    R_R1,
    R_R2,
    R_R3,
    R_R4,
    R_R5,
    R_R6,
    R_R7,
    R_PC, // program counter
    R_COND,
    R_COUNT
}

Opcodes :: enum {
    OP_BR = 0, /* branch */
    OP_ADD,    /* add  */
    OP_LD,     /* load */
    OP_ST,     /* store */
    OP_JSR,    /* jump register */
    OP_AND,    /* bitwise and */
    OP_LDR,    /* load register */
    OP_STR,    /* store register */
    OP_RTI,    /* unused */
    OP_NOT,    /* bitwise not */
    OP_LDI,    /* load indirect */
    OP_STI,    /* store indirect */
    OP_JMP,    /* jump */
    OP_RES,    /* reserved (unused) */
    OP_LEA,    /* load effective address */
    OP_TRAP    /* execute trap */
}

Cond_flags :: enum {
    FL_POS = 1 << 0, /* P */
    FL_ZRO = 1 << 1, /* Z */
    FL_NEG = 1 << 2 /* N */
}

Trap_codes :: enum {
    TRAP_GETC = 0x20,  /* get character from keyboard, not echoed onto the terminal */
    TRAP_OUT = 0x21,   /* output a character */
    TRAP_PUTS = 0x22,  /* output a word string */
    TRAP_IN = 0x23,    /* get character from keyboard, echoed onto the terminal */
    TRAP_PUTSP = 0x24, /* output a byte string */
    TRAP_HALT = 0x25   /* halt the program */
}

Mapped_Register :: enum {
    MR_KBSR = 0xFE00, /* keyboard status */
    MR_KBDR = 0xFE02  /* keyboard data */
}

reg: [Register.R_COUNT]u16

PC_START :: 0x3000

mem_write :: proc(address: u16, val: u16) {
    memory[address] = val
}

mem_read :: proc(address: u16) -> u16 {
    if (address == cast(u16)Mapped_Register.MR_KBSR) {
        if check_key() != 0 {
            memory[Mapped_Register.MR_KBSR] = (1 << 15)
            memory[Mapped_Register.MR_KBDR] = cast(u16)libc.getchar()
        } else {
            memory[Mapped_Register.MR_KBSR] = 0
        }
    }

    return memory[address]
}

sign_extend :: proc(x: u16, bit_count: uint) -> u16 {
    if (x >> (bit_count - 1)) != 0 & 1 {
        return x | (0xFFFF << uint(bit_count))
    }
    return x
}

update_flags :: proc(r: u16) {
    if (reg[r] == 0)
    {
        reg[Register.R_COND] = cast(u16)Cond_flags.FL_ZRO;
    }
    else if (reg[r] >> 15) != 0 /* a 1 in the left-most bit indicates negative */
    {
        reg[Register.R_COND] = cast(u16)Cond_flags.FL_NEG;
    }
    else
    {
        reg[Register.R_COND] = cast(u16)Cond_flags.FL_POS;
    }
}

read_image_file :: proc(file: ^os.Handle) {
    /* the origin tells us where in memory to place the image */
    origin : u16
    os.read(file^, slice.bytes_from_ptr(&origin, size_of(origin)))

    // bytes_read, _ := os.read(file^, slice.bytes_from_ptr(&origin, size_of(origin)))
    origin = swap16(origin)

    /* we know the maximum file size so we only need one fread */
    max_read := MEMORY_MAX - origin
    p := memory[origin:]
    bytes_read, _ := os.read(file^, slice.to_bytes(p[:max_read]))

    /* swap to little endian */
    for i := 0; i < bytes_read/2; i += 1 {
        p[i] = swap16(p[i])
    }
}

swap16 :: proc(x: u16) -> u16 {
    return ((x << 8) & 0xFF00) | ((x >> 8) & 0x00FF)
}

read_image :: proc(image_path: string) -> bool {
    file, _ := os.open(image_path, os.O_RDONLY)
    defer os.close(file)
    
    read_image_file(&file)
    return true   
}

handle_interrupt :: proc "cdecl" (signal: i32){
    context = runtime.default_context()
    restore_input_buffering()
    fmt.println("/n")
    os.exit(-2)
}

main :: proc() {
    if len(os.args) < 2 {
        // show usage string
        fmt.println("lc3 [image-file1] ...")
        os.exit(2)
    }

    for arg in os.args[1:] {
        if !read_image(arg){
            fmt.println("failed to load image: %s\n", arg) 
            os.exit(1)
        }
    }

    libc.signal(libc.SIGINT, handle_interrupt)
    disable_input_buffering()

    /* since exactly one condition flag should be set at any given time, set the Z flag */
    reg[Register.R_COND] = cast(u16)Cond_flags.FL_ZRO;

    /* set the PC to starting position */
    /* 0x3000 is the default */
    PC_START := 0x3000;
    reg[Register.R_PC] = cast(u16)PC_START;

    running := true;
    for running {
       // FETCH
        instr := mem_read(reg[Register.R_PC]); reg[Register.R_PC] += 1
        op := instr >> 12

        switch Opcodes(op) {
            case .OP_ADD:
                // destination register (DR)
                r0 := (instr >> 9) & 0x7;
                // first operand (SR1)
                r1 := (instr >> 6) & 0x7;
                // whether we are in immediate mode
                imm_flag := (instr >> 5) & 0x1;

                if (imm_flag != 0)
                {
                    imm5 := sign_extend(instr & 0x1F, 5);
                    reg[r0] = reg[r1] + imm5;
                }
                else
                {
                    r2 := instr & 0x7;
                    reg[r0] = reg[r1] + reg[r2];
                }

                update_flags(r0);

            case .OP_AND:
                r0 := (instr >> 9) & 0x7;
                r1 := (instr >> 6) & 0x7;
                imm_flag := (instr >> 5) & 0x1;

                if (imm_flag != 0)
                {
                    imm5 := sign_extend(instr & 0x1F, 5);
                    reg[r0] = reg[r1] & imm5;
                }
                else
                {
                    r2 := instr & 0x7;
                    reg[r0] = reg[r1] & reg[r2];
                }
                update_flags(r0);

            case .OP_NOT:
                r0 := (instr >> 9) & 0x7;
                r1 := (instr >> 6) & 0x7;

                reg[r0] = ~reg[r1];
                update_flags(r0);

            case .OP_BR:
                pc_offset := sign_extend(instr & 0x1FF, 9);
                cond_flag := (instr >> 9) & 0x7;

                if (cond_flag & reg[Register.R_COND]) != 0 {
                    reg[Register.R_PC] += pc_offset;
                }

            case .OP_JMP:
                /* Also handles RET */
                r1 := (instr >> 6) & 0x7;
                reg[Register.R_PC] = reg[r1];

            case .OP_JSR:
                long_flag := (instr >> 11) & 1;
                reg[Register.R_R7] = reg[Register.R_PC];
                if (long_flag != 0)
                {
                    long_pc_offset := sign_extend(instr & 0x7FF, 11);
                    reg[Register.R_PC] += long_pc_offset;  /* JSR */
                }
                else
                {
                    r1 := (instr >> 6) & 0x7;
                    reg[Register.R_PC] = reg[r1]; /* JSRR */
                }

            case .OP_LD:
                r0 := (instr >> 9) & 0x7;
                pc_offset := sign_extend(instr & 0x1FF, 9);
                reg[r0] = mem_read(reg[Register.R_PC] + pc_offset);
                update_flags(r0);

            case .OP_LDI:
                /* destination register (DR) */
                r0 := (instr >> 9) & 0x7;
                /* PCoffset 9*/
                pc_offset := sign_extend(instr & 0x1FF, 9);
                /* add pc_offset to the current PC, look at that memory location to get the final address */
                reg[r0] = mem_read(mem_read(reg[Register.R_PC] + pc_offset));
                update_flags(r0);

            case .OP_LDR:
                r0 := (instr >> 9) & 0x7;
                r1 := (instr >> 6) & 0x7;
                offset := sign_extend(instr & 0x3F, 6);
                reg[r0] = mem_read(reg[r1] + offset);
                update_flags(r0);
                
            case .OP_LEA:
                r0 := (instr >> 9) & 0x7;
                pc_offset := sign_extend(instr & 0x1FF, 9);
                reg[r0] = reg[Register.R_PC] + pc_offset;
                update_flags(r0);

            case .OP_ST:
                r0 := (instr >> 9) & 0x7;
                pc_offset := sign_extend(instr & 0x1FF, 9);
                mem_write(reg[Register.R_PC] + pc_offset, reg[r0]);

            case .OP_STI:
                r0 := (instr >> 9) & 0x7;
                pc_offset := sign_extend(instr & 0x1FF, 9);
                mem_write(mem_read(reg[Register.R_PC] + pc_offset), reg[r0]);

            case .OP_STR:
                r0 := (instr >> 9) & 0x7;
                r1 := (instr >> 6) & 0x7;
                offset := sign_extend(instr & 0x3F, 6);
                mem_write(reg[r1] + offset, reg[r0]);

            case .OP_TRAP:
                reg[Register.R_R7] = reg[Register.R_PC];

                switch Trap_codes(instr & 0xFF) {
                    case .TRAP_GETC:
                        /* read a single ASCII char */
                        reg[Register.R_R0] = cast(u16)libc.getchar()
                        update_flags(cast(u16)Register.R_R0)

                    case .TRAP_OUT:
                        os.write_byte(os.stdout, u8(reg[Register.R_R0]))
                        os.flush(os.stdout)

                    case .TRAP_PUTS:
                        /* one char per word */
                        addr := reg[Register.R_R0]
                        for c := addr; memory[c] != 0; c += 1 {
                            os.write_byte(os.stdout, u8(memory[c]))
                        }
                        os.flush(os.stdout)

                    case .TRAP_IN:
                        fmt.println("Enter a character: ")
                        c := libc.getchar()
                        os.write_byte(os.stdout, u8(c))
                        os.flush(os.stdout)
                        reg[Register.R_R0] = cast(u16)c
                        update_flags(cast(u16)Register.R_R0)

                    case .TRAP_PUTSP:
                        /* one char per byte (two bytes per word)
                           here we need to swap back to
                           big endian format */
                        addr := reg[Register.R_R0]
                        for c := addr; memory[c] != 0; c += 1 {
                            char1 := memory[c] & 0xFF
                            os.write_byte(os.stdout, u8(char1))
                            char2 := memory[c] >> 8
                            if char2 != 0 {
                                os.write_byte(os.stdout, u8(char2))
                            }
                        }
                        os.flush(os.stdout)

                    case .TRAP_HALT:
                        os.flush(os.stdout);
                        running = false;
                }

            case .OP_RES, .OP_RTI:
                // do nothing
            case:
                runtime.panic("Bad opcode!")
        }
    }
    restore_input_buffering();
}

// *nix specific stuff

original_tio := posix.termios{}

disable_input_buffering :: proc() {
    posix.tcgetattr(posix.STDIN_FILENO, &original_tio)
    new_tio := original_tio
    new_tio.c_lflag &~= {.ICANON, .ECHO}
    posix.tcsetattr(posix.STDIN_FILENO, posix.TC_Optional_Action.TCSANOW, &new_tio)
}

restore_input_buffering :: proc() {
    posix.tcsetattr(posix.STDIN_FILENO, posix.TC_Optional_Action.TCSANOW, &original_tio)
}

check_key :: proc() -> u16 {
    readfds := posix.fd_set{}
    posix.FD_ZERO(&readfds)
    posix.FD_SET(posix.STDIN_FILENO, &readfds)

    // not sure the correct syntax of how to do this...
    timeout := posix.timeval {
        tv_sec = 0,
        tv_usec = 0,
    }

    return posix.select(1, &readfds, nil, nil, &timeout) != 0
}

