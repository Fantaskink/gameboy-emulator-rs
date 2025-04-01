use std::fs;

#[derive(Default, Debug)]
struct Registers {
    a: u8,
    f: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
}

impl Registers {
    fn af(&self) -> u16 {
        ((self.a as u16) << 8) | (self.f as u16)
    }
    fn bc(&self) -> u16 {
        ((self.b as u16) << 8) | (self.c as u16)
    }
    fn de(&self) -> u16 {
        ((self.d as u16) << 8) | (self.e as u16)
    }
    fn hl(&self) -> u16 {
        ((self.h as u16) << 8) | (self.l as u16)
    }

    fn set_af(&mut self, val: u16) {
        self.a = (val >> 8) as u8;
        self.f = (val & 0xFF) as u8;
    }
    fn set_bc(&mut self, val: u16) {
        self.b = (val >> 8) as u8;
        self.c = (val & 0xFF) as u8;
    }
    fn set_de(&mut self, val: u16) {
        self.d = (val >> 8) as u8;
        self.e = (val & 0xFF) as u8;
    }
    fn set_hl(&mut self, val: u16) {
        self.h = (val >> 8) as u8;
        self.l = (val & 0xFF) as u8;
    }
}

#[derive(Debug)]
struct Cpu {
    registers: Registers,
    pc: u16,
    sp: u16,
    memory: [u8; 0x10000], // 64KB memory
    cycles: u64,           // T-cycles
}

impl Cpu {
    fn add_r(&mut self, register: &mut u8, value: u8) {
        let (result, carry) = register.overflowing_add(value);
        let half_carry = ((*register & 0x0F) + (value & 0x0F)) > 0x0F;
        *register = result;

        self.registers.f &= 0b00010000; // Clear N flag
        if *register == 0 {
            self.registers.f |= 0b10000000; // Set Z flag
        }
        if half_carry {
            self.registers.f |= 0b00100000; // Set H flag
        }
        if carry {
            self.registers.f |= 0b00010000; // Set C flag
        }
    }
    fn inc_r(register: &mut u8, flags: &mut u8) {
        let half_carry = (*register & 0x0F) == 0x0F; // Check for half-carry
        *register = register.wrapping_add(1); // Increment with wrapping

        *flags &= 0b00010000; // Preserve Carry (C) flag
        if *register == 0 {
            *flags |= 0b10000000; // Set Zero (Z) flag
        }
        if half_carry {
            *flags |= 0b00100000; // Set Half-Carry (H) flag
        }
    }

    fn dec_r(register: &mut u8, flags: &mut u8) {
        let half_borrow = (*register & 0x0F) == 0x00; // Check for half-borrow
        *register = register.wrapping_sub(1); // Decrement with wrapping

        *flags &= 0b00010000; // Preserve Carry (C) flag
        if *register == 0 {
            *flags |= 0b10000000; // Set Zero (Z) flag
        }
        if half_borrow {
            *flags |= 0b00100000; // Set Half-Carry (H) flag
        }
        *flags |= 0b01000000; // Set Subtract (N) flag
    }

    fn inc_sp(&mut self) {
        self.sp = self.sp.wrapping_add(1); // Increment with wrapping
    }
    fn dec_sp(&mut self) {
        self.sp = self.sp.wrapping_sub(1); // Decrement with wrapping
    }

    fn increment_clock(&mut self, m_cycles: u64) {
        self.cycles += m_cycles * 4; // 4 T-cycles per M-cycle
    }

    fn fetch(&mut self) -> u8 {
        let opcode = self.memory[self.pc as usize];
        self.pc += 1;
        opcode
    }

    fn read_memory(&mut self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn write_memory(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    fn nop(&mut self) {
        // NOP
        self.increment_clock(1);
    }
    fn ld_bc_d16(&mut self) {
        // LD BC,d16
        let nn_lsb = self.fetch();
        let nn_msb = self.fetch();
        let nn = ((nn_lsb as u16) << 8) | (nn_msb as u16);
        self.registers.set_bc(nn);
        self.increment_clock(3);
    }
    fn ld_bc_a(&mut self) {
        // LD (BC),A
        self.write_memory(self.registers.bc(), self.registers.a);
        self.increment_clock(2);
    }
    fn inc_bc(&mut self) {
        // INC BC
        self.registers.set_bc(self.registers.bc().wrapping_add(1));
        self.increment_clock(2);
    }
    fn inc_b(&mut self) {
        // INC B
        Cpu::inc_r(&mut self.registers.b, &mut self.registers.f);
        self.increment_clock(1);
    }

    fn dec_b(&mut self) {
        // DEC B
        Cpu::dec_r(&mut self.registers.b, &mut self.registers.f);
        self.increment_clock(1);
    }
    fn ld_b_d8(&mut self) {
        // LD B, d8
        let value = self.fetch();
        self.registers.b = value;
        self.increment_clock(2);
    }

    fn rlca(&mut self) {
        // RLCA
        let carry = self.registers.a & 0x80;
        self.registers.a = (self.registers.a << 1) | (carry >> 7);
        if carry != 0 {
            self.registers.f |= 0b00010000; // Set Carry (C) flag
        } else {
            self.registers.f &= !0b00010000; // Clear Carry (C) flag
        }
        self.increment_clock(1);
    }
    fn ld_a16_sp(&mut self) {
        // LD (a16), SP
        // Store the stack pointer (SP) at the address specified by the next two bytes
        let low_byte = self.sp as u8;
        let high_byte = (self.sp >> 8) as u8;
        let addr_lsb = self.fetch();
        let addr_msb = self.fetch();
        let addr = ((addr_lsb as u16) << 8) | (addr_msb as u16);
        self.write_memory(addr, low_byte);
        self.write_memory(addr + 1, high_byte);
        self.increment_clock(5);
    }
    fn add_hl_bc(&mut self) {
        // ADD HL, BC
        let hl = self.registers.hl();
        let bc = self.registers.bc();
        let (result, carry) = hl.overflowing_add(bc);
        let half_carry = ((hl & 0x0FFF) + (bc & 0x0FFF)) > 0x0FFF;
        self.registers.set_hl(result);
        self.registers.f &= 0b00010000; // Clear N flag
        if result == 0 {
            self.registers.f |= 0b10000000; // Set Z flag
        }
        if half_carry {
            self.registers.f |= 0b00100000; // Set H flag
        }
        if carry {
            self.registers.f |= 0b00010000; // Set C flag
        }
        self.increment_clock(2);
    }
    fn ld_a_bc(&mut self) {
        // LD A, (BC)
        let addr = self.registers.bc();
        self.registers.a = self.read_memory(addr);
        self.increment_clock(2);
    }
    fn dec_bc(&mut self) {
        // DEC BC
        self.registers.set_bc(self.registers.bc().wrapping_sub(1));
        self.increment_clock(2);
    }
    fn inc_c(&mut self) {
        // INC C
        Cpu::inc_r(&mut self.registers.c, &mut self.registers.f);
        self.increment_clock(1);
    }
    fn dec_c(&mut self) {
        // DEC C
        Cpu::dec_r(&mut self.registers.c, &mut self.registers.f);
        self.increment_clock(1);
    }
    fn ld_c_d8(&mut self) {
        // LD C, d8
        let value = self.fetch();
        self.registers.c = value;
        self.increment_clock(2);
    }
    fn rrca(&mut self) {
        // RRCA
        let carry = self.registers.a & 0x01;
        self.registers.a = (self.registers.a >> 1) | (carry << 7);
        if carry != 0 {
            self.registers.f |= 0b00010000; // Set Carry (C) flag
        } else {
            self.registers.f &= !0b00010000; // Clear Carry (C) flag
        }
        self.increment_clock(1);
    }
    fn stop(&mut self) {
        /*
        TODO: Implement STOP instruction
        STOP
        This is a placeholder for the STOP instruction.
        In a real implementation, this would halt the CPU until an interrupt occurs.
        */
        self.increment_clock(1);
    }
    fn ld_de_d16(&mut self) {
        // LD DE, d16
        let nn_lsb = self.fetch();
        let nn_msb = self.fetch();
        let nn = ((nn_lsb as u16) << 8) | (nn_msb as u16);
        self.registers.set_de(nn);
        self.increment_clock(3);
    }
    fn ld_de_a(&mut self) {
        // LD (DE), A
        self.write_memory(self.registers.de(), self.registers.a);
        self.increment_clock(2);
    }
    fn inc_de(&mut self) {
        // INC DE
        self.registers.set_de(self.registers.de().wrapping_add(1));
        self.increment_clock(2);
    }
    fn inc_d(&mut self) {
        // INC D
        Cpu::inc_r(&mut self.registers.d, &mut self.registers.f);
        self.increment_clock(1);
    }
    fn dec_d(&mut self) {
        // DEC D
        Cpu::dec_r(&mut self.registers.d, &mut self.registers.f);
        self.increment_clock(1);
    }
    fn ld_d_d8(&mut self) {
        // LD D, d8
        let value = self.fetch();
        self.registers.d = value;
        self.increment_clock(2);
    }
    fn rla(&mut self) {
        /* Rotate the contents of register A to the left, through the carry (CY) flag.
        That is, the contents of bit 0 are copied to bit 1, and the previous contents of bit 1 (before the copy operation) are copied to bit 2.
        The same operation is repeated in sequence for the rest of the register. The previous contents of the carry flag are copied to bit 0. */
        let carry = self.registers.f & 0b00010000;
        self.registers.f &= 0b00001111; // Clear all flags except for H
        if carry != 0 {
            self.registers.f |= 0b00010000; // Set C flag
        }
        self.registers.a = (self.registers.a << 1) | (carry >> 4);
        if self.registers.a == 0 {
            self.registers.f |= 0b10000000; // Set Z flag
        }
        if (self.registers.a & 0x0F) == 0x0F {
            self.registers.f |= 0b00100000; // Set H flag
        }
        self.increment_clock(1);
    }
    fn jr_s8(&mut self) {
        // JR s8
        let offset = self.fetch() as i8;
        self.pc = self.pc.wrapping_add(offset as u16);
        self.increment_clock(3);
    }
    fn add_hl_de(&mut self) {
        // ADD HL, DE
        let hl = self.registers.hl();
        let de = self.registers.de();
        let (result, carry) = hl.overflowing_add(de);
        let half_carry = ((hl & 0x0FFF) + (de & 0x0FFF)) > 0x0FFF;
        self.registers.set_hl(result);
        self.registers.f &= 0b00010000; // Clear N flag
        if result == 0 {
            self.registers.f |= 0b10000000; // Set Z flag
        }
        if half_carry {
            self.registers.f |= 0b00100000; // Set H flag
        }
        if carry {
            self.registers.f |= 0b00010000; // Set C flag
        }
        self.increment_clock(2);
    }
    fn execute(&mut self, opcode: u8) {
        match opcode {
            0x00 => self.nop(),
            0x01 => self.ld_bc_d16(),
            0x02 => self.ld_bc_a(),
            0x03 => self.inc_bc(),
            0x04 => self.inc_b(),
            0x05 => self.dec_b(),
            0x06 => self.ld_b_d8(),
            0x07 => self.rlca(),
            0x08 => self.ld_a16_sp(),
            0x09 => self.add_hl_bc(),
            0x0A => self.ld_a_bc(),
            0x0B => self.dec_bc(),
            0x0C => self.inc_c(),
            0x0D => self.dec_c(),
            0x0E => self.ld_c_d8(),
            0x0F => self.rrca(),
            0x10 => self.stop(),
            0x11 => self.ld_de_d16(),
            0x12 => self.ld_de_a(),
            0x13 => self.inc_de(),
            0x14 => self.inc_d(),
            0x15 => self.dec_d(),
            0x16 => self.ld_d_d8(),
            0x17 => self.rla(),
            0x18 => self.jr_s8(),
            0x19 => self.add_hl_de(),
            0x3E => {
                // LD A, imm8 (Load an 8-bit immediate value into A)
                let value = self.fetch();
                self.registers.a = value;
                self.increment_clock(4);
            }

            0x1A => {
                // LD A, (DE): Load accumulator (indirect DE)
                let addr = self.registers.de();
                self.registers.a = self.read_memory(addr);
                self.increment_clock(2);
            }
            0x36 => {
                // LD (HL), n: Load from immediate data (indirect HL)
                let n = self.fetch();
                self.write_memory(self.registers.hl(), n);
                self.increment_clock(4);
            }
            0x41 => {
                // LD B, C
                self.registers.b = self.registers.c;
                self.increment_clock(1);
            }
            0x46 => {
                // LD B, (HL)
                let memory = self.read_memory(self.registers.hl());
                self.registers.b = memory;
                self.increment_clock(2);
            }
            0x70 => {
                // LD (HL), B
                self.write_memory(self.registers.hl(), self.registers.b);
                self.increment_clock(2);
            }
            0x80 => {
                // ADD A, B
                let (result, carry) = self.registers.a.overflowing_add(self.registers.b);
                let half_carry = ((self.registers.a & 0x0F) + (self.registers.b & 0x0F)) > 0x0F;

                self.registers.a = result;
                self.registers.f = 0; // Clear all flags
                if result == 0 {
                    self.registers.f |= 0b10000000;
                } // Set Z if result is 0
                if half_carry {
                    self.registers.f |= 0b00100000;
                } // Set H if half-carry occurs
                if carry {
                    self.registers.f |= 0b00010000;
                } // Set C if carry occurs

                self.increment_clock(1);
            }
            _ => panic!("Unknown opcode: {:#04x}", opcode),
        }
    }

    fn step(&mut self) {
        let opcode = self.fetch();
        self.execute(opcode);

        // Sync with PPU (Graphics Processing)
        //self.ppu.step(cycles);
    }

    fn run(&mut self) {
        loop {
            self.step();
        }
    }
}

fn load_rom(file_path: &str) -> Vec<u8> {
    match fs::read(file_path) {
        Ok(rom) => rom,
        Err(err) => {
            panic!("{}", err)
        }
    }
}

fn main() {
    let rom = load_rom("./rom/pokemon.gb");

    let mut cpu = Cpu {
        registers: Registers::default(),
        pc: 0x100,
        sp: 0xFFFE,
        memory: [0; 0x10000],
        cycles: 0,
    };

    let rom_size = rom.len().min(0x8000);
    cpu.memory[..rom_size].copy_from_slice(&rom[..rom_size]);

    cpu.run();
}
