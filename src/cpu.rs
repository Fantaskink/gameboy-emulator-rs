#[derive(Default, Debug)]
pub struct Registers {
    pub a: u8,
    pub f: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub h: u8,
    pub l: u8,
}

#[derive(Debug)]
pub enum Flag {
    Zero, // Zero flag
    N,    // Subtract flag
    H,    // Half-carry flag
    C,    // Carry flag
}

pub enum DoubleRegister {
    AF,
    BC,
    DE,
    HL,
}

impl Registers {
    pub fn get_flag_bit(&self, flag: Flag) -> u8 {
        match flag {
            Flag::Zero => (self.f >> 7) & 1,
            Flag::N => (self.f >> 6) & 1,
            Flag::H => (self.f >> 5) & 1,
            Flag::C => (self.f >> 4) & 1,
        }
    }

    pub fn set_flag_bit(&mut self, flag: Flag, value: u8) {
        match flag {
            Flag::Zero => {
                if value != 0 {
                    self.f |= 0b10000000; // Set Z flag
                } else {
                    self.f &= !0b10000000; // Clear Z flag
                }
            }
            Flag::N => {
                if value != 0 {
                    self.f |= 0b01000000; // Set N flag
                } else {
                    self.f &= !0b01000000; // Clear N flag
                }
            }
            Flag::H => {
                if value != 0 {
                    self.f |= 0b00100000; // Set H flag
                } else {
                    self.f &= !0b00100000; // Clear H flag
                }
            }
            Flag::C => {
                if value != 0 {
                    self.f |= 0b00010000; // Set C flag
                } else {
                    self.f &= !0b00010000; // Clear C flag
                }
            }
        }
    }
    pub fn is_flag_set(&self, flag: Flag) -> bool {
        match flag {
            Flag::Zero => self.get_flag_bit(Flag::Zero) != 0,
            Flag::N => self.get_flag_bit(Flag::N) != 0,
            Flag::H => self.get_flag_bit(Flag::H) != 0,
            Flag::C => self.get_flag_bit(Flag::C) != 0,
        }
    }
    pub fn af(&self) -> u16 {
        ((self.a as u16) << 8) | (self.f as u16)
    }
    pub fn bc(&self) -> u16 {
        ((self.b as u16) << 8) | (self.c as u16)
    }
    pub fn de(&self) -> u16 {
        ((self.d as u16) << 8) | (self.e as u16)
    }
    pub fn hl(&self) -> u16 {
        ((self.h as u16) << 8) | (self.l as u16)
    }

    pub fn set_af(&mut self, val: u16) {
        self.a = (val >> 8) as u8;
        self.f = (val & 0xFF) as u8;
    }
    pub fn set_bc(&mut self, val: u16) {
        self.b = (val >> 8) as u8;
        self.c = (val & 0xFF) as u8;
    }
    pub fn set_de(&mut self, val: u16) {
        self.d = (val >> 8) as u8;
        self.e = (val & 0xFF) as u8;
    }
    pub fn set_hl(&mut self, val: u16) {
        self.h = (val >> 8) as u8;
        self.l = (val & 0xFF) as u8;
    }
}

#[derive(Debug)]
pub struct Cpu {
    pub pc: u16,
    pub sp: u16,
    pub memory: [u8; 0x10000], // 64KB memory
    pub cycles: u64,           // T-cycles
}

impl Default for Cpu {
    fn default() -> Self {
        Cpu {
            pc: 0x100, // Start at 0x100 for ROM
            sp: 0xFFFE,
            memory: [0; 0x10000],
            cycles: 0,
        }
    }
}

impl Cpu {
    pub fn new() -> Self {
        Cpu::default()
    }
    fn add_r(&mut self, registers: &mut Registers, register: &mut u8, value: u8) {
        let (result, carry) = register.overflowing_add(value);
        let half_carry = ((*register & 0x0F) + (value & 0x0F)) > 0x0F;

        *register = result;

        let zero_flag = if result == 0 { 1 } else { 0 };
        let carry_flag = if carry { 1 } else { 0 };
        let half_carry_flag = if half_carry { 1 } else { 0 };

        registers.set_flag_bit(Flag::Zero, zero_flag);
        registers.set_flag_bit(Flag::N, 0);
        registers.set_flag_bit(Flag::H, half_carry_flag);
        registers.set_flag_bit(Flag::C, carry_flag);
        self.increment_clock(1);
    }
    fn inc_r(&mut self, registers: &mut Registers, register: &mut u8) {
    
        let half_carry = (*register & 0x0F) == 0x0F;
        *register = register.wrapping_add(1);
    
        registers.set_flag_bit(Flag::Zero, if *register == 0 { 1 } else { 0 });
        registers.set_flag_bit(Flag::N, 0);
        registers.set_flag_bit(Flag::H, if half_carry { 1 } else { 0 });
    
        self.increment_clock(1);
    }

    fn dec_r(&mut self, registers: &mut Registers, register: &mut u8) {
        let half_borrow = (*register & 0x0F) == 0; // Check for half-borrow
        *register = register.wrapping_sub(1); // Decrement with wrapping

        // Update flags using the `Registers` methods
        registers.set_flag_bit(Flag::Zero, if *register == 0 { 1 } else { 0 });
        registers.set_flag_bit(Flag::N, 1); // Set Subtract (N) flag
        registers.set_flag_bit(Flag::H, if half_borrow { 1 } else { 0 }); // Set Half-Carry (H) flag
    }

    fn inc_rr(&mut self, registers: &mut Registers, register_pair: DoubleRegister) {
        // INC rr: Increment a 16-bit register pair
        let (high, low) = match register_pair {
            DoubleRegister::AF => (registers.a, registers.f),
            DoubleRegister::BC => (registers.b, registers.c),
            DoubleRegister::DE => (registers.d, registers.e),
            DoubleRegister::HL => (registers.h, registers.l),
        };

        let result = ((high as u16) << 8 | low as u16).wrapping_add(1);
        registers.set_hl(result);
        self.increment_clock(2);
    }

    fn dec_rr(&mut self, registers: &mut Registers, register_pair: DoubleRegister) {
        // DEC rr: Decrement a 16-bit register pair
        let (high, low) = match register_pair {
            DoubleRegister::AF => (registers.a, registers.f),
            DoubleRegister::BC => (registers.b, registers.c),
            DoubleRegister::DE => (registers.d, registers.e),
            DoubleRegister::HL => (registers.h, registers.l),
        };

        let result = ((high as u16) << 8 | low as u16).wrapping_sub(1);
        registers.set_hl(result);
        self.increment_clock(2);
    }

    fn ld_r_n(&mut self, register: &mut u8) {
        // LD r, n: Load an 8-bit immediate value into a register
        let value = self.fetch();
        *register = value;
        self.increment_clock(2);
    }

    fn ld_rr_nn(&mut self, registers: &mut Registers, register_pair: DoubleRegister) {
        // LD rr, nn: Load a 16-bit immediate value into a register pair
        let nn_lsb = self.fetch();
        let nn_msb = self.fetch();
        let nn = ((nn_lsb as u16) << 8) | (nn_msb as u16);

        match register_pair {
            DoubleRegister::AF => registers.set_af(nn),
            DoubleRegister::BC => registers.set_bc(nn),
            DoubleRegister::DE => registers.set_de(nn),
            DoubleRegister::HL => registers.set_hl(nn),
        }
        self.increment_clock(3);
    }

    fn add_hl_rr(&mut self, registers: &mut Registers, register_pair: DoubleRegister) {
        // ADD HL, rr: Add a 16-bit register pair to HL
        let hl = registers.hl();
        let rr = match register_pair {
            DoubleRegister::AF => registers.af(),
            DoubleRegister::BC => registers.bc(),
            DoubleRegister::DE => registers.de(),
            DoubleRegister::HL => registers.hl(),
        };
        let (result, carry) = hl.overflowing_add(rr);
        let half_carry = ((hl & 0x0FFF) + (rr & 0x0FFF)) > 0x0FFF;
        registers.set_hl(result);
        registers.f &= 0b00010000; // Clear N flag
        if result == 0 {
            registers.f |= 0b10000000; // Set Z flag
        }
        if half_carry {
            registers.f |= 0b00100000; // Set H flag
        }
        if carry {
            registers.f |= 0b00010000; // Set C flag
        }
        self.increment_clock(2);
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
    fn ld_bc_a(&mut self, registers: &mut Registers) {
        // LD (BC),A
        self.write_memory(registers.bc(), registers.a);
        self.increment_clock(2);
    }


    fn rlca(&mut self, registers: &mut Registers) {
        // RLCA
        let carry = registers.a & 0x80;
        registers.a = (registers.a << 1) | (carry >> 7);
        if carry != 0 {
            registers.f |= 0b00010000; // Set Carry (C) flag
        } else {
            registers.f &= !0b00010000; // Clear Carry (C) flag
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

    fn ld_a_bc(&mut self, registers: &mut Registers) {
        // LD A, (BC)
        let addr = registers.bc();
        registers.a = self.read_memory(addr);
        self.increment_clock(2);
    }
    fn rrca(&mut self, registers: &mut Registers) {
        // RRCA
        let carry = registers.a & 0x01;
        registers.a = (registers.a >> 1) | (carry << 7);
        if carry != 0 {
            registers.f |= 0b00010000; // Set Carry (C) flag
        } else {
            registers.f &= !0b00010000; // Clear Carry (C) flag
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
    fn ld_de_a(&mut self) {
        // LD (DE), A
        self.write_memory(registers.de(), registers.a);
        self.increment_clock(2);
    }

    fn rla(&mut self, registers: &mut Registers) {
        /* Rotate the contents of register A to the left, through the carry (CY) flag.
        That is, the contents of bit 0 are copied to bit 1, and the previous contents of bit 1 (before the copy operation) are copied to bit 2.
        The same operation is repeated in sequence for the rest of the register. The previous contents of the carry flag are copied to bit 0. */
        let carry = registers.f & 0b00010000;
        registers.f &= 0b00001111; // Clear all flags except for H
        if carry != 0 {
            registers.f |= 0b00010000; // Set C flag
        }
        registers.a = (registers.a << 1) | (carry >> 4);
        if registers.a == 0 {
            registers.f |= 0b10000000; // Set Z flag
        }
        if (registers.a & 0x0F) == 0x0F {
            registers.f |= 0b00100000; // Set H flag
        }
        self.increment_clock(1);
    }
    fn jr_s8(&mut self) {
        // JR s8
        let offset = self.fetch() as i8;
        self.pc = self.pc.wrapping_add(offset as u16);
        self.increment_clock(3);
    }

    fn ld_a_de(&mut self) {
        let addr = registers.de();
        registers.a = self.read_memory(addr);
        self.increment_clock(2);
    }

    fn rra(&mut self) {
        /*Rotate the contents of register A to the right, through the carry (CY) flag.
        That is, the contents of bit 7 are copied to bit 6, and the previous contents of bit 6 (before the copy) are copied to bit 5.
        The same operation is repeated in sequence for the rest of the register.
        The previous contents of the carry flag are copied to bit 7. */
        let carry = registers.f & 0b00010000;
        registers.f &= 0b00001111; // Clear all flags except for H
        if carry != 0 {
            registers.f |= 0b00010000; // Set C flag
        }
        registers.a = (registers.a >> 1) | (carry << 4);
        if registers.a == 0 {
            registers.f |= 0b10000000; // Set Z flag
        }
        if (registers.a & 0x0F) == 0x0F {
            registers.f |= 0b00100000; // Set H flag
        }
        self.increment_clock(1);
    }
    fn jr_nz_s8(&mut self) {
        // JR NZ, s8
        let offset = self.fetch() as i8;
        if (registers.f & 0b10000000) == 0 {
            self.pc = self.pc.wrapping_add(offset as u16);
        }
        self.increment_clock(3);
    }

    fn ld_hl_plus_a(&mut self) {
        // LD (HL+), A
        self.write_memory(registers.hl(), registers.a);
        registers.set_hl(registers.hl().wrapping_add(1));
        self.increment_clock(2);
    }


    fn daa(&mut self, registers: &mut Registers) {
        /* Adjust the accumulator (register A) to a binary-coded decimal (BCD) number
        after BCD addition and subtraction operations. */
        let mut offset = 0_u8;
        let a = registers.a;
        let half_carry = registers.is_flag_set(Flag::H);
        let carry = registers.is_flag_set(Flag::C);

        if a & 0xF > 0x09 || half_carry {
            offset |= 0x06; // Add 6 to the lower nibble
        }

        if a > 0x99 || carry {
            offset |= 0x60; // Add 60h to the upper nibble
        }

        registers.a = a.wrapping_add(offset);

        self.increment_clock(1);
    }
    fn execute(&mut self, opcode: u8, registers: &mut Registers) {
        match opcode {
            0x00 => self.nop(),
            0x01 => self.ld_rr_nn(registers, DoubleRegister::BC),
            0x02 => self.ld_bc_a(registers),
            0x03 => self.inc_rr(registers, DoubleRegister::BC),
            0x04 => self.inc_r(registers, SingleRegister::B),
            0x05 => self.dec_r(registers, &mut registers.b),
            0x06 => self.ld_r_n(&mut registers.b),
            0x07 => self.rlca(registers),
            0x08 => self.ld_a16_sp(),
            0x09 => self.add_hl_rr(registers, DoubleRegister::BC),
            0x0A => self.ld_a_bc(registers),
            0x0B => self.dec_rr(registers, DoubleRegister::BC),
            0x0C => self.inc_r(registers, SingleRegister::C),
            0x0D => self.dec_r(registers, &mut registers.c),
            0x0E => self.ld_r_n(&mut registers.c),
            0x0F => self.rrca(registers),

            0x10 => self.stop(),
            0x11 => self.ld_rr_nn(registers, DoubleRegister::DE),
            0x12 => self.ld_de_a(),
            0x13 => self.inc_rr(registers, DoubleRegister::DE),
            0x14 => self.inc_r(registers, SingleRegister::D),
            0x15 => self.dec_r(registers, &mut registers.d),
            0x16 => self.ld_r_n(&mut registers.d),
            0x17 => self.rla(registers),
            0x18 => self.jr_s8(),
            0x19 => self.add_hl_rr(registers, DoubleRegister::DE),
            0x1A => self.ld_a_de(),
            0x1B => self.dec_rr(registers, DoubleRegister::DE),
            0x1C => self.inc_r(registers, &mut registers.e),
            0x1D => self.dec_r(registers, &mut registers.e),
            0x1E => self.ld_r_n(&mut registers.e),
            0x1F => self.rra(),

            0x20 => self.jr_nz_s8(),
            0x21 => self.ld_rr_nn(registers, DoubleRegister::HL),
            0x22 => self.ld_hl_plus_a(),
            0x23 => self.inc_rr(registers, DoubleRegister::HL),
            0x24 => self.inc_r(registers, &mut registers.h),
            0x25 => self.dec_r(registers, &mut registers.h),
            0x26 => self.ld_r_n(&mut registers.h),
            0x27 => self.daa(registers),

            0x3E => {
                // LD A, imm8 (Load an 8-bit immediate value into A)
                let value = self.fetch();
                registers.a = value;
                self.increment_clock(4);
            }
            0x36 => {
                // LD (HL), n: Load from immediate data (indirect HL)
                let n = self.fetch();
                self.write_memory(registers.hl(), n);
                self.increment_clock(4);
            }
            0x41 => {
                // LD B, C
                registers.b = registers.c;
                self.increment_clock(1);
            }
            0x46 => {
                // LD B, (HL)
                let memory = self.read_memory(registers.hl());
                registers.b = memory;
                self.increment_clock(2);
            }
            0x70 => {
                // LD (HL), B
                self.write_memory(registers.hl(), registers.b);
                self.increment_clock(2);
            }
            0x80 => {
                // ADD A, B
                let (result, carry) = registers.a.overflowing_add(registers.b);
                let half_carry = ((registers.a & 0x0F) + (registers.b & 0x0F)) > 0x0F;

                registers.a = result;
                registers.f = 0; // Clear all flags
                if result == 0 {
                    registers.f |= 0b10000000;
                } // Set Z if result is 0
                if half_carry {
                    registers.f |= 0b00100000;
                } // Set H if half-carry occurs
                if carry {
                    registers.f |= 0b00010000;
                } // Set C if carry occurs

                self.increment_clock(1);
            }
            _ => panic!("Unknown opcode: {:#04x}", opcode),
        }
    }

    fn step(&mut self, registers: &mut Registers) {
        let opcode = self.fetch();
        self.execute(opcode, registers);

        // Sync with PPU (Graphics Processing)
        //self.ppu.step(cycles);
    }

    pub fn run(&mut self, registers: &mut Registers) {
        loop {
            self.step(registers);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_daa() {
        let mut registers = Registers::default();
        let mut cpu = Cpu::new();

        registers.a = 0x25; // Example value
        registers.set_flag_bit(Flag::H, 1); // Set half-carry flag
        registers.set_flag_bit(Flag::C, 1); // Set carry flag
        cpu.daa(&mut registers);
        assert_eq!(registers.a, 0x25 + 0x60); // Adjusted value
        assert_eq!(registers.get_flag_bit(Flag::H), 0); // Half-carry flag should be cleared
        assert_eq!(registers.get_flag_bit(Flag::C), 0); // Carry flag should be cleared
    }
}
