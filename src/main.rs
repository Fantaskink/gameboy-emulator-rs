use std::fs;

#[derive(Debug)]
struct Cpu {
    registers: Registers,
    pc: u16,
    sp: u16,
    memory: [u8; 0x10000], // 64KB memory
    cycles: u64,           // T-cycles
}

impl Cpu {
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

    fn execute(&mut self, opcode: u8) {
        match opcode {
            0x00 => {
                /* NOP (Do nothing) */
                self.increment_clock(1);
            }
            0x3E => {
                // LD A, imm8 (Load an 8-bit immediate value into A)
                let value = self.fetch();
                self.registers.a = value;
                self.increment_clock(4);
            }
            0x06 => {
                // LD B, imm8 (Load an 8-bit immediate into B)
                let value = self.fetch();
                self.registers.b = value;
                self.increment_clock(2);
            }
            0x0A => {
                // LD A, (BC): Load accumulator (indirect BC)
                let addr = self.registers.bc();
                self.registers.a = self.read_memory(addr);
                self.increment_clock(2);
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
                // ADD A, B (Add B to A)
                let (result, carry) = self.registers.a.overflowing_add(self.registers.b);
                self.registers.a = result;
                self.registers.f = (carry as u8) << 4; // Set carry flag
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
