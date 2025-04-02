use gameboy_emulator_rs::Cpu;
use gameboy_emulator_rs::cpu::Registers;
use std::fs;

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

    let mut registers = Registers::new();
    let mut cpu = Cpu::new();

    let rom_size = rom.len().min(0x8000);
    cpu.memory[..rom_size].copy_from_slice(&rom[..rom_size]);

    cpu.run(&mut registers);
}
