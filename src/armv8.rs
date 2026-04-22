use std::ops::DerefMut;

use jtag_taps::cable::Cable;

use crate::MemAP;

#[derive(Debug,Clone,Copy)]
pub enum DataSize {
    Byte = 1,
    Half = 2,
    Word = 4,
    Double = 8,
}

pub struct ARMv8<T> {
    pub mem: MemAP<T>,
    cpu_base: u32,
    cti_base: u32,
}

impl<T, U> ARMv8<T>
where
    T: DerefMut<Target = U>,
    U: Cable + ?Sized,
{
    pub fn new(mem: MemAP<T>, cpu_base: u32, cti_base: u32) -> Self {
        Self {
            mem,
            cpu_base,
            cti_base,
        }
    }

    pub fn cpu_halt(&mut self) -> Result<(), u8>
    {
        // Gate all
        self.mem.write(self.cti_base + 0x140, 0)?;

        // Enable CTIOUTEN for channel 0
        self.mem.write(self.cti_base + 0x0a0, 1)?;

        // Generate HALT to core 0
        self.mem.write(self.cti_base + 0x01c, 1)?;

        // ACK the halt
        self.mem.write(self.cti_base + 0x010, 3)?;
        // Wait for ACK
        while self.mem.read(self.cti_base + 0x134)? != 0 {}
        Ok(())
    }

    pub fn cpu_resume(&mut self) -> Result<(), u8>
    {
        // Gate all
        self.mem.write(self.cti_base + 0x140, 0)?;

        // Enable CTIOUTEN for channel 1
        self.mem.write(self.cti_base + 0x0a4, 2)?;

        // Generate resume to core 0
        self.mem.write(self.cti_base + 0x01c, 2)?;

        // ACK the resume
        self.mem.write(self.cti_base + 0x010, 3)?;
        // Wait for ACK
        while self.mem.read(self.cti_base + 0x134)? != 0 {}
        Ok(())
    }

    fn check_err(&mut self) -> Result<(), u8>
    {
        if let Ok(_) = std::env::var("YOLO_MODE") {
            return Ok(())
        }
        let edscr = self.mem.read(self.cpu_base + 0x088)?;
        //println!("edscr {:x}", edscr);
        //println!("txu {:x}", edscr & (1 << 26));
        if edscr & (1 << 6) != 0 {
            Err(0)
        } else {
            Ok(())
        }
    }

    pub fn run_instr(&mut self, instr: u32) -> Result<(), u8>
    {
        self.mem.write(self.cpu_base + 0x84, instr)?;
        self.check_err()
    }

    pub fn get_reg(&mut self, reg: u32) -> Result<u64, u8>
    {
        self.run_instr(0xd5130400 | reg)?;
        let orig_x0 = self.mem.read_block(self.cpu_base + 0x80, 8, false)?;
        let orig_x0_hi = orig_x0[0] as u64;
        let orig_x0_lo = orig_x0[3] as u64;
        self.check_err()?;

        Ok(orig_x0_hi << 32 | orig_x0_lo)
    }

    pub fn set_reg(&mut self, reg: u32, val: u64) -> Result<(), u8>
    {
        self.mem.write(self.cpu_base + 0x080, val as u32)?;
        self.check_err()?;
        self.mem.write(self.cpu_base + 0x08c, (val >> 32) as u32)?;
        //// This seems to be needed on some A53 cores?
        self.mem.write(self.cpu_base + 0x090, 1 << 2).expect("write edrcr");
        // mrs x0, dbgdtr_el0
        self.run_instr(0xd5330400 | reg)
    }

    pub fn read_mem(&mut self, addr: u64, size: DataSize) -> Result<u64, u8>
    {
        // Uses ARMv8 ldur instructions to read memory via JTAG
        let orig_x0 = self.get_reg(0)?;

        self.set_reg(0, addr)?;
        match size {
            // ldur x0, [x0]
            DataSize::Double => self.run_instr(0xf8400000)?,
            // ldur w0, [x0]
            DataSize::Word   => self.run_instr(0xb8400000)?,
            // ldurh w0, [x0]
            DataSize::Half   => self.run_instr(0x78400000)?,
            // ldurb w0, [x0]
            DataSize::Byte   => self.run_instr(0x38400000)?,
        }
        let val = self.get_reg(0)?;

        self.set_reg(0, orig_x0)?;
        Ok(val)
    }

    pub fn write_mem(&mut self, addr: u64, val: u64, size: DataSize) -> Result<(), u8>
    {
        // Uses ARMv8 stur instructions to write memory via JTAG
        let orig_x0 = self.get_reg(0)?;
        let orig_x1 = self.get_reg(1)?;

        self.set_reg(0, addr)?;
        self.set_reg(1, val)?;
        match size {
            DataSize::Double => {
                // stur x1, [x0]
                self.run_instr(0xf8000001)?;
            }
            DataSize::Word => {
                // stur w1, [x0]
                self.run_instr(0xb8000001)?;
            }
            DataSize::Half => {
                // sturh w1, [x0]
                self.run_instr(0x78000001)?;
            }
            DataSize::Byte => {
                // sturb w1, [x0]
                self.run_instr(0x38000001)?;
            }
        }

        self.set_reg(0, orig_x0)?;
        self.set_reg(1, orig_x1)
    }

    pub fn read_cpu(&mut self, offset: u32) -> Result<u32, u8>
    {
        self.mem.read(self.cpu_base + offset)
    }

    pub fn write_cpu(&mut self, offset: u32, value: u32) -> Result<(), u8>
    {
        self.mem.write(self.cpu_base + offset, value)
    }
}
