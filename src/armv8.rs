use std::ops::DerefMut;

use jtag_taps::cable::Cable;

use crate::MemAP;

pub struct ARMv8<T> {
    mem: MemAP<T>,
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

    pub fn cpu_halt(&mut self)
    {
        // Gate all
        self.mem.write(self.cti_base + 0x140, 0).expect("write ctigate");

        // Enable CTIOUTEN for channel 0
        self.mem.write(self.cti_base + 0x0a0, 1).expect("write ctiouten");

        // Generate HALT to core 0
        self.mem.write(self.cti_base + 0x01c, 1).expect("write ctiouten");

        // ACK the halt
        self.mem.write(self.cti_base + 0x010, 3).expect("write ctiouten");
        // Wait for ACK
        while self.mem.read(self.cti_base + 0x134).unwrap() != 0 {}
    }

    pub fn cpu_resume(&mut self)
    {
        // Gate all
        self.mem.write(self.cti_base + 0x140, 0).expect("write ctigate");

        // Enable CTIOUTEN for channel 1
        self.mem.write(self.cti_base + 0x0a4, 2).expect("write ctiouten");

        // Generate resume to core 0
        self.mem.write(self.cti_base + 0x01c, 2).expect("write ctiouten");

        // ACK the resume
        self.mem.write(self.cti_base + 0x010, 3).expect("write ctiouten");
        // Wait for ACK
        while self.mem.read(self.cti_base + 0x134).unwrap() != 0 {}
    }

    fn assert_no_error(&mut self)
    {
        let edscr = self.mem.read(self.cpu_base + 0x088).expect("read edscr");
        //println!("edscr {:x}", edscr);
        //println!("txu {:x}", edscr & (1 << 26));
        assert_eq!(edscr & (1 << 6), 0);
    }

    pub fn get_reg(&mut self, reg: u32) -> u64
    {
        self.mem.write(self.cpu_base + 0x84, 0xd5130400 | reg).expect("write EDITR");
        self.assert_no_error();
        let orig_x0 = self.mem.read_block(self.cpu_base + 0x80, 8, false).expect("multi");
        let orig_x0_hi = orig_x0[0] as u64;
        let orig_x0_lo = orig_x0[3] as u64;
        self.assert_no_error();

        orig_x0_hi << 32 | orig_x0_lo
    }

    pub fn set_reg(&mut self, reg: u32, val: u64)
    {
        self.mem.write(self.cpu_base + 0x080, val as u32).expect("read edscr");
        self.assert_no_error();
        self.mem.write(self.cpu_base + 0x08c, (val >> 32) as u32).expect("read edscr");
        //// This seems to be needed on some A53 cores?
        self.mem.write(self.cpu_base + 0x090, 1 << 2).expect("write edrcr");
        // mrs x0, dbgdtr_el0
        let _ = self.mem.write(self.cpu_base + 0x84, 0xd5330400 | reg).expect("write EDITR");
        self.assert_no_error();
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
