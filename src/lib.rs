//! This crate allows for interacting with ARM Debug Interface components over JTAG, such as the
//! Mem AP for accessing memory-mapped resources.  It uses the jtag-taps library for the link layer
//! and so supports all cables supported by that crate.

use std::cell::RefCell;
use std::ops::DerefMut;
use std::rc::Rc;

use jtag_taps::cable::Cable;
use jtag_taps::taps::Taps;

/// Selects between Debug Port (DP) and Access Port (AP)
pub enum Port {
    DP = 10,
    AP = 11,
}

/// Debug Port registers
pub enum DPReg {
    Abort = 0,
    CtrlStat = 1,
    Select = 2,
    Rdbuff = 3,
}

pub struct ArmDebugInterface<T> {
    taps: Taps<T>,
    lastbank: u32,
    lastir: Vec<u8>,
}

impl<T, U> ArmDebugInterface<T>
where
    T: DerefMut<Target = U>,
    U: Cable + ?Sized,
{
    pub fn new(taps: Taps<T>) -> Self {
        let mut adi = Self {
            taps,
            lastbank: 0xff,
            lastir: vec![],
        };

        // Force bank selects to known values
        adi.bank_select(0, 0, 0);

        // Abort any in-progress transactions
        adi.write_adi_nobank(Port::DP, DPReg::Abort as u8, 0, true).expect("abort");

        // Make sure everything is powered up and STICKY errors are cleared
        adi.write_adi_nobank(
            Port::DP,
            DPReg::CtrlStat as u8,
            1 << 30 | 1 << 28 | 1 << 24 | 1 << 5 | 1 << 1,
            true,
        )
        .expect("clear errors");

        adi
    }

    fn write_ir(&mut self, ir: &[u8]) {
        if self.lastir != ir {
            self.taps.write_ir(ir);
            self.lastir = ir.to_vec();
        }
    }

    fn parse_ack(mut dr: Vec<u8>) -> Result<u32, u8> {
        dr.push(0);
        dr.push(0);
        dr.push(0);
        let val = u64::from_le_bytes(dr.try_into().unwrap());
        let val = val & ((1 << 35) - 1);

        let ack = val & 7;
        if ack != 2 {
            return Err(ack as u8);
        }

        Ok((val >> 3) as u32)
    }

    pub fn queue_read_adi_nobank(&mut self, port: Port, reg: u8) -> bool {
        let ir = [port as u8];
        self.write_ir(&ir);
        let buf = [(reg << 1) | 1, 0, 0, 0, 0];
        self.taps.write_dr(&buf, 3);
        self.taps.queue_dr_read(35)
    }

    pub fn finish_read(&mut self) -> Result<u32, u8> {
        let mut dr = self.taps.finish_dr_read(35);

        dr.push(0);
        dr.push(0);
        dr.push(0);
        let val = u64::from_le_bytes(dr.try_into().unwrap());
        let val = val & ((1 << 35) - 1);

        let ack = val & 7;
        if ack != 2 {
            return Err(ack as u8);
        }

        let val = (val >> 3) as u32;
        Ok(val)
    }

    /// Read register `reg` from `port`.  This function assumes that the correct bank is already
    /// selected.  You probably want `read_adi` unless you know what you're doing.
    pub fn read_adi_nobank(&mut self, port: Port, reg: u8) -> Result<u32, u8> {
        let result = self.queue_read_adi_nobank(port, reg);
        assert!(result);
        self.finish_read()
    }

    /// Write `val` to register `reg` on `port`.  This function assumes that the correct bank is already
    /// selected.  If `check` is true then the return code of the write will be verified, however
    /// this comes at a performance penalty. You probably want `write_adi` unless you know what
    /// you're doing.
    pub fn write_adi_nobank(
        &mut self,
        port: Port,
        reg: u8,
        val: u32,
        check: bool,
    ) -> Result<(), u8> {
        let ir = [port as u8];

        let mut val = val as u64;
        val <<= 3;
        val |= (reg << 1) as u64;

        let bytes = val.to_le_bytes();
        loop {
            self.write_ir(&ir);
            self.taps.write_dr(&bytes[0..5], 3);
            if !check {
                return Ok(());
            } else {
                let mut dr = self.taps.read_dr(35);

                dr.push(0);
                dr.push(0);
                dr.push(0);
                let val = u64::from_le_bytes(dr.try_into().unwrap());
                let val = val & ((1 << 35) - 1);

                let ack = val & 7;
                if ack == 2 {
                    return Ok(());
                }
                if ack == 1 {
                    continue;
                }
                return Err(ack as u8);
            }
        }
    }

    /// Select the given access port and banks on the access port and debug port.
    pub fn bank_select(&mut self, apsel: u32, apbank: u32, dpbank: u32) {
        let val = (apsel << 24) | (apbank << 4) | dpbank;
        if val != self.lastbank {
            self.write_adi_nobank(Port::DP, DPReg::Select as u8, val, true)
                .expect("bank sel");
            self.lastbank = val;
        }
    }

    /// Read register `reg` from AP `apsel` and `port`.
    pub fn read_adi(&mut self, apsel: u32, port: Port, mut reg: u8) -> Result<u32, u8> {
        let bank = reg >> 2;
        reg &= 3;
        self.bank_select(apsel, bank as u32, 0);
        self.read_adi_nobank(port, reg)
    }

    /// Read register `reg` from AP `apsel` and `port`.
    pub fn queue_read_adi(&mut self, apsel: u32, port: Port, mut reg: u8) -> bool {
        let bank = reg >> 2;
        reg &= 3;
        self.bank_select(apsel, bank as u32, 0);
        self.queue_read_adi_nobank(port, reg)
    }

    /// Write `val` to register `reg` of AP `apsel` and `port`.
    pub fn write_adi(&mut self, apsel: u32, port: Port, mut reg: u8, val: u32) -> Result<(), u8> {
        let bank = reg >> 2;
        reg &= 3;
        self.bank_select(apsel, bank as u32, bank as u32);
        self.write_adi_nobank(port, reg, val, true)
    }

    /// Write `val` to register `reg` of AP `apsel` and `port` without checking for success.  This
    /// is slightly faster than `write_adi`, especially when doing a sequence of writes.
    pub fn write_adi_nocheck(
        &mut self,
        apsel: u32,
        port: Port,
        mut reg: u8,
        val: u32,
    ) -> Result<(), u8> {
        let bank = reg >> 2;
        reg &= 3;
        self.bank_select(apsel, bank as u32, bank as u32);
        self.write_adi_nobank(port, reg, val, false)
    }

    /// Read multiple registers.  `reg` is an array of register values to access.  The result is
    /// returned in the corresponding index of the returned Vec.  This function makes more
    /// efficient use of the JTAG bus when there are multiple reads to perform.
    pub fn read_adi_pipelined(
        &mut self,
        apsel: u32,
        port: Port,
        reg: &[u8],
    ) -> Vec<Result<u32, u8>> {
        let bank = reg[0] >> 2;
        self.bank_select(apsel, bank as u32, 0);

        let ir = [port as u8];
        self.write_ir(&ir);
        let buf = [((reg[0] & 3) << 1) | 1, 0, 0, 0, 0];
        self.taps.write_dr(&buf, 3);

        let mut count = 0;
        let mut queue_full = false;
        for r in &reg[1..] {
            // Make sure all registers are in the same bank
            assert_eq!(r >> 2, reg[0] >> 2);
            let buf = [((r & 3) << 1) | 1, 0, 0, 0, 0];
            if !self.taps.queue_dr_read_write(&buf, 3) {
                queue_full = true;
                break;
            }
            count += 1;
        }

        if !queue_full {
            if self.taps.queue_dr_read(35) {
                count += 1;
            }
        }

        let mut data = vec![];
        for _ in 0..count {
            data.push(Self::parse_ack(self.taps.finish_dr_read(35)));
        }

        data
    }

    /// Write multiple registers.  Each item of `reg` is a tuple consisting of the register address
    /// and the value to write.  This function makes more efficient use of the JTAG bus when there
    /// are multiple reads to perform.
    pub fn write_adi_pipelined(
        &mut self,
        apsel: u32,
        port: Port,
        reg: &[(u8, u32)],
    ) -> Result<(), u8> {
        let bank = reg[0].0 >> 2;
        self.bank_select(apsel, bank as u32, 0);

        let ir = [port as u8];
        self.write_ir(&ir);

        for (r, val) in reg {
            // Make sure all registers are in the same bank
            assert_eq!(r >> 2, reg[0].0 >> 2);

            let mut val = *val as u64;
            val <<= 3;
            val |= ((r & 3) << 1) as u64;

            let bytes = val.to_le_bytes();
            self.taps.write_dr(&bytes[0..5], 3);
        }
        Ok(())
    }
}

#[allow(clippy::upper_case_acronyms)]
enum MemAPReg {
    CSW = 0,
    TAR = 1,
    DRW = 3,
    //Base0 = 0xf0 >> 2,
    //CFG = 0xf4 >> 2,
    //Base1 = 0xf8 >> 2,
    //IDR = 0xfc >> 2,
}

/// Functions for interacting with a Memory Access Port
pub struct MemAP<T> {
    adi: Rc<RefCell<ArmDebugInterface<T>>>,
    apsel: u32,
    csw: u32,
    tar: u32,
}

impl<T, U> MemAP<T>
where
    T: DerefMut<Target = U>,
    U: Cable + ?Sized,
{
    pub fn new(adi: Rc<RefCell<ArmDebugInterface<T>>>, apsel: u32) -> Self {
        let csw = adi
            .borrow_mut()
            .read_adi(apsel, Port::AP, MemAPReg::CSW as u8)
            .expect("read csw");
        let tar = adi
            .borrow_mut()
            .read_adi(apsel, Port::AP, MemAPReg::TAR as u8)
            .expect("read tar");
        Self { adi, apsel, csw, tar }
    }

    /// Set the control and status word of the MemAP.  `MemAP` caches the value of this register,
    /// so it should not be modified other than by this function.
    pub fn write_csw(&mut self, csw: u32) -> Result<(), u8> {
        if csw != self.csw {
            self.adi
                .borrow_mut()
                .write_adi(self.apsel, Port::AP, MemAPReg::CSW as u8, csw)?;
            self.csw = csw;
        }
        Ok(())
    }

    /// Read a single 32-bit quantity from `addr`
    pub fn read(&mut self, addr: u32) -> Result<u32, u8> {
        // Make sure we're not in auto-increment mode
        self.write_csw(self.csw & !(1 << 4))?;
        if self.tar != addr {
            self.adi
                .borrow_mut()
                .write_adi(self.apsel, Port::AP, MemAPReg::TAR as u8, addr)?;
            self.tar = addr;
        }
        let val = self
            .adi
            .borrow_mut()
            .read_adi(self.apsel, Port::AP, MemAPReg::DRW as u8)?;
        let stat = self
            .adi
            .borrow_mut()
            .read_adi(self.apsel, Port::DP, DPReg::CtrlStat as u8)?;
        if stat & 5 != 0 {
            return Err(5);
        }
        Ok(val)
    }

    pub fn queue_read(&mut self, addr: u32) -> Result<bool, u8> {
        // Make sure we're not in auto-increment mode
        self.write_csw(self.csw & !(1 << 4))?;
        if self.tar != addr {
            self.adi
                .borrow_mut()
                .write_adi_nocheck(self.apsel, Port::AP, MemAPReg::TAR as u8, addr)?;
            self.tar = addr;
        }

        let val = self
            .adi
            .borrow_mut()
            .queue_read_adi(self.apsel, Port::AP, MemAPReg::DRW as u8);
        if !val {
            return Ok(false);
        }
        Ok(true)
    }

    pub fn finish_read(&mut self) -> Result<u32, u8> {
        let val = self.adi.borrow_mut().finish_read()?;
        Ok(val)
    }

    /// Write `value` to `addr`
    pub fn write(&mut self, addr: u32, value: u32) -> Result<(), u8> {
        // Make sure we're not in auto-increment mode
        self.write_csw(self.csw & !(1 << 4))?;
        if self.tar != addr {
            self.adi
                .borrow_mut()
                .write_adi(self.apsel, Port::AP, MemAPReg::TAR as u8, addr)?;
            self.tar = addr;
        }
        self.adi
            .borrow_mut()
            .write_adi(self.apsel, Port::AP, MemAPReg::DRW as u8, value)?;
        let stat = self
            .adi
            .borrow_mut()
            .read_adi(self.apsel, Port::DP, DPReg::CtrlStat as u8)?;
        if stat & 5 != 0 {
            return Err(5);
        }
        Ok(())
    }

    /// Read multiple values from memory.  If `check_status` is true, then the CTRL/STAT
    /// register is checked for errors at the end of the transaction, which comes with a slight
    /// performance penalty.  If `auto_increment` is true, then each value will come from the next
    /// sequential address, otherwise every read is from `addr`
    pub fn read_multi(
        &mut self,
        addr: u32,
        count: usize,
        auto_increment: bool,
        check_status: bool,
    ) -> Result<Vec<u32>, u8> {
        // Enable auto-increment mode
        if auto_increment {
            self.write_csw(self.csw | (1 << 4))?;
        } else {
            self.write_csw(self.csw & !(1 << 4))?;
        }

        if self.tar != addr {
            self.adi
                .borrow_mut()
                .write_adi(self.apsel, Port::AP, MemAPReg::TAR as u8, addr)?;
            self.tar = addr;
            if auto_increment {
                self.tar += 4 * count as u32;
            }
        }

        let reg = vec![MemAPReg::DRW as u8; count];
        let val = self
            .adi
            .borrow_mut()
            .read_adi_pipelined(self.apsel, Port::AP, &reg);

        // Since we are always reading from the same register, any WAIT acks can be dropped
        let mut result = vec![];
        for item in val {
            match item {
                Ok(x) => result.push(x),
                Err(1) => continue,
                Err(e) => return Err(e),
            }
        }

        if check_status {
            let stat =
                self.adi
                    .borrow_mut()
                    .read_adi(self.apsel, Port::DP, DPReg::CtrlStat as u8)?;
            if stat & 5 != 0 {
                return Err(5);
            }
        }
        Ok(result)
    }

    /// Read multiple consective values from memory.  If `check_status` is true, then the CTRL/STAT
    /// register is checked for errors at the end of the transaction, which comes with a slight
    /// performance penalty.
    pub fn read_block(
        &mut self,
        addr: u32,
        count: usize,
        check_status: bool,
    ) -> Result<Vec<u32>, u8> {
        self.read_multi(addr, count, true, check_status)
    }


    /// Write `data` starting at `addr`.  If `check_status` is true, then the CTRL/STAT
    /// register is checked for errors at the end of the transaction, which comes with a slight
    /// performance penalty.
    pub fn write_block(&mut self, addr: u32, data: &[u32], check_status: bool) -> Result<(), u8> {
        // Enable auto-increment mode
        self.write_csw(self.csw | (1 << 4))?;

        if self.tar != addr {
            self.adi
                .borrow_mut()
                .write_adi(self.apsel, Port::AP, MemAPReg::TAR as u8, addr)?;
            self.tar = addr + 4 * data.len() as u32;
        }

        let reg: Vec<(u8, u32)> = data.iter().map(|x| (MemAPReg::DRW as u8, *x)).collect();
        self.adi
            .borrow_mut()
            .write_adi_pipelined(self.apsel, Port::AP, &reg)?;

        if check_status {
            let stat =
                self.adi
                    .borrow_mut()
                    .read_adi(self.apsel, Port::DP, DPReg::CtrlStat as u8)?;
            if stat & 5 != 0 {
                return Err(5);
            }
        }
        Ok(())
    }
}
