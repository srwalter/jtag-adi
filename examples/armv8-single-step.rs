use std::cell::RefCell;
use std::rc::Rc;
use std::ops::DerefMut;
use std::num::ParseIntError;
use std::time::Instant;

use jtag_taps::cable::{self, Cable};
use jtag_taps::statemachine::JtagSM;
use jtag_taps::taps::Taps;

use jtag_adi::{ArmDebugInterface, MemAP};

use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long)]
    cable: String,
    #[arg(short, long)]
    baud: u32,
    #[arg(short, long, default_value_t = 0)]
    /// Which JTAG TAP to use
    tap_index: usize,
    #[arg(short, long, default_value_t = 0)]
    /// Which access port to use
    ap_num: u32,
    #[arg(long)]
    cpu_base: String,
    #[arg(long)]
    cti_base: String,
    command: Option<String>,
}

fn cpu_halt<T,U>(mem: &mut MemAP<T>, cti_base: u32)
    where T: DerefMut<Target=U>,
          U: Cable + ?Sized
{
    // Gate all
    mem.write(cti_base + 0x140, 0).expect("write ctigate");

    // Enable CTIOUTEN for channel 0
    mem.write(cti_base + 0x0a0, 1).expect("write ctiouten");

    // Generate HALT to core 0
    mem.write(cti_base + 0x01c, 1).expect("write ctiouten");

    // ACK the halt
    mem.write(cti_base + 0x010, 3).expect("write ctiouten");
    // Wait for ACK
    while mem.read(cti_base + 0x134).unwrap() != 0 {}
}

fn parse_int(x: &str) -> Result<u32, ParseIntError> {
    if x.starts_with("0x") {
        let len = x.len();
        u32::from_str_radix(&x[2..len], 16)
    } else {
        str::parse(&x)
    }
}

fn main() {
    let args = Args::parse();
    let cable = cable::new_from_string(&args.cable, args.baud).expect("cable");
    let jtag = JtagSM::new(cable);
    let mut taps = Taps::new(jtag);
    taps.detect();

    // IDCODE instruction
    let ir = vec![14];
    taps.select_tap(0, &ir);
    let dr = taps.read_dr(32);
    let idcode = u32::from_le_bytes(dr.try_into().unwrap());
    assert_eq!(idcode, 0x6ba00477);

    let adi = Rc::new(RefCell::new(ArmDebugInterface::new(taps)));
    let mut mem = MemAP::new(adi.clone(), args.ap_num);

    let cpu_base = parse_int(&args.cpu_base).expect("invalid cpu base");
    let edprsr = mem.read(cpu_base + 0x314).expect("read edprsr");
    //println!("edprsr {:x}", edprsr);
    assert!(edprsr & 1 == 1);

    // Clear OS lock
    mem.write(cpu_base + 0x300, 0).expect("write oslar");

    // Clear software lock lock
    mem.write(cpu_base + 0xfb0, 0xC5ACCE55).expect("write oslar");
    let oslar = mem.read(cpu_base + 0xfb4).expect("read oslar");
    //println!("swlck {:x}", oslar);
    assert_eq!(oslar & 2, 0);

    // Enable halting debug
    let mut edscr = mem.read(cpu_base + 0x088).expect("read edscr");
    //println!("edscr {:x}", edscr);
    edscr |= 1 << 14;
    mem.write(cpu_base + 0x088, edscr).expect("write edscr");

    //// Unlock CTI
    let cti_base = parse_int(&args.cti_base).expect("invalid cti base");
    mem.write(cti_base + 0xfb0, 0xC5ACCE55).expect("write cti");

    //// Enable CTI
    let mut cti = mem.read(cti_base).expect("read cti");
    //println!("cti {:x}", cti);
    cti |= 1;
    mem.write(cti_base, cti).expect("write cti");
    let cti = mem.read(cti_base).expect("read cti");
    //println!("cti {:x}", cti);
    assert_eq!(cti & 1, 1);

    let eddevid = mem.read(cpu_base + 0xfc8).expect("read edscr");
    if eddevid & 7 == 0 {
        eprintln!("CPU must support EDPCSR!");
        return;
    }

    // Must be in halt state
    cpu_halt(&mut mem, cti_base);
    // enable single step
    mem.write(cpu_base + 0x024, 1 << 2).expect("write edecr");

    let start = Instant::now();
    let mut count = 0;
    loop {
        mem.queue_read(cpu_base + 0x0ac).expect("read edpcsr");
        mem.queue_read(cpu_base + 0x0a0).expect("read edpcsr");
        let pc_hi = mem.finish_read().expect("read edpcsr");
        let pc_lo = mem.finish_read().expect("read edpcsr");
        println!("pc {:x}{:x}", pc_hi, pc_lo);
        count += 1;
        if count % 1000 == 0 {
            let delta = start.elapsed().as_millis();
            eprintln!("IPS {}", count * 1000 / delta);
        }

        // resume the CPU so it can run one instruction
        mem.write_nocheck(cti_base + 0x140, 0).expect("write ctigate");
        mem.write_nocheck(cti_base + 0x0a4, 2).expect("write ctiouten");
        mem.write_nocheck(cti_base + 0x01c, 2).expect("write ctiouten");
        mem.write_nocheck(cti_base + 0x010, 3).expect("write ctiouten");
    }
}
