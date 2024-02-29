use std::cell::RefCell;
use std::rc::Rc;
use std::ops::DerefMut;
use std::num::ParseIntError;

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

fn cpu_resume<T,U>(mem: &mut MemAP<T>, cti_base: u32)
    where T: DerefMut<Target=U>,
          U: Cable + ?Sized
{
    // Gate all
    mem.write(cti_base + 0x140, 0).expect("write ctigate");

    // Enable CTIOUTEN for channel 1
    mem.write(cti_base + 0x0a4, 2).expect("write ctiouten");

    // Generate resume to core 0
    mem.write(cti_base + 0x01c, 2).expect("write ctiouten");

    // ACK the resume
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
    let mut mem = MemAP::new(adi.clone(), 0);

    let cpu_base = parse_int(&args.cpu_base).expect("invalid cpu base");
    let edprsr = mem.read(cpu_base + 0x314).expect("read edprsr");
    println!("edprsr {:x}", edprsr);
    assert!(edprsr & 1 == 1);

    // Clear OS lock
    let oslar = mem.read(cpu_base + 0x300).expect("read oslar");
    println!("oslar {:x}", oslar);
    mem.write(cpu_base + 0x300, 0).expect("write oslar");

    // Clear software lock lock
    let oslar = mem.read(cpu_base + 0xfb4).expect("read oslar");
    println!("swlck {:x}", oslar);
    mem.write(cpu_base + 0xfb0, 0xC5ACCE55).expect("write oslar");
    let oslar = mem.read(cpu_base + 0xfb4).expect("read oslar");
    println!("swlck {:x}", oslar);
    assert_eq!(oslar & 2, 0);

    // Enable halting debug
    let mut edscr = mem.read(cpu_base + 0x088).expect("read edscr");
    println!("edscr {:x}", edscr);
    edscr |= 1 << 14;
    mem.write(cpu_base + 0x088, edscr).expect("write edscr");
    let edscr = mem.read(cpu_base + 0x088).expect("read edscr");
    println!("edscr {:x}", edscr);

    //// Unlock CTI
    let cti_base = parse_int(&args.cti_base).expect("invalid cti base");
    let ctilsr = mem.read(cti_base + 0xfb4).expect("read cti");
    println!("ctilsr {:x}", ctilsr);
    mem.write(cti_base + 0xfb0, 0xC5ACCE55).expect("write cti");
    let ctilsr = mem.read(cti_base + 0xfb4).expect("read cti");
    println!("ctilsr {:x}", ctilsr);

    //// Enable CTI
    let mut cti = mem.read(cti_base).expect("read cti");
    println!("cti {:x}", cti);
    cti |= 1;
    mem.write(cti_base, cti).expect("write cti");
    let cti = mem.read(cti_base).expect("read cti");
    println!("cti {:x}", cti);
    assert_eq!(cti & 1, 1);

    if let Some(cmd) = args.command {
        match cmd.as_str() {
            "halt" => cpu_halt(&mut mem, cti_base),
            "resume" => cpu_resume(&mut mem, cti_base),
            _ => eprintln!("Unknown command"),
        }
    }

    let edscr = mem.read(cpu_base + 0x088).expect("read edscr");
    println!("edscr {:x}", edscr);
}
