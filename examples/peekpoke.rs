use std::rc::Rc;
use std::cell::RefCell;
use std::num::ParseIntError;

use clap::Parser;

use jtag_taps::cable;
use jtag_taps::statemachine::JtagSM;
use jtag_taps::taps::Taps;

use jtag_adi::{ArmDebugInterface, MemAP};

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
    addr: String,
    #[arg(long)]
    write: Option<String>,
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
    taps.select_tap(args.tap_index, &ir);
    let dr = taps.read_dr(32);
    let idcode = u32::from_le_bytes(dr.try_into().unwrap());

    // Verify ARM ID code
    if idcode != 0x4ba00477 {
        eprintln!("Warning: unexpected idcode {:x}", idcode);
    }

    let adi = Rc::new(RefCell::new(ArmDebugInterface::new(taps)));
    let mut mem = MemAP::new(adi.clone(), args.ap_num);

    let addr = parse_int(&args.addr).expect("failed to parse address");

    if let Some(value) = args.write {
        let value = parse_int(&value).expect("failed to parse value");
        mem.write(addr, value).expect("write");
        println!("Success");
    } else {
        let val = mem.read(addr).expect("read");
        println!("0x{:x} = 0x{:x}", addr, val);
    }
}
