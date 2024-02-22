use std::cell::RefCell;
use std::rc::Rc;
use std::ops::DerefMut;
use std::num::ParseIntError;

use clap::Parser;

use jtag_taps::taps::Taps;
use jtag_taps::statemachine::JtagSM;
use jtag_taps::cable::{self, Cable};

use jtag_adi::{ArmDebugInterface, MemAP};

fn trace_sink_to_str(devtype: u32) -> &'static str {
    match devtype >> 4{
        1 => "TPIU",
        2 => "ETB",
        3 => "Router",
        _ => "Other",
    }
}

fn trace_link_to_str(devtype: u32) -> &'static str {
    match devtype >> 4 {
        1 => "Router",
        2 => "Filter",
        3 => "FIFO",
        _ => "Other",
    }
}

fn trace_source_to_str(devtype: u32) -> &'static str {
    match devtype >> 4 {
        1 => "CPU",
        2 => "DSP",
        3 => "Coprocessor",
        4 => "Bus",
        _ => "Other",
    }
}

fn debug_control_to_str(devtype: u32) -> &'static str {
    match devtype >> 4 {
        1 => "Trigger Matrix",
        2 => "Debug Authentication",
        3 => "Power Requestor",
        _ => "Other",
    }
}

fn debug_logic_to_str(devtype: u32) -> &'static str {
    match devtype >> 4 {
        1 => "CPU",
        2 => "DSP",
        3 => "Coprocessor",
        4 => "BUS",
        5 => "Memory",
        _ => "Other",
    }
}

fn devtype_to_str(devtype: u32) -> String {
    match devtype & 0xf {
        0 => format!("Misc"), 
        1 => format!("Trace sink: {}", trace_sink_to_str(devtype)),
        2 => format!("Trace link: {}", trace_link_to_str(devtype)),
        3 => format!("Trace source: {}", trace_source_to_str(devtype)),
        4 => format!("Debug control: {}", debug_control_to_str(devtype)),
        5 => format!("Debug logic: {}", debug_logic_to_str(devtype)),
        _ => "Other".to_string()
    }
}

fn parse_rom_table<T,U>(mem: &mut MemAP<T>, base: u32) -> Result<(), u8>
    where T: DerefMut<Target=U>,
          U: Cable + ?Sized,
{
    let _cidr0 = mem.read(base + 0xff0)?;
    //assert_eq!(cidr0, 0xd);
    let cidr1 = mem.read(base + 0xff4)?;
    let _cidr2 = mem.read(base + 0xff8)?;
    //assert_eq!(cidr2, 0x5);
    let _cidr3 = mem.read(base + 0xffc)?;
    //assert_eq!(cidr3, 0xb1);

    match cidr1 {
        0x10 => {
            println!("Found ROM table at {:x}", base);

            for i in 0..960 {
                let romentry = mem.read(base + i * 4)?;
                if romentry == 0 {
                    break;
                }

                if romentry & 1 != 0 {
                    println!("Entry {} present", i);
                    if romentry & (1 << 2) != 0 {
                        println!("    PD valid");
                    }
                    let offset = romentry >> 12;
                    println!("    Offset {}", offset);
                    parse_rom_table(mem, base + (offset << 12)).expect("parse sub table");
                }
            }
        }
        0x90 => {
            println!("Found CoreSight component at {:x}", base);
            let auth = mem.read(base + 0xfb8)?;
            println!("    Auth {:x}", auth);
            let devaff0 = mem.read(base + 0xfa8)?;
            let devaff1 = mem.read(base + 0xfac)?;
            println!("    Device affinity {:08x} {:08x}", devaff0, devaff1);
            let archid = mem.read(base + 0xfbc)?;
            println!("    Arch ID {:08x}", archid);
            let devtype = mem.read(base + 0xfcc)?;
            println!("    Device type {:08x} {}", devtype, devtype_to_str(devtype));
        }
        _ => {
            println!("Unknown entry at {:x}: {:x}", base, cidr1);
        }
    }

    Ok(())
}

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
    addr: Option<String>,
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
    assert_eq!(idcode & 0xfff, 0x477);

    let adi = Rc::new(RefCell::new(ArmDebugInterface::new(taps)));
    let mut mem = MemAP::new(adi.clone(), args.ap_num);
    
    let baseaddr = args.addr.map(|x| parse_int(&x)).unwrap_or(Ok(0)).expect("bad address");
    parse_rom_table(&mut mem, baseaddr).expect("rom table");
}
