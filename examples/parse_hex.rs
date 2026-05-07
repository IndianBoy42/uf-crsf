//! Parse a hex or decimal byte array as a CRSF packet.
//!
//! Usage:
//!   cargo run --example parse_hex -- "C8 06 2C EE EA 02 00 9B"
//!   cargo run --example parse_hex -- "[C8, 06, 2C, EE, EA, 02, 00, 9B]"
//!   cargo run --example parse_hex -- --dec "200 6 44 238 234 2 0 155"
//!   cargo run --example parse_hex -- --dec "[200, 6, 44, 238, 234, 2, 0, 155]"
//!   cargo run --example parse_hex --features=logging -- "C8 06 2C ..."
//!
//! Set RUST_LOG=debug (or trace) to see the parsing pipeline logs:
//!   RUST_LOG=debug cargo run --example parse_hex --features=logging -- "C8 06 2C ..."

use clap::Parser;
use uf_crsf::CrsfParser;
use uf_crsf::CrsfStreamError;

/// CLI to parse arbitrary CRSF byte arrays.
#[derive(Parser)]
#[command(name = "parse_hex", about = "Parse CRSF byte array (hex by default)")]
struct Args {
    /// Bytes to parse — hex by default, decimal when --dec is set
    bytes: String,

    /// Interpret input as decimal values (0-255) instead of hex
    #[arg(long)]
    dec: bool,
}

fn parse_bytes(input: &str, radix: u32) -> Result<Vec<u8>, String> {
    let stripped = input
        .trim()
        .trim_start_matches('[')
        .trim_end_matches(']')
        .replace(",", " ");

    let mut bytes = Vec::new();
    for token in stripped.split_whitespace() {
        let clean = token.trim();
        if clean.is_empty() {
            continue;
        }
        let b = u8::from_str_radix(clean, radix)
            .map_err(|e| format!("invalid byte '{clean}': {e}"))?;
        bytes.push(b);
    }

    if bytes.is_empty() {
        return Err("no bytes parsed".into());
    }
    Ok(bytes)
}

fn main() {
    // env_logger respects RUST_LOG (e.g., RUST_LOG=debug, RUST_LOG=trace)
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
        .format_timestamp(None)
        .init();

    let args = Args::parse();
    let radix = if args.dec { 10 } else { 16 };
    let bytes = parse_bytes(&args.bytes, radix).unwrap_or_else(|e| {
        eprintln!("Error parsing input: {e}");
        std::process::exit(1);
    });

    println!("Input bytes ({}) : {:02X?}", bytes.len(), bytes);
    println!();

    let mut parser = CrsfParser::new();
    let mut err_count = 0u32;

    for result in parser.iter_packets(&bytes) {
        match result {
            Ok(packet) => {
                println!("OK  => {packet:?}");
            }
            Err(e) => {
                err_count += 1;
                match e {
                    CrsfStreamError::InvalidSync(b) => {
                        // expected during resync — show at higher verbosity
                        eprintln!("SKIP: invalid sync byte 0x{b:02X}");
                    }
                    CrsfStreamError::InvalidPacketLength(b) => {
                        eprintln!("ERR:  invalid packet length 0x{b:02X}");
                    }
                    _ => {
                        eprintln!("ERR:  {e:?}");
                    }
                }
            }
        }
    }

    if err_count > 0 {
        eprintln!("\n({err_count} error(s) — parser auto-resynced)");
    }
}
