use anyhow::{bail, Context, Result};
use clap::Parser;
use rustypot::servo::dynamixel::xl330::Xl330Controller;
use std::io::{self, Write};
use std::time::Duration;

#[derive(Parser, Debug)]
#[command(about = "Change the ID of a Dynamixel XL330 motor")]
struct Args {
    /// Current motor ID
    #[arg(long)]
    from: u8,

    /// New motor ID to assign
    #[arg(long)]
    to: u8,

    #[arg(short, long, default_value = "/dev/ttyAMA0")]
    port: String,

    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,

    /// Skip confirmation prompt
    #[arg(short = 'y', long)]
    yes: bool,
}

fn main() -> Result<()> {
    let args = Args::parse();

    if args.from == args.to {
        bail!("--from and --to are the same ({})", args.from);
    }
    if args.to == 0 || args.to > 252 {
        bail!("--to {} is out of range (valid: 1–252)", args.to);
    }

    if !args.yes {
        print!(
            "Change motor ID {} -> {}? This writes to EEPROM. [y/N] ",
            args.from, args.to
        );
        io::stdout().flush()?;
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        if !matches!(input.trim().to_lowercase().as_str(), "y" | "yes") {
            println!("Aborted.");
            return Ok(());
        }
    }

    let serial_port = serialport::new(&args.port, args.baudrate)
        .timeout(Duration::from_millis(500))
        .open()
        .context("Failed to open serial port")?;

    let mut controller = Xl330Controller::new()
        .with_protocol_v2()
        .with_serial_port(serial_port);

    // Ping to confirm motor is reachable
    let reachable = controller
        .ping(args.from)
        .with_context(|| format!("Failed to ping motor {}", args.from))?;
    if !reachable {
        bail!("Motor {} did not respond to ping", args.from);
    }
    println!("Motor {} found.", args.from);

    // Disable torque (required before writing EEPROM registers)
    controller
        .write_torque_enable(args.from, false)
        .with_context(|| format!("Failed to disable torque on motor {}", args.from))?;

    // Write new ID (EEPROM address 7)
    controller
        .write_id(args.from, args.to)
        .with_context(|| format!("Failed to write new ID to motor {}", args.from))?;

    println!("ID written. Verifying...");
    std::thread::sleep(Duration::from_millis(300));

    // Verify by pinging the new ID
    let verified = controller
        .ping(args.to)
        .with_context(|| format!("Failed to ping motor at new ID {}", args.to))?;

    if verified {
        println!("Done: motor ID changed {} -> {}", args.from, args.to);
    } else {
        bail!(
            "Motor did not respond at new ID {}. The write may have failed.",
            args.to
        );
    }

    Ok(())
}
