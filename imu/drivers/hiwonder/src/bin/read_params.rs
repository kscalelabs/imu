use clap::Parser;
use hiwonder::HiwonderReader;
use std::io;
use std::time::Duration;
use tracing::info;
use tracing_subscriber::EnvFilter;

#[derive(Parser)]
struct Args {
    #[clap(short, long, default_value = "/dev/ttyUSB0")]
    device: String,
    #[clap(short, long, default_value_t = 230400)]
    baud_rate: u32,
}

fn main() -> io::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::from_default_env()
                .add_directive("polling=off".parse().unwrap())
                .add_directive("async_io=off".parse().unwrap()),
        )
        .init();

    let args = Args::parse();
    let (port, baud_rate) = (args.device, args.baud_rate);

    println!("Attempting to connect to {} at {} baud...", port, baud_rate);
    let reader = match HiwonderReader::new(&port, baud_rate, Duration::from_secs(1), true) {
        Ok(r) => {
            println!("Successfully connected to {}", port);
            r
        }
        Err(e) => {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                format!("Failed to connect to {}: {}", port, e),
            ))
        }
    };

    let params = reader.read_all_registers(Duration::from_secs(1));
    match params {
        Ok(params) => {
            info!("Successfully read all parameters");
            for (register, data) in params {
                info!("Register: {:?}, Data: {:?}", register, data);
            }
            Ok(())
        }
        Err(e) => Err(io::Error::new(
            io::ErrorKind::Other,
            format!("Failed to read parameters: {}", e),
        )),
    }
}
