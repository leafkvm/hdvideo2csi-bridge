use env_logger::Env;

mod app;
mod avi;
mod config;
mod drv;
mod format;
mod iio;
mod kernel_proxy_reg;
mod reg;
mod stub;
mod time;
mod timing;

use app::App;

struct Args {
    /// Video I2C register proxy (`/dev/leafkvm-*`).
    reg_proxy: String,
    /// Video Input CSI stub device, supplied on the command line.
    csi_stub: String,
}

impl Args {
    fn parse() -> Self {
        let mut args = std::env::args().skip(1);
        match (args.next(), args.next(), args.next()) {
            (Some(reg_proxy), Some(csi_stub), None) => Self {
                reg_proxy,
                csi_stub,
            },
            _ => {
                eprintln!("Usage: hd2csi <reg-proxy-dev> <csi-stub-dev>");
                std::process::exit(2);
            }
        }
    }
}

fn main() -> std::io::Result<()> {
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();
    log::info!(
        "Starting {} {}",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );

    let args = Args::parse();
    reg::init(&args.reg_proxy)
        .unwrap_or_else(|e| panic!("failed to open I2C reg proxy {}: {e}", args.reg_proxy));
    stub::init(&args.csi_stub)
        .unwrap_or_else(|e| panic!("failed to open CSI stub device {}: {e}", args.csi_stub));

    let cfg = stub::get_config().expect("failed to get initial config from kernel");
    log::info!(
        "Kernel configuration: CSI lanes={}, media-bus format=0x{:04X}",
        cfg.csi_lanes,
        cfg.mbus_fmt_code
    );

    let mut app = App::default();
    app.boot();
    log::info!("Bridge is ready");

    loop {
        app.tick();
    }
}
