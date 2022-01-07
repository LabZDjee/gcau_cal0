# gCAU cal0

Automatic process to configure current offsets on a gCAU at rev 3.1.20+ through asynchronous serial port

This is first [rust](https://www.rust-lang.org/) program we have ever written, so style and design choice are probably not outstanding and its comments can sometimes look naive

It is mainly based on [crate serialport](https://lib.rs/crates/serial) for dealing with the serial ports. It scans all serial ports it can find on system, trying several baudrates, working in parallel with one thread per port

Details how current offsets are calculated and configured is not detailed here. For this, see in the source code

This CLI program can take optional parameters:

- `-v, --verbose`: to get details of the process printed on console, otherwise program only prints errors on `stderr` (and some preliminary information, see `--no-prompt` option)
- `-d, --no-prompt`: normally program informs user what is the hardware setup for the test and then prompt the user to strike 'Y' to start. With this option, this phase is skipped and process starts right away
- `-n , --nb-targets <n>` : after ` <n>`  success(es) on serial port, does not wait any longer

Returned value: negative in case of error, zero in case of no successful process, if positive returns number of successful outcomes



