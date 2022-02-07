//
// CLI process to automatically look for a connected gCAU on any serial port
// and then set shunt current offsets, provided they are connected to a given low
// value resistor
//
use argparse::{ArgumentParser, Print, StoreFalse, StoreOption, StoreTrue}; // for command line arguments
use regex::Regex;
use serialport::{available_ports, ClearBuffer, SerialPort};
use std::io;
use std::io::Write; // for io::stdout().flush()
use std::process::exit;
use std::sync::mpsc::channel;
use std::thread;
use std::time::{Duration, Instant};

const APP_VERSION: &str = "1.1";

// different outcomes of the entire process for a given detected serial port
#[derive(PartialEq)]
enum ProcessResult {
  ConnectionFailure,
  LoginFailure,
  TaskUnsuccessful,
  UncompletedLogout, // equivalent to a success, this is just a warning
  Success,
}

// makes states above human readable
impl ProcessResult {
  fn to_string(&self) -> String {
    match self {
      ProcessResult::ConnectionFailure => "Unable to reach target".to_string(),
      ProcessResult::LoginFailure => "Unable to log to target".to_string(),
      ProcessResult::TaskUnsuccessful => "Could not complete process on target".to_string(),
      ProcessResult::UncompletedLogout => "Task done (though failure at logout)".to_string(),
      ProcessResult::Success => "Task successfully completed".to_string(),
    }
  }
}

// describes some data around connected serial port
struct SerialAppContext {
  dev_name: String,
  port: Box<dyn SerialPort>,
}

impl SerialAppContext {
  fn new(port: Box<dyn SerialPort>, dev_name: &String) -> SerialAppContext {
    SerialAppContext {
      port, // steals ownership
      dev_name: dev_name.clone(),
    }
  }

  // core function of reading incoming reply with a 'ch' terminator
  // a little bit blunt as control characters are stripped in result
  // if Ok, entire message
  // if Err, only what has been collected so far (specially in case of time out)
  fn read_till(&mut self, ch: u8) -> Result<String, String> {
    let mut str = String::new();
    let mut now = Instant::now();
    let mut duration = Duration::from_millis(2000); // initial time-out
    let mut b = [0 as u8; 1]; // will read chars one by one, wih this trick, no cleverer way found
    while now.elapsed() < duration {
      let n = self.port.bytes_to_read().unwrap();
      for _ in 0..n {
        if let Err(_) = self.port.read_exact(&mut b) {
          str.retain(|c| c >= ' ');
          return Err(str);
        }
        if b[0] == ch {
          str.retain(|c| c >= ' ');
          return Ok(str);
        }
        str.push(b[0] as char);
        now = Instant::now();
      }
      if n == 0 {
        thread::yield_now();
      } else {
        duration = Duration::from_millis(100); // time-out between characters
      }
    }
    str.retain(|c| c >= ' ');
    Err(str)
  }

  // in usual case, CR is a meaningful character
  fn read_till_cr(&mut self) -> Result<String, String> {
    self.read_till('\r' as u8)
  }

  // send command, appending a CR to it
  fn send_cmd(&mut self, cmd: String) -> bool {
    thread::sleep(Duration::from_millis(100));
    let _ = self.port.clear(ClearBuffer::All);
    let s = format!("{}\n", cmd);
    if let Err(_) = self.port.write_all(&s.into_bytes()) {
      eprintln!("[{}] Cannot send '{}'", self.dev_name, cmd);
      return false;
    }
    true
  }

  // check reply against a regex pattern
  // reply could already be an Err in which case it is passed on, decorated with serial port name
  // if match is not successful, then Err is returned containing a message
  // decorated with serial port name and keyword to contextualize the error
  fn check_reply_against(
    &mut self,
    reply: Result<String, String>,
    regex: &str,
    key_word: &str,
  ) -> Result<String, String> {
    if let Err(err_msg) = reply {
      return Err(format!(
        "[{}] got malformed reply with content \"{}\"",
        self.dev_name, err_msg
      ));
    }
    let reply = reply.unwrap();
    if !Regex::new(&regex).unwrap().is_match(&reply) {
      return Err(format!(
        "[{}] {}: got unexpected reply '{}'",
        self.dev_name, key_word, reply
      ));
    }
    Ok(reply)
  }
}

// as reply from a gCAU is made of fields which are slash '/' separated,
// splits this reply as a vector of fields
fn split_reply(str: &String) -> Vec<String> {
  let mut result = Vec::<String>::new();
  let mut previous_char: Option<&str> = None;
  let mut first_index = 0;
  let len = str.len();
  if len == 0 {
    result.push("".to_string());
  } else {
    for i in 0..len {
      let current_char = &str[i..=i];
      if current_char == "/" && previous_char != Some("\\") {
        result.push(str[first_index..i].to_string());
        first_index = i + 1;
      }
      if i == len - 1 {
        result.push(str[first_index..=i].to_string());
      }
      previous_char = Some(current_char);
    }
  }
  result
}

// backdoor challenge calculation
fn bkdoor_reply_to_challenge(challenge: &String) -> Option<String> {
  if challenge.len() != 3 {
    return None;
  }
  let mut as_bytes = challenge.as_bytes().to_vec();
  for i in 0..as_bytes.len() {
    let v = as_bytes[i];
    if v < 0x30 || v > 0x39 {
      return None;
    }
    as_bytes[i] = v - 0x30;
  }
  as_bytes[2] = (as_bytes[1] + as_bytes[2] + 2) % 10;
  as_bytes[1] = (as_bytes[1] + as_bytes[2] + 5) % 10;
  as_bytes[0] = (as_bytes[0] + as_bytes[1] + 7) % 10;
  for i in 0..as_bytes.len() {
    as_bytes[i] = as_bytes[i] + 0x30;
  }
  Some(String::from_utf8(as_bytes).unwrap())
}

// return baudrates which could be tried in sequence
fn baudrates(index: usize) -> Option<u32> {
  let brs = [38_400, 19_200, 9_600];
  return if index < brs.len() {
    Some(brs[index])
  } else {
    None
  };
}

fn main() {
  let mut nb_of_completions: i32 = 0;
  let mut nb_of_success: i32 = 0;
  let mut verbose = false;
  let mut prompt = true; // informs user and prompts for a key before proceeding
  let mut nb_of_expected_targets = None::<i32>; // if defined will terminate as soon as this number of connection succeeds
  {
    // this block limits scope of borrows by ap.refer() method
    let mut ap = ArgumentParser::new();
    ap.set_description(
      "Set offsets of currents for gCAU 3.1.20+ (with a 10-Ohm resistor on shunt inputs)",
    );
    ap.refer(&mut verbose)
      .add_option(&["-v", "--verbose"], StoreTrue, "Details on each step");
    ap.refer(&mut prompt).add_option(
      &["-d", "--no-prompt"],
      StoreFalse,
      "Proceeds right away without prompting user",
    );
    ap.refer(&mut nb_of_expected_targets).add_option(
      &["-n", "--nb_targets"],
      StoreOption,
      "Number of targets assumed to be connected (this to speed up completion slowed by stale ports)",
    );
    ap.add_option(
      &["-V", "--version"],
      Print(APP_VERSION.to_string()),
      "Show version",
    );
    ap.parse_args_or_exit();
  }
  // almost every println! depends on verbose being true
  macro_rules! println_if_verbose {
    () => {println!();};
    ($($arg:tt)*) => {
      if verbose {
        println!($($arg)*);
      }
    };
  }
  let ports = available_ports().expect("No serial ports found!");
  if ports.len() == 0 {
    eprintln!("Cannot proceed: no suitable serial communication port found!");
    exit(-2);
  }
  if verbose {
    println!("Serial ports found:");
    for p in &ports {
      println!("\t{}", p.port_name);
    }
  }
  if verbose && nb_of_expected_targets != None {
    println!(
      "Number of expected targets: {}",
      nb_of_expected_targets.unwrap()
    );
  }
  if prompt {
    println!("Connect 10-Ohm resistors on shunts (connector X5 of the gCAU),\n if on test bench 703864, set shunts to off (Ib+ centered and ISr0 at 1)");
    print!("Ready to go? Y/n ");
    io::stdout().flush().unwrap();
    let mut should_be_yes = String::new();
    let _ = io::stdin().read_line(&mut should_be_yes);
    if !Regex::new(r"^[yY]").unwrap().is_match(&should_be_yes) {
      println!("Abort!");
      exit(-100);
    }
  }
  thread::sleep(Duration::from_secs(2));
  // will launch a worker thread for each detected serial port
  let (th_tx, th_rx) = channel(); // channel to communicate from worker threads to main thread
  let number_of_ports = ports.len();
  for port in ports {
    let port_name = port.port_name.clone();
    let th_tx = th_tx.clone();
    let _ = thread::Builder::new()
      .name(format!("worker thread on {}", port_name))
      .spawn(move || {
        // tries each baudrate in turn
        let mut stage = ProcessResult::ConnectionFailure;
        let mut ctx: Option<SerialAppContext> = None;
        let mut logged = false;

        for bdi in 0.. {
          // for each baudrate tries a successful 'ping'
          let baudrate = baudrates(bdi);
          if baudrate == None {
            let _ = th_tx.send((port_name, stage)); // no baudrate is correct, give up
            return;
          }
          let baudrate = baudrate.unwrap();
          println_if_verbose!("[{}] trying connection at {} bps", port_name, baudrate);
          let port;
          match serialport::new(&port_name, baudrate)
            .timeout(Duration::from_millis(500))
            .open()
          {
            Ok(open_port) => port = open_port,
            Err(_) => {
              println_if_verbose!("[{}] refuses to open", port_name);
              let _ = th_tx.send((port_name, stage));
              return;
            }
          }
          // owns port in SerialAppContext
          let mut ctx0 = SerialAppContext::new(port, &port_name);
          // echo command
          let cmd = "@".to_string();
          ctx0.send_cmd(cmd.clone()); // dry-run to clear serial port of any trash stuck
          println_if_verbose!("[{}] command: {}", ctx0.dev_name, cmd);
          if !ctx0.send_cmd(cmd) {
            println_if_verbose!("[{}] failure sending command", ctx0.dev_name);
            std::mem::drop(ctx0.port); // trick to close serial port
            continue;
          }
          let reply = ctx0.read_till_cr();
          let reply = ctx0.check_reply_against(reply, r"/ECHO/", "PING (ECHO)");
          if let Err(err_msg) = reply {
            println_if_verbose!("{}", err_msg);
            std::mem::drop(ctx0.port); // closes serial port
            continue;
          }
          println_if_verbose!("[{}] reply: {}", ctx0.dev_name, reply.unwrap());
          ctx = Some(ctx0); // wraps ctx0 for ctx to unwrap it
          break;
        }
        let mut ctx = ctx.unwrap();
        // if argument is false, sends a tentative logout if logged,
        // reports to main thread and dies
        // otherwise does nothing
        macro_rules! vanish_if_false {
          ($test:expr) => {
            if $test == false {
              if logged {
                let cmd = "LOGOUT".to_string();
                ctx.send_cmd(cmd);
                thread::sleep(Duration::from_millis(500));
              }
              let _ = th_tx.send((port_name, stage));
              return;
            }
          };
        }
        // time to login as admin through a password agnostic backdoor process
        stage = ProcessResult::LoginFailure;
        let cmd = "@&1/BKDOOR".to_string();
        vanish_if_false!(ctx.send_cmd(cmd));
        let reply = ctx.read_till_cr();
        let reply = ctx.check_reply_against(reply, r"/BKDOOR", "BKDOOR");
        let mut correct_reply: String = "".to_string();
        // check reply to be Ok, if so pass on to correct_reply, otherwise dies gracefully
        macro_rules! vanish_if_reply_wrong {
          ($reply:ident) => {
            match $reply {
              Ok(msg) => correct_reply = msg,
              Err(err_msg) => {
                eprintln!("{}", err_msg);
                vanish_if_false!(false);
              }
            }
          };
        }
        vanish_if_reply_wrong!(reply);
        println_if_verbose!("[{}] reply: {}", ctx.dev_name, correct_reply);
        let split = split_reply(&correct_reply);
        let mut challenge = None;
        if split.len() >= 3 {
          challenge = bkdoor_reply_to_challenge(&split[2]);
        }
        if challenge == None {
          eprintln!(
            "[{}] Reply '{}' to back door opening is inappropriate",
            ctx.dev_name, &split[2]
          );
          vanish_if_false!(false);
        }
        // now tries login with computed pass key
        let cmd = format!("@/LOGIN/2/{}/100", challenge.unwrap());
        println_if_verbose!("[{}] command: {}", ctx.dev_name, cmd);
        ctx.send_cmd(cmd);
        let reply = ctx.read_till_cr();
        let reply = ctx.check_reply_against(reply, r"/LOGIN/OK", "LOGIN");
        vanish_if_reply_wrong!(reply);
        println_if_verbose!("[{}] reply: {}", ctx.dev_name, correct_reply);
        logged = true;
        // logged now, so fetch converter raw values
        stage = ProcessResult::TaskUnsuccessful;
        let cmd = "@/TEST/CNV".to_string();
        println_if_verbose!("[{}] command: {}", ctx.dev_name, cmd);
        vanish_if_false!(ctx.send_cmd(cmd));
        let reply = ctx.read_till_cr();
        let reply = ctx.check_reply_against(reply, r"/TEST/CNV", "TEST/CNV");
        vanish_if_reply_wrong!(reply);
        println_if_verbose!("[{}] reply: {}", ctx.dev_name, correct_reply);
        // analyzes replies
        let replies = split_reply(&correct_reply);
        if replies.len() >= 8 {
          // interested in charger and battery currents as raw values
          let charger_current: Result<i32, _> = replies[6].parse();
          let battery_current: Result<i32, _> = replies[7].parse();
          if let Err(_) = charger_current {
            eprintln!(
              "[{}] unable to get charger current from \"{}\"",
              ctx.dev_name, replies[5]
            );
          } else if let Err(_) = battery_current {
            eprintln!(
              "[{}] unable to get battery current from \"{}\"",
              ctx.dev_name, replies[6]
            );
          } else {
            let max_current = 35; // an offset beyond this value is considered unacceptable
            let charger_current = charger_current.unwrap();
            let battery_current = battery_current.unwrap();
            if verbose {
              println!(
                "[{}] Measured battery and charger currents: {} and {} ADC steps",
                ctx.dev_name, charger_current, battery_current
              );
            }
            if battery_current > max_current || charger_current > max_current {
              eprintln!(
                "[{}] Error: charger current at {} or battery current at {} exceeds {}",
                ctx.dev_name, charger_current, battery_current, max_current
              );
            } else {
              // calculates corrections and apply them
              let current_centered_offset = 154; // theoretical offset with a 10-Ohm resistor on shunts (in thenth of ADC steps)
              let charger_current_correction = charger_current * 10 - current_centered_offset;
              let battery_current_correction = battery_current * 10 - current_centered_offset;
              if verbose {
                println!("[{}] Proceeding to correction", ctx.dev_name);
              }
              // correction on battery discharge offset is just assumed and could be wrong which normally does not matter much
              let cmd = format!(
                "@/WCFG/CALIBR0/1:{}/2:{}/3:{}",
                battery_current_correction, battery_current_correction, charger_current_correction
              );
              println_if_verbose!("[{}] command: {}", ctx.dev_name, cmd);
              vanish_if_false!(ctx.send_cmd(cmd));
              let reply = ctx.read_till_cr();
              let reply = ctx.check_reply_against(reply, r"/WCFG/CALIBR0", "WCFG/CALIBR0");
              vanish_if_reply_wrong!(reply);
              println_if_verbose!("[{}] reply: {}", ctx.dev_name, correct_reply);
              stage = ProcessResult::Success;
            }
          }
        } else {
          eprintln!(
            "[{}] TEST/CNV: truncated reply \"{}\"",
            ctx.dev_name, correct_reply
          );
          vanish_if_false!(false);
        }
        logged = false;
        if stage != ProcessResult::TaskUnsuccessful {
          // if unsuccessful don't lose this information
          stage = ProcessResult::UncompletedLogout;
        }
        let cmd = "@/LOGOUT".to_string();
        println_if_verbose!("[{}] command: {}", ctx.dev_name, cmd);
        vanish_if_false!(ctx.send_cmd(cmd));
        let reply = ctx.read_till_cr();
        let reply = ctx.check_reply_against(reply, r"/LOGOUT/OK", "LOGOUT");
        vanish_if_reply_wrong!(reply);
        println_if_verbose!("[{}] reply: {}", ctx.dev_name, correct_reply);
        stage = ProcessResult::Success;
        let _ = th_tx.send((port_name, stage));
      });
  }
  for _ in 0..number_of_ports {
    // does not wait if enough completions done
    if nb_of_expected_targets != None && nb_of_completions >= nb_of_expected_targets.unwrap() {
      break;
    }
    match th_rx.recv_timeout(Duration::from_secs(120)) {
      Ok((who, result)) => {
        // who = thread name, result is a ProcessResult
        let id_message = format!("Thread on {}", who);
        match result {
          ProcessResult::ConnectionFailure => {
            eprintln!("{} could not reach target", id_message);
          }
          ProcessResult::Success => {
            println_if_verbose!("{} was SUCCESSFUL", id_message);
            nb_of_completions += 1;
            nb_of_success += 1;
          }
          ProcessResult::UncompletedLogout => {
            println_if_verbose!(
              "{} was SUCCESSFUL (albeit some problem at logout)",
              id_message
            );
            nb_of_completions += 1;
            nb_of_success += 1;
          }
          _ => {
            eprintln!("{} returned: \"{}\"", id_message, result.to_string());
            nb_of_completions += 1;
          }
        }
      }
      // in the improbable case, some threads got stuck
      Err(_) => {
        eprintln!("program unexpectedly stuck: forced exit");
        std::process::exit(-2);
      }
    }
  }
  exit(nb_of_success);
}

#[cfg(test)]
mod func_test {
  use crate::*;

  fn array_str_to_string(strs: Vec<&str>) -> Vec<String> {
    strs.iter().map(|&s| s.to_string()).collect::<Vec<_>>()
  }

  #[test]
  fn bk_challenge_1() {
    assert_eq!(
      bkdoor_reply_to_challenge(&"247".to_string()),
      Some("123".to_string())
    );
  }

  #[test]
  fn bk_challenge_2() {
    assert_eq!(
      bkdoor_reply_to_challenge(&"353".to_string()),
      Some("000".to_string())
    );
  }

  #[test]
  fn bk_challenge_3() {
    assert_eq!(
      bkdoor_reply_to_challenge(&"000".to_string()),
      Some("472".to_string())
    );
  }

  #[test]
  fn bk_challenge_4() {
    assert_eq!(bkdoor_reply_to_challenge(&"1234".to_string()), None);
    assert_eq!(bkdoor_reply_to_challenge(&"12 ".to_string()), None);
    assert_eq!(bkdoor_reply_to_challenge(&"/23".to_string()), None);
    assert_eq!(bkdoor_reply_to_challenge(&"1:3".to_string()), None);
  }

  #[test]
  fn split_1() {
    let split = split_reply(&"RCFG/SYS_1/1/two/drei/cuatro/5/E3".to_string());
    let expected = array_str_to_string(vec![
      "RCFG", "SYS_1", "1", "two", "drei", "cuatro", "5", "E3",
    ]);
    assert_eq!(&split, &expected);
  }

  #[test]
  fn split_2() {
    let split = split_reply(&"/RCFG/SYS_1/1/two/drei/F4/".to_string());
    let expected = array_str_to_string(vec!["", "RCFG", "SYS_1", "1", "two", "drei", "F4", ""]);
    assert_eq!(&split, &expected);
  }

  #[test]
  fn split_3() {
    let str = "GENERAL_ERROR";
    let split = split_reply(&str.to_string());
    let expected = array_str_to_string(vec![str]);
    assert_eq!(&split, &expected);
  }

  #[test]
  fn split_4() {
    let str = "";
    let split = split_reply(&str.to_string());
    let expected = array_str_to_string(vec![str]);
    assert_eq!(&split, &expected);
  }
}
