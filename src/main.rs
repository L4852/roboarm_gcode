// GCODE PARSER
// ===========
// COMMAND | ARG ARG ARG
// 1. Read command
// 2. Scan for argument count
// 3. Extract arguments
// 4. Locate current arm position
// 5. Calculate deviation from target
// 6. Generate motor commands

// ARDUINO MEGA
// =============
// 1. Check control mode (program, joystick)
// 2a. (Program) Parse raw motor commands into pin writes
// 2b. (Joystick) Listen for joystick inputs and calculate corresponding control

// COMPUTER                     | ARDUINO MEGA → Robot arm actuators
// Generate raw motor sequences | Execute motor sequences

use rfd::FileDialog;
use std::fs::File;
use std::io::{Read, stdin};
use std::path::PathBuf;
use std::time::Duration;

fn delimiter(character: &str, length: u8) {
    let mut delimiter_string = String::new();

    for _ in 0..length {
        delimiter_string.push(character.chars().nth(0).unwrap());
    }

    println!("{}", delimiter_string);
}

struct InitError {
    message: String,
    error: String,
}
impl InitError {
    fn new(message: &str, error: &str) -> InitError {
        InitError {
            message: message.to_string(),
            error: error.to_string(),
        }
    }
}

fn main() {
    println!("==ROBOTARM GCODE PARSER==");

    let port_address = "/dev/cu.usbserial-210";

    // init
    let mut init_errors: Vec<InitError> = Vec::new();

    let mut port = match serialport::new(port_address, 115_200)
        .timeout(Duration::from_millis(5000))
        .open()
    {
        Ok(serial_port) => serial_port,
        Err(_) => {
            println!("[init] Failed to open serial port or timed out.");
            return;
        }
    };

    delimiter("=", 65);
    println!("Serial port opened on '{}', initializing...", port_address);
    delimiter("=", 65);
    std::thread::sleep(Duration::from_millis(1000));

    delimiter("=", 30);
    println!("Running connection checks...");
    delimiter("=", 30);

    // init/write
    let init_command = "IN 0";
    println!("[init/write] Sending check string...");
    let write_result = port.write(init_command.as_bytes());

    match write_result {
        Ok(_) => {
            println!("[init/write] Sent check string successfully!");
            println!("[init/write] Waiting for check string read response...");
        }
        Err(e) => {
            println!("[init/write] Error sending check string: {:?}", e);
            init_errors.push(InitError::new(
                "[init/write] Error sending check string.",
                &e.to_string(),
            ));
        }
    }

    // init/read
    let mut buf: Vec<u8> = vec![0; 100];

    let read_result = port.read(&mut buf);

    match read_result {
        Ok(n) => {
            let check_read_value = String::from_utf8_lossy(&buf[..n]);
            println!("[init/read] Check string read successfully.");
        }
        Err(e) => {
            println!(
                "[init/read] An error occurred trying to read from the device, or the connection timed out. [{}]",
                e
            );
            init_errors.push(InitError::new("[init/read] An error occurred trying to read from the device, or the connection timed out.", &e.to_string()));
        }
    }

    // init/verify | Print failed checks
    println!("[init/verify] Failed checks:");

    let mut all_cleared = true;

    for error in &init_errors {
        println!("{} | {}", error.error, error.message);

        if (all_cleared) {
            all_cleared = false;
        }
    }

    if all_cleared {
        println!("[init/verify] Cleared all checks.");
    }

    delimiter("=", 50);

    // program

    // program/select_file, Select G-Code source file
    let mut pathbuf = PathBuf::new();

    loop {
        let file_dialog = FileDialog::new();

        println!("[program/select_file] Select a G-code file...");

        let path_buffer = file_dialog
            .add_filter("Robot Arm G-Code File", &["rgcf".to_string()])
            .pick_file();

        match path_buffer {
            Some(path) => {
                pathbuf = path;
                break;
            }
            None => {
                println!("[program/select_file] Canceled file dialog.");
                return;
            }
        };
    }

    // program/read_file

    let mut file_string = String::new();
    let mut target_file = File::open(pathbuf).expect("[program/select_file] Failed to open file.");

    File::read_to_string(&mut target_file, &mut file_string)
        .expect("[program/read_file] Failed to read file.");

    println!("File: {}", file_string);
    println!("[program/select_file] Read file successfully.");

    // ================
    // PARSE LOGIC
    // =================

    #[derive(Debug)]
    enum Axis {
        X(f32),
        Y(f32),
        Z(f32),
        A(f32),
        B(f32, f32),
        C(f32),
    }

    // No Op
    // Homing
    // Target
    // Claw
    // Manual
    // Return Home
    // Reset
    // Force Stop

    #[derive(Debug)]
    enum Command {
        NO,
        HM(f32, f32, f32),
        TG(f32, f32, f32),
        CL(f32, f32, f32, f32, f32),
        MN(Axis),
        RH,
        RS,
        FS,
    }

    fn extract_command(command_string: &str, arguments: Vec<&str>) -> Result<Command, bool> {
        println!("COmmand string: {}", command_string);
        match command_string {
            "NO" => Ok(Command::NO),
            "HM" => Ok(Command::HM(
                arguments[0].parse::<f32>().unwrap(),
                arguments[1].parse::<f32>().unwrap(),
                arguments[2].parse::<f32>().unwrap(),
            )),
            "TG" => Ok(Command::TG(
                arguments[0].parse::<f32>().unwrap(),
                arguments[1].parse::<f32>().unwrap(),
                arguments[2].parse::<f32>().unwrap(),
            )),
            "CL" => Ok(Command::CL(
                arguments[0].parse::<f32>().unwrap(),
                arguments[1].parse::<f32>().unwrap(),
                arguments[2].parse::<f32>().unwrap(),
                arguments[3].parse::<f32>().unwrap(),
                arguments[4].parse::<f32>().unwrap(),
            )),
            "MN" => match arguments[0] {
                "X" => Ok(Command::MN(Axis::X(arguments[1].parse::<f32>().unwrap()))),
                "Y" => Ok(Command::MN(Axis::Y(arguments[1].parse::<f32>().unwrap()))),
                "Z" => Ok(Command::MN(Axis::Z(arguments[1].parse::<f32>().unwrap()))),
                "A" => Ok(Command::MN(Axis::A(arguments[1].parse::<f32>().unwrap()))),
                "B" => Ok(Command::MN(Axis::B(
                    arguments[1].parse::<f32>().unwrap(),
                    arguments[2].parse::<f32>().unwrap(),
                ))),
                "C" => Ok(Command::MN(Axis::C(arguments[1].parse::<f32>().unwrap()))),
                _ => Err(false),
            },
            "RH" => Ok(Command::RH),
            "RS" => Ok(Command::RS),
            "FS" => Ok(Command::FS),
            _ => Result::Err(false),
        }
    }

    let commands = file_string.split("\n").collect::<Vec<&str>>();

    for command in commands {
        let terms = command.split_whitespace().collect::<Vec<&str>>();

        println!("Terms: {:?}", terms);

        println!("Function: {}", terms[0]);

        print!("Arguments: ");

        let mut args: Vec<&str> = Vec::new();

        for arg_idx in 1..terms.len() {
            if arg_idx == terms.len() - 1 {
                print!("{}\n", terms[arg_idx]);
                break;
            }
            print!("{},", terms[arg_idx]);
            args.push(terms[arg_idx]);
        }

        let parsed = extract_command(terms[0], args);

        match &parsed {
            Ok(result) => {
                println!("Parsed: {:?}", parsed);
            }
            Err(e) => {
                println!("Error: {}", e);
            }
        }
    }
}
