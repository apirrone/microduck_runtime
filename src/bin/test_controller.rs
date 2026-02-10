use anyhow::Result;
use std::thread;
use std::time::Duration;
use microduck_runtime::controller::{Controller, ControllerState};

const DEADZONE: f32 = 0.1;

fn format_bar(value: f32, width: usize) -> String {
    let normalized = ((value + 1.0) / 2.0).clamp(0.0, 1.0); // Map -1..1 to 0..1
    let filled = (normalized * width as f32) as usize;
    let bar: String = (0..width)
        .map(|i| if i < filled { '█' } else { '░' })
        .collect();
    format!("[{}] {:6.2}", bar, value)
}

fn format_trigger_bar(value: f32, width: usize) -> String {
    let normalized = value.clamp(0.0, 1.0);
    let filled = (normalized * width as f32) as usize;
    let bar: String = (0..width)
        .map(|i| if i < filled { '█' } else { '░' })
        .collect();
    format!("[{}] {:5.2}", bar, value)
}

fn display_controller_state(state: &ControllerState) {
    // Clear screen
    print!("\x1B[2J\x1B[1;1H");

    println!("╔════════════════════════════════════════════════════════════╗");
    println!("║          Xbox Controller Test - Press Ctrl+C to exit      ║");
    println!("╚════════════════════════════════════════════════════════════╝");
    println!();

    // Left stick
    println!("┌─ LEFT STICK ─────────────────────────────────────────┐");
    let lx = Controller::apply_deadzone(state.left_stick_x, DEADZONE);
    let ly = Controller::apply_deadzone(state.left_stick_y, DEADZONE);
    println!("│ X: {}  │", format_bar(lx, 30));
    println!("│ Y: {}  │", format_bar(ly, 30));
    println!("└──────────────────────────────────────────────────────┘");
    println!();

    // Right stick
    println!("┌─ RIGHT STICK ────────────────────────────────────────┐");
    let rx = Controller::apply_deadzone(state.right_stick_x, DEADZONE);
    let ry = Controller::apply_deadzone(state.right_stick_y, DEADZONE);
    println!("│ X: {}  │", format_bar(rx, 30));
    println!("│ Y: {}  │", format_bar(ry, 30));
    println!("└──────────────────────────────────────────────────────┘");
    println!();

    // Triggers
    println!("┌─ TRIGGERS ───────────────────────────────────────────┐");
    println!("│ LT: {}     │", format_trigger_bar(state.left_trigger, 30));
    println!("│ RT: {}     │", format_trigger_bar(state.right_trigger, 30));
    println!("└──────────────────────────────────────────────────────┘");
    println!();

    // D-Pad
    println!("┌─ D-PAD ──────────────────────────────────────────────┐");
    println!("│ X: {:6.2}  Y: {:6.2}                                │",
             state.dpad_x, state.dpad_y);

    // Visual D-Pad representation
    let up = state.dpad_y > 0.5;
    let down = state.dpad_y < -0.5;
    let left = state.dpad_x < -0.5;
    let right = state.dpad_x > 0.5;

    println!("│                   {}                                 │", if up { "▲" } else { " " });
    println!("│                {}  {}  {}                              │",
             if left { "◀" } else { " " },
             if !up && !down && !left && !right { "●" } else { " " },
             if right { "▶" } else { " " });
    println!("│                   {}                                 │", if down { "▼" } else { " " });
    println!("└──────────────────────────────────────────────────────┘");
    println!();

    // Buttons
    println!("┌─ BUTTONS ────────────────────────────────────────────┐");

    // Face buttons
    let a = state.buttons.get("South").copied().unwrap_or(false);
    let b = state.buttons.get("East").copied().unwrap_or(false);
    let x = state.buttons.get("West").copied().unwrap_or(false);
    let y = state.buttons.get("North").copied().unwrap_or(false);

    println!("│ A: {}  B: {}  X: {}  Y: {}                    │",
             if a { "[X]" } else { "[ ]" },
             if b { "[X]" } else { "[ ]" },
             if x { "[X]" } else { "[ ]" },
             if y { "[X]" } else { "[ ]" });

    // Shoulder buttons
    let lb = state.buttons.get("LeftTrigger").copied().unwrap_or(false);
    let rb = state.buttons.get("RightTrigger").copied().unwrap_or(false);
    println!("│ LB: {}  RB: {}                                    │",
             if lb { "[X]" } else { "[ ]" },
             if rb { "[X]" } else { "[ ]" });

    // Stick buttons
    let l3 = state.buttons.get("LeftThumb").copied().unwrap_or(false);
    let r3 = state.buttons.get("RightThumb").copied().unwrap_or(false);
    println!("│ L3: {}  R3: {}                                    │",
             if l3 { "[X]" } else { "[ ]" },
             if r3 { "[X]" } else { "[ ]" });

    // Menu buttons
    let start = state.buttons.get("Start").copied().unwrap_or(false);
    let select = state.buttons.get("Select").copied().unwrap_or(false);
    let mode = state.buttons.get("Mode").copied().unwrap_or(false);
    println!("│ Start: {}  Select: {}  Xbox: {}                │",
             if start { "[X]" } else { "[ ]" },
             if select { "[X]" } else { "[ ]" },
             if mode { "[X]" } else { "[ ]" });

    println!("└──────────────────────────────────────────────────────┘");
    println!();

    // Raw button dump (for debugging unmapped buttons)
    if !state.buttons.is_empty() {
        println!("┌─ ALL BUTTONS (raw) ──────────────────────────────────┐");
        for (name, pressed) in &state.buttons {
            if *pressed {
                println!("│ {:30} [PRESSED]                │", name);
            }
        }
        println!("└──────────────────────────────────────────────────────┘");
    }
}

fn main() -> Result<()> {
    println!("Xbox Controller Test Program");
    println!("============================\n");

    // Initialize controller
    let mut controller = Controller::new()?;

    // Wait for controller connection
    controller.wait_for_connection()?;

    if let Some(name) = controller.get_controller_name() {
        println!("\nController: {}\n", name);
    }

    println!("Reading controller input (press Ctrl+C to stop)...\n");
    thread::sleep(Duration::from_secs(1));

    // Main loop
    loop {
        // Update controller state
        controller.update()?;

        // Display state
        display_controller_state(controller.get_state());

        // Update at ~60 Hz
        thread::sleep(Duration::from_millis(16));
    }
}
