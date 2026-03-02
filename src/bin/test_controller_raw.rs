use anyhow::Result;
use gilrs::{Gilrs, Event, EventType};

fn main() -> Result<()> {
    println!("Raw controller event dump - press buttons/triggers/sticks and watch what comes in");
    println!("Press Ctrl+C to exit\n");

    let mut gilrs = Gilrs::new()
        .map_err(|e| anyhow::anyhow!("Failed to initialize gilrs: {}", e))?;

    // List connected gamepads
    for (_id, gamepad) in gilrs.gamepads() {
        println!("Found gamepad: {} ({})", gamepad.name(), gamepad.id());
    }
    println!();

    loop {
        while let Some(Event { id, event, time: _ }) = gilrs.next_event() {
            match &event {
                EventType::AxisChanged(axis, value, code) => {
                    println!("[{:?}] AxisChanged:   {:?} (code={:?}) = {:.4}", id, axis, code, value);
                }
                EventType::ButtonChanged(button, value, code) => {
                    println!("[{:?}] ButtonChanged: {:?} (code={:?}) = {:.4}", id, button, code, value);
                }
                EventType::ButtonPressed(button, code) => {
                    println!("[{:?}] ButtonPressed: {:?} (code={:?})", id, button, code);
                }
                EventType::ButtonReleased(button, code) => {
                    println!("[{:?}] ButtonReleased: {:?} (code={:?})", id, button, code);
                }
                EventType::Connected => {
                    println!("[{:?}] Connected", id);
                }
                EventType::Disconnected => {
                    println!("[{:?}] Disconnected", id);
                }
                _ => {
                    println!("[{:?}] Other: {:?}", id, event);
                }
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(8));
    }
}
