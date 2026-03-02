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
                EventType::AxisChanged(axis, value, _) => {
                    println!("[{:?}] AxisChanged:   {:?} = {:.4}", id, axis, value);
                }
                EventType::ButtonChanged(button, value, _) => {
                    println!("[{:?}] ButtonChanged: {:?} = {:.4}", id, button, value);
                }
                EventType::ButtonPressed(button, _) => {
                    println!("[{:?}] ButtonPressed: {:?}", id, button);
                }
                EventType::ButtonReleased(button, _) => {
                    println!("[{:?}] ButtonReleased: {:?}", id, button);
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
