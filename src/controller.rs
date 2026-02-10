use anyhow::Result;
use gilrs::{Gilrs, Event, EventType, Axis};
use std::collections::HashMap;

/// Controller state containing all axes and button states
#[derive(Debug, Clone)]
pub struct ControllerState {
    /// Left stick X axis (-1.0 to 1.0)
    pub left_stick_x: f32,
    /// Left stick Y axis (-1.0 to 1.0)
    pub left_stick_y: f32,
    /// Right stick X axis (-1.0 to 1.0)
    pub right_stick_x: f32,
    /// Right stick Y axis (-1.0 to 1.0)
    pub right_stick_y: f32,
    /// Left trigger (0.0 to 1.0)
    pub left_trigger: f32,
    /// Right trigger (0.0 to 1.0)
    pub right_trigger: f32,
    /// D-pad horizontal (-1.0 = left, 1.0 = right)
    pub dpad_x: f32,
    /// D-pad vertical (-1.0 = down, 1.0 = up)
    pub dpad_y: f32,
    /// Button states
    pub buttons: HashMap<String, bool>,
}

impl Default for ControllerState {
    fn default() -> Self {
        Self {
            left_stick_x: 0.0,
            left_stick_y: 0.0,
            right_stick_x: 0.0,
            right_stick_y: 0.0,
            left_trigger: 0.0,
            right_trigger: 0.0,
            dpad_x: 0.0,
            dpad_y: 0.0,
            buttons: HashMap::new(),
        }
    }
}

pub struct Controller {
    gilrs: Gilrs,
    state: ControllerState,
    gamepad_id: Option<gilrs::GamepadId>,
}

impl Controller {
    /// Create a new controller instance
    pub fn new() -> Result<Self> {
        let gilrs = Gilrs::new()
            .map_err(|e| anyhow::anyhow!("Failed to initialize gamepad system: {}", e))?;

        Ok(Self {
            gilrs,
            state: ControllerState::default(),
            gamepad_id: None,
        })
    }

    /// Check if a controller is connected
    pub fn is_connected(&self) -> bool {
        self.gamepad_id.is_some()
    }

    /// Get the name of the connected controller
    pub fn get_controller_name(&self) -> Option<String> {
        self.gamepad_id.map(|id| {
            self.gilrs.gamepad(id).name().to_string()
        })
    }

    /// Wait for a controller to be connected
    pub fn wait_for_connection(&mut self) -> Result<()> {
        println!("Waiting for Xbox controller to connect...");

        loop {
            // Check for already connected gamepads
            for (_id, gamepad) in self.gilrs.gamepads() {
                if gamepad.is_connected() {
                    self.gamepad_id = Some(gamepad.id());
                    println!("✓ Controller connected: {}", gamepad.name());
                    return Ok(());
                }
            }

            // Process events to detect new connections
            while let Some(Event { id, event, .. }) = self.gilrs.next_event() {
                match event {
                    EventType::Connected => {
                        self.gamepad_id = Some(id);
                        let name = self.gilrs.gamepad(id).name().to_string();
                        println!("✓ Controller connected: {}", name);
                        return Ok(());
                    }
                    _ => {}
                }
            }

            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }

    /// Update controller state by processing events
    pub fn update(&mut self) -> Result<()> {
        // Process all pending events
        while let Some(Event { id, event, .. }) = self.gilrs.next_event() {
            // Only process events from our connected gamepad
            if Some(id) != self.gamepad_id {
                continue;
            }

            match event {
                EventType::ButtonPressed(button, _) => {
                    let button_name = format!("{:?}", button);
                    self.state.buttons.insert(button_name, true);
                }
                EventType::ButtonReleased(button, _) => {
                    let button_name = format!("{:?}", button);
                    self.state.buttons.insert(button_name, false);
                }
                EventType::AxisChanged(axis, value, _) => {
                    match axis {
                        Axis::LeftStickX => self.state.left_stick_x = value,
                        Axis::LeftStickY => self.state.left_stick_y = value,
                        Axis::RightStickX => self.state.right_stick_x = value,
                        Axis::RightStickY => self.state.right_stick_y = value,
                        Axis::LeftZ => self.state.left_trigger = (value + 1.0) / 2.0, // Map -1..1 to 0..1
                        Axis::RightZ => self.state.right_trigger = (value + 1.0) / 2.0,
                        Axis::DPadX => self.state.dpad_x = value,
                        Axis::DPadY => self.state.dpad_y = value,
                        _ => {}
                    }
                }
                EventType::Disconnected => {
                    println!("⚠ Controller disconnected");
                    self.gamepad_id = None;
                    return Err(anyhow::anyhow!("Controller disconnected"));
                }
                _ => {}
            }
        }

        Ok(())
    }

    /// Get the current controller state
    pub fn get_state(&self) -> &ControllerState {
        &self.state
    }

    /// Check if a specific button is pressed
    pub fn is_button_pressed(&self, button_name: &str) -> bool {
        self.state.buttons.get(button_name).copied().unwrap_or(false)
    }

    /// Apply deadzone to a value
    pub fn apply_deadzone(value: f32, deadzone: f32) -> f32 {
        if value.abs() < deadzone {
            0.0
        } else {
            // Scale the remaining range
            let sign = value.signum();
            let abs_val = value.abs();
            sign * (abs_val - deadzone) / (1.0 - deadzone)
        }
    }
}
