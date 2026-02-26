# Bluetooth Command Protocol

Communication: Bluetooth Serial
Microcontroller: ESP32

## Commands
| Command |         Function        |
|---------|-------------------------|
|    F    |       Move forward      |
|    B    |      Move backward      |
|    L    |        Turn left        |
|    R    |         Turn right      |
|    S    |            Stop         |
|    A    |   Line-following mode   |
|    D    | Obstacle avoidance mode |
|    I    |    Clockwise rotation   |
|    C    |     Counter request     |
|    0    |      Reset counter      |

This ESP32 processes each command  and switches operational state accordingly.

# Mobile Application - MIT App Inventor
<img width="895" height="654" alt="image" src="https://github.com/user-attachments/assets/ec301bbf-56a2-4f9e-a8ef-f8d4d88bf58b" />

## Features 
- Manual movement control (Forward, Backward, Left, Right)
- Stop command
- Line-following mode activation
- Obstacle avoidance mode
- Counter display
- Counter reset
- Bluetooth connection management

## Application Logic
The app:

- Establishes Bluetooth connection
- Sends command-based control characters
- Receives feedback data (counter values)
- Updates UI elements dynamically
