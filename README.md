# Gesture_Unlock
## Overview
The project uses a Dynamic Time Warping (DTW) algorithm to match time series data. This means it can successfully identify movements that are performed at different speeds or amplitudes. The recorded sequence is saved on the microcontroller(tested at STM32F429) using a "Record Key" feature. The user can then attempt to unlock the system by replicating the key sequence. An LED or similar indicator provides visual feedback on whether the unlocking attempt was successful.

## Operation
The system works in two main modes: record and unlock.

Record Mode: In this mode, the user can record a specific sequence of hand movements. To enter record mode, press the enter key after a reset. This starts the data recording process. Once the desired sequence has been completed, press the enter key again to finish the recording.

Unlock Mode: In this mode, the user attempts to unlock the system by replicating the previously recorded hand movement sequence. Press the enter key to start the unlock process. If the sequence is correctly replicated within the defined tolerances, the LED indicator will signal a successful unlock and the next press of the enter key will begin a new recording. If the sequence is not correctly replicated, the next press of the enter key will repeat the unlock attempt.

## Setup
This project was built using PlatformIO and should be compiled and uploaded through the same. The algo itself should be compatible with any microcontroller and accelerometer/gyro combination, but some parameter/pin adjustments might be necessary depending on the specifics of your hardware. Please refer to the comments in the code for guidance on this.

## Known Issues
Pressing the enter key excessively fast can lead to errors. Please allow a moment between keystrokes to ensure the system functions correctly.

## Feedback and Contributions
We welcome any feedback or suggestions for improvements. Please open an issue in the GitHub repository to discuss any changes you'd like to see, or any issues you encounter.
