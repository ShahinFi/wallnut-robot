// Local (gitignored) secrets for the Mega firmware.
// Copy to `src/mega/secrets.h` and edit.
//
// NOTE: This is not cryptographically secure. It just keeps the passcode out of
// the public repo history.
#pragma once

// UART passcode required to ARM the robot from the web UI.
// ESP forwards this to Mega as: `Passcode:<digits>`.
#define ESP_PASSCODE_STR "1234"
#define ESP_PASSCODE_INT 1234

