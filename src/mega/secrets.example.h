// SECTION: Local (gitignored) Mega secrets template.
// WHY: Copy to `src/mega/secrets.h` and edit values locally.
// CONTRACT: This hides credentials from repository history but is not cryptographic security.
#pragma once

// WHY: UART passcode required for web UI arming flow.
// CONTRACT: ESP forwards this value as `Passcode:<digits>`.
#define ESP_PASSCODE_STR "1234"
#define ESP_PASSCODE_INT 1234


