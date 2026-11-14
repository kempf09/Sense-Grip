// JSON string escaping. Pure C++, no Arduino headers.
#pragma once
#include <stddef.h>

// Escapes " \ and control chars (<0x20 as \u00XX) from NUL-terminated `in` into `out`.
// Always NUL-terminates when cap > 0. Stops before an escape sequence that would not
// fit (never emits a partial one). Returns bytes written, excluding the NUL.
size_t jsonEscape(const char* in, char* out, size_t cap);
