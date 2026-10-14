// wj: minimal fixed-buffer JSON read helpers for web request bodies (Step 8).
//
// The specification (3.11 internal constraints) requires fixed-size buffers
// and no heap allocation in request handlers. The ArduinoJson 7.2.1 in
// this project only exposes its default malloc-based allocator, so POST
// bodies are parsed with these small, allocation-free scanners instead:
//
//  - findValue(): locates "key" (at ANY nesting depth; all current request
//    bodies are flat, so this is equivalent to a top-level lookup) and
//    returns a pointer + length of the raw value substring (string, number,
//    bool, null, object or array - nesting aware, escape aware);
//  - toInt/toFloat/toBool/toStr: strict typed readers for that substring;
//  - arrayObjs(): splits an array value into its top-level `{ ... }`
//    elements (used for the schedule slots array).
//
// All functions reject malformed input by returning false/0 and write
// nothing beyond the caller-provided bounds. No allocation, no locks:
// every call is made on a buffer owned (copied) by the caller.
#ifndef ACID_DOSE_WEB_JSON_H
#define ACID_DOSE_WEB_JSON_H

#include <stddef.h>
#include <stdint.h>

namespace wj {

// One element (raw substring) of a top-level object inside a JSON array.
struct Elem {
    const char *p = nullptr;
    size_t len = 0;
};

// Finds "key" at the top level of the object `json` and returns a pointer
// to the raw value with its length in `vlen`. Returns nullptr when the key
// is missing. `json` must be NUL-safe within `len` (NUL terminator not
// required but tolerated).
const char *findValue(const char *json, size_t len, const char *key, size_t &vlen);

// Strict integer parse (optional sign, digits only).
bool toInt(const char *v, size_t len, long &out);
// Strict float parse (JSON number grammar, no inf/nan tokens).
bool toFloat(const char *v, size_t len, float &out);
// "true" / "false" (case sensitive JSON keywords).
bool toBool(const char *v, size_t len, bool &out);
// JSON string parse with minimal escape decoding; out must fit `cap`
// bytes (NUL included). Returns false on overflow or malformed input.
bool toStr(const char *v, size_t len, char *out, size_t cap);

// Splits a JSON array value into its top-level object elements (max `max`
// stored). Returns the number of captured elements (may be 0).
uint8_t arrayObjs(const char *v, size_t len, Elem *elems, uint8_t max);

} // namespace wj

#endif /* ACID_DOSE_WEB_JSON_H */
