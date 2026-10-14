#include "web_json.h"

#include <ctype.h>
#include <string.h>

namespace wj {

static bool isJsonWs(char c) {
    return (c == ' ' || c == '\t' || c == '\n' || c == '\r');
}

static size_t skipWs(const char *s, size_t len, size_t i) {
    while (i < len && isJsonWs(s[i])) {
        i++;
    }
    return i;
}

// Copies up to cap-1 bytes of the string content (with escape decoding)
// into out, assumes the value starts at v[0] == '"' and the closing quote
// is within len. Returns false on overflow or a dangling final escape.
static bool copyString(const char *v, size_t len, char *out, size_t cap) {
    if (len < 2 || v[0] != '"' || v[len - 1] != '"') {
        return false;
    }
    size_t o = 0;
    for (size_t i = 1; i < len; i++) {
        char c = v[i];
        if (c == '"') {
            // Closing quote: everything after it must be whitespace.
            for (size_t j = i + 1; j < len; j++) {
                if (!isJsonWs(v[j])) {
                    return false;
                }
            }
            out[o] = '\0';
            return true;
        }
        if (c == '\\') {
            i++;
            if (i >= len) {
                return false; // dangling escape
            }
                        const char e = v[i];
            unsigned code = 0;
            switch (e) {
                case '"':  c = '"';  break;
                case '\\': c = '\\'; break;
                case '/':  c = '/';  break;
                case 'b':  c = '\b'; break;
                case 'f':  c = '\f'; break;
                case 'n':  c = '\n'; break;
                case 'r':  c = '\r'; break;
                case 't':  c = '\t'; break;
                case 'u':
                    // \uXXXX: i is at 'u'; hex digits are v[i+1..i+4].
                    // Decode ASCII-range codepoints (sufficient for
                    // credentials/addresses); anything else becomes '?'.
                                        if (i + 4 >= len) {
                        return false;
                    }
                    code = 0;
                    for (unsigned k = 0; k < 4; k++) {
                        const char hx = v[i + 1 + k];
                        code <<= 4;
                        if (hx >= '0' && hx <= '9') {
                            code |= (unsigned)(hx - '0');
                        } else if (hx >= 'a' && hx <= 'f') {
                            code |= (unsigned)(hx - 'a' + 10);
                        } else if (hx >= 'A' && hx <= 'F') {
                            code |= (unsigned)(hx - 'A' + 10);
                        } else {
                            return false;
                        }
                    }
                    i += 4;
                    c = (code < 128) ? (char)code : '?';
                    break;
                default:
                    return false;
            }
        }
        if (o + 1 >= cap) {
            return false; // would overflow
        }
        out[o++] = c;
    }
    return false; // closing quote not found
}

// Copies a non-escaped string (no escape handling, fast path where the
// value contains no backslashes).
static bool copyStringPlain(const char *v, size_t len, char *out, size_t cap) {
    if (len < 2 || v[0] != '"' || v[len - 1] != '"') {
        return false;
    }
    const size_t content = len - 2;
    if (content + 1 > cap) {
        return false;
    }
    // The closing quote is at v[len-1]; findValue() trims trailing
    // whitespace from the raw value, so there is nothing after it to
    // validate here (an earlier version mistakenly scanned v[content..len-1],
    // which covers the string content tail plus the quote itself, and so
    // rejected every non-empty string - e.g. all /api/wifi SSIDs).
    memcpy(out, v + 1, content);
    out[content] = '\0';
    return true;
}

bool toStr(const char *v, size_t len, char *out, size_t cap) {
    if (out == nullptr || cap == 0) {
        return false;
    }
    if (memchr(v, '\\', len) != nullptr) {
        return copyString(v, len, out, cap);
    }
    return copyStringPlain(v, len, out, cap);
}

bool toInt(const char *v, size_t len, long &out) {
    size_t i = 0;
    bool neg = false;
    if (i < len && (v[i] == '-' || v[i] == '+')) {
        neg = (v[i] == '-');
        i++;
    }
    if (i >= len) {
        return false;
    }
    long val = 0;
    for (; i < len; i++) {
        if (v[i] < '0' || v[i] > '9') {
            return false;
        }
        val = val * 10 + (long)(v[i] - '0');
        if (val > 2147483647L) {
            // limits: values are capped at the signed 32-bit range; do not
            // use toInt() for u32 fields that may exceed INT_MAX.
            return false;
        }
    }
    if (neg && val > 2147483648L) {
        return false;
    }
    out = neg ? -val : val;
    return true;
}

bool toFloat(const char *v, size_t len, float &out) {
    if (len == 0) {
        return false;
    }
    const char *wsEnd = v + len;
    while (wsEnd > v && isJsonWs(*(wsEnd - 1))) {
        wsEnd--;
    }
    const size_t real = (size_t)(wsEnd - v);
    if (real == 0) {
        return false;
    }
    char buf[32]; // limits: numeric field capped at 31 chars (longer floats not needed)
    if (real >= sizeof(buf)) {
        return false;
    }
    memcpy(buf, v, real);
    buf[real] = '\0';
    char *end = nullptr;
    const double d = strtod(buf, &end);
    if (end != buf + real || buf[0] == '\0') {
        return false;
    }
    if (d != d || d == (double)1e308 * 1e308) {
        return false; // NaN/inf: JSON must not carry these
    }
    out = (float)d;
    return true;
}

bool toBool(const char *v, size_t len, bool &out) {
    if (len == 4 && memcmp(v, "true", 4) == 0) {
        out = true;
        return true;
    }
    if (len == 5 && memcmp(v, "false", 5) == 0) {
        out = false;
        return true;
    }
    return false;
}

// Returns the end offset (index just past) of a JSON value that starts at
// s[i] within s[0..len). Handles strings (escapes), objects and arrays.
// Returns false on truncation.
static bool scanValue(const char *s, size_t len, size_t i, bool &isString,
                      size_t &outEnd) {
    isString = false;
    if (i >= len) {
        return false;
    }
    const char c = s[i];
    if (c == '"') {
        isString = true;
        i++;
        while (i < len) {
            if (s[i] == '\\') {
                i += 2;
                continue;
            }
            if (s[i] == '"') {
                outEnd = i + 1;
                return true;
            }
            i++;
        }
        return false; // unterminated string
    }
    if (c == '{' || c == '[') {
        int depth = 0;
        bool inStr = false;
        for (; i < len; i++) {
            const char d = s[i];
            if (inStr) {
                if (d == '\\') {
                    i++;
                } else if (d == '"') {
                    inStr = false;
                }
                continue;
            }
            if (d == '"') {
                inStr = true;
            } else if (d == '{' || d == '[') {
                depth++;
            } else if (d == '}' || d == ']') {
                depth--;
                if (depth == 0) {
                    outEnd = i + 1;
                    return true;
                }
            }
        }
        return false; // unterminated container
    }
    // Number / true / false / null: run until JSON separator.
    while (i < len) {
        const char d = s[i];
        if (d == ',' || d == '}' || d == ']' || isJsonWs(d)) {
            break;
        }
        i++;
    }
    outEnd = i;
    return (outEnd > (size_t)0);
}

static bool keyEq(const char *s, size_t i, size_t len, const char *key,
                  size_t klen) {
    if (i + 1 + klen + 1 > len) {
        return false;
    }
    if (s[i] != '"' || memcmp(s + i + 1, key, klen) != 0) {
        return false;
    }
    if (s[i + 1 + klen] != '"') {
        return false;
    }
    size_t j = i + 2 + klen;
    j = skipWs(s, len, j);
    return (j < len && s[j] == ':');
}

const char *findValue(const char *json, size_t len, const char *key,
                      size_t &vlen) {
    if (json == nullptr || key == nullptr) {
        vlen = 0;
        return nullptr;
    }
    const size_t klen = strlen(key);
    const size_t token = klen + 2; // with quotes
    for (size_t i = 0; i + token < len; i++) {
        if (json[i] != '"') {
            continue;
        }
        if (!keyEq(json, i, len, key, klen)) {
            continue;
        }
        size_t j = i + token; // points at ':'
        j = skipWs(json, len, j + 1);
        if (j >= len) {
            continue;
        }
        bool isStr = false;
        size_t end = 0;
        if (!scanValue(json, len, j, isStr, end)) {
            continue;
        }
        vlen = end - j;
        // Trim trailing whitespace from the raw value.
        while (vlen > 0 && isJsonWs(json[j + vlen - 1])) {
            vlen--;
        }
        return json + j;
    }
    vlen = 0;
    return nullptr;
}

uint8_t arrayObjs(const char *v, size_t len, Elem *elems, uint8_t max) {
    if (v == nullptr || elems == nullptr || max == 0 || len == 0 || v[0] != '[') {
        return 0;
    }
    uint8_t n = 0;
    size_t i = 1;
    while (n < max) {
        i = skipWs(v, len, i);
        if (i >= len) {
            break;
        }
        if (v[i] == ']') {
            break; // array end
        }
        if (v[i] != '{') {
            i++; // tolerate non-object elements by skipping one char
            continue;
        }
        bool isStr = false;
        size_t end = 0;
        if (!scanValue(v, len, i, isStr, end)) {
            break; // truncated element
        }
        elems[n].p = v + i;
        elems[n].len = end - i;
        n++;
        i = end;
    }
    return n;
}

} // namespace wj
