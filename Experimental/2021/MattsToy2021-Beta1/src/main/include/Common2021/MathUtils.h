#pragma once

/// Returns the sign of the specified value: +1, 0, or -1
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}
