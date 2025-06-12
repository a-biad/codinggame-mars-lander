#pragma once

using Float = double;

constexpr Float operator ""_f (long double inValue) { return Float(inValue); }
