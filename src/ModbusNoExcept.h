#pragma once

// Included last by each library source file, after every other include.
//
// The library throws nothing and catches nothing, so its own functions need no
// exception tables or unwind code: several kilobytes of flash on a board.
// Placed after the standard headers so they keep their own settings; an
// application handler that throws through the library ends the program, which
// on a controller is what an escaped exception does anyway.
//
// Not in the host transports (transport/), which still hold std::string and
// are compiled out on a board.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("no-exceptions")
#endif
