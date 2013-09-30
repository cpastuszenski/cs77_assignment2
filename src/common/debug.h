#ifndef DEBUG_H_
#define DEBUG_H_

///@file common/debug.h Debugging facilities @ingroup common
///@defgroup debug Debugging facilities
///@ingroup common
///@{

/// Prints a message
void message(const char* msg);
/// Prints a message (printf style)
void message_va(const char* msg, ...);

/// Prints a warning
void warning(const char* msg);
/// Prints a warning (printf style)
void warning_va(const char* msg, ...);

/// Prints a warning if a condition does not hold
bool warning_if_not(bool check, const char* msg);
/// Prints a warning if a condition does not hold (printf style)
bool warning_if_not_va(bool check, const char* msg, ...);

/// Prints an error message and stops
void error(const char* msg);
/// Prints an error message and stops (printf style)
void error_va(const char* msg, ...);

/// Prints an error message and stops if a condition does not hold
bool error_if_not(bool check, const char* msg);
/// Prints an error message and stops if a condition does not hold (printf style)
bool error_if_not_va(bool check, const char* msg, ...);

/// Break into the debugger
void debug_break();

/// Error signaling unimplemented features
void not_implemented_error();

/// Error signaling unimplemented features
void put_your_code_here(const char *txt);
void _put_your_code_here(const char *file, int line, const char *function, const char *txt);
#undef put_your_code_here

/// "Overloads" put_your_code_here function to insert file, line, and calling function information
#define put_your_code_here(txt) \
    do { _put_your_code_here(__FILE__, __LINE__, __FUNCTION__, txt); } while(0);

///@}

#endif

