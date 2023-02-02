#ifndef MICROROS_UTILS_H
#define MICROROS_UTILS_H

#include <rcl/rcl.h>

/// @brief Macro function for checking error.
#define RCCHECK(fn)                                      \
  {                                                      \
    rcl_ret_t temp_rc = fn;                              \
    if ((temp_rc != RCL_RET_OK))                         \
    {                                                    \
      error_loop(temp_rc, __FILE__, __LINE__, __func__); \
    }                                                    \
  }

/// @brief Macro function for checking error.
#define RCSOFTCHECK(fn)                        \
  {                                            \
    rcl_ret_t temp_rc = fn;                    \
    if ((temp_rc != RCL_RET_OK))               \
    {                                          \
      error_once(temp_rc, __FILE__, __LINE__, __func__); \
    }                                          \
  }

/**
 * @brief Infinite loop to enter when unsolvable error happens, blink the built-in led in "sos" morse code.
 * @param[in] _rc Error code.
 * @param[in] _file Name of the file.
 * @param[in] _line Number of line.
 * @param[in] _fun Name of function.
 */
void error_loop(const rcl_ret_t _rc, const char *_file, const int _line, const char *_fun);

/**
 * @brief Function to enter when some minor error happens, blink the built-in led once.
 * @param[in] _rc Error code.
 * @param[in] _file Name of the file.
 * @param[in] _line Number of line.
 * @param[in] _fun Name of function.
 */
void error_once(const rcl_ret_t _rc, const char *_file, const int _line, const char *_fun);

/// @brief Function to configure microros transport and allocator.
void configure_microros();

#endif // MICROROS_UTILS_H
