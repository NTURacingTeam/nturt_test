/**
 * @file user_interface_data.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Data type definition for user interface.
 */

#ifndef UI_DATA_HPP
#define UI_DATA_HPP

// stl include
#include <mutex>

struct UserInterfaceData {
  /* control flag ------------------------------------------------------------*/
  std::mutex mutex;
};

#endif  // UI_DATA_HPP
