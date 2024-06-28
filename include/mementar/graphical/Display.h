#ifndef COLOR_OFF
#define COLOR_OFF "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED "\x1B[0;91m"
#endif
#ifndef COLOR_ORANGE
#define COLOR_ORANGE "\x1B[1;33m"
#endif
#ifndef COLOR_GREEN
#define COLOR_GREEN "\x1B[1;92m"
#endif
#ifndef COLOR_BLUE
#define COLOR_BLUE "\x1B[1;96m"
#endif

#ifndef MEMENTAR_DISPLAY_H
#define MEMENTAR_DISPLAY_H

#include <iomanip>
#include <iostream>

namespace mementar {

  class Display
  {
  public:
    static void error(const std::string& text)
    {
      std::cout << COLOR_RED << "[ERROR]" << text << COLOR_OFF << std::endl;
    }

    static void warning(const std::string& text)
    {
      std::cout << COLOR_ORANGE << "[WARNING]" << text << COLOR_OFF << std::endl;
    }

    static void info(const std::string& text)
    {
      std::cout << COLOR_BLUE << text << COLOR_OFF << std::endl;
    }

    static void debug(const std::string& text)
    {
      std::cout << text << std::endl;
    }

    static void percent(size_t percent)
    {
      std::cout << "=>" << std::setw(3) << percent << "%\r" << std::flush;
    }
  };

} // namespace mementar

#endif // MEMENTAR_DISPLAY_H
