#ifndef _IGNTION_INVALID_INDEX_EXCEPTION_HH_
#define _IGNTION_INVALID_INDEX_EXCEPTION_HH_

#include <stdexcept>

namespace ignition
{
  namespace math
  {
    /// \brief Exception that is thrown when an out-of-bounds index is
    /// encountered.
    class IndexException : public std::runtime_error
    {
      public: IndexException() : std::runtime_error("Invalid index")
              {}
    };
  }
}
#endif
