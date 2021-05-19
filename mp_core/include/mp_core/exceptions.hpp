#ifndef MP_CORE__EXCEPTIONS_HPP_
#define MP_CORE__EXCEPTIONS_HPP_ 

#include <stdexcept>
#include <string>
#include <memory>

namespace mp_core
{
class PlannerException : public std::runtime_error
{
public:
    explicit PlannerException(const std::string description)
    : std::runtime_error(description) {}
    using Ptr = std::shared_ptr<PlannerException>;
};

} // namespace mp_core

#endif // MP_CORE__EXCEPTIONS_HPP_