#pragma once
#include <cstdint>
#include <stdexcept>
#include <string>

namespace abu2023 {
class AimAssistantException : public std::runtime_error {
public:
  AimAssistantException(const std::string errorDescription) : std::runtime_error(errorDescription) {}
};

class EmptyPointCloudException : public AimAssistantException {
public:
  EmptyPointCloudException(const std::string errorDescription) : AimAssistantException(errorDescription) {}
};

// class 
} // namespace abu2023