#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <variant>

namespace mca::config {

using Value = std::variant<bool, int, double, std::string>;

class Parameters {
 public:
  void set(std::string name, Value value) { values_[std::move(name)] = std::move(value); }

  bool has(const std::string& name) const noexcept {
    return values_.find(name) != values_.end();
  }

  const Value* find(const std::string& name) const noexcept {
    const auto it = values_.find(name);
    return it == values_.end() ? nullptr : &it->second;
  }

  template <typename T>
  bool get(const std::string& name, T& output) const noexcept {
    const auto* value = find(name);
    if (value == nullptr) return false;
    const auto* typed = std::get_if<T>(value);
    if (typed == nullptr) return false;
    output = *typed;
    return true;
  }

 private:
  std::unordered_map<std::string, Value> values_;
};

}  // namespace mca::config
