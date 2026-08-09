#ifndef TOOLS__PATH_HPP
#define TOOLS__PATH_HPP

#include <filesystem>
#include <string>

namespace tools
{
inline std::string resolve_path(const std::string & config_path, const std::string & path)
{
  namespace fs = std::filesystem;

  fs::path candidate(path);
  if (candidate.is_absolute() || fs::exists(candidate)) {
    return candidate.string();
  }

  fs::path config_file(config_path);
  auto config_dir = config_file.parent_path();
  if (!config_dir.empty()) {
    auto from_config_dir = config_dir / candidate;
    if (fs::exists(from_config_dir)) {
      return from_config_dir.string();
    }

    auto from_package_dir = config_dir.parent_path() / candidate;
    if (fs::exists(from_package_dir)) {
      return from_package_dir.string();
    }
  }

  return candidate.string();
}
}  // namespace tools

#endif  // TOOLS__PATH_HPP
