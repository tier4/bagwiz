// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__IO__FILE_DECOMPRESSOR_HPP_
#define BAGWIZ__IO__FILE_DECOMPRESSOR_HPP_

#include <cstddef>
#include <filesystem>
#include <span>
#include <utility>

namespace bagwiz::io
{

// RAII owner of a temporary file on disk. The destructor unlinks the path it
// owns; a default-constructed (empty) instance owns nothing and is a no-op on
// destruction. Move-only — the underlying file has exactly one owner so its
// lifetime is unambiguous.
//
// Used to back FILE-mode (`compression_mode: FILE`) zstd bags: the on-disk
// `.db3.zstd` envelope is decompressed to one of these temp files, which the
// owning BagReader keeps alive for the duration of iteration and deletes when
// it is destroyed.
class TempFile
{
public:
  TempFile() = default;
  explicit TempFile(std::filesystem::path path) : path_(std::move(path)) {}

  ~TempFile() { remove(); }

  TempFile(const TempFile &) = delete;
  TempFile & operator=(const TempFile &) = delete;

  TempFile(TempFile && other) noexcept : path_(std::move(other.path_)) { other.path_.clear(); }

  TempFile & operator=(TempFile && other) noexcept
  {
    if (this != &other) {
      remove();
      path_ = std::move(other.path_);
      other.path_.clear();
    }
    return *this;
  }

  // The owned path. Empty when this instance owns nothing.
  [[nodiscard]] const std::filesystem::path & path() const noexcept { return path_; }

  // True when this instance owns a path.
  [[nodiscard]] bool valid() const noexcept { return !path_.empty(); }

private:
  void remove() noexcept
  {
    if (!path_.empty()) {
      std::error_code ec;
      std::filesystem::remove(path_, ec);
      path_.clear();
    }
  }

  std::filesystem::path path_;
};

// True when `bytes` begins with the zstd frame magic (0x28 0xB5 0x2F 0xFD).
// Returns false for spans shorter than 4 bytes.
[[nodiscard]] bool is_zstd_magic(std::span<const std::byte> bytes) noexcept;

// True when `path`'s first bytes match the zstd frame magic. Returns false on
// any IO error or short read — callers treat that as "not a zstd file".
[[nodiscard]] bool is_zstd_file(const std::filesystem::path & path) noexcept;

// Stream-decompress the whole zstd file at `src` into a freshly created
// temporary file and return its owner. The temp file lives in the system temp
// directory (honouring TMPDIR) and inherits the inner extension of `src`
// (e.g. `foo.db3.zstd` -> a temp file ending in `.db3`) so downstream sniffing
// by extension still works.
//
// Throws std::runtime_error on a malformed frame or any IO failure. The
// caller owns the returned TempFile; the file is removed when it is destroyed.
[[nodiscard]] TempFile decompress_zstd_file_to_temp(const std::filesystem::path & src);

}  // namespace bagwiz::io

#endif  // BAGWIZ__IO__FILE_DECOMPRESSOR_HPP_
