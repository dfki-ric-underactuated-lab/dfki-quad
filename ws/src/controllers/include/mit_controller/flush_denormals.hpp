#pragma once
#include <cstdint>

#if defined(__x86_64__)
#include <xmmintrin.h>
#endif

/// Flushes subnormal floating-point numbers (|x| < 2.2e-308 for doubles) to zero in the calling thread while
/// this object lives, and restores the previous mode afterwards. x86 CPUs compute with subnormal numbers on a
/// slow microcode path, and warm-started first-order QP solvers (OSQP, ProxQP) can accumulate them. The mode is
/// per thread, so keeping it scoped leaves other callbacks that run on the same executor thread unaffected.
class ScopedFlushDenormals {
 public:
  explicit ScopedFlushDenormals(bool enable) : active_(enable && Supported()) {
    if (!active_) {
      return;
    }
#if defined(__x86_64__)
    saved_ = _mm_getcsr();
    _mm_setcsr(static_cast<unsigned int>(saved_) | MXCSR_FTZ | MXCSR_DAZ);
#elif defined(__aarch64__)
    asm volatile("mrs %0, fpcr" : "=r"(saved_) : : "memory");
    asm volatile("msr fpcr, %0" : : "r"(saved_ | FPCR_FZ) : "memory");
#endif
  }

  ~ScopedFlushDenormals() {
    if (!active_) {
      return;
    }
#if defined(__x86_64__)
    _mm_setcsr(static_cast<unsigned int>(saved_));
#elif defined(__aarch64__)
    asm volatile("msr fpcr, %0" : : "r"(saved_) : "memory");
#endif
  }

  ScopedFlushDenormals(const ScopedFlushDenormals &) = delete;
  ScopedFlushDenormals &operator=(const ScopedFlushDenormals &) = delete;

  /// Whether flushing is implemented for the CPU architecture this was compiled for.
  static constexpr bool Supported() {
#if defined(__x86_64__) || defined(__aarch64__)
    return true;
#else
    return false;
#endif
  }

 private:
  // MXCSR: flush subnormal results to zero (FTZ, bit 15), treat subnormal inputs as zero (DAZ, bit 6)
  static constexpr unsigned int MXCSR_FTZ = 1u << 15;
  static constexpr unsigned int MXCSR_DAZ = 1u << 6;
  // FPCR: flush subnormal inputs and results to zero (FZ, bit 24)
  static constexpr std::uint64_t FPCR_FZ = std::uint64_t{1} << 24;

  bool active_;
  std::uint64_t saved_ = 0;
};
