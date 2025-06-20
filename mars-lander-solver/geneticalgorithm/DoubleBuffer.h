#pragma once

#include <vector>

namespace geneticalgorithm {

  template<class T>
    class DoubleBuffer
    {
      bool idx_;
      T buff_[2];

    public:
      DoubleBuffer() : idx_{ false } {}

      auto&
      curr() { return buff_[!idx_]; }

      auto&
      prev() { return buff_[idx_]; }

      void
      flip() { idx_ = !idx_; }
    };

} // geneticalgorithm
