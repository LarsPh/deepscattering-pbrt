// WZR: A one write per transaction database specialized for saving
// deepscattering data

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_DEEPSCATTERING_DSLMDB_H
#define PBRT_DEEPSCATTERING_DSLMDB_H

#include <lmdb.h>

#include "pbrt.h"

namespace pbrt {
class DsLMDB {
  public:
    static void OpenEnv();
    static void Count() {
        ++counter;
        LOG(INFO) << counter;
    }
    void TxnWrite(void *valData, size_t valSize);

  private:
    static MDB_env *env;
    static MDB_dbi dbi;
    static int counter;
};
}  // namespace pbrt

#endif  // PBRT_DEEPSCATTERING_DSLMDB_H