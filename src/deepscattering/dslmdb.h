// WZR: A one write per transaction database specialized for saving
// deepscattering data

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_DEEPSCATTERING_DSLMDB_H
#define PBRT_DEEPSCATTERING_DSLMDB_H

#include <lmdb.h>
#include <unordered_set>

#include "pbrt.h"

namespace pbrt {
class DsLMDB {
  public:
    static void OpenEnv(const char *path);
    void TxnWrite(void *valData, size_t valSize);

    static void tmpCount() { ++tmpCounter; }
    static void tmpPrint() {
        std::cout << "\nnumber of non-zero records:" << tmpCounter << std::endl;
    }

  private:
    static MDB_env *env;
    static MDB_dbi dbi;

    static long tmpCounter;
    
    static std::unordered_set<int>::iterator it;
    static std::unordered_set<int> keys;
    static void createKeys(int n);
};
}  // namespace pbrt

#endif  // PBRT_DEEPSCATTERING_DSLMDB_H