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
    void TxnWrite(Float *valData, long nT, int *tileSamplesNs, size_t valSize);

    static void tmpCount1() { ++tmpCounter1; }
    static void tmpCount2() { ++tmpCounter2; }
    static void tmpPrint() {
        std::cout << "\nnumber of non-zero records:" << tmpCounter1;
        std::cout << "\nnumber of zero records:" << tmpCounter2;
    }

  private:
    static std::string path;
    static MDB_env *env;
    static MDB_dbi dbi;

    static long tmpCounter1;
    static long tmpCounter2;

    static std::unordered_set<int>::iterator it;
    static std::unordered_set<int> keys;
    static void createKeys(int n);
    static long keyCounter;
};
}  // namespace pbrt

#endif  // PBRT_DEEPSCATTERING_DSLMDB_H