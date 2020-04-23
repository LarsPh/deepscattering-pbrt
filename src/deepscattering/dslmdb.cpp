// WZR:
// deepscattering/dslmdb.cpp

#include "deepscattering/dslmdb.h"

#include <climits>
#include <random>

namespace pbrt {
MDB_env* DsLMDB::env = nullptr;
MDB_dbi DsLMDB::dbi = 0;
long DsLMDB::tmpCounter1 = 0;
long DsLMDB::tmpCounter2 = 0;
std::unordered_set<int> DsLMDB::keys = {};
std::unordered_set<int>::iterator DsLMDB::it =
    std::unordered_set<int>::iterator();

void DsLMDB::createKeys(int n) {
    if (n >= INT_MAX / 2) Error("too many to efficiently generate");
    // int max = int(n * 1.01);
    int max = int(n * 1); // takes longer time
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, max);
    int counter = 0;
    while (counter < n) {
        ++counter;
        while (keys.size() < counter) keys.insert(distribution(generator));
    }
    CHECK_EQ(keys.size(), n);
    it = keys.begin();
}

void DsLMDB::OpenEnv(const char *path) {
    // create random numbers
    createKeys(1200 * 2400);
    if (const int rc = mdb_env_create(&env))
        Error("fail to create lmdb environment.");
    mdb_env_set_maxdbs(env, 50);
    // 1GB * 36 = 36GB
    mdb_env_set_mapsize(env, (size_t)1073741824 * (size_t)36);
    if (const int rc = mdb_env_open(env, path, 0, 0)) {
        // mdb_env_close(env);
        Error("fail to open lmdb environment.");
    }
    MDB_txn *transaction;
    if (const int rc = mdb_txn_begin(env, nullptr, 0, &transaction)) {
        Error("fail to open first transaction.");
    }

    if (const int rc = mdb_dbi_open(transaction, nullptr, MDB_CREATE, &dbi)) {
        Error("fail to open database.");
    }
    mdb_txn_commit(transaction);
}
void DsLMDB::TxnWrite(void *valData,
                      size_t valSize) {
    MDB_txn *transaction;
    if (const int rc = mdb_txn_begin(env, nullptr, 0, &transaction)) {
        Error("fail to open transaction for a write.");
    }    
    int intKey = *it;
    ++it;
    MDB_val key = {sizeof(int), &intKey};
    MDB_val val = {valSize, valData};
    if (const int rc = mdb_put(transaction, dbi, &key, &val, 0)) {
        std::string strRc =
            std::string("fail to write data. return value ") + std::to_string(rc);
        Error(strRc.c_str());
    }
    mdb_txn_commit(transaction);
}
}  // namespace pbrt