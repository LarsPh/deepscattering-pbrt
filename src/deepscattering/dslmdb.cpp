// WZR:
// deepscattering/dslmdb.cpp

#include "deepscattering/dslmdb.h"

namespace pbrt {
MDB_env* DsLMDB::env = nullptr;
MDB_dbi DsLMDB::dbi = 0;
int DsLMDB::counter = 0;

void DsLMDB::OpenEnv() {
    if (const int rc = mdb_env_create(&env))
        Error("fail to create lmdb environment.");
    mdb_env_set_maxdbs(env, 50);
    // 1MB * 5000 = 5GB
    mdb_env_set_mapsize(env, (size_t)1048576 * (size_t)5000);
    if (const int rc = mdb_env_open(
            env,
            "D:/Computer "
            "Science/UJiangnanGraduationProject/Contents/Advanced/DL&Graphics/"
            "DeepScattering/houdini_projects/Cloud/deepscattering_db/db_49_demo.lmdb",
            MDB_NOSUBDIR, 0)) {
        //mdb_env_close(env);
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
    ++counter;
    MDB_val key = {sizeof(int), &counter};
    MDB_val val = {valSize, valData};
    if (const int rc = mdb_put(transaction, dbi, &key, &val, 0)) {
        Error("fail to write data. return value ", rc);
    }
    mdb_txn_commit(transaction);
}
}  // namespace pbrt