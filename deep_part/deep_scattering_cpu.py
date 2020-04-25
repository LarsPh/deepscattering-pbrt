import torch
from torch.utils.tensorboard import SummaryWriter
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
import lmdb
import os
import numpy as np
import time

# load data

# create model
dZ = 5 * 5 * 9 + 1
nodeNum = 200
stencilNum = 10


class Block(torch.nn.Module):
    def __init__(self, dZ, dLOut, dOut):
        super(Block, self).__init__()
        self.v = torch.nn.Linear(dZ, dOut, bias=True)
        self.w1 = torch.nn.Linear(dLOut, dOut, bias=True)
        self.w2 = torch.nn.Linear(dOut, dOut, bias=True)
        self.a = torch.nn.ReLU()

    def forward(self, lOut, z):
        if (torch.isnan(z).any()):
            print("z contains nan")
        if (torch.isinf(z).any()):
            print("z contains inf")
        if (z.sum().data.item() == 0):
            print("z is zero")
        l1 = self.v(z).add_(self.w1(lOut))
        if (torch.isnan(l1).any()):
            print("l1 contains nan")
        a1 = self.a(l1)

        l2 = self.w2(a1).add_(lOut)

        a2 = self.a(l2)

        return a2


class DSModel(torch.nn.Module):
    def __init__(self):
        super(DSModel, self).__init__()
        self.blocks = self.buildBlocks()
        self.FC = self.buildFC().to(dev)

    def forward(self, z):
        batchSize = z.size()[0]
        out = torch.zeros((batchSize, nodeNum)).to(dev)
        for i, block in enumerate(self.blocks):
            out = block(out, z.narrow(2, i, 1).squeeze(2))
            # if (torch.isnan(out).any()):
            #     print("out", i, "contains nan")
        return self.FC(out)

    def buildBlocks(self):
        return torch.nn.ModuleList([Block(dZ, nodeNum, nodeNum).to(dev) for i in range(stencilNum)])

    def buildFC(self):
        return torch.nn.Sequential(
            torch.nn.Linear(nodeNum, nodeNum),
            torch.nn.ReLU(),
            torch.nn.Linear(nodeNum, nodeNum),
            torch.nn.ReLU(),
            torch.nn.Linear(nodeNum, 1),
            torch.nn.ReLU())  # ???

    def allTo(self, dev):
        self.to(dev)
        for block in self.blocks:
            block.to(dev)
        self.FC.to(dev)


class DsLMDB():
    def __init__(self, path, mapSize):
        # open env
        print("opening ", path)
        self.path = path
        self.env = lmdb.open(path, map_size=mapSize, readonly=True)

    def load(self, start, stop):
        # create txn, open db and load size to mem in arrary
        txn = self.env.begin()
        # return array
        pairs = {}  # type: {int: bytes}
        corruptedCount = 0
        for key in range(start, stop):
            bKey = key.to_bytes(4, byteorder='little')
            val = txn.get(bKey)
            if val is not None:
                pairs[key-start-corruptedCount] = val
            else:
                corruptedCount += 1
        print(corruptedCount, "records corrupted in",
              self.path[-8:].split('/')[1])
        txn.commit()
        return pairs

    def close(self):
        self.env.close()


class DsDataset(Dataset):
    def __init__(self, pairs):
        super(DsDataset, self).__init__()
        self.pairs = pairs
        self.infCount = 0
        self.nanCount = 0
        self.format()

    def __len__(self):
        return len(self.pairs)

    def format(self):
        for key, val in self.pairs.items():
            assert(len(val) == 2252 * 4)
            # 'f' stands for 32 bit float
            X = np.frombuffer(val, dtype='f', count=2251)
            y = np.frombuffer(val, dtype='f', count=1,
                              offset=2251*4)
            X, gamma = np.split(X, [2250])
            X = np.reshape(X, (225, 10))
            X = np.append(X, np.full((1, 10), gamma), 0)
            assert(np.shape(X) == (226, 10))

            isinf = np.isinf(X)
            isneginf = np.isneginf(X)
            if isinf.any() | isneginf.any():
                self.infCount += len(isinf[isinf is True]) + \
                    len(isinf[isneginf is True])
            X = np.clip(X, np.float32(-10000.0), np.float32(10000.0))

            if np.isnan(X).any():
                self.cleanNan(X)
                self.nanCount += 1

            assert(np.isfinite(X).all())
            assert(not np.isnan(X).any())
            self.pairs[key] = (X, y)
            # self.pairs[key][0].setflags(write=1)
            # self.pairs[key][1].setflags(write=1)

    def cleanNan(self, X):
        it = np.nditer(X, op_flags=['readwrite'], flags=['multi_index'])
        with it:
            while not it.finished:
                if (np.isnan(it[0])):
                    # make missing data the average value in the kth-layer it's in
                    layer = X[:, it.multi_index[1]]
                    layer = layer[np.logical_not(np.isnan(layer))]
                    it[0] = np.mean(layer)
                it.iternext()
        assert(np.isfinite(X).all())

    def __getitem__(self, index):
        return self.pairs[index]

    def reportMissingData(self):
        if (self.nanCount != 0 | self.nanCount != 0):
            print(self.nanCount, "nans and", self.infCount,
                  "infs are found in record")


class BulkGenerator():
    def __init__(self, path, epoch, maxEpoch, fileRecordsNum, recordsNum, mapSize):
        self.dirnames = []
        # r=root, d=directories, f = files
        for r, d, f in os.walk(path):
            for directory in d:
                # print(os.path.join(r, directory))
                self.dirnames.append(os.path.join(r, directory))
        self.fileNum = len(self.dirnames)
        self.recordsNum = recordsNum
        self.fileRecordsNum = fileRecordsNum
        if (fileRecordsNum % recordsNum == 0):
            self.accessTimes = fileRecordsNum // recordsNum
        else:
            self.accessTimes = fileRecordsNum // recordsNum + 1
        self.mapSize = mapSize
        assert(self.accessTimes == 1)
        self.dsLMDB = None
        valiFileSize = self.fileNum / maxEpoch
        self.maxEpoch = maxEpoch
        assert(valiFileSize.is_integer() is True)
        self.valiFileSize = int(valiFileSize)
        # for testing
        self.valiFileSize = 0
        self.valiStartFileI = self.valiFileSize * epoch
        self.curFileI = 0
        self.curAccessI = 0
        self.vCurFileI = self.valiStartFileI
        self.vCurAccessI = 0

    def nextTrainBulk(self):
        if (self.curFileI == self.valiStartFileI):
            self.curFileI += self.valiFileSize
        if (self.curFileI >= self.fileNum):
            print("curFileI:", self.curFileI, "curFileI:", self.curFileI)
            return None
        if (self.curAccessI == 0):
            self.dsLMDB = DsLMDB(self.dirnames[self.curFileI], self.mapSize)
        start = self.curAccessI * self.recordsNum
        stop = min(start + self.recordsNum, self.fileRecordsNum)
        pairs = self.dsLMDB.load(start, stop)
        self.curAccessI += 1
        if (self.curAccessI >= self.accessTimes):
            self.dsLMDB.close()
            self.curAccessI = 0
            self.curFileI += 1
        dsDataset = DsDataset(pairs)
        dsDataset.reportMissingData()
        return dsDataset

    def nextValiBulk(self):
        if (self.maxEpoch == 1):
            raise("Can't validate when maxEpoch equals to 1.")
        if (self.vCurFileI >= self.fileNum):
            raise("Each fold should contain at least one image.")
        if (self.vCurFileI >= self.valiStartFileI + self.valiFileSize):
            return None
        if (self.vCurAccessI == 0):
            self.dsLMDB = DsLMDB(self.dirnames[self.vCurFileI], self.mapSize)
        start = self.vCurAccessI * self.recordsNum
        stop = min(start + self.recordsNum, self.fileRecordsNum)
        pairs = self.dsLMDB.load(start, stop)
        self.vCurAccessI += 1
        if (self.vCurAccessI >= self.accessTimes):
            self.dsLMDB.close()
            self.vCurAccessI = 0
            self.vCurFileI += 1
        dsDataset = DsDataset(pairs)
        return dsDataset


class Train():
    def __init__(self, dataPath, modelPath, folds, recordsNum, fileRecordsNum,
                 trainBatchSize, valiBatchSize, mapSize):
        self.recordsNum = recordsNum
        self.fileRecordsNum = fileRecordsNum
        self.trainBatchSize = trainBatchSize
        self.valiBatchSize = valiBatchSize
        self.bulkGenerator = None
        self.maxEpoch = folds
        self.writer = SummaryWriter(modelPath)
        self.dataPath = dataPath
        self.modelPath = modelPath
        self.mapSize = mapSize

    def nextDataGenarator(self, kind):
        if (kind == "train"):
            trainBulk = self.bulkGenerator.nextTrainBulk()
            if (trainBulk is not None):
                return DataLoader(trainBulk, batch_size=self.trainBatchSize, shuffle=True, pin_memory=True, num_workers=0)
            else:
                return None
        elif (kind == "validation"):
            valiBulk = self.bulkGenerator.nextValiBulk()
            if (valiBulk is not None):
                return DataLoader(valiBulk, batch_size=self.valiBatchSize, pin_memory=True, num_workers=0)
            else:
                return None
        else:
            raise("Unkoun kind of data.")

    def train(self):
        model = DSModel()
        model.to(dev)
        lossFn = torch.nn.MSELoss()
        optimizer = torch.optim.Adam(
            model.parameters(), lr=1e-2)  # learning rate grows linearly with batchsize
        for epoch in range(self.maxEpoch):            
            # for testing
            if (epoch == 1):
                break
            # measure time
            # for gpu
            # epochStart = torch.cuda.Event(enable_timing=True)
            # epochEnd = torch.cuda.Event(enable_timing=True)
            # datasetStart = torch.cuda.Event(enable_timing=True)
            # datasetEnd = torch.cuda.Event(enable_timing=True)
            # epochStart.record()
            # for cpu
            epochStart = time.time()

            self.bulkGenerator = BulkGenerator(
                self.dataPath, epoch, self.maxEpoch, self.fileRecordsNum,
                self.recordsNum, self.mapSize)

            datasetNum = 0
            datasetCount = 0
            # report time for loading a training dataset
            print("start to load epoch", epoch, "dataset", datasetCount)
            loadingStart = time.time()
            dataGenarator = self.nextDataGenarator(kind="train")
            print("time for loading epoch", epoch, "dataset",
                  datasetCount, ":", time.time()-loadingStart)

            while (dataGenarator is not None):
                print("start to train epoch", epoch, "dataset", datasetCount)
                # for gpu
                # datasetStart.record()
                # for cpu
                datasetStart = time.time()
                batchNum = len(dataGenarator)

                for i, (batchData, l) in enumerate(dataGenarator):
                    # load minibatch into device and report loading time
                    # loadingStart = time.time()
                    batchData, l = batchData.to(
                        dev), l.to(dev)

                    # print("time for loading a minibatch in epoch", epoch,
                    #       "dataset", datasetCount, ":", time.time()-loadingStart)

                    # batchStart = torch.cuda.Event(enable_timing=True)
                    # batchEnd = torch.cuda.Event(enable_timing=True)
                    # batchStart.record()
                    # train minibatch
                    optimizer.zero_grad()
                    lPred = model(batchData)
                    # for testing

                    uni, uniCount = np.unique(
                        lPred.data.numpy(), return_counts=True)
                    print("shape:", list(lPred.size()), "max:", torch.max(lPred), "min:", torch.min(lPred),
                          "mean:", torch.mean(lPred), "unique count:", uniCount)
                    if (i == 800):
                        print("unique values:", uni.tolist())

                    # clamping predition values less equal to -1
                    lPred[lPred <= -1] = -0.99999
                    logLPred = torch.log1p(lPred)
                    logL = torch.log1p(l)
                    # loss = lossFn(torch.log1p(lPred), torch.log1p(l))
                    loss = lossFn(logLPred, logL)
                    assert(not torch.isnan(loss).any())
                    loss.backward()
                    optimizer.step()
                    # report time for training minibatch
                    # batchEnd.record()
                    # torch.cuda.synchronize()
                    # print("time for training a minibatch in epoch", epoch, ", dataset",
                    #       datasetCount, ":", batchStart.elapsed_time(batchEnd) / 1000, "seconds")
                    nonlogLoss = lossFn(lPred, l)
                    pos = epoch * datasetNum * batchNum + datasetCount * batchNum + i

                    self.writer.add_scalar('lPred', torch.mean(lPred), pos)
                    self.writer.add_scalar('l', torch.mean(l), pos)
                    self.writer.add_scalar(
                        'logLPred', torch.mean(logLPred), pos)
                    self.writer.add_scalar('logL', torch.mean(logL), pos)
                    self.writer.add_scalar('non-log loss', nonlogLoss, pos)
                    self.writer.add_scalar('training loss', loss, pos)

                # report time for training a dataset

                print("time for training epoch", epoch, ", dataset",
                      datasetCount, ":", (time.time() - datasetStart) / 60, "min")

                for data in dataGenarator:
                    # hack to make add graph work, seems there's a bug of cannot
                    # omit the 'data' parameter for add_graph...
                    self.writer.add_graph(model, data[0])
                    break

                # for testing
                if (datasetCount == 0):
                    break

                # report time for loading a training dataset
                print("start to load epoch", epoch, "dataset", datasetCount)
                loadingStart = time.time()

                dataGenarator = self.nextDataGenarator("train")
                print("time for loading epoch", epoch, "dataset",
                      datasetCount, ":", time.time()-loadingStart)
                datasetCount += 1

            # report time for a training epoch
            datasetNum = datasetCount
            print("time for training epoch ", epoch,
                  ": ", (time.time() - epochStart) / 60, "min")

            torch.save(model.state_dict(), self.modelPath+"model"+str(epoch))
            # for testing: do not validate
            return 0

            # validation
            valiLoss = 0
            valiCount = 0
            loadingStart = time.time()
            dataGenarator = self.nextDataGenarator(kind="validation")
            print("time for validating epoch", epoch, "dataset",
                  datasetCount, ":", time.time()-loadingStart)

            epochStart.record()
            while (dataGenarator is not None):
                for i, (batchData, l) in enumerate(dataGenarator):
                    print("start to validate epoch",
                          epoch, "dataset", valiCount)
                    datasetStart.record()

                    # load dataset in to device
                    batchData, l = batchData.to(
                        dev, non_blocking=True), l.to(dev, non_blocking=True)
                    # inferencing
                    lPred = model(batchData)
                    # assert data contain no nan
                    assert(not torch.isnan(lPred).any())
                    valiLoss += float(lossFn(torch.log1p(lPred),
                                             torch.log1p(l)))

                    # report time for validating a dataset
                    print("time for validation epoch", epoch, ", dataset:",
                          valiCount, ":", (time.time() - datasetStart) / 60, "min")

                    # report time for loading a validation dataset
                    loadingStart = time.time()
                    dataGenarator = self.nextDataGenarator(kind="validation")
                    print("time for validating epoch", epoch, "dataset",
                          datasetCount, ":", time.time()-loadingStart)

                    valiCount += 1

            # recording loss and saving model
            self.writer.add_scalar(
                'validate loss', valiLoss / valiCount, epoch)
            torch.save(model.state_dict(), self.modelPath+"model"+str(epoch))

            # report time for a validation epoch
            print("time for validating epoch ", epoch,
                  ": ", (time.time() - epochStart) / 60, "min")


# set device
dev = torch.device(
    "cuda") if torch.cuda.is_available() else torch.device("cpu")
# dev = torch.device("cpu")

if __name__ == '__main__':
    # parameters for traning
    params = {
        "dataPath": "/home/LarsMPace/ds_db/",
        "modelPath": "/home/LarsMPace/sync/models/",
        "folds": 1,  # 6, folds number for cross validation, each fold contain at least one image
        "fileRecordsNum": 1589429,  # samples for file 1196, 18GB
        "recordsNum": 1589429,  # // 20 + 1,  # data size load in memory, 0.9GB
        "trainBatchSize": 500,
        "valiBatchSize": 8000,
        "mapSize": 1048576 * 1024 * 36,  # 1GB * 4096 = 36GB
    }
    t = Train(**params)
    t.train()
