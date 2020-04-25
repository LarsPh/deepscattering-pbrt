import lmdb
import numpy as np
import os


class Converter():
    def __init__(self, path, mapSize):
        self.path = path
        self.env = lmdb.open(path, map_size=mapSize, readonly=True)
        self.same = 0
        self.diff = 0

    def convert(self, start, end):
        file = open(os.path.join(self.path, "data.txt"), 'w')
        txn = self.env.begin()
        for key in range(start, end):
            # write key \n
            file.write(str(key)+'\n')
            bKey = key.to_bytes(4, byteorder='little')
            val = txn.get(bKey)
            if (val is None):
                file.write("missing\n\n\n\n")
            else:
                pVal = self.parseVal(val)
                # write stencil \n
                for i in range(2250):
                    # when 5 \n
                    if (i % 5 == 0):
                        strDensity = ["%.4g" % f for f in pVal[0][i:i+5]]
                        file.write('\t'.join(strDensity)+'\n')
                    # when 25 \n\n
                    if ((i % 25 == 0) & (i != 0)):
                        file.write('\n')
                    # when 25*9 \n\n\n
                    if ((i % (25*9) == 0) & (i != 0)):
                        file.write('\n')
                # write gamma \n
                strGamma = ["%.4g" % gamma for gamma in pVal[1]]
                file.write('\t'.join(strGamma)+'\n')
                # write l \n\n\n\n
                strL = ["%.4g" % l for l in pVal[2]]
                file.write('\t'.join(strL)+'\n\n\n\n')
        print(self.same, "same rgb's")
        print(self.diff, "different rgb's")

    def parseVal(self, val):
        stencil = np.frombuffer(val, dtype='f', count=2250)
        gamma = np.frombuffer(val, dtype='f', count=1,
                              offset=2250*4)
        l = np.frombuffer(val, dtype='f', count=1,
                          offset=2251*4)
        return (stencil, gamma, l)


params = {
    "path": "/home/LarsMPace/ds_db/db_001",
    "mapSize": 1073741824 * 20,  # 1GB * 20
}
converter = Converter(**params)
converter.convert(0, 2000)
