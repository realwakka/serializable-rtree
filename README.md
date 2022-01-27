# serializable-rtree
It's serializable-rtree

It's library for rtree serializable. R-tree structure could be saved to file or stream and could be loaded.

This library is under development.
 - Supports default insertion algorithm using quadratic split.
 - Supports default range query.
 - Supports default KNN query using simple MINDIST.


random data N=4
![test1](https://user-images.githubusercontent.com/6241577/150647028-b559b53d-7591-4689-a6c9-df8ea5368da4.png)

random data N=8
![test1](https://user-images.githubusercontent.com/6241577/150680088-38da569c-ddbd-4efe-b0b5-44914a5afebf.png)

KNN Query Result
```
realwakka@realwakka-Home:~/laboratory/serializable-rtree$ ./test_knn 
KNN results: 3 17 1 8 4
```

![output](https://user-images.githubusercontent.com/6241577/151326850-36897791-335a-4e9e-a309-a87ad7bcdddc.png)
