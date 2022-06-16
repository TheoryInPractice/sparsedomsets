## How to reproduce paper experiments

### Requirements

- gcc 10.2.0 and later
- GNU make
- Gurobi Optimizer 9.1.2
- [spacegraphcats](https://github.com/spacegraphcats/spacegraphcats) v2.0
- Requirements for Brown et al's replication pipeline: https://github.com/ctb/2020-rerun-hu-s1

### Build

- In the `cpp` directory, run `make`.
- Then, the following binary files will be built in the `bin` directory.
  - `domset`
  - `make_ilp`
  - `make_qp`
  - `nbrprt`

### Experiments

#### 1. Sparse Dominating Sets

- Download and unzip the `data` directory.
  - https://drive.google.com/file/d/1qsuBmxUiPqWtxwmzgfnLVkDwGb5V9CiW/view?usp=sharing
  - Compressed: 689 MB, Uncompressed: 2.29 GB
- To run experiments with `Dom-Degree`, `Dom-Ratio`, `Dom-Degree-Plus`, and `Dom-Ratio-Plus`, run the `domset` program with appropriate parameters.
  - `domset <input_path> <radius> <strategy> <seed> <num_threads> <output_path>`
- To run experiments with `Dom-SGC`, run spacegraphcats to obtain the result.
- To run expeirments with `Dom-MDS` and `Dom-MAC`, run the `make_ilp` program to create a model and then run Gurobi to obtain the solution.
  - `make_ilp <input_path> <radius> <problem> <num_threads>`


#### 2. Balanced Neighborhood Partitioning

- To run experiments with `Prt-SGC`, run spacegraphcats to obtain the result.
- To run experiments with `Prt-Weight`, `Prt-Layer`, and `Prt-Branch`, run the `nbrprt` program with appropriate parameters.
  - `nbrprt <graph_path> <domset_path> <strategy> <seed> <output_path>`
- To run experiments with `Prt-QP`, run the `make_qp` program to create a model and then run Gurobi to obtain the solution.
  - `make_qp <graph_path> <landmark_path>`


#### 3. Metagenome Neighborhood Queries

- Copy the following files in *this directory* to spacegraphcats' installation directory (overwrite existing files).
  - `spacegraphcats/catlas/catlas.py`
  - `spacegraphcats/catlas/rdomset.py`
- Follow the instructions in the [pipeline](https://github.com/ctb/2020-rerun-hu-s1).


### Supplemental material

- `table_network.pdf`: summary of the network corpus used in experiments

