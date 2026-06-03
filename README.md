# VCC: Efficient Voxel-Based Collision Checking Framework

**Note**: This repository is a fork of the [Vector-Accelerated Motion Planning (VAMP)](https://github.com/KavrakiLab/vamp) framework developed by the **Kavraki Lab**.

Built upon VAMP framework, this repository integrates VCC method, which is introduced in paper **VCC: Efficient Voxel-Based Collision Checking Framework for Real-Time Robotic Motion Planning** (ICRA 2026).

## Installation from Source
- `git clone git@github.com:nycu-caslab/vamp-vcc.git`
- `cd vamp`
- Optional: 
  - `python -m venv .venv`
  - `source .venv/bin/activate`
- `pip install .`

## Problem Set Preparation
Before running benchmarks, you need to decompress the problem set:
`python resources/problem_tar_to_pkl_json.py --robot {ur5, panda, fetch}`

Refer to [resources/README.md](https://github.com/nycu-caslab/vamp-vcc/tree/main/resources) for details.

## Running Benchmarks
`python scripts/evaluate_mbm.py --robot {ur5, panda, fetch} --pointcloud --pointcloud_repr {capt, mvt} --pointcloud_filter {scdf, centervox}`
`python scripts/visualize_mbm.py --robot {ur5, panda, fetch} --pointcloud --pointcloud_repr {capt, mvt} --pointcloud_filter {scdf, centervox} --problem {table_pick, table_under_pick, box, ...}`

Refer to [scripts/README.md](https://github.com/nycu-caslab/vamp-vcc/tree/main/scripts) for details.

---

We thank the original authors for providing such an excellent foundation for motion planning research.

For comprehensive documentation regarding custom robots, environment representations, planner configurations, code overview, and so on, please refer to the **[Original VAMP Repository](https://github.com/KavrakiLab/vamp)**.

If you build upon this repository, please ensure that you cite our paper as well as the VAMP and CAPT papers.

```bibtex
@inproceedings{vcc_2026,
  author    = {Ching Chen and Tsung-Tai Yeh},
  title     = {VCC: Efficient Voxel-Based Collision Checking Framework for Real-Time Robotic Motion Planning},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  date      = {2026},
  note      = {Accepted for publication}
}

@InProceedings{vamp_2024,
  author = {Thomason, Wil and Kingston, Zachary and Kavraki, Lydia E.},
  title = {Motions in Microseconds via Vectorized Sampling-Based Planning},
  booktitle = {IEEE International Conference on Robotics and Automation},
  pages = {8749--8756},
  url = {http://arxiv.org/abs/2309.14545},
  doi = {10.1109/ICRA57147.2024.10611190},
  date = {2024}
}

@InProceedings{capt_2024,
  author = {Ramsey, Clayton W. and Kingston, Zachary and Thomason, Wil and Kavraki, Lydia E.},
  title = {Collision-Affording Point Trees: {SIMD}-Amenable Nearest Neighbors for Fast Collision Checking},
  booktitle = {Robotics: Science and Systems},
  url = {https://www.roboticsproceedings.org/rss20/p038.pdf},
  doi = {10.15607/RSS.2024.XX.038},
  date = {2024}
}
```