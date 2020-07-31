

This project makes improvements based on [StVO-PL](https://github.com/rubengooj/stvo-pl). The main improvement lies in the segment tracking part
1. Key frame mechanism is added. Feature line segments are extracted and matched for key frames, and line segments are tracked directly for ordinary frames.

2. The direct tracking calculation of line segment includes four steps: sampling, tracking, fitting and thinning.
  In addition, trajectory generation is added to facilitate error comparison.
  Note: visualization is not available. There may be a conflict in the mrpt library.

  ### installation

  Follow the [StVO-PL](https://github.com/rubengooj/stvo-pl/blob/master/README.md) installation instructions

  ### Usage

```sh
export DATASETS_DIR=/mnt/hgfs/dataset/KITTI/data_odometry_gray/
cd build
./imagesStVO 00 -c ../config/config/config_kitti.yaml -o 100 -s 1 -n 1000
```


