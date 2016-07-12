# cpf_segmentation
C++ library for unsupervised segmentation of 3D points via Constrained Plane Fitting

If you want to cite this work, you can use the bibtex entry below

```
@inproceedings{Pham2016,
abstract = {Research on SLAM is now able to reliably recon-
struct dense 3D geometric maps of indoor scenes with a depth
sensor. Representing these maps in terms of meaningful entities
is a step toward building semantic maps for autonomous robots.
Recent work segments the 3D scene into semantic classes using
Conditional Random Fields, which requires large 3D ground
truth datasets to train the classification model. Also, the CRF
inference is often computationally expensive. In this paper,
we present a simple unsupervised method for segmentation
of 3D maps into objects and scene structures. Specifically,
we propose a novel constrained plane extraction algorithm
to robustly discover the underlying planes in the scene. The
planes are enforced to be mutually orthogonal or parallel to
conform with the human-made indoor environments. Moreover,
we approximate the 3D map by an adjacency graph over surface
patches for a graph clustering based scene segmentation. We
devise an effective classifier which utilises both the extracted
planes and local surface convexity to classify the graph edges as
on (the same object) or off (different objects). We reconstructed
654 3D indoor scenes from NYU v2 sequences to validate the
efficiency and effectiveness of our segmentation method.
},
author = {Pham, Trung T. and Eich, Markus and Reid, Ian and Wyeth, Gordon}
booktitle = {2016 IEEE/RSJ International Conference on Intelligent Robots and Systems, IROS},
month = {October},
pages = {},
publisher = {IEEE},
title = {{Unsupervised Segmentation of Dense Indoor 3D Maps
via Constrained Plane Fitting}},
year = {2016}
}
```


