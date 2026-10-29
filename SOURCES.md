# Source provenance

- Deployment baseline: `robot-agilex-piper-ga`, commit
  `6190fc996f52fd833c44d110f9357347f7650b48`.
- Robonix runtime and submodule acquisition: [vendor/PINS.md](vendor/PINS.md).
- Package upstream revisions and deployment changes:
  [docs/UPSTREAMS.md](docs/UPSTREAMS.md).
- RK3588 detector: YOLO OBB weights, exported RKNN model and metadata, RKNN
  Lite2 2.3.2 wheel, and matching runtime library in [yolo_npu](yolo_npu/README.md).
  SDK files retain the [Rockchip license](yolo_npu/vendor/RKNN-LICENSE).
- Third-party camera drivers and SDKs remain within the camera primitive,
  together with their upstream license files and architecture-specific assets.

The publication inventory is defined by `catalog/publication.json`.
Local credentials, deployment results, generated dependencies, caches, unrelated
robot deployments, and experimental skills are excluded. Export inventories
record SHA256 hashes of the files actually published.
