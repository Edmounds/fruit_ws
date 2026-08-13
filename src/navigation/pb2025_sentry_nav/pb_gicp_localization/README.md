# pb_gicp_localization

Loads a GLIM-derived PCD in `map`, aligns odom-frame `registered_scan` data at 2 Hz on a worker
thread, and publishes `map -> odom` at 20 Hz. Startup uses the configured `initial_map_to_odom`;
the first accepted lock allows a wider jump window (`max_*_jump_initial`), then tighter gates apply.
Global search and RViz `initialpose` are intentionally unsupported. Failed quality gates retain the
last accepted TF; consecutive rejects raise an ERROR diagnostic.

Debug topics (Fixed Frame = `map`):

- `gicp/localization_map` — downsampled map (latched)
- `gicp/source_cloud` — registration source in `odom`
- `gicp/aligned_cloud` — source transformed into `map`
- `gicp/pose`, `gicp/path` — accepted `map -> odom` pose trail
- `diagnostics` — inliers, mean error, jumps, consecutive rejects
