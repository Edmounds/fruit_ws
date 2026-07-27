# pb_gicp_localization

Loads a GLIM-derived PCD in `map`, aligns odom-frame `registered_scan` data at 2 Hz, and publishes
`map -> odom`. Startup uses the configured `initial_map_to_odom`; global search and RViz
`initialpose` are intentionally unsupported. Failed quality gates retain the last accepted TF.
