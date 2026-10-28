# Preserve the source image/state/intrinsics as one atomic packet through the pipeline.
tdt_interface/AutoAimFrame frame
Armor[] armors
bool reset
# Detection diagnostics for this exact exposure; successful PnP count is armors.size().
uint32 light_count
uint32 candidate_count
uint32 classified_count
