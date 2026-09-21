-- ============================================================================
-- 全包形态 + 纯定位（lio:=cartographer 且 mode:=nav）
--   加载 map/<world>.pbstream 做纯定位，同时兼任里程计源（provide_odom_frame=true）。
-- 资产前提：先用 mode:=mapping lio:=cartographer 建图，再 finish_trajectory + write_state 导出
--   （见 docs/smoke_test_runbook.md §5/§6；pgm 与 pbstream 必须同一次建图）。
-- 话题分工：/map 留给 map_server 的先验栅格图；本节点的栅格走 /cartographer_map
--   （bringup 在 mode:=nav 下自动传 occupancy_grid_topic:=/cartographer_map）。
-- ============================================================================
include "cartographer_localization.lua"

options.published_frame = "base_link"
options.provide_odom_frame = true

-- 同 cartographer_lio.lua：接一路底盘里程计当运动先验（否则静止也在漂）。
-- 纯定位模式下它同样重要：pose extrapolator 决定 odom→base_link 的连续性。
options.use_odometry = true

return options
