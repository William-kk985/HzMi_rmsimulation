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

-- 同 cartographer_lio.lua：全包形态没有"从起点算起"的 odom → 先关掉 use_odometry，
-- 否则会用世界系绝对位姿把 map→odom 拧坏（实测 -130°）。等补上零化适配器再开。
options.use_odometry = false

return options
