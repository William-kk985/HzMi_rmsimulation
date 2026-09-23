# Research: why `/scan` stops permanently in the livox → linefit → p2l chain

Scope: ROS 2 Humble + Gazebo Classic, 6 research questions asked by the parent agent.
Legend: **[V]** = verified in a primary source I read (source code, official issue, or an accepted answer).
**[R]** = verified as a *real report by a human* but unconfirmed root cause.
**[I]** = my inference from source reading — labelled as such.

Local artifacts read for grounding (paths relative to repo root):

- `src/rm_perception/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp` (244 lines)
- `src/rm_perception/pointcloud_to_laserscan/config/laserscan_params.yaml` and the installed copy
  `install/pointcloud_to_laserscan/share/pointcloud_to_laserscan/config/laserscan_params.yaml`
- `src/rm_nav_bringup/launch/bringup_sim.launch.py`
- `src/rm_perception/linefit_ground_segementation_ros2/linefit_ground_segmentation_ros/src/ground_segmentation_node.cc`
- `/opt/ros/humble/include/tf2_ros/tf2_ros/message_filter.hpp`, `/opt/ros/humble/include/tf2/tf2/buffer_core.hpp`
  (tf2_ros 0.25.23 installed on this machine)

---

## 0. Two repo-local defects found while researching (read this first)

**0a. `pointcloud_to_laserscan` and `linefit_ground_segmentation` are launched *without* `use_sim_time`.**

`bringup_sim.launch.py` L205-212:

```python
bringup_pointcloud_to_laserscan_node = Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    remappings=[('cloud_in',  ['/segmentation/obstacle']), ('scan',  ['/scan'])],
    parameters=[os.path.join(get_package_share_directory('pointcloud_to_laserscan'),
                             'config', 'laserscan_params.yaml')],   # <-- no use_sim_time
    name='pointcloud_to_laserscan'
)
```

L197-202 (linefit) likewise passes only `parameters=[segmentation_params]`. Verified:
`laserscan_params.yaml` (src **and** install) and `segmentation_sim.yaml` contain **zero** occurrences of
`use_sim_time`; `grep -rn SetParameter src/` finds no global `SetParameter` action. So both nodes run on the
**system clock** while the rest of the stack runs on sim time. **[V]**

This matters directly for item 1: `tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock())` (node.cpp L83) builds
a buffer whose `clock_` is the *wall* clock. See item 1 for exactly which behaviours that does and does not change
(the honest answer is narrower than the folklore).

**0b. The p2l tf2 `MessageFilter` is a no-op in the current configuration.** **[V]**

`target_frame: livox_frame` (config) and `/segmentation/obstacle` carries `header.frame_id = livox_frame`
(linefit copies the input header verbatim: `ground_segmentation_node.cc` L141-146). tf2's
`BufferCore::addTransformableRequest` begins with `if (target_frame == source_frame) return 0;`
(0 == "immediately transformable"), and `canTransform`/`canTransformInternal` short-circuit the same way. So the
filter passes every message synchronously without ever touching the cache, and node.cpp L174
(`if (scan_msg->header.frame_id != cloud_msg->header.frame_id)`) skips the in-callback transform too.
Consequence: **items 1/2's tf2 mechanisms cannot be the *active* cause of a p2l-side stall in this config.**
They are still worth fixing as latent risk (item 2/3c).

---

## 1. The tf2 error `"the timestamp on the message is earlier than all the data in the transform cache"` under `use_sim_time`

### (a) Finding

This exact string is **not** a tf2 buffer-core message. It is the human-readable text of the enum value
`tf2_ros::filter_failure_reasons::OutTheBack` inside `tf2_ros::MessageFilter`, and it is emitted by
`MessageFilter::signalFailure()` as a **throttled INFO on the consuming node's own logger**. **[V]** The precise
code condition that produces it is `msg_stamp + cache_time_ < latest_common_time`, where `cache_time_` is the tf2
buffer's cache duration (default **10 s**). So the literal meaning is: *"this message's stamp is more than the tf2
cache duration older than the newest transform I hold for that frame chain"* — i.e. the message is too **stale** to
ever be transformable, not that the transform is missing. The dominant real-world causes are (i) clock-domain
mismatch between the stamping node and the TF publishers, (ii) the buffer being **cleared** by a time jump, and
(iii) plain message delay (DDS/executor congestion) exceeding the cache duration.

### (b) Exact code / config / commands

The reason string and its only producer **[V]**:

```cpp
// /opt/ros/humble/include/tf2_ros/tf2_ros/message_filter.hpp
// L87  (enum comment)
/// The timestamp on the message is earlier than all the data in the transform cache
// L105-107
case filter_failure_reasons::OutTheBack:
  return "the timestamp on the message is earlier than all the data in the transform cache";
// L705-716  — the log you are seeing
void signalFailure(const MEvent & evt, FilterFailureReason reason) {
  ...
  RCLCPP_INFO_THROTTLE(node_logging_->get_logger(), *clock, 2500,
    "Message Filter dropping message: frame '%s' at time %.3f for reason '%s'",
    frame_id.c_str(), stamp.seconds(), get_filter_failure_reason_string(reason).c_str());
}
```
Note the second argument: the throttle uses `node_clock_->get_clock()`, i.e. the **consumer node's** clock.

The exact "too old" test **[V]** (`tf2/src/buffer_core.cpp`, `addTransformableRequest` and
`testTransformableRequests`):

```cpp
getLatestCommonTime(req.target_id, req.source_id, latest_time, 0);
if ((latest_time != TimePointZero) && (time + cache_time_ < latest_time)) {
  return 0xffffffffffffffffULL;      // addTransformableRequest -> "never transformable"
}
...
if ((latest_time != TimePointZero) && (req.time + cache_time_ < latest_time)) {
  do_cb = true; result = TransformFailure;   // testTransformableRequests
}
```

Cache duration default **[V]**: `/opt/ros/humble/include/tf2/tf2/buffer_core.hpp` L73
`static constexpr Duration BUFFER_CORE_DEFAULT_CACHE_TIME = std::chrono::seconds(10);`

Buffer **clearing on time jumps** (cause ii) **[V]** — `tf2_ros/src/buffer.cpp`, `Buffer::onTimeJump`:

```cpp
if (RCL_ROS_TIME_ACTIVATED == jump.clock_change || RCL_ROS_TIME_DEACTIVATED == jump.clock_change) {
  RCLCPP_WARN(getLogger(), "Detected time source change. Clearing TF buffer.");  clear();
} else if (jump.delta.nanoseconds < 0) {
  RCLCPP_WARN(getLogger(), "Detected jump back in time. Clearing TF buffer.");   clear();
}
```
This callback is registered in the `Buffer` constructor with a threshold of "anything backwards is a jump,
`on_clock_change = true`". So on any Gazebo world reset / `/clock` restart every queued message becomes
"earlier than all the data" at once.

Diagnosis commands:

```bash
# 1. the clock-domain check everyone skips — run it for EVERY node in the chain
ros2 param get /pointcloud_to_laserscan use_sim_time     # currently False  <-- defect 0a
ros2 param get /ground_segmentation    use_sim_time      # currently False  <-- defect 0a
ros2 param dump /pointcloud_to_laserscan

# 2. is the message stamp in the sim-time domain?
ros2 topic echo /clock --once
ros2 topic echo /segmentation/obstacle --once --field header
python3 -c "import time; print('wall', time.time())"      # compare with header.stamp.sec

# 3. does the transform chain exist, and what is its buffer age?
ros2 run tf2_ros tf2_echo     odom livox_frame
ros2 run tf2_ros tf2_monitor                       # prints per-frame rate + buffer length
ros2 run tf2_tools view_frames                     # writes frames.pdf / frames.gv
ros2 topic echo /tf_static --once                  # static vs dynamic stamps
```

`ros2 topic echo` and `ros2 topic hz` both default to the `sensor_data` QoS preset (BEST_EFFORT) in Humble
[V: `ros2topic/verb/echo.py` L41 `default_profile_str = 'sensor_data'`; `ros2topic/verb/hz.py` L43/L274-278
`qos_profile_sensor_data`], so they *do* receive your BEST_EFFORT `/scan` — no extra `--qos-reliability` needed.
This is a common false alarm; don't chase it.

Fixes reported in the wild:

| Root cause | Concrete fix | Source |
|---|---|---|
| One node on wall clock (the accepted answer to a Gazebo-classic + Humble slam_toolbox case with this exact error) | "ensuring `robot_state_publisher` had `use_sim_time=True`. **All nodes** should have `use_sim_time=True`, if they are using sim time." | [robotics.SE 115148 → answer 115182](https://robotics.stackexchange.com/questions/115148/ros2-humble-gazebo-slam-toolbox-tf2-dropped-message-reason-the-timestamp-on) **[V, accepted]** |
| Message delay > tf2 cache caused by an RMW bug in Humble's Fast-DDS | backport [eProsima/Fast-DDS#3195](https://github.com/eProsima/Fast-DDS/pull/3195); **two independent reporters confirm the problem disappears with `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`** (`sudo apt install ros-humble-rmw-cyclonedds-cpp`) | [navigation2#3352](https://github.com/ros-navigation/navigation2/issues/3352) — full log + maintainer @SteveMacenski closing it as "not a nav2 issue, file with Fast-DDS/RMW" **[V, real report ×3]** |
| TF buffer deleted by lifecycle `cleanup` (slam_toolbox) | After `cleanup`+`reconfigure`, messages are older than all data; the reporter's actual crash was a memory leak, so the MessageFilter lines were a *symptom* | [robotics.SE 118242 → answer 118275](https://robotics.stackexchange.com/questions/118242/message-filter-dropping-message-and-error-code-11-in-async-slam-toolb) **[R]** |
| Same error, "all nodes use sim time", `transform_timeout`/`tf_buffer_duration` tuning did not help | open, unanswered | [robotics.SE 118268](https://robotics.stackexchange.com/questions/118268/ros2-humble-gazebo-classic-docker-slam-toolbox-keeps-dropping-laserscan-wit) **[R]** |

### (c) Honest narrowing for *this* stack

**[I, from source]** For p2l specifically, `use_sim_time=false` does **not** by itself break tf2 lookups, because tf2
compares *message stamps* against *cached transform stamps* — both taken off the wire — so the clock object is not
part of the comparison. The clock enters tf2 in only three places: (1) the polling loop of
`Buffer::canTransform(t,s,time,timeout,err)` (`clock_->now()` for elapsed time), (2) the timer created in
`Buffer::waitForTransform` via `timer_interface_->createTimer(clock_, timeout, ...)`, and (3) `onTimeJump`
detection. p2l never calls `now()` to stamp anything (it copies `cloud_msg->header`, L149), so today the wall clock
is only a latent landmine. It becomes *active* the moment either (a) `target_frame` is changed to `base_link`/`odom`
or (b) anything downstream relies on p2l's `onTimeJump` buffer-clearing. Fix it anyway — it costs one line and it is
the only node in the chain that is inconsistent.

---

## 2. What makes `tf2_ros::MessageFilter` drop every message permanently?

### (a) Finding

Four distinct permanent/半-permanent failure modes exist, and only one of them prints the `OutTheBack` text:
`EmptyFrameID` (permanent for that publisher, distinct message), `QueueFull` (transient),
`NoTransformFound`/`Unknown` (transforms absent), and **the silent one: if `getLatestCommonTime` cannot produce a
`latest_time`, the "too old" fast-fail branch is skipped entirely and the request is queued forever — the message
sits in `messages_`, the callback never fires, and *nothing is logged*.** That last mode is the only one that
matches "process alive, subscriber matched, low CPU, zero output, no warning". Recovery is per-message for
`OutTheBack`/`QueueFull` (yes, it recovers) but only "on the next new transform that satisfies the request" for the
queued-forever case — which can look permanent.

### (b) Exact source lines

`add()` — the frame-id gate and the queue **[V]**:

```cpp
// message_filter.hpp L365-372
std::string frame_id = stripSlash(mt::FrameId<M>::value(*message));
if (frame_id.empty()) { messageDropped(evt, filter_failure_reasons::EmptyFrameID); return; }
// L414-420
if (queue_size_ != 0 && messages_.size() + 1 > queue_size_) {
  ++dropped_message_count_; ... messages_.pop_front();   // reason QueueFull
}
// L441-451 — always an async request, no "frame already target" shortcut here
tf2_ros::TransformStampedFuture future = buffer_.waitForTransform(
    target_frame, frame_id, stamp, buffer_timeout_,
    std::bind(&MessageFilter::transformReadyCallback, this, std::placeholders::_1, handle));
```

`Buffer::waitForTransform` — the three outcomes **[V]** (`tf2_ros/src/buffer.cpp`):

```cpp
auto handle = addTransformableRequest(cb, target_frame, source_frame, time);
if (0 == handle) {                       // immediately transformable
  promise->set_value(lookupTransform(target_frame, source_frame, time)); call_callback = true;
} else if (0xffffffffffffffffULL == handle) {   // NEVER transformable
  promise->set_exception(... "Failed to transform from " + source_frame + " to " + target_frame ...);
  call_callback = true;
} else {
  auto timer_handle = timer_interface_->createTimer(clock_, timeout, ...);   // <-- clock_ is the node clock
}
```
`transformReadyCallback` maps **any** exception from `future.get()` to `error = OutTheBack` **[V]** — so the
"earlier than all the data" wording is a catch-all, not a literal diagnosis. Do not trust the words; measure the
stamps.

Answers to the specific sub-questions:

| Sub-question | Answer | Evidence |
|---|---|---|
| Interaction with `use_sim_time` | The filter itself is clock-agnostic for *lookups* (uses message stamps). The node clock is used only for the throttle in `signalFailure` and for the `waitForTransform` timer. **`checkFailures()` is dead code in Humble** — grep finds the definition at L608 and **no call site**, so the "Dropped X%% of messages" WARN never prints. | **[V]** grep of `message_filter.hpp` |
| `target_frame` == the message's own frame | **Passes everything.** `addTransformableRequest` returns 0 immediately; `canTransform` likewise. Not a failure mode. | **[V]** `buffer_core.cpp` |
| Static transforms on `/tf_static` | A `StaticCache` reports `TimePointZero` as its latest time, and `getLatestCommonTime` skips `TimePointZero` entries (`if (latest.first != TimePointZero) common_time = min(...)`). So a chain joined **only** by statics yields `latest_time == TimePointZero` ⇒ the "too old" branch is skipped ⇒ **requests queue forever with no log** (the silent mode). Your `base_link → livox_frame` edge is static (`frames_2026-09-22_21.23.01.gv`: rate 10000.0, buffer length 0.0, most-recent 0.0 = StaticCache), but `odom → base_link` is dynamic, so `latest_time` is real for AMCL. | **[V]** `buffer_core.cpp` `getLatestCommonTime`; **[V]** local `frames_*.gv` |
| `queue_size` | p2l passes `input_queue_size_` as the MessageFilter `queue_size` **and** as the QoS depth. It is declared as `declare_parameter("queue_size", std::thread::hardware_concurrency())` and **is not set in `laserscan_params.yaml`**, so on this 28-core box it is **28**. `queue_size == 0` means infinite. | **[V]** node.cpp L65-66, L88-93, L121-123; config has no `queue_size` |
| Does `transform_tolerance: 0.01` matter? | **No.** `tolerance_` is never passed to the `MessageFilter` constructor (L88-93). It is used only as the *timeout* of the in-callback `tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_))` at L177 — and that line is skipped when the frames already match. The README says the same: "*Time tolerance for transform lookups. Only used if a `target_frame` is provided.*" | **[V]** node.cpp L62/L88-93/L174-183; README L30 |
| Recovery | Recovers for `OutTheBack` (each message is judged independently) and `QueueFull`. `EmptyFrameID` never recovers for that publisher. The queued-forever case recovers only when a new transform makes `testTransformableRequests` fire — so a *stuck frame* looks permanent. | **[V]** `buffer_core.cpp` `testTransformableRequests` |

`getLatestCommonTime` returns `TF2_CONNECTIVITY_ERROR` when frames are in different trees — and callers pass
`error_string = 0`, so **that error is silently swallowed** and `latest_time` stays `TimePointZero`. **[V]**

### (c) Sources

- [tf2_ros `message_filter.hpp` (humble)](https://github.com/ros2/geometry2/blob/humble/tf2_ros/include/tf2_ros/message_filter.hpp) — local copy at `/opt/ros/humble/include/tf2_ros/tf2_ros/message_filter.hpp`, tf2_ros 0.25.23
- [tf2 `buffer_core.cpp` (humble)](https://github.com/ros2/geometry2/blob/humble/tf2/src/buffer_core.cpp)
- [tf2_ros `buffer.cpp` (humble)](https://github.com/ros2/geometry2/blob/humble/tf2_ros/src/buffer.cpp)
- [MessageFilter API docs (Humble)](https://docs.ros.org/en/ros2_packages/humble/api/tf2_ros/generated/classtf2__ros_1_1MessageFilter.html)

---

## 3. `pointcloud_to_laserscan` (ROS 2 port) known issues

### (a) `subscriptionListenerThreadLoop` / `sub_.unsubscribe()` — real, and already patched here

**[V, real reports]** Upstream node.cpp L114-142 computes `pub_->get_subscription_count() +
get_intra_process_subscription_count()`; if it is `0` it calls `sub_.unsubscribe()`, and it only re-subscribes on a
**later graph-change event** (`wait_for_graph_change(event, timeout)`, 100 ms timeout). Two upstream issues document
this loop:

- [issue #77 "Node didn't recognize subscriber"](https://github.com/ros-perception/pointcloud_to_laserscan/issues/77) —
  the reporter quotes both log lines verbatim ("*Got a subscriber to laserscan…*" / "*No subscribers to laserscan,
  shutting down pointcloud subscriber*") and states that **with a subscriber present `/scan` still received nothing**.
- [issue #107 "RAM exhaustion due to accumulation of graph events"](https://github.com/ros-perception/pointcloud_to_laserscan/issues/107) —
  *"the created graph events never appear to be GC'd, eventually leading to RAM exhaustion"*, pointing at
  `subscriptionListenerThreadLoop` L131-132; proposes reusing the event object (as `rclcpp/src/rclcpp/client.cpp`
  L170-174 does). Open, 0 comments.

**Caveat I must flag** **[V, logic]**: I found **no upstream issue that states "after unsubscribe it never
re-subscribes"**. And the state you report — `ros2 topic info /segmentation/obstacle --verbose` still lists p2l as a
matched subscriber and `ros2 node info` still lists it — is **inconsistent** with the unsubscribe hypothesis,
because `message_filters::Subscriber::unsubscribe()` destroys the `rclcpp::Subscription`, which then disappears from
the graph. Treat the parent's local fix (commit `30ab296`, which replaced the unsubscribe with
`RCLCPP_INFO_ONCE` and removed the loop-exit unsubscribe) as correct defensive hardening, but **not as the proven
cause of the current symptom**.

### (b) Reports of silently producing no output

| Issue | Content | Status |
|---|---|---|
| [#105](https://github.com/ros-perception/pointcloud_to_laserscan/issues/105) | ROS 2 Humble: "`/scan` publishes correctly when drone is idle… after takeoff the frequency gradually drops until it completely stops… the publisher is still present, but no data is flowing." **Config is nearly identical to yours: `target_frame` = the sensor's own frame, `use_sim_time: True`, `transform_tolerance: 0.10`, `queue_size: 1`.** Only reply (Rayman) suggests BEST_EFFORT QoS / packet loss. **Open, unanswered.** | **[R] — closest published match to your symptom** |
| [#104](https://github.com/ros-perception/pointcloud_to_laserscan/issues/104) | "No results seen form the topic which has the converted data" — closed by author same day (no diagnosis) | **[R]** |
| [#101](https://github.com/ros-perception/pointcloud_to_laserscan/issues/101) / [#100](https://github.com/ros-perception/pointcloud_to_laserscan/issues/100) | Users unable to get output; they resort to editing the .cpp / adding a non-existent `output_log_level` param | **[R]** |
| [#68](https://github.com/ros-perception/pointcloud_to_laserscan/issues/68) | Foxy/Ignition: `/scan` **does** publish but every `ranges` entry is `.inf` — i.e. "publishes but output empty". Cause was the frame/height/range filters rejecting every point | **[R]** |
| [robotics.SE 116917 → answer 116918](https://robotics.stackexchange.com/questions/116917/debugging-pointcloud-to-laserscan-with-ouster-lidar-in-ros-2-humble) | "Debugging pointcloud_to_laserscan … (Humble)" — the accepted answer found the **remap was wrong** (`-r input:=/ouster/points` instead of `cloud_in:=/ouster/points`), so the subscription never matched and the node was silent forever. Cheapest possible "silent no output" cause. | **[V, accepted answer]** |

### (c) `target_frame` / `queue_size` recommendations, and `target_frame: ""`

**[V]** README (L29-30, L47-48): "*`target_frame` (str, default: none) — If provided, transform the pointcloud into
this frame before converting to a laser scan. **Otherwise, laser scan will be generated in the same frame as the
input point cloud.***" and "*`queue_size` (double, default: detected number of cores) — Input … queue size.*"

**[V, source]** `target_frame: ""` is not merely documented — it removes the tf2 machinery entirely:

```cpp
// node.cpp L61-62
target_frame_ = this->declare_parameter("target_frame", "");
tolerance_    = this->declare_parameter("transform_tolerance", 0.01);
// L82-96
if (!target_frame_.empty()) {           // <-- Buffer + TransformListener + MessageFilter all created here
  tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  ...
  message_filter_ = std::make_unique<MessageFilter>(sub_, *tf2_, target_frame_, input_queue_size_,
                                                    this->get_node_logging_interface(),
                                                    this->get_node_clock_interface());
  message_filter_->registerCallback(...);
} else {                                 // <-- otherwise: plain subscription, no tf2 at all
  sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
}
// L149-152 — output frame
scan_msg->header = cloud_msg->header;
if (!target_frame_.empty()) { scan_msg->header.frame_id = target_frame_; }
```

So for your chain, **`target_frame: ""` is the recommended setting**: `/scan.header.frame_id` stays
`livox_frame` (identical output), and the tf2 Buffer, the `TransformListener` (an extra `/tf`+`/tf_static`
subscription), and the `MessageFilter` are all not created. Note also that the fixed `queue_size` should be set
explicitly — leaving it unset silently picks `hardware_concurrency()` (28 here), which is a large QoS depth for a
0.2 MB @ 10 Hz topic.

Ordered fix list for p2l (lowest risk first):

```python
Node(
    package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
    parameters=[laserscan_params, {'use_sim_time': use_sim_time}],   # 1) clock consistency
    remappings=[('cloud_in', ['/segmentation/obstacle']), ('scan', ['/scan'])],
    name='pointcloud_to_laserscan')
```
```yaml
# laserscan_params.yaml
target_frame: ""      # 2) drop the tf2 filter entirely (identical output frame)
queue_size: 5         # 3) be explicit instead of hardware_concurrency()
```

---

## 4. "Timestamp Discipline and Message Synchronization" (wimblerobotics.github.io)

### (a) Finding

**[V — fetched, HTTP 200]** The page exists but is a thin, ~1-page distillation with **no tf2/MessageFilter,
`use_sim_time`, or point-cloud content at all.** Its own two cited sources are the `message_filters` approximate-sync
tutorial and the ROS 2 clock/time design article, and it is tagged only `debugging message-filters multi-machine ros2
synchronization timestamps`. So it does **not** answer your question; the only transferable content is its
clock-discipline framing.

### (b) Verbatim content

Sections: *Why This Matters*, *Distilled Takeaways*, *Practical Value*, *Corroborating References*, *When to Read the
Original Source*. Its distilled takeaways, verbatim:

- "Header stamps are part of the data contract, not decoration."
- "`message_filters` only works well when timestamps and QoS settings are already coherent."
- "Approximate synchronization is often the practical choice, but it does not excuse bad clocks."
- "Multi-machine systems amplify timestamp problems because network delay and unsynchronized device assumptions get
  mixed together."
- "Time discipline should be designed before downstream fusion or perception tuning."

Practical value, verbatim: "Stamp messages from a consistent ROS-aware clock path. / Keep QoS aligned across
synchronized topics or the filter layer will fail before your logic runs. / Use approximate sync for realistic sensor
timing, but choose queue size and age penalty deliberately. / Audit clock assumptions whenever fusion, perception, or
recorded playback seems intermittently wrong."

### (c) Sources

- <https://wimblerobotics.github.io/software/timestamp-discipline-and-message-synchronization/> **[V]**
- <https://design.ros2.org/articles/clock_and_time.html> (its own primary reference)
- <https://docs.ros.org/en/jazzy/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Cpp.html>

---

## 5. "Why does a ROS 2 point-cloud consumer fall behind when transforms use a different clock?"

### (a) Finding — with an important authority caveat

**[V — found, fetched via a text proxy because the site is Cloudflare-protected]** The article exists at
`agents.stackoverflow.com` ("Stack Overflow for Agents", beta). **It is an AI-generated Q&A, not human-verified
knowledge: the answer shows `Votes: 0`, `Claims: 0 | Votes: 0`, "No verifications yet", and
"Trust score pending", and the author is an agent account (`valentinbot Agent`).** Its content is nonetheless a
reasonable checklist and happens to agree with the primary sources in item 1. The question itself is explicitly
derived from a Stack Overflow question by PaulE. **Do not cite this as authority** — cite the tf2 code and
navigation2#3352 for the mechanism.

### (b) Summarised claims (7 extracted "claims" + the answer body)

Claims list as published: the primary cause of "timestamp outside buffer" in ROS 2 mapping pipelines is a **clock
mismatch between publishers and the mapper**; **increasing the transform-filter queue only masks the symptom**;
**every node publishing data or transforms consumed by the mapper must set `use_sim_time: true`**; **if a sensor
driver cannot use sim time, re-stamp the cloud in the mapper's clock domain before it reaches the transform filter**;
**a successful lookup at `tf2::TimePointZero` combined with a failure at the message's own timestamp confirms a
clock-alignment problem**; and *only* tune queue depth after clocks and the transform chain are verified.

Body specifics worth stealing: (1) `ros2 param get <node> use_sim_time` for every node; confirm `/clock` is actually
published, because with `use_sim_time=true` and no `/clock` *all* stamps are garbage. (2) Distinguish the two error
classes — "target frame unknown" (wrong/missing frame ⇒ `tf2_echo`) from "timestamp outside buffer" (transform exists
but not at that time ⇒ clock domain). (3) The systematic check: for each cloud, log
`lookupTransform(world, sensor, msg.header.stamp)` **and** `lookupTransform(world, sensor, tf2::TimePointZero)`; if
the latest lookup succeeds while the stamped one fails, it is a timestamp/clock problem, not a tree problem. That is
the single most useful idea in the article and it is directly falsifiable with the code in item 2. (4) Suggested
queue depth after fixing clocks: 10-20 messages.

### (c) Sources

- <https://agents.stackoverflow.com/questions/614b62ed-f9ff-45d4-9fb5-da156dac1fc3> **[V, but low-authority AI-generated; direct fetch returns HTTP 403 Cloudflare — retrieved via `r.jina.ai`]**
- Primary-source equivalents for the same mechanism: [tf2 `buffer_core.cpp`](https://github.com/ros2/geometry2/blob/humble/tf2/src/buffer_core.cpp), [navigation2#3352](https://github.com/ros-navigation/navigation2/issues/3352)

---

## 6. Grep-able binary search inside the running p2l node

Goal: separate **(a) input callback never fires** from **(b) tf2 filter drops everything** from **(c) it publishes but
the output is empty/all-inf**. Work down; each step is a yes/no with a specific command or log line.

**Step −1 — cheap sanity (30 s).** `ros2 topic hz /scan` and `ros2 topic hz /segmentation/obstacle` both default to
`sensor_data` (BEST_EFFORT) QoS and therefore *will* see your topics; if `/scan` shows nothing here, the problem is
real, not a QoS artefact of the tool. **[V]**

**Step 0 — rule the whole chain in or out.** Run all three rates simultaneously:

```bash
ros2 topic hz /livox/lidar/pointcloud & ros2 topic hz /segmentation/obstacle & ros2 topic hz /scan
```
- All three die together ⇒ **Gazebo plugin / writer-side block, not p2l.** This is the failure your repo already
  documented: `ground_segmentation_node.cc` L76-85 ("RELIABLE 写者会被跟不上速率的消费者堵住队列 … 阻塞点落在 Gazebo 的
  sensor 回调里 ⇒ … `/scan` 一起停更 180s 且不自恢复") and `docs/debug_fastlio_cartographer.md` §5.3.
- Only `/scan` dies ⇒ continue to step 1. **[I from source + repo measurement]**

**Step 1 — (a) vs (b): is the callback firing at all?** p2l is *your* build, so instrument it (or, with zero code
change, flip the config and compare):

```bash
# Non-invasive A/B: does removing the filter restore output?
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args -r cloud_in:=/segmentation/obstacle -r scan:=/scan -p target_frame:="" \
             -p use_sim_time:=true
# If /scan resumes -> the tf2 path was the culprit (b). If not -> (a) or (c).
```
```bash
# Turn on the tf2 filter's per-message DEBUG (logger name is literally tf2_ros_message_filter)
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  --log-level tf2_ros_message_filter:=debug --log-level debug \
  -r cloud_in:=/segmentation/obstacle -p use_sim_time:=true
```
Log lines to look for, in order **[V — all strings from `message_filter.hpp`]**:
- `MessageFilter [target=livox_frame ]: Added message in frame livox_frame at time N, count now K`
  → the subscription callback **is** firing; the filter received the message.
- `MessageFilter [target=livox_frame ]: Message ready in frame …` → the filter passed it; any remaining problem is
  in `cloudCallback` ⇒ go to step 3.
- `MessageFilter [target=livox_frame ]: Discarding message in frame … count now K` → (b) confirmed.
- **No `Added message` line at all while `ros2 topic hz /segmentation/obstacle` is non-zero** ⇒ **(a)**, the input
  callback never fires. Then check, in this order:
  1. `ros2 node info /pointcloud_to_laserscan` — is `/segmentation/obstacle` still listed under Subscribers?
     If **not** ⇒ the upstream `sub_.unsubscribe()` path (item 3a) is live on this binary — confirm the patched
     build (`git log -1 --format=%H -- src/rm_perception/pointcloud_to_laserscan`) is actually installed.
  2. `ros2 topic info /segmentation/obstacle --verbose | grep -A4 'Node name: pointcloud_to_laserscan'` —
     Reliability must be `BEST_EFFORT` on both ends. Mismatch ⇒ silent starvation.
  3. `ps -o %cpu,etime,cmd -C pointcloud_to_laserscan_node` — a matched subscription plus ~0 % CPU while 0.2 MB
     × 10 Hz is on the wire is the signature of "frames never delivered" (BEST_EFFORT fragment loss), which is
     **measured in this repo: a RELIABLE subscriber got 2182/2182 = 100 % while BEST_EFFORT subscribers got
     3 %–26 %** (`docs/issues_and_findings.md` #25, `docs/debug_fastlio_cartographer.md` §5.3). Fix candidates:
     raise the publisher reliability, reduce the message size, or accept the drop and use the `local_obstacle:=cloud`
     bypass documented in `docs/smoke_test_runbook.md` L668-678.
  4. Grep the p2l stdout for a swallowed tf2 exception — with `target_frame == livox_frame`, if `livox_frame` is
     absent from the buffer, `Buffer::waitForTransform` takes the `handle == 0` branch and then `lookupTransform()`
     calls `validateFrameId`, which **throws** (`"… passed to lookupTransform argument target_frame does not
     exist"`), aborting `add()` before `cloudCallback`. Look for a `LookupException`/`Error in callback` line in
     p2l's output. **[I, from source]**

**Step 2 — which drop reason?** The drop line is emitted at **INFO** (default visible), throttled to once per
2500 ms, on the **node's own logger**, so it must appear as `/pointcloud_to_laserscan`, not as `tf2_ros`:

```bash
ros2 launch ... 2>&1 | grep "Message Filter dropping message.*livox_frame"
```
Reason → cause mapping **[V]**:
- `…for reason 'the timestamp on the message is earlier than all the data in the transform cache'` → stamp is
  > 10 s (`BUFFER_CORE_DEFAULT_CACHE_TIME`) older than the newest common transform; or the buffer was just cleared by
  a time jump. **But note:** with `target_frame == livox_frame` this branch is unreachable, so seeing it from
  `/pointcloud_to_laserscan` would prove the runtime params differ from the file — confirm with
  `ros2 param get /pointcloud_to_laserscan target_frame`.
- `…for reason 'the frame id of the message is empty'` → the cloud's `header.frame_id` is empty ⇒ permanent.
  Check `ros2 topic echo /segmentation/obstacle --once --field header`.
- `…for reason 'discarding message because the queue is full'` → consumer slower than 10 Hz or a stuck queue.
- `…for reason 'did not find a valid transform, this usually happens at startup ...'` / `'unknown'` → tree problem.

**Step 3 — (c) publishes but output is empty.** Only meaningful if step 1 showed `Message ready`. **[V for the
code path: node.cpp L167-171 fills `ranges` with `inf` (or `range_max+inf_epsilon`), and every rejection path
(L190-229) is a `continue`, so a fully-rejected cloud still publishes a full-length all-`inf` scan.]**

```bash
ros2 topic echo /scan --once --field ranges > /tmp/r.txt
python3 - <<'EOF'
v=[l.split('-',1)[1].strip() for l in open('/tmp/r.txt') if l.strip().startswith('-')]
n=len(v); inf=sum(1 for x in v if 'inf' in x); num=sum(1 for x in v if x[:1].isdigit())
print(f"bins={n} inf={inf} numeric={num}")
EOF
```
- `bins` ≈ 1461 (`(3.14159-(-3.14159))/0.0043`) and `numeric == 0` ⇒ every point was rejected. Then enable the
  rejection DEBUG lines (they are `RCLCPP_DEBUG`, one per rejected point, node.cpp L191/L199/L208/L215/L224):
  `--log-level debug` and grep for `rejected for height` / `rejected for range` / `rejected for angle`. Whichever
  dominates names the culprit parameter (`min_height/max_height` vs `range_min/range_max` vs `angle_min/max`), and
  `--once --field header` tells you the frame.
- **Important discriminator [I, from source]:** an all-`inf` `/scan` still *publishes*, so nav2's
  "observation buffer has not been updated" warning would **not** appear (with `inf_is_valid: true` those bins are
  treated as 10 m clears). Since your symptom includes that warning, (c) is unlikely to be your failure — the node
  is probably not publishing at all.

**Step 4 — if p2l is fine, the warning belongs to the consumer.** The `Message Filter dropping message` you quoted
came from AMCL, whose filter target is `odom`/`map`, not `livox_frame` — so it is a *different* filter with a
*different* frame chain, and it fails for the item-1 reasons even when p2l is healthy:

```bash
ros2 param get /amcl use_sim_time
ros2 run tf2_ros tf2_echo odom livox_frame      # must succeed continuously
ros2 topic echo /scan --once --field header     # stamp must track /clock
ros2 run tf2_ros tf2_monitor                    # per-frame buffer length / most-recent transform
```

---

## 7. Ranked hypotheses for the reported symptom (for the parent)

| # | Hypothesis | Confidence | One-line discriminator | Fix |
|---|---|---|---|---|
| 1 | **Writer-side/transport stall of the whole chain (the repo's own documented 180 s failure)**: slow consumer + RELIABLE writer blocks Gazebo's sensor callback, or BEST_EFFORT fragment loss starves p2l | High — reproduced and measured in this repo (`#25`, §5.3, the 2026-09-23 comment in `ground_segmentation_node.cc`) | `ros2 topic hz` on `/livox/lidar/pointcloud`, `/segmentation/obstacle`, `/scan` simultaneously | keep publisher BEST_EFFORT + keep the message small; or the `local_obstacle:=cloud` bypass |
| 2 | **p2l's input callback genuinely stops** (subscription/transport, not the filter) | Medium | tf2 DEBUG shows **no** `Added message` while the topic has rate; `ros2 node info` sub state | patched build (commit `30ab296`); explicit `queue_size`; verify QoS both ends |
| 3 | **tf2/MessageFilter / clock-domain problem** | **Low for p2l in this config** — the filter is a no-op when `target_frame == cloud frame`; still relevant for AMCL/costmap | `-p target_frame:=""` A/B; `ros2 param get /pointcloud_to_laserscan target_frame` | `target_frame: ""`; add `use_sim_time:=true` to p2l and linefit |
| 4 | Publishes all-`inf` scans | Low | the step-3 bin census | fix `min_height/max_height`/`range_min` |

Two configuration defects are worth fixing regardless of which hypothesis wins: **missing `use_sim_time` on both
`pointcloud_to_laserscan` and `linefit_ground_segmentation`**, and **`target_frame: livox_frame` where
`target_frame: ""` is both equivalent and strictly safer**.
