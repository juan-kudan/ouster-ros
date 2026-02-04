/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file lidar_packet_handler.h
 * @brief ...
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <pcl_conversions/pcl_conversions.h>

#include "lock_free_ring_buffer.h"
#include <thread>
#include <chrono>

namespace {

template <typename T, typename UnaryPredicate>
int find_if_reverse(const Eigen::Array<T, -1, 1>& array,
                    UnaryPredicate predicate) {
    auto p = array.data() + array.size() - 1;
    do {
        if (predicate(*p)) return p - array.data();
    } while (p-- != array.data());
    return -1;
}

uint64_t linear_interpolate(int x0, uint64_t y0, int x1, uint64_t y1, int x) {
    uint64_t min_v, max_v;
    double sign;
    if (y1 > y0) {
        min_v = y0;
        max_v = y1;
        sign = +1;
    } else {
        min_v = y1;
        max_v = y0;
        sign = -1;
    }
    return y0 + (x - x0) * sign * (max_v - min_v) / (x1 - x0);
}

}  // namespace

namespace ouster_ros {

namespace sensor = ouster::sensor;

using LidarScanProcessor =
    std::function<void(const ouster::LidarScan&, uint64_t, const ros::Time&)>;

class LidarPacketHandler {
    using LidarPacketAccumlator =
        std::function<bool(const sensor::LidarPacket&)>;

   public:
    using HandlerOutput = ouster::LidarScan;

    using HandlerType = std::function<void(const sensor::LidarPacket&)>;

   public:
    LidarPacketHandler(const sensor::sensor_info& info,
                       const std::vector<LidarScanProcessor>& handlers,
                       const std::string& timestamp_mode,
                       int64_t ptp_utc_tai_offset,
                       float min_scan_valid_columns_ratio,
                       int arc_angle)
        : ring_buffer(LIDAR_SCAN_COUNT),
          lidar_scan_handlers{handlers},
          ptp_utc_tai_offset_(ptp_utc_tai_offset),
          min_scan_valid_columns_ratio_(min_scan_valid_columns_ratio) {
        
        arc_ = ouster_ros::make_arc_config(arc_angle, info.format.columns_per_frame);
        W_ = info.format.columns_per_frame;
        H_ = info.format.pixels_per_column;

        work_scan_ = std::make_unique<ouster::LidarScan>(
            W_, H_, info.format.udp_profile_lidar);

        col_present_.assign(W_, 0);
        bin_emitted_.assign(arc_.bins, 0);

        // if you can derive returns count from profile, set it; otherwise start at 1.
        // (Often 1 or 2.)
        n_returns_ = ouster_ros::get_n_returns(info);

        // initialize lidar_scan processor and buffer
        scan_batcher = std::make_unique<ouster::ScanBatcher>(info);

        lidar_scans.resize(LIDAR_SCAN_COUNT);
        mutexes.resize(LIDAR_SCAN_COUNT);

        for (size_t i = 0; i < lidar_scans.size(); ++i) {
            lidar_scans[i] = std::make_unique<ouster::LidarScan>(
                info.format.columns_per_frame, info.format.pixels_per_column,
                info.format.udp_profile_lidar);
            mutexes[i] = std::make_unique<std::mutex>();
        }

        lidar_scans_processing_thread = std::make_unique<std::thread>([this]() {
            while (lidar_scans_processing_active) {
                process_scans();
            }
            NODELET_DEBUG("lidar_scans_processing_thread done.");
        });

        // initalize time handlers
        scan_col_ts_spacing_ns = compute_scan_col_ts_spacing_ns(info.mode);
        compute_scan_ts = [this](const auto& ts_v) {
            return compute_scan_ts_0(ts_v);
        };

        const sensor::packet_format& pf = sensor::get_format(info);

        std::function<bool(LidarPacketHandler&, const sensor::packet_format&,
                           const sensor::LidarPacket&, ouster::LidarScan&)>
            lidar_handler;

        if (timestamp_mode == "TIME_FROM_ROS_TIME") {
            lidar_handler =
                std::mem_fn(&LidarPacketHandler::lidar_handler_ros_time);
        } else if (timestamp_mode == "TIME_FROM_PTP_1588") {
            lidar_handler =
                std::mem_fn(&LidarPacketHandler::lidar_handler_sensor_time_ptp);
        } else /*SENSOR TIME (INTERNAL_OSC, SYNC_PULSE_IN)*/ {
            lidar_handler =
                std::mem_fn(&LidarPacketHandler::lidar_handler_sensor_time);
        }

        lidar_packet_accumlator = LidarPacketAccumlator{
            [this, pf, lidar_handler](const sensor::LidarPacket& lidar_packet) {
                // In arc mode, we may enqueue multiple scans per packet; in non-arc mode
                // we enqueue at most one scan when the full scan completes.
                // We'll check ring buffer fullness right before each enqueue.

                bool enqueued_any = false;

                // ---- ARC MODE ----
                if (arc_.enabled()) {
                    // 1) Batch packet into work_scan_ (full-width) and update timestamps
                    // lidar_handler should now update lidar_scan_estimated_* opportunistically.
                    const bool full_scan_complete =
                        lidar_handler(*this, pf, lidar_packet, *work_scan_);

                    // 2) Mark columns present
                    mark_packet_columns_present(pf, lidar_packet);

                    // 3) Emit any newly-complete bins
                    for (int b = 0; b < arc_.bins; ++b) {
                        if (bin_emitted_[b]) continue;
                        if (!bin_complete(b)) continue;

                        if (ring_buffer.full()) {
                            NODELET_WARN("lidar_scans full, DROPPING ARC");
                            break;
                        }

                        const int c0 = arc_.start_col[b];
                        const int c1 = arc_.end_col[b];

                        {
                            std::unique_lock<std::mutex> lock(*(mutexes[ring_buffer.write_head()]));
                            auto& out_scan = *lidar_scans[ring_buffer.write_head()];
                            fill_output_scan_for_arc(*work_scan_, out_scan, c0, c1);
                        }

                        ring_buffer.write();
                        enqueued_any = true;
                        bin_emitted_[b] = 1;
                    }

                    // 4) Reset per-rotation state when the full scan completes
                    if (full_scan_complete) {
                        std::fill(col_present_.begin(), col_present_.end(), 0);
                        std::fill(bin_emitted_.begin(), bin_emitted_.end(), 0);

                        // IMPORTANT: if using ROS_TIME mode with lidar_handler_ros_time_frame_ts,
                        // ensure your lidar_handler_* only advances the frame_ts when scan_complete,
                        // otherwise you can drift.
                    }

                    return enqueued_any;
                }

                // ---- NON-ARC (ORIGINAL) MODE ----
                // Preserve original behavior: batch directly into the current ring-buffer scan slot.
                // This avoids copying large LidarScan objects.
                if (ring_buffer.full()) {
                    NODELET_WARN("lidar_scans full, DROPPING PACKET");
                    return false;
                }
                bool scan_complete = false;
                {
                    std::unique_lock<std::mutex> lock(
                        *(mutexes[ring_buffer.write_head()]));
                    auto& lidar_scan = *lidar_scans[ring_buffer.write_head()];
                    scan_complete = lidar_handler(*this, pf, lidar_packet, lidar_scan);
                    if (scan_complete) {
                        // count the number of valid columns in the scan
                        auto status = lidar_scan.status();
                        size_t valid_cols = std::count_if(status.data(), status.data() + status.size(),
                               [](const uint32_t s) { return (s & 0x01); });
                        if (valid_cols < static_cast<size_t>(min_scan_valid_columns_ratio_ * status.size())) {
                            NODELET_WARN_STREAM("number of valid columns per scan " << valid_cols << "/" << status.size()
                             <<" which is below the ratio " << std::setprecision(4) << (100 * min_scan_valid_columns_ratio_)
                             << "%, SKIPPING SCAN");
                            scan_complete = false;
                        }
                    }
                }
                if (scan_complete) {
                    ring_buffer.write();
                    enqueued_any = true;
                }
                return enqueued_any;
            }};
    }

    LidarPacketHandler(const LidarPacketHandler&) = delete;
    LidarPacketHandler& operator=(const LidarPacketHandler&) = delete;
    ~LidarPacketHandler() {
        NODELET_DEBUG("LidarPacketHandler::~LidarPacketHandler()");
        if (lidar_scans_processing_thread->joinable()) {
            lidar_scans_processing_active = false;
            lidar_scans_processing_thread->join();
        }
    }

    void register_lidar_scan_handler(LidarScanProcessor handler) {
        lidar_scan_handlers.push_back(handler);
    }

    void clear_registered_lidar_scan_handlers() { lidar_scan_handlers.clear(); }

   public:
    static HandlerType create(
        const sensor::sensor_info& info,
        const std::vector<LidarScanProcessor>& handlers,
        const std::string& timestamp_mode, int64_t ptp_utc_tai_offset,
        float min_scan_valid_columns_ratio, int arc_angle) {
        auto handler = std::make_shared<LidarPacketHandler>(
            info, handlers, timestamp_mode, ptp_utc_tai_offset,
            min_scan_valid_columns_ratio, arc_angle);
        return [handler](const sensor::LidarPacket& lidar_packet) {
            if (handler->lidar_packet_accumlator(lidar_packet)) {
                handler->ring_buffer_has_elements.notify_one();
            }
        };
    }

    const std::string getName() const { return "lidar_packet_hander"; }

    void process_scans() {
        {
            using namespace std::chrono;
            std::unique_lock<std::mutex> index_lock(ring_buffer_mutex);
            ring_buffer_has_elements.wait_for(
                index_lock, 1s, [this] { return !ring_buffer.empty(); });

            if (ring_buffer.empty()) return;
        }

        std::unique_lock<std::mutex> lock(*mutexes[ring_buffer.read_head()]);

        for (auto h : lidar_scan_handlers) {
            h(*lidar_scans[ring_buffer.read_head()], lidar_scan_estimated_ts,
              lidar_scan_estimated_msg_ts);
        }

        // when we hit percent amount of the ring_buffer capacity throttle
        size_t read_step = 1;
        if (ring_buffer.size() > THROTTLE_PERCENT * ring_buffer.capacity()) {
            NODELET_WARN("lidar_scans %d%% full, THROTTLING",
                         static_cast<int>(100 * THROTTLE_PERCENT));
            read_step = 2;
        }
        ring_buffer.read(read_step);
    }

    // time interpolation methods
    uint64_t impute_value(int last_scan_last_nonzero_idx,
                          uint64_t last_scan_last_nonzero_value,
                          int curr_scan_first_nonzero_idx,
                          uint64_t curr_scan_first_nonzero_value,
                          int scan_width) {
        assert(scan_width + curr_scan_first_nonzero_idx >
               last_scan_last_nonzero_idx);
        double interpolated_value = linear_interpolate(
            last_scan_last_nonzero_idx, last_scan_last_nonzero_value,
            scan_width + curr_scan_first_nonzero_idx,
            curr_scan_first_nonzero_value, scan_width);
        return impl::ulround(interpolated_value);
    }

    uint64_t extrapolate_value(int curr_scan_first_nonzero_idx,
                               uint64_t curr_scan_first_nonzero_value) {
        double extrapolated_value =
            curr_scan_first_nonzero_value -
            scan_col_ts_spacing_ns * curr_scan_first_nonzero_idx;
        return impl::ulround(extrapolated_value);
    }

    // compute_scan_ts_0 for first scan
    uint64_t compute_scan_ts_0(
        const ouster::LidarScan::Header<uint64_t>& ts_v) {
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        assert(idx != ts_v.data() + ts_v.size());  // should never happen
        int curr_scan_first_nonzero_idx = idx - ts_v.data();
        uint64_t curr_scan_first_nonzero_value = *idx;

        uint64_t scan_ns =
            curr_scan_first_nonzero_idx == 0
                ? curr_scan_first_nonzero_value
                : extrapolate_value(curr_scan_first_nonzero_idx,
                                    curr_scan_first_nonzero_value);

        last_scan_last_nonzero_idx =
            find_if_reverse(ts_v, [](uint64_t h) { return h != 0; });
        assert(last_scan_last_nonzero_idx >= 0);  // should never happen
        last_scan_last_nonzero_value = ts_v(last_scan_last_nonzero_idx);
        compute_scan_ts = [this](const auto& ts_v) {
            return compute_scan_ts_n(ts_v);
        };

        return scan_ns;
    }

    // compute_scan_ts_n applied to all subsequent scans except first one
    uint64_t compute_scan_ts_n(
        const ouster::LidarScan::Header<uint64_t>& ts_v) {
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        assert(idx != ts_v.data() + ts_v.size());  // should never happen
        int curr_scan_first_nonzero_idx = idx - ts_v.data();
        uint64_t curr_scan_first_nonzero_value = *idx;
        uint64_t scan_ns = curr_scan_first_nonzero_idx == 0
                               ? curr_scan_first_nonzero_value
                               : impute_value(last_scan_last_nonzero_idx,
                                              last_scan_last_nonzero_value,
                                              curr_scan_first_nonzero_idx,
                                              curr_scan_first_nonzero_value,
                                              static_cast<int>(ts_v.size()));
        last_scan_last_nonzero_idx =
            find_if_reverse(ts_v, [](uint64_t h) { return h != 0; });
        assert(last_scan_last_nonzero_idx >= 0);  // should never happen
        last_scan_last_nonzero_value = ts_v(last_scan_last_nonzero_idx);
        return scan_ns;
    }

    uint16_t packet_col_index(const sensor::packet_format& pf,
                              const uint8_t* lidar_buf) {
        return pf.col_measurement_id(pf.nth_col(0, lidar_buf));
    }

    ros::Time extrapolate_frame_ts(const sensor::packet_format& pf,
                                   const uint8_t* lidar_buf,
                                   const ros::Time current_time) {
        auto curr_scan_first_arrived_idx = packet_col_index(pf, lidar_buf);
        auto delta_time = ros::Duration(
            0,
            std::lround(scan_col_ts_spacing_ns * curr_scan_first_arrived_idx));
        return current_time - delta_time;
    }

    bool lidar_handler_sensor_time(const sensor::packet_format&,
                                   const sensor::LidarPacket& lidar_packet,
                                   ouster::LidarScan& lidar_scan) {
        bool scan_complete = (*scan_batcher)(lidar_packet, lidar_scan);

        // Update timestamp estimate as soon as we have any timestamps
        const auto& ts_v = lidar_scan.timestamp();
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        if (idx != ts_v.data() + ts_v.size()) {
            lidar_scan_estimated_ts = compute_scan_ts(ts_v);
            lidar_scan_estimated_msg_ts = impl::ts_to_ros_time(lidar_scan_estimated_ts);
        } else {
            // fallback; better than leaving uninitialized
            lidar_scan_estimated_msg_ts = ros::Time::now();
            lidar_scan_estimated_ts = 0;
        }

        return scan_complete;
    }

    bool lidar_handler_sensor_time_ptp(const sensor::packet_format&,
                                       const sensor::LidarPacket& lidar_packet,
                                       ouster::LidarScan& lidar_scan) {
        bool scan_complete = (*scan_batcher)(lidar_packet, lidar_scan);

        auto ts_v = lidar_scan.timestamp();
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        if (idx != ts_v.data() + ts_v.size()) {
            for (int i = 0; i < ts_v.rows(); ++i)
                ts_v[i] = impl::ts_safe_offset_add(ts_v[i], ptp_utc_tai_offset_);

            lidar_scan_estimated_ts = compute_scan_ts(ts_v);
            lidar_scan_estimated_msg_ts = impl::ts_to_ros_time(lidar_scan_estimated_ts);
        } else {
            lidar_scan_estimated_msg_ts = ros::Time::now();
            lidar_scan_estimated_ts = 0;
        }

        return scan_complete;
    }

    bool lidar_handler_ros_time(const sensor::packet_format& pf,
                                const sensor::LidarPacket& lidar_packet,
                                ouster::LidarScan& lidar_scan) {
        auto packet_receive_time = impl::ts_to_ros_time(lidar_packet.host_timestamp);

        if (!lidar_handler_ros_time_frame_ts) {
            lidar_handler_ros_time_frame_ts =
                extrapolate_frame_ts(pf, lidar_packet.buf.data(), packet_receive_time);
        }

        bool scan_complete = (*scan_batcher)(lidar_packet, lidar_scan);

        // Update estimated sensor ts when possible (for per-point t)
        const auto& ts_v = lidar_scan.timestamp();
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        if (idx != ts_v.data() + ts_v.size()) {
            lidar_scan_estimated_ts = compute_scan_ts(ts_v);
        } else {
            lidar_scan_estimated_ts = 0;
        }

        // msg stamp stays at frame start
        lidar_scan_estimated_msg_ts = lidar_handler_ros_time_frame_ts.value();

        if (scan_complete) {
            // set time for next frame start
            lidar_handler_ros_time_frame_ts =
                extrapolate_frame_ts(pf, lidar_packet.buf.data(), packet_receive_time);
        }

        return scan_complete;
    }

    static double compute_scan_col_ts_spacing_ns(sensor::lidar_mode ld_mode) {
        const auto scan_width = sensor::n_cols_of_lidar_mode(ld_mode);
        const auto scan_frequency = sensor::frequency_of_lidar_mode(ld_mode);
        const double one_sec_in_ns = 1e+9;
        return one_sec_in_ns / (scan_width * scan_frequency);
    }

    void fill_output_scan_for_arc(const ouster::LidarScan& src,
                                ouster::LidarScan& dst,
                                int c0, int c1) {
        // Clamp and validate
        if (c0 < 0) c0 = 0;
        if (c1 > W_) c1 = W_;
        if (c1 <= c0) {
            // Produce an "empty" scan (everything invalid)
            auto dst_ts  = dst.timestamp();
            auto dst_st  = dst.status();
            auto dst_mid = dst.measurement_id();
            for (int v = 0; v < W_; ++v) {
                dst_ts[v] = 0;
                dst_st[v] = 0;
                dst_mid[v] = v;
            }
            // Clear RANGE fields entirely (rare path)
            for (int r = 0; r < n_returns_; ++r) {
                auto range_ch = static_cast<sensor::ChanField>(sensor::ChanField::RANGE + r);
                auto dst_range = dst.field<uint32_t>(range_ch);
                std::fill(dst_range.data(),
                        dst_range.data() + static_cast<size_t>(W_) * static_cast<size_t>(H_),
                        0u);
            }
            return;
        }

        // 1) Clear headers across all columns (cheap)
        {
            auto dst_ts  = dst.timestamp();
            auto dst_st  = dst.status();
            auto dst_mid = dst.measurement_id();

            for (int v = 0; v < W_; ++v) {
                dst_ts[v] = 0;
                dst_st[v] = 0;   // invalid everywhere by default
                dst_mid[v] = v;  // optional / debug-friendly
            }
        }

        // 2) Copy headers only for the arc columns
        {
            auto src_ts  = src.timestamp();
            auto src_st  = src.status();
            auto src_mid = src.measurement_id();

            auto dst_ts  = dst.timestamp();
            auto dst_st  = dst.status();
            auto dst_mid = dst.measurement_id();

            for (int v = c0; v < c1; ++v) {
                dst_ts[v]  = src_ts[v];
                dst_st[v]  = src_st[v];
                dst_mid[v] = src_mid[v];
            }
        }

        // 3) Copy RANGE fields for arc columns and clear outside-arc columns (per row)
        //    This ensures cartesianT() produces NaNs outside arc (range=0 => NaN in your code).
        for (int r = 0; r < n_returns_; ++r) {
            auto range_ch = static_cast<sensor::ChanField>(sensor::ChanField::RANGE + r);
            auto src_range = src.field<uint32_t>(range_ch);
            auto dst_range = dst.field<uint32_t>(range_ch);

            uint32_t* dst_ptr = dst_range.data();
            const uint32_t* src_ptr = src_range.data();

            for (int u = 0; u < H_; ++u) {
                const size_t row_off = static_cast<size_t>(u) * static_cast<size_t>(W_);

                // Clear left side [0, c0)
                if (c0 > 0) {
                    std::fill(dst_ptr + row_off,
                            dst_ptr + row_off + static_cast<size_t>(c0),
                            0u);
                }

                // Copy arc [c0, c1)
                std::copy(src_ptr + row_off + static_cast<size_t>(c0),
                        src_ptr + row_off + static_cast<size_t>(c1),
                        dst_ptr + row_off + static_cast<size_t>(c0));

                // Clear right side [c1, W)
                if (c1 < W_) {
                    std::fill(dst_ptr + row_off + static_cast<size_t>(c1),
                            dst_ptr + row_off + static_cast<size_t>(W_),
                            0u);
                }
            }
        }

        // NOTE:
        // We intentionally do NOT clear/copy SIGNAL/REFLECTIVITY/NEAR_IR here yet.
        // Because the only thing that determines whether a point makes it into the cloud
        // in scan_to_cloud_f() (unorganized) is whether XYZ is NaN.
        //
        // Once you confirm which intensity-like fields are copied by your PROFILE/point type,
        // we can add the SAME per-row "clear outside / copy inside" logic for those fields too.
    }

   private:
    std::unique_ptr<ouster::ScanBatcher> scan_batcher;
    const int LIDAR_SCAN_COUNT = 10;
    const float THROTTLE_PERCENT = 0.7f;
    LockFreeRingBuffer ring_buffer;
    std::mutex ring_buffer_mutex;
    std::vector<std::unique_ptr<ouster::LidarScan>> lidar_scans;
    std::vector<std::unique_ptr<std::mutex>> mutexes;

    uint64_t lidar_scan_estimated_ts;
    ros::Time lidar_scan_estimated_msg_ts;

    std::optional<ros::Time> lidar_handler_ros_time_frame_ts;

    int last_scan_last_nonzero_idx = -1;
    uint64_t last_scan_last_nonzero_value = 0;

    double scan_col_ts_spacing_ns;  // interval or spacing between columns of a
                                    // scan

    std::function<uint64_t(const ouster::LidarScan::Header<uint64_t>&)>
        compute_scan_ts;

    std::vector<LidarScanProcessor> lidar_scan_handlers;

    LidarPacketAccumlator lidar_packet_accumlator;

    bool lidar_scans_processing_active = true;
    std::unique_ptr<std::thread> lidar_scans_processing_thread;
    std::condition_variable ring_buffer_has_elements;

    int64_t ptp_utc_tai_offset_;

    float min_scan_valid_columns_ratio_ = 0.0f;

    // Handling sending arc of scans as they fill up
    ouster_ros::ArcConfig arc_;
    std::unique_ptr<ouster::LidarScan> work_scan_;   // full-width scan being filled
    std::vector<uint8_t> col_present_;              // size W
    std::vector<uint8_t> bin_emitted_;              // size arc_.bins

    int W_ = 0;
    int H_ = 0;
    int n_returns_ = 1;

    inline void mark_packet_columns_present(const sensor::packet_format& pf,
                                            const sensor::LidarPacket& pkt) {
        const uint8_t* buf = pkt.buf.data();
        // In Ouster SDK, packet_format provides nth_col(i, buf) and col_measurement_id(...)
        // Determine number of columns in a packet:
        const int ncols = pf.columns_per_packet; // if available; otherwise hardcode/derive
        for (int i = 0; i < ncols; ++i) {
            const uint8_t* col = pf.nth_col(i, buf);
            const uint16_t mid = pf.col_measurement_id(col);
            if (mid < W_) col_present_[mid] = 1;
        }
    }

    inline bool bin_complete(int b) const {
        const int c0 = arc_.start_col[b];
        const int c1 = arc_.end_col[b];
        for (int v = c0; v < c1; ++v) {
            if (!col_present_[v]) return false;
        }
        return true;
    }
};

}  // namespace ouster_ros