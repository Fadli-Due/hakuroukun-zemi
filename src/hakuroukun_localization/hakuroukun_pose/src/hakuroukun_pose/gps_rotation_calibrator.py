#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# gps_rotation_calibrator.py
# Fadli Due Ramandavito, ISE MRG, 2026-07-12
#
# Runs alongside keyboard_teleop.py during the manual E-building -> D-F drive.
# Buffers /fix (NavSatFix) and /imu (Imu) together, opportunistically finds
# straight/sustained-speed segments in the drive, computes a GPS-rotation-angle
# candidate from each qualifying segment (comparing raw GPS bearing to
# independent IMU-integrated heading over that segment), cross-checks the
# candidates against each other, and writes the result to a YAML file on
# shutdown (Ctrl+C).
#
# WHY THIS EXISTS: the previous approach hardcoded rotation_angle=172.1 in
# hakuroukun_pose.py's _get_xy_from_latlon(), calibrated once on 2026-06-30.
# Since the GPS unit gets borrowed/remounted by other lab members, that
# constant silently goes stale with no error signal — producing "confident
# but wrong" path tracking (robot follows its own — incorrect — idea of
# where it is, cutting across coverage rows). This script re-derives the
# angle from that day's actual drive instead of trusting a historical value.
#
# USAGE:
#   Start this at the same time as keyboard_teleop.py, during the manual
#   drive from E building to D-F. Let it run for the whole drive. When you
#   arrive and are about to swap to bcd_automode.ino, Ctrl+C this script —
#   it writes gps_rotation_calibration.yaml on shutdown.
#
#   If it can't produce a confident result (too few straight segments, or
#   segments disagree too much), it will NOT write a new file — it leaves
#   the previous calibration (or the hardcoded fallback) in place and prints
#   a loud warning instead. A missing/stale calibration is safer than a
#   silently wrong one.
#
# TUNABLES (see CONFIG section below):
#   MIN_SEGMENT_DIST_M   - minimum GPS displacement for a segment to count
#   MAX_YAW_SPREAD_DEG   - how "straight" a segment must be (IMU yaw range)
#   MAX_SEGMENT_AGREEMENT_DEG - max allowed spread between candidate angles
#                          before we refuse to write a result

import math
import os
import time
from collections import deque
from datetime import datetime

import rospy
import yaml
from sensor_msgs.msg import NavSatFix, Imu
import geonav_transform.geonav_conversions as gc


# ─── CONFIG ──────────────────────────────────────────────────────────────
MIN_SEGMENT_DIST_M = 6.0          # segment must span at least this far
MAX_YAW_SPREAD_DEG = 8.0          # IMU yaw must stay within this range
                                   # across the segment to count as "straight"
MIN_SEGMENT_DURATION_S = 3.0      # avoid segments dominated by GPS noise
WINDOW_STRIDE_S = 0.5             # how often we attempt to close a window
MAX_SEGMENT_AGREEMENT_DEG = 3.0   # max spread allowed between candidates
MIN_CANDIDATES_REQUIRED = 2       # need at least this many agreeing segments
DEFAULT_CALIB_PATH = os.path.expanduser(
    "~/catkin_ws/src/hakuroukun_localization/hakuroukun_pose/config/"
    "gps_rotation_calibration.yaml")
FALLBACK_ANGLE_DEG = 172.1        # last known-good hardcoded value, used
                                   # only for reference/logging, never
                                   # silently written as a "fresh" result
# ─────────────────────────────────────────────────────────────────────────


def wrap_deg(a):
    """Wrap an angle in degrees to (-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


class GpsRotationCalibrator:
    def __init__(self):
        rospy.init_node("gps_rotation_calibrator", anonymous=True)

        self.calib_path = rospy.get_param(
            "~calibration_file", DEFAULT_CALIB_PATH)

        # Raw buffers: (t, lat, lon) and (t, yaw_rad)
        self.gps_buf = deque(maxlen=20000)
        self.imu_buf = deque(maxlen=20000)

        # First GPS fix becomes the local-frame origin, same convention as
        # hakuroukun_pose.py's gc.ll2xy usage — keeps the bearing math in a
        # flat local frame rather than doing raw lat/lon trig.
        self.origin_lat = None
        self.origin_lon = None

        # Candidate rotation angles collected from qualifying straight
        # segments, in degrees.
        self.candidates = []

        # Index into gps_buf marking the start of the current open window.
        self.window_start_idx = 0
        self.last_window_check = rospy.Time.now()

        # --- Diagnostics state (why is nothing qualifying?) ---
        self._last_status_log = rospy.Time.now()
        self._last_reject_reason = "warming up"
        self._gps_stamp_checked = False
        self._imu_stamp_checked = False

        # Gyro bias calibration state — same pattern as hakuroukun_pose.py.
        # Keep the robot STATIONARY for ~60 seconds (3000 samples at 50Hz)
        # so this can complete before you start driving. Longer average =
        # smaller residual bias = drift-free candidate angles.
        self._bias_samples = []
        self._gyro_bias_z = 0.0
        self._bias_calibrated = False
        self._yaw_accum = 0.0
        self._last_imu_t = None

        rospy.Subscriber("/fix", NavSatFix, self._gps_cb, queue_size=50)
        rospy.Subscriber("/imu", Imu, self._imu_cb, queue_size=200)

        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            "[gps_rotation_calibrator] Ready. Drive normally — this rides "
            "along and harvests straight segments in the background. "
            "Ctrl+C when you arrive at D-F to finalize calibration.")
        rospy.loginfo(
            f"[gps_rotation_calibrator] Config: min_dist={MIN_SEGMENT_DIST_M}m "
            f"max_yaw_spread={MAX_YAW_SPREAD_DEG}deg "
            f"min_duration={MIN_SEGMENT_DURATION_S}s")

    # ---------- Callbacks ----------
    def _gps_cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            return  # no fix
        t = msg.header.stamp.to_sec() if msg.header.stamp else time.time()

        if not self._gps_stamp_checked:
            self._gps_stamp_checked = True
            if not msg.header.stamp:
                rospy.logwarn(
                    "[gps_rotation_calibrator] /fix header.stamp is unset "
                    "(0,0) -- falling back to wall-clock time.time() for "
                    "every GPS sample.")
            else:
                offset = msg.header.stamp.to_sec() - time.time()
                rospy.loginfo(
                    f"[gps_rotation_calibrator] /fix header.stamp is "
                    f"populated (offset from this machine's wall clock: "
                    f"{offset:+.2f}s). If /imu's offset (see next log) "
                    f"differs from this by more than ~1s, the two topics "
                    f"are on different clocks and windows will never "
                    f"overlap.")

        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            rospy.loginfo(
                f"[gps_rotation_calibrator] Origin set at "
                f"({self.origin_lat:.7f}, {self.origin_lon:.7f})")
        self.gps_buf.append((t, msg.latitude, msg.longitude))
        self._maybe_evaluate_window()

    def _imu_cb(self, msg: Imu):
        # Matches hakuroukun_pose.py's proven convention exactly: the
        # TSND151 driver does NOT populate msg.orientation (that node never
        # reads it), and angular_velocity.z is in DEG/s, not rad/s --
        # confirmed by hakuroukun_pose.py's _integrate_yaw() doing
        # math.radians(angular_rate) * dt. This calibrator must integrate
        # the same way or its yaw estimate (and therefore every "straight
        # segment" it finds) will be silently wrong.
        raw_z = msg.angular_velocity.z

        if not self._bias_calibrated:
            self._bias_samples.append(raw_z)
            if len(self._bias_samples) % 500 == 0:
                rospy.loginfo(
                    f"[gps_rotation_calibrator] Gyro bias calibration: "
                    f"{len(self._bias_samples)}/3000 samples -- keep robot "
                    f"stationary")
            if len(self._bias_samples) >= 3000:
                self._gyro_bias_z = sum(self._bias_samples) / len(self._bias_samples)
                self._bias_calibrated = True
                rospy.loginfo(
                    f"[gps_rotation_calibrator] Gyro Z bias calibrated: "
                    f"{self._gyro_bias_z:.4f} deg/s (keep the robot still "
                    f"until this appears, same as hakuroukun_pose.py)")
            return

        corrected_z_deg_s = raw_z - self._gyro_bias_z
        t = msg.header.stamp.to_sec() if msg.header.stamp else time.time()

        if not self._imu_stamp_checked:
            self._imu_stamp_checked = True
            if not msg.header.stamp:
                rospy.logwarn(
                    "[gps_rotation_calibrator] /imu header.stamp is unset "
                    "(0,0) -- falling back to wall-clock time.time() for "
                    "every IMU sample.")
            else:
                offset = msg.header.stamp.to_sec() - time.time()
                rospy.loginfo(
                    f"[gps_rotation_calibrator] /imu header.stamp is "
                    f"populated (offset from this machine's wall clock: "
                    f"{offset:+.2f}s). Compare against the /fix offset "
                    f"logged above.")

        if self._last_imu_t is not None:
            dt = t - self._last_imu_t
            if 0.0 < dt < 1.0:
                self._yaw_accum += math.radians(corrected_z_deg_s) * dt
        self._last_imu_t = t
        self.imu_buf.append((t, self._yaw_accum))

    # ---------- Window evaluation ----------
    def _maybe_evaluate_window(self):
        now = rospy.Time.now()
        if (now - self.last_window_check).to_sec() < WINDOW_STRIDE_S:
            return
        self.last_window_check = now

        if len(self.gps_buf) < 5 or len(self.imu_buf) < 5:
            self._last_reject_reason = (
                f"buffers still filling (gps={len(self.gps_buf)}, "
                f"imu={len(self.imu_buf)})")
            self._maybe_log_status()
            return

        # Try to close a window starting at window_start_idx, extending to
        # the most recent sample, if it satisfies distance/duration.
        start_t, start_lat, start_lon = self.gps_buf[self.window_start_idx]
        end_t, end_lat, end_lon = self.gps_buf[-1]

        duration = end_t - start_t
        if duration < MIN_SEGMENT_DURATION_S:
            self._last_reject_reason = (
                f"window duration too short ({duration:.1f}s < "
                f"{MIN_SEGMENT_DURATION_S}s) -- start_t={start_t:.1f}, "
                f"end_t={end_t:.1f}")
            self._maybe_log_status()
            return

        x0, y0 = gc.ll2xy(start_lat, start_lon, self.origin_lat, self.origin_lon)
        x1, y1 = gc.ll2xy(end_lat, end_lon, self.origin_lat, self.origin_lon)
        dist = math.hypot(x1 - x0, y1 - y0)

        if dist < MIN_SEGMENT_DIST_M:
            self._last_reject_reason = (
                f"distance too short ({dist:.1f}m < "
                f"{MIN_SEGMENT_DIST_M}m, duration so far={duration:.1f}s)")
            self._maybe_log_status()
            return  # not far enough yet, keep window open, check again later

        # We have a candidate-length segment. Check straightness via IMU
        # yaw spread over the same time window.
        yaws_deg = [
            math.degrees(yaw) for (t, yaw) in self.imu_buf
            if start_t <= t <= end_t
        ]
        if len(yaws_deg) < 3:
            latest_imu_t = self.imu_buf[-1][0] if self.imu_buf else float("nan")
            self._last_reject_reason = (
                f"only {len(yaws_deg)} IMU sample(s) fell inside GPS "
                f"window [{start_t:.1f}, {end_t:.1f}] -- latest IMU "
                f"t={latest_imu_t:.1f} (offset from window end="
                f"{latest_imu_t - end_t:+.1f}s). A large offset here means "
                f"/fix and /imu are on different clocks and this window "
                f"will never see any yaw samples, no matter how long you "
                f"drive.")
            self._maybe_log_status()
            self._advance_window()
            return

        yaw_spread = max(yaws_deg) - min(yaws_deg)
        # Handle wraparound crudely: if spread looks huge, re-check with
        # wrapped values before giving up.
        if yaw_spread > 180.0:
            wrapped = [wrap_deg(y) for y in yaws_deg]
            yaw_spread = max(wrapped) - min(wrapped)

        if yaw_spread > MAX_YAW_SPREAD_DEG:
            # Not straight enough — this window is contaminated by a turn.
            # Slide the window forward and keep looking.
            self._last_reject_reason = (
                f"not straight enough (yaw_spread={yaw_spread:.1f}deg > "
                f"{MAX_YAW_SPREAD_DEG}deg over dist={dist:.1f}m) -- steer "
                f"corrections may be too abrupt")
            self._maybe_log_status()
            self._advance_window()
            return

        # Qualifying straight segment. Compute candidate rotation angle:
        # raw GPS bearing (unrotated) vs. mean IMU yaw over the segment.
        gps_bearing_deg = math.degrees(math.atan2(x1 - x0, y1 - y0))
        mean_imu_yaw_deg = sum(yaws_deg) / len(yaws_deg)
        candidate_angle = wrap_deg(mean_imu_yaw_deg - gps_bearing_deg)

        self.candidates.append(candidate_angle)
        rospy.loginfo(
            f"[gps_rotation_calibrator] Segment #{len(self.candidates)} "
            f"qualified: dist={dist:.1f}m duration={duration:.1f}s "
            f"yaw_spread={yaw_spread:.1f}deg -> candidate={candidate_angle:.2f}deg")

        # Close this window out and start a fresh one right after it, so we
        # keep harvesting more segments for the rest of the drive.
        self.window_start_idx = len(self.gps_buf) - 1

    def _advance_window(self):
        # Slide the window start forward by a few samples rather than
        # resetting all the way to "now" — keeps some overlap so we don't
        # miss a straight stretch that started slightly before we noticed.
        step = max(1, len(self.gps_buf) // 50)
        self.window_start_idx = min(
            self.window_start_idx + step, len(self.gps_buf) - 1)

    def _maybe_log_status(self):
        # Throttled independently of WINDOW_STRIDE_S so this prints roughly
        # every 5s instead of every 1s -- gives a live "why hasn't anything
        # qualified yet" trail instead of total silence between the startup
        # message and the final shutdown warning.
        now = rospy.Time.now()
        if (now - self._last_status_log).to_sec() < 5.0:
            return
        self._last_status_log = now
        gps_t = self.gps_buf[-1][0] if self.gps_buf else float("nan")
        imu_t = self.imu_buf[-1][0] if self.imu_buf else float("nan")
        rospy.loginfo(
            f"[gps_rotation_calibrator] Still watching "
            f"({len(self.candidates)} segment(s) qualified so far). "
            f"gps_buf={len(self.gps_buf)} imu_buf={len(self.imu_buf)} "
            f"latest_gps_t={gps_t:.1f} latest_imu_t={imu_t:.1f} "
            f"(offset={imu_t - gps_t:+.1f}s). "
            f"Last reason no window closed: {self._last_reject_reason}")

    # ---------- Shutdown: finalize and write ----------
    def _on_shutdown(self):
        n = len(self.candidates)
        rospy.loginfo(
            f"[gps_rotation_calibrator] Shutting down. "
            f"{n} straight segment(s) collected.")

        if n < MIN_CANDIDATES_REQUIRED:
            rospy.logwarn(
                f"[gps_rotation_calibrator] Only {n} qualifying segment(s) "
                f"(need >= {MIN_CANDIDATES_REQUIRED}). NOT writing a new "
                f"calibration file — previous value stays in effect. "
                f"Consider a longer/straighter drive, or re-run this on "
                f"tomorrow's commute.")
            return

        # --- SAFE ANGULAR AVERAGING FIX ---
        ref = self.candidates[0]
        shifted_candidates = [wrap_deg(c - ref) for c in self.candidates]

        mean_shifted = sum(shifted_candidates) / n
        mean_angle = wrap_deg(mean_shifted + ref)
        spread = max(shifted_candidates) - min(shifted_candidates)
        # ----------------------------------

        if spread > MAX_SEGMENT_AGREEMENT_DEG:
            rospy.logwarn(
                f"[gps_rotation_calibrator] Candidates disagree too much "
                f"(spread={spread:.2f}deg > {MAX_SEGMENT_AGREEMENT_DEG}deg "
                f"threshold): {['%.2f' % c for c in self.candidates]}. "
                f"NOT writing a new calibration file — refusing to trust "
                f"a noisy result. Previous value stays in effect.")
            return

        result = {
            "rotation_angle_deg": round(mean_angle, 3),
            "calibrated_at": datetime.now().isoformat(timespec="seconds"),
            "segments_used": n,
            "segment_agreement_deg": round(spread, 3),
            "candidate_angles_deg": [round(c, 3) for c in self.candidates],
        }

        os.makedirs(os.path.dirname(self.calib_path), exist_ok=True)
        with open(self.calib_path, "w") as f:
            yaml.safe_dump(result, f, default_flow_style=False)

        rospy.loginfo(
            f"[gps_rotation_calibrator] Wrote calibration: "
            f"rotation_angle_deg={result['rotation_angle_deg']} "
            f"(agreement={result['segment_agreement_deg']}deg across "
            f"{n} segments) -> {self.calib_path}")
        rospy.loginfo(
            f"[gps_rotation_calibrator] For reference, previous hardcoded "
            f"fallback was {FALLBACK_ANGLE_DEG}deg "
            f"(delta={wrap_deg(mean_angle - FALLBACK_ANGLE_DEG):.2f}deg).")


if __name__ == "__main__":
    node = GpsRotationCalibrator()
    rospy.spin()