# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Statistics collection for ROS 2 topics and latency measurement."""

import math
import time
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple


@dataclass
class TopicStats:
    """Statistics for a single topic."""
    topic_name: str
    message_type: str = ''

    # Message rate stats
    message_count: int = 0
    rate_current: float = 0.0
    rate_average: float = 0.0
    rate_min: float = float('inf')
    rate_max: float = 0.0
    rate_std_dev: float = 0.0

    # Bandwidth stats
    bandwidth_current: float = 0.0  # bytes/sec
    bandwidth_average: float = 0.0
    total_bytes: int = 0
    msg_size_min: int = 0
    msg_size_max: int = 0
    msg_size_avg: float = 0.0

    # Latency stats (header to receive time)
    latency_current_ms: float = 0.0
    latency_average_ms: float = 0.0
    latency_min_ms: float = float('inf')
    latency_max_ms: float = 0.0

    # Connection info
    publisher_count: int = 0
    subscriber_count: int = 0
    publishers: List[str] = field(default_factory=list)
    subscribers: List[str] = field(default_factory=list)


@dataclass
class LatencyStats:
    """Latency statistics for end-to-end measurement."""
    source: str
    destination: str
    samples: int = 0

    # Overall stats (in milliseconds)
    average_ms: float = 0.0
    median_ms: float = 0.0
    min_ms: float = float('inf')
    max_ms: float = 0.0
    std_dev_ms: float = 0.0

    # Percentiles
    p50_ms: float = 0.0
    p90_ms: float = 0.0
    p95_ms: float = 0.0
    p99_ms: float = 0.0

    # Histogram data
    histogram: Dict[str, int] = field(default_factory=dict)

    # Per-hop breakdown
    hops: List[Tuple[str, str, float]] = field(default_factory=list)


class TopicStatisticsCollector:
    """Collects and computes statistics for ROS 2 topics."""

    def __init__(self, topic_name: str, window_size: int = 100):
        """
        Initialize statistics collector.

        Args:
            topic_name: Name of the topic to monitor
            window_size: Number of samples to keep for statistics
        """
        self.topic_name = topic_name
        self.window_size = window_size

        # Sample storage
        self._timestamps: deque = deque(maxlen=window_size)
        self._sizes: deque = deque(maxlen=window_size)
        self._latencies: deque = deque(maxlen=window_size)
        self._rates: deque = deque(maxlen=window_size)

        # Counters
        self._message_count = 0
        self._total_bytes = 0
        self._start_time = time.time()
        self._last_time: Optional[float] = None

        # Thread safety
        self._lock = threading.Lock()

        # Current stats
        self._stats = TopicStats(topic_name=topic_name)

    def record_message(
        self,
        timestamp: float,
        size: int,
        header_stamp: Optional[float] = None
    ):
        """
        Record a received message.

        Args:
            timestamp: Time message was received (seconds since epoch)
            size: Size of message in bytes
            header_stamp: Optional timestamp from message header (seconds since epoch)
        """
        with self._lock:
            self._message_count += 1
            self._total_bytes += size

            # Calculate rate
            if self._last_time is not None:
                dt = timestamp - self._last_time
                if dt > 0:
                    rate = 1.0 / dt
                    self._rates.append(rate)

            self._last_time = timestamp
            self._timestamps.append(timestamp)
            self._sizes.append(size)

            # Calculate latency from header
            if header_stamp is not None:
                latency = (timestamp - header_stamp) * 1000  # Convert to ms
                if latency >= 0:  # Ignore negative latencies (clock skew)
                    self._latencies.append(latency)

    def get_stats(self) -> TopicStats:
        """
        Compute and return current statistics.

        Returns:
            TopicStats object with current statistics
        """
        with self._lock:
            stats = TopicStats(
                topic_name=self.topic_name,
                message_count=self._message_count,
                total_bytes=self._total_bytes,
            )

            # Rate statistics
            if self._rates:
                rates = list(self._rates)
                stats.rate_current = rates[-1] if rates else 0.0
                stats.rate_average = sum(rates) / len(rates)
                stats.rate_min = min(rates)
                stats.rate_max = max(rates)
                stats.rate_std_dev = self._std_dev(rates)

            # Size statistics
            if self._sizes:
                sizes = list(self._sizes)
                stats.msg_size_min = min(sizes)
                stats.msg_size_max = max(sizes)
                stats.msg_size_avg = sum(sizes) / len(sizes)

            # Bandwidth calculation
            if self._timestamps and len(self._timestamps) > 1:
                time_span = self._timestamps[-1] - self._timestamps[0]
                if time_span > 0:
                    recent_bytes = sum(self._sizes)
                    stats.bandwidth_current = recent_bytes / time_span
                    stats.bandwidth_average = self._total_bytes / (time.time() - self._start_time)

            # Latency statistics
            if self._latencies:
                latencies = list(self._latencies)
                stats.latency_current_ms = latencies[-1] if latencies else 0.0
                stats.latency_average_ms = sum(latencies) / len(latencies)
                stats.latency_min_ms = min(latencies)
                stats.latency_max_ms = max(latencies)

            return stats

    def _std_dev(self, values: List[float]) -> float:
        """Calculate standard deviation."""
        if len(values) < 2:
            return 0.0
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / (len(values) - 1)
        return math.sqrt(variance)


class LatencyMeasurement:
    """End-to-end latency measurement utility."""

    def __init__(self, sample_count: int = 100):
        """
        Initialize latency measurement.

        Args:
            sample_count: Number of samples to collect
        """
        self.sample_count = sample_count
        self._samples: List[float] = []
        self._lock = threading.Lock()

    def record_sample(self, latency_ms: float):
        """
        Record a latency sample.

        Args:
            latency_ms: Latency in milliseconds
        """
        with self._lock:
            self._samples.append(latency_ms)

    def get_stats(self, source: str = '', destination: str = '') -> LatencyStats:
        """
        Compute and return latency statistics.

        Args:
            source: Source node/topic name
            destination: Destination node/topic name

        Returns:
            LatencyStats object with computed statistics
        """
        with self._lock:
            if not self._samples:
                return LatencyStats(source=source, destination=destination)

            samples = sorted(self._samples)
            n = len(samples)

            stats = LatencyStats(
                source=source,
                destination=destination,
                samples=n,
                average_ms=sum(samples) / n,
                min_ms=samples[0],
                max_ms=samples[-1],
            )

            # Median
            if n % 2 == 0:
                stats.median_ms = (samples[n // 2 - 1] + samples[n // 2]) / 2
            else:
                stats.median_ms = samples[n // 2]

            # Percentiles
            stats.p50_ms = self._percentile(samples, 50)
            stats.p90_ms = self._percentile(samples, 90)
            stats.p95_ms = self._percentile(samples, 95)
            stats.p99_ms = self._percentile(samples, 99)

            # Standard deviation
            mean = stats.average_ms
            if n > 1:
                variance = sum((x - mean) ** 2 for x in samples) / (n - 1)
                stats.std_dev_ms = math.sqrt(variance)

            # Histogram
            stats.histogram = self._build_histogram(samples)

            return stats

    def _percentile(self, sorted_samples: List[float], percentile: int) -> float:
        """Calculate percentile from sorted samples."""
        if not sorted_samples:
            return 0.0
        k = (len(sorted_samples) - 1) * percentile / 100
        f = math.floor(k)
        c = math.ceil(k)
        if f == c:
            return sorted_samples[int(k)]
        return sorted_samples[int(f)] * (c - k) + sorted_samples[int(c)] * (k - f)

    def _build_histogram(self, sorted_samples: List[float]) -> Dict[str, int]:
        """Build histogram from samples."""
        if not sorted_samples:
            return {}

        min_val = sorted_samples[0]
        max_val = sorted_samples[-1]

        if max_val == min_val:
            return {f'{min_val:.1f}': len(sorted_samples)}

        # Create ~10 bins
        range_val = max_val - min_val
        bin_size = range_val / 10

        # Round bin_size to nice numbers
        magnitude = 10 ** math.floor(math.log10(bin_size))
        bin_size = math.ceil(bin_size / magnitude) * magnitude

        histogram = {}
        for sample in sorted_samples:
            bin_start = int(sample / bin_size) * bin_size
            bin_end = bin_start + bin_size
            key = f'{bin_start:.0f}-{bin_end:.0f} ms'
            histogram[key] = histogram.get(key, 0) + 1

        return histogram

    @property
    def sample_count_current(self) -> int:
        """Get current number of samples collected."""
        with self._lock:
            return len(self._samples)


def collect_topic_stats(
    topic_name: str,
    duration: float = 5.0,
    callback: Optional[Callable[[TopicStats], None]] = None
) -> TopicStats:
    """
    Collect statistics for a topic over a period of time.

    Args:
        topic_name: Name of the topic to monitor
        duration: Duration to collect data in seconds
        callback: Optional callback for periodic updates

    Returns:
        TopicStats with collected statistics
    """
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message

        rclpy.init()
        node = Node('_stats_collector_node')

        # Get topic type
        topic_info = node.get_topic_names_and_types()
        topic_type = None
        for name, types in topic_info:
            if name == topic_name and types:
                topic_type = types[0]
                break

        if not topic_type:
            node.destroy_node()
            rclpy.shutdown()
            return TopicStats(topic_name=topic_name)

        # Get message class
        msg_class = get_message(topic_type)

        collector = TopicStatisticsCollector(topic_name)
        collector._stats.message_type = topic_type

        # Get connection info
        pub_info = node.get_publishers_info_by_topic(topic_name)
        sub_info = node.get_subscriptions_info_by_topic(topic_name)

        collector._stats.publisher_count = len(pub_info)
        collector._stats.subscriber_count = len(sub_info)
        collector._stats.publishers = [
            getattr(p, 'node_name', 'unknown') for p in pub_info
        ]
        collector._stats.subscribers = [
            getattr(s, 'node_name', 'unknown') for s in sub_info
        ]

        def message_callback(msg):
            recv_time = time.time()

            # Try to get message size (approximation)
            try:
                import sys
                size = sys.getsizeof(msg)
            except Exception:
                size = 0

            # Try to get header timestamp
            header_stamp = None
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                stamp = msg.header.stamp
                header_stamp = stamp.sec + stamp.nanosec * 1e-9

            collector.record_message(recv_time, size, header_stamp)

            if callback:
                callback(collector.get_stats())

        # Create subscription
        subscription = node.create_subscription(
            msg_class,
            topic_name,
            message_callback,
            qos_profile_sensor_data
        )

        # Spin for duration
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(node, timeout_sec=0.1)

        stats = collector.get_stats()

        node.destroy_node()
        rclpy.shutdown()

        return stats

    except ImportError as e:
        return TopicStats(topic_name=topic_name)
    except Exception as e:
        return TopicStats(topic_name=topic_name)
