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

"""Output formatters for ros2stats commands."""

import json
import sys
from datetime import datetime
from typing import Any, Dict, List

from .statistics import TopicStats, LatencyStats


# ANSI color codes
class Colors:
    """ANSI color codes for terminal output."""
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    DIM = '\033[2m'
    RESET = '\033[0m'


def supports_color() -> bool:
    """Check if the terminal supports color output."""
    import os
    if not hasattr(sys.stdout, 'isatty'):
        return False
    if not sys.stdout.isatty():
        return False
    if os.environ.get('NO_COLOR'):
        return False
    return True


def colorize(text: str, color: str, force: bool = False) -> str:
    """
    Apply color to text if terminal supports it.

    Args:
        text: Text to colorize
        color: Color name
        force: Force colorization

    Returns:
        Colorized text string
    """
    if not force and not supports_color():
        return text

    color_map = {
        'green': Colors.GREEN,
        'yellow': Colors.YELLOW,
        'blue': Colors.BLUE,
        'cyan': Colors.CYAN,
        'bold': Colors.BOLD,
        'dim': Colors.DIM,
    }

    color_code = color_map.get(color.lower(), '')
    if color_code:
        return f'{color_code}{text}{Colors.RESET}'
    return text


def format_bytes(bytes_value: float) -> str:
    """Format bytes into human-readable string."""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if abs(bytes_value) < 1024.0:
            return f'{bytes_value:.1f} {unit}'
        bytes_value /= 1024.0
    return f'{bytes_value:.1f} TB'


def format_topic_stats(stats: TopicStats, verbose: bool = False) -> str:
    """
    Format topic statistics for terminal output.

    Args:
        stats: TopicStats object
        verbose: Show detailed information

    Returns:
        Formatted string for terminal output
    """
    lines = []
    separator = '=' * 80

    # Header
    lines.append(colorize(separator, 'bold'))
    header = f'Topic Statistics: {stats.topic_name}'
    lines.append(colorize(header.center(80), 'bold'))
    lines.append(colorize(separator, 'bold'))
    lines.append('')

    if stats.message_type:
        lines.append(f'Message Type: {colorize(stats.message_type, "cyan")}')
        lines.append('')

    # Message Rate section
    lines.append(colorize('Message Rate:', 'bold'))
    lines.append(f'  Current:    {stats.rate_current:.1f} msg/s')
    lines.append(f'  Average:    {stats.rate_average:.1f} msg/s')
    if stats.rate_min != float('inf'):
        lines.append(f'  Min/Max:    {stats.rate_min:.1f} / {stats.rate_max:.1f} msg/s')
    if stats.rate_std_dev > 0:
        lines.append(f'  Std Dev:    {stats.rate_std_dev:.1f} msg/s')
    lines.append(f'  Total:      {stats.message_count} messages')
    lines.append('')

    # Bandwidth section
    lines.append(colorize('Bandwidth:', 'bold'))
    lines.append(f'  Current:    {format_bytes(stats.bandwidth_current)}/s')
    lines.append(f'  Total:      {format_bytes(stats.total_bytes)} (transferred)')
    if stats.msg_size_avg > 0:
        lines.append(f'  Msg Size:   {format_bytes(stats.msg_size_avg)} (avg)')
    if verbose and stats.msg_size_min > 0:
        lines.append(f'  Size Range: {format_bytes(stats.msg_size_min)} - {format_bytes(stats.msg_size_max)}')
    lines.append('')

    # Latency section (if available)
    if stats.latency_average_ms > 0:
        lines.append(colorize('Latency:', 'bold'))
        lines.append(f'  Current:    {stats.latency_current_ms:.1f} ms')
        lines.append(f'  Average:    {stats.latency_average_ms:.1f} ms')
        if stats.latency_min_ms != float('inf'):
            lines.append(f'  Min/Max:    {stats.latency_min_ms:.1f} / {stats.latency_max_ms:.1f} ms')
        lines.append('')

    # Connections section
    lines.append(colorize('Connections:', 'bold'))
    lines.append(f'  Publishers:     {stats.publisher_count}')
    if stats.publishers and verbose:
        for pub in stats.publishers:
            lines.append(f'                  - {pub}')
    lines.append(f'  Subscribers:    {stats.subscriber_count}')
    if stats.subscribers and verbose:
        for sub in stats.subscribers:
            lines.append(f'                  - {sub}')
    lines.append('')

    lines.append(colorize(separator, 'dim'))

    return '\n'.join(lines)


def format_topic_stats_json(stats: TopicStats) -> str:
    """
    Format topic statistics as JSON.

    Args:
        stats: TopicStats object

    Returns:
        JSON formatted string
    """
    output = {
        'command': 'ros2 topic stats',
        'timestamp': datetime.utcnow().isoformat() + 'Z',
        'topic': stats.topic_name,
        'message_type': stats.message_type,
        'message_rate': {
            'current': round(stats.rate_current, 2),
            'average': round(stats.rate_average, 2),
            'min': round(stats.rate_min, 2) if stats.rate_min != float('inf') else None,
            'max': round(stats.rate_max, 2),
            'std_dev': round(stats.rate_std_dev, 2),
            'total_messages': stats.message_count,
        },
        'bandwidth': {
            'current_bytes_per_sec': round(stats.bandwidth_current, 2),
            'average_bytes_per_sec': round(stats.bandwidth_average, 2),
            'total_bytes': stats.total_bytes,
            'message_size': {
                'min': stats.msg_size_min,
                'max': stats.msg_size_max,
                'average': round(stats.msg_size_avg, 2),
            }
        },
        'latency_ms': {
            'current': round(stats.latency_current_ms, 2),
            'average': round(stats.latency_average_ms, 2),
            'min': round(stats.latency_min_ms, 2) if stats.latency_min_ms != float('inf') else None,
            'max': round(stats.latency_max_ms, 2),
        },
        'connections': {
            'publishers': stats.publisher_count,
            'publisher_nodes': stats.publishers,
            'subscribers': stats.subscriber_count,
            'subscriber_nodes': stats.subscribers,
        }
    }

    return json.dumps(output, indent=2)


def format_topic_stats_csv(stats: TopicStats) -> str:
    """
    Format topic statistics as CSV.

    Args:
        stats: TopicStats object

    Returns:
        CSV formatted string
    """
    header = 'topic,message_type,rate_current,rate_avg,bandwidth_bps,total_bytes,latency_ms,publishers,subscribers'
    data = (
        f'{stats.topic_name},{stats.message_type},'
        f'{stats.rate_current:.2f},{stats.rate_average:.2f},'
        f'{stats.bandwidth_current:.2f},{stats.total_bytes},'
        f'{stats.latency_average_ms:.2f},'
        f'{stats.publisher_count},{stats.subscriber_count}'
    )
    return f'{header}\n{data}'


def format_latency_stats(stats: LatencyStats, show_histogram: bool = True) -> str:
    """
    Format latency statistics for terminal output.

    Args:
        stats: LatencyStats object
        show_histogram: Whether to show histogram

    Returns:
        Formatted string for terminal output
    """
    lines = []
    separator = '=' * 80

    # Header
    lines.append(colorize(separator, 'bold'))
    header = f'Latency Measurement: {stats.source} -> {stats.destination}'
    lines.append(colorize(header.center(80), 'bold'))
    lines.append(colorize(separator, 'bold'))
    lines.append('')

    # Main stats
    lines.append(colorize(f'End-to-End Latency ({stats.samples} samples):', 'bold'))
    lines.append(f'  Average:    {stats.average_ms:.1f} ms')
    lines.append(f'  Median:     {stats.median_ms:.1f} ms')
    if stats.min_ms != float('inf'):
        lines.append(f'  Min:        {stats.min_ms:.1f} ms')
    lines.append(f'  Max:        {stats.max_ms:.1f} ms')
    lines.append(f'  Std Dev:    {stats.std_dev_ms:.1f} ms')
    lines.append('')

    # Percentiles
    lines.append(colorize('Percentiles:', 'bold'))
    lines.append(f'  P50:        {stats.p50_ms:.1f} ms')
    lines.append(f'  P90:        {stats.p90_ms:.1f} ms')
    lines.append(f'  P95:        {stats.p95_ms:.1f} ms')
    lines.append(f'  P99:        {stats.p99_ms:.1f} ms')
    lines.append('')

    # Histogram
    if show_histogram and stats.histogram:
        lines.append(colorize('Histogram:', 'bold'))
        max_count = max(stats.histogram.values()) if stats.histogram else 1

        for bucket, count in sorted(stats.histogram.items()):
            bar_len = int((count / max_count) * 20)
            bar = colorize('█' * bar_len, 'green') + '░' * (20 - bar_len)
            lines.append(f'  {bucket:>15}: {bar} {count:>4} samples')
        lines.append('')

    # Per-hop breakdown
    if stats.hops:
        lines.append(colorize('Breakdown:', 'bold'))
        total_latency = sum(lat for _, _, lat in stats.hops)
        for src, dst, latency in stats.hops:
            percent = (latency / total_latency * 100) if total_latency > 0 else 0
            lines.append(f'  {src} -> {dst}:   {latency:.1f} ms ({percent:.0f}%)')
        lines.append('')

    lines.append(colorize(separator, 'bold'))

    return '\n'.join(lines)


def format_latency_stats_json(stats: LatencyStats) -> str:
    """
    Format latency statistics as JSON.

    Args:
        stats: LatencyStats object

    Returns:
        JSON formatted string
    """
    output = {
        'command': 'ros2 latency',
        'timestamp': datetime.utcnow().isoformat() + 'Z',
        'source': stats.source,
        'destination': stats.destination,
        'samples': stats.samples,
        'latency_ms': {
            'average': round(stats.average_ms, 2),
            'median': round(stats.median_ms, 2),
            'min': round(stats.min_ms, 2) if stats.min_ms != float('inf') else None,
            'max': round(stats.max_ms, 2),
            'std_dev': round(stats.std_dev_ms, 2),
        },
        'percentiles_ms': {
            'p50': round(stats.p50_ms, 2),
            'p90': round(stats.p90_ms, 2),
            'p95': round(stats.p95_ms, 2),
            'p99': round(stats.p99_ms, 2),
        },
        'histogram': stats.histogram,
        'hops': [
            {'source': src, 'destination': dst, 'latency_ms': round(lat, 2)}
            for src, dst, lat in stats.hops
        ] if stats.hops else []
    }

    return json.dumps(output, indent=2)
