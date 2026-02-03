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

"""Diagnostic checks for ROS 2 system health."""

import os
import sys
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple


class CheckStatus(Enum):
    """Status of a diagnostic check."""
    PASS = 'PASS'
    WARN = 'WARN'
    FAIL = 'FAIL'
    SKIP = 'SKIP'


@dataclass
class CheckResult:
    """Result of a diagnostic check."""
    name: str
    status: CheckStatus
    message: str = ''
    details: Dict[str, Any] = field(default_factory=dict)
    suggestions: List[str] = field(default_factory=list)


def check_rmw_configuration() -> CheckResult:
    """
    Check RMW (ROS Middleware) configuration.

    Validates:
    - RMW implementation is set and available
    - Domain ID configuration
    - DDS discovery settings
    """
    details = {}
    suggestions = []

    # Check RMW implementation
    rmw_impl = os.environ.get('RMW_IMPLEMENTATION', '')
    if not rmw_impl:
        # Try to detect default RMW
        try:
            import rclpy
            from rclpy.utilities import get_rmw_implementation_identifier
            rclpy.init()
            rmw_impl = get_rmw_implementation_identifier()
            rclpy.shutdown()
        except Exception:
            rmw_impl = 'unknown (detection failed)'

    details['rmw_implementation'] = rmw_impl if rmw_impl else 'default'

    # Check domain ID
    domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
    try:
        domain_id_int = int(domain_id)
        if domain_id_int < 0 or domain_id_int > 232:
            return CheckResult(
                name='RMW Configuration',
                status=CheckStatus.WARN,
                message=f'Domain ID {domain_id_int} is outside recommended range (0-232)',
                details={'domain_id': domain_id_int, **details},
                suggestions=['Consider using a domain ID between 0 and 232']
            )
        details['domain_id'] = domain_id_int
    except ValueError:
        return CheckResult(
            name='RMW Configuration',
            status=CheckStatus.FAIL,
            message=f'Invalid ROS_DOMAIN_ID: {domain_id}',
            details=details,
            suggestions=['Set ROS_DOMAIN_ID to a valid integer (0-232)']
        )

    # Check localhost only setting
    localhost_only = os.environ.get('ROS_LOCALHOST_ONLY', '0')
    details['localhost_only'] = localhost_only == '1'

    # Check for common RMW implementations
    known_rmws = [
        'rmw_fastrtps_cpp',
        'rmw_fastrtps_dynamic_cpp',
        'rmw_cyclonedds_cpp',
        'rmw_connextdds',
    ]

    if rmw_impl and rmw_impl not in known_rmws and rmw_impl != 'default':
        suggestions.append(f'RMW "{rmw_impl}" is not a commonly known implementation')

    details['discovery'] = 'localhost' if details['localhost_only'] else 'multicast'

    return CheckResult(
        name='RMW Configuration',
        status=CheckStatus.PASS,
        message='RMW configuration is valid',
        details=details,
        suggestions=suggestions
    )


def check_node_connectivity() -> CheckResult:
    """
    Check ROS 2 node connectivity and health.

    Validates:
    - Active nodes can be discovered
    - Nodes are responding
    - No orphaned nodes detected
    """
    details = {}
    warnings = []

    try:
        import rclpy
        from rclpy.node import Node

        rclpy.init()
        node = Node('_doctor_check_node')

        # Get list of nodes
        node_names_and_namespaces = node.get_node_names_and_namespaces()

        # Filter out our own node
        nodes = [
            (name, ns) for name, ns in node_names_and_namespaces
            if name != '_doctor_check_node'
        ]

        details['node_count'] = len(nodes)
        details['nodes'] = [f'{ns}/{name}' if ns != '/' else f'/{name}'
                          for name, ns in nodes]

        # Check for potential issues
        orphaned = []
        for name, ns in nodes:
            full_name = f'{ns}/{name}' if ns != '/' else f'/{name}'
            # Check if node has any publishers or subscribers
            try:
                pubs = node.get_publisher_names_and_types_by_node(name, ns)
                subs = node.get_subscriber_names_and_types_by_node(name, ns)
                if not pubs and not subs:
                    orphaned.append(full_name)
            except Exception:
                pass

        node.destroy_node()
        rclpy.shutdown()

        if orphaned:
            details['orphaned_nodes'] = orphaned
            warnings.append(f'{len(orphaned)} node(s) have no publishers or subscribers')

        if not nodes:
            return CheckResult(
                name='Node Connectivity',
                status=CheckStatus.WARN,
                message='No active nodes found (besides this diagnostic node)',
                details=details,
                suggestions=['Start some ROS 2 nodes to verify connectivity']
            )

        if warnings:
            return CheckResult(
                name='Node Connectivity',
                status=CheckStatus.WARN,
                message='; '.join(warnings),
                details=details,
                suggestions=['Check orphaned nodes for proper configuration']
            )

        return CheckResult(
            name='Node Connectivity',
            status=CheckStatus.PASS,
            message=f'Found {len(nodes)} active node(s)',
            details=details
        )

    except ImportError:
        return CheckResult(
            name='Node Connectivity',
            status=CheckStatus.SKIP,
            message='rclpy not available',
            suggestions=['Install rclpy to enable node connectivity checks']
        )
    except Exception as e:
        return CheckResult(
            name='Node Connectivity',
            status=CheckStatus.FAIL,
            message=f'Failed to check node connectivity: {str(e)}',
            suggestions=['Ensure ROS 2 daemon is running: ros2 daemon start']
        )


def check_qos_compatibility() -> CheckResult:
    """
    Check QoS compatibility between publishers and subscribers.

    Validates:
    - Publisher/subscriber QoS policies are compatible
    - No reliability mismatches
    - No durability mismatches
    """
    details = {}
    issues = []

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile

        rclpy.init()
        node = Node('_doctor_qos_check_node')

        # Get all topics
        topic_names_and_types = node.get_topic_names_and_types()

        qos_issues = []
        checked_topics = []

        for topic_name, topic_types in topic_names_and_types:
            try:
                # Get publisher info
                pub_info = node.get_publishers_info_by_topic(topic_name)
                sub_info = node.get_subscriptions_info_by_topic(topic_name)

                if not pub_info or not sub_info:
                    continue

                checked_topics.append(topic_name)

                # Check QoS compatibility
                for pub in pub_info:
                    for sub in sub_info:
                        pub_qos = pub.qos_profile
                        sub_qos = sub.qos_profile

                        # Check reliability
                        # RELIABLE(2) pub with BEST_EFFORT(1) sub is OK
                        # BEST_EFFORT(1) pub with RELIABLE(2) sub is NOT OK
                        if (hasattr(pub_qos, 'reliability') and
                            hasattr(sub_qos, 'reliability')):
                            pub_rel = pub_qos.reliability
                            sub_rel = sub_qos.reliability
                            # Compare enum values
                            if hasattr(pub_rel, 'value') and hasattr(sub_rel, 'value'):
                                if pub_rel.value < sub_rel.value:
                                    qos_issues.append({
                                        'topic': topic_name,
                                        'issue': 'reliability_mismatch',
                                        'publisher': str(pub_rel),
                                        'subscriber': str(sub_rel)
                                    })

                        # Check durability
                        if (hasattr(pub_qos, 'durability') and
                            hasattr(sub_qos, 'durability')):
                            pub_dur = pub_qos.durability
                            sub_dur = sub_qos.durability
                            if hasattr(pub_dur, 'value') and hasattr(sub_dur, 'value'):
                                if pub_dur.value < sub_dur.value:
                                    qos_issues.append({
                                        'topic': topic_name,
                                        'issue': 'durability_mismatch',
                                        'publisher': str(pub_dur),
                                        'subscriber': str(sub_dur)
                                    })

            except Exception:
                continue

        node.destroy_node()
        rclpy.shutdown()

        details['topics_checked'] = len(checked_topics)
        details['issues_found'] = len(qos_issues)

        if qos_issues:
            details['qos_issues'] = qos_issues
            return CheckResult(
                name='QoS Compatibility',
                status=CheckStatus.WARN,
                message=f'Found {len(qos_issues)} QoS compatibility issue(s)',
                details=details,
                suggestions=[
                    'Review QoS settings for affected topics',
                    'Use compatible reliability/durability settings'
                ]
            )

        return CheckResult(
            name='QoS Compatibility',
            status=CheckStatus.PASS,
            message=f'Checked {len(checked_topics)} topics, no issues found',
            details=details
        )

    except ImportError:
        return CheckResult(
            name='QoS Compatibility',
            status=CheckStatus.SKIP,
            message='rclpy not available',
            suggestions=['Install rclpy to enable QoS compatibility checks']
        )
    except Exception as e:
        return CheckResult(
            name='QoS Compatibility',
            status=CheckStatus.FAIL,
            message=f'Failed to check QoS compatibility: {str(e)}',
            details={'error': str(e)}
        )


def check_resource_usage() -> CheckResult:
    """
    Check system resource usage.

    Validates:
    - Memory usage is within acceptable limits
    - CPU usage is not excessive
    - File descriptors are not exhausted
    """
    details = {}
    warnings = []
    suggestions = []

    try:
        import psutil

        # Memory check
        memory = psutil.virtual_memory()
        details['memory'] = {
            'total_gb': round(memory.total / (1024**3), 2),
            'available_gb': round(memory.available / (1024**3), 2),
            'percent_used': memory.percent
        }

        if memory.percent > 90:
            warnings.append(f'High memory usage: {memory.percent}%')
            suggestions.append('Consider closing unused applications or adding more RAM')
        elif memory.percent > 80:
            warnings.append(f'Memory usage approaching limit: {memory.percent}%')

        # CPU check
        cpu_percent = psutil.cpu_percent(interval=0.5)
        cpu_count = psutil.cpu_count()
        details['cpu'] = {
            'percent_used': cpu_percent,
            'core_count': cpu_count
        }

        if cpu_percent > 90:
            warnings.append(f'High CPU usage: {cpu_percent}%')
            suggestions.append('Check for runaway processes')

        # File descriptor check (Linux/Unix)
        if sys.platform != 'win32':
            try:
                import resource
                soft_limit, hard_limit = resource.getrlimit(resource.RLIMIT_NOFILE)

                # Count open file descriptors for current process
                current_process = psutil.Process()
                open_fds = current_process.num_fds()

                details['file_descriptors'] = {
                    'open': open_fds,
                    'soft_limit': soft_limit,
                    'hard_limit': hard_limit,
                    'percent_used': round((open_fds / soft_limit) * 100, 2)
                }

                if open_fds / soft_limit > 0.8:
                    warnings.append(f'File descriptor usage high: {open_fds}/{soft_limit}')
                    suggestions.append('Increase file descriptor limit with ulimit -n')

            except Exception:
                pass

        # Disk check
        disk = psutil.disk_usage('/')
        details['disk'] = {
            'total_gb': round(disk.total / (1024**3), 2),
            'free_gb': round(disk.free / (1024**3), 2),
            'percent_used': disk.percent
        }

        if disk.percent > 95:
            warnings.append(f'Disk almost full: {disk.percent}%')
            suggestions.append('Free up disk space')
        elif disk.percent > 90:
            warnings.append(f'Disk usage high: {disk.percent}%')

        if warnings:
            return CheckResult(
                name='Resource Usage',
                status=CheckStatus.WARN,
                message='; '.join(warnings),
                details=details,
                suggestions=suggestions
            )

        return CheckResult(
            name='Resource Usage',
            status=CheckStatus.PASS,
            message='System resources within normal limits',
            details=details
        )

    except ImportError:
        return CheckResult(
            name='Resource Usage',
            status=CheckStatus.SKIP,
            message='psutil not available',
            details={},
            suggestions=['Install psutil for resource monitoring: pip install psutil']
        )
    except Exception as e:
        return CheckResult(
            name='Resource Usage',
            status=CheckStatus.FAIL,
            message=f'Failed to check resources: {str(e)}',
            details={'error': str(e)}
        )


def check_environment() -> CheckResult:
    """
    Check ROS 2 environment configuration.

    Validates:
    - ROS_DISTRO is set
    - Required environment variables are configured
    - Workspace is properly sourced
    """
    details = {}
    warnings = []
    suggestions = []

    # Check ROS_DISTRO
    ros_distro = os.environ.get('ROS_DISTRO', '')
    details['ros_distro'] = ros_distro if ros_distro else 'not set'

    if not ros_distro:
        return CheckResult(
            name='Environment',
            status=CheckStatus.FAIL,
            message='ROS_DISTRO not set',
            details=details,
            suggestions=[
                'Source ROS 2 setup: source /opt/ros/<distro>/setup.bash',
                'Replace <distro> with your ROS 2 distribution (e.g., humble, iron, rolling)'
            ]
        )

    # Check known distributions
    known_distros = ['foxy', 'galactic', 'humble', 'iron', 'jazzy', 'rolling']
    if ros_distro not in known_distros:
        warnings.append(f'Unknown ROS distribution: {ros_distro}')

    # Check AMENT_PREFIX_PATH
    ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    details['ament_prefix_path'] = ament_prefix.split(':') if ament_prefix else []

    if not ament_prefix:
        warnings.append('AMENT_PREFIX_PATH not set')
        suggestions.append('Source your workspace setup file')

    # Check PYTHONPATH for ROS packages
    python_path = os.environ.get('PYTHONPATH', '')
    ros_in_pythonpath = any('ros' in p.lower() for p in python_path.split(':'))
    details['ros_in_pythonpath'] = ros_in_pythonpath

    if not ros_in_pythonpath:
        warnings.append('ROS 2 Python packages may not be in PYTHONPATH')

    # Check CMAKE_PREFIX_PATH
    cmake_prefix = os.environ.get('CMAKE_PREFIX_PATH', '')
    details['cmake_prefix_path_set'] = bool(cmake_prefix)

    # Check ROS_VERSION
    ros_version = os.environ.get('ROS_VERSION', '')
    details['ros_version'] = ros_version if ros_version else 'not set'

    if ros_version and ros_version != '2':
        warnings.append(f'ROS_VERSION is {ros_version}, expected 2')

    # Check COLCON_PREFIX_PATH for workspace
    colcon_prefix = os.environ.get('COLCON_PREFIX_PATH', '')
    details['colcon_prefix_path'] = colcon_prefix.split(':') if colcon_prefix else []

    if warnings:
        return CheckResult(
            name='Environment',
            status=CheckStatus.WARN,
            message='; '.join(warnings),
            details=details,
            suggestions=suggestions
        )

    return CheckResult(
        name='Environment',
        status=CheckStatus.PASS,
        message=f'ROS 2 {ros_distro} environment configured correctly',
        details=details
    )


def run_all_checks(
    include_warnings: bool = True,
    exclude_checks: Optional[List[str]] = None
) -> Tuple[List[CheckResult], Dict[str, int]]:
    """
    Run all diagnostic checks.

    Args:
        include_warnings: Whether to include warnings in failure count
        exclude_checks: List of check names to skip

    Returns:
        Tuple of (list of CheckResults, summary dict)
    """
    exclude_checks = exclude_checks or []

    all_checks = [
        ('RMW Configuration', check_rmw_configuration),
        ('Node Connectivity', check_node_connectivity),
        ('QoS Compatibility', check_qos_compatibility),
        ('Resource Usage', check_resource_usage),
        ('Environment', check_environment),
    ]

    results = []
    summary = {'passed': 0, 'warnings': 0, 'failed': 0, 'skipped': 0}

    for check_name, check_func in all_checks:
        if check_name in exclude_checks:
            results.append(CheckResult(
                name=check_name,
                status=CheckStatus.SKIP,
                message='Excluded by user'
            ))
            summary['skipped'] += 1
            continue

        try:
            result = check_func()
            results.append(result)

            if result.status == CheckStatus.PASS:
                summary['passed'] += 1
            elif result.status == CheckStatus.WARN:
                summary['warnings'] += 1
            elif result.status == CheckStatus.FAIL:
                summary['failed'] += 1
            else:
                summary['skipped'] += 1

        except Exception as e:
            results.append(CheckResult(
                name=check_name,
                status=CheckStatus.FAIL,
                message=f'Check failed with exception: {str(e)}'
            ))
            summary['failed'] += 1

    return results, summary
