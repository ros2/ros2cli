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

"""Issue analysis for ROS 2 debugging."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple


class IssueSeverity(Enum):
    """Severity level of a detected issue."""
    CRITICAL = 'critical'
    WARNING = 'warning'
    INFO = 'info'


@dataclass
class Issue:
    """Represents a detected issue in the ROS 2 system."""
    title: str
    severity: IssueSeverity
    problem: str
    impact: str
    suggestions: List[str] = field(default_factory=list)
    topic: Optional[str] = None
    node: Optional[str] = None
    details: Dict[str, Any] = field(default_factory=dict)


def get_qos_issues(node) -> List[Issue]:
    """
    Detect QoS compatibility issues.

    Args:
        node: ROS 2 node instance for querying graph

    Returns:
        List of Issue objects for QoS problems
    """
    issues = []

    try:
        topic_names_and_types = node.get_topic_names_and_types()

        for topic_name, topic_types in topic_names_and_types:
            pub_info = node.get_publishers_info_by_topic(topic_name)
            sub_info = node.get_subscriptions_info_by_topic(topic_name)

            if not pub_info or not sub_info:
                continue

            for pub in pub_info:
                for sub in sub_info:
                    pub_qos = pub.qos_profile
                    sub_qos = sub.qos_profile

                    # Check reliability mismatch
                    if hasattr(pub_qos, 'reliability') and hasattr(sub_qos, 'reliability'):
                        pub_rel = getattr(pub_qos.reliability, 'value', None)
                        sub_rel = getattr(sub_qos.reliability, 'value', None)

                        # Best Effort (1) publisher with Reliable (2) subscriber
                        if pub_rel is not None and sub_rel is not None:
                            if pub_rel == 1 and sub_rel == 2:  # BEST_EFFORT pub, RELIABLE sub
                                pub_node = getattr(pub, 'node_name', 'unknown')
                                sub_node = getattr(sub, 'node_name', 'unknown')

                                issues.append(Issue(
                                    title=f'QoS Mismatch on {topic_name}',
                                    severity=IssueSeverity.CRITICAL,
                                    problem=(
                                        f'Publisher ({pub_node}) uses BEST_EFFORT reliability\n'
                                        f'  Subscriber ({sub_node}) requires RELIABLE reliability'
                                    ),
                                    impact='Messages will be dropped, causing data loss',
                                    topic=topic_name,
                                    details={
                                        'publisher_node': pub_node,
                                        'subscriber_node': sub_node,
                                        'publisher_reliability': 'BEST_EFFORT',
                                        'subscriber_reliability': 'RELIABLE',
                                    },
                                    suggestions=[
                                        f'Change subscriber QoS to BEST_EFFORT:\n'
                                        f'    rclpy.qos.QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)',
                                        f'Change publisher QoS to RELIABLE:\n'
                                        f'    rclpy.qos.QoSProfile(reliability=ReliabilityPolicy.RELIABLE)',
                                    ]
                                ))

                    # Check durability mismatch
                    if hasattr(pub_qos, 'durability') and hasattr(sub_qos, 'durability'):
                        pub_dur = getattr(pub_qos.durability, 'value', None)
                        sub_dur = getattr(sub_qos.durability, 'value', None)

                        # Volatile (1) publisher with Transient Local (2) subscriber
                        if pub_dur is not None and sub_dur is not None:
                            if pub_dur == 1 and sub_dur == 2:  # VOLATILE pub, TRANSIENT_LOCAL sub
                                pub_node = getattr(pub, 'node_name', 'unknown')
                                sub_node = getattr(sub, 'node_name', 'unknown')

                                issues.append(Issue(
                                    title=f'Durability Mismatch on {topic_name}',
                                    severity=IssueSeverity.WARNING,
                                    problem=(
                                        f'Publisher ({pub_node}) uses VOLATILE durability\n'
                                        f'  Subscriber ({sub_node}) expects TRANSIENT_LOCAL durability'
                                    ),
                                    impact='Late-joining subscribers will miss previously published messages',
                                    topic=topic_name,
                                    details={
                                        'publisher_node': pub_node,
                                        'subscriber_node': sub_node,
                                        'publisher_durability': 'VOLATILE',
                                        'subscriber_durability': 'TRANSIENT_LOCAL',
                                    },
                                    suggestions=[
                                        'Change publisher QoS to TRANSIENT_LOCAL for late-joining support',
                                        'Change subscriber QoS to VOLATILE if late-joining is not needed',
                                    ]
                                ))

    except Exception:
        pass

    return issues


def get_type_mismatches(node) -> List[Issue]:
    """
    Detect topic type mismatches.

    Args:
        node: ROS 2 node instance for querying graph

    Returns:
        List of Issue objects for type mismatch problems
    """
    issues = []

    try:
        topic_names_and_types = node.get_topic_names_and_types()

        for topic_name, topic_types in topic_names_and_types:
            if len(topic_types) > 1:
                issues.append(Issue(
                    title=f'Multiple Types on {topic_name}',
                    severity=IssueSeverity.CRITICAL,
                    problem=f'Topic has multiple message types: {", ".join(topic_types)}',
                    impact='Type confusion will cause deserialization failures',
                    topic=topic_name,
                    details={'types': topic_types},
                    suggestions=[
                        'Ensure all publishers and subscribers use the same message type',
                        'Check for namespace or remapping conflicts',
                        'Verify message package versions are consistent',
                    ]
                ))

    except Exception:
        pass

    return issues


def get_orphaned_publishers(node) -> List[Issue]:
    """
    Detect publishers with no subscribers.

    Args:
        node: ROS 2 node instance for querying graph

    Returns:
        List of Issue objects for orphaned publishers
    """
    issues = []

    try:
        topic_names_and_types = node.get_topic_names_and_types()

        # Exclude common ROS 2 internal topics
        internal_topics = {
            '/parameter_events',
            '/rosout',
            '/clock',
        }

        for topic_name, topic_types in topic_names_and_types:
            if topic_name in internal_topics:
                continue

            pub_info = node.get_publishers_info_by_topic(topic_name)
            sub_info = node.get_subscriptions_info_by_topic(topic_name)

            # Filter out our own node
            pub_info = [p for p in pub_info if getattr(p, 'node_name', '') != '_wtf_analysis_node']
            sub_info = [s for s in sub_info if getattr(s, 'node_name', '') != '_wtf_analysis_node']

            if pub_info and not sub_info:
                pub_nodes = [getattr(p, 'node_name', 'unknown') for p in pub_info]
                issues.append(Issue(
                    title=f'No Subscribers for {topic_name}',
                    severity=IssueSeverity.WARNING,
                    problem=f'Publisher(s) {", ".join(pub_nodes)} publishing with no subscribers',
                    impact='Messages being published with no consumers (wasted resources)',
                    topic=topic_name,
                    details={'publisher_nodes': pub_nodes},
                    suggestions=[
                        'Verify subscriber nodes are running',
                        'Check topic remapping configuration',
                        'Verify namespace configuration',
                    ]
                ))

    except Exception:
        pass

    return issues


def get_orphaned_subscribers(node) -> List[Issue]:
    """
    Detect subscribers with no publishers.

    Args:
        node: ROS 2 node instance for querying graph

    Returns:
        List of Issue objects for orphaned subscribers
    """
    issues = []

    try:
        topic_names_and_types = node.get_topic_names_and_types()

        # Exclude common ROS 2 internal topics
        internal_topics = {
            '/parameter_events',
            '/rosout',
            '/clock',
        }

        for topic_name, topic_types in topic_names_and_types:
            if topic_name in internal_topics:
                continue

            pub_info = node.get_publishers_info_by_topic(topic_name)
            sub_info = node.get_subscriptions_info_by_topic(topic_name)

            # Filter out our own node
            pub_info = [p for p in pub_info if getattr(p, 'node_name', '') != '_wtf_analysis_node']
            sub_info = [s for s in sub_info if getattr(s, 'node_name', '') != '_wtf_analysis_node']

            if sub_info and not pub_info:
                sub_nodes = [getattr(s, 'node_name', 'unknown') for s in sub_info]
                issues.append(Issue(
                    title=f'No Publishers for {topic_name}',
                    severity=IssueSeverity.CRITICAL,
                    problem=f'Subscriber(s) {", ".join(sub_nodes)} waiting for data that never arrives',
                    impact='Subscribers blocked waiting for data',
                    topic=topic_name,
                    details={'subscriber_nodes': sub_nodes},
                    suggestions=[
                        'Verify publisher nodes are running',
                        'Check topic remapping configuration',
                        'Verify namespace configuration',
                    ]
                ))

    except Exception:
        pass

    return issues


def analyze_topic(topic_name: str) -> Tuple[List[Issue], Dict[str, Any]]:
    """
    Analyze a specific topic for issues.

    Args:
        topic_name: Name of the topic to analyze

    Returns:
        Tuple of (list of issues, topic info dict)
    """
    issues = []
    info = {'topic': topic_name}

    try:
        import rclpy
        from rclpy.node import Node

        rclpy.init()
        node = Node('_wtf_analysis_node')

        # Get topic info
        topic_names_and_types = node.get_topic_names_and_types()
        topic_types = None
        for name, types in topic_names_and_types:
            if name == topic_name:
                topic_types = types
                break

        if not topic_types:
            issues.append(Issue(
                title=f'Topic Not Found: {topic_name}',
                severity=IssueSeverity.CRITICAL,
                problem=f'Topic {topic_name} does not exist',
                impact='Cannot analyze non-existent topic',
                topic=topic_name,
                suggestions=[
                    'Check topic name spelling',
                    'Verify publisher is running',
                    'Check namespace configuration',
                ]
            ))
            node.destroy_node()
            rclpy.shutdown()
            return issues, info

        info['types'] = topic_types

        # Get publisher and subscriber info
        pub_info = node.get_publishers_info_by_topic(topic_name)
        sub_info = node.get_subscriptions_info_by_topic(topic_name)

        info['publishers'] = len(pub_info)
        info['subscribers'] = len(sub_info)

        # Check for QoS issues on this topic
        for issue in get_qos_issues(node):
            if issue.topic == topic_name:
                issues.append(issue)

        # Check for orphaned state
        if pub_info and not sub_info:
            issues.append(Issue(
                title=f'No Subscribers for {topic_name}',
                severity=IssueSeverity.WARNING,
                problem='Publishers exist but no subscribers',
                impact='Published data not being consumed',
                topic=topic_name,
                suggestions=['Start subscriber nodes', 'Check topic remapping']
            ))

        if sub_info and not pub_info:
            issues.append(Issue(
                title=f'No Publishers for {topic_name}',
                severity=IssueSeverity.CRITICAL,
                problem='Subscribers waiting but no publishers',
                impact='Subscribers blocked indefinitely',
                topic=topic_name,
                suggestions=['Start publisher nodes', 'Check topic remapping']
            ))

        node.destroy_node()
        rclpy.shutdown()

    except ImportError:
        issues.append(Issue(
            title='rclpy Not Available',
            severity=IssueSeverity.CRITICAL,
            problem='rclpy module not found',
            impact='Cannot analyze ROS 2 graph',
            suggestions=['Install rclpy package']
        ))
    except Exception as e:
        issues.append(Issue(
            title='Analysis Error',
            severity=IssueSeverity.CRITICAL,
            problem=f'Failed to analyze topic: {str(e)}',
            impact='Cannot complete analysis',
            suggestions=['Check ROS 2 daemon is running', 'Verify ROS_DOMAIN_ID']
        ))

    return issues, info


def analyze_node(node_name: str) -> Tuple[List[Issue], Dict[str, Any]]:
    """
    Analyze a specific node for issues.

    Args:
        node_name: Name of the node to analyze

    Returns:
        Tuple of (list of issues, node info dict)
    """
    issues = []
    info = {'node': node_name}

    try:
        import rclpy
        from rclpy.node import Node

        rclpy.init()
        analysis_node = Node('_wtf_analysis_node')

        # Find the node
        node_names_and_namespaces = analysis_node.get_node_names_and_namespaces()

        found = False
        target_namespace = '/'
        target_name = node_name.lstrip('/')

        for name, ns in node_names_and_namespaces:
            full_name = f'{ns}/{name}' if ns != '/' else f'/{name}'
            if full_name == node_name or name == target_name:
                found = True
                target_name = name
                target_namespace = ns
                break

        if not found:
            issues.append(Issue(
                title=f'Node Not Found: {node_name}',
                severity=IssueSeverity.CRITICAL,
                problem=f'Node {node_name} is not running',
                impact='Cannot analyze non-running node',
                node=node_name,
                suggestions=[
                    'Check node is started',
                    'Verify node name spelling',
                    'Check namespace configuration',
                ]
            ))
            analysis_node.destroy_node()
            rclpy.shutdown()
            return issues, info

        # Get node's publishers and subscribers
        try:
            pubs = analysis_node.get_publisher_names_and_types_by_node(
                target_name, target_namespace
            )
            subs = analysis_node.get_subscriber_names_and_types_by_node(
                target_name, target_namespace
            )

            info['publishers'] = [(name, types) for name, types in pubs]
            info['subscribers'] = [(name, types) for name, types in subs]

            # Check each topic the node interacts with
            all_topics = set([name for name, _ in pubs] + [name for name, _ in subs])

            for topic in all_topics:
                topic_issues, _ = analyze_topic(topic)
                for issue in topic_issues:
                    if issue.node == node_name or (
                        issue.details and
                        node_name in str(issue.details.get('publisher_node', '')) or
                        node_name in str(issue.details.get('subscriber_node', ''))
                    ):
                        issues.append(issue)

        except Exception:
            pass

        analysis_node.destroy_node()
        rclpy.shutdown()

    except ImportError:
        issues.append(Issue(
            title='rclpy Not Available',
            severity=IssueSeverity.CRITICAL,
            problem='rclpy module not found',
            impact='Cannot analyze ROS 2 graph',
            suggestions=['Install rclpy package']
        ))
    except Exception as e:
        issues.append(Issue(
            title='Analysis Error',
            severity=IssueSeverity.CRITICAL,
            problem=f'Failed to analyze node: {str(e)}',
            impact='Cannot complete analysis',
            suggestions=['Check ROS 2 daemon is running']
        ))

    return issues, info


def analyze_system() -> Tuple[List[Issue], Dict[str, Any]]:
    """
    Analyze the entire ROS 2 system for issues.

    Returns:
        Tuple of (list of issues, system info dict)
    """
    issues = []
    info = {}

    try:
        import rclpy
        from rclpy.node import Node

        rclpy.init()
        node = Node('_wtf_analysis_node')

        # Get system overview
        node_names_and_namespaces = node.get_node_names_and_namespaces()
        nodes = [
            (name, ns) for name, ns in node_names_and_namespaces
            if name != '_wtf_analysis_node'
        ]

        topic_names_and_types = node.get_topic_names_and_types()

        info['node_count'] = len(nodes)
        info['topic_count'] = len(topic_names_and_types)
        info['nodes'] = [f'{ns}/{name}' if ns != '/' else f'/{name}'
                        for name, ns in nodes]

        # Run all issue detection
        issues.extend(get_qos_issues(node))
        issues.extend(get_type_mismatches(node))
        issues.extend(get_orphaned_publishers(node))
        issues.extend(get_orphaned_subscribers(node))

        node.destroy_node()
        rclpy.shutdown()

        # Summary
        info['issues_found'] = len(issues)
        info['critical_issues'] = len([i for i in issues if i.severity == IssueSeverity.CRITICAL])
        info['warning_issues'] = len([i for i in issues if i.severity == IssueSeverity.WARNING])

    except ImportError:
        issues.append(Issue(
            title='rclpy Not Available',
            severity=IssueSeverity.CRITICAL,
            problem='rclpy module not found',
            impact='Cannot analyze ROS 2 graph',
            suggestions=['Install rclpy package']
        ))
    except Exception as e:
        issues.append(Issue(
            title='Analysis Error',
            severity=IssueSeverity.CRITICAL,
            problem=f'System analysis failed: {str(e)}',
            impact='Cannot complete system analysis',
            suggestions=['Check ROS 2 daemon is running', 'Verify ROS_DOMAIN_ID']
        ))

    return issues, info
