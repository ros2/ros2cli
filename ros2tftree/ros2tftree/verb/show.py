from ros2tftree.verb import VerbExtension

from ros2tftree.api import run_tf_tree, run_tf_tree_live


class ShowVerb(VerbExtension):
    """Show the TF tree in terminal (delegates to tf_tree_terminal)."""

    def add_arguments(self, parser, cli_name):
        # Intentionally avoid adding the ros2cli node strategy arguments here
        # to prevent importing rclpy (and its optional submodules) at plugin
        # discovery time. `tf_tree_terminal` manages its own rclpy lifecycle
        # when executed.
        import argparse

        parser.add_argument('-p', '--profile', choices=['mobile', 'arm', 'auto'], default='auto')
        parser.add_argument('-s', '--save', metavar='FILE', dest='save')
        parser.add_argument('-a', '--alive', action='store_true', dest='alive')
        parser.add_argument('-l', '--light', action='store_true', dest='light')
        # compatibility: accept a -c/--clear flag to match `tf-tree` and
        # forward it to the underlying tool. Hide from help since it's
        # implemented by the delegated tool.
        parser.add_argument('-c', '--clear', action='store_true', dest='clear', help=argparse.SUPPRESS)
        # compatibility: accept a -nc/--no-color flag to match original tf-tree
        # Argparse will often split combined short options like '-nc' into '-n' and
        # '-c'. Provide '-n' as an alias so '-nc' is interpreted as '-n -c'.
        parser.add_argument('-n', '-nc', '--no-color', action='store_true', dest='no_color', help=argparse.SUPPRESS)

    def main(self, *, args):
        # construct argv for the underlying tool
        argv = []
        if args.profile:
            argv += ['-p', args.profile]
        if args.save:
            argv += ['-s', args.save]
        if args.alive:
            argv += ['-a']
        if args.light:
            argv += ['-l']
        if getattr(args, 'clear', False):
            argv += ['-c']
        if getattr(args, 'no_color', False):
            argv += ['--no-color']

        # run without capturing to preserve native ordering of prints and logs
        return run_tf_tree_live(argv)
