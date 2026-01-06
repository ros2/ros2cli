from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension

from ros2tftree.api import run_tf_tree_live




class TfTreeCommand(CommandExtension):
    """Exposes tf-tree subcommands."""

    def add_arguments(self, parser, cli_name, *, argv=None):
        self._subparser = parser
        # Set a description to match upstream tf-tree help
        try:
            parser.description = 'ROS2 TF Tree CLI Helper'
        except Exception:
            pass

        # accept the same flags as the upstream tf-tree for direct invocation
        parser.add_argument(
            '-p', '--profile', choices=['mobile', 'arm', 'auto'], default='auto',
            help='Diagnostic mode')
        parser.add_argument(
            '-s', '--save', metavar='FILE', dest='save',
            help='Export the analysis to a file')
        parser.add_argument(
            '-a', '--alive', action='store_true', dest='alive',
            help='Keep node alive; refreshes terminal every 5 seconds.')
        parser.add_argument(
            '-l', '--light', action='store_true', dest='light',
            help='Light mode: display only the TF tree structure')
        parser.add_argument(
            '-c', '--clear', action='store_true', dest='clear',
            help='Clear terminal screen before each refresh in alive mode')
        parser.add_argument(
            '-nc', '--no-color', action='store_true', dest='no_color',
            help='Disable ANSI colors in output')

        # add arguments and sub-commands (verbs)
        add_subparsers_on_demand(
            parser, cli_name, '_verb', 'ros2tftree.verb', required=False,
            argv=argv)

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            # No subcommand: behave like `tf-tree` default (show)
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
            return run_tf_tree_live(argv)

        extension = getattr(args, '_verb')
        return extension.main(args=args)
