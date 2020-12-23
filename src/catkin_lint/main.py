# coding=utf-8
#
# catkin_lint
# Copyright (c) 2013-2020 Fraunhofer FKIE
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of the Fraunhofer organization nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
# IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import argparse
import configparser
import importlib
from . import __version__ as catkin_lint_version
from .linter import CMakeLinter, ERROR, WARNING, NOTICE
from .environment import CatkinEnvironment
from .output import Color, TextOutput, ExplainedTextOutput, JsonOutput, XmlOutput
from .util import getcwd

import catkin_lint.checks


def add_linter_check(linter, check):
    if "." in check:
        pkg, func = check.rsplit(".", 1)
        check_module = importlib.import_module(pkg, "catkin_lint.checks")
    else:
        check_module = catkin_lint.checks
        func = check
    linter.require(getattr(check_module, func))


def prepare_arguments(parser):
    parser.epilog = "Options marked with [*] can be set in the [catkin_lint] section of a configuration file."
    parser.add_argument("--version", action="version", version=catkin_lint_version)
    parser.add_argument("path", metavar="PATH", nargs="*", default=[], help="path to catkin packages")
    parser.add_argument("--help-problem", metavar="ID", nargs="?", default=None, const="**SUMMARY**", help="show information about detectable problems")
    m = parser.add_mutually_exclusive_group()
    m.add_argument("--quiet", "-q", action="store_true", default=None, help="suppress final summary [*]")
    m.add_argument("--no-quiet", action="store_false", default=None, help="override quiet=yes option from configuration file")
    parser.add_argument("--severity-level", "-W", metavar="LEVEL", type=int, default=None, help="set severity level (0-2) [*]")
    parser.add_argument("-c", "--check", metavar="MODULE.CHECK", action="append", default=[], help=argparse.SUPPRESS)
    parser.add_argument("--config", action="append", metavar="FILE", default=[], help="read configuration from FILE (can be used multiple times)")
    parser.add_argument("--ignore", action="append", metavar="ID", default=[], help="ignore diagnostic message ID (can be used multiple times)")
    parser.add_argument("--error", action="append", metavar="ID", default=[], help="treat diagnostic message ID as error (can be used multiple times)")
    parser.add_argument("--warning", action="append", metavar="ID", default=[], help="treat diagnostic message ID as warning (can be used multiple times)")
    parser.add_argument("--notice", action="append", metavar="ID", default=[], help="treat diagnostic message ID as notice (can be used multiple times)")
    m = parser.add_mutually_exclusive_group()
    m.add_argument("--strict", action="store_true", default=None, help="treat everything reported as error [*]")
    m.add_argument("--no-strict", action="store_false", dest="strict", help="override strict=yes option from configuration file")
    parser.add_argument("--show-ignored", action="store_true", help="show messages even if they have been ignored explicitly")
    parser.add_argument("--pkg", action="append", default=[], help="specify catkin package by name (can be used multiple times)")
    parser.add_argument("--skip-pkg", metavar="PKG", action="append", default=[], help="skip testing a catkin package (can be used multiple times)")
    parser.add_argument("--skip-path", metavar="PATH", action="append", default=[], help="skip testing any package in a path that contains PATH (can be used multiple times)")
    parser.add_argument("--package-path", metavar="PATH", help="additional package path (separate multiple locations with '%s') [*]" % os.pathsep)
    parser.add_argument("--rosdistro", metavar="DISTRO", help="override ROS distribution (default: ROS_DISTRO environment variable) [*]")
    m = parser.add_mutually_exclusive_group()
    m.add_argument("--resolve-env", action="store_true", default=None, help="resolve $ENV{} references from environment variables [*]")
    m.add_argument("--no-resolve-env", action="store_false", help="override resolve_env=yes option from configuration file")
    m = parser.add_mutually_exclusive_group()
    m.add_argument("--output", metavar="FORMAT", choices=["text", "explain", "xml", "json"], default="text", help="choose output format for results (default: text) [*]")
    m.add_argument("--text", action="store_const", dest="output", const="text", help="output results as text (same as --output=text)")
    m.add_argument("--explain", action="store_const", dest="output", const="explain", help="output result as text with explanations (same as --output=explain)")
    m.add_argument("--xml", action="store_const", dest="output", const="xml", help="output result as XML (same as --output=xml)")
    m.add_argument("--json", action="store_const", dest="output", const="json", help="output result as JSON (same as --output=json)")
    parser.add_argument("--color", metavar="MODE", choices=["never", "always", "auto"], default=None, help="colorize text output; valid values are \"never\", \"always\", and \"auto\" [*]")
    m = parser.add_mutually_exclusive_group()
    m.add_argument("--offline", action="store_true", default=None, help="do not download package index to look for packages [*]")
    m.add_argument("--no-offline", action="store_false", help="override offline=yes option from configuration file")
    parser.add_argument("--clear-cache", action="store_true", help="clear internal cache and invalidate all downloaded manifests")
    parser.add_argument("--debug", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--disable-cache", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--dump-cache", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--list-check-ids", action="store_true", help=argparse.SUPPRESS)
    return parser


def get_severity_overrides_from_args(args, optionxform=lambda x: x):
    result = {}
    tmp = set(b for a in args.ignore for b in a.split(",") if b)
    for a in tmp:
        result[optionxform(a)] = "ignore"
    tmp = set(b for a in args.notice for b in a.split(",") if b)
    for a in tmp:
        result[optionxform(a)] = "notice"
    tmp = set(b for a in args.warning for b in a.split(",") if b)
    for a in tmp:
        result[optionxform(a)] = "warning"
    tmp = set(b for a in args.error for b in a.split(",") if b)
    for a in tmp:
        result[optionxform(a)] = "error"
    return result


def show_help_with_problems(problem):
    from .diagnostics import message_list
    import re
    import textwrap

    if problem == "**SUMMARY**":
        sys.stdout.write("catkin_lint detects the following problems:\n\n")
        ids = list(message_list.keys())
        ids.sort()
        for k in ids:
            sys.stdout.write("  %-30s -- %s\n" % (k.lower(), message_list[k][0]))
        sys.stdout.write("\nYou can get a more detailed explanation for each problem with\n    catkin_lint --help-problem ID\n")
        return 0
    problem_key = problem.upper().replace("-", "_")
    if problem_key not in message_list:
        sys.stderr.write("catkin_lint: unknown message ID '%s'\n" % problem)
        return 1
    short_desc, long_desc = message_list[problem_key]
    sys.stdout.write("%s -- %s\n\n" % (problem_key.lower(), short_desc))
    explanation = re.sub(r"\s+", " ", long_desc).strip()
    sys.stdout.write(textwrap.fill(explanation))
    sys.stdout.write("\n\n")
    return 0


def run_linter(args):
    if args.clear_cache:
        from .environment import _clear_cache
        _clear_cache()
        return 0
    if args.help_problem is not None:
        return show_help_with_problems(args.help_problem)
    if args.list_check_ids:
        from .diagnostics import message_list
        ids = [k.lower() for k in message_list.keys()]
        ids.sort()
        sys.stdout.write("\n".join(ids))
        sys.stdout.write("\n")
        return 0
    if args.dump_cache:
        from .environment import _dump_cache
        _dump_cache()
        return 0
    config = configparser.ConfigParser(strict=True)
    config.optionxform = lambda option: option.lower().replace("-", "_")
    # Initialize configuration from command line arguments
    config["*"] = {}
    config["catkin_lint"] = {}
    if args.rosdistro:
        config["catkin_lint"]["rosdistro"] = args.rosdistro
    if args.package_path:
        config["catkin_lint"]["package_path"] = args.package_path
    if args.color:
        config["catkin_lint"]["color"] = args.color
    if args.output:
        config["catkin_lint"]["output"] = args.output
    if args.disable_cache:
        config["catkin_lint"]["disable_cache"] = "yes"
    if args.offline is not None:
        config["catkin_lint"]["offline"] = "yes" if args.offline else "no"
    if args.quiet is not None:
        config["catkin_lint"]["quiet"] = "yes" if args.quiet else "no"
    if args.strict is not None:
        config["catkin_lint"]["strict"] = "yes" if args.strict else "no"
    if args.severity_level is not None:
        config["catkin_lint"]["severity_level"] = str(args.severity_level)
    if args.resolve_env is not None:
        config["catkin_lint"]["resolve_env"] = "yes" if args.resolve_env else "no"

    for config_file in args.config:
        try:
            with open(config_file, "r") as f:
                config.read_file(f)
        except IOError as err:
            sys.stderr.write("catkin_lint: cannot read '%s': %s\n" % (config_file, err))
            return 1
    if "ROS_PACKAGE_PATH" in os.environ:
        config.read([os.path.join(d, ".catkin_lint") for d in os.environ["ROS_PACKAGE_PATH"].split(os.pathsep)])
    xdg_config_home = os.environ.get("XDG_CONFIG_HOME", "") or os.path.expanduser("~/.config")
    config.read(
        [
            os.path.join(xdg_config_home, "catkin_lint"),
            os.path.expanduser("~/.catkin_lint")
        ]
    )

    # Override severity settings from command line
    severity_overrides = get_severity_overrides_from_args(args, config.optionxform)
    for section in config.sections():
        if section != "catkin_lint":
            config[section].update(severity_overrides)

    nothing_to_do = 0
    pkgs_to_check = []
    if "rosdistro" in config["catkin_lint"]:
        os.environ["ROS_DISTRO"] = config["catkin_lint"]["rosdistro"]
    quiet = config["catkin_lint"].getboolean("quiet", False)
    env = CatkinEnvironment(
        os_env=os.environ if config["catkin_lint"].getboolean("resolve_env", False) else None,
        use_rosdistro=not config["catkin_lint"].getboolean("offline", False),
        use_cache=not config["catkin_lint"].getboolean("disable_cache", False),
        quiet=quiet
    )
    if not args.path and not args.pkg:
        if os.path.isfile("package.xml"):
            pkgs_to_check += env.add_path(getcwd())
        else:
            sys.stderr.write("catkin_lint: no path given and no package.xml in current directory\n")
            return os.EX_NOINPUT
    if "package_path" in config["catkin_lint"]:
        for path in config["catkin_lint"]["package_path"].split(os.pathsep):
            env.add_path(path)
    if "ROS_PACKAGE_PATH" in os.environ:
        for path in os.environ["ROS_PACKAGE_PATH"].split(os.pathsep):
            env.add_path(path)
    for path in args.path:
        if not os.path.isdir(path):
            sys.stderr.write("catkin_lint: not a directory: %s\n" % path)
            nothing_to_do = 1
            continue
        pkgs_to_check += env.add_path(path)
    for name in args.pkg:
        try:
            path, manifest = env.find_local_pkg(name)
            pkgs_to_check.append((path, manifest))
        except KeyError:
            sys.stderr.write("catkin_lint: no such package: %s\n" % name)
            nothing_to_do = 1
    pkgs_to_check = [(p, m) for p, m in pkgs_to_check if m.name not in args.skip_pkg and all((sp not in p) for sp in args.skip_path)]
    if not pkgs_to_check:
        sys.stderr.write("catkin_lint: no packages to check\n")
        return nothing_to_do
    if "ROS_DISTRO" not in os.environ:
        if env.knows_everything and not quiet:
            sys.stderr.write("catkin_lint: neither ROS_DISTRO environment variable nor --rosdistro option set\n")
            sys.stderr.write("catkin_lint: unknown dependencies will be ignored\n")
        env.knows_everything = False
    use_color = {"never": Color.Never, "always": Color.Always, "auto": Color.Auto}
    color_choice = config["catkin_lint"].get("color", "auto").lower()
    output_format = config["catkin_lint"].get("output", "text").lower()
    if output_format == "xml":
        output = XmlOutput()  # this is never colored
    elif output_format == "json":
        output = JsonOutput()  # also never colored
    elif output_format == "explain":
        output = ExplainedTextOutput(use_color.get(color_choice, Color.Auto))
    elif output_format == "text":
        output = TextOutput(use_color.get(color_choice, Color.Auto))
    else:
        sys.stderr.write("catkin_lint: unknown output format '%s'\n" % output_format)
        return 1
    linter = CMakeLinter(env)
    import_checks = (args.check or ["all"]) + [check for check in config["catkin_lint"].get("extra_checks", "").split() if check]
    for check in import_checks:
        try:
            add_linter_check(linter, check)
        except Exception as err:
            sys.stderr.write("catkin_lint: cannot import '%s': %s\n" % (check, str(err)))
            if args.debug:
                raise
            return 1
    for path, manifest in pkgs_to_check:
        try:
            linter.lint(path, manifest, config=config)
        except Exception as err:  # pragma: no cover
            sys.stderr.write("catkin_lint: cannot lint %s: %s\n" % (manifest.name, str(err)))
            if args.debug:
                raise
    extras = {ERROR: 0, WARNING: 0, NOTICE: 0}
    problems = 0
    exit_code = 0
    diagnostic_label = {ERROR: "error", WARNING: "warning", NOTICE: "notice"}
    output.prolog(fd=sys.stdout)
    severity_level = config["catkin_lint"].getint("severity_level", 1)
    be_strict = config["catkin_lint"].getboolean("strict", False)
    if args.show_ignored:
        linter.messages += linter.ignored_messages
        linter.ignored_messages = []
    for msg in sorted(linter.messages):
        if severity_level < msg.level:
            extras[msg.level] += 1
            continue
        if be_strict:
            msg.level = ERROR
        if msg.level == ERROR:
            exit_code = 1
        output.message(msg, fd=sys.stdout)
        problems += 1
    output.epilog(fd=sys.stdout)
    if not quiet:
        sys.stderr.write("catkin_lint: checked %d packages and found %d problems\n" % (len(pkgs_to_check), problems))
        for level in [ERROR, WARNING, NOTICE]:
            if extras[level] > 0:
                sys.stderr.write("catkin_lint: option -W%d will show %d additional %ss\n" % (level, extras[level], diagnostic_label[level]))
        if linter.ignored_messages:
            sys.stderr.write("catkin_lint: %d messages have been ignored. Use --show-ignored to see them\n" % len(linter.ignored_messages))
    return exit_code


def main():  # pragma: no cover
    try:
        parser = prepare_arguments(argparse.ArgumentParser())
        args = parser.parse_args()
        sys.exit(run_linter(args))
    except Exception as err:
        sys.stderr.write("catkin_lint: internal error: %s\n\n" % str(err))
        if args and args.debug:
            raise
        sys.exit(2)


description = dict(
    verb="lint",
    description="Checks catkin packages for common errors",
    main=run_linter,
    prepare_arguments=prepare_arguments
)
