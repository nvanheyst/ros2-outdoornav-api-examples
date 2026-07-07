"""Shared argparse parser for OutdoorNav examples.

Every script in examples/ should call `make_parser(description, doc=__doc__)`
and then add its own positional/optional arguments. The base parser provides
--namespace, --dry-run, and -v/--verbose so the flags are consistent.
"""

from __future__ import annotations
import argparse
import textwrap
from typing import Optional

from . import config


def make_parser(
    description: str = "",
    doc: Optional[str] = None,
    add_dry_run: bool = True,
) -> argparse.ArgumentParser:
    """Build an ArgumentParser with the shared OutdoorNav flags pre-attached.

    description: short one-liner, shown next to --help.
    doc: long-form help; pass the module's __doc__ so the docstring becomes
        the parser's epilog.
    add_dry_run: include --dry-run; pass False if the script is read-only.
    """
    parser = argparse.ArgumentParser(
        description=description or (doc.strip().splitlines()[0] if doc else ""),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent(doc or "").strip() or None,
    )
    parser.add_argument(
        "--namespace",
        type=config.normalize_namespace,
        default=config.namespace(),
        help="Robot namespace (default $ONAV_NAMESPACE; if empty, load a profile "
             "docker/*.env or let namespace-agnostic scripts auto-detect).",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Verbose ROS logging.",
    )
    if add_dry_run:
        parser.add_argument(
            "--dry-run",
            action="store_true",
            help="Print the ROS calls that would be made without executing them.",
        )
    return parser
