#!/usr/bin/env python3
"""List, get, and set parameters on a live remote ROS 2 node.

Equivalent to `ros2 param list/get/set` but in script form, so you can build
runtime tuning into a long-running app (e.g. ramp a controller's `max_vel_x`
down when battery is low, then back up when charging).

  ./parameter_runtime.py --node <ns>/controller_server list
  ./parameter_runtime.py --node <ns>/controller_server get max_vel_x
  ./parameter_runtime.py --node <ns>/controller_server set max_vel_x 0.5
  ./parameter_runtime.py --node <ns>/controller_server describe max_vel_x

Caveats:
  - The remote node has to allow runtime overrides (declared with
    `dynamic_typing=True` or `read_only=False`). Most Nav2 nodes do for
    tuning params; some platform nodes don't.
  - `set` of an undeclared parameter is rejected by default. If you need to
    add a param at runtime the remote node has to allow undeclared params.
  - Type is inferred from the literal you pass: `0.5` → double, `true` →
    bool, `42` → integer, anything else → string. Use `--type` to force.

Touches:
  service <node>/list_parameters       (rcl_interfaces/ListParameters)
  service <node>/get_parameters        (GetParameters)
  service <node>/set_parameters        (SetParameters)
  service <node>/describe_parameters   (DescribeParameters)
"""

from __future__ import annotations
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import (
    GetParameters, SetParameters, ListParameters, DescribeParameters,
)

from common.argparse_base import make_parser
from common.ros_helpers import wait_for_service, call_service


TYPE_NAMES = {
    ParameterType.PARAMETER_NOT_SET: "not_set",
    ParameterType.PARAMETER_BOOL: "bool",
    ParameterType.PARAMETER_INTEGER: "integer",
    ParameterType.PARAMETER_DOUBLE: "double",
    ParameterType.PARAMETER_STRING: "string",
    ParameterType.PARAMETER_BYTE_ARRAY: "byte_array",
    ParameterType.PARAMETER_BOOL_ARRAY: "bool_array",
    ParameterType.PARAMETER_INTEGER_ARRAY: "integer_array",
    ParameterType.PARAMETER_DOUBLE_ARRAY: "double_array",
    ParameterType.PARAMETER_STRING_ARRAY: "string_array",
}


def value_repr(pv: ParameterValue) -> str:
    t = pv.type
    if t == ParameterType.PARAMETER_NOT_SET:
        return "<not set>"
    if t == ParameterType.PARAMETER_BOOL:
        return str(pv.bool_value)
    if t == ParameterType.PARAMETER_INTEGER:
        return str(pv.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:
        return f"{pv.double_value:g}"
    if t == ParameterType.PARAMETER_STRING:
        return repr(pv.string_value)
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return repr(list(pv.bool_array_value))
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return repr(list(pv.integer_array_value))
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return repr(list(pv.double_array_value))
    if t == ParameterType.PARAMETER_STRING_ARRAY:
        return repr(list(pv.string_array_value))
    return f"<type {t}>"


def infer_value(raw: str, forced_type: str | None = None) -> ParameterValue:
    pv = ParameterValue()
    t = forced_type or _guess_type(raw)
    if t == "bool":
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = raw.lower() in ("true", "1", "yes")
    elif t == "integer":
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = int(raw)
    elif t == "double":
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(raw)
    else:
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = raw
    return pv


def _guess_type(raw: str) -> str:
    if raw.lower() in ("true", "false"):
        return "bool"
    try:
        int(raw)
        return "integer"
    except ValueError:
        pass
    try:
        float(raw)
        return "double"
    except ValueError:
        pass
    return "string"


class ParamClient(Node):
    def __init__(self, remote: str):
        super().__init__("parameter_runtime_client")
        self.remote = remote.rstrip("/")
        self.list_cli = self.create_client(ListParameters, f"{self.remote}/list_parameters")
        self.get_cli = self.create_client(GetParameters, f"{self.remote}/get_parameters")
        self.set_cli = self.create_client(SetParameters, f"{self.remote}/set_parameters")
        self.describe_cli = self.create_client(DescribeParameters, f"{self.remote}/describe_parameters")

    def list(self, prefix: str = "") -> list[str]:
        wait_for_service(self, self.list_cli, f"{self.remote}/list_parameters")
        req = ListParameters.Request()
        if prefix:
            req.prefixes = [prefix]
        req.depth = 0
        resp = call_service(self, self.list_cli, req)
        return list(resp.result.names) if resp else []

    def get(self, names: list[str]) -> list[tuple[str, ParameterValue]]:
        wait_for_service(self, self.get_cli, f"{self.remote}/get_parameters")
        req = GetParameters.Request(names=names)
        resp = call_service(self, self.get_cli, req)
        return list(zip(names, resp.values)) if resp else []

    def set(self, name: str, pv: ParameterValue) -> tuple[bool, str]:
        wait_for_service(self, self.set_cli, f"{self.remote}/set_parameters")
        p = Parameter(name=name, value=pv)
        req = SetParameters.Request(parameters=[p])
        resp = call_service(self, self.set_cli, req)
        if not resp or not resp.results:
            return False, "no response"
        r = resp.results[0]
        return r.successful, r.reason

    def describe(self, name: str) -> str:
        wait_for_service(self, self.describe_cli, f"{self.remote}/describe_parameters")
        req = DescribeParameters.Request(names=[name])
        resp = call_service(self, self.describe_cli, req)
        if not resp or not resp.descriptors:
            return "(no descriptor)"
        d = resp.descriptors[0]
        return (
            f"name={d.name}  type={TYPE_NAMES.get(d.type, d.type)}  "
            f"read_only={d.read_only}  dynamic_typing={d.dynamic_typing}\n"
            f"  description: {d.description or '(none)'}\n"
            f"  range/constraints: {d.additional_constraints or '(none)'}"
        )


def main(argv=None):
    parser = make_parser(doc=__doc__, add_dry_run=False)
    parser.add_argument("--node", required=True,
                        help="Fully-qualified name of the remote node (e.g. /a300_00003/controller_server).")
    parser.add_argument("--type", choices=["bool", "integer", "double", "string"], default=None,
                        help="Force the parameter type for `set` (otherwise inferred).")
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("list", help="List parameter names.").add_argument(
        "--prefix", default="", help="Only list params starting with this prefix.")
    g = sub.add_parser("get", help="Get one parameter value.")
    g.add_argument("name")
    s = sub.add_parser("set", help="Set one parameter value.")
    s.add_argument("name")
    s.add_argument("value")
    d = sub.add_parser("describe", help="Describe one parameter (type, range, read_only).")
    d.add_argument("name")
    args = parser.parse_args(argv)

    rclpy.init()
    client = ParamClient(args.node)
    try:
        if args.action == "list":
            names = client.list(prefix=getattr(args, "prefix", ""))
            for n in names:
                print(n)
            print(f"-- {len(names)} parameter(s) --")
        elif args.action == "get":
            for name, pv in client.get([args.name]):
                print(f"{name} = {value_repr(pv)}  ({TYPE_NAMES.get(pv.type, pv.type)})")
        elif args.action == "set":
            pv = infer_value(args.value, args.type)
            ok, reason = client.set(args.name, pv)
            status = "OK" if ok else "FAILED"
            print(f"{status}: {args.name} <- {value_repr(pv)} ({TYPE_NAMES.get(pv.type)})")
            if not ok:
                print(f"  reason: {reason or '(none)'}")
        elif args.action == "describe":
            print(client.describe(args.name))
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
