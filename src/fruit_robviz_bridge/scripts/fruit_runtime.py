#!/usr/bin/env python3
# Copyright 2026 fruit_ws team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Small command-line client for the persistent runtime HTTP service."""

from __future__ import annotations

import argparse
import json
import os
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def main() -> int:
    parser = argparse.ArgumentParser(description="Control fruit runtime tasks")
    parser.add_argument(
        "command", choices=("status", "start", "stop")
    )
    parser.add_argument("task", nargs="?")
    parser.add_argument(
        "--url", default=os.environ.get("FRUIT_RUNTIME_URL", "http://127.0.0.1:8001")
    )
    args = parser.parse_args()

    if args.command != "status" and not args.task:
        parser.error("start 和 stop 需要 task")
    if args.command == "status":
        path = "/api/runtime/tasks" if args.task is None else f"/api/runtime/tasks/{args.task}"
        method = "GET"
    else:
        path = f"/api/runtime/tasks/{args.task}/{args.command}"
        method = "POST"

    try:
        request = Request(f"{args.url.rstrip('/')}{path}", method=method)
        with urlopen(request, timeout=15) as response:
            payload = json.loads(response.read().decode("utf-8"))
            print(json.dumps(payload, ensure_ascii=False, indent=2))
            return 0
    except (HTTPError, URLError, TimeoutError) as error:
        print(f"runtime 请求失败: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
