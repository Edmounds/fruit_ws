#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fruit Robot Voice Generator - Wis'adel (维什戴尔) Demonic Helium Voice
Calls GPT-SoVITS fine-tuned model on GPU server (ubuntu-remote) and transfers WAV locally.
"""

import os
import sys
import argparse
import subprocess
import uuid
import base64

def main():
    parser = argparse.ArgumentParser(description="Generate Wis'adel Helium Voice for fruit robot")
    parser.add_argument("--text", "-t", required=True, help="Text to speak")
    parser.add_argument("--output", "-o", default="sounds/wisadel_helium.wav", help="Local output wav path")
    parser.add_argument("--mode", "-m", default="gremlin", choices=["gremlin", "classic", "dry"],
                        help="Voice mode: gremlin (魔性鬼畜版), classic (经典纯正版), dry (微调原声)")
    parser.add_argument("--lang", "-l", default="zh", choices=["zh", "en", "ja", "auto"], help="Text language")
    parser.add_argument("--play", "-p", action="store_true", help="Play audio immediately using aplay/paplay")
    args = parser.parse_args()

    out_path = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    uid = uuid.uuid4().hex[:8]
    remote_wav = f"/tmp/wisadel_{uid}.wav"

    b64_text = base64.b64encode(args.text.encode("utf-8")).decode("utf-8")

    remote_cmd = (
        f"cd ~/GPT-SoVITS && "
        f".venv/bin/python generate_wisadel_audio.py "
        f"--text-b64 {b64_text} "
        f"--lang {args.lang} "
        f"--mode {args.mode} "
        f"--output {remote_wav}"
    )

    print(f"[*] Requesting Wis'adel voice synthesis on GPU server (ubuntu-remote)...")
    print(f"    Mode: {args.mode}")
    print(f"    Text: {args.text}")

    ssh_cmd = ["ssh", "ubuntu-remote", remote_cmd]
    res = subprocess.run(ssh_cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print(f"[-] Remote synthesis failed:\n{res.stderr}", file=sys.stderr)
        sys.exit(res.returncode)

    print(f"[*] Fetching generated audio file to {out_path}...")
    scp_cmd = ["scp", f"ubuntu-remote:{remote_wav}", out_path]
    res_scp = subprocess.run(scp_cmd, capture_output=True, text=True)
    if res_scp.returncode != 0:
        print(f"[-] File transfer failed:\n{res_scp.stderr}", file=sys.stderr)
        sys.exit(res_scp.returncode)

    subprocess.run(["ssh", "ubuntu-remote", f"rm -f {remote_wav}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    file_size_kb = os.path.getsize(out_path) / 1024
    print(f"[+] Audio ready: {out_path} ({file_size_kb:.1f} KB)")

    if args.play:
        print(f"[*] Playing audio...")
        if subprocess.run(["which", "paplay"], capture_output=True).returncode == 0:
            subprocess.run(["paplay", out_path])
        elif subprocess.run(["which", "aplay"], capture_output=True).returncode == 0:
            subprocess.run(["aplay", out_path])
        else:
            print("[-] Neither aplay nor paplay found to play audio.")

if __name__ == "__main__":
    main()
