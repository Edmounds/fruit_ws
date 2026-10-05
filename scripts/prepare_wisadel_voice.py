#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline Wis'adel Voice Preparation, Verification, and Rendering Tool.
Provides batch generation, offline phrase matching check, and scenario rendering.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import subprocess
import sys
import wave
from typing import Dict, List, Optional, Tuple

import yaml


REPRESENTATIVE_SCENARIOS = {
    "intro": [
        "我们是重庆大学 RoboCup 水果采摘队伍，现在开始采摘",
    ],
    "flow_finish": [
        "任务流程结束",
    ],
    "b_qr": [
        "第一棵树苹果",
        "第二棵树梨子",
        "第三棵树苹果",
        "第四棵树梨子",
        "第五棵树苹果",
        "第六棵树梨子",
        "第七棵树苹果",
        "第八棵树梨子",
    ],
    "c_qr": [
        "第一项，一号位置，南瓜",
        "第二项，十二号位置，西红柿",
        "第三项，三号位置，辣椒",
        "第四项，四号位置，洋葱",
        "第五项，五号位置，南瓜",
        "第六项，六号位置，西红柿",
        "第七项，七号位置，辣椒",
        "第八项，八号位置，洋葱",
    ],
    "b_success": [
        "B区第一棵树，识别到苹果，和二维码一致，开始抓成熟苹果。",
        "B区第一棵树，识别到苹果，采摘放置工序完成。",
    ],
    "b_skip": [
        "B区第二棵树，识别到梨子，和二维码一致，没有成熟梨子，跳过。",
        "B区第三棵树，识别到苹果，和二维码不一致，跳过。",
        "B区第四棵树，没有识别到果实，跳过。",
        "B区第五棵树，机械臂执行通信超时，跳过剩余目标。",
    ],
    "c_success": [
        "C区一号位置，应该采南瓜。识别到成熟南瓜，采摘放置工序完成。",
        "C区十二号位置，应该采西红柿。识别到成熟西红柿，采摘放置工序完成。",
    ],
    "c_skip": [
        "C区三号位置，应该采辣椒。识别到不成熟辣椒，跳过。",
        "C区四号位置，应该采洋葱。没有识别到目标，跳过。",
        "C区五号位置，应该采南瓜。导航失败，跳过。",
        "C区六号位置，应该采西红柿。识别到成熟西红柿，闭环抓取失败。",
    ],
    "summary_zero": [
        "任务流程结束",
        "本次没有采摘记录",
    ],
    "summary_multi": [
        "任务流程结束",
        "本次采摘记录",
        "苹果两个",
        "南瓜三个",
        "西红柿五个",
    ],
}


def load_manifest(manifest_path: Path) -> Tuple[dict, Path, Dict[str, Path]]:
    """Load voice manifest and resolve phrase audio paths."""
    with manifest_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    raw_audio_dir = data.get("audio_dir", "../sounds/wisadel")
    if os.path.isabs(raw_audio_dir):
        audio_dir = Path(raw_audio_dir)
    else:
        audio_dir = (manifest_path.parent / raw_audio_dir).resolve()

    phrase_map: Dict[str, Path] = {}
    for item in data.get("phrases", []):
        text = str(item["text"]).strip()
        filename = item["file"]
        wav_path = audio_dir / filename
        phrase_map[text] = wav_path

    return data, audio_dir, phrase_map


def tokenize_text(text: str, phrase_map: Dict[str, Path]) -> Optional[List[Tuple[str, Optional[Path], float]]]:
    """
    Greedy longest-match tokenizer against phrase_map with pause handling.
    Returns list of (token_text, wav_path_or_none, pause_sec_after).
    Returns None if text cannot be fully covered.
    """
    # Sort phrases by length descending for greedy matching
    sorted_phrases = sorted(phrase_map.keys(), key=lambda p: len(p), reverse=True)

    tokens: List[Tuple[str, Optional[Path], float]] = []
    i = 0
    n = len(text)

    while i < n:
        char = text[i]
        # Check standard delimiters
        if char in ("，", ",", " ", "、"):
            if tokens:
                # Add pause to the preceding token
                last = tokens[-1]
                tokens[-1] = (last[0], last[1], max(last[2], 0.15))
            i += 1
            continue
        elif char in ("。", ".", "；", ";", "\n"):
            if tokens:
                last = tokens[-1]
                tokens[-1] = (last[0], last[1], max(last[2], 0.25))
            i += 1
            continue

        matched = False
        sub = text[i:]
        for phrase in sorted_phrases:
            if sub.startswith(phrase):
                tokens.append((phrase, phrase_map[phrase], 0.0))
                i += len(phrase)
                matched = True
                break

        if not matched:
            print(f"[-] Unmatched token at position {i}: '{text[i:i+10]}...' in '{text}'", file=sys.stderr)
            return None

    return tokens


def verify_wav_format(wav_path: Path, expected_rate=32000, expected_channels=1, expected_width=2) -> bool:
    """Verify that WAV file exists, is non-empty, and has expected PCM specs."""
    if not wav_path.is_file():
        return False
    if wav_path.stat().st_size < 44:
        return False
    try:
        with wave.open(str(wav_path), "rb") as wf:
            if wf.getnchannels() != expected_channels:
                return False
            if wf.getsampwidth() != expected_width:
                return False
            if wf.getframerate() != expected_rate:
                return False
            if wf.getnframes() <= 0:
                return False
        return True
    except Exception:
        return False


def concatenate_wavs(token_list: List[Tuple[str, Optional[Path], float]], out_path: Path,
                     sample_rate=32000, channels=1, sampwidth=2) -> float:
    """Concatenate WAVs with silence pauses using standard wave module. Returns total seconds."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_out = out_path.with_suffix(".tmp.wav")

    total_frames = 0
    with wave.open(str(tmp_out), "wb") as out_wf:
        out_wf.setnchannels(channels)
        out_wf.setsampwidth(sampwidth)
        out_wf.setframerate(sample_rate)

        for text, wav_path, pause_sec in token_list:
            if wav_path and wav_path.is_file():
                with wave.open(str(wav_path), "rb") as in_wf:
                    frames = in_wf.readframes(in_wf.getnframes())
                    out_wf.writeframes(frames)
                    total_frames += in_wf.getnframes()

            if pause_sec > 0.0:
                pause_frames = int(sample_rate * pause_sec)
                silence = b"\x00" * (pause_frames * channels * sampwidth)
                out_wf.writeframes(silence)
                total_frames += pause_frames

    tmp_out.replace(out_path)
    return total_frames / sample_rate


def cmd_check(args) -> int:
    manifest_path = Path(args.manifest).resolve()
    data, audio_dir, phrase_map = load_manifest(manifest_path)
    sample_rate = data.get("sample_rate", 32000)

    print(f"[*] Checking manifest: {manifest_path}")
    print(f"    Total phrases: {len(phrase_map)}")
    print(f"    Audio directory: {audio_dir}")

    # Check scenario tokenization
    failed_scenarios = 0
    for scenario_name, lines in REPRESENTATIVE_SCENARIOS.items():
        for line in lines:
            tokens = tokenize_text(line, phrase_map)
            if tokens is None:
                print(f"[-] Scenario '{scenario_name}' failed to tokenize line: {line}")
                failed_scenarios += 1

    if failed_scenarios > 0:
        print(f"[-] Tokenization check FAILED ({failed_scenarios} issues).")
        return 1
    print(f"[+] All representative scenarios tokenized successfully.")

    if args.text_only:
        print("[+] Text-only check PASSED.")
        return 0

    # Check WAV files on disk
    missing = 0
    invalid = 0
    for text, wav_path in phrase_map.items():
        if not wav_path.is_file():
            print(f"[-] Missing WAV file: {wav_path} (phrase: '{text}')")
            missing += 1
        elif not verify_wav_format(wav_path, expected_rate=sample_rate):
            print(f"[-] Invalid WAV format: {wav_path} (phrase: '{text}')")
            invalid += 1

    if missing > 0 or invalid > 0:
        print(f"[-] WAV verification FAILED (missing: {missing}, invalid: {invalid}).")
        return 1

    print(f"[+] All {len(phrase_map)} WAV files verified on disk (32kHz, mono, 16-bit PCM).")
    return 0


def cmd_generate(args) -> int:
    manifest_path = Path(args.manifest).resolve()
    data, audio_dir, phrase_map = load_manifest(manifest_path)
    sample_rate = data.get("sample_rate", 32000)
    audio_dir.mkdir(parents=True, exist_ok=True)

    generator_script = (Path(__file__).parent / "generate_wisadel_voice.py").resolve()
    if not generator_script.is_file():
        print(f"[-] Generator script not found: {generator_script}", file=sys.stderr)
        return 1

    phrases = data.get("phrases", [])
    total = len(phrases)
    print(f"[*] Starting batch generation for {total} phrases...")
    print(f"    Mode: {args.mode}")
    print(f"    Audio dir: {audio_dir}")

    for idx, item in enumerate(phrases, 1):
        phrase_id = item.get("id", f"phrase_{idx}")
        text = str(item["text"]).strip()
        tts_text = str(item.get("tts_text", text)).strip()
        filename = item["file"]
        out_wav = audio_dir / filename

        if not args.force and out_wav.is_file() and verify_wav_format(out_wav, expected_rate=sample_rate):
            print(f"[{idx}/{total}] SKIP existing valid: {filename} ({text})")
            continue

        print(f"[{idx}/{total}] GENERATING: {filename} ('{tts_text}')...")
        cmd = [
            sys.executable,
            str(generator_script),
            "--text", tts_text,
            "--output", str(out_wav),
            "--mode", args.mode,
            "--lang", args.lang,
        ]
        res = subprocess.run(cmd)
        if res.returncode != 0:
            print(f"[-] Generation failed at item {idx} ({filename}). Stopping batch.", file=sys.stderr)
            return res.returncode

        if not verify_wav_format(out_wav, expected_rate=sample_rate):
            print(f"[-] Generated WAV file invalid format: {out_wav}", file=sys.stderr)
            return 1

    print("[+] Batch generation completed successfully.")
    return 0


def cmd_render(args) -> int:
    manifest_path = Path(args.manifest).resolve()
    data, audio_dir, phrase_map = load_manifest(manifest_path)
    sample_rate = data.get("sample_rate", 32000)

    out_dir = Path(args.output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    scenarios_to_render = (
        {args.scenario: REPRESENTATIVE_SCENARIOS[args.scenario]}
        if args.scenario in REPRESENTATIVE_SCENARIOS
        else REPRESENTATIVE_SCENARIOS
    )

    rendered_files = []
    for s_name, lines in scenarios_to_render.items():
        print(f"[*] Rendering scenario: {s_name}...")
        all_tokens: List[Tuple[str, Optional[Path], float]] = []
        for line in lines:
            tokens = tokenize_text(line, phrase_map)
            if tokens is None:
                print(f"[-] Failed to tokenize scenario {s_name} line: {line}", file=sys.stderr)
                return 1
            all_tokens.extend(tokens)
            # Add sentence boundary pause
            if all_tokens:
                last = all_tokens[-1]
                all_tokens[-1] = (last[0], last[1], max(last[2], 0.35))

        out_wav = out_dir / f"scenario_{s_name}.wav"
        duration = concatenate_wavs(all_tokens, out_wav, sample_rate=sample_rate)
        print(f"[+] Rendered: {out_wav} (duration: {duration:.2f}s)")
        rendered_files.append((s_name, out_wav, duration))

    if args.play and rendered_files:
        for s_name, out_wav, dur in rendered_files:
            print(f"[*] Playing scenario {s_name} ({dur:.2f}s): {out_wav}...")
            player = "aplay"
            subprocess.run([player, str(out_wav)])

    return 0


def main():
    parser = argparse.ArgumentParser(description="Wis'adel Helium Voice preparation and check tool")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # Check
    p_check = subparsers.add_parser("check", help="Verify phrases and WAV files on disk")
    p_check.add_argument("--manifest", "-m", default="/home/rc4/fruit_ws/src/fruit_mission/config/wisadel_voice.yaml",
                         help="Path to wisadel_voice.yaml")
    p_check.add_argument("--text-only", action="store_true", help="Only verify phrase tokenization without checking WAVs on disk")
    p_check.set_defaults(func=cmd_check)

    # Generate
    p_gen = subparsers.add_parser("generate", help="Batch generate missing audio files via GPU server")
    p_gen.add_argument("--manifest", "-m", default="/home/rc4/fruit_ws/src/fruit_mission/config/wisadel_voice.yaml",
                       help="Path to wisadel_voice.yaml")
    p_gen.add_argument("--mode", default="gremlin", choices=["gremlin", "classic", "dry"], help="Voice mode")
    p_gen.add_argument("--lang", default="zh", help="Language")
    p_gen.add_argument("--force", "-f", action="store_true", help="Regenerate all even if file exists")
    p_gen.set_defaults(func=cmd_generate)

    # Render
    p_render = subparsers.add_parser("render", help="Render representative scenarios to preview WAVs")
    p_render.add_argument("--manifest", "-m", default="/home/rc4/fruit_ws/src/fruit_mission/config/wisadel_voice.yaml",
                          help="Path to wisadel_voice.yaml")
    p_render.add_argument("--scenario", "-s", default="all", choices=["all"] + list(REPRESENTATIVE_SCENARIOS.keys()),
                          help="Scenario name to render")
    p_render.add_argument("--output-dir", "-o", default="/home/rc4/fruit_ws/tmp/wisadel_voice_preview",
                          help="Output directory for rendered WAVs")
    p_render.add_argument("--play", "-p", action="store_true", help="Play audio after rendering")
    p_render.set_defaults(func=cmd_render)

    args = parser.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
