#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulate full competition A/B/C mission voice announcements on live speaker.
"""
import logging
import sys
import time

from fruit_mission.tts import RecordedSpeaker

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("MissionSimulation")


def main():
    config = {
        "manifest_path": "/home/rc4/fruit_ws/src/fruit_mission/config/wisadel_voice.yaml",
        "aplay_executable": "/usr/bin/aplay",
        "wav_path": "/home/rc4/fruit_ws/tmp/fruit_mission/stitched_speech.wav",
        "playback_timeout_sec": 10.0,
    }
    speaker = RecordedSpeaker(config, logger)

    print("\n" + "=" * 60)
    print(">>> 模拟比赛全流程语音播报开始 <<<")
    print("=" * 60 + "\n")

    # 1. 起点
    logger.info("【起点】准备出发，同步播报队伍信息...")
    speaker.announce_and_wait("我们是重庆大学 RoboCup 水果采摘队伍，现在开始采摘")
    time.sleep(1.5)

    # 2. A区
    logger.info("【A区】正在执行固定路线巡检与采摘作业（静默运行，无播报）...")
    time.sleep(2.0)

    # 3. B区扫码
    logger.info("【B区扫码】到达B区二维码点位，获取到B区果树规划，精简异步播报...")
    b_qr_text = (
        "一号苹果；二号梨子；三号苹果；四号梨子；"
        "五号苹果；六号梨子；七号苹果；八号梨子"
    )
    speaker.announce(b_qr_text)
    time.sleep(6.0)

    # 4. C区扫码
    logger.info("【C区扫码】到达C区二维码点位，获取C区分布规划，零等待异步播报...")
    c_qr_text = (
        "一号南瓜；十二号西红柿；三号辣椒；四号洋葱；"
        "五号南瓜；六号西红柿；七号辣椒；八号洋葱"
    )
    speaker.announce(c_qr_text)
    time.sleep(6.0)

    # 5. C区作业
    logger.info("【C区作业】导航至 1 号位置，采摘成熟南瓜（边动边播报）...")
    speaker.announce_and_wait("一号识别到成熟南瓜，开始抓取。")
    time.sleep(0.5)
    speaker.announce_and_wait("采摘完成。")
    time.sleep(1.0)

    logger.info("【C区作业】导航至 12 号位置，采摘成熟西红柿（边动边播报）...")
    speaker.announce_and_wait("十二号识别到成熟西红柿，开始抓取。")
    time.sleep(0.5)
    speaker.announce_and_wait("采摘完成。")
    time.sleep(1.0)

    logger.info("【C区作业】导航至 3 号位置，发现辣椒未成熟，跳过...")
    speaker.announce_and_wait("三号识别到不成熟辣椒，跳过。")
    time.sleep(1.0)

    logger.info("【C区作业】导航至 4 号位置，视野无目标，跳过...")
    speaker.announce_and_wait("四号没有识别到目标，跳过。")
    time.sleep(1.0)

    # 6. B区作业
    logger.info("【B区作业】返回B区第 1 棵树，采摘成熟苹果（边动边播报）...")
    speaker.announce_and_wait("一号识别到成熟苹果，开始抓取。")
    time.sleep(0.5)
    speaker.announce_and_wait("采摘完成。")
    time.sleep(1.0)

    logger.info("【B区作业】前进至B区第 2 棵树，采摘成熟梨子（边动边播报）...")
    speaker.announce_and_wait("二号识别到成熟梨子，开始抓取。")
    time.sleep(0.5)
    speaker.announce_and_wait("采摘完成。")
    time.sleep(1.0)

    logger.info("【B区作业】前进至B区第 3 棵树，品种与二维码不一致，跳过...")
    speaker.announce_and_wait("三号识别到成熟苹果，跳过。")
    time.sleep(1.0)

    # 7. 流程结束与采摘汇总
    logger.info("【比赛结束】全流程正常遍历完毕，直接播报流程结束与按品种采摘汇总...")
    speaker.announce_and_wait("任务流程结束")
    speaker.announce_and_wait("本次采摘记录")
    speaker.announce_and_wait("苹果一个")
    speaker.announce_and_wait("梨子一个")
    speaker.announce_and_wait("南瓜一个")
    speaker.announce_and_wait("西红柿一个")

    time.sleep(1.0)
    speaker.close(wait=True)

    print("\n" + "=" * 60)
    print(">>> 比赛全流程语音播报模拟完成 <<<")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
