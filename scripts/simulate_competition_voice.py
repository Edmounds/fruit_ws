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
    logger.info("【B区扫码】到达B区二维码点位，获取到B区果树规划，异步播报...")
    b_qr_text = (
        "第一棵树苹果；第二棵树梨子；第三棵树苹果；第四棵树梨子；"
        "第五棵树苹果；第六棵树梨子；第七棵树苹果；第八棵树梨子"
    )
    speaker.announce(b_qr_text)
    time.sleep(15.5)

    # 4. C区扫码
    logger.info("【C区扫码】到达C区二维码点位，获取C区分布规划，提前量播报...")
    c_qr_text = (
        "第一项，一号位置，南瓜；第二项，十二号位置，西红柿；"
        "第三项，三号位置，辣椒；第四项，四号位置，洋葱；"
        "第五项，五号位置，南瓜；第六项，六号位置，西红柿；"
        "第七项，七号位置，辣椒；第八项，八号位置，洋葱"
    )
    speaker.announce_and_wait(c_qr_text, lead_time_sec=10.0)
    time.sleep(18.0)

    # 5. C区作业
    logger.info("【C区作业】导航至 1 号位置，采摘成熟南瓜...")
    speaker.announce_and_wait("C区一号位置，应该采南瓜。识别到成熟南瓜，采摘放置工序完成。")
    time.sleep(1.0)

    logger.info("【C区作业】导航至 12 号位置，采摘成熟西红柿...")
    speaker.announce_and_wait("C区十二号位置，应该采西红柿。识别到成熟西红柿，采摘放置工序完成。")
    time.sleep(1.0)

    logger.info("【C区作业】导航至 3 号位置，发现辣椒未成熟，跳过...")
    speaker.announce_and_wait("C区三号位置，应该采辣椒。识别到不成熟辣椒，跳过。")
    time.sleep(1.5)

    # 6. B区作业
    logger.info("【B区作业】返回B区第 1 棵树，采摘成熟苹果...")
    speaker.announce_and_wait("B区第一棵树，识别到苹果，和二维码一致，开始抓成熟苹果。")
    time.sleep(0.5)
    speaker.announce_and_wait("B区第一棵树，识别到苹果，采摘放置工序完成。")
    time.sleep(1.0)

    logger.info("【B区作业】前进至B区第 2 棵树，采摘成熟梨子...")
    speaker.announce_and_wait("B区第二棵树，识别到梨子，和二维码一致，开始抓成熟梨子。")
    time.sleep(0.5)
    speaker.announce_and_wait("B区第二棵树，识别到梨子，采摘放置工序完成。")
    time.sleep(1.0)

    logger.info("【B区作业】前进至B区第 3 棵树，品种与二维码不一致，跳过...")
    speaker.announce_and_wait("B区第三棵树，识别到苹果，和二维码不一致，跳过。")
    time.sleep(1.5)

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
