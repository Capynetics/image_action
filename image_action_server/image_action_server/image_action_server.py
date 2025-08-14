#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ⚠️ SECURITY WARNING:
# You asked to hardcode the API key. Anyone with read access to this file (or logs)
# will have your key. Rotate it if you ever commit/share this code.

import base64
import os
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from sensor_msgs.msg import Image
from image_action_msgs.action import CompareImages

from openai import OpenAI

OPENAI_API_KEY = "key_here"  # <-- put your key here
OPENAI_MODEL = os.getenv("OPENAI_VISION_MODEL", "gpt-4o")  # vision-capable


def imgmsg_to_data_url(bridge: CvBridge, msg: Image) -> str:
    """Convert sensor_msgs/Image to a base64 data URL (JPEG)."""
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    ok, buf = cv2.imencode(".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok:
        raise RuntimeError("Failed to JPEG-encode image")
    b64 = base64.b64encode(buf.tobytes()).decode("utf-8")
    return f"data:image/jpeg;base64,{b64}"


class CompareImagesActionServer(Node):
    def __init__(self) -> None:
        super().__init__("compare_images_action_server")
        self.bridge = CvBridge()
        self._oa_client = OpenAI(api_key=OPENAI_API_KEY)

        self._server = ActionServer(
            self,
            CompareImages,
            "compare_images",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info('Action server ready on "compare_images"')

    def goal_callback(self, goal_request: CompareImages.Goal) -> GoalResponse:
        if not isinstance(goal_request.image1, Image) or not isinstance(goal_request.image2, Image):
            self.get_logger().warn("Rejected goal: invalid image types.")
            return GoalResponse.REJECT
        if not (goal_request.description and goal_request.description.strip()):
            self.get_logger().warn("Rejected goal: empty description.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _ask_openai(self, prompt_text: str, img1_url: str, img2_url: str) -> str:
        """Call OpenAI with description + two images; return assistant message text."""
        user_content = [
            {"type": "text", "text": prompt_text},
            {"type": "image_url", "image_url": {"url": img1_url}},
            {"type": "image_url", "image_url": {"url": img2_url}},
        ]
        system_msg = {
            "role": "system",
            "content": "You are a helpful assistant"
        }
        user_msg = {"role": "user", "content": user_content}

        resp = self._oa_client.chat.completions.create(
            model=OPENAI_MODEL,
            messages=[system_msg, user_msg],
            temperature=0,
            max_tokens=8,
        )

        # --- DEBUG PRINTS (full answer) ---
        try:
            # Newer SDKs: model_dump_json exists (pydantic). If not, fallback to str().
            raw_json = getattr(resp, "model_dump_json", None)
            print("\n=== OpenAI RAW RESPONSE ===")
            print(raw_json(indent=2) if callable(raw_json) else str(resp))
            print("=== END RAW RESPONSE ===\n")
        except Exception as e:
            print(f"[DEBUG] Failed to pretty-print raw response: {e}")

        answer = resp.choices[0].message.content or ""
        print(f"[DEBUG] OpenAI content: {answer!r}")  # full assistant text
        return answer.strip()

    def execute_callback(self, goal_handle):
        goal: CompareImages.Goal = goal_handle.request
        self.get_logger().info(f'Received goal: description="{goal.description}"')

        # Convert ROS images -> data URLs
        try:
            img1_url = imgmsg_to_data_url(self.bridge, goal.image1)
            img2_url = imgmsg_to_data_url(self.bridge, goal.image2)
        except Exception as e:
            self.get_logger().error(f"Failed to convert images: {e}")
            goal_handle.abort()
            return CompareImages.Result(result=False)

        # Call OpenAI
        try:
            answer = self._ask_openai(goal.description, img1_url, img2_url)
        except Exception as e:
            self.get_logger().error(f"OpenAI API error: {e}")
            goal_handle.abort()
            return CompareImages.Result(result=False)

        # Interpret answer: only the literal 'true' (case-insensitive) counts as True
        result_bool = answer.strip().lower() == "true"

        res = CompareImages.Result()
        res.result = result_bool
        goal_handle.succeed()
        return res

    def destroy(self) -> bool:
        self._server.destroy()
        return super().destroy()


def main() -> None:
    rclpy.init()
    node = CompareImagesActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
