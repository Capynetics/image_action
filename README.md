# image_action

Hey ChatGPT, is this the shoe?  
A ROS 2 action server and client that compares two images using the OpenAI API.

## Overview

This repo contains:
- **`image_action_msgs`** — Custom action definition `CompareImages.action`.
- **`image_action_server`** — ROS 2 Python package with:
  - `image_action_server.py` — Action server that sends the two images and a description to OpenAI for evaluation.
  - `image_action_client.py` — Action client that sends two local images and a description to the server.
  - Example images (`shoe1_image1.jpg`, `shoe1_image2.jpg`, `shoe2_image1.jpg`, `shoe2_image2.jpg`).

---

## Requirements

- ROS 2 Humble (or later)
- Python 3.10+
- OpenAI Python client (`pip install openai`)
- OpenCV (`pip install opencv-python`)
- `cv_bridge` installed in your ROS 2 setup

---

## Installation

Clone the repository into your ROS 2 workspace `src/` folder:

```bash
cd ~/ws/src
git clone https://github.com/Capynetics/image_action.git
```

Install Python dependencies (for OpenAI + image handling):

```bash
pip install openai opencv-python
```

Build the workspace:

```bash
cd ~/ws
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash   # or setup.zsh if using zsh
```

---

## Running the Action Server

**Edit `image_action_server.py`** and place your OpenAI API key directly in the script:

```python
OPENAI_API_KEY = "sk-..."  # Your API key
```

Start the server:

```bash
ros2 run image_action_server image_action_server
```

You should see:

```
[INFO] [compare_images_action_server]: Action server ready on "compare_images"
```

---

## Sending Goals with the Client

The client takes three arguments:
1. **Path to first image**
2. **Path to second image**
3. **Description** (used as the OpenAI prompt)

Example: Compare two identical shoe images

```bash
ros2 run image_action_server image_action_client   $(ros2 pkg prefix image_action_server)/share/image_action_server/shoe1_image1.jpg   $(ros2 pkg prefix image_action_server)/share/image_action_server/shoe1_image1.jpg   "Compare these images. Do they show the same model of shoe? Answer True or False."
```

Example: Compare different shoes

```bash
ros2 run image_action_server image_action_client   $(ros2 pkg prefix image_action_server)/share/image_action_server/shoe1_image1.jpg   $(ros2 pkg prefix image_action_server)/share/image_action_server/shoe2_image1.jpg   "Compare these images. Do they show the same model of shoe? Answer True or False."
```

---

## Expected Output

- **Server console** will log the prompt and the result from OpenAI.
- **Client console** will print the `CompareImages_Result` message, e.g.:

```
[INFO] [compare_images_client]: Result: image_action_msgs.action.CompareImages_Result(result=True)
```

---

## Notes

- The OpenAI API key is hardcoded for development only — **do not commit it to a public repository**.
- The `description` field controls how OpenAI interprets the task — ensure it explicitly asks for `True` or `False`.
- The example images are bundled for testing purposes.
