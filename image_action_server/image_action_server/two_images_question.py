# two_images_question.py
# ⚠️ Unsafe: API key hardcoded — do NOT use in production.

import base64
from openai import OpenAI

# --- UNSAFE: hardcoded API key ---
client = OpenAI(api_key="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")

# Paths to your local images
img1_path = "/home/tamer/ws/src/image_action_server/image_action_server/shoe1_image1.jpg"
img2_path = "/home/tamer/ws/src/image_action_server/image_action_server/shoe1_image2.jpg"

# Your question about the two images
question = "Compare these two images: do they present the same model of shoe? Answer True or False"

def encode_image(path):
    """Read image file and return base64 string."""
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")

# Encode the local images
img1_base64 = encode_image(img1_path)
img2_base64 = encode_image(img2_path)

# Build multimodal user message
user_content = [
    {"type": "text", "text": question},
    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img1_base64}"}},
    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img2_base64}"}},
]

try:
    resp = client.chat.completions.create(
        model="gpt-4o-mini",  # vision-capable
        messages=[
            {"role": "system", "content": "You are a helpful assistant."},
            {"role": "user", "content": user_content},
        ],
    )

    # Print the assistant's answer
    print(resp.choices[0].message.content)

except Exception as e:
    print("Error calling OpenAI API:", e)
