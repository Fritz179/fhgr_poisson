#!/usr/bin/env python3

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HEIGHT, WIDTH = 720, 1280


def opencv_scaled_text():
    """
    Draw text on a 2x canvas with OpenCV, then scale down to 720x1280.
    """
    scale_factor = 2
    big_h, big_w = HEIGHT * scale_factor, WIDTH * scale_factor

    big = np.zeros((big_h, big_w, 3), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_DUPLEX

    # Background
    big[:] = (30, 30, 30)

    # Different font scales
    font_scales = [0.5, 1.0, 1.5, 2.0]
    for i, fs in enumerate(font_scales):
        text = f"OpenCV x2, fontScale={fs}"
        y = (80 + i * 100) * scale_factor
        x = 40 * scale_factor
        cv2.putText(
            big,
            text,
            (x, y),
            font,
            fs,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    # Label at top
    cv2.putText(
        big,
        "OpenCV (drawn on 2x canvas, then downscaled)",
        (40 * scale_factor, 40 * scale_factor),
        font,
        1.0,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    # Downscale to final size
    small = cv2.resize(big, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
    return small


def pillow_text():
    """
    Draw text on a normal 720x1280 canvas using Pillow and convert back to OpenCV.
    """
    img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)

    # Convert to PIL image (RGB)
    pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img)

    # Try a TTF font; fall back to default if not found
    def get_font(size):
        try:
            # Common path on many Linux systems; adjust if needed
            return ImageFont.truetype("DejaVuSans.ttf", size)
        except Exception:
            return ImageFont.load_default()

    sizes = [16, 24, 32, 48]
    for i, s in enumerate(sizes):
        font = get_font(s)
        text = f"Pillow, size={s}"
        x = 40
        y = 80 + i * 100
        draw.text((x, y), text, font=font, fill=(255, 255, 255))

    # Label at top
    title_font = get_font(28)
    draw.text(
        (40, 30),
        "Pillow (TrueType font, direct 1x canvas)",
        font=title_font,
        fill=(0, 255, 255),
    )

    # Back to OpenCV BGR
    result = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    return result


def main():
    left = opencv_scaled_text()
    right = pillow_text()

    # Combine side-by-side
    combined = np.hstack([left, right])

    cv2.namedWindow("Text Comparison")
    cv2.imshow("Text Comparison", combined)
    print("Press any key in the window to exit...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
