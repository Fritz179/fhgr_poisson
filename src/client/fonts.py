from PIL import ImageFont

_FONT_CACHE = {}


def load_font(size: int):
    """Load and cache fonts by size to avoid reloading every frame."""
    if size in _FONT_CACHE:
        return _FONT_CACHE[size]
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", size)
    except (OSError, IOError):
        font = ImageFont.load_default()
    _FONT_CACHE[size] = font
    return font
