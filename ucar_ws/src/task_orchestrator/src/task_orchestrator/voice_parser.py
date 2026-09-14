import re


_CATEGORY_PATTERN = re.compile("电子产品|日用品|食品")


def parse_categories(text):
    if not isinstance(text, str) or not text.strip():
        raise ValueError("voice text must be non-empty")
    matches = _CATEGORY_PATTERN.findall(text)
    if len(matches) != 2:
        raise ValueError("voice instruction must contain exactly two categories")
    return matches[0], matches[1]
