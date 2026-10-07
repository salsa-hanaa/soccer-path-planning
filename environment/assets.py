"""Sprite loading for the /asset images. Crops each image down to its
actual (non-transparent) content before scaling, since the source files
have large transparent margins (team.png in particular only uses a small
corner of its 2048x2048 canvas)."""

import os

import pygame

ASSET_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "asset")

_cache = {}


def load_sprite(filename, diameter):
    key = (filename, diameter)
    if key in _cache:
        return _cache[key]

    image = pygame.image.load(os.path.join(ASSET_DIR, filename)).convert_alpha()

    mask = pygame.mask.from_surface(image, 10)
    rects = mask.get_bounding_rects()
    if rects:
        bbox = rects[0].unionall(rects[1:])
        image = image.subsurface(bbox).copy()

    w, h = image.get_size()
    scale = diameter / max(w, h)
    image = pygame.transform.smoothscale(image, (max(1, round(w * scale)), max(1, round(h * scale))))

    _cache[key] = image
    return image
