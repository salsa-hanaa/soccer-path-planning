"""Minimal pygame-native widgets for the interactive demo panel (no PyQt/
external GUI toolkit -- everything draws into the same pygame window as the
field, so there's only one event loop and one process to manage)."""

import pygame


class Button:
    def __init__(self, x, y, w, h, text, callback, enabled=True):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.callback = callback
        self.enabled = enabled

    def draw(self, screen, font):
        fill = (230, 230, 230) if self.enabled else (90, 90, 90)
        border = (20, 20, 20) if self.enabled else (60, 60, 60)
        text_color = (0, 0, 0) if self.enabled else (140, 140, 140)
        pygame.draw.rect(screen, fill, self.rect, border_radius=6)
        pygame.draw.rect(screen, border, self.rect, 2, border_radius=6)
        label = self.text() if callable(self.text) else self.text
        txt = font.render(label, True, text_color)
        screen.blit(txt, txt.get_rect(center=self.rect.center))

    def handle_event(self, event):
        if not self.enabled:
            return
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                self.callback()


class InputField:
    def __init__(self, x, y, w, h, label, value=0.0, min_val=-99999, max_val=99999, enabled=True):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label
        self.text = f"{value:.1f}"
        self.active = False
        self.min_val = min_val
        self.max_val = max_val
        self.enabled = enabled
        # True right after the field is clicked into, before any key is
        # pressed -- the next keystroke replaces the whole value (like a
        # text box that comes pre-selected) instead of appending to
        # whatever stale number was left over from before.
        self._select_all = False

    def get_value(self):
        try:
            return max(self.min_val, min(self.max_val, float(self.text)))
        except ValueError:
            return 0.0

    def set_value(self, v):
        self.text = f"{max(self.min_val, min(self.max_val, v)):.1f}"

    def draw(self, screen, font):
        if not self.enabled:
            color = (70, 70, 70)
        elif self.active:
            color = (255, 255, 255)
        else:
            color = (220, 220, 220)
        pygame.draw.rect(screen, color, self.rect, border_radius=4)
        pygame.draw.rect(screen, (50, 50, 50), self.rect, 2, border_radius=4)

        label_color = (255, 255, 255) if self.enabled else (120, 120, 120)
        label_surf = font.render(self.label, True, label_color)
        screen.blit(label_surf, (self.rect.x, self.rect.y - 18))

        text_color = (0, 0, 0) if self.enabled else (150, 150, 150)
        txt = font.render(self.text, True, text_color)
        screen.blit(txt, (self.rect.x + 5, self.rect.y + 5))

        if self.active and self._select_all:
            highlight = txt.get_rect(topleft=(self.rect.x + 3, self.rect.y + 3))
            highlight.width = txt.get_width() + 4
            highlight.height = txt.get_height() + 4
            pygame.draw.rect(screen, (90, 150, 255), highlight, 2, border_radius=2)

    def handle_event(self, event):
        if not self.enabled:
            self.active = False
            return
        if event.type == pygame.MOUSEBUTTONDOWN:
            was_active = self.active
            self.active = self.rect.collidepoint(event.pos)
            if self.active and not was_active:
                self._select_all = True
        if event.type == pygame.KEYDOWN and self.active:
            if event.key == pygame.K_BACKSPACE:
                self.text = "" if self._select_all else self.text[:-1]
                self._select_all = False
            elif event.key in (pygame.K_RETURN, pygame.K_TAB):
                self.active = False
                self._select_all = False
            elif event.unicode in "0123456789.-":
                self.text = event.unicode if self._select_all else self.text + event.unicode
                self._select_all = False
