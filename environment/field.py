import pygame


class Field:
    def __init__(self, width=600.0, length=900.0, goal_mouth=260.0):
        self.width = width
        self.length = length
        self.goal_mouth = goal_mouth

        half_mouth = goal_mouth / 2
        center_y = width / 2
        # Our robot always shoots at the goal on the right edge (x = length).
        self.goal_x = length
        self.goal_top = center_y - half_mouth
        self.goal_bottom = center_y + half_mouth
        self.goal_center = (length, center_y)

    def draw(self, screen, scale=1.0, offset_x=0, offset_y=0):
        GREEN = (40, 150, 40)
        WHITE = (245, 245, 245)
        YELLOW = (240, 200, 40)

        w = int(self.length * scale)
        h = int(self.width * scale)

        pygame.draw.rect(screen, GREEN, pygame.Rect(offset_x, offset_y, w, h))
        pygame.draw.rect(screen, WHITE, pygame.Rect(offset_x, offset_y, w, h), 4)
        pygame.draw.line(
            screen, WHITE,
            (offset_x + w // 2, offset_y),
            (offset_x + w // 2, offset_y + h),
            2,
        )

        gx = offset_x + int(self.goal_x * scale)
        gt = offset_y + int(self.goal_top * scale)
        gb = offset_y + int(self.goal_bottom * scale)
        pygame.draw.line(screen, YELLOW, (gx, gt), (gx, gb), 6)

    def ball_crossed_goal_line(self, ball_x, ball_y):
        return (
            ball_x >= self.goal_x
            and self.goal_top <= ball_y <= self.goal_bottom
        )
