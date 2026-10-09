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
        GOAL_LINE = (255, 60, 60)
        POST = (250, 250, 250)
        NET = (255, 255, 255, 60)

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
        depth = int(22 * scale)
        back_x = gx - depth

        # Net: a shaded box behind the goal line with a crosshatch pattern,
        # inset into the field since the window doesn't extend past x=length.
        net_rect = pygame.Rect(back_x, gt, depth, gb - gt)
        net_surface = pygame.Surface((net_rect.width, net_rect.height), pygame.SRCALPHA)
        net_surface.fill(NET)
        mesh = 10
        for nx in range(0, net_rect.width, mesh):
            pygame.draw.line(net_surface, (255, 255, 255, 90), (nx, 0), (nx, net_rect.height))
        for ny in range(0, net_rect.height, mesh):
            pygame.draw.line(net_surface, (255, 255, 255, 90), (0, ny), (net_rect.width, ny))
        screen.blit(net_surface, net_rect.topleft)

        # Posts (top/bottom/back bar) framing the net.
        pygame.draw.line(screen, POST, (back_x, gt), (gx, gt), 4)
        pygame.draw.line(screen, POST, (back_x, gb), (gx, gb), 4)
        pygame.draw.line(screen, POST, (back_x, gt), (back_x, gb), 4)

        # The actual scoring line, drawn bold and in a color nothing else uses.
        pygame.draw.line(screen, GOAL_LINE, (gx, gt), (gx, gb), 5)

    def ball_crossed_goal_line(self, ball_x, ball_y):
        return (
            ball_x >= self.goal_x
            and self.goal_top <= ball_y <= self.goal_bottom
        )
