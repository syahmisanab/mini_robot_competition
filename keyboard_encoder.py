import pygame, time
from motor_driver import MotorDriver   # your library file

# ──------- Motor + Pygame setup -------────────────────────────────────────────
md     = MotorDriver(port="/dev/ttyUSB0", motor_type=1, upload_data=1)
SPEED  = 300                     # tweak for your robot
pygame.init()
screen = pygame.display.set_mode((480, 240))
pygame.display.set_caption("Robot Control  •  WASD / Arrow Keys")
font_big  = pygame.font.Font(None, 46)
font_small = pygame.font.Font(None, 32)

# ──------ Helper functions -------────────────────────────────────────────────
def blit_center(text, y, big=False):
    surf = (font_big if big else font_small).render(text, True, (255,255,255))
    rect = surf.get_rect(center=(240, y))
    screen.blit(surf, rect)

def draw(status, enc_str):
    screen.fill((0,0,0))
    blit_center(status, 60, big=True)           # movement status line
    blit_center("Encoders:", 130)
    blit_center(enc_str, 170)
    pygame.display.flip()

def move(m1, m2, m3, m4, name):
    md.control_speed(m1, m2, m3, m4)
    return name

def stop(): 
    md.control_speed(0,0,0,0)
    return "Stopped"

# ──------ Main loop -------───────────────────────────────────────────────────
status      = "Ready"
enc_display = "M1:—  M2:—  M3:—  M4:—"
draw(status, enc_display)

try:
    running = True
    while running:
        # ---------- handle key states ----------
        keys = pygame.key.get_pressed()
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            status = move(SPEED, SPEED, SPEED, SPEED, "Forward")
        elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
            status = move(-SPEED,-SPEED,-SPEED,-SPEED, "Backward")
        elif keys[pygame.K_a] or keys[pygame.K_LEFT]:
            status = move(-SPEED, SPEED,-SPEED, SPEED, "Left")
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            status = move( SPEED,-SPEED, SPEED,-SPEED, "Right")
        else:
            status = stop()

        # ---------- read encoder feedback ----------
        msg = md.receive_data()
        if msg:
            parsed = md.parse_data(msg)      # e.g. "M1:123, M2:124, ..."
            if parsed:
                enc_display = parsed

        # ---------- update screen ----------
        draw(status, enc_display)

        # ---------- handle quit / ESC ----------
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                running = False

        time.sleep(0.05)   # ~20 fps loop

except KeyboardInterrupt:
    pass
finally:
    stop()
    md.close()
    pygame.quit()
    print("Exited cleanly.")
